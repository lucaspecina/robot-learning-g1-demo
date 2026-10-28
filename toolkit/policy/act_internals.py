"""ACT taken apart: each stage of LeRobot's ACTPolicy as a separate function.

The functions reproduce `ACT.forward` in `lerobot/policies/act/modeling_act.py`
(LeRobot 0.4.1) step by step, so a notebook can stop after any stage and look
at the tensors. The tests check that chaining them gives exactly the same
output as the model.

Shapes use LeRobot's convention inside the transformer: (sequence, batch, dim).
"""

from __future__ import annotations

import contextlib
import dataclasses

import einops
import torch
from torch import Tensor

from lerobot.utils.constants import ACTION, OBS_STATE


@dataclasses.dataclass
class StyleEncoding:
    """Output of the CVAE encoder for a batch of (state, action chunk) pairs."""

    mu: Tensor
    """(B, latent_dim) center of each latent Gaussian."""
    log_sigma_x2: Tensor
    """(B, latent_dim) log of the variance (2 * log sigma), as LeRobot names it."""

    @property
    def sigma(self) -> Tensor:
        return (self.log_sigma_x2 / 2).exp()

    def kl_per_sample(self) -> Tensor:
        """(B,) KL divergence to the standard normal, summed over latent dimensions."""
        return (-0.5 * (1 + self.log_sigma_x2 - self.mu.pow(2) - self.log_sigma_x2.exp())).sum(-1)


@dataclasses.dataclass
class SceneTokens:
    """The token sequence that enters the policy's transformer encoder."""

    tokens: Tensor
    """(S, B, D) latent token, state token, then one token per image patch."""
    positions: Tensor
    """(S, 1 or B, D) position embedding added to queries and keys, never to values."""
    image_grid: tuple[int, int]
    """(rows, cols) of the backbone feature map, to put patch tokens back on the image."""

    @property
    def n_prefix(self) -> int:
        """Tokens before the image patches (latent and, if used, state)."""
        return self.tokens.shape[0] - self.image_grid[0] * self.image_grid[1]


def camera_images(model, batch: dict) -> list[Tensor]:
    return [batch[key] for key in model.config.image_features]


@torch.no_grad()
def encode_style(model, batch: dict) -> StyleEncoding:
    """CVAE encoder: [CLS, state, action_1..action_K] -> Gaussian parameters.

    `batch` must be normalized (preprocessed) and contain the action chunk and
    `action_is_pad`, as during training.
    """
    batch_size = batch[ACTION].shape[0]
    cls = einops.repeat(model.vae_encoder_cls_embed.weight, "1 d -> b 1 d", b=batch_size)
    parts = [cls]
    pad_prefix = 1
    if model.config.robot_state_feature:
        parts.append(model.vae_encoder_robot_state_input_proj(batch[OBS_STATE]).unsqueeze(1))
        pad_prefix = 2
    parts.append(model.vae_encoder_action_input_proj(batch[ACTION]))
    tokens = torch.cat(parts, dim=1)  # (B, K + 2, D)

    key_padding_mask = torch.cat(
        [torch.zeros(batch_size, pad_prefix, dtype=torch.bool, device=tokens.device), batch["action_is_pad"]],
        dim=1,
    )
    out = model.vae_encoder(
        tokens.permute(1, 0, 2),
        pos_embed=model.vae_encoder_pos_enc.clone().detach().permute(1, 0, 2),
        key_padding_mask=key_padding_mask,
    )[0]  # the CLS output, (B, D)
    params = model.vae_encoder_latent_output_proj(out)
    latent_dim = model.config.latent_dim
    return StyleEncoding(mu=params[:, :latent_dim], log_sigma_x2=params[:, latent_dim:])


def sample_latent(style: StyleEncoding, noise: Tensor | None = None) -> Tensor:
    """Reparameterization trick: z = mu + sigma * eps, so gradients reach mu and sigma."""
    eps = torch.randn_like(style.mu) if noise is None else noise
    return style.mu + style.sigma * eps


def zero_latent(model, batch_size: int, device) -> Tensor:
    """What the policy uses at inference time: the encoder is discarded and z = 0."""
    return torch.zeros(batch_size, model.config.latent_dim, device=device)


@torch.no_grad()
def scene_tokens(model, batch: dict, latent: Tensor) -> SceneTokens:
    """Project latent, state and image patches to tokens of size D, with their positions."""
    tokens = [model.encoder_latent_input_proj(latent)]
    positions = list(model.encoder_1d_feature_pos_embed.weight.unsqueeze(1))
    if model.config.robot_state_feature:
        tokens.append(model.encoder_robot_state_input_proj(batch[OBS_STATE]))
    if model.config.env_state_feature:
        raise NotImplementedError("environment-state inputs are not used by our policies")

    grid = (0, 0)
    for image in camera_images(model, batch):
        features = model.backbone(image)["feature_map"]
        grid = tuple(features.shape[-2:])
        image_positions = model.encoder_cam_feat_pos_embed(features).to(dtype=features.dtype)
        features = model.encoder_img_feat_input_proj(features)
        tokens.extend(list(einops.rearrange(features, "b c h w -> (h w) b c")))
        positions.extend(list(einops.rearrange(image_positions, "b c h w -> (h w) b c")))
    return SceneTokens(tokens=torch.stack(tokens), positions=torch.stack(positions), image_grid=grid)


@torch.no_grad()
def encode_scene(model, scene: SceneTokens) -> Tensor:
    """Transformer encoder: every token attends to every other. Returns the memory (S, B, D)."""
    return model.encoder(scene.tokens, pos_embed=scene.positions)


@torch.no_grad()
def decode_chunk(model, memory: Tensor, scene: SceneTokens) -> tuple[Tensor, Tensor]:
    """Transformer decoder with K learned queries, then the action head.

    Returns (decoder output (K, B, D), normalized actions (B, K, action_dim)).
    """
    chunk_size = model.config.chunk_size
    queries = torch.zeros(chunk_size, memory.shape[1], model.config.dim_model, dtype=memory.dtype, device=memory.device)
    decoded = model.decoder(
        queries,
        memory,
        encoder_pos_embed=scene.positions,
        decoder_pos_embed=model.decoder_pos_embed.weight.unsqueeze(1),
    )
    return decoded, model.action_head(decoded.transpose(0, 1))


@contextlib.contextmanager
def capture_attention(model):
    """Record the head-averaged attention weights of every attention module during a forward pass.

    Keys look like "encoder.layers.0.self_attn" or "decoder.layers.0.multihead_attn".
    Values are (B, queries, keys) tensors.
    """
    weights: dict[str, Tensor] = {}
    handles = []
    for name, module in model.named_modules():
        if isinstance(module, torch.nn.MultiheadAttention):
            def hook(_module, _inputs, output, name=name):
                if output[1] is not None:
                    weights[name] = output[1].detach()
            handles.append(module.register_forward_hook(hook))
    try:
        yield weights
    finally:
        for handle in handles:
            handle.remove()
