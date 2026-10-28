"""The ACT pieces chained together must reproduce the model exactly.

A small randomly initialized ACT is enough: the check is about wiring, not
about trained weights.
"""

import pytest
import torch
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy

from toolkit.policy import act_internals as act

CAMERA = "observation.images.cam"


@pytest.fixture(scope="module")
def policy():
    torch.manual_seed(0)
    config = ACTConfig(
        input_features={
            "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(6,)),
            CAMERA: PolicyFeature(type=FeatureType.VISUAL, shape=(3, 64, 96)),
        },
        output_features={"action": PolicyFeature(type=FeatureType.ACTION, shape=(6,))},
        chunk_size=8, n_action_steps=8, dim_model=32, n_heads=2, dim_feedforward=64,
        n_encoder_layers=2, n_vae_encoder_layers=2, latent_dim=4,
        pretrained_backbone_weights=None,
    )
    return ACTPolicy(config)


@pytest.fixture
def batch():
    torch.manual_seed(1)
    return {
        "observation.state": torch.randn(2, 6),
        CAMERA: torch.randn(2, 3, 64, 96),
        "action": torch.randn(2, 8, 6),
        "action_is_pad": torch.tensor([[False] * 8, [False] * 5 + [True] * 3]),
    }


def test_inference_pieces_match_predict_action_chunk(policy, batch):
    model = policy.model
    policy.eval()
    scene = act.scene_tokens(model, batch, act.zero_latent(model, 2, "cpu"))
    _, actions = act.decode_chunk(model, act.encode_scene(model, scene), scene)
    with torch.no_grad():
        expected = policy.predict_action_chunk(batch)
    assert torch.allclose(actions, expected, atol=1e-5)
    assert scene.image_grid == (2, 3)
    assert scene.n_prefix == 2


def test_training_pieces_match_forward_with_the_same_noise(policy, batch):
    model = policy.model
    policy.eval()  # no dropout, so the only randomness is the latent sample
    model.training = True  # but take the VAE branch, as during training
    for module in model.modules():
        if isinstance(module, torch.nn.Dropout):
            module.eval()
    try:
        batch_with_images = dict(batch, **{"observation.images": [batch[CAMERA]]})
        torch.manual_seed(7)
        with torch.no_grad():
            expected, (mu, log_sigma_x2) = model(batch_with_images)

        style = act.encode_style(model, batch)
        torch.manual_seed(7)
        latent = act.sample_latent(style)
        scene = act.scene_tokens(model, batch, latent)
        _, actions = act.decode_chunk(model, act.encode_scene(model, scene), scene)
    finally:
        model.training = False

    assert torch.allclose(style.mu, mu, atol=1e-5)
    assert torch.allclose(style.log_sigma_x2, log_sigma_x2, atol=1e-5)
    assert torch.allclose(actions, expected, atol=1e-5)


def test_kl_is_zero_for_the_standard_normal():
    style = act.StyleEncoding(mu=torch.zeros(3, 4), log_sigma_x2=torch.zeros(3, 4))
    assert torch.allclose(style.kl_per_sample(), torch.zeros(3))


def test_attention_is_captured_for_encoder_and_cross_attention(policy, batch):
    policy.eval()
    with act.capture_attention(policy.model) as weights, torch.no_grad():
        policy.predict_action_chunk(batch)
    cross = weights["decoder.layers.0.multihead_attn"]
    assert cross.shape == (2, 8, 8)  # 8 queries over 2 prefix + 6 image tokens
    assert torch.allclose(cross.sum(-1), torch.ones(2, 8), atol=1e-5)
