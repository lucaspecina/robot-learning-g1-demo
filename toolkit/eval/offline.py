"""Offline evaluation of an action-chunking policy on recorded episodes.

At chosen frames of an episode the policy receives the recorded observation
(camera and joint state) and predicts its next chunk of actions. Those are
compared with what the operator actually did. The policy always sees the true
observation, never the consequence of its own previous actions ("teacher
forcing"), so this measures whether it learned the mapping, not whether the
task would succeed when its errors compound.

Two naive baselines give the error a scale. A policy that cannot beat them has
not learned anything useful:
- hold: every future action equals the current joint state (stand still);
- last action: every future action repeats the action just before t.
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pandas as pd
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from toolkit.policy.load import LoadedPolicy

OBSERVATION_PREFIX = "observation."


@dataclasses.dataclass
class ChunkPrediction:
    episode_index: int
    start: int
    """Frame index within the episode where the chunk starts."""
    predicted: np.ndarray
    """(horizon, action_dim) actions predicted by the policy."""
    target: np.ndarray
    """(horizon, action_dim) actions recorded from the operator."""
    hold: np.ndarray
    last_action: np.ndarray


def episode_frames(dataset: LeRobotDataset, episode_index: int) -> pd.DataFrame:
    """The episode's rows (state, action, indices) without decoding video."""
    table = dataset.hf_dataset.with_format("numpy")
    meta = dataset.meta.episodes[episode_index]
    rows = table[int(meta["dataset_from_index"]) : int(meta["dataset_to_index"])]
    return pd.DataFrame({k: list(v) for k, v in rows.items() if k in ("index", "frame_index", "observation.state", "action")})


@torch.no_grad()
def predict_chunk(loaded: LoadedPolicy, frame: dict) -> np.ndarray:
    observation = {k: v for k, v in frame.items() if k.startswith(OBSERVATION_PREFIX)}
    batch = loaded.preprocessor(observation)
    normalized = loaded.policy.predict_action_chunk(batch)
    return loaded.postprocessor(normalized)[0].cpu().numpy()


def evaluate_episode(
    loaded: LoadedPolicy, dataset: LeRobotDataset, episode_index: int, stride: int | None = None
) -> list[ChunkPrediction]:
    horizon = loaded.policy.config.chunk_size
    stride = stride or horizon
    rows = episode_frames(dataset, episode_index)
    actions = np.stack(rows["action"].to_numpy())
    states = np.stack(rows["observation.state"].to_numpy())

    results = []
    # Only starts with a full future window, so every chunk is scored on the same horizon.
    for start in range(1, len(rows) - horizon + 1, stride):
        frame = dataset[int(rows["index"].iloc[start])]
        results.append(
            ChunkPrediction(
                episode_index=episode_index,
                start=start,
                predicted=predict_chunk(loaded, frame),
                target=actions[start : start + horizon],
                hold=np.repeat(states[start][None], horizon, axis=0),
                last_action=np.repeat(actions[start - 1][None], horizon, axis=0),
            )
        )
    return results


def error_table(predictions: list[ChunkPrediction], joint_names: list[str]) -> pd.DataFrame:
    """Mean absolute error per joint (radians) for the policy and both baselines."""
    columns = {}
    for name in ("predicted", "hold", "last_action"):
        errors = np.concatenate([np.abs(getattr(p, name) - p.target) for p in predictions])
        columns["policy" if name == "predicted" else name] = errors.mean(axis=0)
    return pd.DataFrame(columns, index=joint_names)


def error_by_horizon(predictions: list[ChunkPrediction]) -> pd.DataFrame:
    """Mean absolute error over joints at each step into the future."""
    columns = {}
    for name in ("predicted", "hold", "last_action"):
        errors = np.stack([np.abs(getattr(p, name) - p.target).mean(axis=1) for p in predictions])
        columns["policy" if name == "predicted" else name] = errors.mean(axis=0)
    return pd.DataFrame(columns)
