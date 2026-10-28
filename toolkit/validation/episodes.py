"""Load the numeric part of a LeRobot dataset as one array pair per episode.

Validation works on joint trajectories far more than on images, and reading the
parquet tables directly is much faster than going through LeRobotDataset,
which decodes video for every item.
"""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path

import numpy as np
import pandas as pd

SOURCE_EPISODES_FILE = "source_episodes.json"
FPS_KEY = "fps"


@dataclasses.dataclass
class Episode:
    index: int
    source: str | None
    """Name of the raw recording folder (episode_XXXX), when known."""
    state: np.ndarray
    """(T, D) measured joint positions."""
    action: np.ndarray
    """(T, D) commanded joint positions."""

    @property
    def length(self) -> int:
        return len(self.state)


@dataclasses.dataclass
class JointGroups:
    """Index sets for the G1 + Brainco layout: 7 joints per arm, 6 per hand."""

    names: list[str]

    def _where(self, predicate) -> list[int]:
        return [i for i, name in enumerate(self.names) if predicate(name)]

    @property
    def arms(self) -> list[int]:
        return self._where(lambda n: "Hand" not in n)

    @property
    def hands(self) -> list[int]:
        return self._where(lambda n: "Hand" in n)

    @property
    def left_hand(self) -> list[int]:
        return self._where(lambda n: n.startswith("kLeftHand"))

    @property
    def right_hand(self) -> list[int]:
        return self._where(lambda n: n.startswith("kRightHand"))


def read_info(dataset_root: Path) -> dict:
    return json.loads((Path(dataset_root) / "meta" / "info.json").read_text())


def read_episode_meta(dataset_root: Path) -> pd.DataFrame:
    files = sorted((Path(dataset_root) / "meta" / "episodes").glob("*/*.parquet"))
    return pd.concat([pd.read_parquet(f) for f in files], ignore_index=True)


def read_frames(dataset_root: Path) -> pd.DataFrame:
    files = sorted((Path(dataset_root) / "data").glob("*/*.parquet"))
    return pd.concat([pd.read_parquet(f) for f in files], ignore_index=True)


def read_sources(dataset_root: Path) -> list[str] | None:
    path = Path(dataset_root) / "meta" / SOURCE_EPISODES_FILE
    return json.loads(path.read_text()) if path.exists() else None


def load_episodes(dataset_root: Path) -> list[Episode]:
    frames = read_frames(dataset_root)
    sources = read_sources(dataset_root)
    episodes = []
    for index, rows in frames.groupby("episode_index", sort=True):
        rows = rows.sort_values("frame_index")
        episodes.append(
            Episode(
                index=int(index),
                source=sources[int(index)] if sources else None,
                state=np.stack(rows["observation.state"].to_numpy()).astype(np.float64),
                action=np.stack(rows["action"].to_numpy()).astype(np.float64),
            )
        )
    return episodes


def joint_groups(dataset_root: Path) -> JointGroups:
    return JointGroups(read_info(dataset_root)["features"]["action"]["names"][0])
