"""Normalization statistics of a subset of a LeRobot dataset's episodes.

LeRobot stores statistics per episode and aggregates them for the whole
dataset. Re-aggregating a subset with the same function gives the statistics a
training run on that subset would have computed.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd
from lerobot.datasets.compute_stats import aggregate_stats


def _to_array(value) -> np.ndarray:
    # Parquet returns multi-dimensional stats (images are (3, 1, 1)) as nested
    # object arrays; stacking restores the original shape.
    array = np.asarray(value)
    if array.dtype == object:
        return np.stack([_to_array(item) for item in array])
    return array


def load_episode_stats(dataset_root: Path) -> dict[int, dict[str, dict[str, np.ndarray]]]:
    """Per-episode statistics, keyed by episode index."""
    files = sorted((dataset_root / "meta" / "episodes").glob("*/*.parquet"))
    episodes = pd.concat([pd.read_parquet(f) for f in files], ignore_index=True)
    stat_columns = [c for c in episodes.columns if c.startswith("stats/")]
    result: dict[int, dict[str, dict[str, np.ndarray]]] = {}
    for _, row in episodes.iterrows():
        stats: dict[str, dict[str, np.ndarray]] = {}
        for column in stat_columns:
            _, feature, stat = column.split("/")
            stats.setdefault(feature, {})[stat] = _to_array(row[column])
        result[int(row["episode_index"])] = stats
    return result


def subset_stats(dataset_root: Path, exclude: set[int] | None = None) -> dict[str, dict[str, np.ndarray]]:
    """Aggregated statistics over every episode except those in `exclude`."""
    per_episode = load_episode_stats(dataset_root)
    kept = [stats for index, stats in sorted(per_episode.items()) if index not in (exclude or set())]
    return aggregate_stats(kept)
