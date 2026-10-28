"""Schema, counts and invalid values (report section 1)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd

from toolkit.validation.episodes import Episode, read_episode_meta, read_frames, read_info

EXPECTED_QUANTILES = ("q01", "q99")


def schema_report(dataset_root: Path) -> pd.DataFrame:
    """One row per structural check with its result."""
    info = read_info(dataset_root)
    meta = read_episode_meta(dataset_root)
    frames = read_frames(dataset_root)
    stats = pd.read_json(Path(dataset_root) / "meta" / "stats.json")

    contiguous = all(
        (rows["frame_index"].sort_values().to_numpy() == np.arange(len(rows))).all()
        for _, rows in frames.groupby("episode_index")
    )
    index_spans = (meta["dataset_to_index"] - meta["dataset_from_index"] == meta["length"]).all()
    checks = {
        "episodios en info == filas de meta/episodes": info["total_episodes"] == len(meta),
        "cuadros en info == suma de largos == filas de data": info["total_frames"] == meta["length"].sum() == len(frames),
        "frame_index va de 0 a largo-1 sin huecos": contiguous,
        "rangos de índice coinciden con el largo": bool(index_spans),
        "stats.json trae q01 y q99": all(q in stats.index for q in EXPECTED_QUANTILES),
    }
    return pd.DataFrame({"ok": checks})


def invalid_values(episodes: list[Episode]) -> pd.DataFrame:
    """Non-finite values per episode, for state and action."""
    return pd.DataFrame(
        {
            "state_no_finitos": [int((~np.isfinite(e.state)).sum()) for e in episodes],
            "action_no_finitos": [int((~np.isfinite(e.action)).sum()) for e in episodes],
        },
        index=[e.index for e in episodes],
    )


def timestamp_steps(dataset_root: Path) -> np.ndarray:
    """Intervals between consecutive timestamps within episodes, in seconds."""
    frames = read_frames(dataset_root)
    steps = [np.diff(rows.sort_values("frame_index")["timestamp"].to_numpy()) for _, rows in frames.groupby("episode_index")]
    return np.concatenate(steps)
