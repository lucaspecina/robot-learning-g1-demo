"""Per-frame image metrics: blur, exposure, frozen frames (report section 5).

The video is decoded once, sequentially, which is much faster than random
access through LeRobotDataset. Frames are downscaled before computing metrics;
the metrics are relative across the dataset, not absolute.
"""

from __future__ import annotations

from pathlib import Path

import av
import cv2
import numpy as np
import pandas as pd

from toolkit.validation.episodes import read_episode_meta, read_info

DOWNSCALE = 2


def video_files(dataset_root: Path, camera: str) -> list[Path]:
    return sorted((Path(dataset_root) / "videos" / camera).glob("*/*.mp4"))


def frame_metrics(dataset_root: Path, camera: str) -> pd.DataFrame:
    """One row per decoded frame with blur, luminance and change from the previous frame."""
    rows = []
    for file_index, path in enumerate(video_files(dataset_root, camera)):
        previous = None
        with av.open(str(path)) as container:
            for frame in container.decode(video=0):
                gray = frame.to_ndarray(format="gray")[::DOWNSCALE, ::DOWNSCALE].astype(np.float32)
                rows.append(
                    {
                        "file_index": file_index,
                        "time": float(frame.time),
                        # Variance of the Laplacian: low values mean few sharp edges, i.e. blur.
                        "sharpness": float(cv2.Laplacian(gray, cv2.CV_32F).var()),
                        "luminance": float(gray.mean()),
                        "overexposed": float((gray > 250).mean()),
                        "underexposed": float((gray < 5).mean()),
                        "change": float(np.abs(gray - previous).mean()) if previous is not None else np.nan,
                    }
                )
                previous = gray
    return pd.DataFrame(rows)


def assign_episodes(metrics: pd.DataFrame, dataset_root: Path, camera: str) -> pd.DataFrame:
    """Attach episode_index and frame_index to each decoded frame using the episode time spans."""
    meta = read_episode_meta(dataset_root)
    fps = read_info(dataset_root)["fps"]
    prefix = f"videos/{camera}/"
    parts = []
    for _, ep in meta.iterrows():
        in_file = metrics[metrics["file_index"] == ep[prefix + "file_index"]]
        start, end = ep[prefix + "from_timestamp"], ep[prefix + "to_timestamp"]
        frames = in_file[(in_file["time"] >= start - 0.5 / fps) & (in_file["time"] < end - 0.5 / fps)].copy()
        frames["episode_index"] = int(ep["episode_index"])
        frames["frame_index"] = np.arange(len(frames))
        parts.append(frames)
    return pd.concat(parts, ignore_index=True)


def frozen_runs(change: np.ndarray, threshold: float, min_length: int = 2) -> list[tuple[int, int]]:
    """(start, length) of runs where the image did not change between consecutive frames."""
    still = np.nan_to_num(change, nan=np.inf) < threshold
    runs, start = [], None
    for t, value in enumerate(np.append(still, False)):
        if value and start is None:
            start = t
        elif not value and start is not None:
            if t - start >= min_length:
                runs.append((start, t - start))
            start = None
    return runs
