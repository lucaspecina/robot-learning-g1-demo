"""Drift along the recording order (report section 6)."""

from __future__ import annotations

import dataclasses

import numpy as np


@dataclasses.dataclass
class ChangePoint:
    split: int
    """First episode (in recording order) after the change."""
    score: float
    """|mean before - mean after| divided by the pooled within-group std."""


def change_point(series: np.ndarray, min_size: int = 1) -> ChangePoint:
    """Best single split of a per-episode series into two groups with different means.

    A simple two-segment test; enough to find one regime change such as a
    hardware fix in the middle of a recording campaign.
    """
    best = ChangePoint(split=0, score=0.0)
    for k in range(min_size, len(series) - min_size + 1):
        before, after = series[:k], series[k:]
        pooled = np.sqrt((before.var() * len(before) + after.var() * len(after)) / len(series))
        score = abs(before.mean() - after.mean()) / max(pooled, 1e-12)
        if score > best.score:
            best = ChangePoint(split=k, score=float(score))
    return best
