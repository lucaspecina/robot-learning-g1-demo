"""Joint signal quality: jumps, jitter, saturation, frozen joints (report section 2)."""

from __future__ import annotations

import numpy as np

from toolkit.validation.criteria import Criteria
from toolkit.validation.episodes import Episode


def step_sizes(episodes: list[Episode], field: str) -> list[np.ndarray]:
    return [np.abs(np.diff(getattr(e, field), axis=0)) for e in episodes]


def jump_thresholds(episodes: list[Episode], field: str, criteria: Criteria) -> np.ndarray:
    """Per-joint step size above which a frame-to-frame change is a jump.

    Without the robot's velocity limits at hand, the threshold comes from the
    data: k times the 99.9th percentile step of each joint over the dataset.
    """
    steps = np.concatenate(step_sizes(episodes, field))
    return criteria.jump_factor * np.maximum(np.quantile(steps, 0.999, axis=0), 1e-9)


def jump_counts(episodes: list[Episode], field: str, criteria: Criteria) -> np.ndarray:
    """(episodes, joints) number of jumps."""
    threshold = jump_thresholds(episodes, field, criteria)
    return np.stack([(s > threshold).sum(axis=0) for s in step_sizes(episodes, field)])


def max_step_ratio(episodes: list[Episode], field: str, criteria: Criteria) -> np.ndarray:
    """(episodes, joints) largest step divided by the jump threshold; above 1 is a jump."""
    threshold = jump_thresholds(episodes, field, criteria)
    return np.stack([s.max(axis=0) / threshold for s in step_sizes(episodes, field)])


def jitter(episodes: list[Episode], field: str = "action") -> np.ndarray:
    """(episodes, joints) std of the second difference, a proxy for acceleration noise."""
    return np.stack([np.std(np.diff(getattr(e, field), n=2, axis=0), axis=0) for e in episodes])


def jerkiness(episodes: list[Episode]) -> np.ndarray:
    """Per-episode mean |Δaction| over joints, normalized by each joint's range.

    Mirrors the jerkiness score of the official LeRobot dataset visualizer.
    """
    all_actions = np.concatenate([e.action for e in episodes])
    joint_range = np.maximum(all_actions.max(axis=0) - all_actions.min(axis=0), 1e-9)
    return np.array([(np.abs(np.diff(e.action, axis=0)) / joint_range).mean() for e in episodes])


def hand_saturation(episodes: list[Episode], hand_joints: list[int], field: str, criteria: Criteria) -> np.ndarray:
    """(episodes, hand joints) fraction of frames at the closed end (1) of the hand range.

    A command pinned at the end asks for more than the hand can do; the policy
    learns to request something unreachable.
    """
    threshold = 1 - criteria.hand_saturation_eps
    return np.stack([(getattr(e, field)[:, hand_joints] >= threshold).mean(axis=0) for e in episodes])


def joint_ranges(episodes: list[Episode], field: str = "action") -> np.ndarray:
    """q99 - q01 of each joint over the whole dataset."""
    values = np.concatenate([getattr(e, field) for e in episodes])
    return np.quantile(values, 0.99, axis=0) - np.quantile(values, 0.01, axis=0)


def frozen_joints(episodes: list[Episode], criteria: Criteria, joints: list[int]) -> list[int]:
    """Joints (within `joints`) whose range is tiny compared to the widest one in that group.

    With mean/std normalization a joint that barely moves gets its noise
    amplified into many standard deviations.
    """
    ranges = joint_ranges(episodes)[joints]
    return [joints[i] for i in np.flatnonzero(ranges < criteria.frozen_range_fraction * ranges.max())]
