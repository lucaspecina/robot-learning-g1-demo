"""Quality of each demonstration: idle ends, pauses, grasp, retries, speed (report section 4)."""

from __future__ import annotations

import dataclasses

import numpy as np

from toolkit.validation.criteria import Criteria
from toolkit.validation.episodes import Episode


@dataclasses.dataclass
class IdleEnds:
    head: int
    """Frames before the arms move more than the threshold from the first pose."""
    tail: int
    """Frames after the arms stop moving relative to the last pose."""


def idle_ends(episode: Episode, arm_joints: list[int], criteria: Criteria) -> IdleEnds:
    """Adaptation to joint space of TRI's rule: activity starts once the pose leaves the initial one."""
    arms = episode.state[:, arm_joints]
    from_start = np.abs(arms - arms[0]).max(axis=1) > criteria.idle_threshold_rad
    from_end = np.abs(arms - arms[-1]).max(axis=1) > criteria.idle_threshold_rad
    head = int(np.argmax(from_start)) if from_start.any() else episode.length
    tail = int(np.argmax(from_end[::-1])) if from_end.any() else episode.length
    return IdleEnds(head, tail)


def pause_mask(episode: Episode, arm_joints: list[int], criteria: Criteria) -> np.ndarray:
    """Boolean per frame: inside a run of at least `pause_min_frames` with the arms still.

    Hands are left out: an open hand sits at exactly 0 for long stretches, which
    is rest, not hesitation.
    """
    still = np.abs(np.diff(episode.action[:, arm_joints], axis=0)).max(axis=1) < criteria.pause_velocity_rad
    still = np.concatenate([[False], still])
    mask = np.zeros_like(still)
    start = None
    for t, value in enumerate(np.append(still, False)):
        if value and start is None:
            start = t
        elif not value and start is not None:
            if t - start >= criteria.pause_min_frames:
                mask[start:t] = True
            start = None
    return mask


def mid_episode_pauses(episode: Episode, arm_joints: list[int], criteria: Criteria) -> float:
    """Fraction of the active part (idle ends removed) spent in pauses."""
    ends = idle_ends(episode, arm_joints, criteria)
    active = pause_mask(episode, arm_joints, criteria)[ends.head : episode.length - ends.tail]
    return float(active.mean()) if active.size else 0.0


def hand_closure(episode: Episode, hand_joints: list[int]) -> np.ndarray:
    """Mean command over the hand's joints per frame; higher means more closed."""
    return episode.action[:, hand_joints].mean(axis=1)


def grasp_cycles(closure: np.ndarray, high: float, low: float) -> int:
    """Open-close cycles with hysteresis: counts each time the hand closes past `high` after being below `low`."""
    cycles, closed = 0, False
    for value in closure:
        if not closed and value > high:
            cycles, closed = cycles + 1, True
        elif closed and value < low:
            closed = False
    return cycles


def path_length(episode: Episode, joints: list[int]) -> float:
    """Total joint-space distance travelled by the commanded arm pose."""
    return float(np.linalg.norm(np.diff(episode.action[:, joints], axis=0), axis=1).sum())


def mean_speed(episode: Episode, joints: list[int]) -> float:
    return float(np.linalg.norm(np.diff(episode.action[:, joints], axis=0), axis=1).mean())


def time_normalized(signal: np.ndarray, points: int = 100) -> np.ndarray:
    """Resample a (T, ...) trajectory to a fixed number of points so episodes of different length overlay."""
    source = np.linspace(0, 1, len(signal))
    target = np.linspace(0, 1, points)
    if signal.ndim == 1:
        return np.interp(target, source, signal)
    return np.stack([np.interp(target, source, signal[:, d]) for d in range(signal.shape[1])], axis=1)
