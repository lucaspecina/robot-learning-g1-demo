"""Action versus state: control lag, tracking error, autocorrelation (report section 3)."""

from __future__ import annotations

import numpy as np

from toolkit.validation.episodes import Episode

MAX_LAG = 30


def lag_curve(episode: Episode, joint: int, max_lag: int = MAX_LAG) -> np.ndarray:
    """Mean |action_t - state_{t+k}| for k = 0..max_lag.

    The command is a target the motor reaches some frames later, so the k that
    minimizes this curve is the effective control delay.
    """
    a, s = episode.action[:, joint], episode.state[:, joint]
    usable = min(max_lag, len(a) - 2)
    return np.array([np.abs(a[: len(a) - k] - s[k:]).mean() for k in range(usable + 1)])


def best_lag(episode: Episode, joint: int, max_lag: int = MAX_LAG) -> int:
    return int(np.argmin(lag_curve(episode, joint, max_lag)))


def lag_matrix(episodes: list[Episode], joints: list[int]) -> np.ndarray:
    """(episodes, joints) best lag in frames."""
    return np.array([[best_lag(e, j) for j in joints] for e in episodes])


def tracking_error(episode: Episode, lag: int) -> np.ndarray:
    """(T - lag, D) action_t - state_{t+lag}: what the motor did not reach."""
    return episode.action[: episode.length - lag] - episode.state[lag:]


def action_copies_state(episode: Episode, atol: float = 1e-6) -> bool:
    """True when the recorded action is just the next state, a recording bug that leaves nothing to learn."""
    return bool(np.allclose(episode.action[:-1], episode.state[1:], atol=atol))


def autocorrelation(signal: np.ndarray, max_lag: int) -> np.ndarray:
    x = signal - signal.mean()
    denominator = (x * x).sum()
    if denominator == 0:
        return np.ones(max_lag + 1)
    return np.array([(x[: len(x) - k] * x[k:]).sum() / denominator for k in range(max_lag + 1)])


def decorrelation_lag(episodes: list[Episode], joints: list[int], max_lag: int = 120, level: float = 0.5) -> float:
    """Median lag (frames) where the action autocorrelation drops below `level`.

    The LeRobot visualizer reads it as a natural action-chunk length. It is a
    heuristic of that tool, not a result from a paper.
    """
    lags = []
    for e in episodes:
        for j in joints:
            curve = autocorrelation(e.action[:, j], min(max_lag, e.length - 1))
            below = np.flatnonzero(curve < level)
            if below.size:
                lags.append(below[0])
    return float(np.median(lags)) if lags else float("nan")
