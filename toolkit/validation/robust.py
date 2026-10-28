"""Robust statistics shared by the checks."""

import numpy as np


def mad(x: np.ndarray, axis=None) -> np.ndarray:
    """Median absolute deviation: a spread measure that extreme values do not drag."""
    median = np.median(x, axis=axis, keepdims=True)
    return np.median(np.abs(x - median), axis=axis)


def robust_z(x: np.ndarray) -> np.ndarray:
    """How many MADs each value sits from the median (scaled to match a std for normal data)."""
    x = np.asarray(x, dtype=float)
    spread = 1.4826 * mad(x)
    return (x - np.median(x)) / max(float(spread), 1e-12)
