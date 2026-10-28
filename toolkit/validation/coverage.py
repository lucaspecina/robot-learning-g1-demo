"""Diversity, coverage and train/held-out distance (report sections 7 and 8)."""

from __future__ import annotations

import numpy as np


def pca(points: np.ndarray, components: int = 2) -> tuple[np.ndarray, np.ndarray]:
    """Project standardized points onto their main axes; returns (projection, explained variance ratio)."""
    centered = (points - points.mean(axis=0)) / np.maximum(points.std(axis=0), 1e-9)
    _, singular, vt = np.linalg.svd(centered, full_matrices=False)
    explained = singular**2 / (singular**2).sum()
    return centered @ vt[:components].T, explained[:components]


def nearest_distances(queries: np.ndarray, references: np.ndarray, chunk: int = 2048) -> np.ndarray:
    """Euclidean distance from each query row to its nearest reference row."""
    result = np.empty(len(queries))
    for start in range(0, len(queries), chunk):
        block = queries[start : start + chunk]
        d2 = (block**2).sum(1)[:, None] - 2 * block @ references.T + (references**2).sum(1)[None, :]
        result[start : start + chunk] = np.sqrt(np.maximum(d2.min(axis=1), 0))
    return result
