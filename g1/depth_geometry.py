#!/usr/bin/env python3
"""Geometría pura para unir una detección de color con profundidad."""
from dataclasses import dataclass

import numpy as np

from perception_core import ImageBox

MIN_COLOR_PIXELS = 30


@dataclass(frozen=True)
class CameraPoint:
    """Punto medido respecto de la cámara óptica, en metros."""

    color: str
    right_m: float
    down_m: float
    forward_m: float
    pixel_x: float
    pixel_y: float
    sample_count: int


def colored_table_point(
    rgb_image: np.ndarray,
    depth_image: np.ndarray,
    intrinsics: np.ndarray,
    box: ImageBox,
) -> CameraPoint:
    """Mide la superficie roja o azul dentro de una caja ya detectada."""
    if rgb_image.shape[:2] != depth_image.shape:
        raise ValueError("color y profundidad tienen resoluciones distintas")
    if intrinsics.shape != (3, 3):
        raise ValueError("la calibración debe ser una matriz 3x3")
    if box.width <= 0 or box.height <= 0:
        raise ValueError("la caja visual está vacía")

    rgb = rgb_image[box.y1:box.y2, box.x1:box.x2].astype(np.int16)
    depth = depth_image[box.y1:box.y2, box.x1:box.x2]
    red = rgb[:, :, 0]
    blue = rgb[:, :, 2]
    masks = {
        "red": (red > 80) & (red > blue * 1.35),
        "blue": (blue > 80) & (blue > red * 1.35),
    }
    color = max(masks, key=lambda name: int(np.count_nonzero(masks[name])))
    valid = (
        masks[color]
        & np.isfinite(depth)
        & (depth > 0.05)
        & (depth < 50.0)
    )
    rows, columns = np.nonzero(valid)
    if rows.size < MIN_COLOR_PIXELS:
        raise ValueError(
            f"sólo hay {rows.size} píxeles de color con profundidad válida"
        )

    values = depth[valid].astype(np.float64)
    median_depth = float(np.median(values))
    # Una reflexión o un borde puede producir pocos valores extremos. La
    # mediana fija primero la superficie y este margen conserva su espesor.
    inlier = np.abs(values - median_depth) <= 0.20
    if int(np.count_nonzero(inlier)) < MIN_COLOR_PIXELS:
        raise ValueError("la profundidad del color no forma una superficie")
    rows = rows[inlier]
    columns = columns[inlier]
    values = values[inlier]

    pixel_x = float(np.median(columns + box.x1))
    pixel_y = float(np.median(rows + box.y1))
    forward = float(np.median(values))
    fx, fy = float(intrinsics[0, 0]), float(intrinsics[1, 1])
    cx, cy = float(intrinsics[0, 2]), float(intrinsics[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("la distancia focal no es válida")
    return CameraPoint(
        color=color,
        right_m=(pixel_x - cx) * forward / fx,
        down_m=(pixel_y - cy) * forward / fy,
        forward_m=forward,
        pixel_x=pixel_x,
        pixel_y=pixel_y,
        sample_count=int(values.size),
    )
