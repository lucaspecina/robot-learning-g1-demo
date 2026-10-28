#!/usr/bin/env python3
"""Geometría pura para unir una detección de color con profundidad."""
from dataclasses import dataclass

import numpy as np

from perception_core import ImageBox

MIN_COLOR_PIXELS = 30
MIN_OBJECT_DEPTH_PIXELS = 20


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


@dataclass(frozen=True)
class DepthPoint:
    """Punto de una superficie visible respecto de la cámara, en metros."""

    right_m: float
    down_m: float
    forward_m: float
    pixel_x: float
    pixel_y: float
    sample_count: int


def camera_coordinates(
    pixel_x: float,
    pixel_y: float,
    forward_m: float,
    intrinsics: np.ndarray,
) -> tuple[float, float]:
    """Proyecta un píxel con profundidad a los ejes ópticos de la cámara."""
    if intrinsics.shape != (3, 3):
        raise ValueError("la calibración debe ser una matriz 3x3")
    fx, fy = float(intrinsics[0, 0]), float(intrinsics[1, 1])
    cx, cy = float(intrinsics[0, 2]), float(intrinsics[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("la distancia focal no es válida")
    return (
        (pixel_x - cx) * forward_m / fx,
        (pixel_y - cy) * forward_m / fy,
    )


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
    right, down = camera_coordinates(
        pixel_x,
        pixel_y,
        forward,
        intrinsics,
    )
    return CameraPoint(
        color=color,
        right_m=right,
        down_m=down,
        forward_m=forward,
        pixel_x=pixel_x,
        pixel_y=pixel_y,
        sample_count=int(values.size),
    )


def visible_object_point(
    depth_image: np.ndarray,
    intrinsics: np.ndarray,
    box: ImageBox,
    inset_ratio: float = 0.15,
) -> DepthPoint:
    """Mide la superficie visible dentro de una caja de objeto ya detectada."""
    if depth_image.ndim != 2:
        raise ValueError("la profundidad debe tener dos dimensiones")
    if box.width <= 0 or box.height <= 0:
        raise ValueError("la caja visual está vacía")
    if not 0.0 <= inset_ratio < 0.5:
        raise ValueError("el margen interior debe estar entre 0 y 0,5")

    inset_x = min(
        max(1, round(box.width * inset_ratio)),
        max(0, (box.width - 1) // 2),
    )
    inset_y = min(
        max(1, round(box.height * inset_ratio)),
        max(0, (box.height - 1) // 2),
    )
    sample_box = ImageBox(
        x1=box.x1 + inset_x,
        y1=box.y1 + inset_y,
        x2=box.x2 - inset_x,
        y2=box.y2 - inset_y,
    )
    depth = depth_image[
        sample_box.y1:sample_box.y2,
        sample_box.x1:sample_box.x2,
    ]
    valid = np.isfinite(depth) & (depth > 0.05) & (depth < 50.0)
    rows, columns = np.nonzero(valid)
    if rows.size < MIN_OBJECT_DEPTH_PIXELS:
        raise ValueError(
            f"sólo hay {rows.size} píxeles del objeto con profundidad válida"
        )

    values = depth[valid].astype(np.float64)
    median_depth = float(np.median(values))
    # Una caja de detección nunca calza exactamente con el contorno. Conservar
    # la capa alrededor de la mediana evita que fondo o mesa desplacen el
    # punto, sin asumir la forma ni el tamaño real del objeto.
    inlier = np.abs(values - median_depth) <= 0.08
    if int(np.count_nonzero(inlier)) < MIN_OBJECT_DEPTH_PIXELS:
        raise ValueError("la profundidad del objeto no forma una superficie")
    rows = rows[inlier]
    columns = columns[inlier]
    values = values[inlier]

    pixel_x = float(np.median(columns + sample_box.x1))
    pixel_y = float(np.median(rows + sample_box.y1))
    forward = float(np.median(values))
    right, down = camera_coordinates(
        pixel_x,
        pixel_y,
        forward,
        intrinsics,
    )
    return DepthPoint(
        right_m=right,
        down_m=down,
        forward_m=forward,
        pixel_x=pixel_x,
        pixel_y=pixel_y,
        sample_count=int(values.size),
    )
