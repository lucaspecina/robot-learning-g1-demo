#!/usr/bin/env python3
"""Cálculos puros para cubrir el entorno con una cámara frontal."""

from dataclasses import dataclass
import math


TABLE_DETECTION_NAMES = {
    "red_table": "mesa_roja",
    "blue_table": "mesa_azul",
}


@dataclass(frozen=True)
class ScanPattern:
    view_count: int
    turn_increment_rad: float
    horizontal_fov_deg: float
    actual_overlap_deg: float

    @property
    def view_offsets_rad(self) -> tuple[float, ...]:
        return tuple(
            index * self.turn_increment_rad
            for index in range(self.view_count)
        )


def make_scan_pattern(
    horizontal_fov_deg: float,
    minimum_overlap_deg: float,
) -> ScanPattern:
    """Elige la menor cantidad de vistas que no deja huecos horizontales."""
    horizontal_fov_deg = float(horizontal_fov_deg)
    minimum_overlap_deg = float(minimum_overlap_deg)
    if not 0.0 < horizontal_fov_deg < 360.0:
        raise ValueError("el campo visual debe estar entre 0 y 360 grados")
    if not 0.0 <= minimum_overlap_deg < horizontal_fov_deg:
        raise ValueError(
            "la superposición debe ser menor que el campo visual"
        )

    maximum_step_deg = horizontal_fov_deg - minimum_overlap_deg
    view_count = max(1, math.ceil(360.0 / maximum_step_deg))
    turn_increment_deg = 360.0 / view_count
    return ScanPattern(
        view_count=view_count,
        turn_increment_rad=math.radians(turn_increment_deg),
        horizontal_fov_deg=horizontal_fov_deg,
        actual_overlap_deg=horizontal_fov_deg - turn_increment_deg,
    )


def local_table_candidate(
    status: dict,
    target: str,
    minimum_color_pixels: int = 100,
):
    """Devuelve el estado sólo si el detector local vio la mesa elegida."""
    expected_name = TABLE_DETECTION_NAMES.get(target)
    if expected_name is None or not isinstance(status, dict):
        return None
    if status.get("state") != "complete":
        return None
    reference = status.get("frame_ref")
    if not isinstance(reference, dict):
        return None
    detections = status.get("detections")
    if isinstance(detections, list) and expected_name in detections:
        return status
    color = "red" if target == "red_table" else "blue"
    counts = status.get("color_pixels")
    if (
        isinstance(counts, dict)
        and isinstance(counts.get(color), int)
        and counts[color] >= minimum_color_pixels
    ):
        return status
    return None
