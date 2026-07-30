#!/usr/bin/env python3
"""Selección pura del objeto medido que pertenece a la mesa elegida."""
import math


def horizontal_distance(first: dict, second: dict) -> float:
    return math.hypot(
        float(first["x"]) - float(second["x"]),
        float(first["y"]) - float(second["y"]),
    )


def select_object_near_table(
    candidates: list[dict],
    table_point: dict,
    *,
    max_horizontal_distance_m: float = 0.75,
    minimum_height_delta_m: float = -0.10,
    maximum_height_delta_m: float = 0.70,
) -> dict | None:
    """Elige una medición reciente sobre la mesa, no cualquier objeto visto."""
    if (
        not isinstance(table_point, dict)
        or table_point.get("coordinate_frame") != "map"
    ):
        return None
    if max_horizontal_distance_m <= 0:
        raise ValueError("la distancia máxima debe ser positiva")

    eligible = []
    for candidate in candidates:
        if (
            not isinstance(candidate, dict)
            or candidate.get("class_id") != "transport_object"
            or candidate.get("coordinate_frame") != "map"
        ):
            continue
        try:
            distance = horizontal_distance(candidate, table_point)
            height_delta = float(candidate["z"]) - float(table_point["z"])
            confidence = float(candidate["confidence"])
        except (KeyError, TypeError, ValueError):
            continue
        if (
            not math.isfinite(distance)
            or not math.isfinite(height_delta)
            or not math.isfinite(confidence)
        ):
            continue
        if (
            distance <= max_horizontal_distance_m
            and minimum_height_delta_m
            <= height_delta
            <= maximum_height_delta_m
        ):
            eligible.append((distance, -confidence, candidate))
    if not eligible:
        return None
    return min(eligible, key=lambda item: (item[0], item[1]))[2]
