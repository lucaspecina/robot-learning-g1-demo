#!/usr/bin/env python3
"""Geometría pura para mirar una referencia desde una distancia segura."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ObservationPose:
    x: float
    y: float
    yaw: float
    initial_distance_m: float
    requested_standoff_m: float


def compute_observation_pose(
    *,
    observer_x: float,
    observer_y: float,
    target_x: float,
    target_y: float,
    standoff_m: float,
) -> ObservationPose:
    """Conserva el lado desde el que se vio el objeto y queda mirándolo."""
    values = (observer_x, observer_y, target_x, target_y, standoff_m)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("la pose de observación contiene valores inválidos")
    if standoff_m <= 0.0:
        raise ValueError("la distancia de observación debe ser positiva")

    away_x = float(observer_x) - float(target_x)
    away_y = float(observer_y) - float(target_y)
    initial_distance = math.hypot(away_x, away_y)
    if initial_distance < 0.35:
        raise ValueError("el observador está demasiado cerca del objetivo")

    unit_x = away_x / initial_distance
    unit_y = away_y / initial_distance
    goal_x = float(target_x) + unit_x * float(standoff_m)
    goal_y = float(target_y) + unit_y * float(standoff_m)
    goal_yaw = math.atan2(float(target_y) - goal_y, float(target_x) - goal_x)
    return ObservationPose(
        x=goal_x,
        y=goal_y,
        yaw=goal_yaw,
        initial_distance_m=initial_distance,
        requested_standoff_m=float(standoff_m),
    )
