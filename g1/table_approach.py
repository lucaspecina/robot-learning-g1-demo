#!/usr/bin/env python3
"""Cálculo puro de una pose de preaproximación desde una mesa medida."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TableStagingPose:
    x: float
    y: float
    yaw: float
    requested_standoff_m: float
    initial_surface_distance_m: float


def next_table_approach_attempt(
    current_attempts: int,
    maximum_attempts: int,
) -> tuple[int, bool]:
    """Reserva un intento y dice si todavía está dentro del presupuesto."""
    if isinstance(current_attempts, bool) or not isinstance(
        current_attempts,
        int,
    ):
        raise ValueError("la cantidad previa de intentos no es válida")
    if isinstance(maximum_attempts, bool) or not isinstance(
        maximum_attempts,
        int,
    ):
        raise ValueError("el máximo de intentos no es válido")
    if current_attempts < 0 or maximum_attempts < 1:
        raise ValueError("el presupuesto de intentos no puede ser negativo")
    attempt = current_attempts + 1
    return attempt, attempt <= maximum_attempts


def compute_table_staging_pose(
    *,
    robot_x: float,
    robot_y: float,
    table_x: float,
    table_y: float,
    standoff_m: float,
    world_bounds: dict | None = None,
    boundary_margin_m: float = 0.35,
) -> TableStagingPose:
    """Coloca la base sobre la línea de visión y mirando a la mesa."""
    values = (robot_x, robot_y, table_x, table_y, standoff_m)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("la pose de aproximación contiene valores inválidos")
    if standoff_m <= 0.0:
        raise ValueError("la separación de la mesa debe ser positiva")
    if boundary_margin_m < 0.0:
        raise ValueError("el margen de las paredes no puede ser negativo")

    away_x = float(robot_x) - float(table_x)
    away_y = float(robot_y) - float(table_y)
    initial_distance = math.hypot(away_x, away_y)
    if initial_distance < 0.35:
        raise ValueError(
            "el robot está demasiado cerca para usar la aproximación general"
        )

    unit_x = away_x / initial_distance
    unit_y = away_y / initial_distance
    goal_x = float(table_x) + unit_x * float(standoff_m)
    goal_y = float(table_y) + unit_y * float(standoff_m)
    goal_yaw = math.atan2(
        float(table_y) - goal_y,
        float(table_x) - goal_x,
    )

    if world_bounds is not None:
        required = {"xmin", "xmax", "ymin", "ymax"}
        if not isinstance(world_bounds, dict) or not required.issubset(
            world_bounds
        ):
            raise ValueError("faltan los límites verificables del entorno")
        if not (
            float(world_bounds["xmin"]) + boundary_margin_m
            <= goal_x
            <= float(world_bounds["xmax"]) - boundary_margin_m
            and float(world_bounds["ymin"]) + boundary_margin_m
            <= goal_y
            <= float(world_bounds["ymax"]) - boundary_margin_m
        ):
            raise ValueError(
                "la pose calculada queda demasiado cerca de una pared"
            )

    return TableStagingPose(
        x=goal_x,
        y=goal_y,
        yaw=goal_yaw,
        requested_standoff_m=float(standoff_m),
        initial_surface_distance_m=initial_distance,
    )
