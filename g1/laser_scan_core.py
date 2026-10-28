"""Normalización comprobable de los metadatos de un barrido láser."""
import math


def ray_count_from_metadata(
    angle_min: float,
    angle_max: float,
    angle_increment: float,
) -> int:
    """Calcula cuántos rayos declara el intervalo angular."""
    if not math.isfinite(angle_increment) or angle_increment <= 0.0:
        raise ValueError("angle_increment debe ser positivo y finito")
    return int(round((angle_max - angle_min) / angle_increment)) + 1


def angle_max_for_count(
    angle_min: float,
    angle_increment: float,
    ray_count: int,
) -> float:
    """Cierra el intervalo sobre exactamente la cantidad recibida."""
    if not math.isfinite(angle_increment) or angle_increment <= 0.0:
        raise ValueError("angle_increment debe ser positivo y finito")
    if ray_count <= 0:
        raise ValueError("ray_count debe ser positivo")
    return angle_min + (ray_count - 1) * angle_increment
