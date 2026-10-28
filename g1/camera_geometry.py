#!/usr/bin/env python3
"""Geometría pura de la cámara, independiente de Isaac."""
import math


def horizontal_field_of_view_deg(
    focal_length_mm: float,
    horizontal_aperture_mm: float,
) -> float:
    """Calcula cuánto ancho abarca la cámara en grados."""
    return math.degrees(
        2.0 * math.atan(horizontal_aperture_mm / (2.0 * focal_length_mm))
    )


def camera_rotation(downward_pitch_deg: float):
    """Orienta la cámara al frente con inclinación positiva hacia abajo."""
    # En la convención ROS de la cámara, el giro que baja su eje óptico tiene
    # signo negativo. El signo opuesto hizo que una configuración documentada
    # como 20° abajo mirara realmente 20° arriba.
    half = math.radians(-downward_pitch_deg) / 2.0
    cosine, sine = math.cos(half), math.sin(half)
    base_w, base_x, base_y, base_z = 0.5, -0.5, 0.5, -0.5
    return (
        base_w * cosine - base_x * sine,
        base_w * sine + base_x * cosine,
        base_y * cosine + base_z * sine,
        base_z * cosine - base_y * sine,
    )


def rotate_vector(quaternion, vector):
    """Rota un vector para poder probar la dirección óptica resultante."""
    w, x, y, z = quaternion
    vx, vy, vz = vector
    first_cross = (
        y * vz - z * vy,
        z * vx - x * vz,
        x * vy - y * vx,
    )
    second_cross = (
        y * first_cross[2] - z * first_cross[1],
        z * first_cross[0] - x * first_cross[2],
        x * first_cross[1] - y * first_cross[0],
    )
    return tuple(
        component
        + 2 * w * cross
        + 2 * second
        for component, cross, second in zip(
            vector,
            first_cross,
            second_cross,
        )
    )


def optical_forward(downward_pitch_deg: float):
    """Devuelve hacia dónde apunta el eje óptico en el cuerpo del robot."""
    return rotate_vector(
        camera_rotation(downward_pitch_deg),
        (0.0, 0.0, 1.0),
    )
