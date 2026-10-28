"""Geometría pura para comprobar nubes 3D sin depender de ROS."""

import numpy as np


def transform_points(points, translation, quaternion_xyzw):
    """Lleva puntos a otro marco usando una transformación rígida."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    translation = np.asarray(translation, dtype=np.float64).reshape(3)
    x, y, z, w = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    norm = np.linalg.norm([x, y, z, w])
    if norm == 0.0:
        raise ValueError("el cuaternión no puede ser nulo")
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    rotation = np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    return points @ rotation.T + translation


def points_in_box(points, bounds):
    """Devuelve los puntos dentro de una caja cerrada XYZ."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    minimum = np.asarray(bounds[0], dtype=np.float64).reshape(3)
    maximum = np.asarray(bounds[1], dtype=np.float64).reshape(3)
    if np.any(maximum < minimum):
        raise ValueError("los límites de la caja están invertidos")
    mask = np.all((points >= minimum) & (points <= maximum), axis=1)
    return points[mask]
