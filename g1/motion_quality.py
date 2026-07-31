"""Métricas simples de calidad del movimiento del torso."""
import math

import numpy as np


def motion_quality_metrics(samples: list[dict]) -> dict:
    """Resume oscilación e inclinación sin confundirlas con una sola muestra."""
    if len(samples) < 2:
        raise ValueError("hacen falta al menos dos muestras de movimiento")

    roll = np.asarray([sample["roll_rad"] for sample in samples], dtype=float)
    pitch = np.asarray([sample["pitch_rad"] for sample in samples], dtype=float)
    height = np.asarray([sample["height_m"] for sample in samples], dtype=float)
    angular_x = np.asarray(
        [sample["angular_x_radps"] for sample in samples],
        dtype=float,
    )
    angular_y = np.asarray(
        [sample["angular_y_radps"] for sample in samples],
        dtype=float,
    )
    angular_speed = np.hypot(angular_x, angular_y)

    def central_span(values):
        return float(np.percentile(values, 95) - np.percentile(values, 5))

    tilt = np.maximum(np.abs(roll), np.abs(pitch))
    return {
        "sample_count": len(samples),
        "roll_p90_span_deg": math.degrees(central_span(roll)),
        "pitch_p90_span_deg": math.degrees(central_span(pitch)),
        "tilt_p95_deg": math.degrees(float(np.percentile(tilt, 95))),
        "angular_speed_rms_radps": float(
            np.sqrt(np.mean(np.square(angular_speed)))
        ),
        "angular_speed_p95_radps": float(
            np.percentile(angular_speed, 95)
        ),
        "height_p90_span_m": central_span(height),
    }
