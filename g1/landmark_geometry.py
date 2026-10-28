#!/usr/bin/env python3
"""Selección robusta de una referencia espacial observada varias veces."""
import math
import statistics


def select_consistent_landmark(
    samples: list[dict],
    *,
    class_id: str,
    now: float,
    max_age_s: float,
    minimum_samples: int = 2,
    maximum_spread_m: float = 0.20,
    minimum_height_m: float = 0.40,
    maximum_height_m: float = 2.50,
) -> dict | None:
    """Acepta el grupo reciente más consistente, no una detección aislada."""
    if minimum_samples < 1:
        raise ValueError("la cantidad mínima de mediciones debe ser positiva")
    if max_age_s <= 0.0 or maximum_spread_m <= 0.0:
        raise ValueError("los límites temporales y espaciales deben ser positivos")

    recent = []
    seen_frames = set()
    for sample in reversed(samples):
        if not isinstance(sample, dict):
            continue
        try:
            values = tuple(float(sample[key]) for key in ("x", "y", "z"))
            received_at = float(sample["received_at"])
        except (KeyError, TypeError, ValueError):
            continue
        if (
            sample.get("class_id") != class_id
            or sample.get("coordinate_frame") != "map"
            or now - received_at > max_age_s
            or now < received_at
            or not all(math.isfinite(value) for value in values)
            or not minimum_height_m <= values[2] <= maximum_height_m
        ):
            continue
        frame_ref = sample.get("frame_ref")
        frame_key = repr(frame_ref)
        if frame_key in seen_frames:
            continue
        seen_frames.add(frame_key)
        normalized = dict(sample)
        normalized.update(
            {
                "x": values[0],
                "y": values[1],
                "z": values[2],
                "received_at": received_at,
            }
        )
        recent.append(normalized)

    best_cluster = []
    for anchor in recent:
        cluster = [
            sample
            for sample in recent
            if math.dist(
                (anchor["x"], anchor["y"], anchor["z"]),
                (sample["x"], sample["y"], sample["z"]),
            )
            <= maximum_spread_m
        ]
        if len(cluster) > len(best_cluster):
            best_cluster = cluster
    if len(best_cluster) < minimum_samples:
        return None

    x = statistics.median(float(sample["x"]) for sample in best_cluster)
    y = statistics.median(float(sample["y"]) for sample in best_cluster)
    z = statistics.median(float(sample["z"]) for sample in best_cluster)
    spread = max(
        math.dist(
            (x, y, z),
            (float(sample["x"]), float(sample["y"]), float(sample["z"])),
        )
        for sample in best_cluster
    )
    newest = max(best_cluster, key=lambda sample: sample["received_at"])
    return {
        "class_id": class_id,
        "confidence": sum(
            float(sample.get("confidence", 0.0)) for sample in best_cluster
        )
        / len(best_cluster),
        "x": x,
        "y": y,
        "z": z,
        "coordinate_frame": "map",
        "frame_ref": newest.get("frame_ref"),
        "received_at": float(newest["received_at"]),
        "sample_count": len(best_cluster),
        "spread_m": spread,
    }
