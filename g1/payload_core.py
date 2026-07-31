#!/usr/bin/env python3
"""Contrato puro para agregar y retirar una carga simulada."""
import json
import math


MAX_PAYLOAD_KG = 3.0


def parse_payload_request(raw: str) -> dict:
    """Valida una orden antes de que pueda modificar la física."""
    try:
        request = json.loads(raw)
    except (TypeError, json.JSONDecodeError) as error:
        raise ValueError("la orden de carga no es JSON válido") from error
    if not isinstance(request, dict):
        raise ValueError("la orden de carga debe ser un objeto")

    request_id = request.get("request_id")
    if not isinstance(request_id, str) or not request_id.strip():
        raise ValueError("falta request_id en la orden de carga")

    command = request.get("command")
    if command == "detach":
        return {
            "request_id": request_id.strip(),
            "command": "detach",
            "mass_kg": 0.0,
        }
    if command != "attach":
        raise ValueError(f"orden de carga desconocida: {command}")

    mass_kg = request.get("mass_kg")
    if isinstance(mass_kg, bool) or not isinstance(mass_kg, (int, float)):
        raise ValueError("mass_kg debe ser un número")
    mass_kg = float(mass_kg)
    if not math.isfinite(mass_kg) or mass_kg <= 0.0:
        raise ValueError("mass_kg debe ser mayor que cero")
    if mass_kg > MAX_PAYLOAD_KG:
        raise ValueError(
            f"mass_kg supera el máximo experimental de {MAX_PAYLOAD_KG:.1f} kg"
        )
    return {
        "request_id": request_id.strip(),
        "command": "attach",
        "mass_kg": mass_kg,
    }


def select_payload_body_indices(body_names: list[str]) -> list[int]:
    """Elige dos puntos físicos y evita sumar muñeca y mano a la vez."""
    candidates = []
    for marker in ("rubber_hand", "wrist_yaw_link"):
        indices = [
            index
            for index, name in enumerate(body_names)
            if marker in name
        ]
        sides = {
            "left" if "left" in body_names[index] else
            "right" if "right" in body_names[index] else
            "unknown"
            for index in indices
        }
        if len(indices) == 2 and sides == {"left", "right"}:
            return indices
        candidates.append((marker, indices))
    detail = ", ".join(
        f"{marker}={len(indices)}"
        for marker, indices in candidates
    )
    raise ValueError(
        "se esperaban dos puntos físicos, uno izquierdo y otro derecho; "
        + detail
    )


def payload_mass_values(
    baseline_masses: list[float],
    body_indices: list[int],
    mass_kg: float,
) -> list[float]:
    """Calcula valores absolutos para que repetir una orden no acumule peso."""
    if len(body_indices) != 2:
        raise ValueError("la carga debe repartirse entre exactamente dos puntos")
    if mass_kg < 0.0:
        raise ValueError("la masa no puede ser negativa")
    result = [float(value) for value in baseline_masses]
    extra_per_body = float(mass_kg) / 2.0
    for body_index in body_indices:
        result[body_index] += extra_per_body
    return result


def payload_geometry_measurements(
    wrist_positions: list[list[float]],
    pelvis_position: list[float],
) -> dict:
    """Mide dónde queda el dibujo; verlo entre muñecas no basta para aprobarlo."""
    if len(wrist_positions) != 2 or any(
        len(position) != 3
        for position in wrist_positions
    ):
        raise ValueError("se necesitan dos posiciones 3D de muñeca")
    if len(pelvis_position) != 3:
        raise ValueError("se necesita una posición 3D de pelvis")
    midpoint = [
        (float(wrist_positions[0][axis]) + float(wrist_positions[1][axis]))
        / 2.0
        for axis in range(3)
    ]
    separation = math.dist(wrist_positions[0], wrist_positions[1])
    relative = [
        midpoint[axis] - float(pelvis_position[axis])
        for axis in range(3)
    ]
    return {
        "wrist_separation_m": round(separation, 3),
        "visual_position_world_m": [round(value, 3) for value in midpoint],
        "visual_offset_from_pelvis_world_m": [
            round(value, 3)
            for value in relative
        ],
        "visual_distance_from_pelvis_m": round(
            math.sqrt(sum(value * value for value in relative)),
            3,
        ),
    }
