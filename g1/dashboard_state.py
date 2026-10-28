#!/usr/bin/env python3
"""Reglas puras para que el tablero no mezcle pedidos con mediciones."""


def mission_scope_changed(previous: dict, incoming: dict) -> bool:
    """Indica si el tablero debe descartar los datos de otra misión."""
    previous_id = previous.get("mission_id") if isinstance(previous, dict) else None
    incoming_id = incoming.get("mission_id") if isinstance(incoming, dict) else None
    return previous_id != incoming_id


def measured_arm_label(status: dict) -> str:
    """Resume la pose medida; la orden por sí sola no demuestra movimiento."""
    if not isinstance(status, dict):
        return "sin medición"
    pose = status.get("pose")
    if not isinstance(pose, str) or not pose.strip():
        return "sin medición"
    state = "confirmada" if status.get("reached") else "moviéndose"
    return f"{pose.strip()} · {state}"
