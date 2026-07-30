#!/usr/bin/env python3
"""Contrato puro para pedir búsquedas visuales puntuales."""
import json
import uuid

SEARCH_TARGET_LABELS = {
    # Grounding DINO encuentra la clase física. En las imágenes medidas asignó
    # "azul" con 0,805 a una mesa claramente roja: el color no es un atributo
    # confiable del modelo. El adaptador lo mide después dentro del recuadro.
    "red_table": ["a table"],
    "blue_table": ["a table"],
}


def make_search_request(target: str, request_id: str = None) -> dict:
    """Crea un pedido interno sin aceptar texto libre desde la misión."""
    if target not in SEARCH_TARGET_LABELS:
        raise ValueError(f"objetivo visual desconocido: {target}")
    return {
        "request_id": request_id or str(uuid.uuid4()),
        "target": target,
    }


def parse_search_request(data: str) -> tuple[str, str, list[str]]:
    """Valida el pedido ROS y devuelve identificador, objetivo y etiquetas."""
    try:
        payload = json.loads(data)
    except json.JSONDecodeError as error:
        raise ValueError("el pedido visual no es JSON válido") from error
    if not isinstance(payload, dict):
        raise ValueError("el pedido visual debe ser un objeto")
    request_id = payload.get("request_id")
    target = payload.get("target")
    if not isinstance(request_id, str) or not request_id.strip():
        raise ValueError("falta request_id")
    if target not in SEARCH_TARGET_LABELS:
        raise ValueError(f"objetivo visual desconocido: {target}")
    return request_id, target, list(SEARCH_TARGET_LABELS[target])
