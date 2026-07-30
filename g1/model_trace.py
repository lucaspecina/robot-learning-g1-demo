#!/usr/bin/env python3
"""Contrato observable para llamadas a modelos locales o remotos."""
import time
import uuid


SCHEMA_VERSION = 1
MODEL_EVENT_STATES = {"running", "succeeded", "failed"}


def build_model_event(
    *,
    task: str,
    state: str,
    input_summary: str,
    input_ref: dict = None,
    event_id: str = None,
    request_id: str = None,
    model: str = None,
    raw_output: str = None,
    validated_output=None,
    duration_s: float = None,
    error: str = None,
    created_at: float = None,
) -> dict:
    """Construye un evento sin reescribir la respuesta literal del modelo."""
    if not isinstance(task, str) or not task.strip():
        raise ValueError("falta la tarea del modelo")
    if state not in MODEL_EVENT_STATES:
        raise ValueError(f"estado de modelo inválido: {state}")
    if not isinstance(input_summary, str) or not input_summary.strip():
        raise ValueError("falta el resumen de entrada")
    if raw_output is not None and not isinstance(raw_output, str):
        raise ValueError("la salida literal debe ser texto")
    if state == "succeeded" and raw_output is None:
        raise ValueError("un resultado exitoso debe conservar la salida literal")
    if state == "failed" and not error:
        raise ValueError("una falla debe explicar su causa")
    return {
        "schema_version": SCHEMA_VERSION,
        "event_id": event_id or str(uuid.uuid4()),
        "request_id": request_id,
        "task": task,
        "state": state,
        "input_summary": input_summary,
        "input_ref": input_ref,
        "model": model,
        "raw_output": raw_output,
        "validated_output": validated_output,
        "duration_s": duration_s,
        "error": error,
        "created_at": created_at if created_at is not None else time.time(),
    }
