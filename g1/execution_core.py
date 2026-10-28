#!/usr/bin/env python3
"""Contrato local para vigilar capacidades que tardan en terminar."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class ExecutionState(str, Enum):
    """Estados terminales que entiende el agente sin depender de ROS."""

    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"
    TIMED_OUT = "timed_out"
    UNRESPONSIVE = "unresponsive"


@dataclass(frozen=True)
class WatchdogDecision:
    """Decisión del vigilante y motivo legible cuando debe interrumpir."""

    state: ExecutionState
    reason: str


class FeedbackWatchdog:
    """Detecta un plazo total vencido o una capacidad que dejó de responder.

    El reloj llega desde afuera para que las pruebas sean deterministas. Estos
    plazos usan tiempo de pared deliberadamente: aunque se pause Isaac, la
    computadora de a bordo debe poder detectar un proceso muerto y detenerlo.
    """

    def __init__(self, deadline_s: float, silence_timeout_s: float):
        if deadline_s <= 0.0:
            raise ValueError("el plazo total debe ser positivo")
        if silence_timeout_s <= 0.0:
            raise ValueError("el plazo sin respuesta debe ser positivo")
        self.deadline_s = float(deadline_s)
        self.silence_timeout_s = float(silence_timeout_s)
        self.started_at: Optional[float] = None
        self.last_feedback_at: Optional[float] = None

    def start(self, now: float):
        self.started_at = float(now)
        self.last_feedback_at = None

    def record_feedback(self, now: float):
        if self.started_at is None:
            raise RuntimeError("el vigilante no fue iniciado")
        self.last_feedback_at = float(now)

    def check(self, now: float) -> Optional[WatchdogDecision]:
        if self.started_at is None:
            raise RuntimeError("el vigilante no fue iniciado")

        now = float(now)
        if now - self.started_at > self.deadline_s:
            return WatchdogDecision(
                ExecutionState.TIMED_OUT,
                "venció el plazo total de la capacidad",
            )

        reference = (
            self.started_at
            if self.last_feedback_at is None
            else self.last_feedback_at
        )
        if now - reference > self.silence_timeout_s:
            return WatchdogDecision(
                ExecutionState.UNRESPONSIVE,
                "la capacidad dejó de informar progreso",
            )
        return None
