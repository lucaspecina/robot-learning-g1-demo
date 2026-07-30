#!/usr/bin/env python3
"""Estado estructurado y observable de una misión del G1."""
from copy import deepcopy
import re
import threading
import time
import uuid


SCHEMA_VERSION = 1
MISSION_STATES = {
    "idle",
    "planning",
    "running",
    "succeeded",
    "failed",
    "blocked",
}
STEP_STATES = {
    "pending",
    "running",
    "succeeded",
    "failed",
    "blocked",
    "skipped",
}
STEP_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")


def build_demo_plan() -> list[dict]:
    """Devuelve la misión objetivo sin afirmar que todo ya está implementado."""
    return [
        {
            "id": "remember_home",
            "skill": "remember_home",
            "argument": None,
            "label": "Guardar el punto de partida",
        },
        {
            "id": "navigate_to_clock",
            "skill": "navigate_to",
            "argument": "clock",
            "label": "Ir hasta el reloj",
        },
        {
            "id": "confirm_clock",
            "skill": "look_at",
            "argument": "clock",
            "label": "Confirmar el reloj con la cámara",
        },
        {
            "id": "read_clock",
            "skill": "read_clock",
            "argument": None,
            "label": "Leer la hora",
        },
        {
            "id": "choose_table",
            "skill": "choose_table",
            "argument": None,
            "label": "Elegir mesa A roja o B azul",
        },
        {
            "id": "search_table",
            "skill": "search_table",
            "argument": "$selected_table",
            "label": "Buscar la mesa elegida",
        },
        {
            "id": "approach_table",
            "skill": "approach_table",
            "argument": "$selected_table",
            "label": "Acercarse y alinearse con la mesa",
        },
        {
            "id": "prepare_grasp",
            "skill": "set_arm_pose",
            "argument": "ready",
            "label": "Preparar los brazos",
        },
        {
            "id": "grasp_object",
            "skill": "grasp_object",
            "argument": None,
            "label": "Agarrar el objeto",
        },
        {
            "id": "transport_pose",
            "skill": "set_arm_pose",
            "argument": "transport",
            "label": "Adoptar la postura de transporte",
        },
        {
            "id": "return_home",
            "skill": "navigate_to",
            "argument": "home",
            "label": "Volver al punto de partida",
        },
    ]


def validate_plan(steps: list[dict]) -> list[dict]:
    """Valida el contrato antes de publicarlo o ejecutarlo."""
    if not isinstance(steps, list) or not steps:
        raise ValueError("el plan debe contener pasos")
    normalized = []
    seen_ids = set()
    for step in steps:
        if not isinstance(step, dict):
            raise ValueError("cada paso debe ser un objeto")
        step_id = step.get("id")
        skill = step.get("skill")
        label = step.get("label")
        if not isinstance(step_id, str) or not STEP_ID_PATTERN.fullmatch(step_id):
            raise ValueError(f"identificador de paso inválido: {step_id}")
        if step_id in seen_ids:
            raise ValueError(f"identificador de paso repetido: {step_id}")
        if not isinstance(skill, str) or not STEP_ID_PATTERN.fullmatch(skill):
            raise ValueError(f"skill inválida: {skill}")
        if not isinstance(label, str) or not label.strip():
            raise ValueError(f"falta la descripción de {step_id}")
        seen_ids.add(step_id)
        normalized.append(
            {
                "id": step_id,
                "skill": skill,
                "argument": step.get("argument"),
                "resolved_argument": None,
                "label": label,
                "state": "pending",
                "started_at": None,
                "finished_at": None,
                "result": None,
                "error": None,
            }
        )
    return normalized


class MissionTracker:
    """Conserva un único estado publicable sin depender de frases de log."""

    def __init__(self, publisher=None, clock=None, id_factory=None):
        self.publisher = publisher or (lambda _snapshot: None)
        self.clock = clock or time.time
        self.id_factory = id_factory or (lambda: str(uuid.uuid4()))
        self.lock = threading.RLock()
        self.state = self._empty_state()

    @staticmethod
    def _empty_state() -> dict:
        return {
            "schema_version": SCHEMA_VERSION,
            "mission_id": None,
            "command": None,
            "planner": None,
            "state": "idle",
            "active_step_id": None,
            "created_at": None,
            "updated_at": None,
            "steps": [],
            "decision": None,
            "result": None,
            "error": None,
        }

    def snapshot(self) -> dict:
        with self.lock:
            return deepcopy(self.state)

    def _update(self, mutator):
        with self.lock:
            mutator(self.state)
            self.state["updated_at"] = self.clock()
            snapshot = deepcopy(self.state)
        self.publisher(snapshot)
        return snapshot

    def begin(self, command: str, planner: str) -> dict:
        if not isinstance(command, str) or not command.strip():
            raise ValueError("la misión no puede estar vacía")
        now = self.clock()

        def mutate(state):
            state.clear()
            state.update(self._empty_state())
            state.update(
                {
                    "mission_id": self.id_factory(),
                    "command": command.strip(),
                    "planner": planner,
                    "state": "planning",
                    "created_at": now,
                }
            )

        return self._update(mutate)

    def set_plan(self, steps: list[dict]) -> dict:
        normalized = validate_plan(steps)

        def mutate(state):
            if state["state"] != "planning":
                raise ValueError("la misión no está siendo planificada")
            state["steps"] = normalized
            state["state"] = "running"

        return self._update(mutate)

    def stop(self, error: str, *, blocked: bool = False) -> dict:
        terminal_state = "blocked" if blocked else "failed"

        def mutate(state):
            if state["state"] not in {"planning", "running"}:
                raise ValueError("la misión ya terminó")
            state["state"] = terminal_state
            state["active_step_id"] = None
            state["error"] = error

        return self._update(mutate)

    def start_step(self, step_id: str, resolved_argument=None) -> dict:
        now = self.clock()

        def mutate(state):
            if state["state"] != "running":
                raise ValueError("la misión no está en ejecución")
            step = self._step(state, step_id)
            if step["state"] != "pending":
                raise ValueError(f"{step_id} no está pendiente")
            step["state"] = "running"
            step["started_at"] = now
            step["resolved_argument"] = resolved_argument
            state["active_step_id"] = step_id

        return self._update(mutate)

    def finish_step(self, step_id: str, result: str = None) -> dict:
        now = self.clock()

        def mutate(state):
            step = self._step(state, step_id)
            if step["state"] != "running":
                raise ValueError(f"{step_id} no está en ejecución")
            step["state"] = "succeeded"
            step["finished_at"] = now
            step["result"] = result
            state["active_step_id"] = None

        return self._update(mutate)

    def stop_step(
        self,
        step_id: str,
        error: str,
        *,
        blocked: bool = False,
    ) -> dict:
        now = self.clock()
        terminal_state = "blocked" if blocked else "failed"

        def mutate(state):
            step = self._step(state, step_id)
            if step["state"] != "running":
                raise ValueError(f"{step_id} no está en ejecución")
            step["state"] = terminal_state
            step["finished_at"] = now
            step["error"] = error
            state["active_step_id"] = None
            state["state"] = terminal_state
            state["error"] = error

        return self._update(mutate)

    def set_decision(self, evidence: str, rule: str, outcome: str) -> dict:
        def mutate(state):
            state["decision"] = {
                "evidence": evidence,
                "rule": rule,
                "outcome": outcome,
            }

        return self._update(mutate)

    def complete(self, result: str = None) -> dict:
        def mutate(state):
            if any(step["state"] != "succeeded" for step in state["steps"]):
                raise ValueError("no se puede completar una misión con pasos abiertos")
            state["state"] = "succeeded"
            state["active_step_id"] = None
            state["result"] = result

        return self._update(mutate)

    @staticmethod
    def _step(state: dict, step_id: str) -> dict:
        for step in state["steps"]:
            if step["id"] == step_id:
                return step
        raise ValueError(f"paso desconocido: {step_id}")
