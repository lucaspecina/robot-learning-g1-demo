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
    "superseded",
}
STEP_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")
MAX_PLAN_STEPS = 20


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
            "id": "scan_for_table",
            "skill": "scan_for_table",
            "argument": "$selected_table",
            "label": "Buscar alrededor la mesa elegida",
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
            "id": "align_with_table",
            "skill": "align_with_table",
            "argument": "$selected_table",
            "label": "Alinearse con precisión para agarrar",
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


def validate_plan(
    steps: list[dict],
    *,
    skill_catalog: list[dict] = None,
    initial_facts: list[str] = None,
) -> list[dict]:
    """Valida el contrato antes de publicarlo o ejecutarlo."""
    if not isinstance(steps, list) or not steps:
        raise ValueError("el plan debe contener pasos")
    if len(steps) > MAX_PLAN_STEPS:
        raise ValueError(
            f"el plan supera el máximo de {MAX_PLAN_STEPS} pasos"
        )
    catalog_by_name = {
        entry["name"]: entry
        for entry in (skill_catalog or [])
    }
    known_facts = set(initial_facts or [])
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
        argument = step.get("argument")
        availability = None
        if catalog_by_name:
            spec = catalog_by_name.get(skill)
            if spec is None:
                raise ValueError(f"skill no permitida: {skill}")
            variants = spec.get("variants")
            if not isinstance(variants, list) or not variants:
                raise ValueError(f"skill sin contrato ejecutable: {skill}")
            variant = next(
                (
                    candidate
                    for candidate in variants
                    if candidate.get("argument") == argument
                ),
                None,
            )
            if variant is None:
                raise ValueError(
                    f"argumento no permitido para {skill}: {argument}"
                )
            missing = [
                fact
                for fact in variant.get("preconditions", [])
                if fact not in known_facts
            ]
            if missing:
                raise ValueError(
                    f"{step_id} aparece antes de cumplir: "
                    + ", ".join(missing)
                )
            known_facts.update(variant.get("effects", []))
            availability = spec.get("availability")
        seen_ids.add(step_id)
        normalized.append(
            {
                "id": step_id,
                "skill": skill,
                "argument": argument,
                "resolved_argument": None,
                "label": label,
                "availability": availability,
                "state": "pending",
                "started_at": None,
                "finished_at": None,
                "result": None,
                "error": None,
                "measurements": None,
                "attempts": 0,
                "attempt_history": [],
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

    def set_plan(
        self,
        steps: list[dict],
        *,
        skill_catalog: list[dict] = None,
        initial_facts: list[str] = None,
    ) -> dict:
        normalized = validate_plan(
            steps,
            skill_catalog=skill_catalog,
            initial_facts=initial_facts,
        )

        def mutate(state):
            if state["state"] != "planning":
                raise ValueError("la misión no está siendo planificada")
            state["steps"] = normalized
            state["state"] = "running"

        return self._update(mutate)

    def set_planner(self, planner: str) -> dict:
        """Registra si el plan vino del modelo o del respaldo local."""
        if not isinstance(planner, str) or not planner.strip():
            raise ValueError("falta el nombre del planificador")

        def mutate(state):
            if state["state"] != "planning":
                raise ValueError("el planificador sólo cambia al planificar")
            state["planner"] = planner

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
            step["attempts"] += 1
            state["active_step_id"] = step_id

        return self._update(mutate)

    def finish_step(
        self,
        step_id: str,
        result: str = None,
        measurements: dict = None,
    ) -> dict:
        now = self.clock()

        def mutate(state):
            step = self._step(state, step_id)
            if step["state"] != "running":
                raise ValueError(f"{step_id} no está en ejecución")
            step["state"] = "succeeded"
            step["finished_at"] = now
            step["result"] = result
            step["measurements"] = deepcopy(measurements)
            state["active_step_id"] = None

        return self._update(mutate)

    def record_step_failure(
        self,
        step_id: str,
        error: str,
        *,
        blocked: bool = False,
        measurements: dict = None,
    ) -> dict:
        """Registra la falla sin cerrar aún la misión para poder revisarla."""
        now = self.clock()
        terminal_state = "blocked" if blocked else "failed"

        def mutate(state):
            if state["state"] != "running":
                raise ValueError("la misión no está en ejecución")
            step = self._step(state, step_id)
            if step["state"] != "running":
                raise ValueError(f"{step_id} no está en ejecución")
            step["state"] = terminal_state
            step["finished_at"] = now
            step["error"] = error
            step["measurements"] = deepcopy(measurements)
            step["attempt_history"].append(
                {
                    "attempt": step["attempts"],
                    "finished_at": now,
                    "state": terminal_state,
                    "error": error,
                    "measurements": deepcopy(measurements),
                }
            )
            state["active_step_id"] = None

        return self._update(mutate)

    def retry_step(self, step_id: str) -> dict:
        """Vuelve a dejar pendiente una falla, conservando su historial."""

        def mutate(state):
            if state["state"] != "running":
                raise ValueError("la misión no está en ejecución")
            step = self._step(state, step_id)
            if step["state"] not in {"failed", "blocked"}:
                raise ValueError(f"{step_id} no puede repetirse")
            step["state"] = "pending"
            step["started_at"] = None
            step["finished_at"] = None
            step["result"] = None
            step["error"] = None
            step["measurements"] = None
            state["active_step_id"] = None

        return self._update(mutate)

    def replace_pending_steps(
        self,
        steps: list[dict],
        *,
        skill_catalog: list[dict],
        current_facts: list[str],
    ) -> dict:
        """Reemplaza sólo el futuro; el historial ejecutado queda inmutable."""
        normalized = validate_plan(
            steps,
            skill_catalog=skill_catalog,
            initial_facts=current_facts,
        )

        def mutate(state):
            if state["state"] != "running":
                raise ValueError("la misión no está en ejecución")
            history = [
                step for step in state["steps"] if step["state"] != "pending"
            ]
            history_ids = {step["id"] for step in history}
            collisions = [
                step["id"]
                for step in normalized
                if step["id"] in history_ids
            ]
            if collisions:
                raise ValueError(
                    "la revisión reutiliza pasos ya ejecutados: "
                    + ", ".join(collisions)
                )
            for step in history:
                if step["state"] in {"failed", "blocked"}:
                    step["state"] = "superseded"
            state["steps"] = history + normalized
            state["active_step_id"] = None

        return self._update(mutate)

    def stop_step(
        self,
        step_id: str,
        error: str,
        *,
        blocked: bool = False,
        measurements: dict = None,
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
            step["measurements"] = deepcopy(measurements)
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
            if any(
                step["state"] not in {"succeeded", "superseded"}
                for step in state["steps"]
            ):
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
