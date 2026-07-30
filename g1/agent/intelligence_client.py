#!/usr/bin/env python3
"""Cliente de la Jetson para decisiones lentas del servidor externo."""
import base64
import json
import os
import time
import urllib.error
import urllib.request
import uuid

try:
    from visual_evidence import validate_jpeg
except ImportError:
    from g1.visual_evidence import validate_jpeg


DEFAULT_SERVER_URL = "http://172.30.0.20:8000"
DEFAULT_TIMEOUT_S = 15.0
DEFAULT_DETECTION_TIMEOUT_S = 45.0
DEFAULT_PLANNER_TIMEOUT_S = 20.0
DEFAULT_REVIEW_TIMEOUT_S = 20.0
FAILURE_THRESHOLD = 3
OPEN_INTERVAL_S = 30.0
REVIEW_DECISIONS = {
    "continue",
    "retry",
    "revise",
    "ask_human",
    "stop",
}


def serialize_visual_evidence(evidence: dict) -> dict:
    """Codifica una sola imagen y evita aceptar evidencia ambigua."""
    if not isinstance(evidence, dict):
        raise RemoteIntelligenceError("la evidencia visual no es un objeto")
    if set(evidence) != {"purpose", "image", "input_ref", "detail"}:
        raise RemoteIntelligenceError(
            "la evidencia visual no tiene el contrato esperado"
        )
    purpose = evidence["purpose"]
    image = evidence["image"]
    detail = evidence["detail"]
    if (
        not isinstance(purpose, str)
        or not purpose.strip()
        or len(purpose) > 240
    ):
        raise RemoteIntelligenceError(
            "la evidencia visual no explica para qué se usa"
        )
    try:
        validate_jpeg(image)
    except ValueError as error:
        raise RemoteIntelligenceError(
            str(error)
        ) from error
    if not isinstance(evidence["input_ref"], dict):
        raise RemoteIntelligenceError(
            "la evidencia visual no tiene referencia local"
        )
    if detail not in {"low", "high", "auto"}:
        raise RemoteIntelligenceError(
            "la evidencia visual tiene un detalle inválido"
        )
    return {
        "purpose": purpose.strip(),
        "image_base64": base64.b64encode(image).decode("ascii"),
        "detail": detail,
    }


class RemoteIntelligenceError(RuntimeError):
    def __init__(
        self,
        message: str,
        *,
        request_id: str = None,
        raw_output: str = None,
        input_payload=None,
    ):
        super().__init__(message)
        self.request_id = request_id
        self.raw_output = raw_output
        self.input_payload = input_payload


class CircuitOpenError(RemoteIntelligenceError):
    pass


def validate_reading(reading: dict) -> dict:
    required = {"readable", "hour", "minute", "text"}
    if not isinstance(reading, dict) or set(reading) != required:
        raise RemoteIntelligenceError(
            "la lectura no tiene el formato esperado"
        )
    if not isinstance(reading["readable"], bool):
        raise RemoteIntelligenceError("readable no es booleano")
    if not isinstance(reading["hour"], int) or not 0 <= reading["hour"] <= 23:
        raise RemoteIntelligenceError("hour está fuera de rango")
    if (
        not isinstance(reading["minute"], int)
        or not 0 <= reading["minute"] <= 59
    ):
        raise RemoteIntelligenceError("minute está fuera de rango")
    if not isinstance(reading["text"], str):
        raise RemoteIntelligenceError("text no es una cadena")
    if reading["readable"]:
        expected_text = f"{reading['hour']:02d}:{reading['minute']:02d}"
        if reading["text"] != expected_text:
            raise RemoteIntelligenceError(
                "el texto y los campos numéricos no coinciden"
            )
    return reading


def validate_review(
    review: dict,
    outcome,
    skill_catalog: list[dict] = None,
) -> dict:
    outcome_state = (
        outcome.get("state")
        if isinstance(outcome, dict)
        else outcome
    )
    blocker = outcome.get("blocker") if isinstance(outcome, dict) else None
    required = {"decision", "reason", "revised_steps", "question"}
    if not isinstance(review, dict) or set(review) != required:
        raise RemoteIntelligenceError(
            "la revisión no tiene el formato esperado"
        )
    decision = review["decision"]
    if decision not in REVIEW_DECISIONS:
        raise RemoteIntelligenceError("decisión de revisión inválida")
    if not isinstance(review["reason"], str) or not review["reason"].strip():
        raise RemoteIntelligenceError("la revisión no explica su decisión")
    if not isinstance(review["revised_steps"], list):
        raise RemoteIntelligenceError("revised_steps no es una lista")
    if review["question"] is not None and not isinstance(
        review["question"],
        str,
    ):
        raise RemoteIntelligenceError("question debe ser texto o null")
    if decision == "continue" and outcome_state != "succeeded":
        raise RemoteIntelligenceError(
            "el modelo intentó continuar después de una falla"
        )
    if decision == "retry" and outcome_state not in {"failed", "blocked"}:
        raise RemoteIntelligenceError(
            "el modelo intentó repetir un paso exitoso"
        )
    if decision != "revise" and review["revised_steps"]:
        raise RemoteIntelligenceError(
            "el modelo cambió pasos sin elegir revise"
        )
    if decision == "revise" and not review["revised_steps"]:
        raise RemoteIntelligenceError(
            "el modelo eligió revise sin un plan pendiente"
        )
    if decision == "ask_human":
        if not review["question"] or not review["question"].strip():
            raise RemoteIntelligenceError(
                "el modelo no formuló la pregunta al operador"
            )
    elif review["question"] is not None:
        raise RemoteIntelligenceError(
            "el modelo agregó una pregunta fuera de ask_human"
        )
    if isinstance(blocker, dict) and blocker.get("type") == "missing_skill":
        missing_skill = blocker.get("skill")
        available = any(
            item.get("name") == missing_skill
            and item.get("availability") == "ready"
            for item in (skill_catalog or [])
            if isinstance(item, dict)
        )
        if not available and decision not in {"ask_human", "stop"}:
            raise RemoteIntelligenceError(
                "el modelo intentó continuar sin la skill faltante"
            )
    return review


class IntelligenceClient:
    def __init__(
        self,
        server_url: str = None,
        timeout_s: float = None,
        opener=None,
        monotonic=None,
    ):
        self.server_url = (
            server_url
            or os.environ.get("INTELLIGENCE_SERVER_URL")
            or DEFAULT_SERVER_URL
        ).rstrip("/")
        self.timeout_s = timeout_s or float(
            os.environ.get(
                "INTELLIGENCE_TIMEOUT_S",
                DEFAULT_TIMEOUT_S,
            )
        )
        self.opener = opener or urllib.request.urlopen
        self.monotonic = monotonic or time.monotonic
        self.consecutive_failures = 0
        self.opened_at = None

    def _allow_request(self):
        if self.opened_at is None:
            return
        elapsed = self.monotonic() - self.opened_at
        if elapsed < OPEN_INTERVAL_S:
            raise CircuitOpenError(
                "el servidor sigue temporalmente bloqueado después de fallas"
            )
        # Se permite un pedido de prueba. Si falla, el circuito se abre de
        # nuevo; si funciona, el contador se limpia.
        self.opened_at = None

    def _record_success(self):
        self.consecutive_failures = 0
        self.opened_at = None

    def _record_failure(self):
        self.consecutive_failures += 1
        if self.consecutive_failures >= FAILURE_THRESHOLD:
            self.opened_at = self.monotonic()

    def _post(self, path: str, payload: dict, timeout_s: float):
        self._allow_request()
        request_id = str(uuid.uuid4())
        body = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            f"{self.server_url}{path}",
            data=body,
            headers={
                "Content-Type": "application/json",
                "X-Request-ID": request_id,
            },
            method="POST",
        )

        try:
            with self.opener(request, timeout=timeout_s) as response:
                response_text = response.read().decode("utf-8")
                payload = json.loads(response_text)
        except urllib.error.HTTPError as error:
            raw_output = None
            input_payload = None
            try:
                failure_payload = json.loads(
                    error.read().decode("utf-8")
                )
                raw_output = failure_payload.get("raw_output")
                input_payload = failure_payload.get("model_input")
            except (AttributeError, UnicodeDecodeError, json.JSONDecodeError):
                pass
            self._record_failure()
            raise RemoteIntelligenceError(
                f"falló el pedido remoto: HTTP {error.code}",
                request_id=request_id,
                raw_output=raw_output,
                input_payload=input_payload,
            ) from error
        except (
            OSError,
            TimeoutError,
            urllib.error.URLError,
            json.JSONDecodeError,
            UnicodeDecodeError,
        ) as error:
            self._record_failure()
            raise RemoteIntelligenceError(
                f"falló el pedido remoto: {type(error).__name__}",
                request_id=request_id,
            ) from error

        if (
            not payload.get("ok")
            or payload.get("request_id") != request_id
        ):
            self._record_failure()
            raise RemoteIntelligenceError(
                "el servidor devolvió una respuesta inválida",
                request_id=request_id,
            )
        return payload, request_id, response_text

    def plan_mission(
        self,
        command: str,
        skill_catalog: list[dict],
        initial_facts: list[str],
    ) -> dict:
        """Pide una propuesta; la validación autoritativa queda en la Jetson."""
        timeout_s = float(
            os.environ.get(
                "INTELLIGENCE_PLANNER_TIMEOUT_S",
                DEFAULT_PLANNER_TIMEOUT_S,
            )
        )
        payload, request_id, _response_text = self._post(
            "/v1/plan-mission",
            {
                "command": command,
                "skill_catalog": skill_catalog,
                "initial_facts": initial_facts,
            },
            timeout_s,
        )
        plan = payload.get("plan")
        raw_output = payload.get("raw_output")
        model_input = payload.get("model_input")
        if (
            not isinstance(plan, dict)
            or not isinstance(plan.get("steps"), list)
            or not isinstance(raw_output, str)
            or not isinstance(model_input, dict)
        ):
            self._record_failure()
            raise RemoteIntelligenceError(
                "el servidor no conservó un plan trazable",
                request_id=request_id,
            )
        self._record_success()
        return {
            "steps": plan["steps"],
            "request_id": request_id,
            "model": payload.get("model"),
            "raw_output": raw_output,
            "model_input": model_input,
            "elapsed_s": payload.get("elapsed_s"),
        }

    def review_step(
        self,
        *,
        command: str,
        skill_catalog: list[dict],
        world_facts: list[str],
        completed_steps: list[dict],
        last_step: dict,
        outcome: dict,
        pending_steps: list[dict],
        review_count: int,
        visual_evidence: dict = None,
    ) -> dict:
        timeout_s = float(
            os.environ.get(
                "INTELLIGENCE_REVIEW_TIMEOUT_S",
                DEFAULT_REVIEW_TIMEOUT_S,
            )
        )
        request_payload = {
            "command": command,
            "skill_catalog": skill_catalog,
            "world_facts": world_facts,
            "completed_steps": completed_steps,
            "last_step": last_step,
            "outcome": outcome,
            "pending_steps": pending_steps,
            "review_count": review_count,
        }
        if visual_evidence is not None:
            request_payload["visual_evidence"] = (
                serialize_visual_evidence(visual_evidence)
            )
        payload, request_id, _response_text = self._post(
            "/v1/review-step",
            request_payload,
            timeout_s,
        )
        review = payload.get("review")
        raw_output = payload.get("raw_output")
        model_input = payload.get("model_input")
        if (
            not isinstance(raw_output, str)
            or not isinstance(model_input, dict)
        ):
            self._record_failure()
            raise RemoteIntelligenceError(
                "el servidor no conservó una revisión trazable",
                request_id=request_id,
            )
        try:
            validated = validate_review(
                review,
                outcome,
                skill_catalog,
            )
        except RemoteIntelligenceError:
            self._record_failure()
            raise
        self._record_success()
        return {
            **validated,
            "request_id": request_id,
            "model": payload.get("model"),
            "raw_output": raw_output,
            "model_input": model_input,
            "elapsed_s": payload.get("elapsed_s"),
        }

    def read_clock(self, image: bytes) -> dict:
        payload, request_id, _response_text = self._post(
            "/v1/read-clock",
            {
                "image_base64": base64.b64encode(image).decode("ascii"),
            },
            self.timeout_s,
        )
        if not isinstance(payload.get("reading"), dict):
            self._record_failure()
            raise RemoteIntelligenceError(
                "el servidor no devolvió una lectura"
            )
        raw_output = payload.get("raw_output")
        if not isinstance(raw_output, str):
            self._record_failure()
            raise RemoteIntelligenceError(
                "el servidor no conservó la salida literal del modelo",
                request_id=request_id,
            )

        try:
            reading = validate_reading(payload["reading"])
        except RemoteIntelligenceError:
            self._record_failure()
            raise
        self._record_success()
        return {
            **reading,
            "elapsed_s": payload.get("elapsed_s"),
            "request_id": request_id,
            "model": payload.get("model"),
            "raw_output": raw_output,
        }

    def detect_objects(self, image: bytes, labels: list[str]) -> dict:
        if (
            not isinstance(labels, list)
            or not labels
            or any(not isinstance(label, str) or not label.strip() for label in labels)
        ):
            raise ValueError("labels debe contener categorías no vacías")
        timeout_s = float(
            os.environ.get(
                "INTELLIGENCE_DETECTION_TIMEOUT_S",
                DEFAULT_DETECTION_TIMEOUT_S,
            )
        )
        payload, request_id, response_text = self._post(
            "/v1/detect-objects",
            {
                "image_base64": base64.b64encode(image).decode("ascii"),
                "labels": labels,
            },
            timeout_s,
        )
        detections = payload.get("detections")
        image_width = payload.get("image_width")
        image_height = payload.get("image_height")
        valid_size = (
            isinstance(image_width, int)
            and image_width > 0
            and isinstance(image_height, int)
            and image_height > 0
        )
        if not isinstance(detections, list) or not valid_size:
            self._record_failure()
            raise RemoteIntelligenceError(
                "el servidor no devolvió detecciones válidas"
            )
        validated = []
        for detection in detections:
            if not isinstance(detection, dict):
                self._record_failure()
                raise RemoteIntelligenceError("detección remota inválida")
            label = detection.get("label")
            confidence = detection.get("confidence")
            box = detection.get("box")
            valid_box = (
                isinstance(box, list)
                and len(box) == 4
                and all(isinstance(value, (int, float)) for value in box)
                and 0 <= box[0] < box[2] <= image_width
                and 0 <= box[1] < box[3] <= image_height
            )
            if (
                not isinstance(label, str)
                or not isinstance(confidence, (int, float))
                or not 0.0 <= confidence <= 1.0
                or not valid_box
            ):
                self._record_failure()
                raise RemoteIntelligenceError("detección remota inválida")
            validated.append(
                {
                    "label": label,
                    "confidence": float(confidence),
                    "box": [float(value) for value in box],
                }
            )
        self._record_success()
        return {
            "detections": validated,
            "image_width": image_width,
            "image_height": image_height,
            "model": payload.get("model"),
            "inference_s": payload.get("inference_s"),
            "elapsed_s": payload.get("elapsed_s"),
            "request_id": request_id,
            "raw_output": response_text,
        }
