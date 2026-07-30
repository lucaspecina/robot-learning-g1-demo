#!/usr/bin/env python3
"""El agente local: convierte una misión en pasos observables y los ejecuta.

No toca motores ni velocidades. Publica objetivos para las capacidades del
robot y conserva, por separado, tres tipos de evidencia:

  recibe:  /g1/mission                    orden original en castellano
           /g1/detections                 objetos vistos por la cámara
           /g1/clock_crop/compressed      recorte exacto del reloj
           /g1/perception/evidence/compressed
                                          cuadro visual enlazado por fecha
           /g1/odom                       ubicación medida del cuerpo
           /g1/mobility/status            confirmación de espera segura
           /g1/arm_status                 medición real de los brazos
           /g1/perception/local_detection_status
                                          resultado nuevo del detector local
           /g1/perception/search_status   resultado de búsqueda puntual
           /g1/table_detections_3d        mesa ubicada desde sensores

  usa:     /g1/navigate_to_pose           tarea de navegación cancelable
           /g1/spin                       giro relativo cancelable

  publica: /g1/arm_pose                   postura pedida a los brazos
           /g1/perception/search_request  categoría visual acotada
           /g1/model_input/compressed      JPEG exacto enviado a un modelo
           /g1/mission_status             relato humano, sólo para el historial
           /g1/mission_state              estado estructurado de cada paso
           /g1/model_events               salida literal de LLM/VLM y validación

Las decisiones lentas se piden al servidor externo. Si falla el planificador,
la misión conocida conserva un respaldo local; cualquier ejecución sigue
validada y el control local mantiene el equilibrio.
"""
import json
import math
import os
import sys
import threading
import time
import uuid
from collections import OrderedDict
from copy import deepcopy
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

from intelligence_client import (
    IntelligenceClient,
    RemoteIntelligenceError,
)
from active_search import local_table_candidate, make_scan_pattern
from camera_geometry import horizontal_field_of_view_deg
from execution_core import FeedbackWatchdog
from mission_contract import MissionTracker, build_demo_plan, validate_plan
from model_trace import build_model_event
from navigation_core import normalize_angle
from open_vocabulary_core import make_search_request
from scene_layout import NAVIGATION_TARGETS, WORLD_BOUNDS
from skill_catalog import (
    INITIAL_WORLD_FACTS,
    SKILL_CATALOG,
    skill_catalog_for_model,
)
from table_approach import (
    compute_table_staging_pose,
    next_table_approach_attempt,
)
from visual_evidence import (
    CLOCK_CROP_TOPIC,
    MODEL_INPUT_TOPIC,
    VISUAL_EVIDENCE_TOPIC,
    image_ref,
    image_ref_key,
    is_complete_jpeg,
)


STATE_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
MODEL_EVENT_QOS = QoSProfile(
    depth=20,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

# Estas son poses seguras desde las que usar un objeto, no las coordenadas del
# objeto. La mesa no figura: la misión objetivo exige encontrarla con sensores.
SEMANTIC_MAP = {
    "clock": NAVIGATION_TARGETS["reloj"],
}
DETECTION_NAMES = {
    "clock": "reloj",
    "bottle": "botella",
}
ARM_POSES = {
    "rest": "reposo",
    "ready": "listo",
    "transport": "transporte",
}
MAX_MISSION_REVIEWS = 20
MAX_SAME_STEP_RETRIES = 1
VISUAL_EVIDENCE_CACHE_SIZE = 24
# Grounding DINO puede tardar cerca de un minuto. La fecha exacta evita
# confundir cuadros aunque la memoria deba sobrevivir durante esa inferencia.
VISUAL_EVIDENCE_MAX_AGE_S = 90.0
# La lente actual mide 108,1°. Reservar 30° de superposición deja cinco
# vistas con margen para no depender de una detección justo en el borde.
SEARCH_HORIZONTAL_FOV_DEG = horizontal_field_of_view_deg(7.6, 20.955)
SEARCH_MINIMUM_OVERLAP_DEG = 30.0
LOCAL_UPDATES_PER_VIEW = 2
LOCAL_VIEW_TIMEOUT_S = 12.0
COLOR_SCOUT_MIN_PIXELS = 600
# Un segundo barrido puede ser útil después de cambiar de lugar. Un tercero
# sin evidencia nueva sólo acumula movimiento y deriva en un robot físico.
MAX_ACTIVE_SCANS_PER_MISSION = 2
# A 1,4 m la prueba dedicada sólo veía la botella y perdía la mesa por debajo
# del cuadro; a 2,5 m del centro la mesa se midió repetidamente. Como la
# profundidad entrega un punto de su borde visible, 2,2 m respecto de esa
# superficie reproduce esa zona observable sin inventar el centro del mueble.
TABLE_STAGING_STANDOFF_M = 2.20
TABLE_STAGING_MIN_SURFACE_DISTANCE_M = 1.80
TABLE_STAGING_MAX_SURFACE_DISTANCE_M = 2.80
# Una esquina distinta de la misma mesa puede mover el punto observado varios
# grados. Esta tolerancia sólo valida la vista amplia de preaproximación; la
# alineación de agarre tendrá un límite propio y mucho más estricto.
TABLE_STAGING_MAX_YAW_ERROR_DEG = 20.0
MAX_TABLE_APPROACH_ATTEMPTS_PER_MISSION = 2
MIN_SAFE_BODY_HEIGHT_M = 0.65


def succeeded(message: str, measurements: dict = None) -> dict:
    outcome = {"state": "succeeded", "message": message}
    if measurements:
        outcome["measurements"] = measurements
    return outcome


def failed(message: str, measurements: dict = None) -> dict:
    outcome = {"state": "failed", "message": message}
    if measurements:
        outcome["measurements"] = measurements
    return outcome


def blocked(
    message: str,
    measurements: dict = None,
    blocker: dict = None,
) -> dict:
    outcome = {"state": "blocked", "message": message}
    if measurements:
        outcome["measurements"] = measurements
    if blocker:
        outcome["blocker"] = blocker
    return outcome


class Agent(Node):
    def __init__(self):
        super().__init__("agent")
        self.detections = {}
        self.local_detection_status = None
        self.local_detection_generation = 0
        self.local_detection_condition = threading.Condition()
        self.current_pose = None
        self.mobility_owner = None
        self.arm_status = None
        self.search_status = None
        self.localized_tables = {}
        self.mission_thread = None
        self.clock_crop = None
        self.clock_crop_ref = None
        self.clock_crop_received_at = None
        self.visual_evidence = OrderedDict()
        self.visual_evidence_lock = threading.Lock()
        self.current_review_evidence = None
        self.mission_context = {}
        self.intelligence = IntelligenceClient()

        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            "/g1/navigate_to_pose",
        )
        self.spin_client = ActionClient(
            self,
            Spin,
            "/g1/spin",
        )
        self.arms_pub = self.create_publisher(String, "/g1/arm_pose", 10)
        self.search_pub = self.create_publisher(
            String,
            "/g1/perception/search_request",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/g1/mission_status",
            10,
        )
        self.mission_state_pub = self.create_publisher(
            String,
            "/g1/mission_state",
            STATE_QOS,
        )
        self.model_events_pub = self.create_publisher(
            String,
            "/g1/model_events",
            MODEL_EVENT_QOS,
        )
        self.model_input_pub = self.create_publisher(
            CompressedImage,
            MODEL_INPUT_TOPIC,
            2,
        )
        self.mission_tracker = MissionTracker(
            publisher=self.publish_mission_state,
        )

        self.create_subscription(String, "/g1/mission", self.on_mission, 10)
        self.create_subscription(
            String,
            "/g1/detections",
            self.on_detections,
            10,
        )
        self.create_subscription(
            String,
            "/g1/perception/local_detection_status",
            self.on_local_detection_status,
            10,
        )
        self.create_subscription(
            CompressedImage,
            CLOCK_CROP_TOPIC,
            self.on_clock_crop,
            2,
        )
        self.create_subscription(
            CompressedImage,
            VISUAL_EVIDENCE_TOPIC,
            self.on_visual_evidence,
            qos_profile_sensor_data,
        )
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(
            String,
            "/g1/mobility/status",
            self.on_mobility_status,
            10,
        )
        self.create_subscription(
            String,
            "/g1/arm_status",
            self.on_arm_status,
            10,
        )
        self.create_subscription(
            String,
            "/g1/perception/search_status",
            self.on_search_status,
            10,
        )
        self.create_subscription(
            Detection3DArray,
            "/g1/table_detections_3d",
            self.on_table_detections,
            qos_profile_sensor_data,
        )

        self.publish_mission_state(self.mission_tracker.snapshot())
        self.get_logger().info(
            "agente listo. Esperando misiones en /g1/mission"
        )

    # ---------- entradas ----------

    def on_detections(self, message: String):
        try:
            self.detections = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    def on_local_detection_status(self, message: String):
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(status, dict) or status.get("state") != "complete":
            return
        with self.local_detection_condition:
            self.local_detection_status = status
            self.local_detection_generation += 1
            self.local_detection_condition.notify_all()

    def on_mobility_status(self, message: String):
        try:
            self.mobility_owner = str(json.loads(message.data)["owner"])
        except (json.JSONDecodeError, KeyError, TypeError):
            pass

    def on_clock_crop(self, message: CompressedImage):
        data = bytes(message.data)
        if not is_complete_jpeg(data):
            return
        self.clock_crop = data
        self.clock_crop_ref = image_ref(CLOCK_CROP_TOPIC, message.header)
        self.clock_crop_received_at = time.monotonic()

    def on_visual_evidence(self, message: CompressedImage):
        data = bytes(message.data)
        if not is_complete_jpeg(data):
            return
        reference = image_ref(VISUAL_EVIDENCE_TOPIC, message.header)
        with self.visual_evidence_lock:
            self.visual_evidence[image_ref_key(reference)] = {
                "image": data,
                "input_ref": reference,
                "received_at": time.monotonic(),
            }
            while len(self.visual_evidence) > VISUAL_EVIDENCE_CACHE_SIZE:
                self.visual_evidence.popitem(last=False)

    def on_odom(self, message: Odometry):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0
            * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0
            - 2.0
            * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        self.current_pose = (
            float(position.x),
            float(position.y),
            float(position.z),
            float(yaw),
        )

    def on_arm_status(self, message: String):
        try:
            self.arm_status = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    def on_search_status(self, message: String):
        try:
            self.search_status = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    def on_table_detections(self, message: Detection3DArray):
        frame_reference = image_ref(
            VISUAL_EVIDENCE_TOPIC,
            message.header,
        )
        for detection in message.detections:
            parts = detection.id.split(":")
            if len(parts) < 3 or parts[0] != "grounding_dino":
                continue
            request_id = parts[1]
            if not detection.results:
                continue
            hypothesis = detection.results[0].hypothesis
            point = detection.bbox.center.position
            self.localized_tables.setdefault(request_id, []).append(
                {
                    "class_id": hypothesis.class_id,
                    "confidence": float(hypothesis.score),
                    "x": float(point.x),
                    "y": float(point.y),
                    "z": float(point.z),
                    "coordinate_frame": message.header.frame_id or "map",
                    "frame_ref": frame_reference,
                }
            )

    def on_mission(self, message: String):
        """Ejecuta en otro hilo para no bloquear las entradas de sensores."""
        if self.mission_thread and self.mission_thread.is_alive():
            self.report("ya hay una misión en curso; ignoro la nueva")
            return
        self.mission_thread = threading.Thread(
            target=self.run_mission,
            args=(message.data,),
            daemon=True,
        )
        self.mission_thread.start()

    # ---------- publicación observable ----------

    def publish_mission_state(self, state: dict):
        self.mission_state_pub.publish(
            String(data=json.dumps(state, ensure_ascii=False))
        )

    def publish_model_event(self, event: dict):
        self.model_events_pub.publish(
            String(data=json.dumps(event, ensure_ascii=False))
        )

    def report(self, text: str):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    # ---------- planificación y ejecución ----------

    def run_mission(self, command: str):
        self.mission_context = {}
        self.mission_tracker.begin(command, "azure_llm")
        self.report(f"planificando: {command}")
        try:
            plan = self.plan_with_model(command)
            self.mission_tracker.set_planner("azure_llm")
        except (RemoteIntelligenceError, ValueError) as model_error:
            self.report(
                "el planificador remoto no produjo un plan utilizable; "
                f"uso el respaldo local: {model_error}"
            )
            try:
                plan = self.plan_with_rules(command)
                validate_plan(
                    plan,
                    skill_catalog=SKILL_CATALOG,
                    initial_facts=INITIAL_WORLD_FACTS,
                )
                self.mission_tracker.set_planner("rules_fallback")
            except ValueError as fallback_error:
                self.mission_tracker.stop(str(fallback_error))
                self.report(f"FALLO al planificar: {fallback_error}")
                return
        try:
            self.mission_tracker.set_plan(
                plan,
                skill_catalog=SKILL_CATALOG,
                initial_facts=INITIAL_WORLD_FACTS,
            )
        except ValueError as error:
            self.mission_tracker.stop(str(error))
            self.report(f"FALLO al validar el plan: {error}")
            return
        self.report(
            "plan: " + " → ".join(step["label"] for step in plan)
        )
        self.execute_plan(command, plan)

    def plan_with_model(self, command: str) -> list[dict]:
        """Pide una propuesta y la vuelve a validar dentro de la Jetson."""
        event_id = str(uuid.uuid4())
        started_wall = time.time()
        started_monotonic = time.monotonic()
        catalog = skill_catalog_for_model()
        local_input = {
            "command": command,
            "skill_catalog": catalog,
            "initial_facts": list(INITIAL_WORLD_FACTS),
        }
        self.publish_model_event(
            build_model_event(
                event_id=event_id,
                task="plan_mission",
                state="running",
                input_summary=(
                    "orden original, hechos conocidos y catálogo descriptivo "
                    "de skills"
                ),
                input_payload=local_input,
                created_at=started_wall,
            )
        )
        try:
            proposal = self.intelligence.plan_mission(
                command,
                catalog,
                list(INITIAL_WORLD_FACTS),
            )
        except RemoteIntelligenceError as error:
            self.publish_model_event(
                build_model_event(
                    event_id=event_id,
                    request_id=error.request_id,
                    task="plan_mission",
                    state="failed",
                    input_summary=(
                        "orden original, hechos conocidos y catálogo "
                        "descriptivo de skills"
                    ),
                    input_payload=error.input_payload or local_input,
                    raw_output=error.raw_output,
                    duration_s=round(
                        time.monotonic() - started_monotonic,
                        3,
                    ),
                    error=str(error),
                    created_at=started_wall,
                )
            )
            raise

        try:
            validated = validate_plan(
                proposal["steps"],
                skill_catalog=SKILL_CATALOG,
                initial_facts=INITIAL_WORLD_FACTS,
            )
        except ValueError as error:
            self.publish_model_event(
                build_model_event(
                    event_id=event_id,
                    request_id=proposal.get("request_id"),
                    task="plan_mission",
                    state="failed",
                    input_summary=(
                        "orden original, hechos conocidos y catálogo "
                        "descriptivo de skills"
                    ),
                    input_payload=proposal.get("model_input") or local_input,
                    raw_output=proposal.get("raw_output"),
                    duration_s=round(
                        time.monotonic() - started_monotonic,
                        3,
                    ),
                    error=f"la Jetson rechazó el plan: {error}",
                    created_at=started_wall,
                )
            )
            raise

        self.publish_model_event(
            build_model_event(
                event_id=event_id,
                request_id=proposal.get("request_id"),
                task="plan_mission",
                state="succeeded",
                input_summary=(
                    "orden original, hechos conocidos y catálogo descriptivo "
                    "de skills"
                ),
                input_payload=proposal.get("model_input") or local_input,
                model=proposal.get("model"),
                raw_output=proposal["raw_output"],
                validated_output={"steps": validated},
                duration_s=round(
                    time.monotonic() - started_monotonic,
                    3,
                ),
                created_at=started_wall,
            )
        )
        return proposal["steps"]

    @staticmethod
    def plan_with_rules(command: str) -> list[dict]:
        """Respaldo local para la misión de referencia si falla el servidor."""
        normalized = command.lower()
        if not any(
            word in normalized
            for word in ("reloj", "hora", "mesa", "objeto")
        ):
            raise ValueError("no entendí la misión")
        return build_demo_plan()

    def execute_plan(self, command: str, plan: list[dict]):
        pending = deepcopy(plan)
        world_facts = set(INITIAL_WORLD_FACTS)
        retry_counts = {}
        review_count = 0

        while pending:
            step = pending.pop(0)
            argument = self.resolve_argument(step.get("argument"))
            self.current_review_evidence = None
            self.mission_tracker.start_step(step["id"], argument)
            self.report(
                f"ejecutando: {step['label']}"
                + (f" ({argument})" if argument is not None else "")
            )
            outcome = self.execute_skill(step["skill"], argument)
            if outcome["state"] == "succeeded":
                self.mission_tracker.finish_step(
                    step["id"],
                    outcome["message"],
                    outcome.get("measurements"),
                )
                self.report(f"completado: {outcome['message']}")
                world_facts.update(self.effects_of(step))
            else:
                is_blocked = outcome["state"] == "blocked"
                self.mission_tracker.record_step_failure(
                    step["id"],
                    outcome["message"],
                    blocked=is_blocked,
                    measurements=outcome.get("measurements"),
                )
                prefix = "BLOQUEADO" if is_blocked else "FALLO"
                self.report(f"{prefix}: {outcome['message']}")

            # También se revisa el último paso: sin esta vuelta, una misión
            # terminaba justo antes de que el supervisor viera su evidencia.
            review_count += 1
            if review_count > MAX_MISSION_REVIEWS:
                error = (
                    f"la misión superó {MAX_MISSION_REVIEWS} revisiones"
                )
                self.mission_tracker.stop(error)
                self.report(f"FALLO: {error}")
                return

            try:
                review = self.review_step_with_model(
                    command=command,
                    step=step,
                    outcome=outcome,
                    pending=pending,
                    world_facts=sorted(world_facts),
                    review_count=review_count,
                )
            except (RemoteIntelligenceError, ValueError) as error:
                if outcome["state"] == "succeeded":
                    # El plan pendiente ya fue validado dos veces antes de
                    # arrancar. Una revisión remota rota no debe reemplazarlo
                    # ni detener una misión que localmente sigue siendo válida.
                    self.report(
                        "revisión remota no utilizable; conservo el plan "
                        f"validado: {error}"
                    )
                    continue
                message = (
                    f"{outcome['message']}; no hubo una revisión válida: "
                    f"{error}"
                )
                self.mission_tracker.stop(
                    message,
                    blocked=outcome["state"] == "blocked",
                )
                self.report(f"FALLO: {message}")
                return

            decision = review["decision"]
            reason = review["reason"]
            self.report(f"revisión: {decision} — {reason}")
            if decision == "complete":
                if pending or outcome["state"] != "succeeded":
                    message = (
                        "el servidor intentó completar una misión que "
                        "todavía no satisface su contrato"
                    )
                    self.mission_tracker.stop(message)
                    self.report(f"FALLO: {message}")
                    return
                self.mission_tracker.complete(reason)
                self.report(f"misión completada: {reason}")
                return
            if decision == "continue":
                continue
            if decision == "retry":
                retries = retry_counts.get(step["id"], 0)
                if retries >= MAX_SAME_STEP_RETRIES:
                    message = (
                        f"{step['label']} ya agotó su único reintento"
                    )
                    self.mission_tracker.stop(message)
                    self.report(f"FALLO: {message}")
                    return
                retry_counts[step["id"]] = retries + 1
                self.mission_tracker.retry_step(step["id"])
                pending.insert(0, step)
                continue
            if decision == "revise":
                try:
                    revised = validate_plan(
                        review["revised_steps"],
                        skill_catalog=SKILL_CATALOG,
                        initial_facts=sorted(world_facts),
                    )
                    self.mission_tracker.replace_pending_steps(
                        review["revised_steps"],
                        skill_catalog=SKILL_CATALOG,
                        current_facts=sorted(world_facts),
                    )
                except ValueError as error:
                    if outcome["state"] == "succeeded":
                        self.report(
                            "la revisión local rechazó el cambio; conservo "
                            f"el plan anterior: {error}"
                        )
                        continue
                    message = (
                        f"{outcome['message']}; revisión rechazada: {error}"
                    )
                    self.mission_tracker.stop(message)
                    self.report(f"FALLO: {message}")
                    return
                pending = [
                    {
                        "id": item["id"],
                        "skill": item["skill"],
                        "argument": item["argument"],
                        "label": item["label"],
                    }
                    for item in revised
                ]
                self.report(
                    "plan pendiente revisado: "
                    + " → ".join(item["label"] for item in pending)
                )
                continue
            if decision == "ask_human":
                question = review["question"]
                self.mission_tracker.stop(question, blocked=True)
                self.report(f"NECESITO AYUDA: {question}")
                return
            if decision == "stop":
                self.mission_tracker.stop(
                    reason,
                    blocked=outcome["state"] == "blocked",
                )
                self.report(f"MISIÓN DETENIDA: {reason}")
                return
            raise ValueError(f"decisión no implementada: {decision}")

        self.mission_tracker.complete("misión completada")
        self.report("misión completada")

    @staticmethod
    def effects_of(step: dict) -> list[str]:
        for skill in SKILL_CATALOG:
            if skill["name"] != step["skill"]:
                continue
            for variant in skill["variants"]:
                if variant.get("argument") == step.get("argument"):
                    return list(variant.get("effects", []))
        raise ValueError(
            f"no existe el contrato ejecutado: {step['skill']}"
        )

    def review_step_with_model(
        self,
        *,
        command: str,
        step: dict,
        outcome: dict,
        pending: list[dict],
        world_facts: list[str],
        review_count: int,
    ) -> dict:
        event_id = str(uuid.uuid4())
        started_wall = time.time()
        started_monotonic = time.monotonic()
        snapshot = self.mission_tracker.snapshot()
        completed = [
            {
                key: item.get(key)
                for key in (
                    "id",
                    "skill",
                    "argument",
                    "label",
                    "state",
                    "result",
                    "error",
                    "measurements",
                    "attempts",
                )
            }
            for item in snapshot["steps"]
            if item["state"] != "pending"
        ]
        catalog = skill_catalog_for_model()
        local_input = {
            "command": command,
            "skill_catalog": catalog,
            "world_facts": world_facts,
            "completed_steps": completed,
            "last_step": deepcopy(step),
            "outcome": deepcopy(outcome),
            "pending_steps": deepcopy(pending),
            "review_count": review_count,
        }
        visual_evidence = self.current_review_evidence
        model_input_ref = None
        if visual_evidence is not None:
            model_input_ref = self.publish_model_input(visual_evidence)
        summary = (
            f"resultado medido de {step['id']} y "
            f"{len(pending)} pasos pendientes"
        )
        if visual_evidence is not None:
            summary += f"; imagen: {visual_evidence['purpose']}"
        self.publish_model_event(
            build_model_event(
                event_id=event_id,
                task="review_step",
                state="running",
                input_summary=summary,
                input_ref=(
                    model_input_ref
                    if visual_evidence is not None
                    else None
                ),
                input_payload=local_input,
                created_at=started_wall,
            )
        )
        try:
            review = self.intelligence.review_step(
                **local_input,
                visual_evidence=visual_evidence,
            )
        except RemoteIntelligenceError as error:
            self.publish_model_event(
                build_model_event(
                    event_id=event_id,
                    request_id=error.request_id,
                    task="review_step",
                    state="failed",
                    input_summary=summary,
                    input_ref=(
                        model_input_ref
                        if visual_evidence is not None
                        else None
                    ),
                    input_payload=error.input_payload or local_input,
                    raw_output=error.raw_output,
                    duration_s=round(
                        time.monotonic() - started_monotonic,
                        3,
                    ),
                    error=str(error),
                    created_at=started_wall,
                )
            )
            raise

        try:
            if review["decision"] == "revise":
                validate_plan(
                    review["revised_steps"],
                    skill_catalog=SKILL_CATALOG,
                    initial_facts=world_facts,
                )
        except ValueError as error:
            self.publish_model_event(
                build_model_event(
                    event_id=event_id,
                    request_id=review.get("request_id"),
                    task="review_step",
                    state="failed",
                    input_summary=summary,
                    input_ref=(
                        model_input_ref
                        if visual_evidence is not None
                        else None
                    ),
                    input_payload=review.get("model_input") or local_input,
                    raw_output=review.get("raw_output"),
                    duration_s=round(
                        time.monotonic() - started_monotonic,
                        3,
                    ),
                    error=f"la Jetson rechazó la revisión: {error}",
                    created_at=started_wall,
                )
            )
            raise

        validated_output = {
            key: review[key]
            for key in ("decision", "reason", "revised_steps", "question")
        }
        self.publish_model_event(
            build_model_event(
                event_id=event_id,
                request_id=review.get("request_id"),
                task="review_step",
                state="succeeded",
                input_summary=summary,
                input_ref=(
                    model_input_ref
                    if visual_evidence is not None
                    else None
                ),
                input_payload=review.get("model_input") or local_input,
                model=review.get("model"),
                raw_output=review["raw_output"],
                validated_output=validated_output,
                duration_s=round(
                    time.monotonic() - started_monotonic,
                    3,
                ),
                created_at=started_wall,
            )
        )
        return review

    def resolve_argument(self, argument):
        if argument == "$selected_table":
            return self.mission_context.get("selected_table")
        return argument

    def execute_skill(self, skill: str, argument) -> dict:
        if skill == "remember_home":
            return self.remember_home()
        if skill == "navigate_to":
            return self.navigate_to(str(argument))
        if skill == "look_at":
            return self.look_at(str(argument))
        if skill == "read_clock":
            return self.read_clock()
        if skill == "choose_table":
            return self.choose_table()
        if skill == "search_table":
            return self.search_table(argument)
        if skill == "scan_for_table":
            return self.scan_for_table(argument)
        if skill == "approach_table":
            return self.approach_table(argument)
        if skill == "set_arm_pose":
            return self.set_arm_pose(str(argument))
        if skill == "align_with_table":
            return blocked(
                "falta la alineación visual fina de la base con la mesa",
                blocker={
                    "type": "missing_skill",
                    "skill": "align_with_table",
                },
            )
        if skill == "grasp_object":
            return blocked(
                "el agarre todavía no está implementado; será una policy aparte"
            )
        return failed(f"skill desconocida: {skill}")

    # ---------- skills ----------

    def remember_home(self) -> dict:
        end = time.monotonic() + 5.0
        while self.current_pose is None and time.monotonic() < end:
            time.sleep(0.05)
        if self.current_pose is None:
            return failed("no llegó la posición del robot")
        x, y, _z, yaw = self.current_pose
        self.mission_context["home"] = (x, y, yaw)
        return succeeded(f"home guardado en ({x:.2f}, {y:.2f})")

    def navigate_to(self, target: str) -> dict:
        if target == "home":
            pose = self.mission_context.get("home")
            if pose is None:
                return failed("home no fue guardado")
        else:
            pose = SEMANTIC_MAP.get(target)
            if pose is None:
                return failed(f"destino desconocido: {target}")
        return self.navigate_to_pose(pose, target)

    def navigate_to_pose(
        self,
        pose: tuple[float, float, float],
        target_label: str,
        frame_id: str = "map",
    ) -> dict:
        """Navega a una pose ya validada sin convertir un objeto en destino."""
        x, y, yaw = pose
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = frame_id
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        server_wait_s = float(
            os.environ.get("NAV_ACTION_SERVER_WAIT_S", "5")
        )
        if not self.navigation_client.wait_for_server(
            timeout_sec=server_wait_s
        ):
            safe = self.wait_for_stand(timeout_s=3.0)
            suffix = "" if safe else "; no se confirmó STAND"
            return failed(
                "el servidor local de navegación no está disponible" + suffix
            )

        request = NavigateToPose.Goal()
        request.pose = goal
        watchdog = FeedbackWatchdog(
            deadline_s=float(os.environ.get("NAV_TIMEOUT_S", "600")),
            silence_timeout_s=float(
                os.environ.get("NAV_FEEDBACK_TIMEOUT_S", "6")
            ),
        )
        watchdog.start(time.monotonic())
        measurements = {}

        def on_feedback(message):
            feedback = message.feedback
            watchdog.record_feedback(time.monotonic())
            measurements.update(
                {
                    "distance_remaining_m": round(
                        float(feedback.distance_remaining),
                        4,
                    ),
                    "navigation_time_s": round(
                        self.duration_seconds(feedback.navigation_time),
                        3,
                    ),
                }
            )
            if hasattr(feedback, "position_tracking_error"):
                measurements["position_tracking_error_m"] = round(
                    float(feedback.position_tracking_error),
                    4,
                )
            if hasattr(feedback, "heading_tracking_error"):
                measurements["heading_tracking_error_rad"] = round(
                    float(feedback.heading_tracking_error),
                    4,
                )

        try:
            send_future = self.navigation_client.send_goal_async(
                request,
                feedback_callback=on_feedback,
            )
            if not self.wait_for_future(send_future, timeout_s=server_wait_s):
                safe = self.wait_for_stand(timeout_s=3.0)
                suffix = "" if safe else "; no se confirmó STAND"
                return failed(
                    "la navegación no confirmó si aceptó el objetivo" + suffix
                )
            goal_handle = send_future.result()
        except Exception as error:  # noqa: BLE001
            safe = self.wait_for_stand(timeout_s=3.0)
            suffix = "" if safe else "; no se confirmó STAND"
            return failed(
                f"falló el envío del objetivo: {error}{suffix}",
                measurements,
            )

        if goal_handle is None or not goal_handle.accepted:
            safe = self.wait_for_stand(timeout_s=3.0)
            suffix = "" if safe else "; no se confirmó STAND"
            return failed(
                "el navegador rechazó el objetivo" + suffix,
                measurements,
            )

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            decision = watchdog.check(time.monotonic())
            if decision is not None:
                cancel_confirmed = self.cancel_navigation(goal_handle)
                safe = self.wait_for_stand(timeout_s=3.0)
                details = [decision.reason]
                if not cancel_confirmed:
                    details.append("el servidor no confirmó la cancelación")
                if not safe:
                    details.append("no se confirmó STAND")
                return failed("; ".join(details), measurements)
            time.sleep(0.05)

        try:
            wrapped = result_future.result()
        except Exception as error:  # noqa: BLE001
            safe = self.wait_for_stand(timeout_s=3.0)
            suffix = "" if safe else "; no se confirmó STAND"
            return failed(
                f"el navegador perdió el resultado: {error}{suffix}",
                measurements,
            )

        safe = self.wait_for_stand(timeout_s=3.0)
        if not safe:
            return failed(
                "la navegación terminó pero no devolvió la movilidad a STAND",
                measurements,
            )
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            return succeeded(f"llegó a {target_label}", measurements)

        error_message = str(
            getattr(wrapped.result, "error_msg", "")
        ).strip()
        if not error_message:
            if wrapped.status == GoalStatus.STATUS_CANCELED:
                error_message = "la navegación fue cancelada"
            elif wrapped.status == GoalStatus.STATUS_ABORTED:
                error_message = "la navegación fue abortada"
            else:
                error_message = (
                    f"la navegación terminó con estado {wrapped.status}"
                )
        return failed(error_message, measurements)

    def look_at(self, target: str) -> dict:
        detection_name = DETECTION_NAMES.get(target)
        if detection_name is None:
            return failed(f"objeto visual desconocido: {target}")
        previous = self.detections.get(detection_name, {}).get("frame_ref")
        detection = self.wait_for_detection(
            detection_name,
            timeout_s=12.0,
            after_reference=previous,
        )
        if detection is None:
            return failed(f"la cámara no confirmó {detection_name}")
        reference = detection.get("frame_ref")
        if reference is None or not self.set_review_evidence(
            f"cuadro exacto que confirmó {detection_name}",
            reference,
            detail="high" if target == "clock" else "low",
        ):
            return failed(
                f"se detectó {detection_name}, pero no se conservó su cuadro"
            )
        return succeeded(
            f"confirmó {detection_name} con confianza "
            f"{detection.get('confidence', '?')}",
            {
                key: detection[key]
                for key in ("confidence", "cx", "area", "source")
                if key in detection
            },
        )

    def read_clock(self) -> dict:
        if (
            self.clock_crop is None
            or self.clock_crop_received_at is None
            or time.monotonic() - self.clock_crop_received_at > 10.0
        ):
            return failed("no hay un recorte reciente del reloj")

        event_id = str(uuid.uuid4())
        started_wall = time.time()
        started_monotonic = time.monotonic()
        self.current_review_evidence = {
            "purpose": "recorte exacto usado para leer el reloj",
            "image": self.clock_crop,
            "input_ref": self.clock_crop_ref,
            "detail": "high",
        }
        model_input_ref = self.publish_model_input(
            self.current_review_evidence
        )
        self.publish_model_event(
            build_model_event(
                event_id=event_id,
                task="read_clock",
                state="running",
                input_summary="recorte JPEG exacto del reloj detectado",
                input_ref=model_input_ref,
                created_at=started_wall,
            )
        )
        self.report(
            "reloj detectado: envío sólo su recorte al modelo visual remoto"
        )
        try:
            reading = self.intelligence.read_clock(self.clock_crop)
        except RemoteIntelligenceError as error:
            self.publish_model_event(
                build_model_event(
                    event_id=event_id,
                    request_id=error.request_id,
                    task="read_clock",
                    state="failed",
                    input_summary="recorte JPEG exacto del reloj detectado",
                    input_ref=model_input_ref,
                    raw_output=error.raw_output,
                    duration_s=round(
                        time.monotonic() - started_monotonic,
                        3,
                    ),
                    error=str(error),
                    created_at=started_wall,
                )
            )
            return failed(f"servidor de visión no disponible: {error}")

        validated = {
            key: reading[key]
            for key in ("readable", "hour", "minute", "text")
        }
        self.publish_model_event(
            build_model_event(
                event_id=event_id,
                request_id=reading.get("request_id"),
                task="read_clock",
                state="succeeded",
                input_summary="recorte JPEG exacto del reloj detectado",
                input_ref=model_input_ref,
                model=reading.get("model"),
                raw_output=reading["raw_output"],
                validated_output=validated,
                duration_s=round(
                    time.monotonic() - started_monotonic,
                    3,
                ),
                created_at=started_wall,
            )
        )
        if not reading["readable"]:
            return failed("el modelo informa que el reloj no es legible")
        self.mission_context["clock_reading"] = validated
        return succeeded(f"el modelo leyó {reading['text']}")

    def choose_table(self) -> dict:
        reading = self.mission_context.get("clock_reading")
        if reading is None:
            return failed("no existe una lectura validada del reloj")
        before_noon = (reading["hour"], reading["minute"]) < (12, 0)
        selected_table = "red_table" if before_noon else "blue_table"
        self.mission_context["selected_table"] = selected_table
        table_label = (
            "mesa A roja" if selected_table == "red_table" else "mesa B azul"
        )
        relation = "antes de" if before_noon else "a las 12:00 o después de"
        self.mission_tracker.set_decision(
            evidence=f"el reloj marca {reading['text']}",
            rule=f"{relation} las 12:00",
            outcome=f"buscar {table_label}",
        )
        return succeeded(f"eligió {table_label}")

    def search_table(self, target: str) -> dict:
        outcome = self.search_table_in_current_view(target)
        if (
            outcome["state"] == "failed"
            and outcome.get("failure_kind") == "not_visible"
        ):
            return blocked(
                "la mesa no apareció en la vista actual; hace falta barrer "
                "visualmente la habitación",
                outcome.get("measurements"),
                blocker={
                    "type": "missing_skill",
                    "skill": "scan_for_table",
                },
            )
        return outcome

    def search_table_in_current_view(self, target: str) -> dict:
        if target not in ("red_table", "blue_table"):
            return failed("no hay una mesa elegida")
        request = make_search_request(target)
        request_id = request["request_id"]
        self.search_status = None
        self.localized_tables.pop(request_id, None)
        self.search_pub.publish(
            String(data=json.dumps(request, ensure_ascii=False))
        )
        end = time.monotonic() + 60.0
        while time.monotonic() < end:
            time.sleep(0.1)
            status = self.search_status or {}
            if status.get("request_id") != request_id:
                continue
            if status.get("state") in ("failed", "rejected"):
                reference = status.get("frame_ref")
                if reference is not None:
                    self.set_review_evidence(
                        f"cuadro usado en la búsqueda fallida de {target}",
                        reference,
                    )
                return failed(
                    "la búsqueda visual falló: "
                    + str(status.get("error", status))
                )
            if status.get("state") == "complete":
                break
        else:
            return failed("la búsqueda visual no respondió a tiempo")

        localization_end = time.monotonic() + 6.0
        while time.monotonic() < localization_end:
            candidates = [
                candidate
                for candidate in self.localized_tables.get(request_id, [])
                if candidate["class_id"] == target
            ]
            if candidates:
                best = max(
                    candidates,
                    key=lambda candidate: candidate["confidence"],
                )
                self.mission_context["table_point"] = best
                if not self.set_review_evidence(
                    f"cuadro exacto que ubicó {target}",
                    best["frame_ref"],
                ):
                    return failed(
                        "la mesa fue ubicada, pero no se conservó el cuadro "
                        "exacto que originó la medición"
                    )
                return succeeded(
                    f"ubicó {target} en ({best['x']:.2f}, {best['y']:.2f})",
                    {
                        "confidence": round(best["confidence"], 3),
                        "x_m": round(best["x"], 3),
                        "y_m": round(best["y"], 3),
                        "z_m": round(best["z"], 3),
                    },
                )
            time.sleep(0.1)
        reference = (self.search_status or {}).get("frame_ref")
        if reference is not None:
            self.set_review_evidence(
                f"cuadro donde no se encontró {target}",
                reference,
            )
        outcome = failed(
            "la mesa no apareció en esta vista",
            {"detection_count": 0},
        )
        outcome["failure_kind"] = "not_visible"
        return outcome

    def scan_for_table(self, target: str) -> dict:
        """Barre la sala con el detector local y confirma sólo candidatos."""
        if target not in ("red_table", "blue_table"):
            return failed("no hay una mesa elegida")

        scan_attempt = int(
            self.mission_context.get("active_scan_attempts", 0)
        ) + 1
        self.mission_context["active_scan_attempts"] = scan_attempt
        if scan_attempt > MAX_ACTIVE_SCANS_PER_MISSION:
            return blocked(
                "ya se hicieron dos barridos completos sin evidencia nueva; "
                "hace falta cambiar el punto de observación o pedir ayuda",
                {"scan_attempt": scan_attempt},
                blocker={
                    "type": "search_exhausted",
                    "target": target,
                },
            )

        pattern = make_scan_pattern(
            SEARCH_HORIZONTAL_FOV_DEG,
            SEARCH_MINIMUM_OVERLAP_DEG,
        )
        if self.current_pose is None:
            return failed("no llegó la posición inicial del barrido")
        start_x, start_y, _start_z, start_yaw = self.current_pose
        views_checked = 0
        local_candidates = 0
        remote_checks = 0
        last_local_status = None
        for view_index in range(pattern.view_count):
            with self.local_detection_condition:
                generation = self.local_detection_generation
            status, updates = self.wait_for_local_detection_updates(
                generation,
                minimum_updates=LOCAL_UPDATES_PER_VIEW,
                timeout_s=LOCAL_VIEW_TIMEOUT_S,
            )
            if updates < LOCAL_UPDATES_PER_VIEW:
                return failed(
                    "el detector local no produjo suficientes cuadros nuevos",
                    {
                        "views_checked": views_checked,
                        "local_updates": updates,
                    },
                )

            last_local_status = status
            views_checked += 1
            candidate = local_table_candidate(
                status,
                target,
                minimum_color_pixels=COLOR_SCOUT_MIN_PIXELS,
            )
            if candidate is not None:
                local_candidates += 1
                remote_checks += 1
                observation = self.search_table_in_current_view(target)
                if observation["state"] == "succeeded":
                    measurements = dict(
                        observation.get("measurements", {})
                    )
                    current_pose = self.current_pose
                    if current_pose is None:
                        return failed(
                            "se perdió la posición al confirmar la mesa"
                        )
                    measurements.update(
                        {
                            "scan_attempt": scan_attempt,
                            "views_checked": views_checked,
                            "maximum_views": pattern.view_count,
                            "turn_increment_deg": round(
                                math.degrees(pattern.turn_increment_rad),
                                2,
                            ),
                            "actual_overlap_deg": round(
                                pattern.actual_overlap_deg,
                                2,
                            ),
                            "local_candidates": local_candidates,
                            "remote_checks": remote_checks,
                            "scan_position_drift_m": round(
                                math.hypot(
                                    current_pose[0] - start_x,
                                    current_pose[1] - start_y,
                                ),
                                3,
                            ),
                        }
                    )
                    return succeeded(
                        observation["message"]
                        + f" después de revisar {views_checked} vista(s)",
                        measurements,
                    )
                if observation.get("failure_kind") != "not_visible":
                    return failed(
                        "el barrido no pudo confirmar un candidato: "
                        + observation["message"],
                        {
                            "views_checked": views_checked,
                            "local_candidates": local_candidates,
                            "remote_checks": remote_checks,
                            **observation.get("measurements", {}),
                        },
                    )

            if view_index < pattern.view_count - 1:
                turn = self.spin_relative(pattern.turn_increment_rad)
                if turn["state"] != "succeeded":
                    return failed(
                        "el barrido se interrumpió antes de cubrir la sala: "
                        + turn["message"],
                        {
                            "views_checked": views_checked,
                            **turn.get("measurements", {}),
                        },
                    )

        # Cada giro termina dentro de una tolerancia de cinco grados. Repetir
        # simplemente el incremento acumularía ese error y no cerraría 360°.
        # La corrección final se calcula contra la orientación realmente medida.
        if self.current_pose is None:
            return failed("se perdió la posición al cerrar el barrido")
        remaining_turn = normalize_angle(start_yaw - self.current_pose[3])
        if abs(remaining_turn) > math.radians(1.0):
            restore = self.spin_relative(remaining_turn)
            if restore["state"] != "succeeded":
                return failed(
                    "no se encontró la mesa y falló el regreso a la "
                    "orientación inicial: " + restore["message"],
                    {
                        "views_checked": views_checked,
                        **restore.get("measurements", {}),
                    },
                )
        if self.current_pose is None:
            return failed("no llegó la medición posterior al barrido")
        final_x, final_y, _final_z, final_yaw = self.current_pose
        orientation_error_deg = math.degrees(
            abs(normalize_angle(start_yaw - final_yaw))
        )
        position_drift_m = math.hypot(final_x - start_x, final_y - start_y)
        if orientation_error_deg > 10.0:
            return failed(
                "el barrido terminó seguro pero no recuperó su orientación",
                {
                    "views_checked": views_checked,
                    "orientation_error_deg": round(
                        orientation_error_deg,
                        2,
                    ),
                    "position_drift_m": round(position_drift_m, 3),
                },
            )
        if last_local_status is not None:
            reference = last_local_status.get("frame_ref")
            if reference is not None:
                self.set_review_evidence(
                    f"último cuadro del barrido sin {target}",
                    reference,
                )

        return blocked(
            "no se encontró la mesa elegida después de cubrir 360 grados",
            {
                "scan_attempt": scan_attempt,
                "views_checked": views_checked,
                "maximum_views": pattern.view_count,
                "turn_increment_deg": round(
                    math.degrees(pattern.turn_increment_rad),
                    2,
                ),
                "actual_overlap_deg": round(
                    pattern.actual_overlap_deg,
                    2,
                ),
                "local_candidates": local_candidates,
                "remote_checks": remote_checks,
                "returned_to_start_orientation": True,
                "orientation_error_deg": round(
                    orientation_error_deg,
                    2,
                ),
                "position_drift_m": round(position_drift_m, 3),
            },
            blocker={
                "type": "unresolved_perception",
                "target": target,
            },
        )

    def approach_table(self, target: str) -> dict:
        """Llega a una preaproximación y vuelve a medir antes de manipular."""
        try:
            approach_attempt, allowed = next_table_approach_attempt(
                int(self.mission_context.get("table_approach_attempts", 0)),
                MAX_TABLE_APPROACH_ATTEMPTS_PER_MISSION,
            )
        except (TypeError, ValueError) as error:
            return failed(f"el contador de aproximaciones es inválido: {error}")
        self.mission_context["table_approach_attempts"] = approach_attempt
        if not allowed:
            return blocked(
                "ya se hicieron dos preaproximaciones; hace falta cambiar la "
                "estrategia o pedir ayuda antes de volver a mover la base",
                {"approach_attempt": approach_attempt},
                blocker={
                    "type": "approach_exhausted",
                    "target": target,
                },
            )

        table_point = self.mission_context.get("table_point")
        if (
            target not in ("red_table", "blue_table")
            or not isinstance(table_point, dict)
            or table_point.get("class_id") != target
        ):
            return failed("no existe una medición de la mesa elegida")
        if table_point.get("coordinate_frame", "map") != "map":
            return failed("la mesa medida no está expresada en el mapa")
        if self.current_pose is None:
            return failed("no llegó la posición para calcular la aproximación")

        robot_x, robot_y, _robot_z, _robot_yaw = self.current_pose
        try:
            staging = compute_table_staging_pose(
                robot_x=robot_x,
                robot_y=robot_y,
                table_x=table_point["x"],
                table_y=table_point["y"],
                standoff_m=TABLE_STAGING_STANDOFF_M,
                world_bounds=WORLD_BOUNDS,
            )
        except (KeyError, TypeError, ValueError) as error:
            return failed(f"no se pudo calcular una aproximación segura: {error}")

        self.mission_context["table_staging_pose"] = (
            staging.x,
            staging.y,
            staging.yaw,
        )
        navigation = self.navigate_to_pose(
            self.mission_context["table_staging_pose"],
            f"preaproximación de {target}",
            frame_id="map",
        )
        if navigation["state"] != "succeeded":
            return failed(
                "falló la preaproximación: " + navigation["message"],
                navigation.get("measurements"),
            )

        observation = self.search_table_in_current_view(target)
        if observation["state"] != "succeeded":
            return failed(
                "llegó a la preaproximación, pero no volvió a confirmar la "
                "mesa: " + observation["message"],
                {
                    **navigation.get("measurements", {}),
                    **observation.get("measurements", {}),
                },
            )
        refreshed = self.mission_context.get("table_point")
        if self.current_pose is None or not isinstance(refreshed, dict):
            return failed("faltan mediciones posteriores a la aproximación")

        current_x, current_y, current_z, current_yaw = self.current_pose
        surface_distance_m = math.hypot(
            refreshed["x"] - current_x,
            refreshed["y"] - current_y,
        )
        desired_yaw = math.atan2(
            refreshed["y"] - current_y,
            refreshed["x"] - current_x,
        )
        yaw_error_deg = math.degrees(
            abs(normalize_angle(desired_yaw - current_yaw))
        )
        measurements = {
            "approach_attempt": approach_attempt,
            "staging_x_m": round(staging.x, 3),
            "staging_y_m": round(staging.y, 3),
            "staging_yaw_deg": round(math.degrees(staging.yaw), 2),
            "requested_standoff_m": TABLE_STAGING_STANDOFF_M,
            "initial_surface_distance_m": round(
                staging.initial_surface_distance_m,
                3,
            ),
            "navigation_error_m": navigation.get(
                "measurements",
                {},
            ).get("distance_remaining_m"),
            "confirmed_surface_distance_m": round(surface_distance_m, 3),
            "confirmed_yaw_error_deg": round(yaw_error_deg, 2),
            "body_height_m": round(current_z, 3),
            "confidence": observation.get("measurements", {}).get(
                "confidence"
            ),
            "confirmed_x_m": observation.get("measurements", {}).get("x_m"),
            "confirmed_y_m": observation.get("measurements", {}).get("y_m"),
        }
        if current_z < MIN_SAFE_BODY_HEIGHT_M:
            return failed(
                "el robot llegó pero no conservó una altura segura",
                measurements,
            )
        if not (
            TABLE_STAGING_MIN_SURFACE_DISTANCE_M
            <= surface_distance_m
            <= TABLE_STAGING_MAX_SURFACE_DISTANCE_M
        ):
            return failed(
                "la mesa quedó fuera de la separación de preaproximación",
                measurements,
            )
        if yaw_error_deg > TABLE_STAGING_MAX_YAW_ERROR_DEG:
            return failed(
                "la base no terminó mirando suficientemente hacia la mesa",
                measurements,
            )
        return succeeded(
            f"quedó en preaproximación de {target} y volvió a confirmarla",
            measurements,
        )

    def wait_for_local_detection_updates(
        self,
        after_generation: int,
        minimum_updates: int,
        timeout_s: float,
    ) -> tuple[dict, int]:
        """Espera resultados adquiridos después de que terminó cada giro."""
        deadline = time.monotonic() + timeout_s
        with self.local_detection_condition:
            while (
                self.local_detection_generation - after_generation
                < minimum_updates
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self.local_detection_condition.wait(
                    timeout=min(0.25, remaining)
                )
            updates = self.local_detection_generation - after_generation
            status = deepcopy(self.local_detection_status or {})
        return status, updates

    def spin_relative(self, angle_rad: float) -> dict:
        """Usa la Action estándar de Nav2; nunca publica velocidad directa."""
        server_wait_s = float(
            os.environ.get("NAV_ACTION_SERVER_WAIT_S", "5")
        )
        if not self.spin_client.wait_for_server(timeout_sec=server_wait_s):
            safe = self.wait_for_stand(timeout_s=3.0)
            suffix = "" if safe else "; no se confirmó STAND"
            return failed(
                "el servidor local de giro no está disponible" + suffix
            )

        request = Spin.Goal()
        request.target_yaw = float(angle_rad)
        allowance_s = float(os.environ.get("SPIN_TIMEOUT_S", "120"))
        request.time_allowance.sec = int(allowance_s)
        request.time_allowance.nanosec = int(
            (allowance_s - int(allowance_s)) * 1_000_000_000
        )
        measurements = {
            "requested_turn_deg": round(math.degrees(angle_rad), 2),
        }
        watchdog = FeedbackWatchdog(
            deadline_s=allowance_s + 5.0,
            silence_timeout_s=float(
                os.environ.get("NAV_FEEDBACK_TIMEOUT_S", "6")
            ),
        )
        watchdog.start(time.monotonic())

        def on_feedback(message):
            watchdog.record_feedback(time.monotonic())
            measurements["angular_distance_traveled_deg"] = round(
                math.degrees(
                    float(message.feedback.angular_distance_traveled)
                ),
                2,
            )

        try:
            send_future = self.spin_client.send_goal_async(
                request,
                feedback_callback=on_feedback,
            )
            if not self.wait_for_future(send_future, timeout_s=server_wait_s):
                safe = self.wait_for_stand(timeout_s=3.0)
                suffix = "" if safe else "; no se confirmó STAND"
                return failed(
                    "el giro no confirmó si aceptó el objetivo" + suffix,
                    measurements,
                )
            goal_handle = send_future.result()
        except Exception as error:  # noqa: BLE001
            safe = self.wait_for_stand(timeout_s=3.0)
            suffix = "" if safe else "; no se confirmó STAND"
            return failed(
                f"falló el envío del giro: {error}{suffix}",
                measurements,
            )

        if goal_handle is None or not goal_handle.accepted:
            safe = self.wait_for_stand(timeout_s=3.0)
            suffix = "" if safe else "; no se confirmó STAND"
            return failed("el navegador rechazó el giro" + suffix)

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            decision = watchdog.check(time.monotonic())
            if decision is not None:
                cancel_confirmed = self.cancel_navigation(goal_handle)
                safe = self.wait_for_stand(timeout_s=3.0)
                details = [decision.reason]
                if not cancel_confirmed:
                    details.append("el servidor no confirmó la cancelación")
                if not safe:
                    details.append("no se confirmó STAND")
                return failed("; ".join(details), measurements)
            time.sleep(0.05)

        try:
            wrapped = result_future.result()
        except Exception as error:  # noqa: BLE001
            safe = self.wait_for_stand(timeout_s=3.0)
            suffix = "" if safe else "; no se confirmó STAND"
            return failed(
                f"el navegador perdió el resultado del giro: {error}{suffix}",
                measurements,
            )

        safe = self.wait_for_stand(timeout_s=3.0)
        if not safe:
            return failed(
                "el giro terminó pero no devolvió la movilidad a STAND",
                measurements,
            )
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            return succeeded("giro completado", measurements)
        error_message = str(
            getattr(wrapped.result, "error_msg", "")
        ).strip()
        if not error_message:
            error_message = f"el giro terminó con estado {wrapped.status}"
        return failed(error_message, measurements)

    def set_arm_pose(self, target: str) -> dict:
        ros_pose = ARM_POSES.get(target)
        if ros_pose is None:
            return failed(f"postura de brazos desconocida: {target}")
        end = time.monotonic() + 40.0
        last_send = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now - last_send >= 0.5:
                self.arms_pub.publish(String(data=ros_pose))
                last_send = now
            status = self.arm_status or {}
            if status.get("pose") == ros_pose and status.get("reached"):
                return succeeded(
                    f"brazos llegaron a {ros_pose}; error máximo "
                    f"{status.get('max_error_rad', '?')} rad"
                )
            time.sleep(0.1)
        return failed(f"los brazos no confirmaron la postura {ros_pose}")

    # ---------- esperas ----------

    def set_review_evidence(
        self,
        purpose: str,
        reference: dict,
        detail: str = "low",
        timeout_s: float = 1.5,
    ) -> bool:
        """Enlaza la revisión sólo si el cuadro exacto sigue en memoria."""
        try:
            key = image_ref_key(reference)
        except ValueError:
            return False
        deadline = time.monotonic() + timeout_s
        entry = None
        while time.monotonic() < deadline:
            with self.visual_evidence_lock:
                entry = self.visual_evidence.get(key)
            if entry is not None:
                break
            time.sleep(0.05)
        if entry is None:
            return False
        age_s = time.monotonic() - entry["received_at"]
        if age_s < 0 or age_s > VISUAL_EVIDENCE_MAX_AGE_S:
            return False
        self.current_review_evidence = {
            "purpose": purpose,
            "image": entry["image"],
            "input_ref": entry["input_ref"],
            "detail": detail,
        }
        return True

    def publish_model_input(self, evidence: dict) -> dict:
        """Publica sólo el JPEG que realmente cruza hacia un modelo."""
        source_ref = evidence["input_ref"]
        message = CompressedImage()
        message.header.stamp.sec = int(source_ref["sec"])
        message.header.stamp.nanosec = int(source_ref["nanosec"])
        message.format = "jpeg"
        message.data = evidence["image"]
        self.model_input_pub.publish(message)
        return {
            "topic": MODEL_INPUT_TOPIC,
            "source_topic": source_ref["topic"],
            "sec": int(source_ref["sec"]),
            "nanosec": int(source_ref["nanosec"]),
            "purpose": evidence["purpose"],
            "detail": evidence["detail"],
            "bytes": len(evidence["image"]),
        }

    @staticmethod
    def duration_seconds(duration) -> float:
        return float(duration.sec) + float(duration.nanosec) / 1_000_000_000.0

    @staticmethod
    def wait_for_future(future, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.done()

    def cancel_navigation(self, goal_handle) -> bool:
        """Pide cancelar; la seguridad final se confirma aparte con STAND."""
        try:
            future = goal_handle.cancel_goal_async()
        except Exception:  # noqa: BLE001
            return False
        if not self.wait_for_future(future, timeout_s=2.0):
            return False
        try:
            response = future.result()
        except Exception:  # noqa: BLE001
            return False
        return bool(response and response.goals_canceling)

    def wait_for_stand(self, timeout_s: float) -> bool:
        """Confirma que ninguna autonomía conserva el movimiento del cuerpo."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.mobility_owner == "stand":
                return True
            time.sleep(0.05)
        return self.mobility_owner == "stand"

    def wait_for_detection(
        self,
        target: str,
        timeout_s: float,
        after_reference: dict = None,
    ):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            time.sleep(0.2)
            detection = self.detections.get(target)
            if detection is None:
                continue
            if after_reference is None:
                return detection
            reference = detection.get("frame_ref")
            if reference != after_reference:
                return detection
        return None


def main():
    rclpy.init()
    node = Agent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
