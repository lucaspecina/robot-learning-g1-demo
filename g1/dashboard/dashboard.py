#!/usr/bin/env python3
"""Tablero de operación: muestra hechos del robot sin intervenir en ellos.

La interfaz separa:

* video vivo y estado físico;
* misión y subtareas estructuradas;
* cuadro exacto analizado por percepción;
* entrada y salida literal de cada LLM/VLM;
* relato humano y detalles técnicos.

El tablero no está en ningún lazo de control. Puede caerse o perder la red sin
alterar el equilibrio, la navegación ni la misión local.
"""
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
from pathlib import Path
import sys
import threading
import time

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from scene_layout import DASHBOARD_SCENE  # noqa: E402
from visual_evidence import (  # noqa: E402
    MODEL_INPUT_TOPIC,
    image_ref,
    image_ref_key,
    is_complete_jpeg,
)

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray


PORT = 8080
HISTORY_MAX = 180
MODEL_EVENT_MAX = 20
MODEL_INPUT_MAX = 30
FALLEN_HEIGHT = 0.45
OFFLINE_AFTER_S = 3.0
OPEN_RESULT_HOLD_S = 60.0
ANALYSIS_OFFLINE_AFTER_S = OPEN_RESULT_HOLD_S + 5.0

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

state = {
    "camera_jpeg": None,
    "camera_time": 0.0,
    "analysis_jpeg": None,
    "analysis_time": 0.0,
    "analysis_labels": [],
    "analysis_source": "-",
    "analysis_hold_until": 0.0,
    "model_input_jpeg": None,
    "model_input_time": 0.0,
    "model_input_event_id": None,
    "odom_time": 0.0,
    "frames": 0,
    "detections": {},
    "perception": {
        "backend": "-",
        "latency_ms": None,
        "processed_frames": 0,
        "dropped_frames": 0,
    },
    "search": {"state": "ready"},
    "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
    "real_speed": 0.0,
    "fallen": False,
    "cmd": {"vx": 0.0, "vy": 0.0, "vyaw": 0.0},
    "mobility": {
        "owner": "-",
        "requester": "-",
        "transition_reason": "-",
        "rejected_commands": 0,
    },
    "goal": None,
    "nav": "-",
    "arms": "reposo",
    "mission_state": {
        "schema_version": 1,
        "mission_id": None,
        "command": None,
        "planner": None,
        "state": "idle",
        "active_step_id": None,
        "steps": [],
        "decision": None,
        "error": None,
    },
    "mission_events": [],
    "model_events": [],
}
lock = threading.Lock()


def to_jpeg(image: np.ndarray, text: str) -> bytes:
    """Agrega una prueba de vida visible dentro del cuadro comprimido."""
    from PIL import Image as PILImage, ImageDraw

    output = PILImage.fromarray(image)
    draw = ImageDraw.Draw(output)
    draw.rectangle([0, 0, output.width, 14], fill=(0, 0, 0))
    draw.text((4, 2), text, fill=(0, 255, 140))
    buffer = io.BytesIO()
    output.save(buffer, format="JPEG", quality=70)
    return buffer.getvalue()


def to_analysis_jpeg(
    image: np.ndarray,
    detections,
) -> tuple[bytes, list[str]]:
    """Dibuja cajas sobre el mismo cuadro que produjo las detecciones."""
    from PIL import Image as PILImage, ImageDraw

    output = PILImage.fromarray(image)
    draw = ImageDraw.Draw(output)
    labels = []
    for detection in detections:
        if not detection.results:
            continue
        best = max(
            detection.results,
            key=lambda result: result.hypothesis.score,
        ).hypothesis
        center = detection.bbox.center.position
        x1 = center.x - detection.bbox.size_x / 2
        y1 = center.y - detection.bbox.size_y / 2
        x2 = center.x + detection.bbox.size_x / 2
        y2 = center.y + detection.bbox.size_y / 2
        label = f"{best.class_id} {best.score:.2f}"
        labels.append(label)
        draw.rectangle((x1, y1, x2, y2), outline=(25, 235, 125), width=2)
        text_box = draw.textbbox((x1 + 2, y1 + 2), label)
        draw.rectangle(text_box, fill=(0, 0, 0))
        draw.text((x1 + 2, y1 + 2), label, fill=(25, 235, 125))
    if not labels:
        draw.rectangle((0, 0, 112, 15), fill=(0, 0, 0))
        draw.text((4, 2), "sin detecciones", fill=(170, 175, 185))
    buffer = io.BytesIO()
    output.save(buffer, format="JPEG", quality=82)
    return buffer.getvalue(), labels


class DashboardNode(Node):
    def __init__(self):
        super().__init__("dashboard")
        self.image_cache = OrderedDict()
        self.model_input_cache = OrderedDict()
        self.create_subscription(
            Image,
            "/g1/head_cam/image",
            self.on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection2DArray,
            "/g1/object_detections",
            self.on_rtdetr_detections,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection2DArray,
            "/g1/open_vocabulary_detections",
            self.on_open_vocabulary_detections,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CompressedImage,
            MODEL_INPUT_TOPIC,
            self.on_model_input,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/g1/detections",
            self.on_detections,
            10,
        )
        self.create_subscription(
            String,
            "/g1/perception/status",
            self.on_perception,
            10,
        )
        self.create_subscription(
            String,
            "/g1/perception/search_status",
            self.on_search_status,
            10,
        )
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(
            String,
            "/g1/mobility/status",
            self.on_mobility,
            10,
        )
        self.create_subscription(
            PoseStamped,
            "/g1/navigation/goal",
            self.on_goal,
            10,
        )
        self.create_subscription(String, "/g1/nav_status", self.on_nav, 10)
        self.create_subscription(
            String,
            "/g1/mission_status",
            self.on_mission_event,
            10,
        )
        self.create_subscription(
            String,
            "/g1/mission_state",
            self.on_mission_state,
            STATE_QOS,
        )
        self.create_subscription(
            String,
            "/g1/model_events",
            self.on_model_event,
            MODEL_EVENT_QOS,
        )
        self.create_subscription(String, "/g1/arm_pose", self.on_arms, 10)
        self.get_logger().info(
            f"tablero escuchando; sirve en el puerto {PORT}"
        )

    def on_image(self, message: Image):
        if message.encoding != "rgb8":
            return
        image = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height,
            message.width,
            3,
        ).copy()
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        self.image_cache[key] = image
        while len(self.image_cache) > HISTORY_MAX:
            self.image_cache.popitem(last=False)
        if time.time() - state["camera_time"] < 0.15:
            return
        frame_number = state["frames"] + 1
        text = (
            f"cuadro {frame_number}  {time.strftime('%H:%M:%S')}  "
            "(si avanza, es video)"
        )
        try:
            jpeg = to_jpeg(image, text)
        except Exception:
            return
        with lock:
            state["camera_jpeg"] = jpeg
            state["camera_time"] = time.time()
            state["frames"] = frame_number

    def on_model_input(self, message: CompressedImage):
        data = bytes(message.data)
        if not is_complete_jpeg(data):
            return
        reference = image_ref(MODEL_INPUT_TOPIC, message.header)
        key = image_ref_key(reference)
        self.model_input_cache[key] = data
        while len(self.model_input_cache) > MODEL_INPUT_MAX:
            self.model_input_cache.popitem(last=False)
        self.bind_latest_model_input()

    def bind_latest_model_input(self):
        with lock:
            if not state["model_events"]:
                return
            event = state["model_events"][-1]
            input_ref = event.get("input_ref") or {}
            if not input_ref:
                # Un planificador de texto no debe heredar la imagen de una
                # llamada visual anterior y presentarla como si fuera suya.
                state["model_input_jpeg"] = None
                state["model_input_time"] = 0.0
                state["model_input_event_id"] = None
                return
            try:
                key = image_ref_key(input_ref)
            except ValueError:
                state["model_input_jpeg"] = None
                state["model_input_time"] = 0.0
                state["model_input_event_id"] = None
                return
            jpeg = self.model_input_cache.get(key)
            if jpeg is None:
                state["model_input_jpeg"] = None
                state["model_input_time"] = 0.0
                state["model_input_event_id"] = None
                return
            state["model_input_jpeg"] = jpeg
            state["model_input_time"] = time.time()
            state["model_input_event_id"] = event.get("event_id")

    def on_rtdetr_detections(self, message: Detection2DArray):
        self.on_object_detections(message, "RT-DETR")

    def on_open_vocabulary_detections(self, message: Detection2DArray):
        self.on_object_detections(message, "Grounding DINO")

    def on_object_detections(
        self,
        message: Detection2DArray,
        source: str,
    ):
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        image = self.image_cache.get(key)
        if image is None:
            return
        now = time.time()
        with lock:
            hold_result = state["analysis_hold_until"] > now
        if source != "Grounding DINO" and hold_result:
            return
        try:
            jpeg, labels = to_analysis_jpeg(image, message.detections)
        except Exception:
            return
        with lock:
            state["analysis_jpeg"] = jpeg
            state["analysis_time"] = now
            state["analysis_labels"] = labels
            state["analysis_source"] = source
            state["analysis_hold_until"] = (
                now + OPEN_RESULT_HOLD_S
                if source == "Grounding DINO"
                else 0.0
            )

    def on_detections(self, message: String):
        self.update_json("detections", message.data)

    def on_perception(self, message: String):
        self.update_json("perception", message.data)

    def on_search_status(self, message: String):
        self.update_json("search", message.data)

    @staticmethod
    def update_json(name: str, data: str):
        try:
            value = json.loads(data)
        except json.JSONDecodeError:
            return
        with lock:
            state[name] = value

    def on_odom(self, message: Odometry):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        velocity = message.twist.twist.linear
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
        with lock:
            state["pose"] = {
                "x": round(position.x, 2),
                "y": round(position.y, 2),
                "z": round(position.z, 3),
                "yaw": round(math.degrees(yaw)),
            }
            state["real_speed"] = round(
                math.hypot(velocity.x, velocity.y),
                2,
            )
            state["fallen"] = position.z < FALLEN_HEIGHT
            state["odom_time"] = time.time()

    def on_cmd(self, message: Twist):
        with lock:
            state["cmd"] = {
                "vx": round(message.linear.x, 2),
                "vy": round(message.linear.y, 2),
                "vyaw": round(message.angular.z, 2),
            }

    def on_mobility(self, message: String):
        self.update_json("mobility", message.data)

    def on_goal(self, message: PoseStamped):
        with lock:
            state["goal"] = {
                "x": round(message.pose.position.x, 2),
                "y": round(message.pose.position.y, 2),
            }

    def on_nav(self, message: String):
        with lock:
            state["nav"] = message.data
            if message.data in ("llegue", "cancelado"):
                state["goal"] = None

    def on_arms(self, message: String):
        with lock:
            state["arms"] = message.data

    def on_mission_event(self, message: String):
        with lock:
            state["mission_events"].append(
                {
                    "time": time.strftime("%H:%M:%S"),
                    "text": message.data,
                }
            )
            del state["mission_events"][:-HISTORY_MAX]

    def on_mission_state(self, message: String):
        try:
            mission_state = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if (
            not isinstance(mission_state, dict)
            or mission_state.get("schema_version") != 1
        ):
            return
        with lock:
            state["mission_state"] = mission_state

    def on_model_event(self, message: String):
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if (
            not isinstance(event, dict)
            or event.get("schema_version") != 1
            or not event.get("event_id")
        ):
            return
        with lock:
            events = state["model_events"]
            for index, previous in enumerate(events):
                if previous.get("event_id") == event["event_id"]:
                    events[index] = event
                    break
            else:
                events.append(event)
            del events[:-MODEL_EVENT_MAX]
        self.bind_latest_model_input()


PAGE_TEMPLATE = (
    Path(__file__)
    .with_name("index.html")
    .read_text(encoding="utf-8")
)
PAGE = PAGE_TEMPLATE.replace(
    "__SCENE_LAYOUT__",
    json.dumps(DASHBOARD_SCENE, ensure_ascii=False),
)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def send_headers(self, code: int, content_type: str):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store, must-revalidate")
        self.end_headers()

    def send_jpeg(self, jpeg: bytes, empty_message: bytes):
        if jpeg is None:
            self.send_headers(404, "text/plain; charset=utf-8")
            self.wfile.write(empty_message)
            return
        self.send_headers(200, "image/jpeg")
        self.wfile.write(jpeg)

    def do_GET(self):
        route = self.path.split("?")[0]
        if route == "/":
            self.send_headers(200, "text/html; charset=utf-8")
            self.wfile.write(PAGE.encode("utf-8"))
            return
        if route == "/camera.jpg":
            with lock:
                old = time.time() - state["camera_time"] > OFFLINE_AFTER_S
                jpeg = None if old else state["camera_jpeg"]
            self.send_jpeg(jpeg, "sin imagen todavía".encode("utf-8"))
            return
        if route == "/analysis.jpg":
            with lock:
                old = (
                    time.time() - state["analysis_time"]
                    > ANALYSIS_OFFLINE_AFTER_S
                )
                jpeg = None if old else state["analysis_jpeg"]
            self.send_jpeg(jpeg, "sin análisis todavía".encode("utf-8"))
            return
        if route == "/model-input.jpg":
            with lock:
                jpeg = state["model_input_jpeg"]
            self.send_jpeg(
                jpeg,
                "sin entrada de modelo todavía".encode("utf-8"),
            )
            return
        if route == "/state":
            with lock:
                now = time.time()
                data = {
                    key: value
                    for key, value in state.items()
                    if key
                    not in {
                        "camera_jpeg",
                        "analysis_jpeg",
                        "analysis_hold_until",
                        "model_input_jpeg",
                    }
                }
                data["online"] = (
                    now - state["odom_time"]
                ) < OFFLINE_AFTER_S
                data["silence_s"] = (
                    round(now - state["odom_time"])
                    if state["odom_time"]
                    else None
                )
                data["video_online"] = (
                    state["camera_jpeg"] is not None
                    and (now - state["camera_time"]) < OFFLINE_AFTER_S
                )
                data["camera_available"] = data["video_online"]
                data["analysis_available"] = (
                    state["analysis_jpeg"] is not None
                    and (now - state["analysis_time"])
                    <= ANALYSIS_OFFLINE_AFTER_S
                )
                data["model_input_available"] = (
                    state["model_input_jpeg"] is not None
                )
                data["analysis_age_s"] = (
                    round(now - state["analysis_time"], 1)
                    if state["analysis_time"]
                    else None
                )
                data["model_input_age_s"] = (
                    round(now - state["model_input_time"], 1)
                    if state["model_input_time"]
                    else None
                )
                payload = json.dumps(
                    data,
                    ensure_ascii=False,
                ).encode("utf-8")
            self.send_headers(200, "application/json; charset=utf-8")
            self.wfile.write(payload)
            return
        self.send_headers(404, "text/plain; charset=utf-8")
        self.wfile.write(b"?")


def main():
    rclpy.init()
    node = DashboardNode()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"[tablero] sirviendo en http://localhost:{PORT}", flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
