#!/usr/bin/env python3
"""Verifica una mesa desde una pose conocida sólo por el banco de pruebas.

La coordenada no llega al agente. Se usa para aislar percepción de búsqueda:
primero demostramos que el sensor reconoce la mesa cuando realmente la mira;
después implementamos cómo encontrarla sin conocer su posición.
"""
import argparse
import json
import math
import sys
import time
import uuid
from pathlib import Path

import numpy as np

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import rclpy
import tf2_geometry_msgs  # noqa: F401
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray

from camera_stream import (
    SynchronizedCameraFrames,
    color_array,
    depth_array,
)
from depth_geometry import colored_table_point
from open_vocabulary_core import make_search_request
from perception_core import bounded_box
from scene_layout import SCENE_POSITIONS, TABLE_SIZE

OBSERVATION_POSES = {
    # A 1,4 m la cámara de la cabeza sólo veía la punta de la botella: la mesa
    # quedaba debajo del cuadro. 2,5 m es la distancia medida que la encuadra.
    "red": (1.5, 2.6, 0.0, "mesa_roja"),
    "blue": (1.5, -2.6, 0.0, "mesa_azul"),
}
NAVIGATION_TIMEOUT_S = 180.0
DETECTION_TIMEOUT_S = 12.0
SEARCH_TIMEOUT_S = 50.0
MIN_SAMPLES = 3
MAX_CAMERA_FRAMES = 120


class TableChecker(Node):
    def __init__(self):
        super().__init__("check_table_detection")
        self.pose = None
        self.nav_status = None
        self.robot_mode = None
        self.detections = []
        self.search_status = None
        self.camera_frames = SynchronizedCameraFrames(MAX_CAMERA_FRAMES)
        self.search_detections = {}
        # El análisis remoto puede tardar decenas de segundos. El historial
        # estándar de diez segundos de tf2 perdería justo la pose del cuadro
        # analizado y obligaría a mezclarlo con la pose actual del robot.
        self.tf_buffer = Buffer(cache_time=Duration(seconds=120.0))
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )
        self.control_pub = self.create_publisher(String, "/g1/control", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/g1/goal", 10)
        self.search_pub = self.create_publisher(
            String,
            "/g1/perception/search_request",
            10,
        )
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(String, "/g1/nav_status", self.on_nav, 10)
        self.create_subscription(
            String,
            "/g1/robot_status",
            self.on_robot_status,
            10,
        )
        self.create_subscription(
            String,
            "/g1/detections",
            self.on_detections,
            10,
        )
        self.create_subscription(
            String,
            "/g1/perception/search_status",
            self.on_search_status,
            10,
        )
        self.create_subscription(
            Image,
            "/g1/head_cam/image",
            self.on_color,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/g1/head_cam/depth",
            self.on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/g1/head_cam/camera_info",
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection2DArray,
            "/g1/open_vocabulary_detections",
            self.on_open_vocabulary_detections,
            qos_profile_sensor_data,
        )

    def on_odom(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        self.pose = (position.x, position.y, position.z, yaw)

    def on_nav(self, msg: String):
        self.nav_status = msg.data

    def on_robot_status(self, msg: String):
        try:
            self.robot_mode = json.loads(msg.data)["mode"]
        except (json.JSONDecodeError, KeyError, TypeError):
            pass

    def on_detections(self, msg: String):
        try:
            self.detections.append(json.loads(msg.data))
        except json.JSONDecodeError:
            pass

    def on_search_status(self, msg: String):
        try:
            self.search_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def on_color(self, message: Image):
        self.camera_frames.add("color", message)

    def on_depth(self, message: Image):
        self.camera_frames.add("depth", message)

    def on_camera_info(self, message: CameraInfo):
        self.camera_frames.add("info", message)

    def on_open_vocabulary_detections(self, message: Detection2DArray):
        for detection in message.detections:
            parts = detection.id.split(":")
            if len(parts) >= 3 and parts[0] == "grounding_dino":
                self.search_detections.setdefault(parts[1], []).append(
                    (message.header, detection)
                )

    def wait_for_active(self) -> bool:
        end = time.monotonic() + 10.0
        last_request = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now - last_request >= 0.5:
                self.control_pub.publish(String(data="start"))
                last_request = now
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.robot_mode == "active":
                return True
        return False

    def wait_for_camera_transform(self) -> bool:
        end = time.monotonic() + 10.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            frame = self.camera_frames.latest_complete()
            if frame is None:
                continue
            header = frame["color"].header
            if self.tf_buffer.can_transform(
                "map",
                header.frame_id,
                Time.from_msg(header.stamp),
                timeout=Duration(seconds=0.1),
            ):
                return True
        return False

    def send_goal(self, x: float, y: float, yaw: float):
        message = PoseStamped()
        message.header.frame_id = "odom"
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.orientation.z = math.sin(yaw / 2)
        message.pose.orientation.w = math.cos(yaw / 2)
        self.nav_status = None
        self.goal_pub.publish(message)

    def wait_for_arrival(self) -> bool:
        end = time.monotonic() + NAVIGATION_TIMEOUT_S
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None and self.pose[2] < 0.60:
                raise RuntimeError(
                    f"el robot cayó: altura {self.pose[2]:.3f} m"
                )
            if self.nav_status == "llegue":
                return True
        return False

    def collect_detections(self):
        self.detections.clear()
        end = time.monotonic() + DETECTION_TIMEOUT_S
        while len(self.detections) < MIN_SAMPLES and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return list(self.detections)

    def request_search(self, target: str) -> dict:
        request_id = str(uuid.uuid4())
        request = make_search_request(target, request_id)
        self.search_status = None
        self.detections.clear()
        self.search_detections.pop(request_id, None)

        # Esperar que exista el consumidor evita que un único mensaje se
        # pierda justo mientras se reinician las capas de percepción.
        discovery_end = time.monotonic() + 5.0
        while (
            self.search_pub.get_subscription_count() == 0
            and time.monotonic() < discovery_end
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.search_pub.get_subscription_count() == 0:
            raise RuntimeError("la búsqueda visual no está corriendo")

        self.search_pub.publish(
            String(data=json.dumps(request, ensure_ascii=False))
        )
        end = time.monotonic() + SEARCH_TIMEOUT_S
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            status = self.search_status or {}
            if status.get("request_id") != request_id:
                continue
            if status.get("state") == "complete":
                return status
            if status.get("state") in ("failed", "rejected"):
                raise RuntimeError(
                    f"la búsqueda visual falló: {status.get('error', status)}"
                )
        raise RuntimeError("la búsqueda visual no respondió a tiempo")

    def measure_search_result(self, request_id: str):
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            candidates = self.search_detections.get(request_id, [])
            ready = [
                (header, detection)
                for header, detection in candidates
                if self.camera_frames.complete(header) is not None
            ]
            if ready:
                break
        else:
            raise RuntimeError(
                "la detección no conserva su cuadro sincronizado de cámara"
            )

        def confidence(item):
            detection = item[1]
            if not detection.results:
                return 0.0
            return detection.results[0].hypothesis.score

        header, detection = max(ready, key=confidence)
        frame = self.camera_frames.complete(header)
        color_message = frame["color"]
        depth_message = frame["depth"]
        info = frame["info"]
        if (
            color_message.width != depth_message.width
            or color_message.height != depth_message.height
            or info.width != color_message.width
            or info.height != color_message.height
        ):
            raise RuntimeError("el cuadro sincronizado tiene tamaños distintos")
        center = detection.bbox.center.position
        box = bounded_box(
            center.x,
            center.y,
            detection.bbox.size_x,
            detection.bbox.size_y,
            color_message.width,
            color_message.height,
        )
        try:
            camera_point = colored_table_point(
                color_array(color_message),
                depth_array(depth_message),
                np.asarray(info.k, dtype=np.float64).reshape(3, 3),
                box,
            )
        except ValueError as error:
            raise RuntimeError(
                f"no se pudo medir la superficie de la mesa: {error}"
            ) from error
        point_message = PointStamped()
        point_message.header = header
        point_message.point.x = camera_point.right_m
        point_message.point.y = camera_point.down_m
        point_message.point.z = camera_point.forward_m
        try:
            map_point = self.tf_buffer.transform(
                point_message,
                "map",
                timeout=Duration(seconds=2.0),
            )
        except TransformException as error:
            raise RuntimeError(
                f"no se pudo llevar la medición al mapa: {error}"
            ) from error
        return camera_point, map_point.point


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Navega a una vista de prueba y verifica el color de la mesa."
    )
    parser.add_argument("color", choices=sorted(OBSERVATION_POSES))
    parser.add_argument(
        "--current-view",
        action="store_true",
        help="no navega; analiza lo que la cámara está viendo ahora",
    )
    args = parser.parse_args()
    goal_x, goal_y, goal_yaw, expected_name = OBSERVATION_POSES[args.color]

    rclpy.init()
    checker = TableChecker()
    try:
        if not checker.wait_for_active():
            raise RuntimeError("el robot no confirmó el estado activo")
        if not checker.wait_for_camera_transform():
            raise RuntimeError(
                "no llegó un cuadro con su ubicación temporal en el mapa"
            )
        if args.current_view:
            print(
                f"vista actual: ({checker.pose[0]:.2f}, "
                f"{checker.pose[1]:.2f}), altura {checker.pose[2]:.3f} m"
            )
        else:
            checker.send_goal(goal_x, goal_y, goal_yaw)
            if not checker.wait_for_arrival():
                raise RuntimeError("la navegación no reportó llegada")
            position_error = math.hypot(
                goal_x - checker.pose[0],
                goal_y - checker.pose[1],
            )
            yaw_error = abs(
                math.atan2(
                    math.sin(goal_yaw - checker.pose[3]),
                    math.cos(goal_yaw - checker.pose[3]),
                )
            )
            print(
                f"llegada: error {position_error:.3f} m, "
                f"orientación {math.degrees(yaw_error):.1f}°"
            )
        search_status = checker.request_search(f"{args.color}_table")
        print(
            "búsqueda puntual: "
            f"{search_status.get('count')} resultado(s), "
            f"{search_status.get('elapsed_s')} s totales, "
            f"{search_status.get('inference_s')} s de modelo"
        )
        point, map_point = checker.measure_search_result(
            search_status["request_id"]
        )
        print(
            "distancia visual: "
            f"{point.forward_m:.2f} m hacia delante, "
            f"{point.right_m:+.2f} m a la derecha, "
            f"{point.down_m:+.2f} m hacia abajo "
            f"({point.sample_count} píxeles {point.color})"
        )
        print(
            "posición medida en el mapa: "
            f"({map_point.x:.2f}, {map_point.y:.2f}, {map_point.z:.2f}) m"
        )
        if point.color != args.color:
            raise RuntimeError(
                f"la profundidad pertenece al color {point.color}, "
                f"no al {args.color}"
            )
        table_center = SCENE_POSITIONS[f"{args.color}_table"]
        half_x, half_y = TABLE_SIZE[0] / 2, TABLE_SIZE[1] / 2
        if not (
            table_center[0] - half_x - 0.10
            <= map_point.x
            <= table_center[0] + half_x + 0.10
            and table_center[1] - half_y - 0.10
            <= map_point.y
            <= table_center[1] + half_y + 0.10
            and 0.0 <= map_point.z <= 1.10
        ):
            raise RuntimeError(
                "la distancia visual no cae dentro de la mesa real de la "
                f"escena: ({map_point.x:.2f}, {map_point.y:.2f}, "
                f"{map_point.z:.2f})"
            )
        samples = checker.collect_detections()
        if len(samples) < MIN_SAMPLES:
            raise RuntimeError(
                f"el detector respondió sólo {len(samples)} veces"
            )
        expected_count = sum(expected_name in sample for sample in samples)
        opposite_name = (
            "mesa_azul" if expected_name == "mesa_roja" else "mesa_roja"
        )
        opposite_count = sum(opposite_name in sample for sample in samples)
        print(
            f"mesa esperada: {expected_name} en {expected_count}/{len(samples)} "
            f"respuestas; color opuesto en {opposite_count}/{len(samples)}"
        )
        if expected_count < MIN_SAMPLES:
            raise RuntimeError(
                f"la mesa no fue estable: {expected_count}/{len(samples)}"
            )
        if opposite_count:
            raise RuntimeError("la mesa apareció también con el color opuesto")
        print(
            "APROBADO: la mesa se reconoce y se mide desde el mismo "
            "cuadro de cámara"
        )
        return 0
    except RuntimeError as error:
        print(f"FALLA MESA: {error}")
        if not args.current_view:
            # Una navegación que vence no debe dejar una intención vieja
            # moviendo al robot sin nadie que la supervise. Una consulta desde
            # la vista actual nunca adquirió movilidad y no requiere reset.
            for _ in range(3):
                checker.control_pub.publish(String(data="freeze"))
                rclpy.spin_once(checker, timeout_sec=0.2)
        return 1
    finally:
        checker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
