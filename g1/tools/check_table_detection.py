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

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from open_vocabulary_core import make_search_request

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


class TableChecker(Node):
    def __init__(self):
        super().__init__("check_table_detection")
        self.pose = None
        self.nav_status = None
        self.robot_mode = None
        self.detections = []
        self.search_status = None
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
        print("APROBADO: la mesa y su color se reconocen desde la cámara viva")
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
