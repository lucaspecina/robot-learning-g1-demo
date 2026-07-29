#!/usr/bin/env python3
"""Verifica llegada y detección del reloj sin reemplazar la inspección visual.

La prueba contiene un caso negativo y uno positivo:

1. En el origen, donde el reloj no está en cuadro, no debe detectarlo.
2. Después de navegar, debe quedar cerca, orientado y detectar el display de
   forma estable durante varias imágenes.

Uso (dentro del contenedor jetson):
    python3 /workspace/g1/tools/check_clock.py
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


GOAL_X = 0.8
GOAL_Y = 1.8
GOAL_YAW = 2.4228
MAX_POSITION_ERROR_M = 0.11
MAX_YAW_ERROR_DEG = 6.0
MIN_DETECTION_RATIO = 0.80
MAX_CENTER_ERROR = 0.15
NEGATIVE_OBSERVATION_S = 3.0
POSITIVE_OBSERVATION_S = 5.0
NAVIGATION_TIMEOUT_S = 180.0


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class ClockChecker(Node):
    def __init__(self):
        super().__init__("check_clock")
        self.pose = None
        self.nav_status = None
        self.robot_mode = None
        self.detection_history = []

        self.control_pub = self.create_publisher(String, "/g1/control", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/g1/goal", 10)
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(
            String,
            "/g1/nav_status",
            self.on_nav_status,
            10,
        )
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

    def on_odom(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
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
        self.pose = (position.x, position.y, position.z, yaw)

    def on_nav_status(self, msg: String):
        self.nav_status = msg.data

    def on_robot_status(self, msg: String):
        try:
            self.robot_mode = json.loads(msg.data)["mode"]
        except (json.JSONDecodeError, KeyError, TypeError):
            pass

    def on_detections(self, msg: String):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.detection_history.append(detections)

    def spin_for(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_mode(
        self,
        expected: str,
        timeout_s: float,
        requested_mode: str = None,
    ) -> bool:
        end = time.monotonic() + timeout_s
        last_request = float("-inf")
        while time.monotonic() < end:
            now = time.monotonic()
            if requested_mode is not None and now - last_request >= 0.5:
                # Los topics no guardan esta orden. Repetir hasta recibir la
                # confirmación evita perder el primer mensaje durante el
                # descubrimiento de ROS.
                self.set_mode(requested_mode)
                last_request = now
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.robot_mode == expected:
                return True
        return False

    def set_mode(self, mode: str):
        self.control_pub.publish(String(data=mode))

    def send_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.pose.position.x = GOAL_X
        goal.pose.position.y = GOAL_Y
        goal.pose.orientation.z = math.sin(GOAL_YAW / 2.0)
        goal.pose.orientation.w = math.cos(GOAL_YAW / 2.0)
        self.nav_status = None
        self.goal_pub.publish(goal)


def fail(reason: str) -> int:
    print(f"\n  FALLA RELOJ: {reason}\n")
    return 1


def main() -> int:
    rclpy.init()
    checker = ClockChecker()
    try:
        if not checker.wait_for_mode("frozen", 10.0, "freeze"):
            return fail("el robot no confirmó el estado congelado")

        checker.detection_history.clear()
        checker.spin_for(NEGATIVE_OBSERVATION_S)
        negative_frames = len(checker.detection_history)
        false_clock_frames = sum(
            "reloj" in detections
            for detections in checker.detection_history
        )
        if negative_frames < 3:
            return fail("llegaron muy pocas imágenes en el origen")
        if false_clock_frames:
            return fail(
                f"detectó reloj en {false_clock_frames}/{negative_frames} "
                "imágenes donde no estaba en cuadro"
            )
        print(
            f"  negativo: 0/{negative_frames} falsos relojes en el origen"
        )

        if not checker.wait_for_mode("active", 10.0, "start"):
            return fail("el robot no confirmó el estado activo")
        checker.spin_for(2.0)
        checker.send_goal()

        navigation_end = time.monotonic() + NAVIGATION_TIMEOUT_S
        while time.monotonic() < navigation_end:
            checker.spin_for(1.0)
            if checker.pose is not None and checker.pose[2] < 0.60:
                return fail(
                    f"el robot cayó: altura {checker.pose[2]:.3f} m"
                )
            if checker.nav_status == "llegue":
                break
        if checker.nav_status != "llegue" or checker.pose is None:
            return fail("la navegación no reportó llegada")

        x, y, height, yaw = checker.pose
        position_error = math.hypot(GOAL_X - x, GOAL_Y - y)
        yaw_error_deg = abs(
            math.degrees(normalize_angle(GOAL_YAW - yaw))
        )
        print(
            f"  llegada: error {position_error:.3f} m, "
            f"orientación {yaw_error_deg:.1f}°, altura {height:.3f} m"
        )
        if position_error > MAX_POSITION_ERROR_M:
            return fail(
                f"error de posición {position_error:.3f} m "
                f"(máximo {MAX_POSITION_ERROR_M:.2f})"
            )
        if yaw_error_deg > MAX_YAW_ERROR_DEG:
            return fail(
                f"error de orientación {yaw_error_deg:.1f}° "
                f"(máximo {MAX_YAW_ERROR_DEG:.1f}°)"
            )

        checker.detection_history.clear()
        checker.spin_for(POSITIVE_OBSERVATION_S)
        total_frames = len(checker.detection_history)
        clock_detections = [
            detections["reloj"]
            for detections in checker.detection_history
            if "reloj" in detections
        ]
        bottle_false_frames = sum(
            "botella" in detections
            for detections in checker.detection_history
        )
        detection_ratio = (
            len(clock_detections) / total_frames
            if total_frames
            else 0.0
        )
        if not clock_detections:
            return fail("el reloj no apareció en ninguna imagen final")

        mean_center = sum(
            detection["cx"] for detection in clock_detections
        ) / len(clock_detections)
        print(
            f"  percepción: reloj en {len(clock_detections)}/{total_frames} "
            f"imágenes, centro {mean_center:.3f}, "
            f"falsos botella {bottle_false_frames}"
        )
        if detection_ratio < MIN_DETECTION_RATIO:
            return fail(
                f"detección inestable: {detection_ratio:.0%} "
                f"(mínimo {MIN_DETECTION_RATIO:.0%})"
            )
        if abs(mean_center - 0.5) > MAX_CENTER_ERROR:
            return fail(
                f"reloj descentrado: centro {mean_center:.3f}"
            )
        if bottle_false_frames:
            return fail(
                f"confundió el display con botella en "
                f"{bottle_false_frames}/{total_frames} imágenes"
            )

        print(
            "\n  PASA RELOJ: navegación y detección numérica correctas. "
            "La apariencia sigue pendiente de validación humana.\n"
        )
        return 0
    finally:
        checker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
