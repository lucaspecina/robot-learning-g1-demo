#!/usr/bin/env python3
"""Comprueba que `home` sea una pose capturada, no una constante."""
import json
import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


OUTBOUND_X = 0.8
OUTBOUND_Y = 1.8
OUTBOUND_YAW = 2.4228
MAX_OUTBOUND_ERROR_M = 0.12
MAX_HOME_ERROR_M = 0.11
MAX_YAW_ERROR_DEG = 6.0
MIN_OUTBOUND_DISTANCE_M = 1.50
NAVIGATION_TIMEOUT_S = 180.0


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class PlanarPose:
    x: float
    y: float
    height: float
    yaw: float


class HomeReturnChecker(Node):
    def __init__(self):
        super().__init__("check_home_return")
        self.pose = None
        self.robot_mode = None
        self.nav_status = None

        self.control_pub = self.create_publisher(String, "/g1/control", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/g1/goal", 10)
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(
            String,
            "/g1/robot_status",
            self.on_robot_status,
            10,
        )
        self.create_subscription(
            String,
            "/g1/nav_status",
            self.on_nav_status,
            10,
        )

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
        self.pose = PlanarPose(
            x=position.x,
            y=position.y,
            height=position.z,
            yaw=yaw,
        )

    def on_robot_status(self, message: String):
        try:
            self.robot_mode = json.loads(message.data)["mode"]
        except (json.JSONDecodeError, KeyError, TypeError):
            pass

    def on_nav_status(self, message: String):
        self.nav_status = message.data

    def wait_for_mode(
        self,
        expected: str,
        requested: str,
        timeout_s: float = 10.0,
    ) -> bool:
        end = time.monotonic() + timeout_s
        last_request = float("-inf")
        while time.monotonic() < end:
            now = time.monotonic()
            if now - last_request >= 0.5:
                self.control_pub.publish(String(data=requested))
                last_request = now
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.robot_mode == expected:
                return True
        return False

    def spin_for(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def navigate(self, goal: PlanarPose) -> bool:
        message = PoseStamped()
        message.header.frame_id = "odom"
        message.pose.position.x = goal.x
        message.pose.position.y = goal.y
        message.pose.orientation.z = math.sin(goal.yaw / 2.0)
        message.pose.orientation.w = math.cos(goal.yaw / 2.0)
        self.nav_status = None
        self.goal_pub.publish(message)

        end = time.monotonic() + NAVIGATION_TIMEOUT_S
        while time.monotonic() < end:
            self.spin_for(0.5)
            if self.pose is not None and self.pose.height < 0.60:
                return False
            if self.nav_status == "llegue":
                return True
            if self.nav_status == "cancelado":
                return False
        return False


def pose_error(expected: PlanarPose, actual: PlanarPose):
    position_error = math.hypot(
        expected.x - actual.x,
        expected.y - actual.y,
    )
    yaw_error_deg = abs(
        math.degrees(normalize_angle(expected.yaw - actual.yaw))
    )
    return position_error, yaw_error_deg


def main() -> int:
    rclpy.init()
    checker = HomeReturnChecker()
    try:
        if not checker.wait_for_mode("frozen", "freeze"):
            print("FALLO HOME: el robot no confirmó frozen")
            return 1
        if not checker.wait_for_mode("active", "start"):
            print("FALLO HOME: el robot no confirmó active")
            return 1

        # Se captura la estimación recibida por ROS. En el G1 real vendrá de
        # localización en el mapa; nunca se consulta una coordenada de Isaac.
        checker.spin_for(2.0)
        if checker.pose is None:
            print("FALLO HOME: no llegó odometría")
            return 1
        home = checker.pose
        print(
            f"  home capturado: ({home.x:.3f}, {home.y:.3f}), "
            f"orientación {math.degrees(home.yaw):.1f}°"
        )

        outbound = PlanarPose(
            x=OUTBOUND_X,
            y=OUTBOUND_Y,
            height=0.0,
            yaw=OUTBOUND_YAW,
        )
        if not checker.navigate(outbound):
            print("FALLO HOME: no completó la ida")
            return 1
        outbound_error = pose_error(outbound, checker.pose)
        outbound_distance = math.hypot(
            checker.pose.x - home.x,
            checker.pose.y - home.y,
        )
        print(
            f"  ida: error {outbound_error[0]:.3f} m, "
            f"orientación {outbound_error[1]:.1f}°, "
            f"distancia desde home {outbound_distance:.2f} m"
        )
        # La ida es la preparación de esta prueba, pero debe demostrar que el
        # robot realmente se alejó. Sin este control, un navegador que dijera
        # "llegué" sin moverse podría aprobar un regreso trivial.
        if (
            outbound_error[0] > MAX_OUTBOUND_ERROR_M
            or outbound_error[1] > MAX_YAW_ERROR_DEG
            or outbound_distance < MIN_OUTBOUND_DISTANCE_M
        ):
            print("FALLO HOME: la ida no produjo un alejamiento válido")
            return 1

        if not checker.navigate(home):
            print("FALLO HOME: no completó el regreso")
            return 1
        home_error = pose_error(home, checker.pose)
        print(
            f"  regreso: error {home_error[0]:.3f} m, "
            f"orientación {home_error[1]:.1f}°, "
            f"altura {checker.pose.height:.3f} m"
        )
        if (
            home_error[0] > MAX_HOME_ERROR_M
            or home_error[1] > MAX_YAW_ERROR_DEG
        ):
            print(
                "FALLO HOME: el regreso quedó fuera de "
                f"{MAX_HOME_ERROR_M:.2f} m / {MAX_YAW_ERROR_DEG:.1f}°"
            )
            return 1

        print(
            "APROBADO HOME: guardó una pose observada, se alejó y volvió."
        )
        return 0
    finally:
        checker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
