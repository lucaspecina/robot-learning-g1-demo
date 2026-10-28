#!/usr/bin/env python3
"""Verifica la alineación desde una mesa visible y registra el cuerpo completo."""

import argparse
import json
import math
import sys
import time
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from nav2_msgs.action import DockRobot  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from scene_layout import SCENE_POSITIONS  # noqa: E402


def roll_pitch_from_quaternion(w: float, x: float, y: float, z: float):
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return roll, math.asin(pitch_term)


class AlignmentChecker(Node):
    def __init__(self):
        super().__init__("check_table_alignment")
        self.client = ActionClient(self, DockRobot, "/g1/dock_to_table")
        self.pose = None
        self.mobility_owner = None
        self.alignment_status = None
        self.minimum_height = float("inf")
        self.maximum_tilt = 0.0
        self.maximum_speed = 0.0
        self.samples = 0
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(
            String,
            "/g1/mobility/status",
            self.on_mobility,
            10,
        )
        self.create_subscription(
            String,
            "/g1/alignment_status",
            self.on_alignment,
            10,
        )

    def on_odom(self, message: Odometry):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        linear = message.twist.twist.linear
        roll, pitch = roll_pitch_from_quaternion(
            orientation.w,
            orientation.x,
            orientation.y,
            orientation.z,
        )
        speed = math.hypot(float(linear.x), float(linear.y))
        self.pose = (float(position.x), float(position.y), float(position.z))
        self.minimum_height = min(self.minimum_height, float(position.z))
        self.maximum_tilt = max(self.maximum_tilt, abs(roll), abs(pitch))
        self.maximum_speed = max(self.maximum_speed, speed)
        self.samples += 1

    def on_mobility(self, message: String):
        try:
            self.mobility_owner = json.loads(message.data).get("owner")
        except json.JSONDecodeError:
            pass

    def on_alignment(self, message: String):
        try:
            self.alignment_status = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    def wait_for_inputs(self, timeout_s: float = 8.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None and self.mobility_owner is not None:
                return True
        return False

    def run(self, color: str) -> int:
        if not self.wait_for_inputs():
            print("FALLA ALINEACIÓN: no llegaron posición y autoridad")
            return 1
        start_pose = self.pose
        if not self.client.wait_for_server(timeout_sec=5.0):
            print("FALLA ALINEACIÓN: la Action DockRobot no está disponible")
            return 1

        table_name = f"{color}_table"
        table_x, table_y = SCENE_POSITIONS[table_name]
        request = DockRobot.Goal()
        request.use_dock_id = False
        request.dock_pose.header.frame_id = "map"
        request.dock_pose.pose.position.x = float(table_x)
        request.dock_pose.pose.position.y = float(table_y)
        request.dock_pose.pose.orientation.w = 1.0
        request.dock_type = table_name
        request.navigate_to_staging_pose = False
        request.max_staging_time = 0.0
        last_feedback_state = None

        def on_feedback(message):
            nonlocal last_feedback_state
            state = int(message.feedback.state)
            if state != last_feedback_state:
                print(
                    f"  estado DockRobot={state}, "
                    f"tiempo={message.feedback.docking_time.sec}."
                    f"{message.feedback.docking_time.nanosec:09d} s"
                )
                last_feedback_state = state

        send_future = self.client.send_goal_async(
            request,
            feedback_callback=on_feedback,
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=8.0)
        if not send_future.done() or send_future.result() is None:
            print("FALLA ALINEACIÓN: no se confirmó el envío")
            return 1
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            print("FALLA ALINEACIÓN: el objetivo fue rechazado")
            return 1

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + 200.0
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result_future.done():
            goal_handle.cancel_goal_async()
            print("FALLA ALINEACIÓN: venció el plazo del verificador")
            return 1
        wrapped = result_future.result()
        stand_deadline = time.monotonic() + 5.0
        while self.mobility_owner != "stand" and time.monotonic() < stand_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        status = self.alignment_status or {}
        print(
            "medición final: "
            f"distancia {status.get('distance_error_m', 'sin dato')} m, "
            f"ángulo {status.get('yaw_error_deg', 'sin dato')}°, "
            f"velocidad {status.get('linear_speed_mps', 'sin dato')} m/s, "
            f"giro {status.get('angular_speed_radps', 'sin dato')} rad/s"
        )
        print(
            "cuerpo: "
            f"altura mínima {self.minimum_height:.3f} m, "
            f"inclinación máxima {math.degrees(self.maximum_tilt):.1f}°, "
            f"velocidad máxima {self.maximum_speed:.3f} m/s, "
            f"{self.samples} muestras"
        )
        displacement = math.hypot(
            self.pose[0] - start_pose[0],
            self.pose[1] - start_pose[1],
        )
        print(
            f"base: inicio ({start_pose[0]:.3f}, {start_pose[1]:.3f}), "
            f"final ({self.pose[0]:.3f}, {self.pose[1]:.3f}), "
            f"recorrido neto {displacement:.3f} m"
        )
        print(
            "objetivo: "
            f"fuente {status.get('target_source', 'sin dato')}; "
            "refinamientos visuales "
            f"{status.get('detection_count', 'sin dato')}; "
            f"reintentos: {wrapped.result.num_retries}; "
            f"dueño final: {self.mobility_owner}"
        )
        result = wrapped.result
        if (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and result.success
            and self.mobility_owner == "stand"
        ):
            print("APROBADO: alineación medida y movilidad devuelta a STAND")
            return 0
        print(
            "FALLA ALINEACIÓN: "
            f"estado={wrapped.status}, código={result.error_code}, "
            f"detalle={result.error_msg}"
        )
        return 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("color", choices=("red", "blue"))
    args = parser.parse_args()
    rclpy.init()
    checker = AlignmentChecker()
    try:
        return checker.run(args.color)
    finally:
        checker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
