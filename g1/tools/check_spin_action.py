#!/usr/bin/env python3
"""Mide un giro cancelable completo y verifica el regreso seguro a STAND."""
import argparse
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import Spin
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_odometry(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    return math.atan2(
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


class SpinCheck(Node):
    def __init__(self):
        super().__init__("check_spin_action")
        self.pose = None
        self.owner = None
        self.feedback_angle = 0.0
        self.feedback_count = 0
        self.client = ActionClient(self, Spin, "/g1/spin")
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(
            String,
            "/g1/mobility/status",
            self.on_mobility_status,
            10,
        )

    def on_odom(self, message: Odometry):
        position = message.pose.pose.position
        self.pose = (
            float(position.x),
            float(position.y),
            float(position.z),
            yaw_from_odometry(message),
        )

    def on_mobility_status(self, message: String):
        try:
            self.owner = json.loads(message.data).get("owner")
        except json.JSONDecodeError:
            pass

    def on_feedback(self, message):
        self.feedback_count += 1
        self.feedback_angle = float(
            message.feedback.angular_distance_traveled
        )

    def spin_until(self, condition, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if condition():
                return True
        return condition()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--degrees", type=float, default=90.0)
    parser.add_argument("--timeout-s", type=float, default=130.0)
    parser.add_argument("--cancel-after-s", type=float)
    parser.add_argument("--max-heading-error-deg", type=float, default=10.0)
    parser.add_argument("--max-feedback-error-deg", type=float, default=5.0)
    parser.add_argument("--max-drift-m", type=float, default=0.20)
    parser.add_argument("--min-height-m", type=float, default=0.65)
    args = parser.parse_args()

    rclpy.init()
    node = SpinCheck()
    try:
        if not node.client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("no está disponible /g1/spin")
        if not node.spin_until(
            lambda: node.pose is not None and node.owner == "stand",
            8.0,
        ):
            raise RuntimeError("no llegaron odometría y STAND iniciales")

        initial = node.pose
        request = Spin.Goal()
        request.target_yaw = math.radians(args.degrees)
        request.time_allowance.sec = int(args.timeout_s - 5.0)
        send_future = node.client.send_goal_async(
            request,
            feedback_callback=node.on_feedback,
        )
        if not node.spin_until(send_future.done, 8.0):
            raise RuntimeError("no se confirmó la aceptación del giro")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("el servidor rechazó el giro")

        result_future = goal_handle.get_result_async()
        if args.cancel_after_s is not None:
            cancel_deadline = time.monotonic() + args.cancel_after_s
            node.spin_until(
                lambda: (
                    result_future.done()
                    or time.monotonic() >= cancel_deadline
                ),
                args.cancel_after_s + 1.0,
            )
            if not result_future.done():
                cancel_future = goal_handle.cancel_goal_async()
                if not node.spin_until(cancel_future.done, 5.0):
                    raise RuntimeError("no se confirmó el pedido de cancelación")
        if not node.spin_until(result_future.done, args.timeout_s):
            raise RuntimeError("el giro no terminó dentro del plazo")
        wrapped = result_future.result()
        if not node.spin_until(lambda: node.owner == "stand", 5.0):
            raise RuntimeError("el giro terminó sin devolver STAND")

        final = node.pose
        drift_m = math.hypot(final[0] - initial[0], final[1] - initial[1])
        measured_turn = normalize_angle(final[3] - initial[3])
        requested_turn = math.radians(args.degrees)
        heading_error = abs(
            normalize_angle(measured_turn - requested_turn)
        )
        feedback_error = abs(node.feedback_angle - requested_turn)
        expected_status = (
            GoalStatus.STATUS_CANCELED
            if args.cancel_after_s is not None
            else GoalStatus.STATUS_SUCCEEDED
        )
        report = {
            "status": int(wrapped.status),
            "owner": node.owner,
            "requested_turn_deg": round(args.degrees, 3),
            "measured_turn_deg": round(math.degrees(measured_turn), 3),
            "heading_error_deg": round(math.degrees(heading_error), 3),
            "feedback_distance_deg": round(
                math.degrees(node.feedback_angle),
                3,
            ),
            "feedback_error_deg": round(
                math.degrees(feedback_error),
                3,
            ),
            "feedback_count": node.feedback_count,
            "position_drift_m": round(drift_m, 4),
            "height_m": round(final[2], 4),
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))

        failures = []
        if wrapped.status != expected_status:
            failures.append("la Action terminó con un estado inesperado")
        if node.feedback_count < 1:
            failures.append("la Action no publicó progreso")
        if args.cancel_after_s is None:
            if math.degrees(heading_error) > args.max_heading_error_deg:
                failures.append("el error angular final superó el límite")
            if math.degrees(feedback_error) > args.max_feedback_error_deg:
                failures.append("el progreso no coincide con el giro pedido")
        if drift_m > args.max_drift_m:
            failures.append("el desplazamiento durante el giro superó el límite")
        if final[2] < args.min_height_m:
            failures.append("la altura final indica una caída")
        if node.owner != "stand":
            failures.append("la autoridad final no quedó en STAND")
        if failures:
            raise RuntimeError("; ".join(failures))
        print("OK: giro, progreso y regreso a STAND confirmados")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
