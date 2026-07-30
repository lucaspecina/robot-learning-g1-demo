#!/usr/bin/env python3
"""Verifica objetivo, progreso, cancelación y regreso seguro de navegación."""

import argparse
import json
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_ABORTED: "aborted",
    GoalStatus.STATUS_CANCELED: "canceled",
}


class NavigationActionCheck(Node):
    def __init__(self):
        super().__init__("navigation_action_check")
        self.pose = None
        self.mobility_owner = None
        self.feedback_count = 0
        self.last_feedback = None
        self.client = ActionClient(
            self,
            NavigateToPose,
            "/g1/navigate_to_pose",
        )
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(
            String,
            "/g1/mobility/status",
            self.on_mobility,
            10,
        )

    def on_odom(self, message: Odometry):
        self.pose = message.pose.pose

    def on_mobility(self, message: String):
        try:
            self.mobility_owner = json.loads(message.data)["owner"]
        except (json.JSONDecodeError, KeyError, TypeError):
            pass

    def on_feedback(self, message):
        feedback = message.feedback
        self.feedback_count += 1
        self.last_feedback = {
            "distance_remaining_m": round(
                float(feedback.distance_remaining),
                4,
            ),
            "navigation_time_s": round(
                float(feedback.navigation_time.sec)
                + float(feedback.navigation_time.nanosec) / 1_000_000_000.0,
                3,
            ),
        }

    @staticmethod
    def wait_future(future, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.done()

    def wait_pose(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while self.pose is None and time.monotonic() < deadline:
            time.sleep(0.02)
        return self.pose is not None

    def wait_stand(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.mobility_owner == "stand":
                return True
            time.sleep(0.02)
        return self.mobility_owner == "stand"

    def run(self, args) -> int:
        if not self.wait_pose(5.0):
            print("FALLA: no llegó la posición del robot")
            return 1
        if not self.client.wait_for_server(timeout_sec=5.0):
            print("FALLA: no apareció la Action de navegación")
            return 1

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = (
            float(self.pose.position.x) if args.current else args.x
        )
        goal.pose.pose.position.y = (
            float(self.pose.position.y) if args.current else args.y
        )
        if args.current:
            goal.pose.pose.orientation = self.pose.orientation
        else:
            goal.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(args.yaw / 2.0)

        sent = self.client.send_goal_async(
            goal,
            feedback_callback=self.on_feedback,
        )
        if not self.wait_future(sent, 5.0):
            print("FALLA: no se confirmó el objetivo")
            return 1
        handle = sent.result()
        if handle is None or not handle.accepted:
            print("FALLA: objetivo rechazado")
            return 1

        result_future = handle.get_result_async()
        if args.cancel_after is not None:
            deadline = time.monotonic() + args.cancel_after
            while time.monotonic() < deadline and not result_future.done():
                time.sleep(0.02)
            if not result_future.done():
                cancel = handle.cancel_goal_async()
                if not self.wait_future(cancel, 3.0):
                    print("FALLA: no se confirmó el pedido de cancelación")
                    return 1

        if not self.wait_future(result_future, args.timeout):
            print("FALLA: no llegó el resultado dentro del plazo de la prueba")
            return 1
        wrapped = result_future.result()
        state = STATUS_NAMES.get(wrapped.status, str(wrapped.status))
        safe = self.wait_stand(3.0)
        report = {
            "state": state,
            "feedback_count": self.feedback_count,
            "last_feedback": self.last_feedback,
            "mobility_owner": self.mobility_owner,
            "error": str(getattr(wrapped.result, "error_msg", "")),
        }
        print(json.dumps(report, ensure_ascii=False, sort_keys=True))

        if state != args.expect:
            print(f"FALLA: se esperaba {args.expect} y terminó {state}")
            return 1
        if self.feedback_count < 1:
            print("FALLA: no se recibió ninguna medición de progreso")
            return 1
        if not safe:
            print("FALLA: la movilidad no volvió a STAND")
            return 1
        print(
            f"PASA: {state}, {self.feedback_count} mediciones y dueño final STAND"
        )
        return 0


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--current", action="store_true")
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--cancel-after", type=float)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument(
        "--expect",
        choices=("succeeded", "aborted", "canceled"),
        default="succeeded",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = NavigationActionCheck()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        return_code = node.run(args)
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)
    raise SystemExit(return_code)


if __name__ == "__main__":
    main()
