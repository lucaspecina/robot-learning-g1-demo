#!/usr/bin/env python3
"""Mide velocidad residual durante STAND usando tiempo físico simulado."""
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class StandVelocityMeasurement(Node):
    def __init__(self):
        super().__init__("measure_stand_velocity")
        self.owner = None
        self.samples = []
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(
            String,
            "/g1/mobility/status",
            self.on_mobility,
            10,
        )

    def on_mobility(self, message: String):
        try:
            self.owner = json.loads(message.data).get("owner")
        except (json.JSONDecodeError, AttributeError):
            pass

    def on_odom(self, message: Odometry):
        if self.owner != "stand":
            return
        stamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) / 1_000_000_000.0
        )
        twist = message.twist.twist
        self.samples.append(
            (
                stamp,
                math.hypot(float(twist.linear.x), float(twist.linear.y)),
                abs(float(twist.angular.z)),
            )
        )


def main():
    rclpy.init()
    node = StandVelocityMeasurement()
    deadline = time.monotonic() + 45.0
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.samples and node.samples[-1][0] - node.samples[0][0] >= 5.0:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if len(node.samples) < 2:
        print("FALLA: no llegaron mediciones de STAND")
        return 1
    simulated_duration = node.samples[-1][0] - node.samples[0][0]
    if simulated_duration < 5.0:
        print(
            "FALLA: sólo se midieron "
            f"{simulated_duration:.2f} s físicos antes del plazo de pared"
        )
        return 1
    linear = np.asarray([sample[1] for sample in node.samples])
    angular = np.asarray([sample[2] for sample in node.samples])
    print(
        f"STAND durante {simulated_duration:.2f} s físicos; "
        f"muestras={len(node.samples)}"
    )
    print(
        "velocidad lineal m/s: "
        f"p50={np.percentile(linear, 50):.4f}; "
        f"p90={np.percentile(linear, 90):.4f}; "
        f"p95={np.percentile(linear, 95):.4f}; max={linear.max():.4f}"
    )
    print(
        "velocidad angular rad/s: "
        f"p50={np.percentile(angular, 50):.4f}; "
        f"p90={np.percentile(angular, 90):.4f}; "
        f"p95={np.percentile(angular, 95):.4f}; max={angular.max():.4f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
