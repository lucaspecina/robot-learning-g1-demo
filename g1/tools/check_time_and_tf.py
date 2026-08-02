#!/usr/bin/env python3
"""Verifica que tiempo, odometría y coordenadas formen la base de Nav2."""
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener


TIMEOUT_S = 30.0
MIN_CLOCK_SAMPLES = 5
CLOCK_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class TimeAndTfChecker(Node):
    def __init__(self):
        super().__init__("check_time_and_tf")
        self.clock_samples = []
        self.odom_samples = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            Clock,
            "/clock",
            self.on_clock,
            CLOCK_QOS,
        )
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)

    def on_clock(self, message: Clock):
        self.clock_samples.append(
            (stamp_seconds(message.clock), time.monotonic())
        )

    def on_odom(self, message: Odometry):
        self.odom_samples.append(message)


def finite_transform(transform) -> bool:
    values = [
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w,
    ]
    return all(math.isfinite(value) for value in values)


def main() -> int:
    rclpy.init()
    node = TimeAndTfChecker()
    deadline = time.monotonic() + TIMEOUT_S
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(node.clock_samples) < MIN_CLOCK_SAMPLES or not node.odom_samples:
                continue
            if not node.tf_buffer.can_transform(
                "odom", "base_footprint", rclpy.time.Time(), Duration(seconds=0.1)
            ):
                continue
            if not node.tf_buffer.can_transform(
                "base_footprint", "base_link", rclpy.time.Time(), Duration(seconds=0.1)
            ):
                continue
            if not node.tf_buffer.can_transform(
                "base_link", "lidar_link", rclpy.time.Time(), Duration(seconds=0.1)
            ):
                continue
            break

        if len(node.clock_samples) < MIN_CLOCK_SAMPLES:
            raise RuntimeError("no llegó un reloj simulado estable")
        sim_times = [sample[0] for sample in node.clock_samples]
        if any(right <= left for left, right in zip(sim_times, sim_times[1:])):
            raise RuntimeError("el reloj simulado no avanza de forma estricta")
        odom = node.odom_samples[-1]
        if odom.header.frame_id != "odom" or odom.child_frame_id != "base_link":
            raise RuntimeError(
                "odometría con marcos "
                f"{odom.header.frame_id!r} -> {odom.child_frame_id!r}"
            )

        transforms = [
            node.tf_buffer.lookup_transform(
                "odom", "base_footprint", rclpy.time.Time()
            ),
            node.tf_buffer.lookup_transform(
                "base_footprint", "base_link", rclpy.time.Time()
            ),
            node.tf_buffer.lookup_transform(
                "base_link", "lidar_link", rclpy.time.Time()
            ),
        ]
        if not all(finite_transform(transform) for transform in transforms):
            raise RuntimeError("la cadena de coordenadas contiene valores inválidos")

        sim_elapsed = sim_times[-1] - sim_times[0]
        wall_elapsed = (
            node.clock_samples[-1][1] - node.clock_samples[0][1]
        )
        rtf = sim_elapsed / wall_elapsed if wall_elapsed > 0.0 else 0.0
        print(
            f"reloj: {sim_elapsed:.3f} s simulados en {wall_elapsed:.3f} s "
            f"reales (RTF {rtf:.2f})"
        )
        print(
            "coordenadas: odom -> base_footprint -> "
            "base_link -> lidar_link"
        )
        print("PASA: tiempo y coordenadas cumplen la base requerida por Nav2")
        return 0
    except RuntimeError as error:
        print(f"FALLA TIEMPO/COORDENADAS: {error}")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
