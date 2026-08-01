#!/usr/bin/env python3
"""Mide cuánto se aparta la nube LiDAR de las paredes conocidas."""

import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


G1_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, G1_DIR)

from scene_layout import WORLD_BOUNDS  # noqa: E402


def yaw_from_odometry(message: Odometry) -> float:
    """Extrae el rumbo del robot desde la orientación publicada."""
    orientation = message.pose.pose.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
    )


class WallErrorMeter(Node):
    """Compara puntos medidos con los cuatro planos verticales de la sala."""

    def __init__(self, sample_count: int):
        super().__init__("measure_lidar_wall_error")
        self.sample_count = sample_count
        self.latest_pose = None
        self.frame_metrics = []
        self.positions = []
        self.skipped_clouds = 0
        self.create_subscription(
            Odometry,
            "/g1/odom",
            self.on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/g1/lidar/points",
            self.on_cloud,
            qos_profile_sensor_data,
        )

    def on_odometry(self, message: Odometry):
        position = message.pose.pose.position
        self.latest_pose = (
            float(position.x),
            float(position.y),
            yaw_from_odometry(message),
        )

    def on_cloud(self, message: PointCloud2):
        if self.latest_pose is None or len(self.frame_metrics) >= self.sample_count:
            self.skipped_clouds += 1
            return
        xyz = point_cloud2.read_points_numpy(
            message,
            field_names=["x", "y", "z"],
            skip_nans=True,
        )
        xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
        ranges = np.linalg.norm(xyz, axis=1)
        valid = np.isfinite(ranges) & (ranges > 0.20) & (ranges < 12.0)
        xyz = xyz[valid]
        if xyz.shape[0] == 0:
            self.skipped_clouds += 1
            return

        robot_x, robot_y, yaw = self.latest_pose
        cosine, sine = math.cos(yaw), math.sin(yaw)
        world_x = robot_x + cosine * xyz[:, 0] - sine * xyz[:, 1]
        world_y = robot_y + sine * xyz[:, 0] + cosine * xyz[:, 1]
        residuals = np.minimum.reduce(
            (
                np.abs(world_x - WORLD_BOUNDS["xmin"]),
                np.abs(world_x - WORLD_BOUNDS["xmax"]),
                np.abs(world_y - WORLD_BOUNDS["ymin"]),
                np.abs(world_y - WORLD_BOUNDS["ymax"]),
            )
        )
        # Mesas y robot son retornos válidos pero no sirven para medir la
        # deformación de los muros. El umbral sólo elige puntos cercanos a un
        # plano conocido; el error se calcula con sus distancias sin corregir.
        wall_residuals = residuals[residuals < 0.35]
        if wall_residuals.size < 100:
            self.skipped_clouds += 1
            return
        self.frame_metrics.append(
            (
                float(np.median(wall_residuals)),
                float(np.percentile(wall_residuals, 95)),
                int(wall_residuals.size),
                int(xyz.shape[0]),
            )
        )
        self.positions.append((robot_x, robot_y))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Mide error de paredes con el robot quieto o caminando."
    )
    parser.add_argument("--samples", type=int, default=40)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--label", default="medición")
    args = parser.parse_args()

    rclpy.init()
    node = WallErrorMeter(args.samples)
    deadline = time.monotonic() + args.timeout
    try:
        while (
            len(node.frame_metrics) < args.samples
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.frame_metrics) < args.samples:
            print(
                f"FALLA: {args.label}: {len(node.frame_metrics)}/"
                f"{args.samples} nubes útiles; {node.skipped_clouds} omitidas"
            )
            return 1
        metrics = np.asarray(node.frame_metrics, dtype=np.float64)
        positions = np.asarray(node.positions, dtype=np.float64)
        displacement = float(np.linalg.norm(positions[-1] - positions[0]))
        print(
            f"{args.label}: {len(metrics)} nubes; robot se desplazó "
            f"{displacement:.3f} m"
        )
        print(
            "error contra pared por cuadro: mediana "
            f"{np.median(metrics[:, 0]) * 100:.1f} cm; "
            f"p95 {np.median(metrics[:, 1]) * 100:.1f} cm; "
            f"peor p95 {np.max(metrics[:, 1]) * 100:.1f} cm"
        )
        print(
            "retornos de pared: "
            f"{int(np.min(metrics[:, 2]))}–{int(np.max(metrics[:, 2]))} "
            "por cuadro"
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
