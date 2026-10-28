#!/usr/bin/env python3
"""Comprueba si la nube 3D cruda contiene el obstáculo físico conocido."""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from pointcloud_core import points_in_box, transform_points  # noqa: E402
from scene_layout import NAVIGATION_TEST_OBSTACLE  # noqa: E402


DEFAULT_TOPIC = "/g1/lidar/points"
TARGET_FRAME = "map"
TIMEOUT_S = 35.0
BOX_MARGIN_M = 0.05
LOW_OBSTACLE_HEIGHT_M = 0.45


class PointcloudObstacleChecker(Node):
    def __init__(self, topic):
        super().__init__("check_pointcloud_obstacle")
        self.cloud = None
        self.last_transform_error = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(
            PointCloud2,
            topic,
            self.on_cloud,
            qos_profile_sensor_data,
        )

    def on_cloud(self, message):
        self.cloud = message


def parse_args():
    parser = argparse.ArgumentParser(
        description="Comprueba si una nube 3D ve el cajón de navegación",
    )
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--target-frame", default=TARGET_FRAME)
    parser.add_argument(
        "--obstacle-height",
        type=float,
        default=LOW_OBSTACLE_HEIGHT_M,
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.obstacle_height <= 0.0:
        print("FALLA: la altura del obstáculo debe ser positiva")
        return 2
    rclpy.init()
    node = PointcloudObstacleChecker(args.topic)
    deadline = time.monotonic() + TIMEOUT_S
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.cloud is None:
                continue
            try:
                transform = node.buffer.lookup_transform(
                    args.target_frame,
                    node.cloud.header.frame_id,
                    Time.from_msg(node.cloud.header.stamp),
                    timeout=Duration(seconds=0.2),
                )
            except Exception as error:
                node.last_transform_error = str(error)
                continue
            points = point_cloud2.read_points_numpy(
                node.cloud,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            world_points = transform_points(
                points,
                [translation.x, translation.y, translation.z],
                [rotation.x, rotation.y, rotation.z, rotation.w],
            )
            obstacle = NAVIGATION_TEST_OBSTACLE
            half_x = obstacle["size_x"] / 2.0 + BOX_MARGIN_M
            half_y = obstacle["size_y"] / 2.0 + BOX_MARGIN_M
            selected = points_in_box(
                world_points,
                (
                    [
                        obstacle["x"] - half_x,
                        obstacle["y"] - half_y,
                        0.05,
                    ],
                    [
                        obstacle["x"] + half_x,
                        obstacle["y"] + half_y,
                        args.obstacle_height + BOX_MARGIN_M,
                    ],
                ),
            )
            horizontal_distance = np.hypot(
                world_points[:, 0] - obstacle["x"],
                world_points[:, 1] - obstacle["y"],
            )
            nearest_index = int(np.argmin(horizontal_distance))
            nearest = world_points[nearest_index]
            inside_xy = (
                (np.abs(world_points[:, 0] - obstacle["x"]) <= half_x)
                & (np.abs(world_points[:, 1] - obstacle["y"]) <= half_y)
            )
            print(
                f"nube: {world_points.shape[0]} puntos; "
                f"z {np.min(world_points[:, 2]):.2f}–"
                f"{np.max(world_points[:, 2]):.2f} m"
            )
            print(
                f"puntos dentro del cajón bajo: {selected.shape[0]} "
                f"en {args.target_frame}"
            )
            print(
                f"puntos sobre su planta a cualquier altura: "
                f"{np.count_nonzero(inside_xy)}; punto horizontal más cercano "
                f"{nearest.round(3).tolist()} a "
                f"{horizontal_distance[nearest_index]:.3f} m del centro"
            )
            if selected.size:
                print(
                    "PASA: el sensor 3D ve el obstáculo; si Nav2 no lo marca, "
                    "la falla está en la configuración del mapa"
                )
                return 0
            print(
                "FALLA: la nube 3D no contiene el cajón bajo; Nav2 no puede "
                "inventar un obstáculo que el sensor no midió"
            )
            return 1
        detail = (
            f": {node.last_transform_error}"
            if node.last_transform_error
            else ""
        )
        print(f"FALLA: no llegaron nube y transformación utilizables{detail}")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
