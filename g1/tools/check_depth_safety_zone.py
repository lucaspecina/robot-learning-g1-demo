#!/usr/bin/env python3
"""Mide retornos de profundidad dentro de la reserva inmediata del cuerpo."""

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


TOPIC = "/g1/head_cam/points"
TARGET_FRAME = "base_footprint"
ZONE_BOUNDS = ([-0.55, -0.40, 0.10], [0.55, 0.40, 2.0])


class DepthSafetyZoneChecker(Node):
    def __init__(self):
        super().__init__("check_depth_safety_zone")
        self.cloud = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(
            PointCloud2,
            TOPIC,
            lambda message: setattr(self, "cloud", message),
            qos_profile_sensor_data,
        )


def main() -> int:
    rclpy.init()
    node = DepthSafetyZoneChecker()
    deadline = time.monotonic() + 35.0
    last_error = ""
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.cloud is None:
                continue
            try:
                transform = node.buffer.lookup_transform(
                    TARGET_FRAME,
                    node.cloud.header.frame_id,
                    Time.from_msg(node.cloud.header.stamp),
                    timeout=Duration(seconds=0.2),
                )
            except Exception as error:
                last_error = str(error)
                continue
            points = point_cloud2.read_points_numpy(
                node.cloud,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            body_points = transform_points(
                points,
                [translation.x, translation.y, translation.z],
                [rotation.x, rotation.y, rotation.z, rotation.w],
            )
            selected = points_in_box(body_points, ZONE_BOUNDS)
            print(
                f"nube total {body_points.shape[0]}; dentro de la zona "
                f"inmediata {selected.shape[0]}"
            )
            if not selected.size:
                print("PASA: la profundidad no ve retornos dentro del cuerpo")
                return 0
            bins = ((0.10, 0.40), (0.40, 0.80), (0.80, 1.20), (1.20, 2.00))
            counts = [
                int(np.count_nonzero((selected[:, 2] >= low) & (selected[:, 2] < high)))
                for low, high in bins
            ]
            print(
                "retornos por altura: "
                + ", ".join(
                    f"{low:.1f}–{high:.1f} m={count}"
                    for (low, high), count in zip(bins, counts)
                )
            )
            print(
                "límites medidos XYZ: "
                f"{np.min(selected, axis=0).round(3).tolist()} a "
                f"{np.max(selected, axis=0).round(3).tolist()}"
            )
            print("FALLA: Collision Monitor confundiría estos puntos con peligro")
            return 1
        print(f"FALLA: no hubo nube transformable: {last_error}")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
