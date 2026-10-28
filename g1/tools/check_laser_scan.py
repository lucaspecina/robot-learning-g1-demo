#!/usr/bin/env python3
"""Verifica que el barrido 2D oficial sea completo y útil para SLAM."""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


TOPIC = "/scan"
EXPECTED_FRAME = "lidar_link"
TIMEOUT_S = 35.0
MIN_MESSAGES = 3
MIN_ANGLE_SPAN_DEG = 350.0
MIN_FINITE_RAYS = 100


class LaserScanChecker(Node):
    def __init__(self):
        super().__init__("check_laser_scan")
        self.samples = []
        self.create_subscription(
            LaserScan,
            TOPIC,
            self.on_scan,
            qos_profile_sensor_data,
        )

    def on_scan(self, message: LaserScan):
        ranges = np.asarray(message.ranges, dtype=np.float32)
        valid_mask = (
            np.isfinite(ranges)
            & (ranges >= message.range_min)
            & (ranges <= message.range_max)
        )
        finite = ranges[valid_mask]
        near_indices = np.flatnonzero(valid_mask & (ranges < 0.60))
        near_angles = [
            math.degrees(
                message.angle_min + int(index) * message.angle_increment
            )
            for index in near_indices
        ]
        self.samples.append(
            {
                "frame": message.header.frame_id,
                "stamp": (
                    message.header.stamp.sec,
                    message.header.stamp.nanosec,
                ),
                "rays": int(ranges.size),
                "finite_rays": int(finite.size),
                "minimum": float(np.min(finite)) if finite.size else math.inf,
                "maximum": float(np.max(finite)) if finite.size else math.inf,
                "near_count": int(near_indices.size),
                "near_angles": near_angles,
                "at_minimum_count": int(
                    np.count_nonzero(
                        valid_mask
                        & np.isclose(
                            ranges,
                            message.range_min,
                            rtol=0.0,
                            atol=1e-4,
                        )
                    )
                ),
                "near_maximum": (
                    float(np.max(ranges[near_indices]))
                    if near_indices.size
                    else math.inf
                ),
                "span_deg": math.degrees(
                    float(message.angle_max - message.angle_min)
                ),
                "received_at": time.monotonic(),
            }
        )


def main() -> int:
    rclpy.init()
    node = LaserScanChecker()
    deadline = time.monotonic() + TIMEOUT_S
    try:
        while len(node.samples) < MIN_MESSAGES and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.samples) < MIN_MESSAGES:
            raise RuntimeError(
                f"llegaron {len(node.samples)}/{MIN_MESSAGES} barridos"
            )
        if any(sample["frame"] != EXPECTED_FRAME for sample in node.samples):
            raise RuntimeError("el barrido no está expresado en lidar_link")
        if len({sample["stamp"] for sample in node.samples}) != len(node.samples):
            raise RuntimeError("se repitió la hora de un barrido")
        if min(sample["span_deg"] for sample in node.samples) < MIN_ANGLE_SPAN_DEG:
            raise RuntimeError(
                "el ángulo publicado no completa una vuelta: "
                f"{min(sample['span_deg'] for sample in node.samples):.1f}°"
            )
        if min(sample["finite_rays"] for sample in node.samples) < MIN_FINITE_RAYS:
            raise RuntimeError(
                "muy pocos rayos tocaron la habitación: "
                f"{min(sample['finite_rays'] for sample in node.samples)}"
            )
        elapsed = node.samples[-1]["received_at"] - node.samples[0]["received_at"]
        rate = (len(node.samples) - 1) / elapsed if elapsed > 0.0 else 0.0
        print(
            f"{len(node.samples)} vueltas en {elapsed:.2f} s "
            f"({rate:.2f} Hz de pared)"
        )
        print(
            "barrido: "
            f"{min(sample['rays'] for sample in node.samples)}–"
            f"{max(sample['rays'] for sample in node.samples)} rayos; "
            f"{min(sample['finite_rays'] for sample in node.samples)}–"
            f"{max(sample['finite_rays'] for sample in node.samples)} válidos"
        )
        print(
            "distancias: "
            f"{min(sample['minimum'] for sample in node.samples):.2f}–"
            f"{max(sample['maximum'] for sample in node.samples):.2f} m; "
            f"ángulo {min(sample['span_deg'] for sample in node.samples):.1f}°"
        )
        nearest = max(node.samples, key=lambda sample: sample["near_count"])
        if nearest["near_count"]:
            angles = ", ".join(
                f"{angle:.1f}°" for angle in nearest["near_angles"][:12]
            )
            suffix = "..." if nearest["near_count"] > 12 else ""
            print(
                f"rayos a menos de 0,60 m: {nearest['near_count']}; "
                f"{nearest['at_minimum_count']} exactamente en el mínimo y "
                f"máximo cercano {nearest['near_maximum']:.3f} m; "
                f"rango angular {min(nearest['near_angles']):.1f}° a "
                f"{max(nearest['near_angles']):.1f}°; "
                f"primeros {angles}{suffix}"
            )
        else:
            print("rayos a menos de 0,60 m: 0")
        print("PASA: /scan contiene vueltas completas para SLAM Toolbox")
        return 0
    except RuntimeError as error:
        print(f"FALLA BARRIDO 2D: {error}")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
