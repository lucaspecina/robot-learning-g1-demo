#!/usr/bin/env python3
"""Verifica varias posiciones 3D del objeto sin usar verdad de Isaac en runtime."""
import argparse
import json
import math
import sys
import time
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from vision_msgs.msg import Detection3DArray  # noqa: E402


class ObjectLocalizationVerifier(Node):
    def __init__(self):
        super().__init__("object_localization_verifier")
        self.samples = []
        self.seen_stamps = set()
        self.body_height = None
        self.mobility_owner = None
        self.create_subscription(
            Detection3DArray,
            "/g1/object_detections_3d",
            self.on_detections,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/g1/odom",
            self.on_odom,
            10,
        )
        self.create_subscription(
            String,
            "/g1/mobility/status",
            self.on_mobility,
            10,
        )

    def on_detections(self, message: Detection3DArray):
        stamp = (
            int(message.header.stamp.sec),
            int(message.header.stamp.nanosec),
        )
        if stamp in self.seen_stamps:
            return
        self.seen_stamps.add(stamp)
        for detection in message.detections:
            if not detection.results:
                continue
            hypothesis = detection.results[0].hypothesis
            if hypothesis.class_id != "transport_object":
                continue
            point = detection.bbox.center.position
            self.samples.append(
                {
                    "stamp": stamp,
                    "detector_class": detection.id.split("-", 1)[0],
                    "confidence": float(hypothesis.score),
                    "x": float(point.x),
                    "y": float(point.y),
                    "z": float(point.z),
                    "frame": message.header.frame_id,
                }
            )

    def on_odom(self, message: Odometry):
        self.body_height = float(message.pose.pose.position.z)

    def on_mobility(self, message: String):
        try:
            self.mobility_owner = json.loads(message.data).get("owner")
        except (json.JSONDecodeError, AttributeError):
            pass


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Mide repetibilidad y error de la posición 3D del objeto."
    )
    parser.add_argument("--expected-x", type=float, required=True)
    parser.add_argument("--expected-y", type=float, required=True)
    parser.add_argument("--expected-z", type=float, required=True)
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--timeout-s", type=float, default=35.0)
    parser.add_argument("--maximum-xy-error-m", type=float, default=0.18)
    parser.add_argument("--maximum-z-error-m", type=float, default=0.18)
    args = parser.parse_args()
    if args.samples < 2:
        raise SystemExit("--samples debe ser al menos 2")

    rclpy.init()
    node = ObjectLocalizationVerifier()
    deadline = time.monotonic() + args.timeout_s
    try:
        while (
            len(node.samples) < args.samples
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    selected = node.samples[:args.samples]
    for sample in selected:
        sample["xy_error_m"] = round(
            math.hypot(
                sample["x"] - args.expected_x,
                sample["y"] - args.expected_y,
            ),
            4,
        )
        sample["z_error_m"] = round(
            abs(sample["z"] - args.expected_z),
            4,
        )
        for key in ("confidence", "x", "y", "z"):
            sample[key] = round(sample[key], 4)

    accepted = (
        len(selected) == args.samples
        and all(sample["frame"] == "map" for sample in selected)
        and all(
            sample["xy_error_m"] <= args.maximum_xy_error_m
            and sample["z_error_m"] <= args.maximum_z_error_m
            for sample in selected
        )
        and node.mobility_owner == "stand"
        and node.body_height is not None
        and node.body_height >= 0.65
    )
    report = {
        "accepted": accepted,
        "expected_m": {
            "x": args.expected_x,
            "y": args.expected_y,
            "z": args.expected_z,
        },
        "required_samples": args.samples,
        "received_samples": len(selected),
        "maximum_xy_error_m": args.maximum_xy_error_m,
        "maximum_z_error_m": args.maximum_z_error_m,
        "mobility_owner": node.mobility_owner,
        "body_height_m": (
            round(node.body_height, 4)
            if node.body_height is not None
            else None
        ),
        "samples": selected,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
