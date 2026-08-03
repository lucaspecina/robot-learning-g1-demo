#!/usr/bin/env python3
"""Verifica repetición y error del reloj 3D sin darle su pose al agente."""
import math
import sys
import time
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from vision_msgs.msg import Detection3DArray  # noqa: E402

from landmark_geometry import select_consistent_landmark  # noqa: E402
from scene_layout import CLOCK_HEIGHT, SCENE_POSITIONS  # noqa: E402


class ClockLocalizationCheck(Node):
    def __init__(self):
        super().__init__("check_clock_localization")
        self.samples = []
        self.create_subscription(
            Detection3DArray,
            "/g1/clock_detections_3d",
            self.on_detections,
            qos_profile_sensor_data,
        )

    def on_detections(self, message: Detection3DArray):
        received_at = time.monotonic()
        for detection in message.detections:
            if not detection.results:
                continue
            hypothesis = detection.results[0].hypothesis
            if hypothesis.class_id != "clock":
                continue
            point = detection.bbox.center.position
            self.samples.append(
                {
                    "class_id": "clock",
                    "confidence": float(hypothesis.score),
                    "x": float(point.x),
                    "y": float(point.y),
                    "z": float(point.z),
                    "coordinate_frame": message.header.frame_id,
                    "frame_ref": {
                        "sec": message.header.stamp.sec,
                        "nanosec": message.header.stamp.nanosec,
                    },
                    "received_at": received_at,
                }
            )


def main():
    rclpy.init()
    node = ClockLocalizationCheck()
    deadline = time.monotonic() + 45.0
    estimate = None
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            estimate = select_consistent_landmark(
                node.samples,
                class_id="clock",
                now=time.monotonic(),
                max_age_s=30.0,
                minimum_samples=3,
                maximum_spread_m=0.20,
            )
            if estimate is not None:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if estimate is None:
        print("FALLA: no llegaron tres mediciones coherentes del reloj")
        return 1
    expected_x, expected_y = SCENE_POSITIONS["clock"]
    position_error = math.dist(
        (estimate["x"], estimate["y"], estimate["z"]),
        (expected_x, expected_y, CLOCK_HEIGHT),
    )
    print(
        "reloj medido: "
        f"({estimate['x']:.3f}, {estimate['y']:.3f}, {estimate['z']:.3f}) m; "
        f"muestras={estimate['sample_count']}; "
        f"dispersión={estimate['spread_m']:.3f} m; "
        f"error contra escena={position_error:.3f} m"
    )
    if position_error > 0.25:
        print("FALLA: la medición no coincide con el reloj físico de la escena")
        return 1
    print("PASA: el agente podría ubicar el reloj sin recibir su coordenada")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
