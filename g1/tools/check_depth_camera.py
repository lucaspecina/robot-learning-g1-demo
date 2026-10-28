#!/usr/bin/env python3
"""Verifica que color, profundidad y calibración describan el mismo cuadro."""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

TIMEOUT_S = 20.0
MIN_VALID_FRACTION = 0.05


def stamp_key(message) -> tuple[int, int]:
    return (
        message.header.stamp.sec,
        message.header.stamp.nanosec,
    )


class DepthCameraChecker(Node):
    def __init__(self):
        super().__init__("check_depth_camera")
        self.frames = {}
        self.result = None
        self.create_subscription(
            Image,
            "/g1/head_cam/image",
            self.on_color,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/g1/head_cam/depth",
            self.on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/g1/head_cam/camera_info",
            self.on_info,
            qos_profile_sensor_data,
        )

    def entry(self, message):
        key = stamp_key(message)
        entry = self.frames.setdefault(key, {})
        while len(self.frames) > 10:
            del self.frames[next(iter(self.frames))]
        return entry

    def on_color(self, message: Image):
        self.entry(message)["color"] = message
        self.try_validate(stamp_key(message))

    def on_depth(self, message: Image):
        self.entry(message)["depth"] = message
        self.try_validate(stamp_key(message))

    def on_info(self, message: CameraInfo):
        self.entry(message)["info"] = message
        self.try_validate(stamp_key(message))

    def try_validate(self, key):
        entry = self.frames.get(key, {})
        if set(entry) != {"color", "depth", "info"}:
            return
        color = entry["color"]
        depth_message = entry["depth"]
        info = entry["info"]
        if color.encoding != "rgb8":
            raise RuntimeError(f"color inesperado: {color.encoding}")
        if depth_message.encoding != "32FC1":
            raise RuntimeError(
                f"profundidad inesperada: {depth_message.encoding}"
            )
        dimensions = {
            (color.width, color.height),
            (depth_message.width, depth_message.height),
            (info.width, info.height),
        }
        if len(dimensions) != 1:
            raise RuntimeError(f"resoluciones distintas: {dimensions}")
        depth = np.frombuffer(
            depth_message.data,
            dtype=np.float32,
        ).reshape(depth_message.height, depth_message.width)
        valid = depth[np.isfinite(depth) & (depth > 0.0)]
        valid_fraction = valid.size / depth.size
        if valid_fraction < MIN_VALID_FRACTION:
            raise RuntimeError(
                f"sólo {valid_fraction:.1%} de la profundidad es válida"
            )
        if not (
            info.k[0] > 0.0
            and info.k[4] > 0.0
            and 0.0 <= info.k[2] < info.width
            and 0.0 <= info.k[5] < info.height
        ):
            raise RuntimeError("la calibración de cámara no es válida")
        self.result = {
            "stamp": key,
            "width": color.width,
            "height": color.height,
            "fx": info.k[0],
            "fy": info.k[4],
            "cx": info.k[2],
            "cy": info.k[5],
            "valid_fraction": valid_fraction,
            "min_depth": float(np.min(valid)),
            "median_depth": float(np.median(valid)),
            "max_depth": float(np.max(valid)),
        }


def main() -> int:
    rclpy.init()
    node = DepthCameraChecker()
    deadline = time.monotonic() + TIMEOUT_S
    try:
        while node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.result is None:
            print("FALLA: no llegó un cuadro completo y sincronizado")
            return 1
        result = node.result
        print(
            f"cuadro sincronizado: {result['width']}x{result['height']}, "
            f"hora {result['stamp'][0]}.{result['stamp'][1]:09d}"
        )
        print(
            f"calibración: fx={result['fx']:.1f}, fy={result['fy']:.1f}, "
            f"centro=({result['cx']:.1f}, {result['cy']:.1f})"
        )
        print(
            f"profundidad válida: {result['valid_fraction']:.1%}; "
            f"rango {result['min_depth']:.2f}–{result['max_depth']:.2f} m; "
            f"mediana {result['median_depth']:.2f} m"
        )
        print("PASA: color, profundidad y calibración comparten el cuadro")
        return 0
    except RuntimeError as error:
        print(f"FALLA PROFUNDIDAD: {error}")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
