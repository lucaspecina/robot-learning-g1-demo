#!/usr/bin/env python3
"""Verifica estructura, ritmo y valores básicos de la nube LiDAR viva."""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

TOPIC = "/g1/lidar/points"
EXPECTED_FRAME = "lidar_link"
# La nube completa pesa ~3,3 MiB y el T4 comparte render con cámara y LiDAR
# 2D. La ventana sólo evita cortar antes de la tercera vuelta; no baja la
# cantidad de muestras ni el requisito geométrico.
TIMEOUT_S = 45.0
MIN_MESSAGES = 3
MIN_POINTS = 100
MIN_AZIMUTH_COVERAGE_DEG = 330.0


def azimuth_coverage_deg(xyz: np.ndarray) -> float:
    """Mide el arco cubierto sin confundir un cruce de ±180° con 360°."""
    azimuth = np.mod(np.arctan2(xyz[:, 1], xyz[:, 0]), 2.0 * np.pi)
    azimuth.sort()
    gaps = np.diff(np.concatenate((azimuth, azimuth[:1] + 2.0 * np.pi)))
    return float(np.degrees(2.0 * np.pi - np.max(gaps)))


class LidarChecker(Node):
    def __init__(self):
        super().__init__("check_lidar")
        self.samples = []
        self.empty_messages = 0
        self.create_subscription(
            PointCloud2,
            TOPIC,
            self.on_cloud,
            qos_profile_sensor_data,
        )

    def on_cloud(self, message: PointCloud2):
        try:
            xyz = point_cloud2.read_points_numpy(
                message,
                field_names=["x", "y", "z"],
                skip_nans=True,
            )
        except (AssertionError, KeyError, ValueError) as error:
            raise RuntimeError(f"nube ROS inválida: {error}") from error
        xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
        ranges = np.linalg.norm(xyz, axis=1)
        valid = np.isfinite(ranges) & (ranges > 0.01) & (ranges < 100.0)
        xyz = xyz[valid]
        ranges = ranges[valid]
        if message.header.frame_id != EXPECTED_FRAME:
            raise RuntimeError(
                f"marco {message.header.frame_id!r}; esperaba {EXPECTED_FRAME!r}"
            )
        if ranges.size < MIN_POINTS:
            # Isaac Sim 5.1 emite mensajes vacíos entre barridos completos.
            # Esperarlos evita declarar una falla por el primer cuadro vacío.
            self.empty_messages += 1
            return
        self.samples.append(
            {
                "stamp": (
                    message.header.stamp.sec,
                    message.header.stamp.nanosec,
                ),
                "points": int(ranges.size),
                "bytes": len(message.data),
                "min_range": float(np.min(ranges)),
                "median_range": float(np.median(ranges)),
                "max_range": float(np.max(ranges)),
                "minimum": np.min(xyz, axis=0),
                "maximum": np.max(xyz, axis=0),
                "azimuth_coverage_deg": azimuth_coverage_deg(xyz),
                "received_at": time.monotonic(),
            }
        )


def main() -> int:
    rclpy.init()
    node = LidarChecker()
    deadline = time.monotonic() + TIMEOUT_S
    try:
        while len(node.samples) < MIN_MESSAGES and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.samples) < MIN_MESSAGES:
            print(
                f"FALLA LIDAR: llegaron {len(node.samples)}/{MIN_MESSAGES} "
                f"nubes completas y {node.empty_messages} mensajes vacíos"
            )
            return 1
        stamps = {sample["stamp"] for sample in node.samples}
        if len(stamps) != len(node.samples):
            raise RuntimeError("se repitió la hora de una nube")
        elapsed = (
            node.samples[-1]["received_at"]
            - node.samples[0]["received_at"]
        )
        rate = (len(node.samples) - 1) / elapsed if elapsed > 0.0 else 0.0
        points = [sample["points"] for sample in node.samples]
        sizes = [sample["bytes"] for sample in node.samples]
        last = node.samples[-1]
        coverages = [sample["azimuth_coverage_deg"] for sample in node.samples]
        if min(coverages) < MIN_AZIMUTH_COVERAGE_DEG:
            raise RuntimeError(
                "la nube todavía es un sector parcial: cobertura mínima "
                f"{min(coverages):.1f}°; se requieren "
                f"{MIN_AZIMUTH_COVERAGE_DEG:.1f}°"
            )
        print(
            f"{len(node.samples)} nubes en {elapsed:.2f} s "
            f"({rate:.2f} Hz de pared); "
            f"{node.empty_messages} mensajes vacíos ignorados"
        )
        print(
            f"puntos: {min(points)}–{max(points)}; "
            f"mensaje: {min(sizes) / 1024:.1f}–{max(sizes) / 1024:.1f} KiB"
        )
        print(
            "distancia última: "
            f"{last['min_range']:.2f}–{last['max_range']:.2f} m, "
            f"mediana {last['median_range']:.2f} m"
        )
        print(
            "límites XYZ: "
            f"{np.round(last['minimum'], 2)} a "
            f"{np.round(last['maximum'], 2)} m"
        )
        print(
            f"cobertura horizontal: {min(coverages):.1f}–"
            f"{max(coverages):.1f}°"
        )
        print(
            "PASA: cada nube 3D es finita, renovada y cubre una vuelta "
            "completa alrededor del robot"
        )
        return 0
    except RuntimeError as error:
        print(f"FALLA LIDAR: {error}")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
