#!/usr/bin/env python3
"""Comprueba que el mapa global sea real y cierre el árbol de coordenadas."""
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener


TIMEOUT_S = 75.0
MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
MAX_GROUND_TRUTH_CORRECTION_M = 0.15
MAX_GROUND_TRUTH_CORRECTION_DEG = 5.0
MAX_VERTICAL_CORRECTION_M = 0.02
MAX_TILT_CORRECTION_DEG = 1.0


class SlamMapChecker(Node):
    def __init__(self):
        super().__init__("check_slam_map")
        self.maps = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, MAP_QOS)

    def on_map(self, message: OccupancyGrid):
        self.maps.append(message)


def main() -> int:
    rclpy.init()
    node = SlamMapChecker()
    deadline = time.monotonic() + TIMEOUT_S
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.maps:
                continue
            if node.tf_buffer.can_transform(
                "map", "odom", rclpy.time.Time(), Duration(seconds=0.2)
            ):
                break
        if not node.maps:
            raise RuntimeError("ninguna fuente publicó /map")
        message = node.maps[-1]
        if message.header.frame_id != "map":
            raise RuntimeError(f"el mapa usa el marco {message.header.frame_id!r}")
        if not node.tf_buffer.can_transform(
            "map", "odom", rclpy.time.Time(), Duration(seconds=0.2)
        ):
            raise RuntimeError("falta la corrección map -> odom")
        transform = node.tf_buffer.lookup_transform(
            "map", "odom", rclpy.time.Time()
        ).transform
        translation_correction = math.hypot(
            transform.translation.x,
            transform.translation.y,
        )
        rotation = transform.rotation
        yaw_correction_deg = abs(
            math.degrees(
                math.atan2(
                    2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                    1.0 - 2.0 * (rotation.y**2 + rotation.z**2),
                )
            )
        )
        tilt_correction_deg = math.degrees(
            2.0
            * math.asin(
                min(1.0, math.hypot(rotation.x, rotation.y))
            )
        )
        if abs(transform.translation.z) > MAX_VERTICAL_CORRECTION_M:
            raise RuntimeError(
                "el mapa desplazó verticalmente la habitación: "
                f"{transform.translation.z:.3f} m"
            )
        if tilt_correction_deg > MAX_TILT_CORRECTION_DEG:
            raise RuntimeError(
                "el mapa inclinó el piso: "
                f"{tilt_correction_deg:.2f}°"
            )
        # La odometría actual es la pose exacta de Isaac. Mientras siga así,
        # una corrección global grande no es deriva real: cuantifica el error
        # que introdujeron el barrido o el ajuste de SLAM.
        if (
            translation_correction > MAX_GROUND_TRUTH_CORRECTION_M
            or yaw_correction_deg > MAX_GROUND_TRUTH_CORRECTION_DEG
        ):
            raise RuntimeError(
                "SLAM contradice la referencia exacta de Isaac: "
                f"{translation_correction:.3f} m / "
                f"{yaw_correction_deg:.2f}°"
            )

        cells = np.asarray(message.data, dtype=np.int16)
        unknown = int(np.count_nonzero(cells < 0))
        free = int(np.count_nonzero((cells >= 0) & (cells < 50)))
        occupied = int(np.count_nonzero(cells >= 50))
        if message.info.width < 20 or message.info.height < 20:
            raise RuntimeError(
                f"mapa demasiado pequeño: {message.info.width}x{message.info.height}"
            )
        if free < 100 or occupied < 20:
            raise RuntimeError(
                f"contenido insuficiente: libres={free}, ocupadas={occupied}"
            )
        print(
            f"mapa: {message.info.width}x{message.info.height} celdas, "
            f"{message.info.resolution:.3f} m por celda"
        )
        print(
            f"contenido: {free} libres, {occupied} ocupadas, "
            f"{unknown} desconocidas"
        )
        print(
            "corrección sobre Isaac: "
            f"{translation_correction:.3f} m / "
            f"{yaw_correction_deg:.2f}°"
        )
        print(
            "coordenadas: map -> odom -> base_footprint -> "
            "base_link -> lidar_link"
        )
        print("PASA: el mapa global y la posición son utilizables")
        return 0
    except RuntimeError as error:
        print(f"FALLA MAPA GLOBAL: {error}")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
