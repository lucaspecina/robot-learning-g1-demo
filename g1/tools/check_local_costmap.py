#!/usr/bin/env python3
"""Verifica que el mapa local siga al robot y contenga obstáculos reales."""

import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


COSTMAP_TOPIC = "/local_costmap/costmap"
ODOM_TOPIC = "/g1/odom"
EXPECTED_FRAME = "odom"
EXPECTED_RESOLUTION_M = 0.05
EXPECTED_SIZE_M = 3.0
TIMEOUT_S = 25.0
MIN_MAPS = 3
MIN_LETHAL_CELLS = 2
MIN_INFLATED_CELLS = 10


class LocalCostmapChecker(Node):
    def __init__(self):
        super().__init__("check_local_costmap")
        self.pose = None
        self.maps = []
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid,
            COSTMAP_TOPIC,
            self.on_costmap,
            map_qos,
        )
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)

    def on_odom(self, message: Odometry):
        self.pose = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )

    def on_costmap(self, message: OccupancyGrid):
        if self.pose is None:
            return
        data = np.asarray(message.data, dtype=np.int16)
        width = int(message.info.width)
        height = int(message.info.height)
        if data.size != width * height:
            return
        center = (
            float(message.info.origin.position.x)
            + width * float(message.info.resolution) / 2.0,
            float(message.info.origin.position.y)
            + height * float(message.info.resolution) / 2.0,
        )
        center_index = (height // 2) * width + width // 2
        self.maps.append(
            {
                "frame": message.header.frame_id,
                "stamp": (
                    message.header.stamp.sec,
                    message.header.stamp.nanosec,
                ),
                "resolution": float(message.info.resolution),
                "width": width,
                "height": height,
                "center_error": math.hypot(
                    center[0] - self.pose[0],
                    center[1] - self.pose[1],
                ),
                "lethal": int(np.count_nonzero(data >= 99)),
                "inflated": int(np.count_nonzero((data > 0) & (data < 99))),
                "unknown": int(np.count_nonzero(data < 0)),
                "center_cost": int(data[center_index]),
            }
        )


def main() -> int:
    rclpy.init()
    node = LocalCostmapChecker()
    deadline = time.monotonic() + TIMEOUT_S
    try:
        while len(node.maps) < MIN_MAPS and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.maps) < MIN_MAPS:
            raise RuntimeError(
                f"llegaron {len(node.maps)}/{MIN_MAPS} mapas locales"
            )
        if any(sample["frame"] != EXPECTED_FRAME for sample in node.maps):
            raise RuntimeError("el mapa local no está expresado en odom")
        if len({sample["stamp"] for sample in node.maps}) != len(node.maps):
            raise RuntimeError("el mapa local repitió su hora")
        if any(
            not math.isclose(
                sample["resolution"],
                EXPECTED_RESOLUTION_M,
                abs_tol=1e-6,
            )
            for sample in node.maps
        ):
            raise RuntimeError("la resolución no es 5 cm por celda")
        expected_cells = round(EXPECTED_SIZE_M / EXPECTED_RESOLUTION_M)
        if any(
            sample["width"] != expected_cells
            or sample["height"] != expected_cells
            for sample in node.maps
        ):
            raise RuntimeError("la ventana local no mide 3 x 3 m")
        if max(sample["center_error"] for sample in node.maps) > 0.10:
            raise RuntimeError("la ventana no permanece centrada en el robot")
        if min(sample["lethal"] for sample in node.maps) < MIN_LETHAL_CELLS:
            raise RuntimeError("el LiDAR no marcó ninguna pared como letal")
        if min(sample["inflated"] for sample in node.maps) < MIN_INFLATED_CELLS:
            raise RuntimeError("falta el margen inflado alrededor de obstáculos")
        if any(sample["unknown"] for sample in node.maps):
            raise RuntimeError("el mapa local contiene huecos desconocidos")
        if any(sample["center_cost"] != 0 for sample in node.maps):
            raise RuntimeError("la propia huella del robot quedó ocupada")
        last = node.maps[-1]
        print(
            f"{len(node.maps)} mapas de {last['width']} x {last['height']} "
            f"a {last['resolution']:.2f} m/celda"
        )
        print(
            f"obstáculos: {last['lethal']} celdas letales y "
            f"{last['inflated']} de margen; centro libre"
        )
        print(
            "PASA: el mapa local está centrado en el robot y representa el LiDAR "
            "sin marcar al propio cuerpo"
        )
        return 0
    except RuntimeError as error:
        print(f"FALLA MAPA LOCAL: {error}")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
