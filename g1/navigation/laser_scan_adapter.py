#!/usr/bin/env python3
"""Corrige el intervalo angular inconsistente del LaserScan de Isaac 5.1."""
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from laser_scan_core import angle_max_for_count, ray_count_from_metadata  # noqa: E402


RAW_TOPIC = "/scan_raw"
OUTPUT_TOPIC = "/scan"


class LaserScanAdapter(Node):
    def __init__(self):
        super().__init__("laser_scan_adapter")
        self.corrections = 0
        self.publisher = self.create_publisher(
            LaserScan,
            OUTPUT_TOPIC,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            RAW_TOPIC,
            self.on_scan,
            qos_profile_sensor_data,
        )

    def on_scan(self, message: LaserScan):
        actual_count = len(message.ranges)
        declared_count = ray_count_from_metadata(
            message.angle_min,
            message.angle_max,
            message.angle_increment,
        )
        if declared_count != actual_count:
            # Example_Rotary_2D declara 32000/30 rayos por vuelta, una división
            # no entera. Isaac publica 1066 valores pero redondea el extremo
            # como si fueran 1067; Karto rechaza toda la medición por seguridad.
            message.angle_max = angle_max_for_count(
                message.angle_min,
                message.angle_increment,
                actual_count,
            )
            self.corrections += 1
            if self.corrections == 1 or self.corrections % 100 == 0:
                self.get_logger().warning(
                    "intervalo angular corregido: "
                    f"{declared_count} declarados, {actual_count} recibidos"
                )
        self.publisher.publish(message)


def main() -> int:
    rclpy.init()
    node = LaserScanAdapter()
    try:
        rclpy.spin(node)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
