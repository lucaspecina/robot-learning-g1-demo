#!/usr/bin/env python3
"""Mantiene la pose de la base cuando la autoridad está en modo stand.

Este controlador no sostiene las articulaciones ni reemplaza al equilibrio de
la policy. Cierra un lazo más lento sobre la odometría para compensar su deriva
residual. Vive separado de navegación porque quedarse quieto no es una misión
de navegación.
"""

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


POSITION_DEADBAND_M = 0.05
POSITION_GAIN = 2.0
MAX_CORRECTION_MPS = 0.45
TELEPORT_DISTANCE_M = 1.0
RATE_HZ = 10.0


def yaw_from_quaternion(w, x, y, z) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class StandHold(Node):
    def __init__(self):
        super().__init__("stand_hold")
        self.pose = None
        self.anchor = None
        self.is_owner = False

        self.pub_cmd = self.create_publisher(Twist, "/g1/cmd_vel/stand", 10)
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(
            String,
            "/g1/mobility/status",
            self.on_mobility_status,
            10,
        )
        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.get_logger().info("stand_hold listo; espera autoridad de movilidad")

    def on_odom(self, msg: Odometry):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        previous = self.pose
        self.pose = (p.x, p.y, yaw_from_quaternion(o.w, o.x, o.y, o.z))

        if previous is not None:
            jump = math.hypot(p.x - previous[0], p.y - previous[1])
            if jump > TELEPORT_DISTANCE_M:
                self.anchor = None
                self.get_logger().warn(
                    f"salto de odometría de {jump:.2f} m: descarto el anclaje"
                )

        if self.is_owner and self.anchor is None:
            self.anchor = self.pose
            self.get_logger().info(
                f"anclado en ({p.x:.2f}, {p.y:.2f}) para mantener la pose"
            )

    def on_mobility_status(self, msg: String):
        try:
            owner = json.loads(msg.data)["owner"]
        except (json.JSONDecodeError, KeyError, TypeError):
            return

        was_owner = self.is_owner
        self.is_owner = owner == "stand"
        if self.is_owner and not was_owner:
            # El anclaje se toma al recuperar el recurso, no al perderlo. Así
            # una cancelación no hace volver al robot al inicio de la misión.
            self.anchor = self.pose
            if self.anchor is not None:
                self.get_logger().info(
                    f"recuperé movilidad; nuevo anclaje "
                    f"({self.anchor[0]:.2f}, {self.anchor[1]:.2f})"
                )
        elif not self.is_owner and was_owner:
            self.anchor = None

    def tick(self):
        if not self.is_owner or self.pose is None or self.anchor is None:
            return

        x, y, yaw = self.pose
        ax, ay, _ = self.anchor
        dx, dy = ax - x, ay - y
        error = math.hypot(dx, dy)

        cmd = Twist()
        if error >= POSITION_DEADBAND_M:
            speed = min(POSITION_GAIN * error, MAX_CORRECTION_MPS)
            heading = normalize_angle(math.atan2(dy, dx) - yaw)
            cmd.linear.x = speed * math.cos(heading)
            cmd.linear.y = speed * math.sin(heading)
        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = StandHold()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
