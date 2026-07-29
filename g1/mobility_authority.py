#!/usr/bin/env python3
"""Único árbitro entre las fuentes de velocidad y el robot.

Entradas separadas:

  /g1/cmd_vel/stand
  /g1/cmd_vel/navigation
  /g1/cmd_vel/manual
  /g1/cmd_vel/test

Control:

  /g1/mobility/request
    {"operation":"acquire","source":"navigation","requester":"go_to"}
    {"operation":"release","source":"navigation","requester":"go_to",
     "reason":"objetivo terminado"}

Salida:

  /cmd_vel
  /g1/mobility/status

Mientras no exista un filtro de colisiones, este nodo es deliberadamente el
único publicador del topic final. Cuando se agregue ese filtro, la salida del
árbitro se remapeará a un topic intermedio.
"""

import json
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mobility_core import MobilityAuthority, MobilitySource  # noqa: E402


OUTPUT_RATE_HZ = 20.0
STATUS_RATE_HZ = 10.0
LEASE_TIMEOUT_S = 0.75
COMMAND_TIMEOUT_S = 0.35


def velocity_from_twist(msg: Twist):
    return (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))


def twist_from_velocity(velocity):
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = velocity
    return msg


class MobilityAuthorityNode(Node):
    def __init__(self):
        super().__init__("mobility_authority")
        self.authority = MobilityAuthority(
            lease_timeout_s=LEASE_TIMEOUT_S,
            command_timeout_s=COMMAND_TIMEOUT_S,
        )
        self.last_status_publish = 0.0
        self.last_transition_count = -1

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_status = self.create_publisher(String, "/g1/mobility/status", 10)

        for source in MobilitySource:
            self.create_subscription(
                Twist,
                f"/g1/cmd_vel/{source.value}",
                lambda msg, selected=source: self.on_command(selected, msg),
                10,
            )
        self.create_subscription(
            String,
            "/g1/mobility/request",
            self.on_request,
            10,
        )
        self.create_timer(1.0 / OUTPUT_RATE_HZ, self.tick)
        self.get_logger().info(
            "autoridad de movilidad lista: dueño inicial stand_hold"
        )

    def on_command(self, source: MobilitySource, msg: Twist):
        now = time.monotonic()
        self.authority.submit_command(source, velocity_from_twist(msg), now)

    def on_request(self, msg: String):
        try:
            request = json.loads(msg.data)
            operation = str(request["operation"]).lower()
            source = MobilitySource(str(request["source"]).lower())
            requester = str(request["requester"]).strip()
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f"pedido de movilidad inválido: {exc}")
            return

        if operation == "acquire":
            result = self.authority.acquire(source, requester, time.monotonic())
        elif operation == "release":
            reason = str(request.get("reason", "liberación solicitada"))
            result = self.authority.release(source, requester, reason)
        else:
            self.get_logger().error(
                f"operación de movilidad desconocida: {operation}"
            )
            return

        # rclpy identifica el punto de llamada del log y no permite cambiar su
        # severidad entre invocaciones. Dos ramas explícitas evitan que un
        # rechazo posterior reutilice como INFO el mismo punto de un éxito.
        if result.accepted:
            self.get_logger().info(result.reason)
        else:
            self.get_logger().warn(result.reason)
        self.publish_status(time.monotonic())

    def tick(self):
        now = time.monotonic()
        selected = self.authority.tick(now)
        self.pub_cmd.publish(twist_from_velocity(selected))

        transition_changed = (
            self.authority.transition_count != self.last_transition_count
        )
        if transition_changed or now - self.last_status_publish >= 1.0 / STATUS_RATE_HZ:
            self.publish_status(now)

    def publish_status(self, now: float):
        status = self.authority.status(now)
        self.pub_status.publish(
            String(data=json.dumps(status, separators=(",", ":")))
        )
        self.last_status_publish = now
        self.last_transition_count = self.authority.transition_count


def main():
    rclpy.init()
    node = MobilityAuthorityNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Publicar cero antes de salir reduce la ventana hasta que actúe el
        # watchdog del robot. Si ROS ya cerró, publicar sólo genera una traza
        # engañosa y el watchdog sigue siendo la defensa definitiva.
        if rclpy.ok():
            node.pub_cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
