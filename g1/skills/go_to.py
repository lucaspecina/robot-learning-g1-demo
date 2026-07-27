#!/usr/bin/env python3
"""Skill go_to: llevar el robot a un punto del mapa.

Es la primera skill de verdad del sistema, y el ejemplo del patron que van a
seguir todas: recibe un pedido de alto nivel ("anda a este punto"), lo cumple
usando las capas de abajo, y reporta si lo logro.

Corre en la Jetson (fuera del robot), y solo habla los canales publicos:

  recibe:  /g1/goal        (geometry_msgs/PoseStamped — a donde ir)
           /g1/odom        (nav_msgs/Odometry — donde esta)
  publica: /cmd_vel        (geometry_msgs/Twist — como moverse)
           /g1/nav_status  (std_msgs/String — moviendo | llegue | cancelado)

Como decide: primero se orienta hacia el objetivo girando en el lugar, despues
avanza mientras corrige el rumbo. Es navegacion "a la vista", sin mapa ni
esquive de obstaculos — suficiente para una habitacion despejada. Cuando haga
falta esquivar, este nodo se reemplaza por Nav2 hablando los mismos canales, y
el resto del sistema no se entera.

Uso (dentro del contenedor jetson):
    python3 go_to.py
    # y desde otra terminal:
    ros2 topic pub --once /g1/goal geometry_msgs/msg/PoseStamped \
        "{pose: {position: {x: 2.0, y: 1.0}}}"
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# --- parametros de navegacion ---
TOLERANCIA_M = 0.35        # a esta distancia del objetivo damos por llegado
TOLERANCIA_RAD = 0.25      # error de rumbo tolerado antes de empezar a avanzar
VEL_LINEAL = 0.3           # m/s de crucero
VEL_ANGULAR = 0.5          # rad/s al girar
RITMO_HZ = 10.0            # cada cuanto recalculamos y publicamos


def yaw_desde_quaternion(w, x, y, z) -> float:
    """Angulo de giro sobre el eje vertical, en radianes."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalizar(angulo: float) -> float:
    """Lleva un angulo al rango [-pi, pi] — asi el robot gira para el lado corto."""
    return math.atan2(math.sin(angulo), math.cos(angulo))


class GoTo(Node):
    def __init__(self):
        super().__init__("go_to")
        self.pose = None            # (x, y, yaw) del robot
        self.goal = None            # (x, y) objetivo

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_status = self.create_publisher(String, "/g1/nav_status", 10)
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(PoseStamped, "/g1/goal", self.on_goal, 10)
        self.create_timer(1.0 / RITMO_HZ, self.tick)

        self.get_logger().info("go_to listo. Esperando objetivos en /g1/goal")

    def on_odom(self, msg: Odometry):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_desde_quaternion(o.w, o.x, o.y, o.z))

    def on_goal(self, msg: PoseStamped):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"objetivo nuevo: ({self.goal[0]:.2f}, {self.goal[1]:.2f})")
        self.publicar_estado("moviendo")

    def publicar_estado(self, texto: str):
        self.pub_status.publish(String(data=texto))

    def frenar(self):
        self.pub_cmd.publish(Twist())

    def tick(self):
        """Un ciclo de navegacion: mirar donde estoy, decidir, mandar velocidad."""
        if self.goal is None or self.pose is None:
            return

        x, y, yaw = self.pose
        gx, gy = self.goal
        dx, dy = gx - x, gy - y
        distancia = math.hypot(dx, dy)

        # ¿Llegamos?
        if distancia < TOLERANCIA_M:
            self.frenar()
            self.get_logger().info(f"llegue: quede a {distancia:.2f} m del objetivo")
            self.publicar_estado("llegue")
            self.goal = None
            return

        # Error de rumbo: cuanto tengo que girar para apuntar al objetivo.
        error_rumbo = normalizar(math.atan2(dy, dx) - yaw)

        cmd = Twist()
        if abs(error_rumbo) > TOLERANCIA_RAD:
            # Muy desalineado: girar en el lugar antes de avanzar. Caminar de
            # costado hacia el objetivo seria mas rapido, pero girar primero es
            # mas estable en un bipedo y mas facil de mirar.
            cmd.angular.z = VEL_ANGULAR * (1.0 if error_rumbo > 0 else -1.0)
        else:
            # Alineado: avanzar corrigiendo el rumbo sobre la marcha. Cerca del
            # objetivo, bajar la velocidad para no pasarse de largo.
            cmd.linear.x = VEL_LINEAL * min(1.0, distancia / 1.0)
            cmd.angular.z = 1.0 * error_rumbo

        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = GoTo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.frenar()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
