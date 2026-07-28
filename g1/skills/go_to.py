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
avanza mientras corrige el heading. Es navegacion "a la vista", sin mapa ni
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
TOLERANCE_M = 0.35        # a esta distance del objetivo damos por llegado
MAX_DRIFT_M = 0.25        # si se aleja mas que esto del punto donde debe estar,
                           # lo traemos de vuelta (ver "mantener posicion")
TOLERANCE_RAD = 0.25      # error de heading tolerado antes de empezar a avanzar
LINEAR_VEL = 0.3           # m/s de crucero
ANGULAR_VEL = 0.5          # rad/s al girar
RATE_HZ = 10.0            # cada cuanto recalculamos y publicamos


def yaw_from_quaternion(w, x, y, z) -> float:
    """Angulo de giro sobre el eje vertical, en radianes."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_angle(angle: float) -> float:
    """Lleva un angle al rango [-pi, pi] — asi el robot gira para el lado corto."""
    return math.atan2(math.sin(angle), math.cos(angle))


class GoTo(Node):
    def __init__(self):
        super().__init__("go_to")
        self.pose = None            # (x, y, yaw) del robot
        self.goal = None            # (x, y) objetivo
        self.anchor = None         # donde tiene que quedarse cuando no hay objetivo

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_status = self.create_publisher(String, "/g1/nav_status", 10)
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(PoseStamped, "/g1/goal", self.on_goal, 10)
        self.create_timer(1.0 / RATE_HZ, self.tick)

        self.get_logger().info("go_to listo. Esperando objetivos en /g1/goal")

    def on_odom(self, msg: Odometry):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(o.w, o.x, o.y, o.z))
        # Anclarse donde nace: la policy con comando cero deriva varios cm/s,
        # y sin anclaje inicial el robot se va caminando solo hasta que alguien
        # le da el primer objetivo. Desde el primer dato de odometria, su
        # lugar es donde esta parado.
        if self.anchor is None:
            self.anchor = (p.x, p.y)
            self.get_logger().info(
                f"anclado al nacer en ({p.x:.2f}, {p.y:.2f}): "
                f"si deriva mas de {MAX_DRIFT_M} m, vuelve solo")

    def on_goal(self, msg: PoseStamped):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"objetivo nuevo: ({self.goal[0]:.2f}, {self.goal[1]:.2f})")
        self.publish_status("moviendo")

    def publish_status(self, text: str):
        self.pub_status.publish(String(data=text))

    def stop(self):
        self.pub_cmd.publish(Twist())

    def hold_position(self):
        """Corrige la deriva: si se alejo del anchor, lo trae de vuelta."""
        x, y, yaw = self.pose
        ax, ay = self.anchor
        dx, dy = ax - x, ay - y
        error = math.hypot(dx, dy)

        if error < MAX_DRIFT_M:
            self.stop()
            return

        # Correccion suave: caminar despacio hacia el punto, sin girar el cuerpo
        # (el robot puede desplazarse de costado, asi no pierde su orientacion).
        heading = normalize_angle(math.atan2(dy, dx) - yaw)
        cmd = Twist()
        cmd.linear.x = 0.15 * math.cos(heading)
        cmd.linear.y = 0.15 * math.sin(heading)
        self.pub_cmd.publish(cmd)

    def tick(self):
        """Un ciclo de navegacion: mirar donde estoy, decidir, mandar velocidad."""
        if self.pose is None:
            return

        # Sin objetivo: mantener la posicion. La policy de locomocion, con
        # comando cero, no queda perfectamente quieta — deriva unos centimetros
        # por segundo porque sigue su ciclo de paso y los pies patinan. Ninguna
        # policy es perfecta en esto, y el robot real tampoco lo es. La solucion
        # no es pedirle mas a la locomocion sino ponerle realimentacion de
        # posicion encima: si se corrio, se le ordena volver.
        if self.goal is None:
            if self.anchor is not None:
                self.hold_position()
            return

        x, y, yaw = self.pose
        gx, gy = self.goal
        dx, dy = gx - x, gy - y
        distance = math.hypot(dx, dy)

        # ¿Llegamos?
        if distance < TOLERANCE_M:
            self.stop()
            self.get_logger().info(f"llegue: quede a {distance:.2f} m del objetivo")
            self.publish_status("llegue")
            self.goal = None
            self.anchor = (x, y)     # de aca en mas, quedarse en este punto
            return

        # Error de heading: cuanto tengo que girar para apuntar al objetivo.
        heading_error = normalize_angle(math.atan2(dy, dx) - yaw)

        cmd = Twist()
        if abs(heading_error) > TOLERANCE_RAD:
            # Muy desalineado: girar en el lugar antes de avanzar. Caminar de
            # costado hacia el objetivo seria mas rapido, pero girar primero es
            # mas estable en un bipedo y mas facil de mirar.
            cmd.angular.z = ANGULAR_VEL * (1.0 if heading_error > 0 else -1.0)
        else:
            # Alineado: avanzar corrigiendo el heading sobre la marcha. Cerca del
            # objetivo, bajar la velocidad para no pasarse de largo.
            cmd.linear.x = LINEAR_VEL * min(1.0, distance / 1.0)
            cmd.angular.z = 1.0 * heading_error

        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = GoTo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
