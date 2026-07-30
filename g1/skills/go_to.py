#!/usr/bin/env python3
"""Skill go_to: llevar el robot a un punto del mapa.

Es la primera skill de verdad del sistema, y el ejemplo del patron que van a
seguir todas: recibe un pedido de alto nivel ("anda a este punto"), lo cumple
usando las capas de abajo, y reporta si lo logro.

Corre en la Jetson (fuera del robot), y solo habla los canales publicos:

  recibe:  /g1/goal        (geometry_msgs/PoseStamped — a donde ir)
           /g1/odom        (nav_msgs/Odometry — donde esta)
           /g1/mobility/status (quien tiene permiso para mover la base)
  publica: /g1/cmd_vel/navigation (geometry_msgs/Twist — como moverse)
           /g1/mobility/request (adquirir o liberar la movilidad)
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
import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# --- parametros de navegacion ---
# Nav2 usa 10 cm como tolerancia fina y reserva 25 cm para una salida
# adaptativa cuando el robot deja de progresar. Primero medimos la condición
# fina sola; 35 cm declaraba éxito demasiado lejos para nuestra demo.
TOLERANCE_M = 0.10        # a esta distancia del objetivo damos por llegado
# Nav2 conserva el logro de posición mientras termina la orientación. Sin ese
# margen, el balanceo del bípedo cruza 10 cm y alterna eternamente entre
# corregir posición y corregir rumbo.
POSITION_LATCH_BUFFER_M = 0.10
FINAL_YAW_TOLERANCE_RAD = math.radians(5.0)
FINAL_YAW_GAIN = 1.5
MIN_FINAL_ANGULAR_VEL = 0.10
SALTO_M = 1.0              # un salto mayor a esto entre dos lecturas solo puede
                           # ser un teletransporte (reinicio o freeze), no un paso
TOLERANCE_RAD = 0.25      # error de heading tolerado antes de empezar a avanzar
LINEAR_VEL = 0.3           # m/s de crucero
MIN_APPROACH_VEL = 0.10    # por debajo, la medición mostró que no progresa
ANGULAR_VEL = 0.5          # rad/s al girar
RATE_HZ = 10.0            # cada cuanto recalculamos y publicamos
CLAIM_RETRY_S = 0.5       # si otro dueño se retira, volver a pedir sin inundar ROS


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
        self.goal = None            # (x, y, yaw opcional) objetivo
        self.position_reached = False
        self.mobility_owner = None
        self.last_claim_at = float("-inf")

        self.pub_cmd = self.create_publisher(
            Twist, "/g1/cmd_vel/navigation", 10
        )
        self.pub_mobility = self.create_publisher(
            String, "/g1/mobility/request", 10
        )
        self.pub_status = self.create_publisher(String, "/g1/nav_status", 10)
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(PoseStamped, "/g1/goal", self.on_goal, 10)
        self.create_subscription(
            String,
            "/g1/mobility/status",
            self.on_mobility_status,
            10,
        )
        self.create_timer(1.0 / RATE_HZ, self.tick)

        self.get_logger().info("go_to listo. Esperando objetivos en /g1/goal")

    def on_odom(self, msg: Odometry):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        anterior = self.pose
        self.pose = (p.x, p.y, yaw_from_quaternion(o.w, o.x, o.y, o.z))

        # Detectar que el robot se TELETRANSPORTO: un reinicio o un freeze lo
        # devuelven al origen de un salto. Caminando nunca se mueve un metro
        # entre dos lecturas consecutivas, asi que un salto asi solo puede ser
        # eso. Cuando pasa hay que olvidar todo: el objetivo y el anclaje son
        # de un episodio que ya no existe. (Sin esto, la navegacion arrastraba
        # un anclaje viejo y hacia caminar al robot recien nacido hasta la
        # posicion donde habia estado la corrida anterior.)
        if anterior is not None:
            if math.hypot(p.x - anterior[0], p.y - anterior[1]) > SALTO_M:
                had_goal = self.goal is not None
                self.goal = None
                self.position_reached = False
                self.get_logger().warn(
                    f"el robot se teletransporto a ({p.x:.2f}, {p.y:.2f}): "
                    "cancelo el objetivo")
                if had_goal:
                    self.stop()
                    self.publish_status("cancelado")
                    self.request_mobility(
                        "release", "salto de odometría durante navegación"
                    )
                return

    def on_mobility_status(self, msg: String):
        try:
            owner = json.loads(msg.data)["owner"]
        except (json.JSONDecodeError, KeyError, TypeError):
            return
        previous = self.mobility_owner
        self.mobility_owner = owner
        if owner == "navigation" and previous != "navigation":
            self.get_logger().info("autoridad de navegación concedida")
            if self.goal is not None:
                self.publish_status("moviendo")
        elif previous == "navigation" and owner != "navigation":
            self.get_logger().warn(
                f"perdí la autoridad de navegación; dueño actual: {owner}"
            )
            if self.goal is not None:
                # Una orden manual o una falla de la concesión invalida el
                # objetivo vigente. Reanudarlo al liberar el joystick haría
                # que una intención vieja mueva el robot sin una orden nueva.
                self.goal = None
                self.position_reached = False
                self.publish_status("cancelado")

    def on_goal(self, msg: PoseStamped):
        orientation = msg.pose.orientation
        quaternion_norm = math.sqrt(
            orientation.w * orientation.w
            + orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        goal_yaw = None
        if quaternion_norm > 0.5:
            goal_yaw = yaw_from_quaternion(
                orientation.w,
                orientation.x,
                orientation.y,
                orientation.z,
            )
        self.goal = (msg.pose.position.x, msg.pose.position.y, goal_yaw)
        self.position_reached = False
        yaw_text = "libre" if goal_yaw is None else f"{math.degrees(goal_yaw):.1f}°"
        self.get_logger().info(
            f"objetivo nuevo: ({self.goal[0]:.2f}, {self.goal[1]:.2f}), "
            f"orientación final {yaw_text}"
        )
        self.publish_status("esperando_control")
        self.claim_mobility(force=True)

    def publish_status(self, text: str):
        self.pub_status.publish(String(data=text))

    def request_mobility(self, operation: str, reason: str = None):
        request = {
            "operation": operation,
            "source": "navigation",
            "requester": "go_to",
        }
        if reason is not None:
            request["reason"] = reason
        self.pub_mobility.publish(String(data=json.dumps(request)))

    def claim_mobility(self, force: bool = False):
        now = time.monotonic()
        if force or now - self.last_claim_at >= CLAIM_RETRY_S:
            self.request_mobility("acquire")
            self.last_claim_at = now

    def stop(self):
        self.pub_cmd.publish(Twist())

    def tick(self):
        """Un ciclo de navegacion: mirar donde estoy, decidir, mandar velocidad."""
        if self.pose is None or self.goal is None:
            return

        # La navegación puede calcular todo lo que quiera, pero no publica
        # movimiento hasta que el árbitro confirme que posee la movilidad.
        if self.mobility_owner != "navigation":
            self.claim_mobility()
            return

        x, y, yaw = self.pose
        gx, gy, goal_yaw = self.goal
        dx, dy = gx - x, gy - y
        distance = math.hypot(dx, dy)

        # Llegar a un sensor o una mesa incluye terminar mirando hacia ellos.
        # Un objetivo sin cuaternión conserva el comportamiento anterior y
        # acepta cualquier orientación final.
        if distance < TOLERANCE_M:
            self.position_reached = True
        elif (
            self.position_reached
            and distance > TOLERANCE_M + POSITION_LATCH_BUFFER_M
        ):
            self.position_reached = False

        if self.position_reached:
            if goal_yaw is not None:
                final_yaw_error = normalize_angle(goal_yaw - yaw)
                if abs(final_yaw_error) > FINAL_YAW_TOLERANCE_RAD:
                    cmd = Twist()
                    # A velocidad fija el G1 cruzaba el objetivo y volvía a
                    # cruzarlo indefinidamente. Reducir la orden junto con el
                    # error conserva el giro rápido lejos y permite cerrar los
                    # últimos grados sin rebotar de un lado al otro.
                    requested_speed = max(
                        MIN_FINAL_ANGULAR_VEL,
                        min(
                            ANGULAR_VEL,
                            FINAL_YAW_GAIN * abs(final_yaw_error),
                        ),
                    )
                    cmd.angular.z = math.copysign(
                        requested_speed,
                        final_yaw_error,
                    )
                    self.pub_cmd.publish(cmd)
                    return

            self.stop()
            self.get_logger().info(
                f"llegue: quedé a {distance:.2f} m del objetivo"
            )
            self.publish_status("llegue")
            self.goal = None
            self.position_reached = False
            self.request_mobility("release", "objetivo de navegación terminado")
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
            # Con 14 cm restantes la fórmula anterior pidió 0,04 m/s y la
            # posición no cambió durante más de un minuto. Nav2 usa una
            # velocidad mínima de aproximación; 0,10 m/s es el primer valor
            # medido para este bípedo, no un ajuste copiado de otro robot.
            cmd.linear.x = max(
                MIN_APPROACH_VEL,
                LINEAR_VEL * min(1.0, distance / 1.0),
            )
            cmd.angular.z = 1.0 * heading_error

        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = GoTo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ExternalShutdownException significa que el contexto ya murió;
        # publicar en ese punto sólo genera una traza roja engañosa.
        if rclpy.ok():
            node.stop()
            node.request_mobility("release", "nodo de navegación detenido")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
