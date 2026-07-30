#!/usr/bin/env python3
"""Navegación simple detrás del contrato estándar de Nav2.

La misión usa `/g1/navigate_to_pose` y `/g1/spin`, Actions ROS 2 estándar de
Nav2. Una Action es una tarea larga con objetivo, progreso, resultado y
cancelación. Por ahora este nodo calcula movimiento directo en una habitación
despejada; más adelante Nav2 podrá reemplazarlo sin cambiar al agente.

Canales de control:

  recibe:  /g1/navigate_to_pose       objetivo cancelable de la misión
           /g1/spin                   giro relativo cancelable
           /g1/goal                   compatibilidad temporal con pruebas viejas
           /g1/odom                   posición medida
           /g1/mobility/status        dueño actual del movimiento
  publica: /g1/cmd_vel/navigation     pedido de movimiento al árbitro
           /g1/mobility/request       adquirir o liberar el movimiento
           /g1/navigation/goal        copia observable para el tablero
           /g1/nav_status             compatibilidad y relato humano

El nodo nunca publica en `/cmd_vel`: el árbitro sigue siendo el único dueño de
esa salida. Antes de devolver cualquier éxito, falla o cancelación, publica
cero y entrega el movimiento a `STAND`.

Esta adaptación conserva las interfaces de Nav2, pero todavía no tiene su
mapa local para anticipar colisiones durante un giro. Por eso sólo se usa en
la habitación despejada de la demo; esa garantía llegará con LiDAR + Nav2.
"""

import json
import math
import os
import sys
import threading
import time
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from navigation_core import (
    NavigationController,
    NavigationGoal,
    NavigationPose,
    ProgressChecker,
    SpinController,
    normalize_angle,
)


RATE_HZ = 10.0
CLAIM_RETRY_S = 0.5
ODOM_JUMP_M = 1.0
ODOM_WAIT_TIMEOUT_S = 5.0
AUTHORITY_WAIT_TIMEOUT_S = 10.0
SAFE_STAND_TIMEOUT_S = 3.0

# Estos plazos usan tiempo de pared porque también deben actuar si Isaac se
# pausa. Son guardas iniciales; las tolerancias físicas siguen siendo las que
# ya fueron medidas en las regresiones de navegación.
EXECUTION_TIMEOUT_S = float(os.environ.get("NAV_EXECUTION_TIMEOUT_S", "600"))
PROGRESS_TIMEOUT_S = float(os.environ.get("NAV_PROGRESS_TIMEOUT_S", "30"))
PROGRESS_RADIUS_M = float(os.environ.get("NAV_PROGRESS_RADIUS_M", "0.05"))
PROGRESS_ANGLE_RAD = math.radians(
    float(os.environ.get("NAV_PROGRESS_ANGLE_DEG", "5"))
)


def yaw_from_quaternion(w: float, x: float, y: float, z: float) -> float:
    """Obtiene el giro horizontal de una orientación."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def duration_message(seconds: float):
    """Convierte segundos de pared al mensaje que exige la interfaz estándar."""
    from builtin_interfaces.msg import Duration

    seconds = max(0.0, float(seconds))
    whole = int(seconds)
    return Duration(
        sec=whole,
        nanosec=int((seconds - whole) * 1_000_000_000),
    )


def goal_from_pose(message: PoseStamped) -> NavigationGoal:
    orientation = message.pose.orientation
    norm = math.sqrt(
        orientation.w * orientation.w
        + orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
    )
    yaw = None
    if norm > 0.5:
        yaw = yaw_from_quaternion(
            orientation.w,
            orientation.x,
            orientation.y,
            orientation.z,
        )
    return NavigationGoal(
        x=float(message.pose.position.x),
        y=float(message.pose.position.y),
        yaw=yaw,
    )


class GoTo(Node):
    def __init__(self):
        super().__init__("go_to")
        self.callback_group = ReentrantCallbackGroup()
        self.state_lock = threading.RLock()
        self.pose = None
        self.pose_message = None
        self.mobility_owner = None
        self.last_claim_at = float("-inf")
        self.goal_reserved = False
        self.navigation_active = False
        self.had_authority = False
        self.finishing = False
        self.interrupt_reason = None

        self.pub_cmd = self.create_publisher(
            Twist,
            "/g1/cmd_vel/navigation",
            10,
        )
        self.pub_mobility = self.create_publisher(
            String,
            "/g1/mobility/request",
            10,
        )
        self.pub_status = self.create_publisher(
            String,
            "/g1/nav_status",
            10,
        )
        self.pub_goal = self.create_publisher(
            PoseStamped,
            "/g1/navigation/goal",
            10,
        )
        self.create_subscription(
            Odometry,
            "/g1/odom",
            self.on_odom,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            "/g1/mobility/status",
            self.on_mobility_status,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            PoseStamped,
            "/g1/goal",
            self.on_legacy_goal,
            10,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            "/g1/navigate_to_pose",
            execute_callback=self.execute_action,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel,
            callback_group=self.callback_group,
        )
        self.spin_action_server = ActionServer(
            self,
            Spin,
            "/g1/spin",
            execute_callback=self.execute_spin_action,
            goal_callback=self.handle_spin_goal,
            cancel_callback=self.handle_cancel,
            callback_group=self.callback_group,
        )
        self.get_logger().info(
            "navegación lista en /g1/navigate_to_pose y /g1/spin; "
            "/g1/goal queda sólo para compatibilidad"
        )

    def on_odom(self, message: Odometry):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        new_pose = NavigationPose(
            x=float(position.x),
            y=float(position.y),
            yaw=yaw_from_quaternion(
                orientation.w,
                orientation.x,
                orientation.y,
                orientation.z,
            ),
        )
        with self.state_lock:
            previous = self.pose
            self.pose = new_pose
            observed = PoseStamped()
            observed.header = message.header
            observed.pose = message.pose.pose
            self.pose_message = observed
            if (
                previous is not None
                and math.hypot(
                    new_pose.x - previous.x,
                    new_pose.y - previous.y,
                )
                > ODOM_JUMP_M
                and self.navigation_active
            ):
                self.interrupt_reason = (
                    "salto de odometría durante navegación"
                )

    def on_mobility_status(self, message: String):
        try:
            owner = str(json.loads(message.data)["owner"])
        except (json.JSONDecodeError, KeyError, TypeError):
            return
        with self.state_lock:
            previous = self.mobility_owner
            self.mobility_owner = owner
            if owner == "navigation":
                self.had_authority = True
            elif (
                previous == "navigation"
                and self.navigation_active
                and self.had_authority
                and not self.finishing
            ):
                self.interrupt_reason = (
                    f"se perdió la autoridad; dueño actual: {owner}"
                )

    def handle_goal(self, request: NavigateToPose.Goal):
        pose = request.pose
        if pose.header.frame_id not in ("", "odom"):
            self.get_logger().warn(
                f"objetivo rechazado: marco {pose.header.frame_id!r}; "
                "este navegador simple sólo conoce odom"
            )
            return GoalResponse.REJECT
        if not all(
            math.isfinite(value)
            for value in (pose.pose.position.x, pose.pose.position.y)
        ):
            self.get_logger().warn("objetivo rechazado: coordenadas inválidas")
            return GoalResponse.REJECT
        with self.state_lock:
            if self.goal_reserved or self.navigation_active:
                self.get_logger().warn(
                    "objetivo rechazado: ya existe una navegación activa"
                )
                return GoalResponse.REJECT
            self.goal_reserved = True
        return GoalResponse.ACCEPT

    def handle_cancel(self, _goal_handle):
        self.get_logger().info("cancelación de navegación aceptada")
        return CancelResponse.ACCEPT

    def handle_spin_goal(self, request: Spin.Goal):
        if not math.isfinite(float(request.target_yaw)):
            self.get_logger().warn("giro rechazado: ángulo inválido")
            return GoalResponse.REJECT
        with self.state_lock:
            if self.goal_reserved or self.navigation_active:
                self.get_logger().warn(
                    "giro rechazado: ya existe un movimiento activo"
                )
                return GoalResponse.REJECT
            self.goal_reserved = True
        return GoalResponse.ACCEPT

    def on_legacy_goal(self, message: PoseStamped):
        """Puente transitorio para las herramientas que aún publican un topic."""
        with self.state_lock:
            if self.goal_reserved or self.navigation_active:
                self.publish_status("fallo: ya existe una navegación activa")
                return
            self.goal_reserved = True
        thread = threading.Thread(
            target=self.execute_legacy,
            args=(message,),
            daemon=True,
        )
        thread.start()

    def execute_legacy(self, message: PoseStamped):
        self._run_navigation(message, goal_handle=None)

    def execute_action(self, goal_handle):
        return self._run_navigation(
            goal_handle.request.pose,
            goal_handle=goal_handle,
        )

    def execute_spin_action(self, goal_handle):
        deadline = time.monotonic() + ODOM_WAIT_TIMEOUT_S
        pose = None
        while rclpy.ok() and time.monotonic() < deadline:
            with self.state_lock:
                pose = self.pose
            if pose is not None:
                break
            time.sleep(0.05)
        if pose is None:
            with self.state_lock:
                self.goal_reserved = False
            result = Spin.Result()
            result.error_code = Spin.Result.TF_ERROR
            result.error_msg = "no llegó la posición del robot"
            goal_handle.abort()
            return result

        target_yaw = normalize_angle(
            pose.yaw + float(goal_handle.request.target_yaw)
        )
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "odom"
        target.pose.position.x = pose.x
        target.pose.position.y = pose.y
        target.pose.orientation.z = math.sin(target_yaw / 2.0)
        target.pose.orientation.w = math.cos(target_yaw / 2.0)
        allowance = (
            float(goal_handle.request.time_allowance.sec)
            + float(goal_handle.request.time_allowance.nanosec) / 1e9
        )
        return self._run_navigation(
            target,
            goal_handle=goal_handle,
            action_kind="spin",
            spin_start_yaw=pose.yaw,
            spin_target_yaw=float(goal_handle.request.target_yaw),
            execution_timeout_s=(
                allowance if allowance > 0.0 else EXECUTION_TIMEOUT_S
            ),
        )

    def _run_navigation(
        self,
        goal_message: PoseStamped,
        goal_handle,
        *,
        action_kind: str = "navigate",
        spin_start_yaw: float = None,
        spin_target_yaw: float = None,
        execution_timeout_s: float = EXECUTION_TIMEOUT_S,
    ):
        goal = goal_from_pose(goal_message)
        controller = (
            SpinController(spin_target_yaw)
            if action_kind == "spin"
            else NavigationController()
        )
        progress = ProgressChecker(
            movement_radius_m=PROGRESS_RADIUS_M,
            movement_angle_rad=PROGRESS_ANGLE_RAD,
            allowance_s=PROGRESS_TIMEOUT_S,
        )
        result = (
            Spin.Result()
            if action_kind == "spin"
            else NavigateToPose.Result()
        )
        result.error_code = (
            Spin.Result.NONE
            if action_kind == "spin"
            else NavigateToPose.Result.NONE
        )
        result.error_msg = ""
        started_at = time.monotonic()
        authority_deadline = started_at + AUTHORITY_WAIT_TIMEOUT_S
        odom_deadline = started_at + ODOM_WAIT_TIMEOUT_S

        with self.state_lock:
            self.goal_reserved = False
            self.navigation_active = True
            self.had_authority = False
            self.finishing = False
            self.interrupt_reason = None
        if action_kind == "spin":
            controller.reset(spin_start_yaw)
        else:
            controller.reset()
        progress.reset()
        self.pub_goal.publish(goal_message)
        if action_kind == "spin":
            self.get_logger().info(
                "giro relativo aceptado: "
                f"{math.degrees(spin_target_yaw):.1f}°"
            )
        else:
            yaw_text = (
                "libre"
                if goal.yaw is None
                else f"{math.degrees(goal.yaw):.1f}°"
            )
            self.get_logger().info(
                f"objetivo aceptado: ({goal.x:.2f}, {goal.y:.2f}), "
                f"orientación final {yaw_text}"
            )
        self.publish_status("esperando_control")
        self.claim_mobility(force=True)

        terminal = "failed"
        message = "la navegación terminó sin resultado"
        try:
            while rclpy.ok():
                now = time.monotonic()
                with self.state_lock:
                    pose = self.pose
                    owner = self.mobility_owner
                    interrupt_reason = self.interrupt_reason

                if goal_handle is not None and goal_handle.is_cancel_requested:
                    terminal = "canceled"
                    message = "navegación cancelada por el solicitante"
                    break
                if interrupt_reason is not None:
                    terminal = "failed"
                    message = interrupt_reason
                    break
                if now - started_at > execution_timeout_s:
                    terminal = "timed_out"
                    message = "venció el plazo total de navegación"
                    break
                if pose is None:
                    if now > odom_deadline:
                        terminal = "failed"
                        message = "no llegó la posición del robot"
                        break
                    time.sleep(1.0 / RATE_HZ)
                    continue

                if action_kind == "spin":
                    command = controller.step(pose.yaw)
                    angular_distance_traveled = (
                        command.angular_distance_traveled
                    )
                else:
                    command = controller.step(pose, goal)
                    angular_distance_traveled = 0.0
                self.publish_feedback(
                    goal_handle,
                    command,
                    started_at,
                    action_kind=action_kind,
                    angular_distance_traveled=angular_distance_traveled,
                )

                if owner != "navigation":
                    if now > authority_deadline:
                        terminal = "failed"
                        message = (
                            "no se obtuvo la autoridad de movilidad a tiempo"
                        )
                        break
                    self.claim_mobility()
                    time.sleep(1.0 / RATE_HZ)
                    continue

                if command.goal_reached:
                    terminal = "succeeded"
                    if action_kind == "spin":
                        message = (
                            "giro completado: "
                            f"{math.degrees(angular_distance_traveled):.1f}° "
                            "medidos"
                        )
                    else:
                        message = (
                            f"objetivo alcanzado a "
                            f"{command.distance_remaining:.3f} m"
                        )
                    break

                if not progress.update(pose, now):
                    terminal = "stalled"
                    message = (
                        "navegación sin progreso: no avanzó "
                        f"{PROGRESS_RADIUS_M:.2f} m ni giró "
                        f"{math.degrees(PROGRESS_ANGLE_RAD):.1f}° durante "
                        f"{PROGRESS_TIMEOUT_S:.1f} s"
                    )
                    break

                velocity = Twist()
                velocity.linear.x = (
                    0.0
                    if action_kind == "spin"
                    else command.linear_x
                )
                velocity.angular.z = command.angular_z
                self.pub_cmd.publish(velocity)
                self.publish_status(
                    "girando" if action_kind == "spin" else "moviendo"
                )
                time.sleep(1.0 / RATE_HZ)
        except Exception as error:  # noqa: BLE001
            terminal = "failed"
            message = f"error interno de navegación: {error}"
            self.get_logger().error(message)
        finally:
            safe = self.return_to_safe_state(message)
            with self.state_lock:
                self.navigation_active = False
                self.goal_reserved = False
                self.had_authority = False
                self.finishing = False
                self.interrupt_reason = None

        if not safe and terminal != "canceled":
            terminal = "failed"
            message += "; no se confirmó el regreso a STAND"

        if terminal == "succeeded":
            self.publish_status(
                "giro_completado"
                if action_kind == "spin"
                else "llegue"
            )
            self.get_logger().info(message)
            if goal_handle is not None:
                goal_handle.succeed()
        elif terminal == "canceled":
            self.publish_status("cancelado")
            self.get_logger().warn(message)
            if goal_handle is not None:
                goal_handle.canceled()
        else:
            self.publish_status(f"fallo: {message}")
            self.get_logger().error(message)
            # Jazzy publica `error_msg`, pero su paquete estable todavía no
            # define los códigos TIMEOUT/UNKNOWN agregados después en Nav2.
            # El estado ABORTED y este texto son el contrato portable; inventar
            # números locales haría incompatible al futuro servidor oficial.
            if action_kind == "spin":
                result.error_code = (
                    Spin.Result.TIMEOUT
                    if terminal == "timed_out"
                    else Spin.Result.UNKNOWN
                )
            else:
                result.error_code = NavigateToPose.Result.NONE
            result.error_msg = message
            if goal_handle is not None:
                goal_handle.abort()
        if action_kind == "spin":
            result.total_elapsed_time = duration_message(
                time.monotonic() - started_at
            )
        return result

    def publish_feedback(
        self,
        goal_handle,
        command,
        started_at: float,
        *,
        action_kind: str = "navigate",
        angular_distance_traveled: float = 0.0,
    ):
        if goal_handle is None:
            return
        if action_kind == "spin":
            feedback = Spin.Feedback()
            feedback.angular_distance_traveled = float(
                angular_distance_traveled
            )
            goal_handle.publish_feedback(feedback)
            return
        feedback = NavigateToPose.Feedback()
        with self.state_lock:
            if self.pose_message is not None:
                feedback.current_pose = self.pose_message
        feedback.navigation_time = duration_message(
            time.monotonic() - started_at
        )
        estimate = (
            command.distance_remaining / max(0.01, 0.30)
            + abs(command.heading_error) / max(0.01, 0.50)
        )
        feedback.estimated_time_remaining = duration_message(estimate)
        feedback.number_of_recoveries = 0
        feedback.distance_remaining = float(command.distance_remaining)
        if hasattr(feedback, "position_tracking_error"):
            feedback.position_tracking_error = float(
                command.distance_remaining
            )
        if hasattr(feedback, "heading_tracking_error"):
            feedback.heading_tracking_error = float(command.heading_error)
        goal_handle.publish_feedback(feedback)

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
        self.pub_mobility.publish(
            String(data=json.dumps(request, ensure_ascii=False))
        )

    def claim_mobility(self, force: bool = False):
        now = time.monotonic()
        if force or now - self.last_claim_at >= CLAIM_RETRY_S:
            self.request_mobility("acquire")
            self.last_claim_at = now

    def stop(self):
        self.pub_cmd.publish(Twist())

    def return_to_safe_state(self, reason: str) -> bool:
        """Publica cero, libera la movilidad y espera confirmación de STAND."""
        with self.state_lock:
            self.finishing = True
            owner = self.mobility_owner
        self.stop()
        if owner == "navigation":
            self.request_mobility("release", reason)
        elif owner not in (None, "stand"):
            # Si el operador tomó el control, no debemos arrebatárselo para
            # satisfacer artificialmente una poscondición de la autonomía.
            return True

        deadline = time.monotonic() + SAFE_STAND_TIMEOUT_S
        while rclpy.ok() and time.monotonic() < deadline:
            self.stop()
            with self.state_lock:
                if self.mobility_owner == "stand":
                    return True
            time.sleep(0.05)
        with self.state_lock:
            return self.mobility_owner == "stand"


def main():
    rclpy.init()
    node = GoTo()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
            node.request_mobility(
                "release",
                "nodo de navegación detenido",
            )
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
