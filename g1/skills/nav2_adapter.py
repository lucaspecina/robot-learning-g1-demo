#!/usr/bin/env python3
"""Conserva el contrato de la demo y delega el movimiento completo a Nav2.

El agente sigue usando `/g1/navigate_to_pose` y `/g1/spin`. Este adaptador
adquiere la autoridad exclusiva, reenvía el objetivo a Nav2 y entrega el
control a STAND ante éxito, falla o cancelación. No calcula trayectorias ni
publica velocidades: esas responsabilidades pertenecen a Nav2.
"""

import json
import math
import threading
import time

from action_msgs.msg import GoalStatus
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


AUTHORITY_WAIT_S = 10.0
SERVER_WAIT_S = 20.0
INNER_RESPONSE_WAIT_S = 20.0
SAFE_STAND_WAIT_S = 3.0
CLAIM_PERIOD_S = 0.25
REQUESTER = "nav2_adapter"


class Nav2Adapter(Node):
    def __init__(self):
        super().__init__("nav2_adapter")
        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.reserved = False
        self.active = False
        self.mobility_owner = None
        self.last_claim_at = float("-inf")
        self.pub_mobility = self.create_publisher(
            String,
            "/g1/mobility/request",
            10,
        )
        self.pub_status = self.create_publisher(String, "/g1/nav_status", 10)
        self.pub_goal = self.create_publisher(
            PoseStamped,
            "/g1/navigation/goal",
            10,
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
        self.nav2_navigation = ActionClient(
            self,
            NavigateToPose,
            "/nav2/navigate_to_pose",
            callback_group=self.callback_group,
        )
        self.nav2_spin = ActionClient(
            self,
            Spin,
            "/nav2/spin",
            callback_group=self.callback_group,
        )
        self.legacy_navigation = ActionClient(
            self,
            NavigateToPose,
            "/g1/navigate_to_pose",
            callback_group=self.callback_group,
        )
        self.navigation_server = ActionServer(
            self,
            NavigateToPose,
            "/g1/navigate_to_pose",
            execute_callback=self.execute_navigation,
            goal_callback=self.handle_navigation_goal,
            cancel_callback=self.handle_cancel,
            callback_group=self.callback_group,
        )
        self.spin_server = ActionServer(
            self,
            Spin,
            "/g1/spin",
            execute_callback=self.execute_spin,
            goal_callback=self.handle_spin_goal,
            cancel_callback=self.handle_cancel,
            callback_group=self.callback_group,
        )
        self.get_logger().info(
            "adaptador listo: la misión usa /g1 y Nav2 ejecuta en /nav2"
        )

    def on_mobility_status(self, message: String):
        try:
            owner = str(json.loads(message.data)["owner"])
        except (json.JSONDecodeError, KeyError, TypeError):
            return
        with self.lock:
            self.mobility_owner = owner

    @staticmethod
    def navigation_error(request):
        pose = request.pose
        if pose.header.frame_id != "map":
            return f"el objetivo usa {pose.header.frame_id!r} y debe usar 'map'"
        values = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return "el objetivo contiene valores no finitos"
        return None

    def reserve(self, error):
        if error is not None:
            self.get_logger().warn(f"objetivo rechazado: {error}")
            return GoalResponse.REJECT
        with self.lock:
            if self.reserved or self.active:
                self.get_logger().warn(
                    "objetivo rechazado: ya hay un movimiento activo"
                )
                return GoalResponse.REJECT
            self.reserved = True
        return GoalResponse.ACCEPT

    def handle_navigation_goal(self, request):
        return self.reserve(self.navigation_error(request))

    def handle_spin_goal(self, request):
        error = None
        if not math.isfinite(float(request.target_yaw)):
            error = "el giro no es finito"
        return self.reserve(error)

    def handle_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def on_legacy_goal(self, pose: PoseStamped):
        """Hace pasar las pruebas antiguas por la misma Action real."""
        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.legacy_navigation.send_goal_async(goal)
        future.add_done_callback(self.on_legacy_response)

    def on_legacy_response(self, future):
        try:
            if not future.result().accepted:
                self.publish_status("fallo: objetivo heredado rechazado")
        except Exception as error:  # noqa: BLE001
            self.publish_status(f"fallo: no se pudo enviar el objetivo: {error}")

    def execute_navigation(self, goal_handle):
        self.pub_goal.publish(goal_handle.request.pose)
        return self.execute_proxy(
            goal_handle,
            self.nav2_navigation,
            success_status="llegue",
            moving_status="moviendo",
        )

    def execute_spin(self, goal_handle):
        return self.execute_proxy(
            goal_handle,
            self.nav2_spin,
            success_status="giro_completado",
            moving_status="girando",
        )

    def execute_proxy(
        self,
        outer_goal,
        inner_client,
        *,
        success_status,
        moving_status,
    ):
        with self.lock:
            self.reserved = False
            self.active = True
        inner_goal = None
        try:
            self.publish_status("esperando_control")
            if not self.wait_for_authority(outer_goal):
                return self.finish_without_inner(
                    outer_goal,
                    "no se obtuvo la autoridad de movilidad",
                )
            if not inner_client.wait_for_server(timeout_sec=SERVER_WAIT_S):
                return self.finish_without_inner(
                    outer_goal,
                    "Nav2 no publicó su servidor de acciones",
                )

            feedback_callback = lambda message: outer_goal.publish_feedback(
                message.feedback
            )
            send_future = inner_client.send_goal_async(
                outer_goal.request,
                feedback_callback=feedback_callback,
            )
            state = self.wait_future(send_future, outer_goal)
            if state != "done":
                return self.finish_without_inner(
                    outer_goal,
                    "objetivo cancelado antes de llegar a Nav2",
                    canceled=state == "canceled",
                )
            inner_goal = send_future.result()
            if not inner_goal.accepted:
                return self.finish_without_inner(
                    outer_goal,
                    "Nav2 rechazó el objetivo",
                )

            self.publish_status(moving_status)
            result_future = inner_goal.get_result_async()
            state = self.wait_future(result_future, outer_goal)
            if state != "done":
                self.cancel_inner(inner_goal)
                return self.finish_without_inner(
                    outer_goal,
                    "movimiento cancelado" if state == "canceled" else
                    "se perdió la autoridad durante el movimiento",
                    canceled=state == "canceled",
                )

            wrapped = result_future.result()
            result = wrapped.result
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                outer_goal.succeed()
                self.publish_status(success_status)
            elif wrapped.status == GoalStatus.STATUS_CANCELED:
                outer_goal.canceled()
                self.publish_status("cancelado")
            else:
                outer_goal.abort()
                detail = getattr(result, "error_msg", "Nav2 abortó")
                self.publish_status(f"fallo: {detail or 'Nav2 abortó'}")
            return result
        except Exception as error:  # noqa: BLE001
            if inner_goal is not None:
                self.cancel_inner(inner_goal)
            return self.finish_without_inner(
                outer_goal,
                f"error del adaptador Nav2: {error}",
            )
        finally:
            self.release_mobility("acción Nav2 terminada")
            self.wait_for_stand()
            with self.lock:
                self.active = False
                self.reserved = False

    def wait_for_authority(self, outer_goal):
        deadline = time.monotonic() + AUTHORITY_WAIT_S
        while rclpy.ok() and time.monotonic() < deadline:
            if outer_goal.is_cancel_requested:
                return False
            self.claim_mobility()
            with self.lock:
                if self.mobility_owner == "navigation":
                    return True
            time.sleep(0.05)
        return False

    def wait_future(self, future, outer_goal):
        while rclpy.ok() and not future.done():
            if outer_goal.is_cancel_requested:
                return "canceled"
            self.claim_mobility()
            with self.lock:
                if self.mobility_owner not in {None, "navigation"}:
                    return "lost_authority"
            time.sleep(0.05)
        return "done" if future.done() else "lost_authority"

    def finish_without_inner(self, goal_handle, message, canceled=False):
        result_type = (
            Spin.Result
            if isinstance(goal_handle.request, Spin.Goal)
            else NavigateToPose.Result
        )
        result = result_type()
        if hasattr(result, "error_msg"):
            result.error_msg = message
        if canceled or goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.publish_status("cancelado")
        else:
            goal_handle.abort()
            self.publish_status(f"fallo: {message}")
        return result

    def cancel_inner(self, inner_goal):
        try:
            future = inner_goal.cancel_goal_async()
            deadline = time.monotonic() + INNER_RESPONSE_WAIT_S
            while (
                rclpy.ok()
                and not future.done()
                and time.monotonic() < deadline
            ):
                time.sleep(0.05)
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(
                f"no se pudo cancelar la acción interna de Nav2: {error}"
            )

    def request_mobility(self, operation, reason=None):
        request = {
            "operation": operation,
            "source": "navigation",
            "requester": REQUESTER,
        }
        if reason is not None:
            request["reason"] = reason
        self.pub_mobility.publish(
            String(data=json.dumps(request, ensure_ascii=False))
        )

    def claim_mobility(self):
        now = time.monotonic()
        if now - self.last_claim_at >= CLAIM_PERIOD_S:
            self.request_mobility("acquire")
            self.last_claim_at = now

    def release_mobility(self, reason):
        self.request_mobility("release", reason)

    def wait_for_stand(self):
        deadline = time.monotonic() + SAFE_STAND_WAIT_S
        while rclpy.ok() and time.monotonic() < deadline:
            with self.lock:
                if self.mobility_owner == "stand":
                    return True
            time.sleep(0.05)
        self.get_logger().error("no se confirmó el regreso a STAND")
        return False

    def publish_status(self, status):
        self.pub_status.publish(String(data=status))


def main():
    rclpy.init()
    node = Nav2Adapter()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
