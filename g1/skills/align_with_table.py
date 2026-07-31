#!/usr/bin/env python3
"""Alineación visual fina detrás de la Action estándar DockRobot de Nav2.

La navegación general deja al G1 en una zona de observación. Este nodo toma
mediciones nuevas de mesa, corrige despacio la base y devuelve la movilidad a
STAND ante éxito, cancelación o falla. La mesa se trata como infraestructura
de acople no cargadora; todavía no se afirma que las manos puedan agarrar.
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

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav2_msgs.action import DockRobot  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from vision_msgs.msg import Detection3DArray  # noqa: E402

from navigation_core import ProgressChecker  # noqa: E402
from table_alignment_core import (  # noqa: E402
    AlignmentPose,
    AlignmentTarget,
    TableAlignmentController,
    TargetFilter,
)


RATE_HZ = 10.0
CLAIM_RETRY_S = 0.5
AUTHORITY_WAIT_TIMEOUT_S = 10.0
INITIAL_PERCEPTION_TIMEOUT_S = 15.0
# RT-DETR tarda 2,1--2,4 s en esta Jetson simulada. Dos cuadros que no pudieron
# emparejar color y profundidad produjeron un hueco medido de 6,4 s; ocho
# segundos conservan el corte seguro sin declarar caída una latencia normal.
DETECTION_TIMEOUT_S = 8.0
EXECUTION_TIMEOUT_S = 180.0
SAFE_STAND_TIMEOUT_S = 3.0
MINIMUM_BODY_HEIGHT_M = 0.60
PROGRESS_TIMEOUT_S = 35.0


def yaw_from_quaternion(w: float, x: float, y: float, z: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def duration_message(seconds: float):
    from builtin_interfaces.msg import Duration

    seconds = max(0.0, float(seconds))
    whole = int(seconds)
    return Duration(
        sec=whole,
        nanosec=int((seconds - whole) * 1_000_000_000),
    )


class TableAlignment(Node):
    def __init__(self):
        super().__init__("align_with_table")
        self.callback_group = ReentrantCallbackGroup()
        self.state_lock = threading.RLock()
        self.pose = None
        self.mobility_owner = None
        self.last_claim_at = float("-inf")
        self.goal_reserved = False
        self.alignment_active = False
        self.finishing = False
        self.had_authority = False
        self.interrupt_reason = None
        self.selected_table = None
        self.filtered_target = None
        self.detection_received_at = None
        self.detection_count = 0
        self.target_filter = TargetFilter(coefficient=0.1)
        self.controller = TableAlignmentController(
            standoff_m=float(os.environ.get("G1_TABLE_STANDOFF_M", "0.70")),
        )

        self.pub_cmd = self.create_publisher(
            Twist,
            "/g1/cmd_vel/alignment",
            10,
        )
        self.pub_mobility = self.create_publisher(
            String,
            "/g1/mobility/request",
            10,
        )
        self.pub_status = self.create_publisher(
            String,
            "/g1/alignment_status",
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
            Detection3DArray,
            "/g1/table_detections_3d",
            self.on_table_detections,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            "/g1/mobility/status",
            self.on_mobility_status,
            10,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            DockRobot,
            "/g1/dock_to_table",
            execute_callback=self.execute_action,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel,
            callback_group=self.callback_group,
        )
        self.get_logger().info(
            "alineación lista en /g1/dock_to_table con interfaz DockRobot"
        )

    def on_odom(self, message: Odometry):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        observed = AlignmentPose(
            x=float(position.x),
            y=float(position.y),
            yaw=yaw_from_quaternion(
                orientation.w,
                orientation.x,
                orientation.y,
                orientation.z,
            ),
            linear_speed=math.hypot(float(linear.x), float(linear.y)),
            angular_speed=abs(float(angular.z)),
        )
        with self.state_lock:
            self.pose = observed
            if observed.yaw != observed.yaw or observed.linear_speed != observed.linear_speed:
                self.interrupt_reason = "la odometría dejó de ser válida"
            if self.alignment_active and message.pose.pose.position.z < MINIMUM_BODY_HEIGHT_M:
                self.interrupt_reason = "el cuerpo perdió una altura segura"

    def on_table_detections(self, message: Detection3DArray):
        with self.state_lock:
            selected_table = self.selected_table
            active = self.alignment_active
        if not active or selected_table is None:
            return
        candidates = []
        for detection in message.detections:
            if not detection.results:
                continue
            best = max(
                detection.results,
                key=lambda result: result.hypothesis.score,
            )
            if best.hypothesis.class_id != selected_table:
                continue
            point = detection.bbox.center.position
            candidates.append(
                (
                    float(best.hypothesis.score),
                    AlignmentTarget(float(point.x), float(point.y)),
                )
            )
        if not candidates:
            return
        _confidence, target = max(candidates, key=lambda item: item[0])
        try:
            filtered = self.target_filter.update(target)
        except ValueError as error:
            with self.state_lock:
                self.interrupt_reason = f"medición de mesa inválida: {error}"
            return
        with self.state_lock:
            self.filtered_target = filtered
            self.detection_received_at = time.monotonic()
            self.detection_count += 1

    def on_mobility_status(self, message: String):
        try:
            owner = str(json.loads(message.data)["owner"])
        except (json.JSONDecodeError, KeyError, TypeError):
            return
        with self.state_lock:
            previous = self.mobility_owner
            self.mobility_owner = owner
            if owner == "alignment":
                self.had_authority = True
            elif (
                previous == "alignment"
                and self.alignment_active
                and self.had_authority
                and not self.finishing
            ):
                self.interrupt_reason = (
                    f"se perdió la autoridad; dueño actual: {owner}"
                )

    def handle_goal(self, request):
        if request.use_dock_id:
            self.get_logger().warning(
                "se rechazó la alineación: requiere una pose visual, no un ID"
            )
            return GoalResponse.REJECT
        if request.dock_pose.header.frame_id != "map":
            self.get_logger().warning(
                "se rechazó la alineación: la mesa no está expresada en map"
            )
            return GoalResponse.REJECT
        if request.dock_type not in ("red_table", "blue_table"):
            self.get_logger().warning("se rechazó una mesa desconocida")
            return GoalResponse.REJECT
        with self.state_lock:
            if self.goal_reserved or self.alignment_active:
                return GoalResponse.REJECT
            self.goal_reserved = True
        return GoalResponse.ACCEPT

    @staticmethod
    def handle_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def execute_action(self, goal_handle):
        request = goal_handle.request
        started_at = time.monotonic()
        initial_perception_deadline = started_at + INITIAL_PERCEPTION_TIMEOUT_S
        authority_deadline = started_at + AUTHORITY_WAIT_TIMEOUT_S
        progress = ProgressChecker(
            movement_radius_m=0.03,
            movement_angle_rad=math.radians(2.0),
            allowance_s=PROGRESS_TIMEOUT_S,
        )
        result = DockRobot.Result()
        result.success = False
        result.error_code = DockRobot.Result.UNKNOWN
        result.num_retries = 0
        result.error_msg = ""

        with self.state_lock:
            self.goal_reserved = False
            self.alignment_active = True
            self.finishing = False
            self.had_authority = False
            self.interrupt_reason = None
            self.selected_table = request.dock_type
            self.filtered_target = None
            self.detection_received_at = None
            self.detection_count = 0
        self.target_filter.reset()
        self.controller.reset()
        progress.reset()
        self.publish_status("esperando_percepcion")

        terminal = "failed"
        message = "la alineación terminó sin resultado"
        error_code = DockRobot.Result.UNKNOWN
        last_command = None
        try:
            while rclpy.ok():
                now = time.monotonic()
                with self.state_lock:
                    pose = self.pose
                    target = self.filtered_target
                    detection_at = self.detection_received_at
                    detection_count = self.detection_count
                    owner = self.mobility_owner
                    interrupt_reason = self.interrupt_reason

                if goal_handle.is_cancel_requested:
                    terminal = "canceled"
                    message = "alineación cancelada por el solicitante"
                    break
                if interrupt_reason is not None:
                    message = interrupt_reason
                    error_code = DockRobot.Result.FAILED_TO_CONTROL
                    break
                if now - started_at > EXECUTION_TIMEOUT_S:
                    message = "venció el plazo total de alineación"
                    error_code = DockRobot.Result.FAILED_TO_CONTROL
                    break
                if pose is None:
                    if now > initial_perception_deadline:
                        message = "no llegó la posición del robot"
                        error_code = DockRobot.Result.FAILED_TO_CONTROL
                        break
                    self.publish_feedback(
                        goal_handle,
                        DockRobot.Feedback.INITIAL_PERCEPTION,
                        started_at,
                    )
                    time.sleep(1.0 / RATE_HZ)
                    continue
                if target is None or detection_at is None:
                    if now > initial_perception_deadline:
                        message = "la cámara no volvió a medir la mesa elegida"
                        error_code = DockRobot.Result.FAILED_TO_DETECT_DOCK
                        break
                    self.publish_feedback(
                        goal_handle,
                        DockRobot.Feedback.INITIAL_PERCEPTION,
                        started_at,
                    )
                    time.sleep(1.0 / RATE_HZ)
                    continue
                if now - detection_at > DETECTION_TIMEOUT_S:
                    message = "se perdió la medición reciente de la mesa"
                    error_code = DockRobot.Result.FAILED_TO_DETECT_DOCK
                    break

                if owner != "alignment":
                    if now > authority_deadline:
                        message = "no se obtuvo la autoridad de movilidad"
                        error_code = DockRobot.Result.FAILED_TO_CONTROL
                        break
                    self.claim_mobility(force=not self.had_authority)
                    self.publish_feedback(
                        goal_handle,
                        DockRobot.Feedback.CONTROLLING,
                        started_at,
                    )
                    time.sleep(1.0 / RATE_HZ)
                    continue

                command = self.controller.step(pose, target, now)
                last_command = command
                self.publish_command(command.linear_x, command.angular_z)
                self.publish_feedback(
                    goal_handle,
                    DockRobot.Feedback.CONTROLLING,
                    started_at,
                )
                self.publish_status(
                    command.phase,
                    distance_error_m=round(command.distance_error_m, 4),
                    yaw_error_deg=round(
                        math.degrees(command.yaw_error_rad),
                        2,
                    ),
                    linear_speed_mps=round(pose.linear_speed, 4),
                    angular_speed_radps=round(pose.angular_speed, 4),
                    detection_count=detection_count,
                )
                if command.stable:
                    terminal = "succeeded"
                    message = "alineación fina confirmada"
                    error_code = DockRobot.Result.NONE
                    break
                if not progress.update(pose, now):
                    message = "la alineación no mostró progreso medible"
                    error_code = DockRobot.Result.FAILED_TO_CONTROL
                    break
                time.sleep(1.0 / RATE_HZ)
        except Exception as error:  # noqa: BLE001
            message = f"error interno de alineación: {error}"
            error_code = DockRobot.Result.UNKNOWN
            self.get_logger().error(message)
        finally:
            safe = self.return_to_safe_state(message)
            with self.state_lock:
                self.alignment_active = False
                self.goal_reserved = False
                self.finishing = False
                self.had_authority = False
                self.interrupt_reason = None
                self.selected_table = None

        if not safe and terminal != "canceled":
            terminal = "failed"
            message += "; no se confirmó el regreso a STAND"
            error_code = DockRobot.Result.FAILED_TO_CONTROL

        result.error_code = error_code
        result.error_msg = message
        if terminal == "succeeded":
            result.success = True
            goal_handle.succeed()
            self.publish_status(
                "alineado",
                distance_error_m=round(last_command.distance_error_m, 4),
                yaw_error_deg=round(
                    math.degrees(last_command.yaw_error_rad),
                    2,
                ),
                linear_speed_mps=round(pose.linear_speed, 4),
                angular_speed_radps=round(pose.angular_speed, 4),
                detection_count=detection_count,
            )
            self.get_logger().info(message)
        elif terminal == "canceled":
            goal_handle.canceled()
            self.publish_status("cancelado", error=message)
            self.get_logger().warning(message)
        else:
            goal_handle.abort()
            failure_fields = {"error": message}
            if last_command is not None:
                failure_fields.update(
                    {
                        "distance_error_m": round(
                            last_command.distance_error_m,
                            4,
                        ),
                        "yaw_error_deg": round(
                            math.degrees(last_command.yaw_error_rad),
                            2,
                        ),
                    }
                )
            self.publish_status("fallo", **failure_fields)
            self.get_logger().error(message)
        return result

    def publish_feedback(self, goal_handle, state: int, started_at: float):
        feedback = DockRobot.Feedback()
        feedback.state = state
        feedback.docking_time = duration_message(time.monotonic() - started_at)
        feedback.num_retries = 0
        goal_handle.publish_feedback(feedback)

    def publish_command(self, linear_x: float, angular_z: float):
        message = Twist()
        message.linear.x = float(linear_x)
        message.angular.z = float(angular_z)
        self.pub_cmd.publish(message)

    def publish_status(self, state: str, **fields):
        self.pub_status.publish(
            String(
                data=json.dumps(
                    {"state": state, **fields},
                    ensure_ascii=False,
                )
            )
        )

    def request_mobility(self, operation: str, reason: str = None):
        request = {
            "operation": operation,
            "source": "alignment",
            "requester": "align_with_table",
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

    def return_to_safe_state(self, reason: str) -> bool:
        with self.state_lock:
            self.finishing = True
            owner = self.mobility_owner
        self.publish_command(0.0, 0.0)
        if owner == "alignment":
            self.request_mobility("release", reason)
        elif owner not in (None, "stand"):
            return True

        deadline = time.monotonic() + SAFE_STAND_TIMEOUT_S
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_command(0.0, 0.0)
            with self.state_lock:
                if self.mobility_owner == "stand":
                    return True
            time.sleep(0.05)
        with self.state_lock:
            return self.mobility_owner == "stand"


def main():
    rclpy.init()
    node = TableAlignment()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_command(0.0, 0.0)
            node.request_mobility("release", "nodo de alineación detenido")
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
