#!/usr/bin/env python3
"""Exige que Nav2 rodee el cajón que corta el camino directo."""

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavigationPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from scene_layout import (  # noqa: E402
    NAVIGATION_TEST_GOAL,
    NAVIGATION_TEST_OBSTACLE,
)


TIMEOUT_S = 240.0
MIN_HEIGHT_M = 0.60
MAX_GOAL_ERROR_M = 0.15
MIN_CENTER_CLEARANCE_M = 0.45
MIN_DETOUR_M = 0.25
MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


def distance_to_obstacle(x: float, y: float) -> float:
    obstacle = NAVIGATION_TEST_OBSTACLE
    dx = max(
        abs(x - obstacle["x"]) - obstacle["size_x"] / 2.0,
        0.0,
    )
    dy = max(
        abs(y - obstacle["y"]) - obstacle["size_y"] / 2.0,
        0.0,
    )
    return math.hypot(dx, dy)


def line_distance(point, start, end) -> float:
    x, y = point
    sx, sy = start
    ex, ey = end
    length = math.hypot(ex - sx, ey - sy)
    if length == 0.0:
        return math.hypot(x - sx, y - sy)
    return abs((ex - sx) * (sy - y) - (sx - x) * (ey - sy)) / length


class ObstacleNavigationCheck(Node):
    def __init__(self):
        super().__init__("check_obstacle_navigation")
        self.client = ActionClient(self, NavigateToPose, "/g1/navigate_to_pose")
        self.pose = None
        self.samples = []
        self.plan = []
        self.plan_history = []
        self.owner = None
        self.static_map = None
        self.global_costmap = None
        self.feedback_count = 0
        self.last_remaining = math.inf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(
            OccupancyGrid,
            "/map",
            lambda message: setattr(self, "static_map", message),
            MAP_QOS,
        )
        self.create_subscription(
            OccupancyGrid,
            "/nav2/global_costmap/costmap",
            lambda message: setattr(self, "global_costmap", message),
            MAP_QOS,
        )
        self.create_subscription(
            NavigationPath,
            "/nav2/plan",
            self.on_plan,
            10,
        )
        self.create_subscription(
            String,
            "/g1/mobility/status",
            self.on_mobility,
            10,
        )

    def on_odom(self, message):
        point = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        )
        self.pose = point
        self.samples.append(point)

    def on_plan(self, message):
        self.plan = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in message.poses
        ]
        if self.plan:
            self.plan_history.append(self.plan)

    def on_mobility(self, message):
        try:
            self.owner = json.loads(message.data)["owner"]
        except (json.JSONDecodeError, KeyError, TypeError):
            pass

    def on_feedback(self, message):
        self.feedback_count += 1
        self.last_remaining = float(message.feedback.distance_remaining)


def wait_until(predicate, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return bool(predicate())


def cancel_safely(handle, result_future) -> None:
    cancel = handle.cancel_goal_async()
    wait_until(cancel.done, 3.0)
    wait_until(result_future.done, 5.0)


def grid_value(message, x: float, y: float):
    """Devuelve la ocupación de una coordenada mundial o None si queda fuera."""
    resolution = float(message.info.resolution)
    column = math.floor((x - message.info.origin.position.x) / resolution)
    row = math.floor((y - message.info.origin.position.y) / resolution)
    if not (0 <= column < message.info.width and 0 <= row < message.info.height):
        return None
    return int(message.data[row * message.info.width + column])


def parse_args():
    parser = argparse.ArgumentParser(
        description="Comprueba detección viva y, opcionalmente, el rodeo",
    )
    parser.add_argument(
        "--sensing-only",
        action="store_true",
        help="verificar los mapas sin entregar una orden de movimiento",
    )
    return parser.parse_args()


def report_measurements(node, start, goal):
    """Imprime también la evidencia de una corrida incompleta."""
    goal_x, goal_y = goal
    trajectory = [(x, y) for x, y, _ in node.samples]
    all_plan_points = [
        point for plan in node.plan_history for point in plan
    ]
    plan_clearance = (
        min(distance_to_obstacle(x, y) for x, y in all_plan_points)
        if all_plan_points
        else math.nan
    )
    actual_clearance = (
        min(distance_to_obstacle(x, y) for x, y in trajectory)
        if trajectory
        else math.nan
    )
    detour = (
        max(
            line_distance(point, start, (goal_x, goal_y))
            for point in trajectory
        )
        if trajectory
        else math.nan
    )
    minimum_height = (
        min(z for _, _, z in node.samples) if node.samples else math.nan
    )
    final_error = (
        math.hypot(node.pose[0] - goal_x, node.pose[1] - goal_y)
        if node.pose is not None
        else math.nan
    )
    correction_m = math.nan
    correction_deg = math.nan
    if node.tf_buffer.can_transform(
        "map", "odom", rclpy.time.Time(), Duration(seconds=0.2)
    ):
        transform = node.tf_buffer.lookup_transform(
            "map", "odom", rclpy.time.Time()
        ).transform
        correction_m = math.hypot(
            transform.translation.x,
            transform.translation.y,
        )
        q = transform.rotation
        correction_deg = abs(
            math.degrees(
                math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y**2 + q.z**2),
                )
            )
        )
    print(
        f"ruta: {len(node.plan_history)} versiones; margen planeado "
        f"{plan_clearance:.3f} m"
    )
    print(
        f"movimiento: desvío de la recta {detour:.3f} m; "
        f"margen real {actual_clearance:.3f} m"
    )
    print(
        f"llegada física: error {final_error:.3f} m; altura mínima "
        f"{minimum_height:.3f} m; restante informado "
        f"{node.last_remaining:.2f} m"
    )
    print(
        f"corrección de localización: {correction_m:.2f} m / "
        f"{correction_deg:.1f}°; dueño final {node.owner}"
    )
    return {
        "plan_clearance": plan_clearance,
        "actual_clearance": actual_clearance,
        "detour": detour,
        "minimum_height": minimum_height,
        "final_error": final_error,
    }


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = ObstacleNavigationCheck()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        if not wait_until(lambda: node.pose is not None, 10.0):
            raise RuntimeError("no llegó la posición inicial")
        if not args.sensing_only and not node.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("no apareció la Action de navegación")

        if not wait_until(
            lambda: node.static_map is not None
            and node.global_costmap is not None,
            15.0,
        ):
            raise RuntimeError("no llegaron el mapa fijo y el mapa de Nav2")

        start = node.pose[:2]
        goal_x, goal_y, goal_yaw = NAVIGATION_TEST_GOAL
        obstacle_x = NAVIGATION_TEST_OBSTACLE["x"]
        obstacle_y = NAVIGATION_TEST_OBSTACLE["y"]
        static_value = grid_value(node.static_map, obstacle_x, obstacle_y)
        if static_value is None or static_value >= 50:
            raise RuntimeError(
                "el cajón ya está grabado en el mapa fijo; la prueba no "
                "demostraría detección en vivo"
            )
        if not wait_until(
            lambda: (
                node.global_costmap is not None
                and (grid_value(
                    node.global_costmap,
                    obstacle_x,
                    obstacle_y,
                ) or -1) >= 50
            ),
            15.0,
        ):
            value = grid_value(node.global_costmap, obstacle_x, obstacle_y)
            raise RuntimeError(
                "los sensores no agregaron el cajón al mapa vivo de Nav2 "
                f"(ocupación {value})"
            )
        live_value = grid_value(node.global_costmap, obstacle_x, obstacle_y)
        print(
            "detección viva: mapa fijo libre "
            f"({static_value}), mapa de Nav2 ocupado ({live_value})"
        )
        if args.sensing_only:
            print("PASA: detección comprobada sin ordenar movimiento")
            return 0
        # La prueba sólo discrimina esquive si la caja corta la recta ideal.
        if distance_to_obstacle(
            NAVIGATION_TEST_OBSTACLE["x"],
            NAVIGATION_TEST_OBSTACLE["y"],
        ) != 0.0:
            raise RuntimeError("la geometría declarada del obstáculo es inválida")
        if line_distance(
            (NAVIGATION_TEST_OBSTACLE["x"], NAVIGATION_TEST_OBSTACLE["y"]),
            start,
            (goal_x, goal_y),
        ) > 0.10:
            raise RuntimeError("el obstáculo ya no corta el camino directo")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(goal_yaw / 2.0)
        sent = node.client.send_goal_async(goal, feedback_callback=node.on_feedback)
        if not wait_until(sent.done, 10.0):
            raise RuntimeError("Nav2 no confirmó el objetivo")
        handle = sent.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("Nav2 rechazó el objetivo")
        result_future = handle.get_result_async()
        if not wait_until(result_future.done, TIMEOUT_S):
            cancel_safely(handle, result_future)
            report_measurements(node, start, (goal_x, goal_y))
            raise RuntimeError("la navegación agotó su plazo y fue cancelada")
        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            detail = str(getattr(wrapped.result, "error_msg", ""))
            raise RuntimeError(f"Nav2 abortó el rodeo: {detail or wrapped.status}")
        if not wait_until(lambda: node.owner == "stand", 5.0):
            raise RuntimeError("la autoridad no volvió a STAND")
        if not node.plan:
            raise RuntimeError("Nav2 no publicó la ruta que ejecutó")

        metrics = report_measurements(node, start, (goal_x, goal_y))
        plan_clearance = metrics["plan_clearance"]
        actual_clearance = metrics["actual_clearance"]
        detour = metrics["detour"]
        minimum_height = metrics["minimum_height"]
        final_error = metrics["final_error"]

        failures = []
        if plan_clearance < MIN_CENTER_CLEARANCE_M:
            failures.append("la ruta planeada pasa demasiado cerca del cajón")
        if actual_clearance < MIN_CENTER_CLEARANCE_M:
            failures.append("el cuerpo pasó demasiado cerca del cajón")
        if detour < MIN_DETOUR_M:
            failures.append("la trayectoria no demuestra un rodeo")
        if final_error > MAX_GOAL_ERROR_M:
            failures.append("terminó fuera de la tolerancia de llegada")
        if minimum_height < MIN_HEIGHT_M:
            failures.append("el robot perdió la postura durante el rodeo")
        if failures:
            raise RuntimeError("; ".join(failures))
        print("PASA: Nav2 rodeó el obstáculo, llegó y devolvió el mando a STAND")
        return 0
    except RuntimeError as error:
        print(f"FALLA RODEO: {error}")
        return 1
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
