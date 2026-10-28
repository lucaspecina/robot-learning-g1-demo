#!/usr/bin/env python3
"""Verifica la barrera oficial de colisiones entre la autoridad y el robot."""

import json
import math
import statistics
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from scene_layout import ROOM_INTERIOR_BOUNDS  # noqa: E402
from checks import Checker  # noqa: E402


AUTHORIZED_TOPIC = "/g1/cmd_vel/authorized"
FINAL_TOPIC = "/cmd_vel"
SCAN_TOPIC = "/scan"
WALL_SPEED_MPS = -0.25
MIN_SAFE_CENTER_CLEARANCE_M = 0.40
MIN_USEFUL_TRAVEL_M = 0.30
WIRING_SPEED_MPS = 0.30


class SafetyChecker(Checker):
    def __init__(self):
        super().__init__()
        self.authorized_command = None
        self.final_command = None
        self.scan_messages = 0
        self.minimum_scan_range = math.inf
        self.latest_odom_twist = (0.0, 0.0)
        self.collision_state = (CollisionMonitorState.DO_NOTHING, "")
        self.create_subscription(
            Twist,
            AUTHORIZED_TOPIC,
            self.on_authorized_command,
            10,
        )
        self.create_subscription(Twist, FINAL_TOPIC, self.on_final_command, 10)
        self.create_subscription(
            LaserScan,
            SCAN_TOPIC,
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CollisionMonitorState,
            "/g1/safety/collision_state",
            self.on_collision_state,
            10,
        )

    def on_odom(self, message):
        super().on_odom(message)
        self.latest_odom_twist = (
            float(message.twist.twist.linear.x),
            float(message.twist.twist.linear.y),
        )

    def on_authorized_command(self, message: Twist):
        self.authorized_command = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )

    def on_final_command(self, message: Twist):
        self.final_command = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )

    def on_scan(self, message: LaserScan):
        self.scan_messages += 1
        valid_ranges = [
            value
            for value in message.ranges
            if math.isfinite(value)
            and message.range_min <= value <= message.range_max
        ]
        self.minimum_scan_range = min(valid_ranges, default=math.inf)

    def on_collision_state(self, message: CollisionMonitorState):
        self.collision_state = (message.action_type, message.polygon_name)

    def publishers(self, topic: str) -> list[str]:
        return sorted(
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in self.get_publishers_info_by_topic(topic)
        )


def wait_for_runtime(node: SafetyChecker, timeout_s: float = 20.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        node.spin_for(0.2)
        if (
            node.pose is not None
            and node.mobility_owner is not None
            and node.scan_messages >= 3
        ):
            return True
    return False


def hold_test_command(
    node: SafetyChecker,
    vx: float,
    duration_s: float,
) -> list[float]:
    outputs = []
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        node.send_cmd(vx=vx)
        node.spin_for(0.1)
        if node.final_command is not None:
            outputs.append(node.final_command[0])
    return outputs


def check_wiring(node: SafetyChecker) -> bool:
    print("\n=== SEGURIDAD: cadena de autoridad y filtro ===")
    if not wait_for_runtime(node):
        print(
            "FALLA: estado incompleto: "
            f"posición={node.pose is not None}, "
            f"dueño={node.mobility_owner}, barridos={node.scan_messages}, "
            f"salida previa={node.final_command}"
        )
        return False

    authorized_publishers = node.publishers(AUTHORIZED_TOPIC)
    final_publishers = node.publishers(FINAL_TOPIC)
    print(f"entrada de seguridad: {authorized_publishers}")
    print(f"salida final: {final_publishers}")
    if len(authorized_publishers) != 1 or not any(
        "mobility_authority" in name for name in authorized_publishers
    ):
        print("FALLA: la intención autorizada no tiene un único árbitro")
        return False
    if len(final_publishers) != 1 or not any(
        "collision_monitor" in name for name in final_publishers
    ):
        print("FALLA: /cmd_vel no pertenece únicamente a Collision Monitor")
        return False

    node.reset_robot()
    node.spin_for(1.0)
    if not node.acquire_test_mobility():
        print(f"FALLA: TEST no obtuvo autoridad; dueño {node.mobility_owner}")
        return False
    try:
        outputs = hold_test_command(
            node,
            vx=WIRING_SPEED_MPS,
            duration_s=2.0,
        )
    finally:
        node.release_test_mobility("cableado de seguridad verificado")
    median_output = statistics.median(outputs) if outputs else 0.0
    print(
        f"orden {WIRING_SPEED_MPS:.2f} m/s; salida mediana "
        f"{median_output:.2f} m/s; estado {node.collision_state}"
    )
    if median_output < WIRING_SPEED_MPS - 0.05:
        print(
            "FALLA: el filtro limitó movimiento en el espacio libre inicial"
        )
        return False
    print(
        f"PASA: LiDAR vivo ({node.scan_messages} barridos), orden autorizada "
        f"y salida final separadas"
    )
    return True


def center_clearance_to_room_wall(x: float, y: float) -> float:
    return min(
        x - ROOM_INTERIOR_BOUNDS["xmin"],
        ROOM_INTERIOR_BOUNDS["xmax"] - x,
        y - ROOM_INTERIOR_BOUNDS["ymin"],
        ROOM_INTERIOR_BOUNDS["ymax"] - y,
    )


def approach_wall_once(node: SafetyChecker, trial: int) -> tuple[bool, dict]:
    node.reset_robot()
    node.spin_for(1.0)
    if not node.acquire_test_mobility():
        return False, {"error": f"TEST no obtuvo autoridad: {node.mobility_owner}"}

    start = node.pose
    stopped_samples = 0
    minimum_clearance = math.inf
    intervention_clearance = None
    trigger_scan_range = None
    peak_odom_speed = 0.0
    deadline = time.monotonic() + 45.0
    try:
        while time.monotonic() < deadline:
            node.send_cmd(vx=WALL_SPEED_MPS)
            node.spin_for(0.1)
            x, y, z, _yaw = node.pose
            peak_odom_speed = max(
                peak_odom_speed,
                math.hypot(*node.latest_odom_twist),
            )
            clearance = center_clearance_to_room_wall(x, y)
            minimum_clearance = min(minimum_clearance, clearance)
            travel = math.hypot(x - start[0], y - start[1])
            if z < 0.60:
                return False, {"error": "el robot cayó durante la prueba"}
            if clearance < MIN_SAFE_CENTER_CLEARANCE_M:
                return False, {
                    "error": f"entró en la reserva física: {clearance:.3f} m",
                }
            authorized = node.authorized_command or (0.0, 0.0, 0.0)
            final = node.final_command or (0.0, 0.0, 0.0)
            filtered_stop = authorized[0] < -0.20 and abs(final[0]) < 0.02
            if filtered_stop and travel >= MIN_USEFUL_TRAVEL_M:
                stopped_samples += 1
                intervention_clearance = clearance
                trigger_scan_range = node.minimum_scan_range
            else:
                stopped_samples = 0
            if stopped_samples >= 5:
                before = node.pose
                hold_test_command(node, vx=WALL_SPEED_MPS, duration_s=2.0)
                after = node.pose
                extra_travel = math.hypot(after[0] - before[0], after[1] - before[1])
                return extra_travel <= 0.08, {
                    "trial": trial,
                    "travel_m": travel,
                    "clearance_m": intervention_clearance,
                    "extra_travel_m": extra_travel,
                    "minimum_clearance_m": minimum_clearance,
                    "trigger_scan_range_m": trigger_scan_range,
                    "peak_odom_speed_mps": peak_odom_speed,
                    "collision_state": node.collision_state,
                }
        return False, {"error": "no frenó antes del plazo de 45 s"}
    finally:
        node.release_test_mobility("prueba de pared terminada")


def check_wall(node: SafetyChecker, repetitions: int = 3) -> bool:
    print("\n=== SEGURIDAD: obstáculo real visto por el LiDAR ===")
    if not check_wiring(node):
        return False
    results = []
    for trial in range(1, repetitions + 1):
        ok, result = approach_wall_once(node, trial)
        if not ok:
            print(f"FALLA corrida {trial}: {result}")
            return False
        results.append(result)
        print(
            f"corrida {trial}: recorrió {result['travel_m']:.2f} m, "
            f"frenó con {result['clearance_m']:.2f} m al muro y avanzó "
            f"{result['extra_travel_m']:.2f} m después del corte; "
            f"LiDAR mínimo {result['trigger_scan_range_m']:.2f} m, "
            f"velocidad odométrica pico {result['peak_odom_speed_mps']:.2f} m/s, "
            f"estado {result['collision_state']}"
        )
    clearances = [result["clearance_m"] for result in results]
    spread = max(clearances) - min(clearances)
    if spread > 0.15:
        print(f"FALLA: el punto de corte varió {spread:.2f} m entre corridas")
        return False
    print(
        "PASA: tres frenadas independientes; distancia media al muro "
        f"{statistics.mean(clearances):.2f} m, variación {spread:.2f} m"
    )
    return True


def main() -> int:
    mode = sys.argv[1] if len(sys.argv) > 1 else "wiring"
    if mode not in {"wiring", "wall"}:
        print("uso: check_collision_safety.py [wiring|wall]")
        return 2
    rclpy.init()
    node = SafetyChecker()
    try:
        passed = check_wiring(node) if mode == "wiring" else check_wall(node)
        return 0 if passed else 1
    finally:
        node.send_cmd()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
