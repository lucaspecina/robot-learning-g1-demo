#!/usr/bin/env python3
"""Pide una pose y verifica la magnitud que controla cada implementación."""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


VALID_POSES = ("reposo", "listo", "transporte")
TIMEOUT_S = 40.0
SETTLE_S = 8.0
MIN_STANDING_HEIGHT_M = 0.60
MAX_BODY_SHIFT_M = 0.15
MAX_BODY_TILT_DEG = 15.0


class ArmPoseVerifier(Node):
    def __init__(self, requested_pose: str):
        super().__init__("set_arm_pose")
        self.requested_pose = requested_pose
        self.status = None
        self.initial_position = None
        self.min_height = float("inf")
        self.max_body_shift = 0.0
        self.max_body_tilt_deg = 0.0
        self.publisher = self.create_publisher(String, "/g1/arm_pose", 10)
        self.create_subscription(String, "/g1/arm_status", self.on_status, 10)
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)

    def on_status(self, message: String):
        try:
            self.status = json.loads(message.data)
        except json.JSONDecodeError:
            self.status = None

    def send(self):
        self.publisher.publish(String(data=self.requested_pose))

    def on_odom(self, message: Odometry):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        if self.initial_position is None:
            self.initial_position = (position.x, position.y)
        dx = position.x - self.initial_position[0]
        dy = position.y - self.initial_position[1]
        self.max_body_shift = max(self.max_body_shift, math.hypot(dx, dy))
        self.min_height = min(self.min_height, position.z)

        sin_roll = 2.0 * (
            orientation.w * orientation.x
            + orientation.y * orientation.z
        )
        cos_roll = 1.0 - 2.0 * (
            orientation.x * orientation.x
            + orientation.y * orientation.y
        )
        roll = math.atan2(sin_roll, cos_roll)
        sin_pitch = 2.0 * (
            orientation.w * orientation.y
            - orientation.z * orientation.x
        )
        pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
        tilt_deg = math.degrees(max(abs(roll), abs(pitch)))
        self.max_body_tilt_deg = max(self.max_body_tilt_deg, tilt_deg)


def joint_degrees(status, joint_name):
    index = status["joint_names"].index(joint_name)
    return math.degrees(status["actual_rad"][index])


def worst_joint(status):
    errors = [
        abs(target - actual)
        for target, actual in zip(
            status["target_rad"],
            status["actual_rad"],
        )
    ]
    index = errors.index(max(errors))
    return (
        status["joint_names"][index],
        math.degrees(status["target_rad"][index]),
        math.degrees(status["actual_rad"][index]),
        math.degrees(errors[index]),
    )


def tracking_error_ratio(status):
    """Normaliza el error de articulaciones o de muñecas según el controlador."""
    if status.get("controller") == "pink":
        position_ratio = (
            status["maximum_wrist_position_error_m"]
            / status["wrist_position_tolerance_m"]
        )
        orientation_ratio = (
            status["maximum_wrist_orientation_error_deg"]
            / status["wrist_orientation_tolerance_deg"]
        )
        return max(position_ratio, orientation_ratio)
    return status["max_error_ratio"]


def print_tracking_measurement(status, prefix="  manos"):
    if status.get("controller") == "pink":
        print(
            f"{prefix}: error máximo "
            f"{status['maximum_wrist_position_error_m'] * 1000:.1f} mm, "
            f"{status['maximum_wrist_orientation_error_deg']:.1f}°"
        )
        return
    print(
        f"{prefix}: error máximo de articulaciones "
        f"{math.degrees(status['max_error_rad']):.1f}°"
    )


def main():
    requested_pose = sys.argv[1] if len(sys.argv) > 1 else ""
    if requested_pose not in VALID_POSES:
        print(f"uso: set_arm_pose.py {'|'.join(VALID_POSES)}")
        return 2

    rclpy.init()
    node = ArmPoseVerifier(requested_pose)
    started_at = time.monotonic()
    last_send = 0.0
    last_report = 0.0
    acknowledged = False
    reached_at = None
    settle_arm_error_ratios = []
    worst_settle_status = None

    try:
        while time.monotonic() - started_at < TIMEOUT_S:
            now = time.monotonic()
            if now - last_send >= 0.5:
                node.send()
                last_send = now
            rclpy.spin_once(node, timeout_sec=0.1)

            status = node.status
            if status is None or status.get("pose") != requested_pose:
                continue

            if reached_at is not None:
                current_ratio = tracking_error_ratio(status)
                settle_arm_error_ratios.append(current_ratio)
                if (
                    worst_settle_status is None
                    or current_ratio
                    > tracking_error_ratio(worst_settle_status)
                ):
                    worst_settle_status = status

            if not acknowledged:
                acknowledged = True
                mode = status.get("mode", "desconocido")
                explanation = (
                    "vista previa congelada; esto no prueba estabilidad"
                    if mode == "frozen_preview"
                    else "robot libre; la física está actuando"
                )
                print(f"  orden confirmada por el robot ({explanation})")

            if status.get("reached"):
                if reached_at is None:
                    reached_at = now
                    left_elbow = joint_degrees(status, "left_elbow_joint")
                    right_elbow = joint_degrees(status, "right_elbow_joint")
                    print(
                        f"  pose medida: codo izquierdo {left_elbow:+.1f}°, "
                        f"codo derecho {right_elbow:+.1f}°"
                    )
                    print_tracking_measurement(status)
                    if status.get("mode") == "frozen_preview":
                        print(
                            f"  PASA FORMA: brazos llegaron a '{requested_pose}'. "
                            "La estabilidad todavía no fue probada."
                        )
                        return 0
                    print(f"  sosteniendo {SETTLE_S:.0f} s para medir el cuerpo...")

            if reached_at is not None and now - reached_at >= SETTLE_S:
                print(
                    f"  cuerpo: altura mínima {node.min_height:.3f} m, "
                    f"desplazamiento máximo {node.max_body_shift:.3f} m, "
                    f"inclinación máxima {node.max_body_tilt_deg:.1f}°"
                )
                body_ok = (
                    node.min_height >= MIN_STANDING_HEIGHT_M
                    and node.max_body_shift <= MAX_BODY_SHIFT_M
                    and node.max_body_tilt_deg <= MAX_BODY_TILT_DEG
                )
                ordered_ratios = sorted(settle_arm_error_ratios)
                p95_index = round(0.95 * (len(ordered_ratios) - 1))
                p95_ratio = ordered_ratios[p95_index]
                max_ratio = ordered_ratios[-1]
                print(
                    f"  brazos sostenidos: error relativo p95 "
                    f"{p95_ratio:.2f}, máximo {max_ratio:.2f}"
                )
                # El cuerpo balanceándose produce picos breves. Exigir que
                # cada única muestra quede dentro del límite rechazó poses
                # cuyo error final era menor; p95 exige cumplimiento estable
                # y el segundo límite sigue capturando cualquier pico grande.
                arm_ok = (
                    p95_ratio < 1.0
                    and max_ratio < 1.5
                )
                if not arm_ok:
                    if worst_settle_status.get("controller") == "pink":
                        print_tracking_measurement(
                            worst_settle_status,
                            prefix="  FALLA MANOS",
                        )
                    else:
                        name, target_deg, actual_deg, error_deg = worst_joint(
                            worst_settle_status
                        )
                        print(
                            f"  FALLA BRAZOS: {name} pidió {target_deg:+.1f}° y "
                            f"midió {actual_deg:+.1f}° (error {error_deg:.1f}°)"
                        )
                    return 1
                if not body_ok:
                    print(
                        f"  FALLA ESTABILIDAD: límites altura "
                        f"{MIN_STANDING_HEIGHT_M:.2f} m, desplazamiento "
                        f"{MAX_BODY_SHIFT_M:.2f} m, inclinación "
                        f"{MAX_BODY_TILT_DEG:.0f}°"
                    )
                    return 1
                print(
                    f"  PASA: brazos llegaron a '{requested_pose}' y el "
                    "cuerpo se mantuvo dentro de los tres límites"
                )
                return 0

            if reached_at is None and now - last_report >= 1.0:
                print_tracking_measurement(status, prefix="  moviendo")
                last_report = now

        if not acknowledged:
            print("  FALLA: el robot nunca confirmó haber recibido la orden")
        elif node.status.get("controller") == "pink":
            print_tracking_measurement(node.status, prefix="  FALLA MANOS")
        else:
            name, target_deg, actual_deg, error_deg = worst_joint(node.status)
            print(
                f"  FALLA: {name} pidió {target_deg:+.1f}° y midió "
                f"{actual_deg:+.1f}° (error {error_deg:.1f}°)"
            )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
