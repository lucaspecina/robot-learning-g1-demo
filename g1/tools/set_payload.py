#!/usr/bin/env python3
"""Agrega o retira carga y exige confirmación de la masa física."""
import json
from pathlib import Path
import sys
import time
import uuid

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from payload_core import MAX_PAYLOAD_KG


TIMEOUT_S = 10.0


class PayloadVerifier(Node):
    def __init__(self, command: str, mass_kg: float):
        super().__init__("set_payload")
        self.request = {
            "request_id": str(uuid.uuid4()),
            "command": command,
            "mass_kg": mass_kg,
        }
        self.status = None
        self.publisher = self.create_publisher(
            String,
            "/g1/payload_request",
            10,
        )
        self.create_subscription(
            String,
            "/g1/payload_status",
            self.on_status,
            10,
        )

    def on_status(self, message: String):
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if status.get("request_id") == self.request["request_id"]:
            self.status = status

    def send(self):
        self.publisher.publish(
            String(data=json.dumps(self.request, ensure_ascii=False))
        )


def parse_arguments() -> tuple[str, float]:
    command = sys.argv[1] if len(sys.argv) > 1 else ""
    if command == "detach":
        return command, 0.0
    if command != "attach" or len(sys.argv) != 3:
        raise ValueError(
            "uso: set_payload.py attach <kg> | set_payload.py detach"
        )
    try:
        mass_kg = float(sys.argv[2])
    except ValueError as error:
        raise ValueError("la carga debe expresarse en kilogramos") from error
    if mass_kg <= 0.0 or mass_kg > MAX_PAYLOAD_KG:
        raise ValueError(
            f"la carga debe estar entre 0 y {MAX_PAYLOAD_KG:.1f} kg"
        )
    return command, mass_kg


def main():
    try:
        command, mass_kg = parse_arguments()
    except ValueError as error:
        print(error)
        return 2

    rclpy.init()
    node = PayloadVerifier(command, mass_kg)
    started_at = time.monotonic()
    last_send = 0.0
    try:
        while time.monotonic() - started_at < TIMEOUT_S:
            now = time.monotonic()
            if now - last_send >= 0.5:
                node.send()
                last_send = now
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.status is None:
                continue
            status = node.status
            expected_state = "attached" if command == "attach" else "detached"
            if status.get("state") != expected_state:
                print(f"FALLA: {status.get('error', 'orden rechazada')}")
                return 1
            print(
                f"PASA: {status.get('applied_mass_kg', 0.0):.2f} kg "
                f"verificados en {status.get('attachment_points', [])}"
            )
            if command == "attach":
                print(
                    "El bulto es visible y la masa es física; "
                    "esto todavía no prueba agarre."
                )
                print(
                    "Siguiente: bash run_demo.sh check stand; "
                    "después bash run_demo.sh check walk"
                )
            return 0
        print("FALLA: el robot no confirmó la orden de carga")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
