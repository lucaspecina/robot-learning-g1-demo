#!/usr/bin/env python3
"""Lee varias veces el último recorte vivo del reloj."""
import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


AGENT_DIR = Path(__file__).resolve().parents[1] / "agent"
sys.path.insert(0, str(AGENT_DIR))

from intelligence_client import (  # noqa: E402
    IntelligenceClient,
    RemoteIntelligenceError,
)


class ClockCropReceiver(Node):
    def __init__(self):
        super().__init__("check_clock_reading")
        self.image = None
        self.create_subscription(
            CompressedImage,
            "/g1/clock_crop/compressed",
            self.on_image,
            2,
        )

    def on_image(self, message: CompressedImage):
        if message.format.lower() in {"jpeg", "jpg"}:
            self.image = bytes(message.data)

    def wait_for_image(self, timeout_s: float) -> bytes:
        end = time.monotonic() + timeout_s
        while self.image is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.image is None:
            raise RuntimeError(
                "no llegó un recorte JPEG reciente del reloj"
            )
        return self.image


def main():
    parser = argparse.ArgumentParser(
        description="Comprueba detector, red, servidor y modelo visual."
    )
    parser.add_argument("--expected")
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--image-timeout-s", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    receiver = ClockCropReceiver()
    try:
        image = receiver.wait_for_image(args.image_timeout_s)
    except RuntimeError as error:
        raise SystemExit(f"FALLO: {error}") from error
    finally:
        receiver.destroy_node()
        rclpy.shutdown()

    client = IntelligenceClient()
    readings = []
    for index in range(args.repetitions):
        started_at = time.monotonic()
        try:
            result = client.read_clock(image)
        except RemoteIntelligenceError as error:
            raise SystemExit(
                f"FALLO: lectura remota {index + 1}: {error}"
            ) from error
        elapsed_s = time.monotonic() - started_at
        readings.append(result["text"])
        print(
            f"lectura {index + 1}: {result['text']} | "
            f"servidor {result.get('elapsed_s', '?')} s | "
            f"punta a punta {elapsed_s:.3f} s"
        )

    if len(set(readings)) != 1:
        raise SystemExit(
            "FALLO: el modelo no devolvió la misma hora en todas las lecturas"
        )
    if args.expected is not None and readings[0] != args.expected:
        raise SystemExit(
            f"FALLO: esperaba {args.expected} y leyó {readings[0]}"
        )
    print(
        f"APROBADO: recorte vivo leído {len(readings)} veces como {readings[0]}"
    )


if __name__ == "__main__":
    main()
