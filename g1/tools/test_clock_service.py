#!/usr/bin/env python3
"""Prueba la lectura remota del reloj con una imagen JPEG guardada."""
import argparse
import sys
import time
from pathlib import Path


AGENT_DIR = Path(__file__).resolve().parents[1] / "agent"
sys.path.insert(0, str(AGENT_DIR))

from intelligence_client import (  # noqa: E402
    CircuitOpenError,
    IntelligenceClient,
    RemoteIntelligenceError,
)


def main():
    parser = argparse.ArgumentParser(
        description="Mide la lectura Jetson-servidor de una imagen del reloj."
    )
    parser.add_argument("image", type=Path)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument(
        "--expect-failure",
        action="store_true",
        help="aprueba solamente si todas las lecturas fallan",
    )
    args = parser.parse_args()

    image = args.image.read_bytes()
    client = IntelligenceClient()
    readings = []
    failures = []

    for index in range(args.repetitions):
        started_at = time.monotonic()
        try:
            result = client.read_clock(image)
            elapsed_s = time.monotonic() - started_at
            readings.append(result["text"])
            print(
                f"lectura {index + 1}: {result['text']} | "
                f"servidor {result.get('elapsed_s', '?')} s | "
                f"punta a punta {elapsed_s:.3f} s"
            )
        except RemoteIntelligenceError as error:
            elapsed_s = time.monotonic() - started_at
            failures.append(type(error).__name__)
            print(
                f"lectura {index + 1}: fallo {type(error).__name__} | "
                f"punta a punta {elapsed_s:.3f} s"
            )

    if args.expect_failure:
        if readings or len(failures) != args.repetitions:
            raise SystemExit(
                "FALLO: el enlace debía fallar en todas las lecturas"
            )
        if CircuitOpenError.__name__ not in failures:
            raise SystemExit(
                "FALLO: el cliente no abrió el corte automático"
            )
        print(
            "APROBADO: la caída fue explícita y el cuarto pedido "
            "se rechazó localmente"
        )
        return

    if failures:
        raise SystemExit(
            f"FALLO: hubo {len(failures)} lecturas remotas fallidas"
        )
    if len(set(readings)) != 1:
        raise SystemExit(
            "FALLO: el modelo no devolvió la misma hora en todas las lecturas"
        )
    print(f"APROBADO: {len(readings)} lecturas coinciden en {readings[0]}")


if __name__ == "__main__":
    main()
