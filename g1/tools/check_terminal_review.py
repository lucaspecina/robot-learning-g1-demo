#!/usr/bin/env python3
"""Comprueba el contrato de cierre sin volver a mover el robot."""

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "agent"))

from intelligence_client import IntelligenceClient


def main():
    catalog = [
        {
            "name": "scan_for_table",
            "description": (
                "Busca la mesa elegida y mide su posición desde sensores."
            ),
            "availability": "ready",
            "variants": [
                {
                    "argument": "$selected_table",
                    "argument_description": "Mesa elegida previamente.",
                    "preconditions": ["selected_table_known"],
                    "effects": ["table_location_known"],
                }
            ],
        }
    ]
    last_step = {
        "id": "scan_for_table",
        "skill": "scan_for_table",
        "argument": "$selected_table",
        "label": "Buscar activamente la mesa elegida",
    }
    outcome = {
        "state": "succeeded",
        "message": "ubicó red_table mediante sensores",
        "measurements": {
            "views_checked": 4,
            "x_m": 3.65,
            "y_m": 2.78,
        },
    }
    review = IntelligenceClient().review_step(
        command="Buscá la mesa elegida alrededor de la habitación.",
        skill_catalog=catalog,
        world_facts=["selected_table_known", "table_location_known"],
        completed_steps=[
            {
                **last_step,
                "state": "succeeded",
                "result": outcome["message"],
            }
        ],
        last_step=last_step,
        outcome=outcome,
        pending_steps=[],
        review_count=1,
    )

    print(f"modelo: {review.get('model')}")
    print(f"decisión: {review['decision']}")
    print(f"razón: {review['reason']}")
    if review["decision"] != "complete":
        raise SystemExit(
            "FALLO: el último paso exitoso no cerró la misión como complete"
        )
    print("OK: el cierre exitoso ya no se confunde con detener por falla")


if __name__ == "__main__":
    main()
