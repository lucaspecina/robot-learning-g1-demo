#!/usr/bin/env python3
"""Comprueba con el modelo real que una falla recuperable cambie el plan."""
import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from agent.intelligence_client import IntelligenceClient  # noqa: E402
from skill_catalog import skill_catalog_for_model  # noqa: E402


def main():
    outcome = {
        "state": "blocked",
        "message": "la mesa no apareció en la vista actual",
        "blocker": {
            "type": "recoverable_with_skill",
            "skill": "scan_for_table",
        },
    }
    review = IntelligenceClient().review_step(
        command="Encontrá la mesa elegida.",
        skill_catalog=skill_catalog_for_model(),
        world_facts=["robot_pose_known", "selected_table_known"],
        completed_steps=[
            {
                "id": "search_table_current_view",
                "skill": "search_table",
                "argument": "$selected_table",
                "label": "Buscar la mesa en la vista actual",
                "state": "blocked",
                "error": outcome["message"],
            }
        ],
        last_step={
            "id": "search_table_current_view",
            "skill": "search_table",
            "argument": "$selected_table",
            "label": "Buscar la mesa en la vista actual",
        },
        outcome=outcome,
        pending_steps=[],
        review_count=1,
    )
    revised = review["revised_steps"]
    first_skill = revised[0]["skill"] if revised else None
    print(f"decisión={review['decision']} · primer paso={first_skill}")
    print(f"motivo={review['reason']}")
    if review["decision"] != "revise" or first_skill != "scan_for_table":
        raise SystemExit(
            "FALLA: el modelo no eligió la recuperación disponible"
        )
    print("OK: la revisión reemplazó la vista fija por un barrido activo")


if __name__ == "__main__":
    main()
