"""Pruebas del estado de misión que consume el tablero."""
import unittest

from mission_contract import (
    MissionTracker,
    build_demo_plan,
    validate_plan,
)


class MissionContractTest(unittest.TestCase):
    def setUp(self):
        self.now = [100.0]
        self.published = []
        self.tracker = MissionTracker(
            publisher=self.published.append,
            clock=lambda: self.now[0],
            id_factory=lambda: "mission-test",
        )

    def test_demo_plan_matches_current_mission(self):
        steps = validate_plan(build_demo_plan())
        ids = [step["id"] for step in steps]

        self.assertIn("choose_table", ids)
        self.assertIn("scan_for_table", ids)
        self.assertIn("find_object", ids)
        self.assertIn("align_with_table", ids)
        self.assertIn("attach_payload", ids)
        self.assertIn("return_home", ids)
        self.assertNotIn("grasp_object", ids)
        self.assertNotIn("search_person", ids)

    def test_tracks_a_successful_step_without_parsing_logs(self):
        self.tracker.begin("Traé el objeto", "rules")
        self.tracker.set_plan(build_demo_plan())
        self.now[0] = 101.0
        self.tracker.start_step("remember_home")
        self.now[0] = 102.0
        state = self.tracker.finish_step(
            "remember_home",
            "home guardado en (0.00, 0.00)",
        )

        step = state["steps"][0]
        self.assertEqual(state["mission_id"], "mission-test")
        self.assertEqual(state["state"], "running")
        self.assertEqual(step["state"], "succeeded")
        self.assertEqual(step["result"], "home guardado en (0.00, 0.00)")
        self.assertGreaterEqual(len(self.published), 4)

    def test_preserves_measured_evidence_from_a_step(self):
        self.tracker.begin("Traé el objeto", "rules")
        self.tracker.set_plan(build_demo_plan())
        self.tracker.start_step("navigate_to_clock")
        state = self.tracker.finish_step(
            "navigate_to_clock",
            "llegó al reloj",
            {"distance_remaining_m": 0.073},
        )

        step = next(
            item
            for item in state["steps"]
            if item["id"] == "navigate_to_clock"
        )
        self.assertEqual(
            step["measurements"],
            {"distance_remaining_m": 0.073},
        )

    def test_blocked_step_stops_the_mission_honestly(self):
        self.tracker.begin("Traé el objeto", "rules")
        self.tracker.set_plan(build_demo_plan())
        self.tracker.start_step("align_with_table")
        state = self.tracker.stop_step(
            "align_with_table",
            "la alineación fina todavía no está implementada",
            blocked=True,
        )

        self.assertEqual(state["state"], "blocked")
        self.assertEqual(
            next(
                step
                for step in state["steps"]
                if step["id"] == "align_with_table"
            )["state"],
            "blocked",
        )

    def test_failed_step_can_be_reviewed_and_retried_once(self):
        self.tracker.begin("Traé el objeto", "rules")
        self.tracker.set_plan(build_demo_plan())
        self.tracker.start_step("navigate_to_clock")
        state = self.tracker.record_step_failure(
            "navigate_to_clock",
            "sin progreso",
            measurements={"distance_remaining_m": 1.0},
        )

        self.assertEqual(state["state"], "running")
        failed_step = next(
            step
            for step in state["steps"]
            if step["id"] == "navigate_to_clock"
        )
        self.assertEqual(failed_step["attempts"], 1)
        self.assertEqual(len(failed_step["attempt_history"]), 1)

        self.tracker.retry_step("navigate_to_clock")
        retried = self.tracker.start_step("navigate_to_clock")
        retried_step = next(
            step
            for step in retried["steps"]
            if step["id"] == "navigate_to_clock"
        )
        self.assertEqual(retried_step["attempts"], 2)

    def test_revision_preserves_history_and_replaces_only_pending(self):
        self.tracker.begin("Volvé a home", "rules")
        self.tracker.set_plan(build_demo_plan())
        self.tracker.start_step("remember_home")
        self.tracker.finish_step("remember_home", "home guardado")
        state = self.tracker.replace_pending_steps(
            [
                {
                    "id": "revised_return_home",
                    "skill": "navigate_to",
                    "argument": "home",
                    "label": "Volver al inicio",
                }
            ],
            skill_catalog=[
                {
                    "name": "navigate_to",
                    "availability": "ready",
                    "variants": [
                        {
                            "argument": "home",
                            "preconditions": ["home_saved"],
                            "effects": ["at_home"],
                        }
                    ],
                }
            ],
            current_facts=["home_saved"],
        )

        self.assertEqual(
            [step["id"] for step in state["steps"]],
            ["remember_home", "revised_return_home"],
        )
        self.assertEqual(state["steps"][0]["state"], "succeeded")
        self.assertEqual(state["steps"][1]["state"], "pending")

    def test_revision_cannot_delete_the_original_mission_goal(self):
        catalog = [
            {
                "name": "scan",
                "availability": "ready",
                "variants": [
                    {
                        "argument": None,
                        "preconditions": [],
                        "effects": ["target_found"],
                    }
                ],
            },
            {
                "name": "return_home",
                "availability": "ready",
                "variants": [
                    {
                        "argument": None,
                        "preconditions": ["target_found"],
                        "effects": ["at_home"],
                    }
                ],
            },
        ]
        original = [
            {"id": "scan", "skill": "scan", "argument": None, "label": "Buscar"},
            {
                "id": "return_home",
                "skill": "return_home",
                "argument": None,
                "label": "Volver",
            },
        ]
        self.tracker.begin("Buscá y volvé", "llm")
        self.tracker.set_plan(original, skill_catalog=catalog)

        with self.assertRaisesRegex(ValueError, "at_home"):
            self.tracker.replace_pending_steps(
                [
                    {
                        "id": "scan_recovery",
                        "skill": "scan",
                        "argument": None,
                        "label": "Buscar otra vez",
                    }
                ],
                skill_catalog=catalog,
                current_facts=[],
            )

    def test_completion_requires_measured_mission_goals(self):
        catalog = [
            {
                "name": "return_home",
                "availability": "ready",
                "variants": [
                    {
                        "argument": None,
                        "preconditions": [],
                        "effects": ["at_home"],
                    }
                ],
            }
        ]
        self.tracker.begin("Volvé", "llm")
        self.tracker.set_plan(
            [
                {
                    "id": "return_home",
                    "skill": "return_home",
                    "argument": None,
                    "label": "Volver",
                }
            ],
            skill_catalog=catalog,
        )
        self.tracker.start_step("return_home")
        self.tracker.finish_step("return_home", "llegó")

        with self.assertRaisesRegex(ValueError, "at_home"):
            self.tracker.complete("listo", achieved_facts=[])

    def test_records_verifiable_decision(self):
        self.tracker.begin("Traé el objeto", "rules")
        state = self.tracker.set_decision(
            "el reloj marca 09:00",
            "antes de 12:00",
            "buscar la mesa A roja",
        )

        self.assertEqual(
            state["decision"]["outcome"],
            "buscar la mesa A roja",
        )

    def test_can_report_a_planning_failure(self):
        self.tracker.begin("Traé el objeto", "llm")
        state = self.tracker.stop("el plan no pasó la validación")

        self.assertEqual(state["state"], "failed")
        self.assertEqual(state["error"], "el plan no pasó la validación")


if __name__ == "__main__":
    unittest.main()
