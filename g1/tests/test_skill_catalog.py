"""Pruebas del catálogo que recibe el planificador remoto."""
import unittest

from mission_contract import build_demo_plan, validate_plan
from skill_catalog import (
    INITIAL_WORLD_FACTS,
    SKILL_CATALOG,
    initial_world_facts_for_profile,
    skill_catalog_for_profile,
    skill_catalog_for_model,
)


class SkillCatalogTest(unittest.TestCase):
    def test_every_skill_explains_its_contract(self):
        for skill in SKILL_CATALOG:
            self.assertTrue(skill["name"])
            self.assertTrue(skill["description"])
            self.assertIn(skill["availability"], {"ready", "placeholder"})
            self.assertTrue(skill["variants"])
            for variant in skill["variants"]:
                self.assertIn("argument", variant)
                self.assertTrue(variant["argument_description"])
                self.assertIsInstance(variant["preconditions"], list)
                self.assertIsInstance(variant["effects"], list)

    def test_current_demo_plan_respects_preconditions(self):
        validated = validate_plan(
            build_demo_plan(),
            skill_catalog=SKILL_CATALOG,
            initial_facts=INITIAL_WORLD_FACTS,
        )

        self.assertEqual(len(validated), 14)
        self.assertEqual(
            next(
                step
                for step in validated
                if step["id"] == "find_object"
            )["availability"],
            "ready",
        )
        self.assertEqual(
            next(
                step
                for step in validated
                if step["id"] == "approach_table"
            )["availability"],
            "ready",
        )
        self.assertEqual(
            next(
                step
                for step in validated
            if step["id"] == "align_with_table"
            )["availability"],
            "ready",
        )
        self.assertEqual(
            next(
                step
                for step in validated
                if step["id"] == "attach_payload"
            )["availability"],
            "ready",
        )

    def test_rejects_a_skill_invented_by_the_model(self):
        with self.assertRaisesRegex(ValueError, "skill no permitida"):
            validate_plan(
                [
                    {
                        "id": "fly",
                        "skill": "fly",
                        "argument": None,
                        "label": "Volar",
                    }
                ],
                skill_catalog=SKILL_CATALOG,
                initial_facts=INITIAL_WORLD_FACTS,
            )

    def test_rejects_a_step_before_its_precondition(self):
        with self.assertRaisesRegex(ValueError, "clock_confirmed"):
            validate_plan(
                [
                    {
                        "id": "read_clock",
                        "skill": "read_clock",
                        "argument": None,
                        "label": "Leer la hora",
                    }
                ],
                skill_catalog=SKILL_CATALOG,
                initial_facts=INITIAL_WORLD_FACTS,
            )

    def test_model_receives_a_copy(self):
        copy = skill_catalog_for_model()
        copy[0]["description"] = "mutado"

        self.assertNotEqual(copy[0]["description"], SKILL_CATALOG[0]["description"])

    def test_deployment_rehearsal_removes_simulation_only_skill(self):
        catalog = skill_catalog_for_profile("deployment_rehearsal")

        self.assertNotIn(
            "attach_payload",
            {skill["name"] for skill in catalog},
        )
        self.assertIn(
            "grasp_object",
            {skill["name"] for skill in catalog},
        )

    def test_deployment_rehearsal_must_measure_clock_position(self):
        facts = initial_world_facts_for_profile("deployment_rehearsal")

        self.assertEqual(facts, ["robot_pose_known"])
        clock_prefix = build_demo_plan()[:3]
        validated = validate_plan(
            clock_prefix,
            skill_catalog=skill_catalog_for_profile(
                "deployment_rehearsal"
            ),
            initial_facts=facts,
        )
        self.assertEqual(validated[1]["skill"], "find_clock")

        plan_without_measurement = [
            step
            for step in clock_prefix
            if step["skill"] != "find_clock"
        ]
        with self.assertRaisesRegex(ValueError, "clock_location_known"):
            validate_plan(
                plan_without_measurement,
                skill_catalog=skill_catalog_for_profile(
                    "deployment_rehearsal"
                ),
                initial_facts=facts,
            )

    def test_unknown_profile_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "perfil de operación inválido"):
            skill_catalog_for_profile("deploymet_typo")


if __name__ == "__main__":
    unittest.main()
