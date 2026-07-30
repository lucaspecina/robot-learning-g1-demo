"""Pruebas del catálogo que recibe el planificador remoto."""
import unittest

from mission_contract import build_demo_plan, validate_plan
from skill_catalog import (
    INITIAL_WORLD_FACTS,
    SKILL_CATALOG,
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

        self.assertEqual(len(validated), 13)
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
            "placeholder",
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


if __name__ == "__main__":
    unittest.main()
