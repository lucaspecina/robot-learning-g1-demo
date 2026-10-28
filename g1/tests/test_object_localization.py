"""Pruebas para asociar el objeto con la mesa elegida."""
import unittest

from object_localization import select_object_near_table


class ObjectLocalizationTests(unittest.TestCase):
    def setUp(self):
        self.table = {
            "x": 3.65,
            "y": 2.60,
            "z": 0.54,
            "coordinate_frame": "map",
        }

    def test_selects_the_object_on_the_chosen_table(self):
        nearby = {
            "class_id": "transport_object",
            "confidence": 0.51,
            "x": 3.98,
            "y": 2.61,
            "z": 0.76,
            "coordinate_frame": "map",
        }
        other_table = {
            **nearby,
            "confidence": 0.90,
            "x": 4.0,
            "y": -2.6,
        }

        selected = select_object_near_table(
            [other_table, nearby],
            self.table,
        )

        self.assertIs(selected, nearby)

    def test_rejects_background_and_wrong_coordinate_frame(self):
        candidates = [
            {
                "class_id": "transport_object",
                "confidence": 0.8,
                "x": 5.0,
                "y": 2.6,
                "z": 0.8,
                "coordinate_frame": "map",
            },
            {
                "class_id": "transport_object",
                "confidence": 0.8,
                "x": 3.9,
                "y": 2.6,
                "z": 0.8,
                "coordinate_frame": "camera",
            },
        ]

        self.assertIsNone(
            select_object_near_table(candidates, self.table)
        )

    def test_prefers_nearest_then_confidence(self):
        low = {
            "class_id": "transport_object",
            "confidence": 0.4,
            "x": 3.9,
            "y": 2.6,
            "z": 0.8,
            "coordinate_frame": "map",
        }
        high = {**low, "confidence": 0.7}

        self.assertIs(
            select_object_near_table([low, high], self.table),
            high,
        )


if __name__ == "__main__":
    unittest.main()
