"""Pruebas del contrato que protege los cambios de masa del robot."""
import json
import unittest

from payload_core import (
    parse_payload_request,
    payload_geometry_measurements,
    payload_mass_values,
    select_payload_body_indices,
)


class PayloadCoreTest(unittest.TestCase):
    def test_accepts_a_bounded_attach_request(self):
        request = parse_payload_request(json.dumps({
            "request_id": "mission-1",
            "command": "attach",
            "mass_kg": 1.0,
        }))

        self.assertEqual(request["mass_kg"], 1.0)

    def test_rejects_an_unsafe_mass(self):
        with self.assertRaisesRegex(ValueError, "máximo experimental"):
            parse_payload_request(json.dumps({
                "request_id": "mission-1",
                "command": "attach",
                "mass_kg": 4.0,
            }))

    def test_prefers_hands_over_wrist_links(self):
        names = [
            "left_wrist_yaw_link",
            "right_wrist_yaw_link",
            "left_rubber_hand",
            "right_rubber_hand",
        ]

        self.assertEqual(select_payload_body_indices(names), [2, 3])

    def test_falls_back_to_wrist_links_when_hands_are_disabled(self):
        names = ["pelvis", "left_wrist_yaw_link", "right_wrist_yaw_link"]

        self.assertEqual(select_payload_body_indices(names), [1, 2])

    def test_repeated_orders_use_the_original_mass(self):
        baseline = [10.0, 0.4, 0.4]

        first = payload_mass_values(baseline, [1, 2], 1.0)
        second = payload_mass_values(baseline, [1, 2], 1.0)

        self.assertEqual(first, [10.0, 0.9, 0.9])
        self.assertEqual(second, first)

    def test_reports_visual_position_relative_to_pelvis(self):
        geometry = payload_geometry_measurements(
            [[0.4, 0.2, 1.0], [0.4, -0.2, 1.0]],
            [0.0, 0.0, 0.75],
        )

        self.assertEqual(geometry["wrist_separation_m"], 0.4)
        self.assertEqual(
            geometry["visual_position_world_m"],
            [0.4, 0.0, 1.0],
        )
        self.assertEqual(
            geometry["visual_offset_from_pelvis_world_m"],
            [0.4, 0.0, 0.25],
        )


if __name__ == "__main__":
    unittest.main()
