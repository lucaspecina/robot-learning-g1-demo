#!/usr/bin/env python3
"""Pruebas de cobertura y selección de candidatos del barrido visual."""

import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from active_search import local_table_candidate, make_scan_pattern


class ScanPatternTest(unittest.TestCase):
    def test_wide_g1_camera_covers_room_with_five_views(self):
        pattern = make_scan_pattern(108.1, 30.0)

        self.assertEqual(pattern.view_count, 5)
        self.assertAlmostEqual(
            math.degrees(pattern.turn_increment_rad),
            72.0,
        )
        self.assertAlmostEqual(pattern.actual_overlap_deg, 36.1)
        self.assertAlmostEqual(
            pattern.view_count * pattern.turn_increment_rad,
            2.0 * math.pi,
        )

    def test_rejects_overlap_that_would_make_scanning_impossible(self):
        with self.assertRaises(ValueError):
            make_scan_pattern(90.0, 90.0)


class LocalCandidateTest(unittest.TestCase):
    def setUp(self):
        self.status = {
            "state": "complete",
            "detections": ["reloj", "mesa_roja"],
            "frame_ref": {
                "topic": "/g1/perception/evidence/compressed",
                "sec": 10,
                "nanosec": 20,
            },
        }

    def test_accepts_only_the_selected_color(self):
        self.assertIs(
            local_table_candidate(self.status, "red_table"),
            self.status,
        )
        self.assertIsNone(
            local_table_candidate(self.status, "blue_table")
        )

    def test_rejects_candidate_without_exact_frame_reference(self):
        self.status.pop("frame_ref")

        self.assertIsNone(
            local_table_candidate(self.status, "red_table")
        )

    def test_color_cue_can_trigger_the_expensive_confirmation(self):
        self.status["detections"] = []
        self.status["color_pixels"] = {"red": 180, "blue": 2}

        self.assertIs(
            local_table_candidate(
                self.status,
                "red_table",
                minimum_color_pixels=100,
            ),
            self.status,
        )
        self.assertIsNone(
            local_table_candidate(
                self.status,
                "blue_table",
                minimum_color_pixels=100,
            )
        )


if __name__ == "__main__":
    unittest.main()
