#!/usr/bin/env python3
"""Pruebas del cálculo de preaproximación sin mover el robot."""

import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from table_approach import (
    compute_table_staging_pose,
    next_table_approach_attempt,
)


ROOM = {
    "xmin": -1.5,
    "xmax": 5.5,
    "ymin": -3.5,
    "ymax": 4.0,
}


class TableApproachTest(unittest.TestCase):
    def test_stops_on_observation_line_and_faces_table(self):
        pose = compute_table_staging_pose(
            robot_x=1.0,
            robot_y=2.0,
            table_x=4.0,
            table_y=2.0,
            standoff_m=2.2,
            world_bounds=ROOM,
        )

        self.assertAlmostEqual(pose.x, 1.8)
        self.assertAlmostEqual(pose.y, 2.0)
        self.assertAlmostEqual(pose.yaw, 0.0)
        self.assertAlmostEqual(
            math.hypot(4.0 - pose.x, 2.0 - pose.y),
            2.2,
        )

    def test_preserves_standoff_on_a_diagonal_view(self):
        pose = compute_table_staging_pose(
            robot_x=1.0,
            robot_y=1.0,
            table_x=3.0,
            table_y=2.0,
            standoff_m=0.8,
            world_bounds=ROOM,
        )

        self.assertAlmostEqual(
            math.hypot(3.0 - pose.x, 2.0 - pose.y),
            0.8,
        )
        self.assertAlmostEqual(
            pose.yaw,
            math.atan2(2.0 - pose.y, 3.0 - pose.x),
        )

    def test_rejects_a_target_too_close_for_general_navigation(self):
        with self.assertRaisesRegex(ValueError, "demasiado cerca"):
            compute_table_staging_pose(
                robot_x=1.0,
                robot_y=1.0,
                table_x=1.1,
                table_y=1.1,
                standoff_m=2.2,
                world_bounds=ROOM,
            )

    def test_rejects_a_pose_near_a_wall(self):
        with self.assertRaisesRegex(ValueError, "pared"):
            compute_table_staging_pose(
                robot_x=-1.45,
                robot_y=0.0,
                table_x=-1.0,
                table_y=0.0,
                standoff_m=2.2,
                world_bounds=ROOM,
            )

    def test_limits_revised_plans_to_two_approaches(self):
        first, first_allowed = next_table_approach_attempt(0, 2)
        second, second_allowed = next_table_approach_attempt(first, 2)
        third, third_allowed = next_table_approach_attempt(second, 2)

        self.assertEqual((first, first_allowed), (1, True))
        self.assertEqual((second, second_allowed), (2, True))
        self.assertEqual((third, third_allowed), (3, False))

    def test_rejects_an_invalid_attempt_budget(self):
        with self.assertRaisesRegex(ValueError, "presupuesto"):
            next_table_approach_attempt(0, 0)


if __name__ == "__main__":
    unittest.main()
