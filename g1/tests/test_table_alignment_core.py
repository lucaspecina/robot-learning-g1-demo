#!/usr/bin/env python3
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from table_alignment_core import (  # noqa: E402
    AlignmentPose,
    AlignmentTarget,
    TableAlignmentController,
    TargetFilter,
)


def pose(x=0.0, yaw=0.0, linear=0.0, angular=0.0):
    return AlignmentPose(
        x=x,
        y=0.0,
        yaw=yaw,
        linear_speed=linear,
        angular_speed=angular,
    )


class TargetFilterTest(unittest.TestCase):
    def test_uses_first_measurement_without_delay(self):
        target_filter = TargetFilter(0.1)
        self.assertEqual(
            target_filter.update(AlignmentTarget(2.0, 3.0)),
            AlignmentTarget(2.0, 3.0),
        )

    def test_filters_later_measurements(self):
        target_filter = TargetFilter(0.1)
        target_filter.update(AlignmentTarget(2.0, 3.0))
        filtered = target_filter.update(AlignmentTarget(3.0, 1.0))
        self.assertAlmostEqual(filtered.x, 2.1)
        self.assertAlmostEqual(filtered.y, 2.8)


class TableAlignmentControllerTest(unittest.TestCase):
    def setUp(self):
        self.controller = TableAlignmentController(stable_duration_s=1.5)
        self.target = AlignmentTarget(2.0, 0.0)

    def test_turns_before_advancing(self):
        command = self.controller.step(
            pose(yaw=math.radians(30.0)),
            self.target,
            0.0,
        )
        self.assertEqual(command.phase, "turning")
        self.assertEqual(command.linear_x, 0.0)
        self.assertLess(command.angular_z, 0.0)

    def test_approaches_slowly(self):
        command = self.controller.step(pose(), self.target, 0.0)
        self.assertEqual(command.phase, "approaching")
        self.assertGreater(command.linear_x, 0.0)
        self.assertLessEqual(command.linear_x, 0.12)

    def test_uses_measured_minimum_near_the_goal(self):
        command = self.controller.step(pose(x=1.19), self.target, 0.0)
        self.assertAlmostEqual(command.linear_x, 0.08)

    def test_backs_up_when_too_close(self):
        command = self.controller.step(pose(x=1.5), self.target, 0.0)
        self.assertEqual(command.phase, "backing_up")
        self.assertLess(command.linear_x, 0.0)

    def test_requires_pose_speed_and_time(self):
        moving = self.controller.step(
            pose(x=1.30, linear=0.03),
            self.target,
            0.0,
        )
        first_still = self.controller.step(pose(x=1.30), self.target, 1.0)
        almost = self.controller.step(pose(x=1.30), self.target, 2.4)
        done = self.controller.step(pose(x=1.30), self.target, 2.5)
        self.assertEqual(moving.phase, "settling")
        self.assertFalse(first_still.stable)
        self.assertFalse(almost.stable)
        self.assertTrue(done.stable)

    def test_leaving_tolerance_resets_stability(self):
        self.controller.step(pose(x=1.30), self.target, 0.0)
        self.controller.step(pose(x=1.20), self.target, 1.0)
        restarted = self.controller.step(pose(x=1.30), self.target, 2.0)
        self.assertFalse(restarted.stable)


if __name__ == "__main__":
    unittest.main()
