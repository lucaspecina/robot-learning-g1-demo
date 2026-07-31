#!/usr/bin/env python3
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from table_alignment_core import (  # noqa: E402
    AlignmentPose,
    AlignmentTarget,
    AlignmentTargetTracker,
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


class AlignmentTargetTrackerTest(unittest.TestCase):
    def test_fixed_pose_is_immediately_available_and_never_expires(self):
        tracker = AlignmentTargetTracker(
            use_external_detection=False,
            detection_timeout_s=1.0,
        )
        requested = AlignmentTarget(3.6, 3.0)
        tracker.reset(requested)

        initial = tracker.snapshot(0.0)
        much_later = tracker.snapshot(120.0)

        self.assertEqual(initial.target, requested)
        self.assertEqual(initial.source, "requested_pose")
        self.assertFalse(initial.waiting_for_external)
        self.assertFalse(much_later.stale)
        self.assertFalse(
            tracker.update_external(AlignmentTarget(9.0, 9.0), 121.0)
        )
        self.assertEqual(tracker.snapshot(122.0).target, requested)

    def test_external_mode_waits_for_and_expires_live_measurements(self):
        tracker = AlignmentTargetTracker(
            use_external_detection=True,
            detection_timeout_s=1.0,
        )
        tracker.reset(AlignmentTarget(3.6, 3.0))

        self.assertTrue(tracker.snapshot(0.0).waiting_for_external)
        self.assertTrue(
            tracker.update_external(AlignmentTarget(3.7, 3.1), 0.5)
        )
        fresh = tracker.snapshot(1.0)
        stale = tracker.snapshot(1.6)

        self.assertEqual(fresh.source, "external_detection")
        self.assertEqual(fresh.detection_count, 1)
        self.assertFalse(fresh.stale)
        self.assertTrue(stale.stale)


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
        self.assertLessEqual(command.linear_x, 0.15)

    def test_uses_measured_minimum_near_the_goal(self):
        command = self.controller.step(pose(x=1.19), self.target, 0.0)
        self.assertAlmostEqual(command.linear_x, 0.10)

    def test_backs_up_when_too_close(self):
        command = self.controller.step(pose(x=1.5), self.target, 0.0)
        self.assertEqual(command.phase, "backing_up")
        self.assertLess(command.linear_x, 0.0)

    def test_requires_pose_speed_and_time(self):
        moving = self.controller.step(
            pose(x=1.30, linear=0.11),
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
