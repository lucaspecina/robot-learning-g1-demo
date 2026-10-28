#!/usr/bin/env python3
"""Pruebas del navegador y de su criterio de progreso."""

import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from navigation_core import (
    NavigationController,
    NavigationGoal,
    NavigationPose,
    ProgressChecker,
    SpinController,
)


class NavigationControllerTest(unittest.TestCase):
    def setUp(self):
        self.controller = NavigationController()

    def test_turns_before_advancing_when_target_is_to_the_side(self):
        result = self.controller.step(
            NavigationPose(0.0, 0.0, 0.0),
            NavigationGoal(0.0, 2.0, None),
        )
        self.assertEqual(result.phase, "turning")
        self.assertEqual(result.linear_x, 0.0)
        self.assertGreater(result.angular_z, 0.0)

    def test_advances_when_aligned(self):
        result = self.controller.step(
            NavigationPose(0.0, 0.0, 0.0),
            NavigationGoal(2.0, 0.0, None),
        )
        self.assertEqual(result.phase, "moving")
        self.assertGreater(result.linear_x, 0.0)

    def test_finishes_position_and_then_orientation(self):
        goal = NavigationGoal(1.0, 0.0, math.pi / 2.0)
        turning = self.controller.step(
            NavigationPose(0.95, 0.0, 0.0),
            goal,
        )
        self.assertEqual(turning.phase, "final_turn")
        done = self.controller.step(
            NavigationPose(0.95, 0.0, math.pi / 2.0),
            goal,
        )
        self.assertTrue(done.goal_reached)

    def test_reacquires_position_if_the_final_turn_drifts_outside_tolerance(self):
        goal = NavigationGoal(0.0, 0.0, math.pi / 2.0)
        final_turn = self.controller.step(
            NavigationPose(0.05, 0.0, 0.0),
            goal,
        )
        drifted = self.controller.step(
            NavigationPose(0.15, 0.0, math.pi / 2.0),
            goal,
        )

        self.assertEqual(final_turn.phase, "final_turn")
        self.assertFalse(drifted.goal_reached)
        self.assertGreater(drifted.distance_remaining, 0.10)


class ProgressCheckerTest(unittest.TestCase):
    def setUp(self):
        self.checker = ProgressChecker(
            movement_radius_m=0.05,
            movement_angle_rad=math.radians(5.0),
            allowance_s=10.0,
        )

    def test_stationary_robot_fails_after_allowance(self):
        pose = NavigationPose(0.0, 0.0, 0.0)
        self.assertTrue(self.checker.update(pose, 0.0))
        self.assertTrue(self.checker.update(pose, 10.0))
        self.assertFalse(self.checker.update(pose, 10.1))

    def test_translation_resets_allowance(self):
        self.checker.update(NavigationPose(0.0, 0.0, 0.0), 0.0)
        self.assertTrue(
            self.checker.update(NavigationPose(0.06, 0.0, 0.0), 9.0)
        )
        self.assertTrue(
            self.checker.update(NavigationPose(0.06, 0.0, 0.0), 18.0)
        )

    def test_rotation_counts_as_progress(self):
        self.checker.update(NavigationPose(0.0, 0.0, 0.0), 0.0)
        self.assertTrue(
            self.checker.update(
                NavigationPose(0.0, 0.0, math.radians(6.0)),
                9.0,
            )
        )


class SpinControllerTest(unittest.TestCase):
    def test_keeps_positive_direction_across_angle_wrap(self):
        controller = SpinController(math.radians(270.0))
        controller.reset(math.radians(170.0))

        first = controller.step(math.radians(-170.0))
        second = controller.step(math.radians(-80.0))

        self.assertGreater(first.angular_distance_traveled, 0.0)
        self.assertAlmostEqual(
            math.degrees(second.angular_distance_traveled),
            110.0,
            places=5,
        )
        self.assertGreater(second.angular_z, 0.0)

    def test_respects_negative_requested_direction(self):
        controller = SpinController(-math.pi / 2.0)
        controller.reset(0.0)

        command = controller.step(0.0)

        self.assertLess(command.angular_z, 0.0)
        self.assertFalse(command.goal_reached)

    def test_finishes_inside_biped_tolerance(self):
        controller = SpinController(math.pi / 2.0)
        controller.reset(0.0)

        command = controller.step(math.radians(86.0))

        self.assertTrue(command.goal_reached)
        self.assertEqual(command.angular_z, 0.0)


if __name__ == "__main__":
    unittest.main()
