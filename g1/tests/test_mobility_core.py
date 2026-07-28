#!/usr/bin/env python3
"""Pruebas del contrato de autoridad sin necesitar ROS ni Isaac."""

import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from mobility_core import MobilityAuthority, MobilitySource, ZERO_VELOCITY


class MobilityAuthorityTest(unittest.TestCase):
    def setUp(self):
        self.authority = MobilityAuthority(
            lease_timeout_s=1.0,
            command_timeout_s=0.25,
        )

    def test_stand_is_the_safe_default(self):
        self.assertEqual(self.authority.owner, MobilitySource.STAND)
        self.assertEqual(self.authority.tick(0.0), ZERO_VELOCITY)

    def test_only_owner_command_reaches_output(self):
        result = self.authority.acquire(MobilitySource.NAVIGATION, "go_to", 1.0)
        self.assertTrue(result.accepted)
        self.assertFalse(
            self.authority.submit_command(MobilitySource.TEST, (0.4, 0.0, 0.0), 1.1)
        )
        self.assertTrue(
            self.authority.submit_command(
                MobilitySource.NAVIGATION, (0.2, 0.0, 0.1), 1.1
            )
        )
        self.assertEqual(self.authority.tick(1.2), (0.2, 0.0, 0.1))
        self.assertEqual(self.authority.rejected_commands, 1)

    def test_lease_expiry_returns_to_stand(self):
        self.authority.acquire(MobilitySource.TEST, "checks", 2.0)
        self.authority.submit_command(MobilitySource.TEST, (0.3, 0.0, 0.0), 2.1)
        self.assertEqual(self.authority.tick(2.2), (0.3, 0.0, 0.0))
        self.assertEqual(self.authority.tick(3.2), ZERO_VELOCITY)
        self.assertEqual(self.authority.owner, MobilitySource.STAND)

    def test_navigation_cannot_take_control_from_test(self):
        self.authority.acquire(MobilitySource.TEST, "checks", 4.0)
        result = self.authority.acquire(MobilitySource.NAVIGATION, "go_to", 4.1)
        self.assertFalse(result.accepted)
        self.assertEqual(self.authority.owner, MobilitySource.TEST)

    def test_manual_can_preempt_navigation(self):
        self.authority.acquire(MobilitySource.NAVIGATION, "go_to", 5.0)
        result = self.authority.acquire(MobilitySource.MANUAL, "operator", 5.1)
        self.assertTrue(result.accepted)
        self.assertEqual(self.authority.owner, MobilitySource.MANUAL)

    def test_wrong_requester_cannot_release(self):
        self.authority.acquire(MobilitySource.TEST, "checks", 6.0)
        result = self.authority.release(MobilitySource.TEST, "otro")
        self.assertFalse(result.accepted)
        self.assertEqual(self.authority.owner, MobilitySource.TEST)

    def test_owner_release_returns_to_stand(self):
        self.authority.acquire(MobilitySource.NAVIGATION, "go_to", 7.0)
        result = self.authority.release(
            MobilitySource.NAVIGATION,
            "go_to",
            "objetivo terminado",
        )
        self.assertTrue(result.accepted)
        self.assertEqual(self.authority.owner, MobilitySource.STAND)
        self.assertEqual(self.authority.tick(7.1), ZERO_VELOCITY)

    def test_stale_command_is_zero_before_lease_expires(self):
        self.authority.acquire(MobilitySource.TEST, "checks", 8.0)
        self.authority.submit_command(MobilitySource.TEST, (0.3, 0.0, 0.0), 8.1)
        self.assertEqual(self.authority.tick(8.4), ZERO_VELOCITY)
        self.assertEqual(self.authority.owner, MobilitySource.TEST)


if __name__ == "__main__":
    unittest.main()
