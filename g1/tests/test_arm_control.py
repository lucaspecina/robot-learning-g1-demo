"""Pruebas de la simetría que evita apoyar el bulto contra el torso."""
import unittest

import numpy as np

from arm_control import ARM_JOINTS, POSES, arm_tracking_tolerances


class ArmControlTest(unittest.TestCase):
    def test_transport_candidate_is_bilaterally_mirrored(self):
        left = POSES["transporte"][:7]
        right = POSES["transporte"][7:]

        np.testing.assert_allclose(left[[0, 3, 5]], right[[0, 3, 5]])
        np.testing.assert_allclose(
            left[[1, 2, 4, 6]],
            -right[[1, 2, 4, 6]],
        )

    def test_tracking_tolerance_matches_measured_active_limits(self):
        tolerances = arm_tracking_tolerances(ARM_JOINTS)

        for name, tolerance in zip(ARM_JOINTS, tolerances):
            expected = (
                0.05
                if "_shoulder_" in name or "_wrist_" in name
                else 0.03
            )
            self.assertAlmostEqual(float(tolerance), expected)


if __name__ == "__main__":
    unittest.main()
