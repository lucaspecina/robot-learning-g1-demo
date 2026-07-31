"""Pruebas de la simetría que evita apoyar el bulto contra el torso."""
import unittest

import numpy as np

from arm_control import POSES


class ArmControlTest(unittest.TestCase):
    def test_transport_candidate_is_bilaterally_mirrored(self):
        left = POSES["transporte"][:7]
        right = POSES["transporte"][7:]

        np.testing.assert_allclose(left[[0, 3, 5]], right[[0, 3, 5]])
        np.testing.assert_allclose(
            left[[1, 2, 4, 6]],
            -right[[1, 2, 4, 6]],
        )


if __name__ == "__main__":
    unittest.main()
