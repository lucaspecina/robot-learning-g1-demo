"""Pruebas de la pose desde la que se mira una referencia medida."""
import math
import unittest

from observation_geometry import compute_observation_pose


class ObservationGeometryTest(unittest.TestCase):
    def test_stays_on_observed_side_and_faces_target(self):
        pose = compute_observation_pose(
            observer_x=0.0,
            observer_y=0.0,
            target_x=0.0,
            target_y=2.5,
            standoff_m=1.2,
        )

        self.assertAlmostEqual(pose.x, 0.0)
        self.assertAlmostEqual(pose.y, 1.3)
        self.assertAlmostEqual(pose.yaw, math.pi / 2)

    def test_rejects_target_too_close(self):
        with self.assertRaisesRegex(ValueError, "demasiado cerca"):
            compute_observation_pose(
                observer_x=0.0,
                observer_y=0.0,
                target_x=0.1,
                target_y=0.1,
                standoff_m=1.2,
            )


if __name__ == "__main__":
    unittest.main()
