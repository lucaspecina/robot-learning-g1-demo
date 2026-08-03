"""Pruebas de consistencia espacial sin ROS ni simulador."""
import unittest

from landmark_geometry import select_consistent_landmark


def sample(x, y, z, received_at, frame):
    return {
        "class_id": "clock",
        "confidence": 0.8,
        "x": x,
        "y": y,
        "z": z,
        "coordinate_frame": "map",
        "frame_ref": {"stamp_ns": frame},
        "received_at": received_at,
    }


class LandmarkGeometryTest(unittest.TestCase):
    def test_accepts_two_close_measurements(self):
        estimate = select_consistent_landmark(
            [sample(1.0, 2.0, 1.5, 9.0, 1), sample(1.04, 2.0, 1.51, 9.5, 2)],
            class_id="clock",
            now=10.0,
            max_age_s=3.0,
        )

        self.assertIsNotNone(estimate)
        self.assertEqual(estimate["sample_count"], 2)
        self.assertAlmostEqual(estimate["x"], 1.02)

    def test_rejects_one_isolated_measurement(self):
        estimate = select_consistent_landmark(
            [sample(1.0, 2.0, 1.5, 9.0, 1)],
            class_id="clock",
            now=10.0,
            max_age_s=3.0,
        )

        self.assertIsNone(estimate)

    def test_does_not_count_the_same_frame_twice(self):
        repeated = sample(1.0, 2.0, 1.5, 9.0, 1)
        estimate = select_consistent_landmark(
            [repeated, dict(repeated)],
            class_id="clock",
            now=10.0,
            max_age_s=3.0,
        )

        self.assertIsNone(estimate)

    def test_ignores_an_outlier_and_keeps_the_consistent_group(self):
        estimate = select_consistent_landmark(
            [
                sample(4.0, -2.0, 1.5, 9.0, 1),
                sample(1.0, 2.0, 1.5, 9.2, 2),
                sample(1.03, 2.01, 1.5, 9.4, 3),
            ],
            class_id="clock",
            now=10.0,
            max_age_s=3.0,
        )

        self.assertIsNotNone(estimate)
        self.assertEqual(estimate["sample_count"], 2)
        self.assertLess(estimate["x"], 1.1)

    def test_rejects_old_or_implausibly_low_measurements(self):
        estimate = select_consistent_landmark(
            [
                sample(1.0, 2.0, 0.1, 9.5, 1),
                sample(1.0, 2.0, 1.5, 2.0, 2),
            ],
            class_id="clock",
            now=10.0,
            max_age_s=3.0,
        )

        self.assertIsNone(estimate)


if __name__ == "__main__":
    unittest.main()
