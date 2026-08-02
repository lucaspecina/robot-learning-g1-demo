"""Pruebas del error angular medido en Isaac Sim 5.1."""
import unittest

from laser_scan_core import angle_max_for_count, ray_count_from_metadata


class LaserScanCoreTest(unittest.TestCase):
    def test_measured_isaac_metadata_claims_one_extra_ray(self):
        angle_min = -3.1415927410125732
        angle_max = 3.135702133178711
        increment = 0.005890486296266317
        self.assertEqual(
            ray_count_from_metadata(angle_min, angle_max, increment),
            1067,
        )

    def test_normalized_interval_matches_received_ray_count(self):
        angle_min = -3.1415927410125732
        increment = 0.005890486296266317
        corrected = angle_max_for_count(angle_min, increment, 1066)
        self.assertEqual(
            ray_count_from_metadata(angle_min, corrected, increment),
            1066,
        )


if __name__ == "__main__":
    unittest.main()
