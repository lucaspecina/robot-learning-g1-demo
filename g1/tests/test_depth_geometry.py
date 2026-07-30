import unittest

import numpy as np

from depth_geometry import colored_table_point
from perception_core import ImageBox


class DepthGeometryTests(unittest.TestCase):
    def test_measures_colored_surface_and_ignores_background(self):
        rgb = np.full((80, 100, 3), 220, dtype=np.uint8)
        depth = np.full((80, 100), 8.0, dtype=np.float32)
        rgb[30:60, 40:70] = (30, 40, 180)
        depth[30:60, 40:70] = 2.0
        intrinsics = np.array(
            [[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]]
        )

        point = colored_table_point(
            rgb,
            depth,
            intrinsics,
            ImageBox(30, 20, 80, 70),
        )

        self.assertEqual(point.color, "blue")
        self.assertAlmostEqual(point.forward_m, 2.0)
        self.assertAlmostEqual(point.right_m, 0.09)
        self.assertAlmostEqual(point.down_m, 0.09)
        self.assertEqual(point.sample_count, 900)

    def test_rejects_box_without_enough_colored_depth(self):
        rgb = np.full((20, 20, 3), 220, dtype=np.uint8)
        depth = np.full((20, 20), np.inf, dtype=np.float32)
        intrinsics = np.eye(3)

        with self.assertRaisesRegex(ValueError, "píxeles"):
            colored_table_point(
                rgb,
                depth,
                intrinsics,
                ImageBox(0, 0, 20, 20),
            )


if __name__ == "__main__":
    unittest.main()
