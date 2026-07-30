import unittest

import numpy as np

from depth_geometry import colored_table_point, visible_object_point
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

    def test_measures_object_surface_without_using_background(self):
        depth = np.full((80, 100), 6.0, dtype=np.float32)
        depth[25:65, 40:60] = 2.4
        # Pocos píxeles del fondo dentro de la caja no deben mover el punto.
        depth[25:29, 40:60] = 5.0
        intrinsics = np.array(
            [[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]]
        )

        point = visible_object_point(
            depth,
            intrinsics,
            ImageBox(38, 22, 62, 68),
        )

        self.assertAlmostEqual(point.forward_m, 2.4, places=6)
        self.assertAlmostEqual(point.right_m, -0.012, places=6)
        self.assertAlmostEqual(point.down_m, 0.108, places=6)
        self.assertGreater(point.sample_count, 400)

    def test_rejects_sparse_object_depth(self):
        depth = np.full((30, 30), np.inf, dtype=np.float32)
        depth[12:15, 12:15] = 1.0

        with self.assertRaisesRegex(ValueError, "píxeles"):
            visible_object_point(
                depth,
                np.eye(3),
                ImageBox(5, 5, 25, 25),
            )


if __name__ == "__main__":
    unittest.main()
