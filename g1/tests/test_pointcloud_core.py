import math
import unittest

import numpy as np

from pointcloud_core import points_in_box, transform_points


class PointcloudCoreTest(unittest.TestCase):
    def test_transform_applies_rotation_then_translation(self):
        half_turn = math.sqrt(0.5)
        transformed = transform_points(
            [[1.0, 0.0, 0.0]],
            [2.0, 3.0, 4.0],
            [0.0, 0.0, half_turn, half_turn],
        )
        np.testing.assert_allclose(transformed, [[2.0, 4.0, 4.0]])

    def test_points_in_box_includes_boundaries(self):
        selected = points_in_box(
            [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0], [1.1, 0.5, 0.5]],
            ([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]),
        )
        self.assertEqual(selected.shape, (2, 3))

    def test_transform_rejects_null_quaternion(self):
        with self.assertRaisesRegex(ValueError, "no puede ser nulo"):
            transform_points([[0.0, 0.0, 0.0]], [0.0, 0.0, 0.0], [0, 0, 0, 0])


if __name__ == "__main__":
    unittest.main()
