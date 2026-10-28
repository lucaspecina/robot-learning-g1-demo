"""Pruebas de que el signo de inclinación coincide con lo documentado."""
import unittest

from camera_geometry import horizontal_field_of_view_deg, optical_forward


class CameraGeometryTest(unittest.TestCase):
    def test_zero_pitch_looks_forward(self):
        x, y, z = optical_forward(0.0)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(z, 0.0)

    def test_positive_pitch_really_looks_down(self):
        x, y, z = optical_forward(20.0)
        self.assertGreater(x, 0.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertLess(z, 0.0)

    def test_official_focal_length_widens_the_view(self):
        previous = horizontal_field_of_view_deg(18.0, 20.955)
        official = horizontal_field_of_view_deg(7.6, 20.955)
        self.assertAlmostEqual(previous, 60.4, places=1)
        self.assertAlmostEqual(official, 108.1, places=1)


if __name__ == "__main__":
    unittest.main()
