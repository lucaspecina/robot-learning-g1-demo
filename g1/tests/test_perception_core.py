"""Pruebas de geometría y atributos de la percepción."""
import unittest

import numpy as np

from perception_core import (
    CLASS_NAMES,
    TABLE_CLASS_NAMES,
    ImageBox,
    bounded_box,
    classify_table_color,
    legacy_detection,
    padded_box,
)


class PerceptionCoreTest(unittest.TestCase):
    def test_table_labels_from_both_backends_share_one_contract(self):
        self.assertEqual(CLASS_NAMES["diningtable"], "mesa")
        self.assertEqual(CLASS_NAMES["dining table"], "mesa")
        self.assertIn("diningtable", TABLE_CLASS_NAMES)
        self.assertIn("dining table", TABLE_CLASS_NAMES)

    def test_box_is_clipped_to_image(self):
        self.assertEqual(
            bounded_box(5, 5, 20, 20, 12, 10),
            ImageBox(0, 0, 12, 10),
        )

    def test_padding_keeps_valid_bounds(self):
        box = padded_box(ImageBox(1, 2, 9, 8), 10, 10)
        self.assertEqual(box, ImageBox(0, 0, 10, 10))

    def test_table_color_is_read_only_inside_detected_box(self):
        image = np.zeros((20, 40, 3), dtype=np.uint8)
        image[:, :20] = (190, 25, 25)
        image[:, 20:] = (25, 40, 190)
        self.assertEqual(
            classify_table_color(image, ImageBox(0, 0, 20, 20)),
            "mesa_roja",
        )
        self.assertEqual(
            classify_table_color(image, ImageBox(20, 0, 40, 20)),
            "mesa_azul",
        )

    def test_legacy_detection_uses_normalized_geometry(self):
        result = legacy_detection(
            "clock",
            0.94,
            ImageBox(10, 20, 30, 60),
            100,
            100,
        )
        self.assertEqual(result["cx"], 0.2)
        self.assertEqual(result["area"], 0.08)
        self.assertEqual(result["confidence"], 0.94)


if __name__ == "__main__":
    unittest.main()
