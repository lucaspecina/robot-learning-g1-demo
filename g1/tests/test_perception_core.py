"""Pruebas de geometría y atributos de la percepción."""
import unittest

import numpy as np

from perception_core import (
    CLASS_NAMES,
    CLOCK_CLASS_NAMES,
    TABLE_CLASS_NAMES,
    TRANSPORT_OBJECT_CLASS_NAMES,
    ImageBox,
    bounded_box,
    classify_table_color,
    color_pixel_counts,
    is_clock_class,
    legacy_detection,
    merge_source_detections,
    padded_box,
)


class PerceptionCoreTest(unittest.TestCase):
    def test_table_labels_from_both_backends_share_one_contract(self):
        self.assertEqual(CLASS_NAMES["diningtable"], "mesa")
        self.assertEqual(CLASS_NAMES["dining table"], "mesa")
        self.assertIn("diningtable", TABLE_CLASS_NAMES)
        self.assertIn("dining table", TABLE_CLASS_NAMES)
        self.assertIn("a red table", TABLE_CLASS_NAMES)
        self.assertIn("a table", TABLE_CLASS_NAMES)

    def test_cup_and_bottle_share_the_transport_role(self):
        self.assertEqual(CLASS_NAMES["cup"], "objeto")
        self.assertIn("cup", TRANSPORT_OBJECT_CLASS_NAMES)
        self.assertIn("bottle", TRANSPORT_OBJECT_CLASS_NAMES)

    def test_clock_labels_from_both_detectors_share_one_role(self):
        self.assertEqual(CLASS_NAMES["a digital wall clock"], "reloj")
        self.assertIn("clock", CLOCK_CLASS_NAMES)
        self.assertIn("a digital wall clock", CLOCK_CLASS_NAMES)
        self.assertTrue(is_clock_class("clock"))
        self.assertTrue(is_clock_class("a digital wall clock"))
        self.assertFalse(is_clock_class("watch"))

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

    def test_counts_global_color_cues_without_claiming_an_object(self):
        image = np.zeros((10, 10, 3), dtype=np.uint8)
        image[:2, :, 0] = 160
        image[2:3, :, 2] = 160

        self.assertEqual(
            color_pixel_counts(image),
            {"red": 20, "blue": 10},
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

    def test_recent_open_detection_is_not_erased_by_empty_rtdetr(self):
        merged = merge_source_detections(
            {
                "rtdetr": (100.0, {}),
                "grounding_dino": (
                    99.0,
                    {"mesa_roja": {"source": "grounding_dino"}},
                ),
            },
            now=101.0,
        )
        self.assertIn("mesa_roja", merged)

    def test_expired_open_detection_is_removed(self):
        merged = merge_source_detections(
            {
                "rtdetr": (110.0, {}),
                "grounding_dino": (
                    100.0,
                    {"mesa_roja": {"source": "grounding_dino"}},
                ),
            },
            now=110.0,
        )
        self.assertNotIn("mesa_roja", merged)


if __name__ == "__main__":
    unittest.main()
