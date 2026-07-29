"""Pruebas de que la escena y su mapa visible siguen siendo coherentes."""
import unittest

from scene_layout import DASHBOARD_SCENE, NAVIGATION_TARGETS, SCENE_POSITIONS


class SceneLayoutTest(unittest.TestCase):
    def test_every_scene_object_is_drawn(self):
        drawn_ids = {
            landmark["id"]
            for landmark in DASHBOARD_SCENE["landmarks"]
        }
        self.assertEqual(drawn_ids, set(SCENE_POSITIONS))

    def test_every_landmark_fits_inside_visible_world(self):
        world = DASHBOARD_SCENE["world"]
        for landmark in DASHBOARD_SCENE["landmarks"]:
            if landmark["shape"] == "rectangle":
                half_x = landmark["size_x"] / 2
                half_y = landmark["size_y"] / 2
            else:
                half_x = half_y = landmark["radius"]
            with self.subTest(landmark=landmark["id"]):
                self.assertGreaterEqual(landmark["x"] - half_x, world["xmin"])
                self.assertLessEqual(landmark["x"] + half_x, world["xmax"])
                self.assertGreaterEqual(landmark["y"] - half_y, world["ymin"])
                self.assertLessEqual(landmark["y"] + half_y, world["ymax"])

    def test_navigation_targets_do_not_point_to_object_centers(self):
        self.assertNotEqual(
            NAVIGATION_TARGETS["reloj"][:2],
            SCENE_POSITIONS["clock"],
        )

    def test_tables_are_not_given_to_navigation(self):
        self.assertNotIn("mesa", NAVIGATION_TARGETS)
        self.assertNotIn("mesa_roja", NAVIGATION_TARGETS)
        self.assertNotIn("mesa_azul", NAVIGATION_TARGETS)


if __name__ == "__main__":
    unittest.main()
