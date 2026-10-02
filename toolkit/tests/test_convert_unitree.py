import cv2
import numpy as np

from toolkit.dataset.convert_unitree import build_features, read_image_shape
from toolkit.dataset.robot_configs import G1_BRAINCO_1CAM, register_robot_configs


def test_image_shape_is_read_from_the_first_frame(tmp_path):
    colors = tmp_path / "episode_0001" / "colors"
    colors.mkdir(parents=True)
    cv2.imwrite(str(colors / "000000_color_0.jpg"), np.zeros((36, 64, 3), dtype=np.uint8))
    assert read_image_shape(tmp_path) == (36, 64, 3)


def test_features_match_what_the_act_policy_expects():
    register_robot_configs()
    features = build_features(G1_BRAINCO_1CAM, (360, 640, 3))
    assert features["observation.state"]["shape"] == (26,)
    assert features["action"]["shape"] == (26,)
    assert features["observation.images.cam_right_high"]["shape"] == (360, 640, 3)
