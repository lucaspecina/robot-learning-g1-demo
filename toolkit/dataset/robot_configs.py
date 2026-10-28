"""Robot configurations for our recordings, added to the official unitree_lerobot ones."""

from unitree_lerobot.utils.constants import G1_BRAINCO_CONFIG, ROBOT_CONFIGS, RobotConfig

G1_BRAINCO_1CAM = "Unitree_G1_Brainco_1Cam"

# Our teleoperation setup records a single head camera (color_0) instead of the
# four cameras the stock Brainco config expects. The camera is named
# cam_right_high because that is the input key our first ACT policy asks for.
G1_BRAINCO_1CAM_CONFIG = RobotConfig(
    motors=G1_BRAINCO_CONFIG.motors,
    cameras=["cam_right_high"],
    camera_to_image_key={"color_0": "cam_right_high"},
    json_state_data_name=["left_arm.qpos", "right_arm.qpos", "left_ee.qpos", "right_ee.qpos"],
    json_action_data_name=["left_arm.qpos", "right_arm.qpos", "left_ee.qpos", "right_ee.qpos"],
)


def register_robot_configs() -> None:
    """Make our configs visible to the official reader, which looks them up by name."""
    ROBOT_CONFIGS.setdefault(G1_BRAINCO_1CAM, G1_BRAINCO_1CAM_CONFIG)
