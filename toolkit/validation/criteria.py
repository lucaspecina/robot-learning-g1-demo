"""Thresholds for the validation checks, declared before looking at the data.

They are starting points taken from the sources in docs/08_validacion_de_datos.md
and must be calibrated by looking at the flagged episodes. Changing one after
seeing the results is fine, but the change and its reason go in the notebook.
"""

import dataclasses


@dataclasses.dataclass(frozen=True)
class Criteria:
    robust_z: float = 3.0
    """|x - median| / MAD above which a per-episode metric is an outlier."""
    jump_factor: float = 2.0
    """A frame-to-frame step above k times the joint's 99.9th percentile step is a jump.

    A median + MAD rule fails here: most frames are still, so the median step is
    ~0 and any motion looks like a jump."""
    idle_threshold_rad: float = 0.05
    """Arm motion from the first (or last) pose that counts as the start (or end) of activity."""
    pause_min_frames: int = 15
    """Half a second without action change counts as a pause."""
    pause_velocity_rad: float = 2e-3
    """Max |Δaction| per frame over the arm joints considered still (0.06 rad/s).
    Calibrated on our data: arms move less than 1e-3 rad per frame only 5% of the time."""
    hand_saturation_eps: float = 1e-3
    """Hand commands within this distance of 1 (fully closed end of the range) count as saturated.
    The other end, 0, is the open hand at rest and is not a problem."""
    frozen_range_fraction: float = 0.02
    """A joint whose q99 - q01 is below this fraction of the widest joint range is frozen."""
    grasp_min_closure: float = 0.3
    """A hand command that never exceeds this value suggests no grasp happened."""
    speed_cv_consistent: float = 0.2
    speed_cv_large: float = 0.4
    """Coefficient of variation of mean speed across episodes (LeRobot visualizer heuristic)."""
    frozen_frame_diff: float = 0.5
    """Mean absolute pixel change (0-255) below which two consecutive frames are identical."""
