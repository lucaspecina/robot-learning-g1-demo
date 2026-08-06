#!/usr/bin/env python3
"""Aísla el LiDAR RTX de NVIDIA del robot y de ROS 2."""
from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {"headless": True, "enable_motion_bvh": True}
)

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.sensors.rtx import LidarRtx

ANNOTATOR = "IsaacExtractRTXSensorPointCloudNoAccumulator"
MAX_FRAMES = 600
MIN_POINTS = 100


def main() -> int:
    world = World(
        physics_dt=1.0 / 60.0,
        rendering_dt=1.0 / 60.0,
        stage_units_in_meters=1.0,
    )
    world.scene.add_default_ground_plane()
    # Las paredes cercanas garantizan retornos sin depender de recursos
    # descargados ni de la geometría del G1.
    world.scene.add(
        FixedCuboid(
            prim_path="/World/front_wall",
            name="front_wall",
            position=np.array([3.0, 0.0, 1.5]),
            scale=np.array([0.2, 8.0, 3.0]),
        )
    )
    lidar = world.scene.add(
        LidarRtx(
            prim_path="/World/lidar",
            name="lidar",
            position=np.array([0.0, 0.0, 1.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            config_file_name="Example_Rotary",
        )
    )
    world.reset()
    lidar.attach_annotator(ANNOTATOR)

    empty_frames = 0
    best_points = 0
    for _ in range(MAX_FRAMES):
        world.step(render=True)
        payload = lidar.get_current_frame().get(ANNOTATOR, {})
        points = np.asarray(payload.get("data", []))
        best_points = max(best_points, int(points.size // 3))
        if points.size == 0:
            empty_frames += 1
        if best_points >= MIN_POINTS:
            print(
                f"PASA: LiDAR oficial aislado produjo {best_points} puntos "
                f"({empty_frames} cuadros vacíos previos)",
                flush=True,
            )
            return 0
    print(
        f"FALLA: LiDAR oficial aislado produjo como máximo {best_points} "
        f"puntos en {MAX_FRAMES} cuadros",
        flush=True,
    )
    return 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        simulation_app.close()
