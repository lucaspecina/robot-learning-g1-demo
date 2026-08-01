#!/usr/bin/env python3
"""Aísla si AppLauncher cambia la salida del LiDAR RTX de NVIDIA."""

import argparse

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(
    description="Prueba mínima del LiDAR RTX arrancado mediante Isaac Lab."
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import FixedCuboid  # noqa: E402
from isaacsim.sensors.rtx import LidarRtx  # noqa: E402


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
    # La escena es idéntica a la referencia positiva. Así el único cambio es
    # el lanzador que también usa g1_robot.py.
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
                f"PASA: AppLauncher produjo {best_points} puntos "
                f"({empty_frames} cuadros vacíos previos)",
                flush=True,
            )
            return 0
    print(
        f"FALLA: AppLauncher produjo como máximo {best_points} puntos "
        f"en {MAX_FRAMES} cuadros",
        flush=True,
    )
    return 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        simulation_app.close()
