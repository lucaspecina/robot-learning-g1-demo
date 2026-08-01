#!/usr/bin/env python3
"""Aísla si AppLauncher cambia la salida del LiDAR RTX de NVIDIA."""

import argparse
import os
import sys

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(
    description="Prueba mínima del LiDAR RTX arrancado mediante Isaac Lab."
)
parser.add_argument(
    "--simulation-context",
    action="store_true",
    help="usar el ciclo de simulación de g1_robot.py en vez de World",
)
parser.add_argument(
    "--physics-dt",
    type=float,
    default=1.0 / 60.0,
    help="paso físico en segundos; 0.002 reproduce el G1",
)
parser.add_argument(
    "--render-every",
    type=int,
    default=1,
    help="dibujar un cuadro cada N pasos físicos; 25 reproduce el G1",
)
parser.add_argument(
    "--with-g1",
    action="store_true",
    help="agregar solamente el cuerpo oficial G1 usado por AGILE",
)
parser.add_argument(
    "--with-demo-scene",
    action="store_true",
    help="agregar la habitación completa de la demo",
)
parser.add_argument(
    "--with-ros-bridge",
    action="store_true",
    help="habilitar el mismo puente ROS 2 que usa g1_robot.py",
)
parser.add_argument(
    "--mount-on-head",
    action="store_true",
    help="montar el LiDAR sobre la cabeza como en la demo",
)
parser.add_argument(
    "--max-frames",
    type=int,
    default=600,
    help="cantidad máxima de cuadros dibujados antes de fallar",
)
parser.add_argument(
    "--sensor-height",
    type=float,
    default=0.35,
    help="altura local sobre head_link al montar el sensor",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import numpy as np  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import FixedCuboid  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402


# La experiencia mínima de AppLauncher no incluye sensores RTX. Habilitar la
# extensión explícitamente reproduce el contrato que debe usar g1_robot.py.
enable_extension("isaacsim.sensors.rtx")
if args_cli.with_ros_bridge:
    enable_extension("isaacsim.ros2.bridge")


from isaacsim.sensors.rtx import LidarRtx  # noqa: E402


G1_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, G1_DIR)


ANNOTATOR = "IsaacExtractRTXSensorPointCloudNoAccumulator"
MIN_POINTS = 100


def main() -> int:
    if args_cli.simulation_context:
        return run_with_simulation_context()
    return run_with_world()


def sample_points(step, lidar, mode: str) -> int:
    """Mide la misma salida cruda con cualquiera de los dos ciclos."""
    empty_frames = 0
    best_points = 0
    rendered_frames = 0
    while rendered_frames < args_cli.max_frames:
        if step() is False:
            continue
        rendered_frames += 1
        payload = lidar.get_current_frame().get(ANNOTATOR, {})
        points = np.asarray(payload.get("data", []))
        best_points = max(best_points, int(points.size // 3))
        if points.size == 0:
            empty_frames += 1
        if best_points >= MIN_POINTS:
            print(
                f"PASA: {mode} produjo {best_points} puntos "
                f"({empty_frames} cuadros vacíos previos)",
                flush=True,
            )
            return 0
    print(
        f"FALLA: {mode} produjo como máximo {best_points} puntos "
        f"en {args_cli.max_frames} cuadros",
        flush=True,
    )
    return 1


def run_with_world() -> int:
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
    return sample_points(
        lambda: world.step(render=True),
        lidar,
        "AppLauncher con World",
    )


def run_with_simulation_context() -> int:
    """Reproduce el ciclo manual usado por el proceso completo del G1."""
    if args_cli.mount_on_head and not args_cli.with_g1:
        raise ValueError("--mount-on-head requiere --with-g1")
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=args_cli.physics_dt, device=args_cli.device)
    )
    ground = sim_utils.GroundPlaneCfg()
    ground.func("/World/ground", ground)
    FixedCuboid(
        prim_path="/World/front_wall",
        name="front_wall",
        position=np.array([3.0, 0.0, 1.5]),
        scale=np.array([0.2, 8.0, 3.0]),
    )
    if args_cli.with_g1:
        from g1_asset import make_wbc_agile_g1_cfg

        # Mantener el sensor fijo permite atribuir cualquier cambio al cuerpo
        # agregado, no al montaje móvil sobre la cabeza.
        robot = Articulation(make_wbc_agile_g1_cfg())
    if args_cli.with_demo_scene:
        from demo_scene import build_demo_scene

        build_demo_scene()
    sensor_path = (
        "/World/G1/torso_link/head_link/lidar"
        if args_cli.mount_on_head
        else "/World/lidar"
    )
    sensor_pose = (
        {"translation": np.array([0.0, 0.0, args_cli.sensor_height])}
        if args_cli.mount_on_head
        else {"position": np.array([0.0, 0.0, 1.0])}
    )
    lidar = LidarRtx(
        prim_path=sensor_path,
        name="lidar",
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        config_file_name="Example_Rotary",
        **sensor_pose,
    )
    sim.reset()
    # Fuera de World no existe una escena que inicialice sus elementos. Esta
    # llamada replica exactamente el ciclo manual de g1_robot.py.
    lidar.initialize()
    lidar.attach_annotator(ANNOTATOR)
    print(
        f"MONTAJE LIDAR: {sensor_path} pose={sensor_pose}",
        flush=True,
    )

    step_index = 0

    def step_simulation() -> bool:
        nonlocal step_index
        rendered = step_index % args_cli.render_every == 0
        sim.step(render=rendered)
        step_index += 1
        return rendered

    return sample_points(
        step_simulation,
        lidar,
        (
            f"SimulationContext a dt={args_cli.physics_dt:.6f}, "
            f"render cada {args_cli.render_every} pasos, "
            f"G1={'sí' if args_cli.with_g1 else 'no'}, "
            f"habitación={'sí' if args_cli.with_demo_scene else 'no'}, "
            f"ROS={'sí' if args_cli.with_ros_bridge else 'no'}, "
            f"cabeza={'sí' if args_cli.mount_on_head else 'no'}"
        ),
    )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        simulation_app.close()
