#!/usr/bin/env python3
"""Mide el LiDAR PhysX oficial quieto y sobre un soporte móvil."""

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": True})


import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.sensors.physx import RotatingLidarPhysX


PHYSICS_HZ = 60.0
SPEED_MPS = 0.30
FRONT_WALL_FACE_X = 2.95
MAX_MEDIAN_ERROR_M = 0.02
MAX_P95_ERROR_M = 0.05
MIN_SAMPLES = 100


def forward_wall_error(points, sensor_x):
    """Compara retornos frontales con la cara conocida de la pared."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    expected = FRONT_WALL_FACE_X - sensor_x
    candidates = points[
        (points[:, 0] > 0.2)
        & (np.abs(points[:, 1]) < 1.0)
        & (np.abs(points[:, 2]) < 0.5)
    ]
    if candidates.size == 0:
        return None
    # La pared es perpendicular al eje x; su coordenada local debe ser la
    # distancia conocida aunque el soporte se esté moviendo.
    return float(np.median(np.abs(candidates[:, 0] - expected)))


def main() -> int:
    world = World(
        physics_dt=1.0 / PHYSICS_HZ,
        rendering_dt=1.0 / PHYSICS_HZ,
        stage_units_in_meters=1.0,
    )
    world.scene.add_default_ground_plane()
    world.scene.add(
        FixedCuboid(
            prim_path="/World/front_wall",
            name="front_wall",
            position=np.array([3.0, 0.0, 1.0]),
            scale=np.array([0.1, 8.0, 2.0]),
        )
    )
    carrier = world.scene.add(
        SingleXFormPrim(
            prim_path="/World/carrier",
            name="carrier",
            position=np.array([0.0, 0.0, 1.0]),
        )
    )
    lidar = world.scene.add(
        RotatingLidarPhysX(
            prim_path="/World/carrier/lidar",
            name="physx_lidar",
            translation=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            # Cero dispara la vuelta entera en el mismo paso físico. Es una
            # prueba de diagnóstico, no una réplica del LiDAR real rotativo.
            rotation_frequency=0.0,
            fov=(360.0, 10.0),
            resolution=(0.5, 5.0),
            valid_range=(0.05, 12.0),
        )
    )
    world.reset()
    lidar.add_point_cloud_data_to_frame()
    lidar.add_linear_depth_data_to_frame()
    lidar.add_azimuth_data_to_frame()

    for _ in range(10):
        world.step(render=True)

    static_errors = []
    moving_errors = []
    nonempty_frames = 0
    first_shape = None
    first_minimum = None
    first_maximum = None
    first_depth = None
    first_azimuth = None
    sensor_x = 0.0
    for step in range(360):
        moving = step >= 120
        if moving:
            sensor_x += SPEED_MPS / PHYSICS_HZ
            carrier.set_world_pose(
                position=np.array([sensor_x, 0.0, 1.0])
            )
        world.step(render=True)
        raw_points = np.asarray(
            lidar.get_current_frame().get("point_cloud", [])
        )
        if raw_points.size:
            nonempty_frames += 1
            if first_shape is None:
                points = raw_points.reshape(-1, 3)
                first_shape = raw_points.shape
                first_minimum = np.min(points, axis=0)
                first_maximum = np.max(points, axis=0)
                first_depth = np.asarray(
                    lidar.get_current_frame().get("linear_depth", [])
                )
                first_azimuth = np.asarray(
                    lidar.get_current_frame().get("azimuth", [])
                )
        error = forward_wall_error(raw_points, sensor_x)
        if error is None:
            continue
        (moving_errors if moving else static_errors).append(error)

    print(
        f"cuadros con puntos: {nonempty_frames}; forma {first_shape}; "
        f"mínimo {first_minimum}; máximo {first_maximum}",
        flush=True,
    )
    print(
        "profundidad: "
        f"forma {None if first_depth is None else first_depth.shape}, "
        f"mínimo {None if first_depth is None else np.min(first_depth)}, "
        f"máximo {None if first_depth is None else np.max(first_depth)}",
        flush=True,
    )
    print(
        "azimut: "
        f"forma {None if first_azimuth is None else first_azimuth.shape}, "
        f"primeros {None if first_azimuth is None else first_azimuth[:5]}, "
        f"últimos {None if first_azimuth is None else first_azimuth[-5:]}",
        flush=True,
    )
    print(
        f"muestras de pared: quieto {len(static_errors)}, "
        f"móvil {len(moving_errors)}",
        flush=True,
    )
    if min(len(static_errors), len(moving_errors)) < MIN_SAMPLES:
        print("FALLA: el LiDAR PhysX no produjo suficientes barridos", flush=True)
        return 1

    for label, errors in (("quieto", static_errors), ("móvil", moving_errors)):
        median = float(np.median(errors))
        p95 = float(np.percentile(errors, 95))
        maximum = float(np.max(errors))
        print(
            f"{label}: mediana {median:.4f} m, p95 {p95:.4f} m, "
            f"máximo {maximum:.4f} m",
            flush=True,
        )
        if median > MAX_MEDIAN_ERROR_M or p95 > MAX_P95_ERROR_M:
            print(f"FALLA: error {label} fuera del límite", flush=True)
            return 1

    print(
        "PASA: el LiDAR PhysX mantuvo la pared al mover el soporte",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        simulation_app.close()
