#!/usr/bin/env python3
"""Comprueba los metadatos necesarios para localizarse con LiDAR e IMU."""
from isaacsim import SimulationApp


simulation_app = SimulationApp(
    {"headless": True, "enable_motion_bvh": True}
)


import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.sensors.rtx import LidarRtx


ANNOTATOR = "IsaacCreateRTXLidarScanBuffer"
MAX_FRAMES = 600
MIN_POINTS = 100


def main() -> int:
    world = World(
        physics_dt=1.0 / 60.0,
        rendering_dt=1.0 / 60.0,
        stage_units_in_meters=1.0,
    )
    world.scene.add_default_ground_plane()
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
            **{"omni:sensor:Core:auxOutputType": "BASIC"},
        )
    )
    world.reset()
    # Point-LIO corrige el movimiento ocurrido dentro de una vuelta. Por eso
    # una sola hora en el encabezado no alcanza: necesitamos una por retorno.
    lidar.attach_annotator(
        ANNOTATOR,
        outputTimestamp=True,
        outputIntensity=True,
        outputEmitterId=True,
    )

    for _ in range(MAX_FRAMES):
        world.step(render=True)
        payload = lidar.get_current_frame().get(ANNOTATOR, {})
        points = np.asarray(payload.get("data", []))
        timestamps = np.asarray(payload.get("timestamp", []))
        intensities = np.asarray(payload.get("intensity", []))
        emitter_ids = np.asarray(payload.get("emitterId", []))
        return_indexes = np.asarray(payload.get("index", []))
        echo_count = int(payload.get("info", {}).get("numEchos", 0))
        point_count = int(points.size // 3)
        if point_count < MIN_POINTS:
            continue

        # El perfil puede entregar dos ecos del mismo disparo. NVIDIA publica
        # un tiempo por disparo y un índice por retorno; dividir el índice por
        # la cantidad de ecos recupera qué tiempo corresponde a cada punto.
        fire_indexes = (
            return_indexes.astype(np.int64) // echo_count
            if echo_count > 0 and return_indexes.size == point_count
            else np.asarray([], dtype=np.int64)
        )
        timestamps_per_return = (
            timestamps[fire_indexes]
            if fire_indexes.size == point_count
            and fire_indexes.size > 0
            and int(np.max(fire_indexes)) < timestamps.size
            else np.asarray([], dtype=timestamps.dtype)
        )
        matching_sizes = all(
            values.size == point_count
            for values in (
                timestamps_per_return,
                intensities,
                emitter_ids,
                return_indexes,
            )
        )
        monotonic_time = bool(
            timestamps_per_return.size > 1
            and np.all(np.diff(timestamps_per_return) >= 0)
        )
        time_span_ms = (
            float(timestamps_per_return[-1] - timestamps_per_return[0])
            / 1.0e6
            if timestamps_per_return.size > 1
            else 0.0
        )
        emitter_count = int(np.unique(emitter_ids).size)
        print(
            "METADATOS LIDAR: "
            f"puntos={point_count} forma={points.shape} tipo={points.dtype}, "
            f"tiempos={timestamps.size} forma={timestamps.shape} "
            f"tipo={timestamps.dtype}, "
            f"intensidades={intensities.size} "
            f"forma={intensities.shape} tipo={intensities.dtype}, "
            f"emisores_publicados={emitter_ids.size} "
            f"forma={emitter_ids.shape} tipo={emitter_ids.dtype}, "
            f"índices={return_indexes.size}, ecos={echo_count}, "
            f"tamaños_coinciden={matching_sizes}, "
            f"tiempo_monótono={monotonic_time}, "
            f"duración={time_span_ms:.3f} ms, "
            f"emisores={emitter_count}, "
            f"intensidad=[{float(np.min(intensities)):.3f}, "
            f"{float(np.max(intensities)):.3f}]",
            flush=True,
        )
        if matching_sizes and monotonic_time and time_span_ms > 0.0:
            print(
                "PASA: Isaac entrega tiempo, intensidad y emisor por punto",
                flush=True,
            )
            return 0
        print(
            "FALLA: la nube existe pero no conserva metadatos utilizables",
            flush=True,
        )
        return 1

    print(
        f"FALLA: no apareció una nube de {MIN_POINTS} puntos en "
        f"{MAX_FRAMES} cuadros",
        flush=True,
    )
    return 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        simulation_app.close()
