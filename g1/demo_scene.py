#!/usr/bin/env python3
"""La habitación de la demo: reloj y dos mesas de colores con objetos.

El reloj es una referencia conocida durante la primera etapa de validación.
Las mesas existen en posiciones fijas dentro del mundo, pero esas posiciones
no se entregan al agente: debe encontrarlas con sus sensores como en el robot
real.
"""
import math
import os

import isaaclab.sim as sim_utils

from scene_layout import (
    COLORED_TABLES,
    NAVIGATION_TARGETS,
    SCENE_POSITIONS,
    TABLE_SIZE,
    WORLD_BOUNDS,
)

TABLE_HEIGHT = 0.75
CLOCK_HEIGHT = 1.55
ROOM_HEIGHT = 2.7
WALL_THICKNESS = 0.12
CLOCK_APPROACH = NAVIGATION_TARGETS["reloj"][:2]
CLOCK_FACE_YAW = math.atan2(
    CLOCK_APPROACH[1] - SCENE_POSITIONS["clock"][1],
    CLOCK_APPROACH[0] - SCENE_POSITIONS["clock"][0],
)

DIGIT_SEGMENTS = {
    "0": "ab cdef".replace(" ", ""),
    "1": "bc",
    "2": "abdeg",
    "3": "abcdg",
    "4": "bcfg",
    "5": "acdfg",
    "6": "acdefg",
    "7": "abc",
    "8": "abcdefg",
    "9": "abcdfg",
}


def _color(rgb):
    return sim_utils.PreviewSurfaceCfg(diffuse_color=rgb)


def _yaw_quaternion(yaw: float):
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def _clock_time() -> str:
    value = os.environ.get("DEMO_CLOCK_TIME", "09:00")
    valid_shape = (
        len(value) == 5
        and value[2] == ":"
        and value[:2].isdigit()
        and value[3:].isdigit()
    )
    if not valid_shape:
        raise ValueError(
            "DEMO_CLOCK_TIME debe tener formato HH:MM, por ejemplo 09:00"
        )
    hour, minute = int(value[:2]), int(value[3:])
    if hour > 23 or minute > 59:
        raise ValueError("DEMO_CLOCK_TIME contiene una hora imposible")
    return value


def _clock_world_position(
    clock_x: float,
    clock_y: float,
    normal_offset: float,
    horizontal_offset: float,
    vertical_offset: float,
):
    """Convierte coordenadas de la cara del reloj al mundo."""
    cosine, sine = math.cos(CLOCK_FACE_YAW), math.sin(CLOCK_FACE_YAW)
    return (
        clock_x + cosine * normal_offset - sine * horizontal_offset,
        clock_y + sine * normal_offset + cosine * horizontal_offset,
        CLOCK_HEIGHT + vertical_offset,
    )


def _spawn_digital_clock():
    """Crea un reloj real, legible y orientado hacia su punto de observación."""
    clock_x, clock_y = SCENE_POSITIONS["clock"]
    orientation = _yaw_quaternion(CLOCK_FACE_YAW)

    panel_depth = 0.05
    panel = sim_utils.CuboidCfg(
        size=(panel_depth, 0.62, 0.32),
        visual_material=_color((0.025, 0.035, 0.045)),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
    )
    panel.func(
        "/World/reloj/panel",
        panel,
        translation=(clock_x, clock_y, CLOCK_HEIGHT),
        orientation=orientation,
    )

    # Los segmentos son geometría, no texto pintado en la captura. Así el
    # modelo visual recibe el mismo reloj desde cualquier distancia y ángulo.
    digit_material = sim_utils.PreviewSurfaceCfg(
        diffuse_color=(0.08, 0.95, 0.22),
        emissive_color=(0.04, 0.55, 0.10),
    )
    segment_depth = 0.015
    segment_length = 0.065
    segment_width = 0.016
    horizontal_segment = sim_utils.CuboidCfg(
        size=(segment_depth, segment_length, segment_width),
        visual_material=digit_material,
    )
    vertical_segment = sim_utils.CuboidCfg(
        size=(segment_depth, segment_width, segment_length),
        visual_material=digit_material,
    )
    dot = sim_utils.CuboidCfg(
        size=(segment_depth, segment_width, segment_width),
        visual_material=digit_material,
    )

    segment_centers = {
        "a": (0.0, 0.075),
        "b": (0.040, 0.038),
        "c": (0.040, -0.038),
        "d": (0.0, -0.075),
        "e": (-0.040, -0.038),
        "f": (-0.040, 0.038),
        "g": (0.0, 0.0),
    }
    horizontal_names = {"a", "d", "g"}
    digit_offsets = (-0.20, -0.08, 0.08, 0.20)
    value = _clock_time()
    digits = value.replace(":", "")
    normal_offset = panel_depth / 2.0 + segment_depth / 2.0 + 0.002

    for digit_index, (digit, digit_offset) in enumerate(
        zip(digits, digit_offsets)
    ):
        for segment_name in DIGIT_SEGMENTS[digit]:
            horizontal, vertical = segment_centers[segment_name]
            cfg = (
                horizontal_segment
                if segment_name in horizontal_names
                else vertical_segment
            )
            cfg.func(
                f"/World/reloj/digit_{digit_index}/{segment_name}",
                cfg,
                translation=_clock_world_position(
                    clock_x,
                    clock_y,
                    normal_offset,
                    digit_offset + horizontal,
                    vertical,
                ),
                orientation=orientation,
            )

    for dot_index, vertical in enumerate((-0.035, 0.035)):
        dot.func(
            f"/World/reloj/colon_{dot_index}",
            dot,
            translation=_clock_world_position(
                clock_x,
                clock_y,
                normal_offset,
                0.0,
                vertical,
            ),
            orientation=orientation,
        )

    print(
        f"[escena] reloj digital {value}, cara {math.degrees(CLOCK_FACE_YAW):.1f}°",
        flush=True,
    )


def _spawn_room_walls():
    """Convierte el piso abierto en una habitación física medible."""
    xmin, xmax = WORLD_BOUNDS["xmin"], WORLD_BOUNDS["xmax"]
    ymin, ymax = WORLD_BOUNDS["ymin"], WORLD_BOUNDS["ymax"]
    width, depth = xmax - xmin, ymax - ymin
    walls = (
        (
            "west",
            (WALL_THICKNESS, depth + 2 * WALL_THICKNESS, ROOM_HEIGHT),
            (xmin, (ymin + ymax) / 2.0, ROOM_HEIGHT / 2.0),
        ),
        (
            "east",
            (WALL_THICKNESS, depth + 2 * WALL_THICKNESS, ROOM_HEIGHT),
            (xmax, (ymin + ymax) / 2.0, ROOM_HEIGHT / 2.0),
        ),
        (
            "south",
            (width, WALL_THICKNESS, ROOM_HEIGHT),
            ((xmin + xmax) / 2.0, ymin, ROOM_HEIGHT / 2.0),
        ),
        (
            "north",
            (width, WALL_THICKNESS, ROOM_HEIGHT),
            ((xmin + xmax) / 2.0, ymax, ROOM_HEIGHT / 2.0),
        ),
    )
    for name, size, position in walls:
        wall = sim_utils.CuboidCfg(
            size=size,
            visual_material=_color((0.72, 0.74, 0.77)),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        )
        wall.func(
            f"/World/room/{name}_wall",
            wall,
            translation=position,
        )
    print(
        f"[escena] habitación física de {width:.1f} x {depth:.1f} m, "
        f"paredes de {ROOM_HEIGHT:.1f} m",
        flush=True,
    )


def build_demo_scene():
    """Crea la habitacion. Llamar despues del piso y antes de sim.reset()."""

    _spawn_room_walls()

    # --- el reloj: display digital visible desde el punto de observación ---
    _spawn_digital_clock()

    # Cada mesa tiene un objeto dinámico independiente. No alcanza con dibujar
    # una botella: debe poder caerse y, más adelante, responder al agarre.
    for table in COLORED_TABLES:
        table_x, table_y = SCENE_POSITIONS[table["id"]]
        top = sim_utils.CuboidCfg(
            size=(*TABLE_SIZE, 0.05),
            visual_material=_color(table["rgb"]),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        )
        top.func(
            f"/World/{table['id']}/top",
            top,
            translation=(table_x, table_y, TABLE_HEIGHT),
        )
        leg_color = tuple(channel * 0.72 for channel in table["rgb"])
        leg_height = TABLE_HEIGHT - 0.025
        leg = sim_utils.CuboidCfg(
            size=(0.08, 0.08, leg_height),
            visual_material=_color(leg_color),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        )
        # Una base maciza producía la silueta de un cajón y el detector
        # preentrenado no la reconocía como mesa. Las cuatro patas dejan la
        # estructura visual que también tendrá el mueble real.
        for leg_index, (offset_x, offset_y) in enumerate(
            (
                (-0.31, -0.51),
                (-0.31, 0.51),
                (0.31, -0.51),
                (0.31, 0.51),
            )
        ):
            leg.func(
                f"/World/{table['id']}/leg_{leg_index}",
                leg,
                translation=(
                    table_x + offset_x,
                    table_y + offset_y,
                    leg_height / 2,
                ),
            )
        bottle = sim_utils.CylinderCfg(
            radius=0.035,
            height=0.22,
            visual_material=_color((0.15, 0.55, 0.25)),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.5),
        )
        bottle.func(
            f"/World/{table['id']}_object",
            bottle,
            translation=(table_x, table_y, TABLE_HEIGHT + 0.14),
        )

    print(
        f"[escena] reloj en {SCENE_POSITIONS['clock']}; "
        f"mesa roja en {SCENE_POSITIONS['red_table']}; "
        f"mesa azul en {SCENE_POSITIONS['blue_table']}; "
        "un objeto dinámico sobre cada mesa",
        flush=True,
    )
