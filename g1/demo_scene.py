#!/usr/bin/env python3
"""La habitación de la demo: mesa, objeto, reloj y dos personas provisionales.

La misión vigente ya no usa personas; se conservan temporalmente para no
mezclar el cambio de escenario de las mesas con la validación visual del reloj.
Cada reemplazo se hace y se mide por separado.

Las personas son cilindros de colores. No hace falta que parezcan personas: lo
que importa es que la camara vea una forma vertical de color distinguible, que
es exactamente lo que un detector va a usar para decidir "roja" o "azul".
Cuando la demo este cerca, se reemplazan por los modelos animados de NVIDIA
sin cambiar nada de la logica.

Las posiciones fijas (mesa, reloj) son el "mapa semantico" de la demo: lo que
el robot ya sabe donde esta. El objeto y las personas se detectan con la
camara, porque en la vida real se mueven.
"""
import math
import os

import isaaclab.sim as sim_utils

# --- el mapa semantico: lo que el robot ya sabe donde esta ---
SEMANTIC_MAP = {
    "mesa": (3.0, 0.0),
    "reloj": (0.0, 2.5),
}

TABLE_HEIGHT = 0.75
CLOCK_HEIGHT = 1.55
CLOCK_APPROACH = (0.8, 1.8)
CLOCK_FACE_YAW = math.atan2(
    CLOCK_APPROACH[1] - SEMANTIC_MAP["reloj"][1],
    CLOCK_APPROACH[0] - SEMANTIC_MAP["reloj"][0],
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
    clock_x, clock_y = SEMANTIC_MAP["reloj"]
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


def build_demo_scene():
    """Crea la habitacion. Llamar despues del piso y antes de sim.reset()."""

    # --- la mesa: un tablero sobre cuatro patas simplificadas a un bloque ---
    mesa_x, mesa_y = SEMANTIC_MAP["mesa"]
    tablero = sim_utils.CuboidCfg(
        size=(0.8, 1.2, 0.05),
        visual_material=_color((0.45, 0.30, 0.18)),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
    )
    tablero.func("/World/mesa/tablero", tablero,
                 translation=(mesa_x, mesa_y, TABLE_HEIGHT))

    base = sim_utils.CuboidCfg(
        size=(0.6, 1.0, TABLE_HEIGHT),
        visual_material=_color((0.35, 0.24, 0.14)),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
    )
    base.func("/World/mesa/base", base,
              translation=(mesa_x, mesa_y, TABLE_HEIGHT / 2))

    # --- el objeto a buscar: una botella (cilindro), SI dinamica: hay que
    #     poder agarrarla y que se caiga si la sueltan ---
    botella = sim_utils.CylinderCfg(
        radius=0.035,
        height=0.22,
        visual_material=_color((0.15, 0.55, 0.25)),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        mass_props=sim_utils.MassPropertiesCfg(mass=0.5),
    )
    botella.func("/World/botella", botella,
                 translation=(mesa_x, mesa_y, TABLE_HEIGHT + 0.14))

    # --- el reloj: display digital visible desde el punto de observación ---
    _spawn_digital_clock()

    # --- las personas: cilindros de color, altura de persona ---
    personas = {
        "persona_roja": ((5.0, 3.0), (0.75, 0.12, 0.12)),
        "persona_azul": ((5.0, -3.0), (0.12, 0.20, 0.75)),
    }
    for nombre, ((px, py), rgb) in personas.items():
        cuerpo = sim_utils.CylinderCfg(
            radius=0.20,
            height=1.7,
            visual_material=_color(rgb),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        )
        cuerpo.func(f"/World/{nombre}", cuerpo, translation=(px, py, 0.85))

    print(f"[escena] mesa en {SEMANTIC_MAP['mesa']}, reloj en {SEMANTIC_MAP['reloj']}, "
          f"botella sobre la mesa, personas roja y azul", flush=True)
