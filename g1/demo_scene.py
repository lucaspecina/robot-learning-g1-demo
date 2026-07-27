#!/usr/bin/env python3
"""La habitacion de la demo: mesa, objeto, reloj y dos personas.

Escena minima pero completa para la tarea objetivo: el robot mira un reloj,
va a buscar un objeto a una mesa, y se lo lleva a la persona de la remera del
color que corresponda.

Las personas son cilindros de colores. No hace falta que parezcan personas: lo
que importa es que la camara vea una forma vertical de color distinguible, que
es exactamente lo que un detector va a usar para decidir "roja" o "azul".
Cuando la demo este cerca, se reemplazan por los modelos animados de NVIDIA
sin cambiar nada de la logica.

Las posiciones fijas (mesa, reloj) son el "mapa semantico" de la demo: lo que
el robot ya sabe donde esta. El objeto y las personas se detectan con la
camara, porque en la vida real se mueven.
"""
import isaaclab.sim as sim_utils

# --- el mapa semantico: lo que el robot ya sabe donde esta ---
SEMANTIC_MAP = {
    "mesa": (3.0, 0.0),
    "reloj": (0.0, 2.5),
}

TABLE_HEIGHT = 0.75
CLOCK_HEIGHT = 1.8


def _color(rgb):
    return sim_utils.PreviewSurfaceCfg(diffuse_color=rgb)


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

    # --- el reloj: un panel claro en la pared, a la altura de la vista ---
    reloj_x, reloj_y = SEMANTIC_MAP["reloj"]
    reloj = sim_utils.CuboidCfg(
        size=(0.05, 0.4, 0.4),
        visual_material=_color((0.95, 0.95, 0.90)),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
    )
    reloj.func("/World/reloj", reloj, translation=(reloj_x, reloj_y, CLOCK_HEIGHT))

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
