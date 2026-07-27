"""Configuracion del G1 de 12 articulaciones — el robot que la policy conoce.

Por que un asset propio y no el G1 que trae IsaacLab: la policy pre-entrenada
de Unitree fue entrenada sobre SU modelo del G1, que tiene 12 articulaciones
(solo piernas; los brazos son masa fija, sin motores) y pesa 32 kg. El G1 que
trae IsaacLab es otro robot: 37 articulaciones, con brazos y torso moviles.

Una policy de locomocion aprende la dinamica de UN cuerpo. Puesta en otro
cuerpo se cae, por mas bien entrenada que este — como aprender a andar en
bicicleta y que te pongan en una moto. Por eso traemos el modelo original de
Unitree (convertido de URDF a USD con la herramienta de IsaacLab) en vez de
adaptar la policy.

Las ganancias y la pose nominal salen de config/g1_locomotion.yaml, que a su
vez viene del despliegue oficial de Unitree.
"""
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

ASSET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
USD_PATH = os.path.join(ASSET_DIR, "g1_12dof.usd")

# Altura del pelvis con las piernas en la pose nominal, tomada del modelo
# original de Unitree.
STANDING_HEIGHT = 0.793


def make_g1_12dof_cfg(cfg: dict, prim_path: str = "/World/G1") -> ArticulationCfg:
    """Arma la configuracion del robot a partir del yaml de locomocion."""
    nombres = cfg["joint_names"]
    angulos = cfg["default_angles"]
    kps = cfg["kps"]
    kds = cfg["kds"]

    return ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=USD_PATH,
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=4,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, STANDING_HEIGHT),
            joint_pos={n: a for n, a in zip(nombres, angulos)},
            joint_vel={".*": 0.0},
        ),
        soft_joint_pos_limit_factor=0.9,
        actuators={
            "legs": ImplicitActuatorCfg(
                joint_names_expr=list(nombres),
                effort_limit_sim=300.0,
                velocity_limit_sim=100.0,
                stiffness={n: float(k) for n, k in zip(nombres, kps)},
                damping={n: float(d) for n, d in zip(nombres, kds)},
                armature=0.01,
            ),
        },
    )
