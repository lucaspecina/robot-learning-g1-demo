"""Configuraciones del G1 para Isaac: el cuerpo del robot.

Unitree publica varias descripciones del MISMO robot, con distinto nivel de
detalle. Las dos que nos importan:

  g1_12dof   12 articulaciones (solo piernas), 32.1 kg. Los brazos estan ahi
             con su peso pero soldados. Es el cuerpo sobre el que Unitree
             entreno la policy de locomocion que usamos.
  g1_29dof   29 articulaciones (piernas + brazos + cintura), 35.1 kg. Es el
             robot completo — el que corresponde al G1 EDU real, y el unico
             que sirve para la demo, porque hace falta mover los brazos.

La diferencia de masa entre ambos es de 3 kg (9 %). La pregunta abierta es si
la policy entrenada sobre el cuerpo de 12 tolera el cuerpo de 29 con los brazos
quietos; se responde midiendo, no razonando.

Los archivos USD se generan con la herramienta de IsaacLab:
    IsaacLab/scripts/tools/convert_urdf.py <urdf> assets/<nombre>.usd \
        --headless --joint-stiffness 0 --joint-damping 0
Viven en assets/ y no se versionan (son binarios regenerables).
"""
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

ASSET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")

# Altura del pelvis con las piernas en la pose nominal, del modelo original.
STANDING_HEIGHT = 0.793

MODELS = {
    "12dof": os.path.join(ASSET_DIR, "g1_12dof.usd"),
    "29dof": os.path.join(ASSET_DIR, "g1_29dof.usd"),
}

# Ganancias para las articulaciones que NO controla la policy de locomocion.
#
# La CINTURA va mucho mas rigida que los brazos, y el motivo es importante: en
# el modelo de 12 la cintura no existe — el torso es una sola pieza con el
# pelvis. Para que el cuerpo de 29 se parezca al que la policy conoce, la
# cintura tiene que comportarse como si estuviera soldada. Si se bambolea, el
# torso entero se mueve y la policy (que solo controla piernas) no tiene con
# que compensarlo.
ARM_STIFFNESS = 40.0     # oficial: los motores N5020-16 de los brazos
ARM_DAMPING = 1.0
WAIST_STIFFNESS = 200.0  # oficial: waist_yaw lleva 200; roll/pitch 40
WAIST_DAMPING = 5.0

# Limites reales de cada motor del G1, de la configuracion oficial de Unitree
# para IsaacLab (unitree_rl_lab/assets/robots/unitree.py). El G1 monta cuatro
# modelos de motor distintos y cada uno tiene su fuerza y velocidad maximas.
# Ponerles a todos un limite generoso e igual (lo que haciamos antes: 300 N.m)
# le da al robot musculos que no tiene.
MOTOR_LIMITS = {
    # motor            fuerza max [N.m], velocidad max [rad/s]
    "hip_pitch_yaw":   (88.0, 32.0),    # N7520-14.3
    "hip_roll_knee":   (139.0, 20.0),   # N7520-22.5
    "ankle_arm_waist": (25.0, 37.0),    # N5020-16
    "wrist":           (5.0, 22.0),     # W4010-25
}


def make_g1_cfg(cfg: dict, model: str = "12dof", prim_path: str = "/World/G1") -> ArticulationCfg:
    """Arma la configuracion del robot a partir del yaml de locomocion.

    model: "12dof" (el cuerpo que la policy conoce) o "29dof" (con brazos).
    """
    if model not in MODELS:
        raise ValueError(f"modelo desconocido: {model}. Opciones: {list(MODELS)}")

    leg_names = cfg["joint_names"]
    leg_angles = cfg["default_angles"]
    kps = cfg["kps"]
    kds = cfg["kds"]

    # Las piernas se agrupan POR MODELO DE MOTOR, como en la configuracion
    # oficial: cada grupo tiene los limites de fuerza y velocidad de su motor,
    # y las ganancias que le corresponden a cada articulacion.
    def ganancias(patron_joints):
        return ({n: float(k) for n, k in zip(leg_names, kps) if n in patron_joints},
                {n: float(d) for n, d in zip(leg_names, kds) if n in patron_joints})

    caderas_yaw_pitch = [n for n in leg_names if "hip_pitch" in n or "hip_yaw" in n]
    caderas_roll_rodillas = [n for n in leg_names if "hip_roll" in n or "knee" in n]
    tobillos = [n for n in leg_names if "ankle" in n]

    actuadores = {}
    for etiqueta, joints, motor in (
        ("hip_pitch_yaw", caderas_yaw_pitch, "hip_pitch_yaw"),
        ("hip_roll_knee", caderas_roll_rodillas, "hip_roll_knee"),
        ("ankles", tobillos, "ankle_arm_waist"),
    ):
        if not joints:
            continue
        k, d = ganancias(joints)
        fuerza, velocidad = MOTOR_LIMITS[motor]
        actuadores[etiqueta] = ImplicitActuatorCfg(
            joint_names_expr=joints,
            effort_limit_sim=fuerza,
            velocity_limit_sim=velocidad,
            stiffness=k,
            damping=d,
            armature=0.01,
        )

    # El modelo con brazos necesita quien sostenga todo lo que la policy de
    # locomocion no toca: brazos, muñecas y cintura.
    if model != "12dof":
        actuadores["arms"] = ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_.*", ".*_elbow_.*", ".*_wrist_roll.*"],
            effort_limit_sim=MOTOR_LIMITS["ankle_arm_waist"][0],
            velocity_limit_sim=MOTOR_LIMITS["ankle_arm_waist"][1],
            stiffness=ARM_STIFFNESS,
            damping=ARM_DAMPING,
            armature=0.01,
        )
        actuadores["wrists"] = ImplicitActuatorCfg(
            joint_names_expr=[".*_wrist_pitch.*", ".*_wrist_yaw.*"],
            effort_limit_sim=MOTOR_LIMITS["wrist"][0],
            velocity_limit_sim=MOTOR_LIMITS["wrist"][1],
            stiffness=ARM_STIFFNESS,
            damping=ARM_DAMPING,
            armature=0.01,
        )
        actuadores["waist"] = ImplicitActuatorCfg(
            joint_names_expr=["waist_.*"],
            effort_limit_sim=MOTOR_LIMITS["hip_pitch_yaw"][0],
            velocity_limit_sim=MOTOR_LIMITS["hip_pitch_yaw"][1],
            stiffness={"waist_yaw_joint": WAIST_STIFFNESS,
                       "waist_roll_joint": 40.0, "waist_pitch_joint": 40.0},
            damping=WAIST_DAMPING,
            armature=0.01,
        )

    return ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=MODELS[model],
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
            # Piernas en la pose nominal de la policy. Las demas articulaciones
            # (brazos, cintura) no se listan: arrancan en cero por defecto, que
            # es justo la pose en la que estan soldados los brazos del modelo
            # de 12 — asi el cuerpo se parece al que la policy conoce.
            # (No se puede poner ".*" ademas de los nombres: IsaacLab rechaza
            # que una articulacion coincida con dos patrones a la vez.)
            joint_pos={n: a for n, a in zip(leg_names, leg_angles)},
            joint_vel={".*": 0.0},
        ),
        soft_joint_pos_limit_factor=0.9,
        actuators=actuadores,
    )


# Compatibilidad con el nombre anterior
def make_g1_12dof_cfg(cfg: dict, prim_path: str = "/World/G1") -> ArticulationCfg:
    return make_g1_cfg(cfg, model="12dof", prim_path=prim_path)
