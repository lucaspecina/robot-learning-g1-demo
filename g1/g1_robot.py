#!/usr/bin/env python3
"""El robot G1 simulado: fisica + locomocion, con el lazo de control cerrado.

Este proceso ocupa el lugar del robot completo — cuerpo, computadora de control
y locomocion — igual que el robot real es una unidad. Su interfaz con el resto
del sistema es la misma que tendra el robot fisico:

  recibe:  /cmd_vel          (geometry_msgs/Twist — a que velocidad ir)
           /g1/payload_request (std_msgs/String — agregar o retirar carga)
  publica: /g1/odom          (nav_msgs/Odometry — donde esta el cuerpo)
           /g1/joint_states  (sensor_msgs/JointState — estado del cuerpo completo)
           /g1/arm_status    (std_msgs/String — orden y medición de los brazos)
           /g1/payload_status (std_msgs/String — masa física verificada)

Por que fisica y locomocion van juntas: un bipedo necesita que la decision de
la policy y el paso de fisica esten sincronizados. Separarlos en procesos
asincronos abre el lazo en el tiempo — el robot ejecuta ordenes calculadas para
un estado viejo, con un retraso variable — y se cae. En el robot real, la
policy tambien vive pegada al hardware por la misma razon.

La locomocion sigue siendo reemplazable: es una clase con interfaz fija
(ver locomotion.py). La modularidad del sistema esta en /cmd_vel, que es donde
el robot real tambien la tiene.

Uso:
    source ~/go2-lab/isaac_ros_env.sh
    bash run_g1.sh wbc 29dof 0 --free
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import yaml

# --- 1. Arrancar Isaac ANTES de importar nada de omni/isaaclab ---
from isaaclab.app import AppLauncher

_here = os.path.dirname(os.path.abspath(__file__))
parser = argparse.ArgumentParser(description="Robot G1 simulado (fisica + locomocion)")
parser.add_argument("--config", default=os.path.join(_here, "config", "g1_locomotion.yaml"))
parser.add_argument(
    "--locomotion",
    choices=["wbc", "legacy", "stand"],
    default="wbc",
    help="wbc usa el conjunto oficial NVIDIA AGILE; legacy conserva la base anterior",
)
parser.add_argument("--policy", default=None, help="archivo .pt de la locomoción elegida")
parser.add_argument(
    "--policy-descriptor",
    default=None,
    help="contrato YAML de entradas y salidas; obligatorio para NVIDIA AGILE",
)
parser.add_argument(
    "--pelvis-height",
    type=float,
    default=0.72,
    help="altura solicitada a la locomoción de NVIDIA AGILE",
)
parser.add_argument("--settle_s", type=float, default=0.0,
                    help="segundos con las piernas rigidas antes de soltar la policy; "
                         "en un bipedo conviene 0: rigido se cae, la policy es quien lo para")
parser.add_argument("--model", default="12dof", choices=["12dof", "29dof"],
                    help="cuerpo del robot: 12dof (el que la policy conoce) o "
                         "29dof (con brazos, el que necesita la demo)")
parser.add_argument("--payload_kg", type=float, default=0.0,
                    help="masa extra repartida entre las dos manos, para medir "
                         "cuanta carga tolera la locomocion (la policy de "
                         "Unitree fue entrenada con el robot vacio)")
parser.add_argument("--scene", action="store_true",
                    help="construir la habitación de la demo (reloj, mesas y objetos)")
parser.add_argument("--camera", action="store_true",
                    help="montar la camara de la cabeza y publicarla por ROS 2")
parser.add_argument("--camera_hz", type=float, default=3.0,
                    help="imagenes por segundo de la camara del robot. Cada una "
                         "cuesta un render completo: bajarla de 10 a 3 recupera "
                         "casi un tercio de la velocidad de simulacion, y a la "
                         "percepcion le alcanza de sobra")
parser.add_argument(
    "--lidar",
    action="store_true",
    help="EXPERIMENTAL: montar LiDAR RTX; hoy no pasa la prueba integrada",
)
parser.add_argument(
    "--lidar-profile",
    default="Example_Rotary",
    help="perfil RTX; el inicial es provisional hasta confirmar el hardware",
)
parser.add_argument("--render_hz", type=float, default=20.0,
                    help="cuadros por segundo a dibujar (mas bajo = simulacion mas rapida)")
parser.add_argument(
    "--follow-viewer",
    action="store_true",
    help="hace que la vista 3D siga al robot; apagado permite moverla a mano",
)
parser.add_argument("--free", action="store_true",
                    help="arrancar con la policy andando directamente, sin pasar "
                         "por el estado congelado (para pruebas automaticas)")
parser.add_argument(
    "--frozen-height",
    type=float,
    default=0.74,
    help="altura del robot mientras espera congelado antes de comenzar",
)
parser.add_argument(
    "--leg-effort-mode",
    choices=["wide", "rated"],
    default="wide",
    help="wide conserva 300 N.m; rated usa 88/139/50 N.m del URDF",
)
parser.add_argument(
    "--leg-velocity-mode",
    choices=["wide", "rated"],
    default="wide",
    help="wide conserva 100 rad/s; rated usa 32/20/37 rad/s como tope duro",
)
parser.add_argument(
    "--diagnostics-path",
    default=None,
    help="CSV con pose, articulaciones, objetivos y par pedido a 50 Hz",
)
parser.add_argument(
    "--experiment-duration-s",
    type=float,
    default=0.0,
    help="terminar solo después de esta cantidad de segundos simulados",
)
parser.add_argument(
    "--experiment-command-x",
    type=float,
    default=None,
    help="reemplazar ROS por una velocidad longitudinal constante durante la prueba",
)
parser.add_argument(
    "--experiment-command-start-s",
    type=float,
    default=0.0,
    help="segundo simulado en el que comienza la orden automática",
)
parser.add_argument(
    "--experiment-command-stop-s",
    type=float,
    default=None,
    help="segundo simulado en el que termina la orden automática",
)
parser.add_argument(
    "--experiment-arm-pose",
    choices=["reposo", "listo", "transporte"],
    default=None,
    help="mantener una pose de brazos durante una prueba automática",
)
parser.add_argument(
    "--foot-contact-offset",
    type=float,
    default=None,
    help="margen de detección de contacto aplicado sólo a las ocho esferas de los pies",
)
parser.add_argument(
    "--solver-position-iterations",
    type=int,
    default=8,
    help="iteraciones de corrección de posición; el entrenamiento usó 4",
)
parser.add_argument(
    "--solver-velocity-iterations",
    type=int,
    default=4,
    help="iteraciones de corrección de velocidad; el entrenamiento usó 0",
)
AppLauncher.add_app_launcher_args(parser)
args_cli, _extra = parser.parse_known_args()
sys.argv = [sys.argv[0]] + [a for a in _extra if a.startswith("--/")]

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- 2. Ahora si, el resto ---
import numpy as np
import torch

from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.simulation_manager import SimulationManager

enable_extension("isaacsim.ros2.bridge")

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from pxr import Gf, UsdGeom

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String as RosString

sys.path.insert(0, _here)
from arm_control import ARM_JOINTS, POSES, PoseArmController  # noqa: E402
from demo_scene import build_demo_scene  # noqa: E402
from g1_asset import make_g1_cfg, make_wbc_agile_g1_cfg  # noqa: E402
from perception import (  # noqa: E402
    CAMERA_PARENT_PRIM,
    CameraPublisher,
    make_camera_cfg,
)
from locomotion import (  # noqa: E402
    LocomotionState,
    RLPolicyLocomotion,
    StandStillLocomotion,
    WbcAgileLocomotion,
    gravity_in_body_frame,
)
from payload_core import (  # noqa: E402
    parse_payload_request,
    payload_geometry_measurements,
    payload_mass_values,
    select_payload_body_indices,
)


PAYLOAD_STATE_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class PayloadMassController:
    """Cambia masa real en dos muñecas sin acumular órdenes repetidas."""

    def __init__(self, robot):
        self.robot = robot
        self.body_indices = select_payload_body_indices(robot.body_names)
        self.body_names = [
            robot.body_names[index]
            for index in self.body_indices
        ]
        self.baseline_masses = (
            robot.root_physx_view.get_masses()[0].detach().clone()
        )
        self.current_mass_kg = 0.0

    def set_mass(self, mass_kg: float) -> dict:
        target_values = payload_mass_values(
            self.baseline_masses.cpu().tolist(),
            self.body_indices,
            mass_kg,
        )
        masses = self.robot.root_physx_view.get_masses()
        masses[0] = torch.tensor(
            target_values,
            dtype=masses.dtype,
            device=masses.device,
        )
        self.robot.root_physx_view.set_masses(masses, torch.arange(1))
        verified = self.robot.root_physx_view.get_masses()[0]
        baseline_total = float(
            self.baseline_masses[self.body_indices].sum().item()
        )
        final_total = float(verified[self.body_indices].sum().item())
        applied_mass_kg = final_total - baseline_total
        if abs(applied_mass_kg - mass_kg) > 1e-4:
            raise RuntimeError(
                f"se pidieron {mass_kg:.3f} kg pero Isaac aplicó "
                f"{applied_mass_kg:.3f} kg"
            )
        self.current_mass_kg = float(mass_kg)
        return {
            "state": "attached" if mass_kg > 0.0 else "detached",
            "attached": mass_kg > 0.0,
            "requested_mass_kg": round(float(mass_kg), 3),
            "applied_mass_kg": round(applied_mass_kg, 3),
            "mass_per_point_kg": round(float(mass_kg) / 2.0, 3),
            "attachment_points": self.body_names,
            "physical_model": "mass_split_between_wrists",
            "grasp_validated": False,
        }


class PayloadVisual:
    """Muestra el bulto entre las muñecas sin duplicar su masa física."""

    PRIM_PATH = "/World/carried_payload_visual"

    @classmethod
    def spawn(cls):
        cfg = sim_utils.CuboidCfg(
            size=(0.20, 0.14, 0.14),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.95, 0.58, 0.10),
            ),
        )
        cfg.func(cls.PRIM_PATH, cfg, translation=(0.0, 0.0, -10.0))
        return cls()

    def __init__(self):
        import omni.usd

        prim = omni.usd.get_context().get_stage().GetPrimAtPath(self.PRIM_PATH)
        if not prim.IsValid():
            raise RuntimeError("no se pudo crear el objeto visual transportado")
        self.imageable = UsdGeom.Imageable(prim)
        xformable = UsdGeom.Xformable(prim)
        self.translation_op = next(
            (
                operation
                for operation in xformable.GetOrderedXformOps()
                if operation.GetOpType() == UsdGeom.XformOp.TypeTranslate
            ),
            None,
        )
        if self.translation_op is None:
            self.translation_op = xformable.AddTranslateOp()
        self.attached = False
        self.imageable.MakeInvisible()

    def set_attached(self, attached: bool):
        self.attached = bool(attached)
        if self.attached:
            self.imageable.MakeVisible()
        else:
            self.imageable.MakeInvisible()

    def update(self, wrist_positions):
        if not self.attached:
            return
        midpoint = wrist_positions.mean(dim=0).detach().cpu().numpy()
        self.translation_op.Set(
            Gf.Vec3d(float(midpoint[0]), float(midpoint[1]), float(midpoint[2]))
        )


def euler_from_quaternion(quaternion):
    """Devuelve inclinación lateral, frontal y rumbo, en radianes."""
    w, x, y, z = quaternion
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = 2.0 * (w * y - z * x)
    pitch = math.asin(float(np.clip(pitch_term, -1.0, 1.0)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class ExperimentDiagnostics:
    """Registra lo necesario para separar causa física de síntoma."""

    def __init__(
        self,
        path,
        joint_names,
        effort_limits,
        velocity_limits,
        kps,
        kds,
    ):
        self.joint_names = list(joint_names)
        self.effort_limits = np.asarray(effort_limits, dtype=np.float64)
        self.velocity_limits = np.asarray(velocity_limits, dtype=np.float64)
        self.kps = np.asarray(kps, dtype=np.float64)
        self.kds = np.asarray(kds, dtype=np.float64)
        self.initial_xy = None
        self.last_report_second = -1
        self.max_displacement = 0.0
        self.max_abs_velocity = np.zeros(len(joint_names), dtype=np.float64)
        self.max_abs_torque = np.zeros(len(joint_names), dtype=np.float64)
        self.min_height = float("inf")
        self.last_time = 0.0
        self.last_displacement = 0.0
        self.file = None
        self.writer = None

        if path:
            directory = os.path.dirname(os.path.abspath(path))
            os.makedirs(directory, exist_ok=True)
            self.file = open(path, "w", newline="", encoding="utf-8")
            self.writer = csv.writer(self.file)
            header = [
                "sim_time_s",
                "x_m",
                "y_m",
                "height_m",
                "displacement_m",
                "world_vx_mps",
                "world_vy_mps",
                "world_speed_mps",
                "roll_rad",
                "pitch_rad",
                "yaw_rad",
                "command_x_mps",
                "command_y_mps",
                "command_yaw_radps",
                "max_abs_joint_velocity_radps",
                "max_abs_requested_torque_nm",
                "velocity_limit_ratio",
                "effort_limit_ratio",
            ]
            for name in self.joint_names:
                header.extend(
                    [
                        f"{name}.position_rad",
                        f"{name}.velocity_radps",
                        f"{name}.target_rad",
                        f"{name}.requested_torque_nm",
                    ]
                )
            self.writer.writerow(header)

    def sample(
        self,
        sim_time,
        command,
        q,
        dq,
        target,
        root_pos,
        root_quat,
        root_lin_vel,
    ):
        q = np.asarray(q, dtype=np.float64)
        dq = np.asarray(dq, dtype=np.float64)
        target = np.asarray(target, dtype=np.float64)
        root_pos = np.asarray(root_pos, dtype=np.float64)
        root_lin_vel = np.asarray(root_lin_vel, dtype=np.float64)
        requested_torque = self.kps * (target - q) - self.kds * dq

        if self.initial_xy is None:
            self.initial_xy = root_pos[:2].copy()
        displacement = float(np.linalg.norm(root_pos[:2] - self.initial_xy))
        world_speed = float(np.linalg.norm(root_lin_vel[:2]))
        roll, pitch, yaw = euler_from_quaternion(root_quat)
        velocity_ratio = float(
            np.max(np.abs(dq) / np.maximum(self.velocity_limits, 1e-9))
        )
        effort_ratio = float(
            np.max(np.abs(requested_torque) / np.maximum(self.effort_limits, 1e-9))
        )

        self.max_displacement = max(self.max_displacement, displacement)
        self.max_abs_velocity = np.maximum(self.max_abs_velocity, np.abs(dq))
        self.max_abs_torque = np.maximum(
            self.max_abs_torque, np.abs(requested_torque)
        )
        self.min_height = min(self.min_height, float(root_pos[2]))
        self.last_time = sim_time
        self.last_displacement = displacement

        if self.writer is not None:
            row = [
                sim_time,
                root_pos[0],
                root_pos[1],
                root_pos[2],
                displacement,
                root_lin_vel[0],
                root_lin_vel[1],
                world_speed,
                roll,
                pitch,
                yaw,
                command[0],
                command[1],
                command[2],
                float(np.max(np.abs(dq))),
                float(np.max(np.abs(requested_torque))),
                velocity_ratio,
                effort_ratio,
            ]
            for position, velocity, desired, torque in zip(
                q, dq, target, requested_torque
            ):
                row.extend([position, velocity, desired, torque])
            self.writer.writerow(row)

        whole_second = int(sim_time)
        if whole_second > self.last_report_second:
            self.last_report_second = whole_second
            average = displacement / sim_time if sim_time > 0 else 0.0
            print(
                f"\n[diagnóstico] t={sim_time:.1f} s  deriva {displacement:.3f} m "
                f"({average:.3f} m/s)  altura {root_pos[2]:.3f} m  "
                f"|dq|/límite {velocity_ratio:.2f}  "
                f"|par|/límite {effort_ratio:.2f}",
                flush=True,
            )
            if self.file is not None:
                self.file.flush()

    def close(self):
        average = (
            self.last_displacement / self.last_time if self.last_time > 0 else 0.0
        )
        print(
            f"\n[diagnóstico final] t={self.last_time:.2f} s  "
            f"deriva final {self.last_displacement:.3f} m "
            f"({average:.3f} m/s)  máximo {self.max_displacement:.3f} m  "
            f"altura mínima {self.min_height:.3f} m",
            flush=True,
        )
        for name, velocity, velocity_limit, torque, effort_limit in zip(
            self.joint_names,
            self.max_abs_velocity,
            self.velocity_limits,
            self.max_abs_torque,
            self.effort_limits,
        ):
            print(
                f"[diagnóstico articulación] {name}: "
                f"|dq| {velocity:.2f}/{velocity_limit:.1f} rad/s, "
                f"|par pedido| {torque:.1f}/{effort_limit:.1f} N.m",
                flush=True,
            )
        if self.file is not None:
            self.file.close()


class G1RobotNode(Node):
    """La cara ROS 2 del robot: recibe intencion, publica estado."""

    def __init__(self, cfg, joint_names):
        super().__init__("g1_robot")
        self.joint_names = list(joint_names)
        self.max_lin = cfg["max_lin_vel"]
        self.max_ang = cfg["max_ang_vel"]
        self.cmd_timeout = cfg["cmd_timeout"]

        self.command = np.zeros(3, dtype=np.float32)
        self.last_cmd_time = 0.0

        self.arm_pose_request = None   # lo lee el lazo principal
        self.payload_request = None    # la física sólo cambia en el lazo principal
        self.reset_request = False     # idem

        # El robot nace CONGELADO: cargado, de pie y quieto, sin que la policy
        # intervenga. Asi se puede acomodar el cliente de Isaac y el tablero
        # con calma, y soltar cuando uno quiere (run_demo.sh start). Volver a
        # congelar (freeze) lo teletransporta al origen sin recargar Isaac.
        self.frozen = not args_cli.free

        self.pub_state = self.create_publisher(JointState, "/g1/joint_states", 10)
        self.pub_odom = self.create_publisher(Odometry, "/g1/odom", 10)
        self.pub_arm_status = self.create_publisher(
            RosString, "/g1/arm_status", 10
        )
        self.pub_robot_status = self.create_publisher(
            RosString, "/g1/robot_status", 10
        )
        self.pub_payload_status = self.create_publisher(
            RosString, "/g1/payload_status", PAYLOAD_STATE_QOS
        )
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(RosString, "/g1/arm_pose", self.on_arm_pose, 10)
        self.create_subscription(
            RosString,
            "/g1/payload_request",
            self.on_payload_request,
            10,
        )
        self.create_subscription(RosString, "/g1/reset", self.on_reset, 10)
        self.create_subscription(RosString, "/g1/control", self.on_control, 10)

    def on_control(self, msg: RosString):
        """start = soltar la policy | freeze = volver al origen, quieto."""
        order = msg.data.strip().lower()
        if order == "start":
            self.frozen = False
        elif order == "freeze":
            self.frozen = True
            self.payload_request = {
                "request_id": "robot_freeze",
                "command": "detach",
                "mass_kg": 0.0,
            }

    def on_reset(self, msg: RosString):
        """Pide devolver el robot al punto de partida."""
        self.reset_request = True
        self.payload_request = {
            "request_id": "robot_reset",
            "command": "detach",
            "mass_kg": 0.0,
        }

    def on_arm_pose(self, msg: RosString):
        """Pide una pose de brazos por nombre: reposo | listo | transporte."""
        self.arm_pose_request = msg.data.strip()

    def on_payload_request(self, msg: RosString):
        """Valida la orden aquí; aplicar masa dentro del callback sería asíncrono."""
        try:
            self.payload_request = parse_payload_request(msg.data)
        except ValueError as error:
            self.publish_payload_status({
                "state": "failed",
                "attached": False,
                "error": str(error),
                "grasp_validated": False,
            })

    def publish_payload_status(self, status: dict):
        self.pub_payload_status.publish(
            RosString(data=json.dumps(status, ensure_ascii=False))
        )

    def on_cmd_vel(self, msg: Twist):
        self.command = np.array([
            float(np.clip(msg.linear.x, -self.max_lin, self.max_lin)),
            float(np.clip(msg.linear.y, -self.max_lin, self.max_lin)),
            float(np.clip(msg.angular.z, -self.max_ang, self.max_ang)),
        ], dtype=np.float32)
        self.last_cmd_time = time.time()

    def current_command(self):
        """Comando vigente, con hombre muerto: sin cmd fresco, velocidad cero."""
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            return np.zeros(3, dtype=np.float32)
        return self.command

    def publish(self, joint_pos, joint_vel, root_pos, root_quat, lin_vel, ang_vel):
        stamp = self.get_clock().now().to_msg()

        state = JointState()
        state.header.stamp = stamp
        state.name = self.joint_names
        state.position = joint_pos.tolist()
        state.velocity = joint_vel.tolist()
        self.pub_state.publish(state)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "pelvis"
        odom.pose.pose.position.x = float(root_pos[0])
        odom.pose.pose.position.y = float(root_pos[1])
        odom.pose.pose.position.z = float(root_pos[2])
        odom.pose.pose.orientation.w = float(root_quat[0])
        odom.pose.pose.orientation.x = float(root_quat[1])
        odom.pose.pose.orientation.y = float(root_quat[2])
        odom.pose.pose.orientation.z = float(root_quat[3])
        odom.twist.twist.linear.x = float(lin_vel[0])
        odom.twist.twist.linear.y = float(lin_vel[1])
        odom.twist.twist.linear.z = float(lin_vel[2])
        odom.twist.twist.angular.x = float(ang_vel[0])
        odom.twist.twist.angular.y = float(ang_vel[1])
        odom.twist.twist.angular.z = float(ang_vel[2])
        self.pub_odom.publish(odom)
        # Una pose inmóvil no demuestra que el equilibrio esté funcionando:
        # el modo congelado reescribe el estado y puede producir un falso éxito.
        self.pub_robot_status.publish(RosString(data=json.dumps({
            "mode": "frozen" if self.frozen else "active",
        })))

    def publish_arm_status(
        self,
        pose_name,
        joint_names,
        target,
        actual,
    ):
        """Confirma la orden con mediciones; publicar el pedido no prueba movimiento."""
        target = np.asarray(target, dtype=np.float32)
        actual = np.asarray(actual, dtype=np.float32)
        errors = np.abs(target - actual)
        # Las muñecas del modelo oficial tienen mucha menos rigidez que
        # hombros y codos. La prueba activa midió un piso repetible de 2,4°
        # bajo gravedad; exigirles 1,7° convertía una limitación conocida del
        # motor en un falso fallo de toda la pose.
        tolerances = np.asarray(
            [0.05 if "_wrist_" in name else 0.03 for name in joint_names],
            dtype=np.float32,
        )
        max_error = float(np.max(errors))
        max_error_ratio = float(np.max(errors / tolerances))
        status = {
            "pose": pose_name,
            "mode": "frozen_preview" if self.frozen else "active",
            "joint_names": list(joint_names),
            "target_rad": target.round(4).tolist(),
            "actual_rad": actual.round(4).tolist(),
            "tolerance_rad": tolerances.round(4).tolist(),
            "max_error_rad": round(max_error, 4),
            "max_error_ratio": round(max_error_ratio, 4),
            "reached": max_error_ratio < 1.0,
        }
        self.pub_arm_status.publish(
            RosString(data=json.dumps(status, ensure_ascii=False))
        )


def main():
    with open(args_cli.config, "r") as f:
        cfg = yaml.safe_load(f)

    if args_cli.locomotion == "wbc":
        if not args_cli.policy or not args_cli.policy_descriptor:
            raise ValueError(
                "NVIDIA AGILE necesita --policy y --policy-descriptor; "
                "run_g1.sh los completa automáticamente"
            )
        controller = WbcAgileLocomotion(
            args_cli.policy,
            args_cli.policy_descriptor,
            pelvis_height=args_cli.pelvis_height,
            device="cpu",
        )
        action_joint_names = controller.action_joint_names
        observation_joint_names = controller.observation_joint_names
        default_action_angles = controller.action_offset
        diagnostic_stiffness = controller.action_stiffness
        diagnostic_damping = controller.action_damping
        robot_cfg = make_wbc_agile_g1_cfg()

        # La policy elegida fue entrenada realmente a 200 Hz de física y
        # decide a 50 Hz. Los comentarios antiguos de AGILE dicen 500 Hz,
        # pero su configuración ejecutable y nuestras mediciones dan 200 Hz.
        physics_dt = 0.005
        control_dt = 0.02
        print(
            f"[robot] locomoción: NVIDIA AGILE "
            f"({os.path.basename(args_cli.policy)})",
            flush=True,
        )
    else:
        action_joint_names = list(cfg["joint_names"])
        observation_joint_names = action_joint_names
        default_action_angles = np.asarray(cfg["default_angles"], dtype=np.float32)
        diagnostic_stiffness = np.asarray(cfg["kps"], dtype=np.float32)
        diagnostic_damping = np.asarray(cfg["kds"], dtype=np.float32)
        robot_cfg = make_g1_cfg(
            cfg,
            model=args_cli.model,
            leg_effort_mode=args_cli.leg_effort_mode,
            leg_velocity_mode=args_cli.leg_velocity_mode,
            solver_position_iterations=args_cli.solver_position_iterations,
            solver_velocity_iterations=args_cli.solver_velocity_iterations,
        )
        physics_dt = 0.002
        control_dt = cfg["control_dt"]
        if args_cli.locomotion == "legacy":
            if not args_cli.policy:
                raise ValueError("la locomoción anterior necesita --policy")
            controller = RLPolicyLocomotion(cfg, args_cli.policy, device="cpu")
            print(
                f"[robot] locomoción anterior: {os.path.basename(args_cli.policy)}",
                flush=True,
            )
        else:
            controller = StandStillLocomotion(cfg)
            print("[robot] locomoción: sostener pose para diagnóstico", flush=True)

    steps_per_control = max(1, round(control_dt / physics_dt))
    render_every = max(1, round((1.0 / args_cli.render_hz) / physics_dt)) if args_cli.render_hz > 0 else 10**9

    # --- escena ---
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=physics_dt, device=args_cli.device)
    )
    # La vista del operador nace fuera y muy por encima de las paredes para
    # mostrar la habitación como una maqueta sin perder la perspectiva. Esto
    # no es un sensor del robot ni modifica lo que recibe la cámara frontal.
    sim.set_camera_view(eye=(-4.5, -7.0, 10.5), target=(2.0, 0.25, 0.4))
    # Piso con la MISMA friccion que el simulador donde Unitree valida la
    # policy. IsaacLab trae 0.5 por defecto; MuJoCo usa 1.0. Con la mitad de
    # agarre los pies patinan, y el robot deriva varios centimetros por segundo
    # aunque el comando sea cero. La policy no aprendio a caminar sobre hielo.
    suelo = sim_utils.GroundPlaneCfg(
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        )
    )
    suelo.func("/World/ground", suelo)
    light = sim_utils.DomeLightCfg(intensity=2000.0)
    light.func("/World/light", light)

    if args_cli.scene:
        build_demo_scene()

    robot = Articulation(robot_cfg)
    payload_visual = PayloadVisual.spawn()

    # Los ojos: se crean con la escena, antes del reset.
    camera = None
    if args_cli.camera:
        import omni.usd
        from isaaclab.sensors import Camera

        stage = omni.usd.get_context().get_stage()
        if not stage.GetPrimAtPath(CAMERA_PARENT_PRIM).IsValid():
            raise RuntimeError(
                "no existe la pieza de la cabeza donde debe montarse la "
                f"cámara: {CAMERA_PARENT_PRIM}"
            )
        camera = Camera(make_camera_cfg(update_period=1.0 / args_cli.camera_hz))

    lidar_bridge = None
    if args_cli.lidar:
        import omni.usd
        from lidar import LIDAR_PARENT_PRIM, LidarBridge

        stage = omni.usd.get_context().get_stage()
        if not stage.GetPrimAtPath(LIDAR_PARENT_PRIM).IsValid():
            raise RuntimeError(
                "no existe la pieza de la cabeza donde debe montarse el "
                f"LiDAR: {LIDAR_PARENT_PRIM}"
            )
        lidar_bridge = LidarBridge(profile=args_cli.lidar_profile)

    sim.reset()
    if lidar_bridge is not None:
        lidar_bridge.initialize()
    print(f"[robot] articulaciones del modelo: {robot.num_joints} "
          f"({', '.join(robot.joint_names[:3])}...)", flush=True)

    # La salida siempre controla 12 piernas. AGILE además observa el cuerpo
    # completo para compensar el movimiento y la carga de los brazos.
    sim_ids = [robot.joint_names.index(name) for name in action_joint_names]
    observation_ids = [
        robot.joint_names.index(name) for name in observation_joint_names
    ]
    default_angles = torch.tensor(
        default_action_angles, dtype=torch.float32, device=args_cli.device
    )
    effort_limits = robot.data.joint_effort_limits[0, sim_ids].cpu().numpy()
    velocity_limits = robot.data.joint_vel_limits[0, sim_ids].cpu().numpy()
    physics_view = SimulationManager.get_physics_sim_view()
    foot_physx_view = physics_view.create_rigid_body_view(
        [
            "/World/G1/left_ankle_roll_link",
            "/World/G1/right_ankle_roll_link",
        ]
    )
    foot_contact_offsets = foot_physx_view.get_contact_offsets().clone()
    if args_cli.foot_contact_offset is not None:
        foot_contact_offsets.fill_(args_cli.foot_contact_offset)
        foot_physx_view.set_contact_offsets(
            foot_contact_offsets,
            torch.arange(foot_physx_view.count, device="cpu"),
        )
        foot_contact_offsets = foot_physx_view.get_contact_offsets().clone()
    contact_offsets = robot.root_physx_view.get_contact_offsets().cpu().numpy()
    rest_offsets = robot.root_physx_view.get_rest_offsets().cpu().numpy()
    print(
        f"[robot] límites efectivos de piernas: fuerza={effort_limits.tolist()}, "
        f"velocidad={velocity_limits.tolist()}",
        flush=True,
    )
    articulation_props = robot_cfg.spawn.articulation_props
    print(
        f"[robot] solucionador configurado: "
        f"posición={articulation_props.solver_position_iteration_count}, "
        f"velocidad={articulation_props.solver_velocity_iteration_count}",
        flush=True,
    )
    print(
        f"[robot] contacto de pies: forma={tuple(foot_contact_offsets.shape)}, "
        f"contact_offset={foot_contact_offsets.cpu().numpy().tolist()}",
        flush=True,
    )
    print(
        f"[robot] contacto efectivo: forma={contact_offsets.shape}, "
        f"contact_offset={np.unique(contact_offsets).tolist()}, "
        f"rest_offset={np.unique(rest_offsets).tolist()}",
        flush=True,
    )

    # PASO OBLIGATORIO: escribir el estado inicial. Sin esto el articulado
    # arranca colapsado en el piso por mas que su configuracion declare que
    # esta de pie.
    #
    # Y un detalle que importa: las piernas arrancan en la pose nominal DE LA
    # POLICY, no en la que trae la configuracion de IsaacLab (que es parecida
    # pero no igual). Si arrancaran distintas, el robot se moveria en el primer
    # instante para alcanzar el objetivo y se desestabilizaria justo cuando la
    # policy todavia no tomo el control. Ademas la policy espera partir de una
    # postura que conoce.
    root_state = robot.data.default_root_state.clone()
    if args_cli.locomotion == "wbc" and not args_cli.free:
        # El entorno de entrenamiento deja caer al robot desde 0,90 m para
        # variar el arranque. En la demo congelada eso se vería como levitar;
        # 0,74 m es la altura estable medida antes de soltar la física.
        root_state[:, 2] = args_cli.frozen_height
    home_root_state = root_state.clone()
    robot.write_root_pose_to_sim(root_state[:, :7])
    robot.write_root_velocity_to_sim(root_state[:, 7:])

    joint_pos = robot.data.default_joint_pos.clone()
    joint_pos[0, sim_ids] = default_angles
    robot.write_joint_state_to_sim(joint_pos, robot.data.default_joint_vel.clone())
    robot.reset()
    print(f"[robot] estado inicial: altura {float(root_state[0, 2]):.3f} m, "
          f"piernas en la pose nominal de la policy", flush=True)

    if args_cli.locomotion != "wbc":
        # La base anterior necesita escribir estas ganancias a mano. En AGILE
        # ya forman parte del modelo de motores y tocarlas rompería el conjunto.
        kp = torch.tensor(
            cfg["kps"], dtype=torch.float32, device=args_cli.device
        ).unsqueeze(0)
        kd = torch.tensor(
            cfg["kds"], dtype=torch.float32, device=args_cli.device
        ).unsqueeze(0)
        robot.write_joint_stiffness_to_sim(kp, joint_ids=sim_ids)
        robot.write_joint_damping_to_sim(kd, joint_ids=sim_ids)

    # --- control de brazos (solo en el cuerpo que los tiene) ---
    tiene_brazos = args_cli.locomotion == "wbc" or args_cli.model != "12dof"
    if tiene_brazos:
        arm_ids = [robot.joint_names.index(n) for n in ARM_JOINTS]
        arms = PoseArmController()
        if args_cli.experiment_arm_pose is not None:
            arms.set_pose(args_cli.experiment_arm_pose)
        initial_arm_pose = args_cli.experiment_arm_pose or "reposo"
        print(
            f"[robot] brazos: {len(ARM_JOINTS)} articulaciones, "
            f"pose inicial {initial_arm_pose}",
            flush=True,
        )
    else:
        arm_ids, arms = None, None

    # La masa se aplica a las dos muñecas porque este cuerpo oficial desactiva
    # los dedos. El bulto visible es sólo una representación: separar ambos
    # modelos evita contar su peso dos veces y deja explícito que no hay agarre.
    payload_controller = None
    if tiene_brazos:
        payload_controller = PayloadMassController(robot)
        initial_payload_status = payload_controller.set_mass(args_cli.payload_kg)
        payload_visual.set_attached(args_cli.payload_kg > 0.0)
        print(
            f"[robot] carga inicial verificada: "
            f"{initial_payload_status['applied_mass_kg']:.2f} kg en "
            f"{initial_payload_status['attachment_points']}",
            flush=True,
        )
    elif args_cli.payload_kg > 0.0:
        raise RuntimeError("no se puede aplicar carga a un cuerpo sin brazos")

    rclpy.init()
    # joint_states representa al robot completo. Publicar sólo las piernas
    # ocultaba si una orden de brazos se había ejecutado realmente.
    node = G1RobotNode(cfg, robot.joint_names)
    if payload_controller is not None:
        node.publish_payload_status({
            "request_id": "startup",
            **initial_payload_status,
            "visual_object": args_cli.payload_kg > 0.0,
        })
    cam_pub = CameraPublisher(node, camera) if camera is not None else None
    if cam_pub is not None:
        print("[robot] camara de cabeza publicando en /g1/head_cam/image", flush=True)
    if lidar_bridge is not None:
        print(
            "[robot] AVISO: LiDAR experimental conectado a "
            "/g1/lidar/points; validar con check_lidar.py antes de usar "
            f"(perfil {lidar_bridge.profile})",
            flush=True,
        )

    # El robot arranca sosteniendo la pose nominal: la policy necesita partir
    # de una postura que conozca, no de una arbitraria.
    target = default_angles.clone().unsqueeze(0)
    robot.set_joint_position_target(target, joint_ids=sim_ids)

    settle_steps = int(args_cli.settle_s / physics_dt)
    step = 0
    t_wall = time.perf_counter()
    print(f"[robot] corriendo: fisica {1/physics_dt:.0f} Hz, control {1/control_dt:.0f} Hz, "
          f"asentando {args_cli.settle_s:.1f} s", flush=True)
    print("[robot] LISTO: ROS y simulación inicializados", flush=True)

    was_frozen = None
    experiment_start_step = None
    diagnostics = None
    if args_cli.diagnostics_path or args_cli.experiment_duration_s > 0:
        diagnostics = ExperimentDiagnostics(
            args_cli.diagnostics_path,
            action_joint_names,
            effort_limits,
            velocity_limits,
            diagnostic_stiffness,
            diagnostic_damping,
        )

    def consume_arm_pose_request():
        """Aplica una orden pendiente y deja una confirmación inequívoca en el log."""
        if arms is None or node.arm_pose_request is None:
            return
        requested_pose = node.arm_pose_request
        node.arm_pose_request = None
        try:
            arms.set_pose(requested_pose)
            print(f"\n[robot] brazos -> pose '{requested_pose}'", flush=True)
        except ValueError as error:
            print(f"\n[robot] brazos: {error}", flush=True)

    def consume_payload_request():
        """Aplica y relee la masa dentro del mismo lazo que avanza PhysX."""
        request = node.payload_request
        if request is None:
            return
        node.payload_request = None
        if payload_controller is None:
            node.publish_payload_status({
                "request_id": request["request_id"],
                "state": "failed",
                "attached": False,
                "error": "el cuerpo activo no tiene dos puntos de carga",
                "grasp_validated": False,
            })
            return
        try:
            status = payload_controller.set_mass(request["mass_kg"])
            wrist_positions = robot.data.body_pos_w[
                0,
                payload_controller.body_indices,
            ]
            status.update(payload_geometry_measurements(
                wrist_positions.detach().cpu().tolist(),
                robot.data.root_pos_w[0].detach().cpu().tolist(),
            ))
            payload_visual.set_attached(status["attached"])
            status.update({
                "request_id": request["request_id"],
                "visual_object": status["attached"],
            })
            node.publish_payload_status(status)
            print(
                f"\n[robot] carga -> {status['state']}: "
                f"{status['applied_mass_kg']:.2f} kg verificados en "
                f"{status['attachment_points']}",
                flush=True,
            )
        except (RuntimeError, ValueError) as error:
            node.publish_payload_status({
                "request_id": request["request_id"],
                "state": "failed",
                "attached": payload_controller.current_mass_kg > 0.0,
                "error": str(error),
                "grasp_validated": False,
            })

    while simulation_app.is_running():
        # --- congelado: clavado en el punto de partida, la policy no toca ---
        # La fisica sigue corriendo (para dibujar y para la camara), pero en
        # cada paso se reescribe el estado inicial: el robot ni se mueve ni se
        # cae. Se sale con la orden "start" en /g1/control.
        if node.frozen:
            if was_frozen is not True:
                if arms is not None:
                    arms.reset()
                if payload_controller is not None:
                    node.payload_request = {
                        "request_id": "robot_freeze",
                        "command": "detach",
                        "mass_kg": 0.0,
                    }
                print("\n[robot] CONGELADO en el punto de partida. "
                      "Soltar con: run_demo.sh start", flush=True)
                was_frozen = True
            if arms is not None and step % steps_per_control == 0:
                # En congelado es una vista previa: movemos la pose de forma
                # determinista, sin confundirla con una prueba de estabilidad.
                consume_arm_pose_request()
                arms.compute(control_dt)
            if step % steps_per_control == 0:
                consume_payload_request()
            robot.write_root_pose_to_sim(home_root_state[:, :7])
            robot.write_root_velocity_to_sim(
                torch.zeros_like(home_root_state[:, 7:]))
            frozen_pos = robot.data.default_joint_pos.clone()
            frozen_pos[0, sim_ids] = default_angles
            if arms is not None:
                frozen_pos[0, arm_ids] = torch.tensor(
                    arms.actual,
                    dtype=torch.float32,
                    device=args_cli.device,
                )
            robot.write_joint_state_to_sim(
                frozen_pos, torch.zeros_like(robot.data.default_joint_vel))
            robot.set_joint_position_target(default_angles.unsqueeze(0),
                                            joint_ids=sim_ids)
            robot.write_data_to_sim()
            sim.step(render=(step % render_every == 0))
            robot.update(physics_dt)
            if payload_controller is not None:
                payload_visual.update(
                    robot.data.body_pos_w[0, payload_controller.body_indices]
                )
            step += 1
            if camera is not None:
                camera.update(physics_dt)
            if step % steps_per_control == 0:
                if cam_pub is not None:
                    cam_pub.publish()
                node.publish(
                    robot.data.joint_pos[0].cpu().numpy(),
                    robot.data.joint_vel[0].cpu().numpy(),
                    robot.data.root_pos_w[0].cpu().numpy(),
                    robot.data.root_quat_w[0].cpu().numpy(),
                    robot.data.root_lin_vel_w[0].cpu().numpy(),
                    robot.data.root_ang_vel_w[0].cpu().numpy(),
                )
                if arms is not None:
                    node.publish_arm_status(
                        arms.pose_actual,
                        ARM_JOINTS,
                        arms.objetivo,
                        robot.data.joint_pos[0, arm_ids].cpu().numpy(),
                    )
                rclpy.spin_once(node, timeout_sec=0.0)
            continue

        if was_frozen is not False:
            # Recien soltado: la policy arranca de cero, como recien encendida.
            controller.reset()
            print("\n[robot] SOLTADO: la policy toma el control", flush=True)
            was_frozen = False
            experiment_start_step = step

        # --- decidir (cada steps_per_control pasos de fisica) ---
        if step >= settle_steps and step % steps_per_control == 0:
            observed_q = robot.data.joint_pos[0, observation_ids].cpu().numpy()
            observed_dq = robot.data.joint_vel[0, observation_ids].cpu().numpy()
            action_q = robot.data.joint_pos[0, sim_ids].cpu().numpy()
            action_dq = robot.data.joint_vel[0, sim_ids].cpu().numpy()
            quat = robot.data.root_quat_w[0].cpu().numpy()
            ang = robot.data.root_ang_vel_b[0].cpu().numpy()

            state = LocomotionState(
                joint_pos=observed_q,
                joint_vel=observed_dq,
                ang_vel=ang,
                gravity=gravity_in_body_frame(*quat),
            )
            command = node.current_command()
            if args_cli.experiment_command_x is not None:
                experiment_time = (
                    (step - experiment_start_step) * physics_dt
                    if experiment_start_step is not None
                    else 0.0
                )
                command_is_active = (
                    experiment_time >= args_cli.experiment_command_start_s
                    and (
                        args_cli.experiment_command_stop_s is None
                        or experiment_time < args_cli.experiment_command_stop_s
                    )
                )
                command = np.array(
                    [
                        args_cli.experiment_command_x if command_is_active else 0.0,
                        0.0,
                        0.0,
                    ],
                    dtype=np.float32,
                )
            objetivo = controller.compute(state, command)
            target = torch.tensor(objetivo, dtype=torch.float32,
                                  device=args_cli.device).unsqueeze(0)
            robot.set_joint_position_target(target, joint_ids=sim_ids)

            if diagnostics is not None and experiment_start_step is not None:
                diagnostics.sample(
                    (step - experiment_start_step) * physics_dt,
                    command,
                    action_q,
                    action_dq,
                    objetivo,
                    robot.data.root_pos_w[0].cpu().numpy(),
                    quat,
                    robot.data.root_lin_vel_w[0].cpu().numpy(),
                )

            # Reinicio: devolver el robot al punto de partida, de pie y quieto.
            # Sirve para repetir la mision desde cero sin relanzar el simulador
            # (que tarda un minuto en arrancar).
            if node.reset_request:
                node.reset_request = False
                robot.write_root_pose_to_sim(home_root_state[:, :7])
                robot.write_root_velocity_to_sim(torch.zeros_like(
                    home_root_state[:, 7:]))
                joint_pos = robot.data.default_joint_pos.clone()
                joint_pos[0, sim_ids] = default_angles
                robot.write_joint_state_to_sim(joint_pos,
                                               torch.zeros_like(robot.data.default_joint_vel))
                robot.reset()
                controller.reset()
                if arms is not None:
                    arms.reset()
                consume_payload_request()
                print("\n[robot] reiniciado en el punto de partida", flush=True)

            # Los brazos, en paralelo: su propio controlador, sus propias
            # articulaciones. La locomocion no se entera.
            if arms is not None:
                consume_arm_pose_request()
                arm_target = torch.tensor(arms.compute(control_dt), dtype=torch.float32,
                                          device=args_cli.device).unsqueeze(0)
                robot.set_joint_position_target(arm_target, joint_ids=arm_ids)
            consume_payload_request()

        # --- avanzar la fisica ---
        # render=False en la mayoria de los pasos: dibujar la escena cuesta
        # tanto como la fisica y no aporta nada al control. Se renderiza cada
        # render_every pasos, suficiente para mirar por el visor.
        robot.write_data_to_sim()
        sim.step(render=(step % render_every == 0))
        robot.update(physics_dt)
        if payload_controller is not None:
            payload_visual.update(
                robot.data.body_pos_w[0, payload_controller.body_indices]
            )
        step += 1

        # --- publicar estado y atender ROS (al ritmo del control) ---
        if camera is not None:
            camera.update(physics_dt)

        if step % steps_per_control == 0:
            node.publish(
                robot.data.joint_pos[0].cpu().numpy(),
                robot.data.joint_vel[0].cpu().numpy(),
                robot.data.root_pos_w[0].cpu().numpy(),
                robot.data.root_quat_w[0].cpu().numpy(),
                robot.data.root_lin_vel_b[0].cpu().numpy(),
                robot.data.root_ang_vel_b[0].cpu().numpy(),
            )
            if arms is not None:
                node.publish_arm_status(
                    arms.pose_actual,
                    ARM_JOINTS,
                    arms.objetivo,
                    robot.data.joint_pos[0, arm_ids].cpu().numpy(),
                )
            if cam_pub is not None:
                cam_pub.publish()
            rclpy.spin_once(node, timeout_sec=0.0)

        # El seguimiento es opcional porque imponer esta vista varias veces por
        # segundo impide que el operador de WebRTC haga zoom o mire otra zona.
        if args_cli.follow_viewer and step % (steps_per_control * 5) == 0:
            p = robot.data.root_pos_w[0].cpu().numpy()
            sim.set_camera_view(
                eye=(float(p[0]) - 3.0, float(p[1]) - 3.0, float(p[2]) + 1.5),
                target=(float(p[0]), float(p[1]), float(p[2])),
            )

        # --- telemetria: cuanto tarda el simulador vs el tiempo real ---
        if step % 500 == 0:
            ahora = time.perf_counter()
            rtf = (500 * physics_dt) / (ahora - t_wall)
            t_wall = ahora
            altura = float(robot.data.root_pos_w[0, 2])
            print(f"\r[robot] RTF {rtf:.2f}  altura {altura:.3f} m  "
                  f"cmd {node.current_command()}", end="", flush=True)

        if (
            args_cli.experiment_duration_s > 0
            and experiment_start_step is not None
            and (step - experiment_start_step) * physics_dt
            >= args_cli.experiment_duration_s
        ):
            print("\n[robot] duración simulada del experimento cumplida", flush=True)
            break

    if diagnostics is not None:
        diagnostics.close()
    node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
