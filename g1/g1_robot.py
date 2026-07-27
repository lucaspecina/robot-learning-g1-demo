#!/usr/bin/env python3
"""El robot G1 simulado: fisica + locomocion, con el lazo de control cerrado.

Este proceso ocupa el lugar del robot completo — cuerpo, computadora de control
y locomocion — igual que el robot real es una unidad. Su interfaz con el resto
del sistema es la misma que tendra el robot fisico:

  recibe:  /cmd_vel          (geometry_msgs/Twist — a que velocidad ir)
  publica: /g1/odom          (nav_msgs/Odometry — donde esta el cuerpo)
           /g1/joint_states  (sensor_msgs/JointState — estado de las piernas)

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
    ~/go2-lab/IsaacLab/isaaclab.sh -p g1_robot.py --headless \
        --policy ~/go2-lab/unitree_rl_gym/deploy/pre_train/g1/motion.pt
    # sin --policy arranca en modo "solo pararse" (util para diagnostico)
"""
import argparse
import os
import sys
import time

import yaml

# --- 1. Arrancar Isaac ANTES de importar nada de omni/isaaclab ---
from isaaclab.app import AppLauncher

_here = os.path.dirname(os.path.abspath(__file__))
parser = argparse.ArgumentParser(description="Robot G1 simulado (fisica + locomocion)")
parser.add_argument("--config", default=os.path.join(_here, "config", "g1_locomotion.yaml"))
parser.add_argument("--policy", default=None, help="motion.pt; si falta, modo solo-pararse")
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
parser.add_argument("--render_hz", type=float, default=20.0,
                    help="cuadros por segundo a dibujar (mas bajo = simulacion mas rapida)")
AppLauncher.add_app_launcher_args(parser)
args_cli, _extra = parser.parse_known_args()
sys.argv = [sys.argv[0]] + [a for a in _extra if a.startswith("--/")]

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- 2. Ahora si, el resto ---
import numpy as np
import torch

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String as RosString

sys.path.insert(0, _here)
from arm_control import ARM_JOINTS, POSES, PoseArmController  # noqa: E402
from g1_asset import make_g1_cfg  # noqa: E402
from locomotion import (  # noqa: E402
    LocomotionState,
    RLPolicyLocomotion,
    StandStillLocomotion,
    gravity_in_body_frame,
)


class G1RobotNode(Node):
    """La cara ROS 2 del robot: recibe intencion, publica estado."""

    def __init__(self, cfg):
        super().__init__("g1_robot")
        self.joint_names = cfg["joint_names"]
        self.max_lin = cfg["max_lin_vel"]
        self.max_ang = cfg["max_ang_vel"]
        self.cmd_timeout = cfg["cmd_timeout"]

        self.command = np.zeros(3, dtype=np.float32)
        self.last_cmd_time = 0.0

        self.arm_pose_request = None   # lo lee el lazo principal

        self.pub_state = self.create_publisher(JointState, "/g1/joint_states", 10)
        self.pub_odom = self.create_publisher(Odometry, "/g1/odom", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(RosString, "/g1/arm_pose", self.on_arm_pose, 10)

    def on_arm_pose(self, msg: RosString):
        """Pide una pose de brazos por nombre: reposo | listo | transporte."""
        self.arm_pose_request = msg.data.strip()

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


def main():
    with open(args_cli.config, "r") as f:
        cfg = yaml.safe_load(f)

    # 500 Hz de fisica, igual que el despliegue oficial de Unitree. Con paso
    # mas grueso los contactos pie-piso se integran mal y la policy se
    # desestabiliza — la misma leccion que aprendimos con el Go2, y en un
    # bipedo pesa mucho mas.
    physics_dt = 0.002
    control_dt = cfg["control_dt"]          # 0.02 s = 50 Hz de decision
    steps_per_control = max(1, round(control_dt / physics_dt))
    render_every = max(1, round((1.0 / args_cli.render_hz) / physics_dt)) if args_cli.render_hz > 0 else 10**9

    # --- escena ---
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=physics_dt, device=args_cli.device))
    sim.set_camera_view(eye=(2.5, 2.5, 1.8), target=(0.0, 0.0, 0.8))
    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    light = sim_utils.DomeLightCfg(intensity=2000.0)
    light.func("/World/light", light)

    robot = Articulation(make_g1_cfg(cfg, model=args_cli.model))
    sim.reset()
    print(f"[robot] articulaciones del modelo: {robot.num_joints} "
          f"({', '.join(robot.joint_names[:3])}...)", flush=True)

    # --- indices de las 12 articulaciones que controla la locomocion ---
    sim_ids = [robot.joint_names.index(n) for n in cfg["joint_names"]]
    default_angles = torch.tensor(cfg["default_angles"], dtype=torch.float32, device=args_cli.device)

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
    robot.write_root_pose_to_sim(root_state[:, :7])
    robot.write_root_velocity_to_sim(root_state[:, 7:])

    joint_pos = robot.data.default_joint_pos.clone()
    joint_pos[0, sim_ids] = default_angles
    robot.write_joint_state_to_sim(joint_pos, robot.data.default_joint_vel.clone())
    robot.reset()
    print(f"[robot] estado inicial: altura {float(root_state[0, 2]):.3f} m, "
          f"piernas en la pose nominal de la policy", flush=True)

    # Ganancias de Unitree para las piernas (el "PD" que en el robot real vive
    # en la electronica de cada motor).
    kp = torch.tensor(cfg["kps"], dtype=torch.float32, device=args_cli.device).unsqueeze(0)
    kd = torch.tensor(cfg["kds"], dtype=torch.float32, device=args_cli.device).unsqueeze(0)
    robot.write_joint_stiffness_to_sim(kp, joint_ids=sim_ids)
    robot.write_joint_damping_to_sim(kd, joint_ids=sim_ids)

    # --- controlador de locomocion (intercambiable) ---
    if args_cli.policy:
        controller = RLPolicyLocomotion(cfg, args_cli.policy, device="cpu")
        print(f"[robot] locomocion: policy RL ({os.path.basename(args_cli.policy)})", flush=True)
    else:
        controller = StandStillLocomotion(cfg)
        print("[robot] locomocion: solo pararse (sin policy)", flush=True)

    # --- control de brazos (solo en el cuerpo que los tiene) ---
    tiene_brazos = args_cli.model != "12dof"
    if tiene_brazos:
        arm_ids = [robot.joint_names.index(n) for n in ARM_JOINTS]
        arms = PoseArmController()
        print(f"[robot] brazos: {len(ARM_JOINTS)} articulaciones, pose inicial reposo", flush=True)
    else:
        arm_ids, arms = None, None

    # --- carga en las manos (para medir cuanto tolera la locomocion) ---
    if args_cli.payload_kg > 0 and tiene_brazos:
        manos = [i for i, n in enumerate(robot.body_names) if "rubber_hand" in n]
        if manos:
            masas = robot.root_physx_view.get_masses()
            extra = args_cli.payload_kg / len(manos)
            for i in manos:
                masas[0, i] += extra
            robot.root_physx_view.set_masses(masas, torch.arange(1))
            print(f"[robot] carga: {args_cli.payload_kg:.2f} kg repartidos en "
                  f"{len(manos)} manos ({extra:.2f} kg c/u)", flush=True)

    rclpy.init()
    node = G1RobotNode(cfg)

    # El robot arranca sosteniendo la pose nominal: la policy necesita partir
    # de una postura que conozca, no de una arbitraria.
    target = default_angles.clone().unsqueeze(0)
    robot.set_joint_position_target(target, joint_ids=sim_ids)

    settle_steps = int(args_cli.settle_s / physics_dt)
    step = 0
    t_wall = time.perf_counter()
    print(f"[robot] corriendo: fisica {1/physics_dt:.0f} Hz, control {1/control_dt:.0f} Hz, "
          f"asentando {args_cli.settle_s:.1f} s", flush=True)

    while simulation_app.is_running():
        # --- decidir (cada steps_per_control pasos de fisica) ---
        if step >= settle_steps and step % steps_per_control == 0:
            q = robot.data.joint_pos[0, sim_ids].cpu().numpy()
            dq = robot.data.joint_vel[0, sim_ids].cpu().numpy()
            quat = robot.data.root_quat_w[0].cpu().numpy()
            ang = robot.data.root_ang_vel_b[0].cpu().numpy()

            state = LocomotionState(
                joint_pos=q, joint_vel=dq, ang_vel=ang,
                gravity=gravity_in_body_frame(*quat),
            )
            objetivo = controller.compute(state, node.current_command())
            target = torch.tensor(objetivo, dtype=torch.float32,
                                  device=args_cli.device).unsqueeze(0)
            robot.set_joint_position_target(target, joint_ids=sim_ids)

            # Los brazos, en paralelo: su propio controlador, sus propias
            # articulaciones. La locomocion no se entera.
            if arms is not None:
                if node.arm_pose_request is not None:
                    try:
                        arms.set_pose(node.arm_pose_request)
                        print(f"\n[robot] brazos -> pose '{node.arm_pose_request}'", flush=True)
                    except ValueError as e:
                        print(f"\n[robot] brazos: {e}", flush=True)
                    node.arm_pose_request = None
                arm_target = torch.tensor(arms.compute(control_dt), dtype=torch.float32,
                                          device=args_cli.device).unsqueeze(0)
                robot.set_joint_position_target(arm_target, joint_ids=arm_ids)

        # --- avanzar la fisica ---
        # render=False en la mayoria de los pasos: dibujar la escena cuesta
        # tanto como la fisica y no aporta nada al control. Se renderiza cada
        # render_every pasos, suficiente para mirar por el visor.
        robot.write_data_to_sim()
        sim.step(render=(step % render_every == 0))
        robot.update(physics_dt)
        step += 1

        # --- publicar estado y atender ROS (al ritmo del control) ---
        if step % steps_per_control == 0:
            node.publish(
                robot.data.joint_pos[0, sim_ids].cpu().numpy(),
                robot.data.joint_vel[0, sim_ids].cpu().numpy(),
                robot.data.root_pos_w[0].cpu().numpy(),
                robot.data.root_quat_w[0].cpu().numpy(),
                robot.data.root_lin_vel_b[0].cpu().numpy(),
                robot.data.root_ang_vel_b[0].cpu().numpy(),
            )
            rclpy.spin_once(node, timeout_sec=0.0)

        # --- telemetria: cuanto tarda el simulador vs el tiempo real ---
        if step % 500 == 0:
            ahora = time.perf_counter()
            rtf = (500 * physics_dt) / (ahora - t_wall)
            t_wall = ahora
            altura = float(robot.data.root_pos_w[0, 2])
            print(f"\r[robot] RTF {rtf:.2f}  altura {altura:.3f} m  "
                  f"cmd {node.current_command()}", end="", flush=True)

    node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
