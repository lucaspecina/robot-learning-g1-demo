#!/usr/bin/env python3
"""Controladores de locomocion del G1: convierten intencion en articulaciones.

Cada controlador implementa la misma interfaz, asi que son intercambiables:

    controller.reset()
    targets = controller.compute(state, command)   # 12 angulos objetivo

Corren DENTRO del proceso del robot, no como nodo aparte. Motivo: el lazo de
control de un bipedo necesita que la decision y el paso de fisica esten
sincronizados; separarlos en dos procesos asincronos abre el lazo en el tiempo
y el robot se cae. En el robot real pasa lo mismo — la policy vive en la
computadora de control o en la Jetson, pegada al hardware por un enlace
dedicado, nunca del otro lado de una red incierta.

La modularidad del sistema esta un piso mas arriba, en /cmd_vel: quien manda
velocidades no sabe ni le importa cual de estos controladores esta abajo.
"""
import math

import numpy as np
import torch
import yaml


class LocomotionState:
    """Lo que el controlador necesita saber del robot en cada ciclo."""

    def __init__(self, joint_pos, joint_vel, ang_vel, gravity):
        self.joint_pos = joint_pos    # (12,) angulos actuales [rad]
        self.joint_vel = joint_vel    # (12,) velocidades [rad/s]
        self.ang_vel = ang_vel        # (3,) velocidad angular del cuerpo [rad/s]
        self.gravity = gravity        # (3,) direccion de la gravedad vista desde el cuerpo


class WbcAgileLocomotion:
    """Locomoción desplegable de NVIDIA AGILE para el G1 completo.

    El descriptor YAML viaja junto con la policy y define el orden exacto de
    articulaciones, desplazamientos y escalas. Leer ese contrato evita copiar
    números a mano y separarnos silenciosamente de la versión oficial.
    """

    def __init__(
        self,
        policy_path,
        descriptor_path,
        pelvis_height=0.72,
        device="cpu",
    ):
        with open(descriptor_path, "r", encoding="utf-8") as descriptor_file:
            descriptor = yaml.safe_load(descriptor_file)

        observations = {
            term["name"]: term for term in descriptor["observations"]["policy"]
        }
        joint_position = observations["joint_pos_rel"]
        joint_velocity = observations["joint_vel_rel"]
        action = descriptor["actions"][0]
        articulation = descriptor["articulations"]["robot"]

        self.observation_joint_names = list(joint_position["joint_names"])
        if self.observation_joint_names != list(joint_velocity["joint_names"]):
            raise ValueError(
                "el descriptor de AGILE usa órdenes distintas para posición y velocidad"
            )

        self.action_joint_names = list(action["joint_names"])
        self.default_joint_positions = np.asarray(
            joint_position["joint_pos_offsets"], dtype=np.float32
        )
        self.joint_velocity_scale = float(
            joint_velocity["overloads"].get("scale") or 1.0
        )
        self.action_offset = np.asarray(action["offset"], dtype=np.float32)
        self.action_scale = np.asarray(action["scale"], dtype=np.float32).reshape(-1)
        self.action_clip = np.asarray(action["clip"], dtype=np.float32)
        self.pelvis_height = float(pelvis_height)

        all_joint_names = list(articulation["joint_names"])
        all_stiffness = np.asarray(
            articulation["default_joint_stiffness"], dtype=np.float32
        )
        all_damping = np.asarray(
            articulation["default_joint_damping"], dtype=np.float32
        )
        action_indices = [all_joint_names.index(name) for name in self.action_joint_names]
        self.action_stiffness = all_stiffness[action_indices]
        self.action_damping = all_damping[action_indices]

        self.policy = torch.jit.load(policy_path, map_location=device)
        self.policy.eval()
        self.device = device
        self.reset()

    def reset(self):
        self.last_action = np.zeros(len(self.action_joint_names), dtype=np.float32)
        if hasattr(self.policy, "reset_flat"):
            self.policy.reset_flat()

    def compute(self, state: LocomotionState, command) -> np.ndarray:
        """Devuelve objetivos para las 12 piernas usando el contrato oficial."""
        command = np.asarray(command, dtype=np.float32)
        if command.shape != (3,):
            raise ValueError(f"AGILE esperaba 3 órdenes de movimiento y recibió {command.shape}")

        observation = np.concatenate(
            [
                command,
                np.asarray([self.pelvis_height], dtype=np.float32),
                np.asarray(state.ang_vel, dtype=np.float32),
                np.asarray(state.gravity, dtype=np.float32),
                np.asarray(state.joint_pos, dtype=np.float32)
                - self.default_joint_positions,
                np.asarray(state.joint_vel, dtype=np.float32)
                * self.joint_velocity_scale,
                self.last_action,
            ]
        )
        if observation.shape != (80,):
            raise ValueError(
                f"AGILE esperaba 80 mediciones y recibió {observation.shape[0]}"
            )

        with torch.no_grad():
            action = (
                self.policy(
                    torch.from_numpy(observation).to(self.device)
                )
                .cpu()
                .numpy()
                .squeeze()
                .astype(np.float32)
            )

        # AGILE usa la acción cruda anterior como parte de la próxima entrada;
        # limitarla aquí reproduce el procesamiento de su entorno oficial.
        clip_min = self.action_clip[:, 0]
        clip_max = self.action_clip[:, 1]
        self.last_action = np.clip(action, clip_min, clip_max)
        return self.action_offset + self.last_action * self.action_scale


class RLPolicyLocomotion:
    """Locomocion con la policy pre-entrenada de unitree_rl_gym.

    Controla las 12 articulaciones de las piernas. Los brazos quedan fuera de
    su alcance: son responsabilidad de otro controlador.
    """

    def __init__(self, cfg, policy_path, device="cpu"):
        self.cfg = cfg
        self.n = cfg["num_actions"]
        self.num_obs = cfg["num_obs"]
        self.default_angles = np.array(cfg["default_angles"], dtype=np.float32)
        self.cmd_scale = np.array(cfg["cmd_scale"], dtype=np.float32)
        self.action_scale = cfg["action_scale"]
        self.ang_vel_scale = cfg["ang_vel_scale"]
        self.dof_pos_scale = cfg["dof_pos_scale"]
        self.dof_vel_scale = cfg["dof_vel_scale"]
        self.control_dt = cfg["control_dt"]
        self.gait_period = cfg["gait_period"]

        self.policy = torch.jit.load(policy_path, map_location=device)
        self.policy.eval()
        self.device = device
        self.reset()

    def reset(self):
        self.last_action = np.zeros(self.n, dtype=np.float32)
        self.phase_time = 0.0

    def compute(self, state: LocomotionState, command) -> np.ndarray:
        """Devuelve los 12 angulos objetivo para este ciclo."""
        # La fase avanza con el tiempo SIMULADO, no con el reloj de pared: asi
        # el paso del robot es correcto aunque el simulador corra lento.
        self.phase_time += self.control_dt
        phase = (self.phase_time % self.gait_period) / self.gait_period

        obs = np.zeros(self.num_obs, dtype=np.float32)
        obs[0:3] = state.ang_vel * self.ang_vel_scale
        obs[3:6] = state.gravity
        obs[6:9] = command * self.cmd_scale
        obs[9:9 + self.n] = (state.joint_pos - self.default_angles) * self.dof_pos_scale
        obs[9 + self.n:9 + 2 * self.n] = state.joint_vel * self.dof_vel_scale
        obs[9 + 2 * self.n:9 + 3 * self.n] = self.last_action
        obs[9 + 3 * self.n:9 + 3 * self.n + 2] = [math.sin(2 * math.pi * phase),
                                                  math.cos(2 * math.pi * phase)]

        with torch.no_grad():
            action = self.policy(torch.from_numpy(obs).unsqueeze(0)).cpu().numpy().squeeze()
        self.last_action = action.astype(np.float32)

        # La policy emite desviaciones respecto de la pose nominal.
        return self.default_angles + self.last_action * self.action_scale


class StandStillLocomotion:
    """Controlador trivial: sostiene la pose nominal, ignora el comando.

    Sirve para dos cosas: verificar que el robot se sostiene sin policy (aisla
    problemas del simulador de problemas de la policy) y como comportamiento
    seguro de reserva.
    """

    def __init__(self, cfg):
        self.default_angles = np.array(cfg["default_angles"], dtype=np.float32)

    def reset(self):
        pass

    def compute(self, state: LocomotionState, command) -> np.ndarray:
        return self.default_angles


def gravity_in_body_frame(qw, qx, qy, qz):
    """Direccion de la gravedad vista desde el cuerpo (señal de inclinacion).

    Con el robot derecho da aproximadamente (0, 0, -1); al inclinarse, las dos
    primeras componentes crecen.
    """
    return np.array([
        2.0 * (-qz * qx + qw * qy),
        -2.0 * (qz * qy + qw * qx),
        1.0 - 2.0 * (qw * qw + qz * qz),
    ], dtype=np.float32)
