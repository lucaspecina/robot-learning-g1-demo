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


class LocomotionState:
    """Lo que el controlador necesita saber del robot en cada ciclo."""

    def __init__(self, joint_pos, joint_vel, ang_vel, gravity):
        self.joint_pos = joint_pos    # (12,) angulos actuales [rad]
        self.joint_vel = joint_vel    # (12,) velocidades [rad/s]
        self.ang_vel = ang_vel        # (3,) velocidad angular del cuerpo [rad/s]
        self.gravity = gravity        # (3,) direccion de la gravedad vista desde el cuerpo


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
        """Deja la policy como recien cargada.

        Ojo con la memoria: esta policy es RECURRENTE — tiene una red LSTM
        interna con estado propio (hidden_state y cell_state, 64 numeros cada
        uno) que recuerda lo que vino pasando. Si no se borra, cada corrida
        arranca contaminada con el estado de la anterior (incluidas las
        caidas), las mediciones dejan de ser independientes y la policy puede
        partir de un estado interno que nunca vio al entrenar.
        """
        self.last_action = np.zeros(self.n, dtype=np.float32)
        self.phase_time = 0.0
        self._reset_memory()

    def _reset_memory(self):
        """Pone en cero la memoria recurrente de la policy, si la tiene."""
        borrados = 0
        for nombre, buf in self.policy.named_buffers():
            if any(k in nombre for k in ("hidden_state", "cell_state")):
                buf.zero_()
                borrados += 1
        self.memoria_borrada = borrados

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
