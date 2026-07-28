#!/usr/bin/env python3
"""Mide la deriva de la policy en MuJoCo — el experimento que separa culpas.

La misma policy, con comando cero, en el simulador donde Unitree la valida.

  Si en MuJoCo NO deriva -> el problema es nuestra configuracion de Isaac.
  Si en MuJoCo TAMBIEN deriva -> es la policy, y en el robot real va a pasar
                                 lo mismo. Deja de ser un bug nuestro y pasa a
                                 ser una caracteristica a compensar por arriba.

Es el codigo del despliegue oficial de Unitree (deploy_mujoco.py) reducido a lo
minimo: sin visor, con comando cero, midiendo cuanto se desplaza el pelvis.

Uso (inside del contenedor jetson):
    python3 drift_in_mujoco.py [segundos]
"""
import sys

import mujoco
import numpy as np
import torch
import yaml

BASE = "/workspace/unitree_rl_gym"
XML = f"{BASE}/resources/robots/g1_description/scene.xml"
POLICY = f"{BASE}/deploy/pre_train/g1/motion.pt"
CFG = f"{BASE}/deploy/deploy_mujoco/configs/g1.yaml"
DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0


def gravity_orientation(quaternion):
    qw, qx, qy, qz = quaternion
    return np.array([
        2 * (-qz * qx + qw * qy),
        -2 * (qz * qy + qw * qx),
        1 - 2 * (qw * qw + qz * qz),
    ])


def pd_control(target_q, q, kp, target_dq, dq, kd):
    return (target_q - q) * kp + (target_dq - dq) * kd


with open(CFG, "r") as f:
    c = yaml.safe_load(f)

kps = np.array(c["kps"], dtype=np.float32)
kds = np.array(c["kds"], dtype=np.float32)
default_angles = np.array(c["default_angles"], dtype=np.float32)
cmd_scale = np.array(c["cmd_scale"], dtype=np.float32)
num_actions, num_obs = c["num_actions"], c["num_obs"]
sim_dt, decimation = c["simulation_dt"], c["control_decimation"]

# EL PUNTO DEL EXPERIMENTO: comando cero. El robot deberia quedarse quieto.
cmd = np.zeros(3, dtype=np.float32)

action = np.zeros(num_actions, dtype=np.float32)
target_dof_pos = default_angles.copy()
obs = np.zeros(num_obs, dtype=np.float32)

m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)
m.opt.timestep = sim_dt
policy = torch.jit.load(POLICY)

start = np.array([d.qpos[0], d.qpos[1]])
steps = int(DURATION_S / sim_dt)
counter = 0
samples = []

for i in range(steps):
    tau = pd_control(target_dof_pos, d.qpos[7:], kps, np.zeros_like(kds), d.qvel[6:], kds)
    d.ctrl[:] = tau
    mujoco.mj_step(m, d)
    counter += 1

    if counter % decimation == 0:
        qj = (d.qpos[7:] - default_angles) * c["dof_pos_scale"]
        dqj = d.qvel[6:] * c["dof_vel_scale"]
        omega = d.qvel[3:6] * c["ang_vel_scale"]
        grav = gravity_orientation(d.qpos[3:7])

        period = 0.8
        phase = (counter * sim_dt) % period / period

        obs[:3] = omega
        obs[3:6] = grav
        obs[6:9] = cmd * cmd_scale
        obs[9:9 + num_actions] = qj
        obs[9 + num_actions:9 + 2 * num_actions] = dqj
        obs[9 + 2 * num_actions:9 + 3 * num_actions] = action
        obs[9 + 3 * num_actions:9 + 3 * num_actions + 2] = [np.sin(2 * np.pi * phase),
                                                            np.cos(2 * np.pi * phase)]
        action = policy(torch.from_numpy(obs).unsqueeze(0)).detach().numpy().squeeze()
        target_dof_pos = action * c["action_scale"] + default_angles

    if counter % int(5.0 / sim_dt) == 0:   # cada 5 s simulados
        t = counter * sim_dt
        pos = np.array([d.qpos[0], d.qpos[1]])
        samples.append((t, pos - start, d.qpos[2]))

print(f"\n=== DERIVA EN MUJOCO (comando cero, {DURATION_S:.0f} s simulados) ===")
for t, delta, height in samples:
    print(f"  t={t:5.1f}s   desplazamiento {np.linalg.norm(delta):.3f} m "
          f"(x {delta[0]:+.3f}, y {delta[1]:+.3f})   height {height:.3f} m")

final = np.array([d.qpos[0], d.qpos[1]]) - start
total = np.linalg.norm(final)
print(f"\n  TOTAL: {total:.3f} m en {DURATION_S:.0f} s = {total/DURATION_S*100:.1f} cm/s")
print(f"  height final: {d.qpos[2]:.3f} m  ({'de pie' if d.qpos[2] > 0.5 else 'CAIDO'})")
