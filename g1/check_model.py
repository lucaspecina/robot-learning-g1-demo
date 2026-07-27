#!/usr/bin/env python3
"""Compara el robot del simulador con el que espera la policy.

La policy pre-entrenada de Unitree fue entrenada sobre un modelo concreto del
G1 (12 articulaciones, solo piernas). Si el robot que cargamos en Isaac tiene
otra masa u otra distribucion, la policy esta controlando un robot que no
conoce — y la mejor policy del mundo se cae si el cuerpo no es el que aprendio.

Esto imprime los numeros para poder compararlos y decidir con datos.

Uso:
    ~/go2-lab/IsaacLab/isaaclab.sh -p check_model.py --headless
"""
import sys

from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli, _extra = parser.parse_known_args()
sys.argv = [sys.argv[0]] + [a for a in _extra if a.startswith("--/")]
app = AppLauncher(args_cli).app

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab_assets import G1_CFG

sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.005, device=args_cli.device))
robot = Articulation(G1_CFG.replace(prim_path="/World/G1"))
sim.reset()

masas = robot.root_physx_view.get_masses()[0]
print("\n===== EL G1 DE ISAACLAB =====")
print(f"articulaciones: {robot.num_joints}")
print(f"cuerpos: {robot.num_bodies}")
print(f"masa total: {float(masas.sum()):.1f} kg")
print(f"altura inicial: {float(robot.data.default_root_state[0, 2]):.3f} m")

print("\n--- masa por cuerpo ---")
for nombre, m in zip(robot.body_names, masas.tolist()):
    print(f"  {nombre:32s} {m:6.2f} kg")

print("\n--- articulaciones (nombre: pose inicial) ---")
for nombre, q in zip(robot.joint_names, robot.data.default_joint_pos[0].tolist()):
    print(f"  {nombre:32s} {q:+.3f} rad")

app.close()
