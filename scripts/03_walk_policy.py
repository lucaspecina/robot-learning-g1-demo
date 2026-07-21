#!/usr/bin/env python3
"""Camina con una policy de RL pre-entrenada, comandada por teclado (sim o real).

Este script ocupa el "asiento del medio" de la pirámide: carga una red
pre-entrenada (la policy robot_lab de rl_sar, entrenada en IsaacLab) y corre
el loop de deployment estándar a 50 Hz:

    rt/lowstate → observación (45 números) → forward pass → 12 objetivos → rt/lowcmd

Vos ocupás el asiento de navegación con el teclado (comandos de velocidad):

    w/s: vx ±0.1 m/s   a/d: vy ±0.1 m/s   q/e: vyaw ±0.1 rad/s
    espacio: comando (0,0,0)   x: terminar (deja el robot en amortiguación)

Secuencia al arrancar: rampa interpolada hasta la pose nominal (igual que
01_stand, con ganancias rígidas kp=80) y recién ahí engancha la policy (con
las ganancias blandas del entrenamiento, kp=20/kd=0.5 — el trote quiere patas
elásticas).

La observación, en el orden exacto en que la policy fue entrenada (config de
rl_sar/policy/go2/robot_lab/config.yaml):
    [ velocidad angular del cuerpo * 0.25        (3)  ← IMU giróscopo
      gravedad proyectada en el cuerpo           (3)  ← del quaternion de la IMU
      comando (vx, vy, vyaw)                     (3)  ← tu teclado
      (q - pose_nominal) * 1.0                  (12)
      dq * 0.05                                 (12)
      acción del paso anterior                  (12) ]

Uso:
    python 03_walk_policy.py              # sim (arrancar antes scripts/sim.sh)
    python 03_walk_policy.py --real eth0  # robot real (¡ARNÉS! y sport mode se libera solo)
"""

import argparse
import sys
import termios
import threading
import time
import tty
from pathlib import Path

import numpy as np
import torch

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# --- Parámetros de la policy robot_lab (rl_sar/policy/go2/robot_lab/config.yaml) ---
POLICY_PATH = (
    Path(__file__).resolve().parent.parent
    / "external" / "rl_sar" / "policy" / "go2" / "robot_lab" / "policy.pt"
)
# Pose nominal [rad], orden lowstate: FR(hip,thigh,calf), FL, RR, RL
DEFAULT_POSE = np.array([0.0, 0.8, -1.5] * 4)
ACTION_SCALE = np.array([0.125, 0.25, 0.25] * 4)  # hip más chico: menos autoridad lateral
ANG_VEL_SCALE = 0.25
DOF_VEL_SCALE = 0.05
CLIP_OBS = 100.0
RL_KP, RL_KD = 20.0, 0.5      # ganancias con las que se entrenó (blandas, elásticas)
RAMP_KP, RAMP_KD = 80.0, 3.0  # ganancias rígidas solo para la rampa inicial
POLICY_DT = 0.02              # 50 Hz, el ritmo con el que se entrenó
CMD_STEP = 0.1
CMD_LIMITS = np.array([1.0, 0.5, 1.0])  # |vx|, |vy|, |vyaw| máximos por seguridad


def gravity_in_body_frame(quat) -> np.ndarray:
    """Proyecta la gravedad al marco del cuerpo desde el quaternion (w,x,y,z).

    Robot derecho → (0, 0, -1); inclinado, la componente horizontal crece.
    Es la representación de inclinación que la policy vio en entrenamiento.
    """
    w, x, y, z = quat
    return np.array([
        2 * (w * y - x * z),
        -2 * (z * y + w * x),
        1 - 2 * (w * w + z * z),
    ])


class KeyboardTeleop:
    """Lee teclas del terminal (sin enter) y mantiene el comando de velocidad."""

    def __init__(self):
        self.cmd = np.zeros(3)  # vx, vy, vyaw
        self.quit = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)  # modo "una tecla a la vez", sin esperar enter
            while not self.quit:
                ch = sys.stdin.read(1).lower()
                if ch == "w":
                    self.cmd[0] += CMD_STEP
                elif ch == "s":
                    self.cmd[0] -= CMD_STEP
                elif ch == "a":
                    self.cmd[1] += CMD_STEP
                elif ch == "d":
                    self.cmd[1] -= CMD_STEP
                elif ch == "q":
                    self.cmd[2] += CMD_STEP
                elif ch == "e":
                    self.cmd[2] -= CMD_STEP
                elif ch == " ":
                    self.cmd[:] = 0.0
                elif ch == "x":
                    self.quit = True
                self.cmd = np.clip(self.cmd, -CMD_LIMITS, CMD_LIMITS)
                print(f"\rcmd: vx={self.cmd[0]:+.1f}  vy={self.cmd[1]:+.1f}  "
                      f"vyaw={self.cmd[2]:+.1f}   ", end="", flush=True)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--real", metavar="IFACE", default=None,
                    help="interfaz hacia el robot real (ej: eth0). Sin esto: sim.")
    ap.add_argument("--device", default="cpu", choices=["cpu", "cuda"],
                    help="dónde correr la red (cpu alcanza de sobra)")
    ap.add_argument("--cmd", nargs=3, type=float, metavar=("VX", "VY", "VYAW"),
                    default=None,
                    help="comando fijo en vez de teclado (ej: --cmd 0.4 0 0)")
    ap.add_argument("--dur", type=float, default=0.0,
                    help="duración en segundos con --cmd (0 = infinito)")
    args = ap.parse_args()

    # --- Cargar la policy (TorchScript exportada por rsl_rl) ---
    policy = torch.jit.load(str(POLICY_PATH), map_location=args.device)
    policy.eval()
    print(f"Policy cargada: {POLICY_PATH.name} en {args.device}")

    if args.real:
        ChannelFactoryInitialize(0, args.real)
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient,
        )
        msc = MotionSwitcherClient()
        msc.SetTimeout(5.0)
        msc.Init()
        while True:
            code, result = msc.CheckMode()
            if not result or not result.get("name"):
                break
            print(f"Liberando modo activo: {result['name']}...")
            msc.ReleaseMode()
            time.sleep(1.0)
    else:
        ChannelFactoryInitialize(1, "lo")

    # --- Estado: siempre el último lowstate ---
    latest = {"msg": None}
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(lambda m: latest.update(msg=m), 10)

    print("Esperando rt/lowstate...")
    while latest["msg"] is None:
        time.sleep(0.1)

    # --- Publisher de lowcmd (idéntico a 01_stand) ---
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    crc = CRC()
    cmd_msg = unitree_go_msg_dds__LowCmd_()
    cmd_msg.head[0], cmd_msg.head[1] = 0xFE, 0xEF
    cmd_msg.level_flag = 0xFF
    for i in range(20):
        cmd_msg.motor_cmd[i].mode = 0x01

    def send_cmd(q_des: np.ndarray, kp: float, kd: float):
        for i in range(12):
            cmd_msg.motor_cmd[i].q = float(q_des[i])
            cmd_msg.motor_cmd[i].dq = 0.0
            cmd_msg.motor_cmd[i].tau = 0.0
            cmd_msg.motor_cmd[i].kp = kp
            cmd_msg.motor_cmd[i].kd = kd
        cmd_msg.crc = crc.Crc(cmd_msg)
        pub.Write(cmd_msg)

    def read_q_dq():
        msg = latest["msg"]
        q = np.array([msg.motor_state[i].q for i in range(12)])
        dq = np.array([msg.motor_state[i].dq for i in range(12)])
        return msg, q, dq

    # --- Fase 1: rampa a la pose nominal (el stand de siempre) ---
    _, q_start, _ = read_q_dq()
    print("Rampa a la pose nominal...")
    t, ramp_time = 0.0, 3.0
    while t < ramp_time:
        s = np.tanh(t / 1.2)
        send_cmd((1 - s) * q_start + s * DEFAULT_POSE, RAMP_KP, RAMP_KD)
        t += 0.002
        time.sleep(0.002)

    # --- Fase 2: la policy toma el control a 50 Hz ---
    if args.cmd is not None:
        # Modo comando fijo (sin teclado): útil para tests y corridas headless
        class FixedCmd:
            cmd = np.clip(np.array(args.cmd), -CMD_LIMITS, CMD_LIMITS)
            quit = False
        teleop = FixedCmd()
        print(f"\nPolicy activa con comando fijo {teleop.cmd}"
              + (f" durante {args.dur:.0f} s" if args.dur > 0 else ""))
    else:
        print("\nPolicy activa. Teclas: w/s=vx  a/d=vy  q/e=vyaw  espacio=frenar  x=salir")
        teleop = KeyboardTeleop()
    last_action = np.zeros(12)
    t_end = time.time() + args.dur if args.dur > 0 else None

    try:
        while not teleop.quit and (t_end is None or time.time() < t_end):
            step_start = time.perf_counter()

            msg, q, dq = read_q_dq()
            ang_vel = np.array(msg.imu_state.gyroscope)
            gravity = gravity_in_body_frame(msg.imu_state.quaternion)

            # La observación, en el orden y con las escalas del entrenamiento
            obs = np.concatenate([
                ang_vel * ANG_VEL_SCALE,
                gravity,
                teleop.cmd,
                (q - DEFAULT_POSE),
                dq * DOF_VEL_SCALE,
                last_action,
            ]).astype(np.float32)
            obs = np.clip(obs, -CLIP_OBS, CLIP_OBS)

            with torch.no_grad():
                action = policy(torch.from_numpy(obs).unsqueeze(0).to(args.device))
            last_action = action.squeeze(0).cpu().numpy()

            q_des = DEFAULT_POSE + ACTION_SCALE * last_action
            send_cmd(q_des, RL_KP, RL_KD)

            time_left = POLICY_DT - (time.perf_counter() - step_start)
            if time_left > 0:
                time.sleep(time_left)
    except KeyboardInterrupt:
        pass

    # --- Salida segura: amortiguación (kp=0) ---
    print("\nDejando el robot en amortiguación...")
    _, q_now, _ = read_q_dq()
    for _ in range(500):
        send_cmd(q_now, 0.0, 2.0)
        time.sleep(0.002)
    print("Listo.")


if __name__ == "__main__":
    main()
