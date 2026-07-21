#!/usr/bin/env python3
"""Camina con una policy de RL pre-entrenada, comandada por teclado (sim o real).

Este script ocupa el "asiento del medio" de la pirámide: carga una red
pre-entrenada y corre el loop de deployment estándar:

    rt/lowstate → observación → forward pass → 12 objetivos → rt/lowcmd

Vos ocupás el asiento de navegación con el teclado (comandos de velocidad):

    w/s: vx ±0.1 m/s   a/d: vy ±0.1 m/s   q/e: vyaw ±0.1 rad/s
    espacio: comando (0,0,0)   x: terminar (deja el robot en amortiguación)

Hay dos policies disponibles (--policy), ambas de rl_sar, con interfaces
declaradas en external/rl_sar/policy/go2/*/config.yaml:

  himloco   (default) HIMLoco (paper 2023, muy probada). Obs de 45 con
            HISTORIA de 6 pasos (input 270, más nuevo primero). Orden de
            articulaciones del entrenamiento: FL,FR,RL,RR (≠ SDK).
  robot_lab Entrenada con IsaacLab (fan-ziqi/robot_lab). Obs de 45 sin
            historia. Orden de articulaciones = SDK (FR,FL,RR,RL).

Detalles que importan (aprendidos debugueando):
  - El "joint mapping": la red fue entrenada con SUS índices de articulación;
    hay que traducir al leer el estado y des-traducir al mandar objetivos.
  - Timing con agenda absoluta (deadline += dt), no sleep(resto): en WSL el
    sleep se pasa ~8% y la policy corría a 46 Hz en vez de 50.
  - El bridge Python del sim recalcula el PD solo al recibir lowcmd → aunque
    la policy decide a 50 Hz, re-publicamos a 500 Hz para refrescar el PD
    (en el robot real esto es gratis: el PD corre a kHz en cada motor).

Uso:
    python 03_walk_policy.py                     # sim, himloco, teclado
    python 03_walk_policy.py --policy robot_lab  # la otra policy
    python 03_walk_policy.py --cmd 0.5 0 0 --dur 12   # comando fijo, sin teclado
    python 03_walk_policy.py --real eth0         # robot real (¡ARNÉS!)
"""

import argparse
import sys
import termios
import threading
import time
import tty
from collections import deque
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

POLICY_ROOT = Path(__file__).resolve().parent.parent / "external" / "rl_sar" / "policy" / "go2"

# Interfaces de cada policy, transcritas de external/rl_sar/policy/go2/*/config.yaml.
# joint_map traduce orden-de-entrenamiento -> orden-SDK: train[i] <-> sdk[joint_map[i]].
# default_pose y action_scale están en ORDEN DE ENTRENAMIENTO.
POLICIES = {
    "himloco": {
        "file": POLICY_ROOT / "himloco" / "himloco.pt",
        "obs_order": ["commands", "ang_vel", "gravity_vec", "dof_pos", "dof_vel", "actions"],
        "cmd_scale": np.array([2.0, 2.0, 0.25]),
        "history": 6,  # input = 6 obs de 45 concatenadas, más nueva primero
        "joint_map": [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8],  # train=FL,FR,RL,RR
        "default_pose": np.array([0.1, 0.8, -1.5, -0.1, 0.8, -1.5,
                                  0.1, 1.0, -1.5, -0.1, 1.0, -1.5]),
        "action_scale": np.array([0.125, 0.25, 0.25] * 4),
        "rl_kp": 40.0, "rl_kd": 1.0,
        "ramp_kp": 60.0, "ramp_kd": 5.0,
    },
    "robot_lab": {
        "file": POLICY_ROOT / "robot_lab" / "policy.pt",
        "obs_order": ["ang_vel", "gravity_vec", "commands", "dof_pos", "dof_vel", "actions"],
        "cmd_scale": np.array([1.0, 1.0, 1.0]),
        "history": 1,  # sin historia: input = la obs de 45 tal cual
        "joint_map": list(range(12)),  # train = SDK (FR,FL,RR,RL)
        "default_pose": np.array([0.0, 0.8, -1.5] * 4),
        "action_scale": np.array([0.125, 0.25, 0.25] * 4),
        "rl_kp": 20.0, "rl_kd": 0.5,
        "ramp_kp": 80.0, "ramp_kd": 3.0,
    },
}

ANG_VEL_SCALE = 0.25
DOF_VEL_SCALE = 0.05
CLIP_OBS = 100.0
POLICY_DT = 0.02        # 50 Hz: el ritmo de decisión con el que se entrenaron
PD_REFRESH_DT = 0.002   # 500 Hz: re-publicación para refrescar el PD del bridge
CMD_STEP = 0.1
CMD_LIMITS = np.array([1.0, 0.5, 1.0])  # |vx|, |vy|, |vyaw| máximos por seguridad


def gravity_in_body_frame(quat) -> np.ndarray:
    """Proyecta la gravedad al marco del cuerpo desde el quaternion (w,x,y,z).

    Robot derecho → (0, 0, -1); inclinado, la componente horizontal crece.
    Misma fórmula que QuatRotateInverse(q, [0,0,-1]) de rl_sar.
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
    ap.add_argument("--policy", default="himloco", choices=list(POLICIES),
                    help="qué policy pre-entrenada usar")
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

    cfg = POLICIES[args.policy]
    jmap = np.array(cfg["joint_map"])

    policy = torch.jit.load(str(cfg["file"]), map_location=args.device)
    policy.eval()
    print(f"Policy '{args.policy}' cargada ({cfg['file'].name}, "
          f"obs {45 * cfg['history']}, device {args.device})")

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

    def send_cmd(q_des_sdk: np.ndarray, kp: float, kd: float):
        for i in range(12):
            cmd_msg.motor_cmd[i].q = float(q_des_sdk[i])
            cmd_msg.motor_cmd[i].dq = 0.0
            cmd_msg.motor_cmd[i].tau = 0.0
            cmd_msg.motor_cmd[i].kp = kp
            cmd_msg.motor_cmd[i].kd = kd
        cmd_msg.crc = crc.Crc(cmd_msg)
        pub.Write(cmd_msg)

    def read_state():
        """Devuelve (msg, q_train, dq_train): estado ya en orden de entrenamiento."""
        msg = latest["msg"]
        q_sdk = np.array([msg.motor_state[i].q for i in range(12)])
        dq_sdk = np.array([msg.motor_state[i].dq for i in range(12)])
        return msg, q_sdk[jmap], dq_sdk[jmap]

    def to_sdk_order(v_train: np.ndarray) -> np.ndarray:
        v_sdk = np.empty(12)
        v_sdk[jmap] = v_train
        return v_sdk

    # --- Fase 1: rampa a la pose nominal (el stand de siempre) ---
    msg = latest["msg"]
    q_start_sdk = np.array([msg.motor_state[i].q for i in range(12)])
    default_pose_sdk = to_sdk_order(cfg["default_pose"])
    print("Rampa a la pose nominal...")
    t = 0.0
    deadline = time.perf_counter()
    while t < 3.0:
        s = np.tanh(t / 1.2)
        send_cmd((1 - s) * q_start_sdk + s * default_pose_sdk,
                 cfg["ramp_kp"], cfg["ramp_kd"])
        t += PD_REFRESH_DT
        deadline += PD_REFRESH_DT
        pause = deadline - time.perf_counter()
        if pause > 0:
            time.sleep(pause)

    # --- Fase 2: la policy toma el control ---
    if args.cmd is not None:
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
    # Historia de observaciones (himloco): arranca en ceros, como el buffer de rl_sar
    obs_history = deque([np.zeros(45, dtype=np.float32)] * cfg["history"],
                        maxlen=cfg["history"])
    q_des_sdk = default_pose_sdk.copy()
    steps_per_policy = int(round(POLICY_DT / PD_REFRESH_DT))  # 10
    step = 0
    t_end = time.time() + args.dur if args.dur > 0 else None
    deadline = time.perf_counter()

    try:
        while not teleop.quit and (t_end is None or time.time() < t_end):
            if step % steps_per_policy == 0:
                # Cada 20 ms: nueva decisión de la policy
                msg, q_train, dq_train = read_state()
                ang_vel = np.array(msg.imu_state.gyroscope)
                gravity = gravity_in_body_frame(msg.imu_state.quaternion)

                parts = {
                    "commands": teleop.cmd * cfg["cmd_scale"],
                    "ang_vel": ang_vel * ANG_VEL_SCALE,
                    "gravity_vec": gravity,
                    "dof_pos": q_train - cfg["default_pose"],
                    "dof_vel": dq_train * DOF_VEL_SCALE,
                    "actions": last_action,
                }
                obs = np.concatenate([parts[k] for k in cfg["obs_order"]]).astype(np.float32)
                obs = np.clip(obs, -CLIP_OBS, CLIP_OBS)

                obs_history.appendleft(obs)          # más nueva primero
                net_input = np.concatenate(obs_history)

                with torch.no_grad():
                    action = policy(torch.from_numpy(net_input).unsqueeze(0).to(args.device))
                last_action = action.squeeze(0).cpu().numpy()

                q_des_train = cfg["default_pose"] + cfg["action_scale"] * last_action
                q_des_sdk = to_sdk_order(q_des_train)

            # Cada 2 ms: re-publicar para que el bridge recalcule el PD fresco
            send_cmd(q_des_sdk, cfg["rl_kp"], cfg["rl_kd"])
            step += 1

            deadline += PD_REFRESH_DT
            pause = deadline - time.perf_counter()
            if pause > 0:
                time.sleep(pause)
            elif pause < -0.05:
                deadline = time.perf_counter()  # nos atrasamos mucho: resincronizar
    except KeyboardInterrupt:
        pass

    # --- Salida segura: amortiguación (kp=0) ---
    print("\nDejando el robot en amortiguación...")
    q_now_sdk = np.array([latest["msg"].motor_state[i].q for i in range(12)])
    for _ in range(500):
        send_cmd(q_now_sdk, 0.0, 2.0)
        time.sleep(0.002)
    print("Listo.")


if __name__ == "__main__":
    main()
