#!/usr/bin/env python3
"""Hace que el Go2 se pare y se vuelva a agachar, comandando motores por rt/lowcmd.

Conceptos de robótica que aparecen acá:

1) Control PD por articulación (a.k.a. "impedance control" simplificado).
   No le mandás al motor "andá a este ángulo" y listo: le mandás una CONSIGNA
   (q_des, dq_des, tau_ff, kp, kd) y el driver del motor calcula a 10+ kHz:

       tau_salida = tau_ff + kp * (q_des - q_medida) + kd * (dq_des - dq_medida)

   kp = rigidez del "resorte" virtual hacia q_des (más kp = más duro).
   kd = amortiguación (frena oscilaciones; también es fricción viscosa).
   Con kp/kd bajos el robot queda "blando" (podés moverle las patas a mano);
   con kp/kd altos, rígido. Elegir ganancias ES parte del diseño de control.

2) Trayectoria por interpolación. Saltar de la pose actual a la pose "parado"
   en un paso rompería todo (torques infinitos). Se interpola suavemente:
   q_des(t) = (1-s(t)) * q_inicial + s(t) * q_final, con s(t) yendo de 0 a 1.
   Usamos s = tanh(t/T) como el ejemplo oficial de Unitree.

3) Seguridad: leemos la pose REAL del robot antes de mover (no asumimos que
   está agachado), y el loop corre a 500 Hz con timing explícito. En el robot
   físico, antes de usar lowcmd hay que apagar el servicio de movimiento a
   bordo (sport mode) con MotionSwitcherClient — si no, ambos controladores
   pelean por los motores. En sim no existe ese servicio.

Uso:
    python 01_stand.py              # sim
    python 01_stand.py --real eth0  # robot real (¡primero en el arnés!)
"""

import argparse
import time

import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# Poses de referencia [rad], orden: FR(hip,thigh,calf), FL, RR, RL.
# Vienen del ejemplo oficial de unitree_mujoco.
STAND_POSE = np.array([
    0.00571868, 0.608813, -1.21763,
    -0.00571868, 0.608813, -1.21763,
    0.00571868, 0.608813, -1.21763,
    -0.00571868, 0.608813, -1.21763,
])
CROUCH_POSE = np.array([
    0.0473455, 1.22187, -2.44375,
    -0.0473455, 1.22187, -2.44375,
    0.0473455, 1.22187, -2.44375,
    -0.0473455, 1.22187, -2.44375,
])

DT = 0.002           # período del loop de control: 500 Hz
RAMP_TIME_CONST = 1.2  # constante de tiempo de la interpolación tanh [s]
KP, KD = 50.0, 3.5   # ganancias PD (las del ejemplo oficial)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--real", metavar="IFACE", default=None,
                    help="interfaz hacia el robot real (ej: eth0). Sin esto: sim.")
    args = ap.parse_args()

    if args.real:
        ChannelFactoryInitialize(0, args.real)
        # En el robot real, liberar el controlador de a bordo antes de lowcmd.
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient,
        )
        msc = MotionSwitcherClient()
        msc.SetTimeout(5.0)
        msc.Init()
        while True:
            code, result = msc.CheckMode()
            if not result or not result.get("name"):
                break  # ya no hay ningún modo activo
            print(f"Liberando modo activo: {result['name']}...")
            msc.ReleaseMode()
            time.sleep(1.0)
    else:
        ChannelFactoryInitialize(1, "lo")

    # --- Suscripción a lowstate: necesitamos la pose actual para arrancar ---
    latest = {"msg": None}
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(lambda m: latest.update(msg=m), 10)

    print("Esperando rt/lowstate...")
    while latest["msg"] is None:
        time.sleep(0.1)
    q_start = np.array([latest["msg"].motor_state[i].q for i in range(12)])
    print(f"Pose inicial leída. q[0:3] (FR) = {np.round(q_start[:3], 3)}")

    # --- Publisher de lowcmd ---
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    crc = CRC()

    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0], cmd.head[1] = 0xFE, 0xEF  # magic bytes del protocolo
    cmd.level_flag = 0xFF                  # 0xFF = comando de bajo nivel
    for i in range(20):                    # el mensaje tiene 20 slots; Go2 usa 12
        cmd.motor_cmd[i].mode = 0x01       # 0x01 = motor en modo servo (FOC)

    def send_cmd(q_des: np.ndarray, kp: float, kd: float):
        for i in range(12):
            cmd.motor_cmd[i].q = float(q_des[i])
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = kp
            cmd.motor_cmd[i].kd = kd
        cmd.crc = crc.Crc(cmd)  # checksum obligatorio: el robot descarta sin CRC
        pub.Write(cmd)

    def ramp(q_from: np.ndarray, q_to: np.ndarray, duration: float, label: str):
        """Interpola q_from → q_to publicando a 500 Hz."""
        print(label)
        t = 0.0
        while t < duration:
            step_start = time.perf_counter()
            s = np.tanh(t / RAMP_TIME_CONST)  # 0 → ~1, suave al arranque
            send_cmd((1 - s) * q_from + s * q_to, KP, KD)
            t += DT
            time_left = DT - (time.perf_counter() - step_start)
            if time_left > 0:
                time.sleep(time_left)  # timing explícito: loop de período fijo

    def hold(q_des: np.ndarray, duration: float, label: str):
        print(label)
        t_end = time.time() + duration
        while time.time() < t_end:
            step_start = time.perf_counter()
            send_cmd(q_des, KP, KD)
            time_left = DT - (time.perf_counter() - step_start)
            if time_left > 0:
                time.sleep(time_left)

    try:
        ramp(q_start, STAND_POSE, 4.0, "Parándose...")
        hold(STAND_POSE, 3.0, "De pie. Manteniendo 3 s...")
        ramp(STAND_POSE, CROUCH_POSE, 4.0, "Agachándose...")
        # "Damping": kp=0 deja solo el término kd → el robot queda blando y
        # se asienta sin resistencia. Es el estado seguro para terminar.
        print("Aflojando (solo amortiguación)...")
        for _ in range(int(1.0 / DT)):
            send_cmd(CROUCH_POSE, 0.0, 2.0)
            time.sleep(DT)
    except KeyboardInterrupt:
        print("\nInterrumpido: dejando el robot en amortiguación...")
        for _ in range(int(1.0 / DT)):
            send_cmd(np.array([latest["msg"].motor_state[i].q for i in range(12)]), 0.0, 2.0)
            time.sleep(DT)

    print("Listo.")


if __name__ == "__main__":
    main()
