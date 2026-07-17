#!/usr/bin/env python3
"""Lee y muestra rt/lowstate — el "pulso" del robot (sim o real).

Qué es lowstate (concepto de robótica):
    Es la telemetría de bajo nivel que el robot publica a ~500 Hz por DDS.
    Contiene el estado de cada motor, la IMU y los sensores de pie. Es la
    "observación" cruda sobre la que se construye todo lo demás (estimación
    de estado, políticas RL, control).

    Por motor (12 en el Go2 = 4 patas x 3 articulaciones):
      q       posición angular [rad]   — dónde está la articulación
      dq      velocidad angular [rad/s]
      tau_est torque estimado [N·m]    — cuánto esfuerzo está haciendo
    Orden de los 12: FR_hip, FR_thigh, FR_calf, FL_..., RR_..., RL_...
    (F/R = front/rear, R/L = right/left; hip=abducción, thigh=muslo, calf=rodilla)

    IMU (Inertial Measurement Unit): acelerómetro + giróscopo fusionados en
    una orientación (quaternion y roll-pitch-yaw). Es el sentido del
    equilibrio del robot. OJO: da orientación, NO posición — la posición en
    el mundo hay que estimarla (odometría) o medirla externamente.

    foot_force: 4 sensores de contacto en los pies (uno por pata). En el
    bridge Python del sim no se simulan (quedan en 0); en el real sí.

Uso:
    python 00_read_lowstate.py              # sim (DDS domain 1, interfaz lo)
    python 00_read_lowstate.py --real eth0  # robot real (domain 0, ethernet)
    python 00_read_lowstate.py -n 5         # imprime 5 snapshots y sale
"""

import argparse
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

JOINT_NAMES = ["hip", "thigh", "calf"]
LEG_NAMES = ["FR", "FL", "RR", "RL"]


class LowStateBuffer:
    """Guarda el último mensaje recibido y cuenta cuántos llegaron."""

    def __init__(self):
        self.msg = None
        self.count = 0

    def on_msg(self, msg: LowState_):
        # Este callback corre en el hilo de DDS cada vez que llega un mensaje
        # (~500 Hz). Solo guardamos referencia; el trabajo pesado va afuera.
        self.msg = msg
        self.count += 1


def print_snapshot(msg: LowState_, measured_hz: float):
    rpy = msg.imu_state.rpy
    gyro = msg.imu_state.gyroscope
    acc = msg.imu_state.accelerometer
    print(f"\n=== tick={msg.tick}  ({measured_hz:.0f} msgs/s) ===")
    print(f"IMU  rpy  [rad]: roll={rpy[0]:+.3f}  pitch={rpy[1]:+.3f}  yaw={rpy[2]:+.3f}")
    print(f"IMU  gyro [rad/s]: {gyro[0]:+.3f} {gyro[1]:+.3f} {gyro[2]:+.3f}")
    print(f"IMU  acc  [m/s²]:  {acc[0]:+.3f} {acc[1]:+.3f} {acc[2]:+.3f}   (quieto ≈ +9.81 en z)")
    print(f"{'pata':<4} {'q hip':>8} {'q thigh':>8} {'q calf':>8} | {'tau est [N·m]':>20} | pie[N]")
    for leg in range(4):
        motors = [msg.motor_state[leg * 3 + j] for j in range(3)]
        qs = " ".join(f"{m.q:+8.3f}" for m in motors)
        taus = " ".join(f"{m.tau_est:+6.2f}" for m in motors)
        print(f"{LEG_NAMES[leg]:<4} {qs} | {taus} | {msg.foot_force[leg]:4d}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--real", metavar="IFACE", default=None,
                    help="interfaz de red hacia el robot real (ej: eth0). Sin esto: sim.")
    ap.add_argument("--hz", type=float, default=2.0, help="frecuencia de impresión")
    ap.add_argument("-n", type=int, default=0, help="cantidad de snapshots (0 = infinito)")
    args = ap.parse_args()

    # ChannelFactoryInitialize(domain, interfaz) arranca DDS:
    #  - sim:  domain 1 sobre la interfaz de loopback "lo" (todo en tu máquina)
    #  - real: domain 0 sobre la interfaz ethernet conectada al robot
    # El domain id es un "canal aislado": participantes en domains distintos
    # no se ven entre sí, por eso sim y real no se pisan.
    if args.real:
        ChannelFactoryInitialize(0, args.real)
    else:
        ChannelFactoryInitialize(1, "lo")

    buf = LowStateBuffer()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(buf.on_msg, 10)  # 10 = tamaño de cola interna

    print("Esperando mensajes en rt/lowstate... (¿está corriendo el simulador?)")
    printed = 0
    prev_count = 0
    try:
        while args.n == 0 or printed < args.n:
            time.sleep(1.0 / args.hz)
            if buf.msg is None:
                continue
            measured_hz = (buf.count - prev_count) * args.hz
            prev_count = buf.count
            print_snapshot(buf.msg, measured_hz)
            printed += 1
    except KeyboardInterrupt:
        pass
    print(f"\nTotal de mensajes recibidos: {buf.count}")


if __name__ == "__main__":
    main()
