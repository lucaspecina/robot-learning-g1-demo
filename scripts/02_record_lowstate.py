#!/usr/bin/env python3
"""Graba rt/lowstate a un .npz para explorarlo después en un notebook.

La idea: el stream DDS es efímero (si no lo escuchás, se pierde). Para
analizar señales (IMU, torques, contactos) conviene grabar un rato de datos
y mirarlos offline con numpy/matplotlib — igual que harías con un dataset.

Uso:
    python 02_record_lowstate.py --dur 10          # graba 10 s (sim)
    python 02_record_lowstate.py --real eth0       # contra el robot real
Salida: data/lowstate_YYYYMMDD_HHMMSS.npz con arrays:
    t (N,), q/dq/tau (N,12), rpy/gyro/acc (N,3), quat (N,4), foot (N,4)
"""

import argparse
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_


class Recorder:
    def __init__(self):
        self.rows = []

    def on_msg(self, msg: LowState_):
        # Corre a ~500 Hz en el hilo DDS: solo copiamos números, nada pesado.
        self.rows.append((
            time.time(),
            [m.q for m in msg.motor_state[:12]],
            [m.dq for m in msg.motor_state[:12]],
            [m.tau_est for m in msg.motor_state[:12]],
            list(msg.imu_state.rpy),
            list(msg.imu_state.gyroscope),
            list(msg.imu_state.accelerometer),
            list(msg.imu_state.quaternion),
            list(msg.foot_force),
        ))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dur", type=float, default=10.0, help="duración de la grabación [s]")
    ap.add_argument("--real", metavar="IFACE", default=None,
                    help="interfaz hacia el robot real (ej: eth0). Sin esto: sim.")
    ap.add_argument("--out", default=None, help="archivo de salida .npz")
    args = ap.parse_args()

    if args.real:
        ChannelFactoryInitialize(0, args.real)
    else:
        ChannelFactoryInitialize(1, "lo")

    rec = Recorder()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(rec.on_msg, 10)

    print(f"Grabando {args.dur:.0f} s de rt/lowstate...")
    time.sleep(args.dur)
    n = len(rec.rows)
    rows = rec.rows[:n]  # congelar (el callback puede seguir llegando)

    if n == 0:
        raise SystemExit("No llegó ningún mensaje. ¿Está corriendo el simulador?")

    t0 = rows[0][0]
    arrays = {
        "t": np.array([r[0] - t0 for r in rows]),
        "q": np.array([r[1] for r in rows]),
        "dq": np.array([r[2] for r in rows]),
        "tau": np.array([r[3] for r in rows]),
        "rpy": np.array([r[4] for r in rows]),
        "gyro": np.array([r[5] for r in rows]),
        "acc": np.array([r[6] for r in rows]),
        "quat": np.array([r[7] for r in rows]),
        "foot": np.array([r[8] for r in rows]),
    }

    if args.out:
        out = Path(args.out)
    else:
        data_dir = Path(__file__).resolve().parent.parent / "data"
        data_dir.mkdir(exist_ok=True)
        out = data_dir / f"lowstate_{datetime.now():%Y%m%d_%H%M%S}.npz"
    np.savez_compressed(out, **arrays)
    hz = n / args.dur
    print(f"Guardado {out}  ({n} muestras, ~{hz:.0f} Hz)")


if __name__ == "__main__":
    main()
