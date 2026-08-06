#!/usr/bin/env python3
"""Resume la trayectoria producida por el adaptador oficial de MuJoCo."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import pandas as pd


def select_time(
    frame: pd.DataFrame,
    start_s: float,
    end_s: float,
) -> pd.DataFrame:
    selected = frame[
        (frame["timestep"] >= start_s) & (frame["timestep"] <= end_s)
    ]
    if len(selected) < 2:
        raise ValueError(
            f"No hay suficientes muestras entre {start_s:.1f} y {end_s:.1f} s."
        )
    return selected


def motion(frame: pd.DataFrame) -> dict[str, float]:
    dx = float(frame["root_pos_0"].iloc[-1] - frame["root_pos_0"].iloc[0])
    dy = float(frame["root_pos_1"].iloc[-1] - frame["root_pos_1"].iloc[0])
    path_angle = abs(math.degrees(math.atan2(dy, max(dx, 1e-9))))
    dt = frame["timestep"].diff().fillna(0.0)
    yaw_change = float((frame["root_ang_vel_robot_2"] * dt).sum())
    return {
        "delta_x_m": round(dx, 6),
        "delta_y_m": round(dy, 6),
        "displacement_m": round(math.hypot(dx, dy), 6),
        "path_angle_deg": round(path_angle, 6),
        "integrated_yaw_change_deg": round(math.degrees(yaw_change), 6),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("trajectory", type=Path)
    args = parser.parse_args()

    frame = pd.read_parquet(args.trajectory).sort_values("timestep")
    before_walk = select_time(frame, 2.0, 5.0)
    walk = select_time(frame, 5.0, 15.0)
    stopped = select_time(frame, 20.0, 30.0)
    report = {
        "samples": len(frame),
        "minimum_base_height_m": round(float(frame["root_pos_2"].min()), 6),
        "before_walk": motion(before_walk),
        "walk": {
            **motion(walk),
            "mean_commanded_x_mps": round(
                float(walk["commands_0"].mean()),
                6,
            ),
            "mean_actual_x_mps": round(
                float(walk["root_lin_vel_robot_0"].mean()),
                6,
            ),
            "mean_actual_y_mps": round(
                float(walk["root_lin_vel_robot_1"].mean()),
                6,
            ),
        },
        "settled_stop": motion(stopped),
    }
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
