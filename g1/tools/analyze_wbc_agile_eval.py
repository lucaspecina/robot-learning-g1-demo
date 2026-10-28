#!/usr/bin/env python3
"""Calcula métricas comparables de las evaluaciones intactas de WBC-AGILE."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Resume una trayectoria de WBC-AGILE sin modificar sus datos."
    )
    parser.add_argument("scenario", choices=("stand", "walk_stop"))
    parser.add_argument("trajectory", type=Path)
    return parser.parse_args()


def rounded(value: Any) -> float:
    return round(float(value), 6)


def planar_distance(frame: pd.DataFrame) -> pd.Series:
    start_x = frame["root_pos_0"].iloc[0]
    start_y = frame["root_pos_1"].iloc[0]
    delta_x = frame["root_pos_0"] - start_x
    delta_y = frame["root_pos_1"] - start_y
    return (delta_x * delta_x + delta_y * delta_y).pow(0.5)


def planar_speed(frame: pd.DataFrame) -> pd.Series:
    speed_x = frame["root_lin_vel_0"]
    speed_y = frame["root_lin_vel_1"]
    return (speed_x * speed_x + speed_y * speed_y).pow(0.5)


def heading_metrics(frame: pd.DataFrame) -> dict[str, float]:
    w = frame["root_rot_0"].to_numpy()
    x = frame["root_rot_1"].to_numpy()
    y = frame["root_rot_2"].to_numpy()
    z = frame["root_rot_3"].to_numpy()
    yaw = np.unwrap(
        np.arctan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
    )
    relative_yaw = yaw - yaw[0]
    return {
        "net_yaw_change_deg": rounded(np.degrees(relative_yaw[-1])),
        "max_absolute_yaw_change_deg": rounded(
            np.degrees(np.max(np.abs(relative_yaw)))
        ),
    }


def select_time(frame: pd.DataFrame, start_s: float, end_s: float) -> pd.DataFrame:
    selected = frame[
        (frame["timestep"] >= start_s) & (frame["timestep"] <= end_s)
    ]
    if selected.empty:
        raise ValueError(
            f"No hay muestras entre {start_s:.1f} y {end_s:.1f} segundos."
        )
    return selected


def displacement_metrics(frame: pd.DataFrame) -> dict[str, float]:
    distance = planar_distance(frame)
    duration_s = frame["timestep"].iloc[-1] - frame["timestep"].iloc[0]
    return {
        "duration_s": rounded(duration_s),
        "delta_x_m": rounded(frame["root_pos_0"].iloc[-1] - frame["root_pos_0"].iloc[0]),
        "delta_y_m": rounded(frame["root_pos_1"].iloc[-1] - frame["root_pos_1"].iloc[0]),
        "net_displacement_m": rounded(distance.iloc[-1]),
        "max_displacement_m": rounded(distance.max()),
        "net_drift_rate_mps": rounded(distance.iloc[-1] / duration_s),
        **heading_metrics(frame),
    }


def speed_metrics(frame: pd.DataFrame) -> dict[str, float]:
    speed = planar_speed(frame)
    return {
        "mean_planar_speed_mps": rounded(speed.mean()),
        "p95_planar_speed_mps": rounded(speed.quantile(0.95)),
        "max_planar_speed_mps": rounded(speed.max()),
    }


def upper_body_motion(
    frame: pd.DataFrame, metadata_path: Path
) -> dict[str, float | str] | None:
    if not metadata_path.exists():
        return None

    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    joint_names = metadata.get("joint_names", [])
    ranges: list[tuple[str, float]] = []
    for index, name in enumerate(joint_names):
        if any(part in name for part in ("hip", "knee", "ankle")):
            continue
        column = f"joint_pos_{index}"
        if column in frame:
            ranges.append((name, float(frame[column].max() - frame[column].min())))

    if not ranges:
        return None

    largest_name, largest_range = max(ranges, key=lambda item: item[1])
    return {
        "largest_moving_joint": largest_name,
        "largest_joint_range_rad": rounded(largest_range),
        "mean_joint_range_rad": rounded(
            sum(value for _, value in ranges) / len(ranges)
        ),
    }


def analyze_stand(frame: pd.DataFrame) -> dict[str, Any]:
    stable = select_time(frame, 5.0, frame["timestep"].iloc[-1])
    return {
        "full": displacement_metrics(frame),
        "after_settling": {
            **displacement_metrics(stable),
            **speed_metrics(stable),
        },
    }


def analyze_walk_stop(
    frame: pd.DataFrame, metadata_path: Path
) -> dict[str, Any]:
    before_walk = select_time(frame, 2.0, 5.0)
    steady_walk = select_time(frame, 7.0, 15.0)
    after_command_stop = select_time(frame, 15.0, frame["timestep"].iloc[-1])
    settled_stop = select_time(frame, 20.0, frame["timestep"].iloc[-1])

    commanded_x = steady_walk["commands_0"]
    actual_x = steady_walk["root_lin_vel_robot_0"]
    actual_y = steady_walk["root_lin_vel_robot_1"]
    actual_yaw = steady_walk["root_ang_vel_2"]

    before_walk_metrics: dict[str, Any] = {
        **displacement_metrics(before_walk),
        **speed_metrics(before_walk),
    }
    settled_stop_metrics: dict[str, Any] = {
        **displacement_metrics(settled_stop),
        **speed_metrics(settled_stop),
    }
    before_walk_upper = upper_body_motion(before_walk, metadata_path)
    settled_stop_upper = upper_body_motion(settled_stop, metadata_path)
    if before_walk_upper is not None:
        before_walk_metrics["upper_body_motion"] = before_walk_upper
    if settled_stop_upper is not None:
        settled_stop_metrics["upper_body_motion"] = settled_stop_upper

    return {
        "before_walk": before_walk_metrics,
        "steady_walk": {
            **displacement_metrics(steady_walk),
            "mean_commanded_x_mps": rounded(commanded_x.mean()),
            "mean_actual_x_mps": rounded(actual_x.mean()),
            "mean_absolute_x_error_mps": rounded((actual_x - commanded_x).abs().mean()),
            "mean_absolute_y_mps": rounded(actual_y.abs().mean()),
            "mean_absolute_yaw_rps": rounded(actual_yaw.abs().mean()),
        },
        "after_command_stop": {
            **displacement_metrics(after_command_stop),
            **speed_metrics(after_command_stop),
        },
        "settled_stop": settled_stop_metrics,
    }


def main() -> None:
    args = parse_args()
    frame = pd.read_parquet(args.trajectory).sort_values("timestep")
    if len(frame) < 2:
        raise ValueError("La trayectoria no tiene suficientes muestras.")

    required_columns = {
        "timestep",
        "is_success",
        "root_pos_0",
        "root_pos_1",
        "root_pos_2",
        "root_rot_0",
        "root_rot_1",
        "root_rot_2",
        "root_rot_3",
        "root_lin_vel_0",
        "root_lin_vel_1",
    }
    if args.scenario == "walk_stop":
        required_columns.update(
            {
                "commands_0",
                "root_lin_vel_robot_0",
                "root_lin_vel_robot_1",
                "root_ang_vel_2",
            }
        )
    missing = sorted(required_columns.difference(frame.columns))
    if missing:
        raise ValueError(f"Faltan columnas necesarias: {', '.join(missing)}")

    metrics: dict[str, Any] = {
        "scenario": args.scenario,
        "samples": len(frame),
        "duration_s": rounded(frame["timestep"].iloc[-1] - frame["timestep"].iloc[0]),
        "success": bool(frame["is_success"].iloc[-1]),
        "minimum_base_height_m": rounded(frame["root_pos_2"].min()),
    }
    upper_motion = upper_body_motion(frame, args.trajectory.parent / "metadata.json")
    if upper_motion is not None:
        metrics["upper_body_motion"] = upper_motion
    metrics.update(
        analyze_stand(frame)
        if args.scenario == "stand"
        else analyze_walk_stop(frame, args.trajectory.parent / "metadata.json")
    )

    # JSON permite comparar corridas sin depender del formato visual de pandas.
    print(json.dumps(metrics, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
