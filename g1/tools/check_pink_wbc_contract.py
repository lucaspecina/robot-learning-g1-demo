#!/usr/bin/env python3
"""Comprueba que Pink resuelva las muñecas del cuerpo WBC-AGILE de la demo."""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(
    description=(
        "Verifica el contrato cinemático entre Pink, el URDF y el cuerpo "
        "WBC-AGILE antes de incorporarlo al lazo físico."
    )
)
parser.add_argument("--descriptor", type=Path, required=True)
parser.add_argument("--urdf", type=Path, required=True)
parser.add_argument("--iterations", type=int, default=200)
parser.add_argument("--dt", type=float, default=0.02)
parser.add_argument("--maximum-position-error-m", type=float, default=0.005)
parser.add_argument("--maximum-orientation-error-deg", type=float, default=1.0)
parser.add_argument(
    "--enable_pinocchio",
    action="store_true",
    default=True,
    help="Carga Pinocchio antes de Isaac, como requiere el flujo oficial.",
)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

from isaaclab.controllers.pink_ik.local_frame_task import LocalFrameTask
from isaaclab.controllers.pink_ik.null_space_posture_task import (
    NullSpacePostureTask,
)
from isaaclab.controllers.pink_ik.pink_ik import PinkIKController
from isaaclab.controllers.pink_ik.pink_ik_cfg import PinkIKControllerCfg

from agile.rl_env.assets.robots.unitree_g1 import G1_29DOF


G1_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(G1_ROOT))

from arm_control import ARM_JOINTS, POSES  # noqa: E402


def frame_error(current: pin.SE3, target: pin.SE3) -> dict:
    """Mide por separado el error de posición y el de orientación."""
    position_error = float(np.linalg.norm(current.translation - target.translation))
    rotation_error = target.rotation.T @ current.rotation
    orientation_error = float(np.linalg.norm(pin.log3(rotation_error)))
    return {
        "position_error_m": position_error,
        "orientation_error_deg": math.degrees(orientation_error),
    }


def main() -> int:
    if not args.descriptor.is_file():
        raise FileNotFoundError(f"no existe el descriptor: {args.descriptor}")
    if not args.urdf.is_file():
        raise FileNotFoundError(f"no existe el URDF: {args.urdf}")
    if args.iterations < 1 or args.dt <= 0.0:
        raise ValueError("iterations y dt deben ser positivos")

    descriptor = yaml.safe_load(args.descriptor.read_text(encoding="utf-8"))
    articulation = descriptor["articulations"]["robot"]
    all_joint_names = list(articulation["joint_names"])
    current_joint_positions = np.asarray(
        articulation["default_joint_pos"], dtype=np.float64
    )
    arm_joint_indices = [all_joint_names.index(name) for name in ARM_JOINTS]

    left_task = LocalFrameTask(
        "left_wrist_yaw_link",
        base_link_frame_name="pelvis",
        position_cost=8.0,
        orientation_cost=2.0,
        lm_damping=10,
        gain=0.5,
    )
    right_task = LocalFrameTask(
        "right_wrist_yaw_link",
        base_link_frame_name="pelvis",
        position_cost=8.0,
        orientation_cost=2.0,
        lm_damping=10,
        gain=0.5,
    )
    posture_task = NullSpacePostureTask(
        cost=0.5,
        lm_damping=1,
        controlled_frames=["left_wrist_yaw_link", "right_wrist_yaw_link"],
        controlled_joints=[
            name for name in ARM_JOINTS if "_shoulder_" in name
        ],
        gain=0.3,
    )
    controller_cfg = PinkIKControllerCfg(
        urdf_path=str(args.urdf),
        mesh_path=str(args.urdf.parent),
        num_hand_joints=0,
        variable_input_tasks=[left_task, right_task, posture_task],
        fixed_input_tasks=[],
        joint_names=list(ARM_JOINTS),
        all_joint_names=all_joint_names,
        articulation_name="robot",
        base_link_name="pelvis",
        show_ik_warnings=True,
        fail_on_joint_limit_violation=True,
    )
    controller = PinkIKController(
        cfg=controller_cfg,
        robot_cfg=G1_29DOF,
        device="cpu",
        controlled_joint_indices=arm_joint_indices,
    )

    # Isaac Lab copia los objetos de configuración al construir el
    # controlador. Los objetivos deben escribirse en esas copias, que son las
    # que realmente lee el resolvedor, y no en los objetos usados como molde.
    left_task, right_task, posture_task = controller.cfg.variable_input_tasks

    target_joint_positions = current_joint_positions.copy()
    target_joint_positions[arm_joint_indices] = POSES["transporte"]
    target_in_pink_order = target_joint_positions[
        controller.isaac_lab_to_pink_ordering
    ]
    controller.pink_configuration.update(target_in_pink_order)
    left_target = controller.pink_configuration.get_transform(
        "left_wrist_yaw_link", "pelvis"
    ).copy()
    right_target = controller.pink_configuration.get_transform(
        "right_wrist_yaw_link", "pelvis"
    ).copy()
    left_task.set_target(left_target)
    right_task.set_target(right_target)
    posture_task.set_target(controller.pink_configuration.q)

    controller.pink_configuration.update(
        current_joint_positions[controller.isaac_lab_to_pink_ordering]
    )
    initial_left_error = frame_error(
        controller.pink_configuration.get_transform(
            "left_wrist_yaw_link", "pelvis"
        ),
        left_target,
    )
    initial_right_error = frame_error(
        controller.pink_configuration.get_transform(
            "right_wrist_yaw_link", "pelvis"
        ),
        right_target,
    )

    for _ in range(args.iterations):
        arm_targets = controller.compute(current_joint_positions, args.dt)
        current_joint_positions[arm_joint_indices] = (
            arm_targets.detach().cpu().numpy()
        )

    final_in_pink_order = current_joint_positions[
        controller.isaac_lab_to_pink_ordering
    ]
    controller.pink_configuration.update(final_in_pink_order)
    left_error = frame_error(
        controller.pink_configuration.get_transform(
            "left_wrist_yaw_link", "pelvis"
        ),
        left_target,
    )
    right_error = frame_error(
        controller.pink_configuration.get_transform(
            "right_wrist_yaw_link", "pelvis"
        ),
        right_target,
    )
    maximum_position_error = max(
        left_error["position_error_m"], right_error["position_error_m"]
    )
    maximum_orientation_error = max(
        left_error["orientation_error_deg"],
        right_error["orientation_error_deg"],
    )
    accepted = (
        maximum_position_error <= args.maximum_position_error_m
        and maximum_orientation_error <= args.maximum_orientation_error_deg
        and np.isfinite(current_joint_positions).all()
    )
    report = {
        "accepted": bool(accepted),
        "urdf": str(args.urdf),
        "body_joint_count": len(all_joint_names),
        "controlled_arm_joint_count": len(arm_joint_indices),
        "iterations": args.iterations,
        "dt_s": args.dt,
        "initial_left_wrist": initial_left_error,
        "initial_right_wrist": initial_right_error,
        "left_wrist": left_error,
        "right_wrist": right_error,
        "maximum_arm_joint_error_rad": float(
            np.max(
                np.abs(
                    current_joint_positions[arm_joint_indices]
                    - target_joint_positions[arm_joint_indices]
                )
            )
        ),
        "maximum_position_error_m": maximum_position_error,
        "maximum_orientation_error_deg": maximum_orientation_error,
    }
    # Isaac cierra sus salidas junto con la aplicación; vaciamos el búfer para
    # no perder la medición que decide si esta integración es compatible.
    print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
    return 0 if accepted else 1


if __name__ == "__main__":
    exit_code = main()
    simulation_app.close()
    raise SystemExit(exit_code)
