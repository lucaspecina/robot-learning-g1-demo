#!/usr/bin/env python3
"""Compara nuestro adaptador de AGILE contra el adaptador oficial de NVIDIA."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compara entradas, salidas y articulaciones de AGILE."
    )
    parser.add_argument("--agile-root", type=Path, required=True)
    parser.add_argument("--descriptor", type=Path, required=True)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def maximum_error(first, second) -> float:
    return float(
        np.max(
            np.abs(
                np.asarray(first, dtype=np.float32)
                - np.asarray(second, dtype=np.float32)
            )
        )
    )


def main() -> None:
    args = parse_args()
    g1_root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(g1_root))
    sys.path.insert(0, str(args.agile_root))

    from locomotion import (  # pylint: disable=import-outside-toplevel
        LocomotionState,
        WbcAgileLocomotion,
        gravity_in_body_frame,
    )
    from agile.sim2mujoco.actions import (  # pylint: disable=import-outside-toplevel
        ActionProcessor,
    )
    from agile.sim2mujoco.commands import (  # pylint: disable=import-outside-toplevel
        CommandManager,
    )
    from agile.sim2mujoco.observations import (  # pylint: disable=import-outside-toplevel
        ObservationProcessor,
    )
    from agile.sim2mujoco.simulation import (  # pylint: disable=import-outside-toplevel
        SimState,
    )

    descriptor = yaml.safe_load(args.descriptor.read_text(encoding="utf-8"))
    device = torch.device("cpu")
    full_joint_names = descriptor["articulations"]["robot"]["joint_names"]
    default_joint_positions = np.asarray(
        descriptor["articulations"]["robot"]["default_joint_pos"],
        dtype=np.float32,
    )

    official_commands = CommandManager(
        device=device,
        defaults={
            "linear_x": 0.0,
            "linear_y": 0.0,
            "angular_z": 0.0,
            "height": 0.72,
        },
    )
    official_observations = ObservationProcessor(
        descriptor,
        full_joint_names,
        device,
        command_manager=official_commands,
    )
    official_actions = ActionProcessor(descriptor, full_joint_names, device)
    ours = WbcAgileLocomotion(
        args.policy,
        args.descriptor,
        pelvis_height=0.72,
        device="cpu",
    )
    official_policy = torch.jit.load(str(args.policy), map_location="cpu")
    official_policy.eval()

    observation_indices = [
        full_joint_names.index(name) for name in ours.observation_joint_names
    ]
    action_indices = [
        full_joint_names.index(name) for name in ours.action_joint_names
    ]

    rng = np.random.default_rng(20260729)
    observation_errors: list[float] = []
    policy_errors: list[float] = []
    target_errors: list[float] = []

    ours.reset()
    if hasattr(official_policy, "reset_flat"):
        official_policy.reset_flat()
    official_observations.reset()

    for case_index in range(8):
        command = rng.uniform(
            low=np.array([-0.5, -0.5, -1.0], dtype=np.float32),
            high=np.array([0.5, 0.5, 1.0], dtype=np.float32),
        ).astype(np.float32)
        height = float(rng.uniform(0.4, 0.72))
        official_commands.set_command(*command, height=height)
        ours.pelvis_height = height

        joint_positions = default_joint_positions + rng.uniform(
            -0.2, 0.2, len(full_joint_names)
        ).astype(np.float32)
        joint_velocities = rng.uniform(
            -2.0, 2.0, len(full_joint_names)
        ).astype(np.float32)
        root_angular_velocity = rng.uniform(-1.0, 1.0, 3).astype(np.float32)
        quaternion = rng.normal(size=4).astype(np.float32)
        quaternion /= np.linalg.norm(quaternion)

        # El primer caso fuerza la diferencia entre el límite de memoria
        # (±10) y el límite enviado al motor (±6).
        if case_index == 0:
            previous_action = np.linspace(-9.0, 9.0, 12, dtype=np.float32)
        else:
            previous_action = rng.uniform(-12.0, 12.0, 12).astype(np.float32)

        official_observations.set_last_action(
            torch.from_numpy(previous_action)
        )
        ours.last_action = np.clip(
            previous_action,
            ours.last_action_clip[0],
            ours.last_action_clip[1],
        )

        official_state = SimState(
            joint_pos=torch.from_numpy(joint_positions),
            joint_vel=torch.from_numpy(joint_velocities),
            root_pos=torch.zeros(3),
            root_quat=torch.from_numpy(quaternion),
            root_lin_vel=torch.zeros(3),
            root_ang_vel=torch.from_numpy(root_angular_velocity),
        )
        official_observation = (
            official_observations.compute(official_state).cpu().numpy()
        )
        our_state = LocomotionState(
            joint_pos=joint_positions[observation_indices],
            joint_vel=joint_velocities[observation_indices],
            ang_vel=root_angular_velocity,
            gravity=gravity_in_body_frame(*quaternion),
        )
        our_observation = ours.build_observation(our_state, command)
        observation_errors.append(
            maximum_error(official_observation, our_observation)
        )

        with torch.no_grad():
            official_raw_action = (
                official_policy(torch.from_numpy(official_observation))
                .cpu()
                .numpy()
                .reshape(-1)
            )
            our_raw_action = (
                ours.policy(torch.from_numpy(our_observation))
                .cpu()
                .numpy()
                .reshape(-1)
            )
        policy_errors.append(maximum_error(official_raw_action, our_raw_action))

        official_target = (
            official_actions.process(torch.from_numpy(official_raw_action))
            .position[action_indices]
            .cpu()
            .numpy()
        )
        our_target = ours.process_action(our_raw_action)
        target_errors.append(maximum_error(official_target, our_target))
        official_observations.set_last_action(
            torch.from_numpy(official_raw_action)
        )

    report = {
        "cases": len(observation_errors),
        "observation_dimension": official_observations.total_obs_dim,
        "action_dimension": official_actions.total_action_dim,
        "full_joint_count": len(full_joint_names),
        "observation_joint_count": len(ours.observation_joint_names),
        "action_joint_count": len(ours.action_joint_names),
        "observation_joint_names_match": (
            ours.observation_joint_names
            == descriptor["observations"]["policy"][3]["joint_names"]
        ),
        "action_joint_names_match": (
            ours.action_joint_names == descriptor["actions"][0]["joint_names"]
        ),
        "maximum_observation_error": max(observation_errors),
        "maximum_policy_output_error": max(policy_errors),
        "maximum_joint_target_error": max(target_errors),
        "passed": (
            max(observation_errors) <= 1e-6
            and max(policy_errors) <= 1e-6
            and max(target_errors) <= 1e-6
        ),
    }
    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
