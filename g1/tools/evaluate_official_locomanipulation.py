#!/usr/bin/env python3
"""Mide sin modificar la tarea oficial de locomoción y manipulación del G1.

La configuración, el cuerpo y la policy se cargan directamente desde la
versión instalada de Isaac Lab. Este archivo construye la orden neutral desde
la postura real de inicio y observa el resultado; no reemplaza ni ajusta
ninguna pieza oficial.
"""

import argparse
import importlib
import inspect
import json
import math
from pathlib import Path

from isaaclab.app import AppLauncher


DEFAULT_TASK = "Isaac-PickPlace-Locomanipulation-G1-Abs-v0"
OFFICIAL_TASK_PACKAGE = (
    "isaaclab_tasks.manager_based.locomanipulation.pick_place"
)


parser = argparse.ArgumentParser(
    description=(
        "Evalúa quietud, caminata y frenado en la tarea oficial del G1 "
        "sin cambiar su configuración."
    )
)
parser.add_argument("--task", default=DEFAULT_TASK)
parser.add_argument("--duration-s", type=float, default=10.0)
parser.add_argument("--repetitions", type=int, default=3)
parser.add_argument("--output", type=Path)
parser.add_argument("--expected-isaaclab-root", type=Path)
parser.add_argument("--maximum-drift-m", type=float, default=0.10)
parser.add_argument("--minimum-height-m", type=float, default=0.65)
parser.add_argument("--maximum-tilt-deg", type=float, default=20.0)
parser.add_argument("--standing-hip-height-m", type=float, default=0.72)
parser.add_argument("--run-walk-stop", action="store_true")
parser.add_argument("--walk-command", type=float, default=0.30)
parser.add_argument("--walk-duration-s", type=float, default=4.0)
parser.add_argument("--stop-duration-s", type=float, default=4.0)
parser.add_argument("--minimum-walk-progress-m", type=float, default=0.50)
parser.add_argument("--maximum-lateral-error-m", type=float, default=0.30)
parser.add_argument("--maximum-braking-drift-m", type=float, default=0.15)
parser.add_argument("--maximum-final-speed-mps", type=float, default=0.12)
parser.add_argument(
    "--enable_pinocchio",
    action="store_true",
    help="Carga Pinocchio antes de Isaac, como exige el flujo oficial.",
)
parser.add_argument(
    "--disable-fabric",
    action="store_true",
    help="Usa lecturas USD en lugar de la ruta rápida de Isaac.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# NVIDIA importa Pinocchio antes de AppLauncher para que Isaac no elija su
# copia incompatible. Hacerlo aquí también convierte una instalación rota en
# un error temprano, en lugar de dejar Isaac congelado durante horas.
if args_cli.enable_pinocchio:
    import numpy

    if int(numpy.__version__.split(".", maxsplit=1)[0]) >= 2:
        raise RuntimeError(
            "Isaac Lab 2.3.2 exige NumPy < 2 para Pinocchio; "
            f"se encontró {numpy.__version__} en {numpy.__file__}"
        )
    import pinocchio  # noqa: F401

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
print("[referencia] Isaac inició; cargando la tarea oficial.", flush=True)


import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import isaaclab_tasks  # noqa: E402, F401
import isaaclab  # noqa: E402
import isaaclab.utils.math as math_utils  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402


def root_tilt_deg(quaternion_wxyz: torch.Tensor) -> float:
    """Devuelve cuánto se aparta el eje vertical del cuerpo de la vertical."""
    _, x, y, _ = [float(value) for value in quaternion_wxyz]
    vertical_component = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
    return math.degrees(math.acos(vertical_component))


def tensor_flag(value) -> bool:
    """Normaliza las banderas escalares o vectoriales que devuelve Gym."""
    if hasattr(value, "any"):
        return bool(value.any().item())
    return bool(value)


def yaw_rad(quaternion_wxyz: torch.Tensor) -> float:
    """Devuelve la orientación horizontal del cuerpo en radianes."""
    w, x, y, z = [float(value) for value in quaternion_wxyz]
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def displacement_in_initial_body(
    position: torch.Tensor,
    initial_position: torch.Tensor,
    initial_yaw: float,
) -> tuple[float, float]:
    """Separa avance y costado respecto de cómo arrancó el robot."""
    delta_x = float(position[0] - initial_position[0])
    delta_y = float(position[1] - initial_position[1])
    cosine = math.cos(initial_yaw)
    sine = math.sin(initial_yaw)
    return (
        cosine * delta_x + sine * delta_y,
        -sine * delta_x + cosine * delta_y,
    )


def capture_upper_body_reference(env) -> dict:
    """Guarda muñecas respecto de la pelvis para que acompañen al caminar."""
    unwrapped = env.unwrapped
    manager = unwrapped.action_manager
    robot = unwrapped.scene["robot"]
    term = manager.get_term("upper_body_ik")
    target_links = list(term.cfg.target_eef_link_names.values())
    expected_pose_dim = len(target_links) * term.pose_dim
    if expected_pose_dim + term.hand_joint_dim != term.action_dim:
        raise RuntimeError(
            "el contrato oficial de brazos cambió: "
            f"poses={expected_pose_dim}, dedos={term.hand_joint_dim}, "
            f"dimensión={term.action_dim}"
        )

    base_pose = robot.data.body_link_state_w[:, term._base_link_idx, :7].clone()
    base_pose[:, :3] -= unwrapped.scene.env_origins
    wrist_poses_in_base = []
    for link_name in target_links:
        body_ids, _ = robot.find_bodies(link_name)
        if len(body_ids) != 1:
            raise RuntimeError(
                f"se esperaba un único cuerpo para {link_name}: {body_ids}"
            )
        wrist_pose = robot.data.body_link_state_w[:, body_ids[0], :7].clone()
        wrist_pose[:, :3] -= unwrapped.scene.env_origins
        relative_position, relative_orientation = (
            math_utils.subtract_frame_transforms(
                base_pose[:, :3],
                base_pose[:, 3:7],
                wrist_pose[:, :3],
                wrist_pose[:, 3:7],
            )
        )
        wrist_poses_in_base.append(
            (relative_position.clone(), relative_orientation.clone())
        )

    return {
        "target_links": target_links,
        "wrist_poses_in_base": wrist_poses_in_base,
        "hand_positions": robot.data.joint_pos[
            :, term._hand_joint_ids
        ].clone(),
    }


def build_action(
    env,
    upper_body_reference: dict,
    lower_body_command: tuple[float, float, float, float],
) -> tuple[torch.Tensor, dict]:
    """Arma una orden válida y conserva brazos respecto de la pelvis."""
    unwrapped = env.unwrapped
    manager = unwrapped.action_manager
    robot = unwrapped.scene["robot"]
    actions = torch.zeros(
        (unwrapped.num_envs, manager.total_action_dim),
        dtype=torch.float32,
        device=unwrapped.device,
    )
    metadata = {
        "term_names": list(manager.active_terms),
        "term_dimensions": list(manager.action_term_dim),
        "standing_hip_height_m": lower_body_command[3],
    }

    offset = 0
    for term_name, term_dim in zip(
        manager.active_terms,
        manager.action_term_dim,
        strict=True,
    ):
        term = manager.get_term(term_name)
        if term_name == "upper_body_ik":
            target_links = upper_body_reference["target_links"]
            expected_pose_dim = len(target_links) * term.pose_dim
            base_pose = robot.data.body_link_state_w[
                :, term._base_link_idx, :7
            ].clone()
            base_pose[:, :3] -= unwrapped.scene.env_origins
            for pose_index, relative_pose in enumerate(
                upper_body_reference["wrist_poses_in_base"]
            ):
                position, orientation = math_utils.combine_frame_transforms(
                    base_pose[:, :3],
                    base_pose[:, 3:7],
                    relative_pose[0],
                    relative_pose[1],
                )
                pose_start = offset + pose_index * term.pose_dim
                actions[:, pose_start : pose_start + 3] = position
                actions[:, pose_start + 3 : pose_start + term.pose_dim] = (
                    orientation
                )

            hand_start = offset + expected_pose_dim
            actions[:, hand_start : hand_start + term.hand_joint_dim] = (
                upper_body_reference["hand_positions"]
            )
            metadata["upper_body_target_links"] = target_links
            metadata["hand_joint_count"] = term.hand_joint_dim
        elif term_name == "lower_body_joint_pos":
            if term_dim != 4:
                raise RuntimeError(
                    "el contrato oficial de piernas dejó de ser "
                    f"[vx, vy, giro, altura]: dimensión={term_dim}"
                )
            actions[:, offset : offset + term_dim] = torch.tensor(
                lower_body_command,
                dtype=torch.float32,
                device=unwrapped.device,
            )
        else:
            raise RuntimeError(
                f"apareció un término de acción oficial no auditado: {term_name}"
            )
        offset += term_dim

    return actions, metadata


def evaluate_trial(env, duration_s: float, repetition: int) -> dict:
    """Ejecuta una repetición neutral y conserva sus métricas."""
    env.reset()
    robot = env.unwrapped.scene["robot"]
    step_dt = float(env.unwrapped.step_dt)
    steps = max(1, math.ceil(duration_s / step_dt))
    upper_body_reference = capture_upper_body_reference(env)
    neutral_command = (0.0, 0.0, 0.0, args_cli.standing_hip_height_m)
    actions, action_metadata = build_action(
        env,
        upper_body_reference,
        neutral_command,
    )

    initial_position = robot.data.root_pos_w[0].detach().cpu().clone()
    minimum_height = float(initial_position[2])
    maximum_height = float(initial_position[2])
    maximum_displacement = 0.0
    maximum_tilt_deg = 0.0
    termination_step = None
    truncation_step = None

    for step in range(steps):
        actions, _ = build_action(
            env,
            upper_body_reference,
            neutral_command,
        )
        _, _, terminated, truncated, _ = env.step(actions)
        position = robot.data.root_pos_w[0].detach().cpu()
        orientation = robot.data.root_quat_w[0].detach().cpu()
        displacement = float(
            torch.linalg.vector_norm(position[:2] - initial_position[:2])
        )
        minimum_height = min(minimum_height, float(position[2]))
        maximum_height = max(maximum_height, float(position[2]))
        maximum_displacement = max(maximum_displacement, displacement)
        maximum_tilt_deg = max(
            maximum_tilt_deg,
            root_tilt_deg(orientation),
        )
        if tensor_flag(terminated):
            termination_step = step + 1
            break
        if tensor_flag(truncated):
            truncation_step = step + 1
            break

    final_position = robot.data.root_pos_w[0].detach().cpu()
    final_displacement = float(
        torch.linalg.vector_norm(final_position[:2] - initial_position[:2])
    )
    return {
        "repetition": repetition,
        "requested_duration_s": duration_s,
        "measured_duration_s": round((step + 1) * step_dt, 4),
        "steps": step + 1,
        "initial_position_m": [
            round(float(value), 6) for value in initial_position
        ],
        "final_position_m": [
            round(float(value), 6) for value in final_position
        ],
        "final_displacement_m": round(final_displacement, 6),
        "maximum_displacement_m": round(maximum_displacement, 6),
        "minimum_height_m": round(minimum_height, 6),
        "maximum_height_m": round(maximum_height, 6),
        "maximum_tilt_deg": round(maximum_tilt_deg, 4),
        "terminated_early": termination_step is not None,
        "termination_step": termination_step,
        "truncated_early": truncation_step is not None,
        "truncation_step": truncation_step,
        "neutral_action": [round(float(value), 6) for value in actions[0]],
        "action_contract": action_metadata,
    }


def evaluate_walk_stop_trial(env, repetition: int) -> dict:
    """Mide caminata y frenado manteniendo intacta la referencia oficial."""
    env.reset()
    robot = env.unwrapped.scene["robot"]
    step_dt = float(env.unwrapped.step_dt)
    walk_steps = max(1, math.ceil(args_cli.walk_duration_s / step_dt))
    stop_steps = max(1, math.ceil(args_cli.stop_duration_s / step_dt))
    upper_body_reference = capture_upper_body_reference(env)
    walk_command = (
        args_cli.walk_command,
        0.0,
        0.0,
        args_cli.standing_hip_height_m,
    )
    stop_command = (0.0, 0.0, 0.0, args_cli.standing_hip_height_m)

    initial_position = robot.data.root_pos_w[0].detach().cpu().clone()
    initial_orientation = robot.data.root_quat_w[0].detach().cpu().clone()
    initial_yaw = yaw_rad(initial_orientation)
    minimum_height = float(initial_position[2])
    maximum_tilt = root_tilt_deg(initial_orientation)
    maximum_abs_lateral = 0.0
    termination_step = None
    truncation_step = None
    executed_steps = 0
    action_metadata = None

    for step in range(walk_steps):
        actions, action_metadata = build_action(
            env,
            upper_body_reference,
            walk_command,
        )
        _, _, terminated, truncated, _ = env.step(actions)
        executed_steps += 1
        position = robot.data.root_pos_w[0].detach().cpu()
        orientation = robot.data.root_quat_w[0].detach().cpu()
        _, lateral = displacement_in_initial_body(
            position,
            initial_position,
            initial_yaw,
        )
        maximum_abs_lateral = max(maximum_abs_lateral, abs(lateral))
        minimum_height = min(minimum_height, float(position[2]))
        maximum_tilt = max(maximum_tilt, root_tilt_deg(orientation))
        if tensor_flag(terminated):
            termination_step = step + 1
            break
        if tensor_flag(truncated):
            truncation_step = step + 1
            break

    walk_end_position = robot.data.root_pos_w[0].detach().cpu().clone()
    walk_end_orientation = robot.data.root_quat_w[0].detach().cpu().clone()
    walk_forward, walk_lateral = displacement_in_initial_body(
        walk_end_position,
        initial_position,
        initial_yaw,
    )

    if termination_step is None and truncation_step is None:
        for step in range(stop_steps):
            actions, action_metadata = build_action(
                env,
                upper_body_reference,
                stop_command,
            )
            _, _, terminated, truncated, _ = env.step(actions)
            executed_steps += 1
            position = robot.data.root_pos_w[0].detach().cpu()
            orientation = robot.data.root_quat_w[0].detach().cpu()
            _, lateral = displacement_in_initial_body(
                position,
                initial_position,
                initial_yaw,
            )
            maximum_abs_lateral = max(maximum_abs_lateral, abs(lateral))
            minimum_height = min(minimum_height, float(position[2]))
            maximum_tilt = max(maximum_tilt, root_tilt_deg(orientation))
            if tensor_flag(terminated):
                termination_step = walk_steps + step + 1
                break
            if tensor_flag(truncated):
                truncation_step = walk_steps + step + 1
                break

    final_position = robot.data.root_pos_w[0].detach().cpu().clone()
    final_forward, final_lateral = displacement_in_initial_body(
        final_position,
        initial_position,
        initial_yaw,
    )
    final_planar_speed = float(
        torch.linalg.vector_norm(robot.data.root_lin_vel_b[0, :2])
    )
    braking_drift = math.hypot(
        final_forward - walk_forward,
        final_lateral - walk_lateral,
    )
    accepted = (
        termination_step is None
        and truncation_step is None
        and walk_forward >= args_cli.minimum_walk_progress_m
        and maximum_abs_lateral <= args_cli.maximum_lateral_error_m
        and braking_drift <= args_cli.maximum_braking_drift_m
        and final_planar_speed <= args_cli.maximum_final_speed_mps
        and minimum_height >= args_cli.minimum_height_m
        and maximum_tilt <= args_cli.maximum_tilt_deg
    )
    return {
        "repetition": repetition,
        "walk_duration_s": args_cli.walk_duration_s,
        "stop_duration_s": args_cli.stop_duration_s,
        "measured_duration_s": round(executed_steps * step_dt, 4),
        "commanded_forward_input": args_cli.walk_command,
        "walk_forward_progress_m": round(walk_forward, 6),
        "walk_lateral_displacement_m": round(walk_lateral, 6),
        "walk_yaw_change_deg": round(
            math.degrees(yaw_rad(walk_end_orientation) - initial_yaw),
            4,
        ),
        "braking_drift_m": round(braking_drift, 6),
        "final_forward_progress_m": round(final_forward, 6),
        "final_lateral_displacement_m": round(final_lateral, 6),
        "maximum_abs_lateral_m": round(maximum_abs_lateral, 6),
        "minimum_height_m": round(minimum_height, 6),
        "maximum_tilt_deg": round(maximum_tilt, 4),
        "final_planar_speed_mps": round(final_planar_speed, 6),
        "terminated_early": termination_step is not None,
        "termination_step": termination_step,
        "truncated_early": truncation_step is not None,
        "truncation_step": truncation_step,
        "initial_position_m": [
            round(float(value), 6) for value in initial_position
        ],
        "walk_end_position_m": [
            round(float(value), 6) for value in walk_end_position
        ],
        "final_position_m": [
            round(float(value), 6) for value in final_position
        ],
        "action_contract": action_metadata,
        "accepted": accepted,
    }


def main():
    if args_cli.duration_s <= 0:
        raise ValueError("--duration-s debe ser mayor que cero")
    if args_cli.repetitions <= 0:
        raise ValueError("--repetitions debe ser mayor que cero")
    if args_cli.maximum_drift_m < 0:
        raise ValueError("--maximum-drift-m no puede ser negativo")
    if args_cli.minimum_height_m <= 0:
        raise ValueError("--minimum-height-m debe ser mayor que cero")
    if not 0.4 <= args_cli.standing_hip_height_m <= 1.0:
        raise ValueError(
            "--standing-hip-height-m debe estar entre 0.4 y 1.0"
        )
    if args_cli.walk_command <= 0:
        raise ValueError("--walk-command debe ser mayor que cero")
    if args_cli.walk_duration_s <= 0 or args_cli.stop_duration_s <= 0:
        raise ValueError(
            "--walk-duration-s y --stop-duration-s deben ser mayores que cero"
        )
    if args_cli.minimum_walk_progress_m <= 0:
        raise ValueError("--minimum-walk-progress-m debe ser mayor que cero")
    if (
        args_cli.maximum_lateral_error_m < 0
        or args_cli.maximum_braking_drift_m < 0
        or args_cli.maximum_final_speed_mps < 0
    ):
        raise ValueError("las tolerancias de caminata no pueden ser negativas")
    if not 0 < args_cli.maximum_tilt_deg <= 90:
        raise ValueError("--maximum-tilt-deg debe estar entre 0 y 90")

    isaaclab_source = Path(inspect.getfile(isaaclab)).resolve()
    isaaclab_tasks_source = Path(inspect.getfile(isaaclab_tasks)).resolve()
    if args_cli.expected_isaaclab_root is not None:
        expected_root = args_cli.expected_isaaclab_root.resolve()
        if (
            not isaaclab_source.is_relative_to(expected_root)
            or not isaaclab_tasks_source.is_relative_to(expected_root)
        ):
            raise RuntimeError(
                "la prueba mezcló versiones de Isaac Lab: "
                f"isaaclab={isaaclab_source}, "
                f"isaaclab_tasks={isaaclab_tasks_source}, "
                f"esperado={expected_root}"
            )

    if not args_cli.enable_pinocchio:
        raise RuntimeError(
            "esta referencia oficial requiere --enable_pinocchio"
        )

    # El flujo oficial registra estas tareas sólo después de cargar Pinocchio.
    # Importar el paquete no cambia su cuerpo, controladores ni configuración.
    importlib.import_module(OFFICIAL_TASK_PACKAGE)
    if args_cli.task not in gym.registry:
        available = sorted(
            task_id
            for task_id in gym.registry
            if "Locomanipulation-G1" in task_id
        )
        raise RuntimeError(
            f"la tarea oficial {args_cli.task} no quedó registrada; "
            f"disponibles={available}"
        )

    print(
        f"[referencia] preparando {args_cli.task} sin modificaciones.",
        flush=True,
    )
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=1,
        use_fabric=not args_cli.disable_fabric,
    )
    env = gym.make(args_cli.task, cfg=env_cfg)
    print("[referencia] entorno listo; comenzando las mediciones.", flush=True)
    try:
        robot = env.unwrapped.scene["robot"]
        hand_joint_names = [
            name
            for name in robot.joint_names
            if "index" in name or "middle" in name or "thumb" in name
        ]
        trials = []
        for repetition in range(1, args_cli.repetitions + 1):
            print(
                f"[referencia] repetición {repetition}/"
                f"{args_cli.repetitions}.",
                flush=True,
            )
            trials.append(
                evaluate_trial(env, args_cli.duration_s, repetition)
            )
        walk_stop_trials = []
        if args_cli.run_walk_stop:
            for repetition in range(1, args_cli.repetitions + 1):
                print(
                    f"[referencia] caminata y frenado {repetition}/"
                    f"{args_cli.repetitions}.",
                    flush=True,
                )
                walk_stop_trials.append(
                    evaluate_walk_stop_trial(env, repetition)
                )
        stand_accepted = all(
            not trial["terminated_early"]
            and not trial["truncated_early"]
            and trial["maximum_displacement_m"]
            <= args_cli.maximum_drift_m
            and trial["minimum_height_m"] >= args_cli.minimum_height_m
            and trial["maximum_tilt_deg"] <= args_cli.maximum_tilt_deg
            for trial in trials
        )
        walk_stop_accepted = all(
            trial["accepted"] for trial in walk_stop_trials
        )
        report = {
            "reference": "Isaac Lab sin modificaciones",
            "task": args_cli.task,
            "isaaclab_source": str(isaaclab_source),
            "isaaclab_tasks_source": str(isaaclab_tasks_source),
            "device": str(env.unwrapped.device),
            "simulation_step_s": float(env.unwrapped.physics_dt),
            "environment_step_s": float(env.unwrapped.step_dt),
            "action_shape": list(env.action_space.shape),
            "joint_count": int(robot.num_joints),
            "hand_joint_count": len(hand_joint_names),
            "hand_joint_names": hand_joint_names,
            "acceptance_criteria": {
                "stand": {
                    "maximum_displacement_m": args_cli.maximum_drift_m,
                    "minimum_height_m": args_cli.minimum_height_m,
                    "maximum_tilt_deg": args_cli.maximum_tilt_deg,
                    "early_termination_allowed": False,
                },
                "walk_stop": {
                    "enabled": args_cli.run_walk_stop,
                    "minimum_forward_progress_m": (
                        args_cli.minimum_walk_progress_m
                    ),
                    "maximum_lateral_error_m": (
                        args_cli.maximum_lateral_error_m
                    ),
                    "maximum_braking_drift_m": (
                        args_cli.maximum_braking_drift_m
                    ),
                    "maximum_final_speed_mps": (
                        args_cli.maximum_final_speed_mps
                    ),
                    "minimum_height_m": args_cli.minimum_height_m,
                    "maximum_tilt_deg": args_cli.maximum_tilt_deg,
                    "early_termination_allowed": False,
                },
            },
            "stand_trials": trials,
            "walk_stop_trials": walk_stop_trials,
            "stand_accepted": stand_accepted,
            "walk_stop_accepted": (
                walk_stop_accepted if args_cli.run_walk_stop else None
            ),
            "accepted": stand_accepted and walk_stop_accepted,
        }
        serialized = json.dumps(report, ensure_ascii=False, indent=2)
        print(serialized, flush=True)
        if args_cli.output is not None:
            args_cli.output.parent.mkdir(parents=True, exist_ok=True)
            args_cli.output.write_text(serialized + "\n", encoding="utf-8")
    finally:
        env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
