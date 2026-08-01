#!/usr/bin/env python3
"""Control de muñecas con el resolvedor Pink integrado por Isaac Lab.

Este controlador mantiene la locomoción WBC-AGILE en las piernas y usa Pink
sólo en los 14 movimientos de los brazos. La cintura queda fuera a propósito:
la policy recurrente actual fue entrenada con esa zona prácticamente fija, y
moverla ahora mezclaría dos cambios físicos en el mismo experimento.

Las poses con nombre siguen siendo objetivos temporales. La diferencia es que
Pink vuelve a calcular los ángulos desde la posición realmente medida de las
articulaciones, que es el contrato necesario para apuntar las muñecas a un
objeto medido más adelante.
"""

from pathlib import Path
import math

import numpy as np
import pinocchio as pin

from isaaclab.controllers.pink_ik.local_frame_task import LocalFrameTask
from isaaclab.controllers.pink_ik.null_space_posture_task import (
    NullSpacePostureTask,
)
from isaaclab.controllers.pink_ik.pink_ik import PinkIKController
from isaaclab.controllers.pink_ik.pink_ik_cfg import PinkIKControllerCfg

from arm_control import ARM_JOINTS, POSES


LEFT_WRIST_FRAME = "left_wrist_yaw_link"
RIGHT_WRIST_FRAME = "right_wrist_yaw_link"
BASE_FRAME = "pelvis"
WRIST_POSITION_TOLERANCE_M = 0.02
WRIST_ORIENTATION_TOLERANCE_DEG = 5.0


class PinkArmController:
    """Mantiene dos muñecas en poses relativas a la pelvis del robot."""

    def __init__(
        self,
        *,
        urdf_path: str,
        all_joint_names,
        arm_joint_indices,
        initial_joint_positions,
        robot_cfg,
        device: str,
    ):
        urdf = Path(urdf_path)
        if not urdf.is_file():
            raise FileNotFoundError(f"no existe el URDF para Pink: {urdf}")

        self.all_joint_names = list(all_joint_names)
        self.arm_joint_indices = list(arm_joint_indices)
        self.reference_joint_positions = np.asarray(
            initial_joint_positions, dtype=np.float64
        ).copy()
        if len(self.reference_joint_positions) != len(self.all_joint_names):
            raise ValueError(
                "la posición inicial y la lista de articulaciones no coinciden"
            )

        left_task = LocalFrameTask(
            LEFT_WRIST_FRAME,
            base_link_frame_name=BASE_FRAME,
            position_cost=8.0,
            orientation_cost=2.0,
            lm_damping=10,
            gain=0.5,
        )
        right_task = LocalFrameTask(
            RIGHT_WRIST_FRAME,
            base_link_frame_name=BASE_FRAME,
            position_cost=8.0,
            orientation_cost=2.0,
            lm_damping=10,
            gain=0.5,
        )
        posture_task = NullSpacePostureTask(
            cost=0.5,
            lm_damping=1,
            controlled_frames=[LEFT_WRIST_FRAME, RIGHT_WRIST_FRAME],
            controlled_joints=[
                name for name in ARM_JOINTS if "_shoulder_" in name
            ],
            gain=0.3,
        )
        cfg = PinkIKControllerCfg(
            urdf_path=str(urdf),
            mesh_path=str(urdf.parent),
            num_hand_joints=0,
            variable_input_tasks=[left_task, right_task, posture_task],
            fixed_input_tasks=[],
            joint_names=list(ARM_JOINTS),
            all_joint_names=self.all_joint_names,
            articulation_name="robot",
            base_link_name=BASE_FRAME,
            show_ik_warnings=True,
            fail_on_joint_limit_violation=True,
        )
        self.controller = PinkIKController(
            cfg=cfg,
            robot_cfg=robot_cfg,
            device=device,
            controlled_joint_indices=self.arm_joint_indices,
        )

        # Isaac Lab copia la configuración. Sólo estas instancias internas son
        # leídas por el resolvedor cuando calcula el siguiente movimiento.
        (
            self.left_task,
            self.right_task,
            self.posture_task,
        ) = self.controller.cfg.variable_input_tasks
        self.reset()

    def reset(self):
        """Vuelve a la referencia de arranque sin conservar otro objetivo."""
        self.current_joint_positions = self.reference_joint_positions.copy()
        self.actual = self.current_joint_positions[
            self.arm_joint_indices
        ].astype(np.float32)
        self.objetivo = POSES["reposo"].copy()
        self.pose_actual = "reposo"
        self._set_wrist_targets(self.objetivo)

    def set_pose(self, nombre: str):
        if nombre not in POSES:
            raise ValueError(
                f"pose desconocida: {nombre}. Opciones: {list(POSES)}"
            )
        self.objetivo = POSES[nombre].copy()
        self.pose_actual = nombre
        self._set_wrist_targets(self.objetivo)

    def set_joint_targets(self, angulos):
        """Convierte un objetivo articular en objetivos de ambas muñecas."""
        targets = np.asarray(angulos, dtype=np.float32)
        if targets.shape != (len(ARM_JOINTS),):
            raise ValueError(
                f"se esperaban {len(ARM_JOINTS)} ángulos de brazos"
            )
        self.objetivo = targets.copy()
        self.pose_actual = "manual"
        self._set_wrist_targets(self.objetivo)

    def _set_wrist_targets(self, arm_targets: np.ndarray):
        target_joint_positions = self.reference_joint_positions.copy()
        target_joint_positions[self.arm_joint_indices] = arm_targets
        target_in_pink_order = target_joint_positions[
            self.controller.isaac_lab_to_pink_ordering
        ]
        self.controller.pink_configuration.update(target_in_pink_order)
        self.left_task.set_target(
            self.controller.pink_configuration.get_transform(
                LEFT_WRIST_FRAME, BASE_FRAME
            )
        )
        self.right_task.set_target(
            self.controller.pink_configuration.get_transform(
                RIGHT_WRIST_FRAME, BASE_FRAME
            )
        )
        self.posture_task.set_target(
            self.controller.pink_configuration.q
        )

    def llego(self, tolerancia_rad: float = 0.02) -> bool:
        return bool(self.tracking_status()["reached"])

    def tracking_status(self, current_joint_positions=None) -> dict:
        """Mide las muñecas; Pink puede usar otros ángulos para la misma pose."""
        if current_joint_positions is not None:
            measured = np.asarray(current_joint_positions, dtype=np.float64)
            if measured.shape != (len(self.all_joint_names),):
                raise ValueError(
                    "Pink necesita la medición de todas las articulaciones"
                )
            self.current_joint_positions = measured.copy()
        self.controller.pink_configuration.update(
            self.current_joint_positions[
                self.controller.isaac_lab_to_pink_ordering
            ]
        )

        errors = {}
        for side, frame_name, task in (
            ("left", LEFT_WRIST_FRAME, self.left_task),
            ("right", RIGHT_WRIST_FRAME, self.right_task),
        ):
            current = self.controller.pink_configuration.get_transform(
                frame_name, BASE_FRAME
            )
            target = task.transform_target_to_base
            rotation_error = target.rotation.T @ current.rotation
            errors[side] = {
                "position_error_m": float(
                    np.linalg.norm(current.translation - target.translation)
                ),
                "orientation_error_deg": math.degrees(
                    float(np.linalg.norm(pin.log3(rotation_error)))
                ),
            }

        maximum_position_error = max(
            error["position_error_m"] for error in errors.values()
        )
        maximum_orientation_error = max(
            error["orientation_error_deg"] for error in errors.values()
        )
        return {
            "controller": "pink",
            "wrist_errors": errors,
            "maximum_wrist_position_error_m": round(
                maximum_position_error, 5
            ),
            "maximum_wrist_orientation_error_deg": round(
                maximum_orientation_error, 3
            ),
            "wrist_position_tolerance_m": WRIST_POSITION_TOLERANCE_M,
            "wrist_orientation_tolerance_deg": (
                WRIST_ORIENTATION_TOLERANCE_DEG
            ),
            "reached": (
                maximum_position_error <= WRIST_POSITION_TOLERANCE_M
                and maximum_orientation_error
                <= WRIST_ORIENTATION_TOLERANCE_DEG
            ),
        }

    def compute(self, dt: float, current_joint_positions=None) -> np.ndarray:
        """Calcula los próximos ángulos desde el estado realmente medido."""
        if current_joint_positions is not None:
            measured = np.asarray(current_joint_positions, dtype=np.float64)
            if measured.shape != (len(self.all_joint_names),):
                raise ValueError(
                    "Pink necesita la medición de todas las articulaciones"
                )
            self.current_joint_positions = measured.copy()
        result = self.controller.compute(self.current_joint_positions, dt)
        self.actual = result.detach().cpu().numpy().astype(np.float32)
        return self.actual.copy()
