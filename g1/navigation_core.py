#!/usr/bin/env python3
"""Cálculo de navegación simple y verificación de progreso, sin depender de ROS."""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class NavigationGoal:
    x: float
    y: float
    yaw: Optional[float]


@dataclass(frozen=True)
class NavigationPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class NavigationCommand:
    linear_x: float
    angular_z: float
    distance_remaining: float
    heading_error: float
    goal_reached: bool
    phase: str


@dataclass(frozen=True)
class SpinCommand:
    angular_z: float
    angular_distance_traveled: float
    angle_remaining: float
    goal_reached: bool
    phase: str


def normalize_angle(angle: float) -> float:
    """Lleva un ángulo a [-pi, pi] para elegir siempre el giro más corto."""
    return math.atan2(math.sin(angle), math.cos(angle))


class SpinController:
    """Controla un giro relativo sin perder vueltas al cruzar ±pi.

    Sigue el criterio de la Action Spin oficial de Nav2: acumula el cambio
    entre mediciones consecutivas, frena según el ángulo restante y conserva
    el signo pedido. La tolerancia no es la microscópica del robot con ruedas:
    cinco grados es el valor ya medido por nuestro navegador para este bípedo.
    """

    def __init__(
        self,
        target_yaw: float,
        tolerance_rad: float = math.radians(5.0),
        minimum_velocity: float = 0.10,
        maximum_velocity: float = 0.50,
        acceleration_limit: float = 1.0,
    ):
        if not math.isfinite(target_yaw):
            raise ValueError("el ángulo pedido debe ser finito")
        if tolerance_rad <= 0.0:
            raise ValueError("la tolerancia angular debe ser positiva")
        if minimum_velocity <= 0.0:
            raise ValueError("la velocidad angular mínima debe ser positiva")
        if maximum_velocity < minimum_velocity:
            raise ValueError(
                "la velocidad angular máxima no puede ser menor que la mínima"
            )
        if acceleration_limit <= 0.0:
            raise ValueError("la aceleración angular debe ser positiva")
        self.target_yaw = float(target_yaw)
        self.tolerance_rad = float(tolerance_rad)
        self.minimum_velocity = float(minimum_velocity)
        self.maximum_velocity = float(maximum_velocity)
        self.acceleration_limit = float(acceleration_limit)
        self.previous_yaw: Optional[float] = None
        self.angular_distance_traveled = 0.0

    def reset(self, initial_yaw: Optional[float] = None):
        self.previous_yaw = initial_yaw
        self.angular_distance_traveled = 0.0

    def step(self, current_yaw: float) -> SpinCommand:
        if self.previous_yaw is None:
            self.previous_yaw = float(current_yaw)
        else:
            self.angular_distance_traveled += normalize_angle(
                float(current_yaw) - self.previous_yaw
            )
            self.previous_yaw = float(current_yaw)

        remaining = max(
            0.0,
            abs(self.target_yaw)
            - abs(self.angular_distance_traveled),
        )
        if remaining <= self.tolerance_rad:
            return SpinCommand(
                angular_z=0.0,
                angular_distance_traveled=self.angular_distance_traveled,
                angle_remaining=remaining,
                goal_reached=True,
                phase="done",
            )

        speed = math.sqrt(2.0 * self.acceleration_limit * remaining)
        speed = min(
            self.maximum_velocity,
            max(self.minimum_velocity, speed),
        )
        return SpinCommand(
            angular_z=math.copysign(speed, self.target_yaw),
            angular_distance_traveled=self.angular_distance_traveled,
            angle_remaining=remaining,
            goal_reached=False,
            phase="spinning",
        )


class NavigationController:
    """Convierte pose y objetivo en una orden pequeña de avance y giro."""

    def __init__(
        self,
        tolerance_m: float = 0.10,
        # El valor oficial por defecto de Nav2 es cero. Los 10 cm anteriores
        # permitieron que una vuelta a home terminara a 18,8 cm mientras el
        # bípedo giraba; la histéresis no puede duplicar el contrato declarado.
        position_latch_buffer_m: float = 0.0,
        final_yaw_tolerance_rad: float = math.radians(5.0),
        heading_tolerance_rad: float = 0.25,
        linear_velocity: float = 0.30,
        minimum_approach_velocity: float = 0.10,
        angular_velocity: float = 0.50,
        final_yaw_gain: float = 1.50,
        minimum_final_angular_velocity: float = 0.10,
    ):
        self.tolerance_m = tolerance_m
        self.position_latch_buffer_m = position_latch_buffer_m
        self.final_yaw_tolerance_rad = final_yaw_tolerance_rad
        self.heading_tolerance_rad = heading_tolerance_rad
        self.linear_velocity = linear_velocity
        self.minimum_approach_velocity = minimum_approach_velocity
        self.angular_velocity = angular_velocity
        self.final_yaw_gain = final_yaw_gain
        self.minimum_final_angular_velocity = minimum_final_angular_velocity
        self.position_reached = False

    def reset(self):
        self.position_reached = False

    def step(
        self,
        pose: NavigationPose,
        goal: NavigationGoal,
    ) -> NavigationCommand:
        dx = goal.x - pose.x
        dy = goal.y - pose.y
        distance = math.hypot(dx, dy)

        if distance < self.tolerance_m:
            self.position_reached = True
        elif (
            self.position_reached
            and distance > self.tolerance_m + self.position_latch_buffer_m
        ):
            self.position_reached = False

        if self.position_reached:
            final_error = 0.0
            if goal.yaw is not None:
                final_error = normalize_angle(goal.yaw - pose.yaw)
                if abs(final_error) > self.final_yaw_tolerance_rad:
                    speed = max(
                        self.minimum_final_angular_velocity,
                        min(
                            self.angular_velocity,
                            self.final_yaw_gain * abs(final_error),
                        ),
                    )
                    return NavigationCommand(
                        linear_x=0.0,
                        angular_z=math.copysign(speed, final_error),
                        distance_remaining=distance,
                        heading_error=final_error,
                        goal_reached=False,
                        phase="final_turn",
                    )
            return NavigationCommand(
                linear_x=0.0,
                angular_z=0.0,
                distance_remaining=distance,
                heading_error=final_error,
                goal_reached=True,
                phase="done",
            )

        heading_error = normalize_angle(math.atan2(dy, dx) - pose.yaw)
        if abs(heading_error) > self.heading_tolerance_rad:
            return NavigationCommand(
                linear_x=0.0,
                angular_z=math.copysign(
                    self.angular_velocity,
                    heading_error,
                ),
                distance_remaining=distance,
                heading_error=heading_error,
                goal_reached=False,
                phase="turning",
            )

        return NavigationCommand(
            linear_x=max(
                self.minimum_approach_velocity,
                self.linear_velocity * min(1.0, distance),
            ),
            angular_z=heading_error,
            distance_remaining=distance,
            heading_error=heading_error,
            goal_reached=False,
            phase="moving",
        )


class ProgressChecker:
    """Exige movimiento lineal o angular dentro de una ventana de tiempo.

    Contar también el giro evita declarar bloqueo mientras el robot todavía se
    está orientando. El criterio replica la idea del verificador simple de
    Nav2, pero usa valores iniciales conservadores para este bípedo.
    """

    def __init__(
        self,
        movement_radius_m: float,
        movement_angle_rad: float,
        allowance_s: float,
    ):
        if movement_radius_m <= 0.0:
            raise ValueError("el movimiento lineal requerido debe ser positivo")
        if movement_angle_rad <= 0.0:
            raise ValueError("el giro requerido debe ser positivo")
        if allowance_s <= 0.0:
            raise ValueError("la ventana de progreso debe ser positiva")
        self.movement_radius_m = movement_radius_m
        self.movement_angle_rad = movement_angle_rad
        self.allowance_s = allowance_s
        self.anchor_pose: Optional[NavigationPose] = None
        self.anchor_time: Optional[float] = None

    def reset(self):
        self.anchor_pose = None
        self.anchor_time = None

    def update(self, pose: NavigationPose, now: float) -> bool:
        now = float(now)
        if self.anchor_pose is None:
            self.anchor_pose = pose
            self.anchor_time = now
            return True

        linear_change = math.hypot(
            pose.x - self.anchor_pose.x,
            pose.y - self.anchor_pose.y,
        )
        angular_change = abs(
            normalize_angle(pose.yaw - self.anchor_pose.yaw)
        )
        if (
            linear_change >= self.movement_radius_m
            or angular_change >= self.movement_angle_rad
        ):
            self.anchor_pose = pose
            self.anchor_time = now
            return True

        return now - self.anchor_time <= self.allowance_s
