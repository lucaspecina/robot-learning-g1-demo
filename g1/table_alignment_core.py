#!/usr/bin/env python3
"""Control puro para la aproximación visual fina a una mesa."""

from dataclasses import dataclass
import math

from navigation_core import normalize_angle


@dataclass(frozen=True)
class AlignmentPose:
    x: float
    y: float
    yaw: float
    linear_speed: float
    angular_speed: float


@dataclass(frozen=True)
class AlignmentTarget:
    x: float
    y: float


@dataclass(frozen=True)
class AlignmentCommand:
    linear_x: float
    angular_z: float
    distance_error_m: float
    yaw_error_rad: float
    stable: bool
    phase: str


class TargetFilter:
    """Suaviza mediciones nuevas sin ocultar cuánto se desplazó el objetivo."""

    def __init__(self, coefficient: float = 0.1):
        if not 0.0 < coefficient <= 1.0:
            raise ValueError("el coeficiente debe estar entre 0 y 1")
        self.coefficient = float(coefficient)
        self.target = None

    def reset(self):
        self.target = None

    def update(self, target: AlignmentTarget) -> AlignmentTarget:
        if not all(math.isfinite(value) for value in (target.x, target.y)):
            raise ValueError("la medición de la mesa no es finita")
        if self.target is None:
            self.target = target
        else:
            keep = 1.0 - self.coefficient
            self.target = AlignmentTarget(
                x=keep * self.target.x + self.coefficient * target.x,
                y=keep * self.target.y + self.coefficient * target.y,
            )
        return self.target


class TableAlignmentController:
    """Convierte base y mesa medidas en una corrección lenta y verificable."""

    def __init__(
        self,
        *,
        standoff_m: float = 0.70,
        distance_tolerance_m: float = 0.03,
        yaw_tolerance_rad: float = math.radians(2.0),
        stable_duration_s: float = 1.5,
        stopped_linear_speed_mps: float = 0.02,
        stopped_angular_speed_radps: float = 0.03,
        maximum_linear_speed_mps: float = 0.12,
        maximum_angular_speed_radps: float = 0.20,
    ):
        positive = {
            "separación": standoff_m,
            "tolerancia lineal": distance_tolerance_m,
            "tolerancia angular": yaw_tolerance_rad,
            "tiempo estable": stable_duration_s,
            "velocidad lineal detenida": stopped_linear_speed_mps,
            "velocidad angular detenida": stopped_angular_speed_radps,
            "velocidad lineal máxima": maximum_linear_speed_mps,
            "velocidad angular máxima": maximum_angular_speed_radps,
        }
        for label, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{label} debe ser positiva y finita")
        self.standoff_m = float(standoff_m)
        self.distance_tolerance_m = float(distance_tolerance_m)
        self.yaw_tolerance_rad = float(yaw_tolerance_rad)
        self.stable_duration_s = float(stable_duration_s)
        self.stopped_linear_speed_mps = float(stopped_linear_speed_mps)
        self.stopped_angular_speed_radps = float(stopped_angular_speed_radps)
        self.maximum_linear_speed_mps = float(maximum_linear_speed_mps)
        self.maximum_angular_speed_radps = float(maximum_angular_speed_radps)
        self.stable_since = None

    def reset(self):
        self.stable_since = None

    @staticmethod
    def _bounded_with_minimum(
        value: float,
        minimum_magnitude: float,
        maximum_magnitude: float,
    ) -> float:
        magnitude = min(maximum_magnitude, max(minimum_magnitude, abs(value)))
        return math.copysign(magnitude, value)

    def step(
        self,
        pose: AlignmentPose,
        target: AlignmentTarget,
        now: float,
    ) -> AlignmentCommand:
        values = (
            pose.x,
            pose.y,
            pose.yaw,
            pose.linear_speed,
            pose.angular_speed,
            target.x,
            target.y,
            now,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("la alineación recibió una medición no finita")

        dx = target.x - pose.x
        dy = target.y - pose.y
        distance_error = math.hypot(dx, dy) - self.standoff_m
        yaw_error = normalize_angle(math.atan2(dy, dx) - pose.yaw)
        inside_pose = (
            abs(distance_error) <= self.distance_tolerance_m
            and abs(yaw_error) <= self.yaw_tolerance_rad
        )
        stopped = (
            abs(pose.linear_speed) <= self.stopped_linear_speed_mps
            and abs(pose.angular_speed) <= self.stopped_angular_speed_radps
        )

        if inside_pose:
            if stopped:
                if self.stable_since is None:
                    self.stable_since = float(now)
                stable = now - self.stable_since >= self.stable_duration_s
                return AlignmentCommand(
                    linear_x=0.0,
                    angular_z=0.0,
                    distance_error_m=distance_error,
                    yaw_error_rad=yaw_error,
                    stable=stable,
                    phase="stable" if stable else "settling",
                )
            self.stable_since = None
            return AlignmentCommand(
                linear_x=0.0,
                angular_z=0.0,
                distance_error_m=distance_error,
                yaw_error_rad=yaw_error,
                stable=False,
                phase="settling",
            )

        self.stable_since = None
        angular = self._bounded_with_minimum(
            1.2 * yaw_error,
            minimum_magnitude=0.04,
            maximum_magnitude=self.maximum_angular_speed_radps,
        )
        if abs(yaw_error) > math.radians(10.0):
            return AlignmentCommand(
                linear_x=0.0,
                angular_z=angular,
                distance_error_m=distance_error,
                yaw_error_rad=yaw_error,
                stable=False,
                phase="turning",
            )

        linear = 0.0
        if abs(distance_error) > self.distance_tolerance_m:
            linear = self._bounded_with_minimum(
                0.35 * distance_error,
                minimum_magnitude=0.04,
                maximum_magnitude=self.maximum_linear_speed_mps,
            )
        if abs(yaw_error) <= self.yaw_tolerance_rad:
            angular = 0.0
        return AlignmentCommand(
            linear_x=linear,
            angular_z=angular,
            distance_error_m=distance_error,
            yaw_error_rad=yaw_error,
            stable=False,
            phase="approaching" if linear >= 0.0 else "backing_up",
        )
