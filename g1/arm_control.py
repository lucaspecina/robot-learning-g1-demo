#!/usr/bin/env python3
"""Control de los brazos del G1 — la capa que la policy de locomocion no toca.

La policy de Unitree controla las 12 articulaciones de las piernas y nada mas.
Los brazos (7 articulaciones por lado) son responsabilidad de este modulo, que
corre en paralelo: mientras la policy camina, el controlador de brazos los
mantiene donde tienen que estar.

Igual que la locomocion, es una clase con interfaz fija para poder cambiarla:

    controller.set_pose(nombre)          # ir a una pose con nombre
    targets = controller.compute(dt)     # angulos objetivo de los 14 brazos

Empezamos con poses predefinidas e interpolacion suave entre ellas. Es
suficiente para la demo (llevar los brazos a "listo para agarrar", cerrar sobre
un objeto, pasar a "transporte") y no necesita entrenamiento. El paso siguiente
—apuntar la mano a un punto del espacio calculado por la camara— es cinematica
inversa, y se agrega como otra implementacion de esta misma interfaz.

Por que interpolar en vez de saltar al objetivo: un movimiento brusco del brazo
corre el centro de masa de golpe, y las piernas —que no saben que el brazo se
va a mover— lo sienten como un empujon. Suave es mas facil de compensar.
"""
import numpy as np

# Los 14 movimientos de los brazos, en el orden en que los manejamos.
ARM_JOINTS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

# Poses con nombre (radianes, en el orden de ARM_JOINTS).
# "reposo" es la pose en la que los brazos estan soldados en el modelo de 12
# articulaciones: la que la policy de locomocion conoce.
POSES = {
    "reposo": np.zeros(14, dtype=np.float32),

    # Brazos adelante, codos flexionados: la postura desde la que se alcanza
    # un objeto sobre una mesa.
    "listo": np.array([
        0.35, 0.16, 0.0, 0.87, 0.0, 0.0, 0.0,      # izquierdo
        0.35, -0.16, 0.0, 0.87, 0.0, 0.0, 0.0,     # derecho
    ], dtype=np.float32),

    # Brazos recogidos contra el cuerpo: para caminar llevando algo, mantiene
    # la carga cerca del centro y molesta menos al equilibrio.
    "transporte": np.array([
        0.20, 0.25, 0.0, 1.20, 0.0, 0.0, 0.0,
        0.20, -0.25, 0.0, 1.20, 0.0, 0.0, 0.0,
    ], dtype=np.float32),
}


class PoseArmController:
    """Lleva los brazos a poses con nombre, interpolando suavemente."""

    def __init__(self, velocidad_rad_s: float = 0.6):
        self.velocidad = velocidad_rad_s
        self.actual = POSES["reposo"].copy()
        self.objetivo = POSES["reposo"].copy()
        self.pose_actual = "reposo"

    def set_pose(self, nombre: str):
        if nombre not in POSES:
            raise ValueError(f"pose desconocida: {nombre}. Opciones: {list(POSES)}")
        self.objetivo = POSES[nombre].copy()
        self.pose_actual = nombre

    def set_joint_targets(self, angulos):
        """Objetivo directo, articulacion por articulacion (para cinematica
        inversa o teleoperacion)."""
        self.objetivo = np.asarray(angulos, dtype=np.float32)
        self.pose_actual = "manual"

    def llego(self, tolerancia_rad: float = 0.02) -> bool:
        return bool(np.max(np.abs(self.objetivo - self.actual)) < tolerancia_rad)

    def compute(self, dt: float) -> np.ndarray:
        """Avanza hacia el objetivo a velocidad limitada y devuelve los angulos."""
        paso = self.velocidad * dt
        delta = np.clip(self.objetivo - self.actual, -paso, paso)
        self.actual = self.actual + delta
        return self.actual.copy()
