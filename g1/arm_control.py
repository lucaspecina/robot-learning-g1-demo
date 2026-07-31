#!/usr/bin/env python3
"""Control de los brazos del G1 — la capa que la policy de locomocion no toca.

La policy de Unitree controla las 12 articulaciones de las piernas y nada mas.
Los brazos (7 articulaciones por lado) son responsabilidad de este modulo, que
corre en paralelo: mientras la policy camina, el controlador de brazos los
mantiene donde tienen que estar.

Igual que la locomocion, es una clase con interfaz fija para poder cambiarla:

    controller.set_pose(nombre)          # ir a una pose con nombre
    targets = controller.compute(dt)     # angulos objetivo de los 14 brazos

Empezamos con poses predefinidas e interpolacion suave entre ellas. Esto sólo
prepara los brazos y permite medir la locomoción con distintas distribuciones
de peso. Todavía no alcanza para agarrar: falta llevar la mano a la posición
medida del objeto y controlar los dedos. El primer paso se puede resolver con
cinemática inversa —calcular los ángulos que colocan la mano en un punto— y el
agarre completo se agregará como otra implementación de esta misma interfaz.

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

    # Esta pose sólo deja los brazos preparados; la posición exacta de la mano
    # dependerá de dónde mida la cámara que está el objeto.
    "listo": np.array([
        0.35, 0.16, 0.0, 0.87, 0.0, 0.0, 0.0,      # izquierdo
        0.35, -0.16, 0.0, 0.87, 0.0, 0.0, 0.0,     # derecho
    ], dtype=np.float32),

    # La pose anterior usaba hombro positivo y llevó las muñecas contra la
    # pelvis: los ángulos se cumplían, pero la inspección visual la rechazó.
    # Esta candidata espeja el brazo derecho del cuadro 178 de la demostración
    # NVIDIA object_pick_and_place_retarget_motion_g1_3finger_hands. NVIDIA
    # controla las muñecas por posición; el espejo bilateral sigue siendo una
    # adaptación nuestra y debe aprobar visualmente antes de llamarse estable.
    "transporte": np.array([
        -0.49, 0.51, -0.51, 0.79, -0.38, -0.34, -0.15,
        -0.49, -0.51, 0.51, 0.79, 0.38, -0.34, 0.15,
    ], dtype=np.float32),
}


class PoseArmController:
    """Lleva los brazos a poses con nombre, interpolando suavemente."""

    def __init__(self, velocidad_rad_s: float = 0.6):
        self.velocidad = velocidad_rad_s
        self.reset()

    def reset(self):
        """Vuelve a reposo sin conservar una transición de la corrida anterior."""
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
