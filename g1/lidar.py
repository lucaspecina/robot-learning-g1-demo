#!/usr/bin/env python3
"""LiDAR RTX del simulador con salida estándar de ROS 2.

El perfil inicial sólo verifica la integración. Unitree no publica el modelo
exacto de LiDAR incluido en cada configuración del G1 EDU; por eso el perfil
queda inyectable y no se presenta como réplica del sensor físico.
"""
import numpy as np
from isaacsim.sensors.rtx import LidarRtx

LIDAR_PARENT_PRIM = "/World/G1/torso_link/head_link"
LIDAR_PRIM_NAME = "lidar"
LIDAR_FRAME = "lidar_link"
LIDAR_TOPIC = "/g1/lidar/points"
PROVISIONAL_PROFILE = "Example_Rotary"
LIDAR_OFFSET = (0.0, 0.0, 0.12)


class LidarBridge:
    """Crea el sensor RTX y conecta su nube de puntos con ROS 2."""

    def __init__(
        self,
        profile: str = PROVISIONAL_PROFILE,
        parent_prim: str = LIDAR_PARENT_PRIM,
    ):
        sensor_path = f"{parent_prim}/{LIDAR_PRIM_NAME}"
        # La clase completa hace una inicialización que el comando de bajo
        # nivel omite; sin ella Isaac publicaba mensajes ROS vacíos.
        self.sensor = LidarRtx(
            prim_path=sensor_path,
            name="g1_lidar",
            translation=np.asarray(LIDAR_OFFSET, dtype=np.float64),
            orientation=np.asarray(
                [1.0, 0.0, 0.0, 0.0],
                dtype=np.float64,
            ),
            config_file_name=profile,
        )
        self.profile = profile
        self.initialized = False

    def initialize(self):
        """Termina de iniciar el sensor después del primer reset físico."""
        self.sensor.initialize()
        self.sensor.attach_writer(
            "RtxLidarROS2PublishPointCloud",
            topicName=LIDAR_TOPIC,
            frameId=LIDAR_FRAME,
        )
        self.initialized = True
