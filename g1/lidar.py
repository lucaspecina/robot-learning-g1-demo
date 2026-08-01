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
# El perfil de 360° no puede quedar dentro de la carcasa opaca del USD. A
# 0.12 m entregó 0 puntos; a 0.35 m recuperó los 9.216 puntos del caso fijo.
# Es un montaje de simulación provisional hasta confirmar el LiDAR y soporte
# mecánico exactos del G1 EDU que se compre.
LIDAR_OFFSET = (0.0, 0.0, 0.35)
POINT_CLOUD_ANNOTATOR = "IsaacExtractRTXSensorPointCloudNoAccumulator"


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
        self.best_internal_points = 0
        self.rendered_frames = 0

    def initialize(self):
        """Termina de iniciar el sensor después del primer reset físico."""
        self.sensor.initialize()
        # Medir antes de ROS separa un sensor vacío de un puente que pierde
        # datos ya calculados. Es el mismo lector de la referencia positiva.
        self.sensor.attach_annotator(POINT_CLOUD_ANNOTATOR)
        self.sensor.attach_writer(
            "RtxLidarROS2PublishPointCloud",
            topicName=LIDAR_TOPIC,
            frameId=LIDAR_FRAME,
        )
        self.initialized = True

    def record_internal_points(self) -> tuple[int, bool]:
        """Devuelve puntos crudos y avisa sólo cuando aparece un nuevo máximo."""
        payload = self.sensor.get_current_frame().get(
            POINT_CLOUD_ANNOTATOR,
            {},
        )
        points = np.asarray(payload.get("data", []))
        point_count = int(points.size // 3)
        self.rendered_frames += 1
        improved = point_count > self.best_internal_points
        self.best_internal_points = max(
            self.best_internal_points,
            point_count,
        )
        return point_count, improved
