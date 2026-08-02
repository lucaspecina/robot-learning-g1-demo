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
NAVIGATION_LIDAR_PRIM_NAME = "navigation_lidar"
LIDAR_FRAME = "lidar_link"
LIDAR_TOPIC = "/g1/lidar/points"
PROVISIONAL_PROFILE = "Example_Rotary"
NAVIGATION_PROFILE = "Example_Rotary_2D"
PROVISIONAL_SCAN_RATE_HZ = 10.0
LIDAR_NEAR_RANGE_M = 0.05
# El perfil de 360° no puede quedar dentro de la carcasa opaca del USD. A
# 0.12 m entregó 0 puntos; a 0.35 m recuperó los 9.216 puntos del caso fijo.
# Es un montaje de simulación provisional hasta confirmar el LiDAR y soporte
# mecánico exactos del G1 EDU que se compre.
LIDAR_OFFSET = (0.0, 0.0, 0.35)
POINT_CLOUD_ANNOTATOR = "IsaacExtractRTXSensorPointCloudNoAccumulator"
# Es el escritor que selecciona el helper oficial de NVIDIA con fullScan=true.
# El escritor sin "Buffer" publica el sector de un cuadro y deja puntos ciegos.
POINT_CLOUD_WRITER = "RtxLidarROS2PublishPointCloudBuffer"
LASER_SCAN_WRITER = "RtxLidarROS2PublishLaserScan"
LASER_SCAN_TOPIC = "/scan_raw"


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
            # NVIDIA exige que el ritmo del sensor coincida con el perfil y
            # que la acumulación esté activa para publicar una vuelta entera.
            # Sin ambas propiedades, cada mensaje contenía sólo el sector
            # recorrido durante un cuadro y la protección tenía puntos ciegos.
            **{
                "omni:sensor:tickRate": PROVISIONAL_SCAN_RATE_HZ,
                "omni:sensor:Core:accumulateOutputs": True,
            },
        )
        # Isaac 5.1 sólo acumula LaserScan con un perfil 2D. NVIDIA también
        # separa sus sensores 3D y 2D en el tutorial oficial. Ambos comparten
        # montaje y marco: el segundo es un adaptador del simulador, no otro
        # dispositivo prometido para el G1 físico. El perfil pierde la pared
        # debajo de 1 m; se corrige sólo su zona ciega para igualar los 0,05 m
        # declarados por Unitree. RPLIDAR_S2E produjo cortes por datos tardíos.
        self.navigation_sensor = LidarRtx(
            prim_path=f"{parent_prim}/{NAVIGATION_LIDAR_PRIM_NAME}",
            name="g1_navigation_lidar",
            translation=np.asarray(LIDAR_OFFSET, dtype=np.float64),
            orientation=np.asarray(
                [1.0, 0.0, 0.0, 0.0],
                dtype=np.float64,
            ),
            config_file_name=NAVIGATION_PROFILE,
            **{
                "omni:sensor:Core:nearRangeM": LIDAR_NEAR_RANGE_M,
            },
        )
        self.profile = profile
        self.initialized = False
        self.best_internal_points = 0
        self.rendered_frames = 0

    def initialize(self):
        """Termina de iniciar el sensor después del primer reset físico."""
        self.sensor.initialize()
        self.navigation_sensor.initialize()
        # Medir antes de ROS separa un sensor vacío de un puente que pierde
        # datos ya calculados. Es el mismo lector de la referencia positiva.
        self.sensor.attach_annotator(POINT_CLOUD_ANNOTATOR)
        self.sensor.attach_writer(
            # La nube 3D acumulada es el contrato estándar para obstáculos;
            # el barrido 2D queda sólo como adaptación temporal de navegación.
            POINT_CLOUD_WRITER,
            topicName=LIDAR_TOPIC,
            frameId=LIDAR_FRAME,
        )
        self.navigation_sensor.attach_writer(
            # NVIDIA usa este escritor para alimentar navegación con un
            # LaserScan completo; evitar una proyección propia mantiene el
            # contrato estándar que tendrá el driver del sensor físico.
            LASER_SCAN_WRITER,
            topicName=LASER_SCAN_TOPIC,
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
