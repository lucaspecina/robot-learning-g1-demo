#!/usr/bin/env python3
"""Los ojos del G1: camara montada en la cabeza, publicada por ROS 2.

La camara va sobre `head_link`, mirando hacia adelante, y publica lo que ve en
`/g1/head_cam/image` (sensor_msgs/Image, rgb8). Es la materia prima de toda la
percepcion de la demo: encontrar la botella, leer el reloj, distinguir a las
personas por el color de la remera.

Sobre el ritmo: la camara se renderiza cada N pasos de fisica, no en todos.
Dibujar cuesta caro y ningun consumidor necesita 500 imagenes por segundo —
un detector de objetos trabaja a 10-30 por segundo y un modelo de lenguaje con
vision, a una cada varios segundos. Publicar mas es tirar computo.
"""
import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.sensors import Camera, CameraCfg

from sensor_msgs.msg import Image as ImageMsg

from camera_geometry import camera_rotation

# El cuerpo oficial de NVIDIA anida la cabeza debajo del torso. La cámara debe
# ser hija de esa pieza para heredar su movimiento; colgarla de una ruta que
# no existe hace que Isaac cree silenciosamente una cámara fija en el mundo.
CAMERA_PARENT_PRIM = "/World/G1/torso_link/head_link"
CAMERA_PRIM = f"{CAMERA_PARENT_PRIM}/head_cam"
CAMERA_OFFSET = (0.08, 0.0, 0.05)      # metros, respecto de head_link
# La configuración oficial publica 640×480. La prueba a 320×240 dejó las
# patas de la mesa reducidas a pocos píxeles y volvió inestable el reloj.
RESOLUTION = (640, 480)

# La configuración oficial de Unitree para el G1 simulado usa 7.6 mm y una
# apertura de 20 mm. Conservamos por ahora nuestra apertura de 20.955 mm para
# medir sólo el cambio de lente; la diferencia restante es menor al 5 %.
FOCAL_LENGTH_MM = 7.6
HORIZONTAL_APERTURE_MM = 20.955

# Unitree monta su cámara frontal sin una rotación adicional. La prueba
# separada confirmó que el valor anterior apuntaba 20° hacia arriba.
CAMERA_DOWNWARD_PITCH_DEG = 0.0


def make_camera_cfg(update_period: float = 0.1) -> CameraCfg:
    """Camara de color mirando hacia adelante desde la cabeza.

    update_period: segundos entre renders (0.1 = 10 imagenes por segundo).
    """
    return CameraCfg(
        prim_path=CAMERA_PRIM,
        update_period=update_period,
        height=RESOLUTION[1],
        width=RESOLUTION[0],
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=FOCAL_LENGTH_MM,
            focus_distance=400.0,
            horizontal_aperture=HORIZONTAL_APERTURE_MM,
            clipping_range=(0.05, 50.0),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=CAMERA_OFFSET,
            rot=camera_rotation(CAMERA_DOWNWARD_PITCH_DEG),
            convention="ros",
        ),
    )


class CameraPublisher:
    """Toma lo que ve la camara y lo publica como imagen ROS 2."""

    def __init__(self, node, camera: Camera, topic: str = "/g1/head_cam/image"):
        self.camera = camera
        self.pub = node.create_publisher(ImageMsg, topic, 1)
        self.node = node
        self.frames = 0
        self.last_camera_frame = -1

    def publish(self):
        """Publica el ultimo cuadro disponible. Silencioso si todavia no hay."""
        datos = self.camera.data.output.get("rgb")
        if datos is None or datos.shape[0] == 0:
            return False
        camera_frame = int(self.camera.frame[0].item())
        if camera_frame == self.last_camera_frame:
            return False

        img = datos[0]                        # primer (unico) robot
        if hasattr(img, "cpu"):
            img = img.cpu().numpy()
        img = np.asarray(img, dtype=np.uint8)
        if img.ndim == 3 and img.shape[2] == 4:
            img = img[:, :, :3]               # descartar el canal alfa

        msg = ImageMsg()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "head_cam"
        msg.height, msg.width = img.shape[0], img.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        self.pub.publish(msg)
        self.frames += 1
        self.last_camera_frame = camera_frame
        return True
