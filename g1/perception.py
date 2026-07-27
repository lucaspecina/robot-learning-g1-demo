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

# Montaje: sobre la cabeza, un poco adelante y arriba del centro del link.
CAMERA_PRIM = "/World/G1/head_link/head_cam"
CAMERA_OFFSET = (0.08, 0.0, 0.05)      # metros, respecto de head_link
RESOLUTION = (320, 240)                 # suficiente para deteccion; barato de renderizar


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
            focal_length=18.0,
            focus_distance=400.0,
            horizontal_aperture=20.955,
            clipping_range=(0.05, 50.0),
        ),
        # Isaac usa la convencion de camara de graficos ("world"): mirando por
        # -Z. Rotamos para que mire hacia adelante del robot (+X).
        offset=CameraCfg.OffsetCfg(
            pos=CAMERA_OFFSET,
            rot=(0.5, -0.5, 0.5, -0.5),   # (w, x, y, z)
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

    def publish(self):
        """Publica el ultimo cuadro disponible. Silencioso si todavia no hay."""
        datos = self.camera.data.output.get("rgb")
        if datos is None or datos.shape[0] == 0:
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
        return True
