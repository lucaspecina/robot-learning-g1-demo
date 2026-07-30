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

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image as ImageMsg
from tf2_ros import TransformBroadcaster

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
CAMERA_FRAME = "head_cam_optical"

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
        # Isaac Lab omite esta consulta por rendimiento y deja la pose en
        # ceros. Una cámara móvil necesita su pose en cada cuadro para ubicar
        # en la sala lo que mide con profundidad.
        update_latest_camera_pose=True,
        # El G1 real trae una cámara de profundidad. Pedir ambos canales al
        # mismo sensor garantiza que color y distancia describan el mismo
        # instante y evita usar posiciones internas de Isaac.
        data_types=["rgb", "distance_to_image_plane"],
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
    """Publica color, profundidad y calibración del mismo cuadro."""

    def __init__(self, node, camera: Camera, topic: str = "/g1/head_cam/image"):
        self.camera = camera
        self.pub = node.create_publisher(ImageMsg, topic, 1)
        self.depth_pub = node.create_publisher(
            ImageMsg,
            "/g1/head_cam/depth",
            1,
        )
        self.info_pub = node.create_publisher(
            CameraInfo,
            "/g1/head_cam/camera_info",
            1,
        )
        self.tf_broadcaster = TransformBroadcaster(node)
        self.node = node
        self.frames = 0
        self.last_camera_frame = -1

    def publish(self):
        """Publica el ultimo cuadro disponible. Silencioso si todavia no hay."""
        rgb_data = self.camera.data.output.get("rgb")
        depth_data = self.camera.data.output.get(
            "distance_to_image_plane"
        )
        if (
            rgb_data is None
            or rgb_data.shape[0] == 0
            or depth_data is None
            or depth_data.shape[0] == 0
        ):
            return False
        camera_frame = int(self.camera.frame[0].item())
        if camera_frame == self.last_camera_frame:
            return False

        img = rgb_data[0]                     # primer (unico) robot
        if hasattr(img, "cpu"):
            img = img.cpu().numpy()
        img = np.asarray(img, dtype=np.uint8)
        if img.ndim == 3 and img.shape[2] == 4:
            img = img[:, :, :3]               # descartar el canal alfa

        stamp = self.node.get_clock().now().to_msg()
        msg = ImageMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = CAMERA_FRAME
        msg.height, msg.width = img.shape[0], img.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        self.pub.publish(msg)

        depth = depth_data[0]
        if hasattr(depth, "cpu"):
            depth = depth.cpu().numpy()
        depth = np.asarray(depth, dtype=np.float32).reshape(
            msg.height,
            msg.width,
        )
        depth_msg = ImageMsg()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = CAMERA_FRAME
        depth_msg.height, depth_msg.width = depth.shape
        depth_msg.encoding = "32FC1"
        depth_msg.is_bigendian = False
        depth_msg.step = depth_msg.width * np.dtype(np.float32).itemsize
        depth_msg.data = depth.tobytes()
        self.depth_pub.publish(depth_msg)

        intrinsics = self.camera.data.intrinsic_matrices[0]
        if hasattr(intrinsics, "cpu"):
            intrinsics = intrinsics.cpu().numpy()
        intrinsics = np.asarray(intrinsics, dtype=np.float64)
        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = CAMERA_FRAME
        camera_info.height = msg.height
        camera_info.width = msg.width
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0] * 5
        camera_info.k = intrinsics.reshape(-1).tolist()
        camera_info.r = np.eye(3, dtype=np.float64).reshape(-1).tolist()
        camera_info.p = [
            float(intrinsics[0, 0]), 0.0,
            float(intrinsics[0, 2]), 0.0,
            0.0, float(intrinsics[1, 1]),
            float(intrinsics[1, 2]), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self.info_pub.publish(camera_info)

        position = self.camera.data.pos_w[0]
        orientation = self.camera.data.quat_w_ros[0]
        if hasattr(position, "cpu"):
            position = position.cpu().numpy()
        if hasattr(orientation, "cpu"):
            orientation = orientation.cpu().numpy()
        position = np.asarray(position, dtype=np.float64)
        orientation = np.asarray(orientation, dtype=np.float64)
        camera_transform = TransformStamped()
        camera_transform.header.stamp = stamp
        camera_transform.header.frame_id = "map"
        camera_transform.child_frame_id = CAMERA_FRAME
        camera_transform.transform.translation.x = float(position[0])
        camera_transform.transform.translation.y = float(position[1])
        camera_transform.transform.translation.z = float(position[2])
        # Isaac Lab entrega este cuaternión como (w, x, y, z). Es la pose del
        # marco óptico exacto usado para proyectar la profundidad; publicar
        # otra convención produciría puntos espejados aunque la imagen se vea.
        camera_transform.transform.rotation.w = float(orientation[0])
        camera_transform.transform.rotation.x = float(orientation[1])
        camera_transform.transform.rotation.y = float(orientation[2])
        camera_transform.transform.rotation.z = float(orientation[3])
        self.tf_broadcaster.sendTransform(camera_transform)

        self.frames += 1
        self.last_camera_frame = camera_frame
        return True
