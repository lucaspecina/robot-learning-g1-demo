#!/usr/bin/env python3
"""Convierte detecciones 2D en puntos 3D del mapa.

Corre en la Jetson y sólo usa interfaces transferibles al G1 real: color,
profundidad, calibración, la relación temporal entre cámara y mapa, y la
detección 2D. No consulta posiciones internas de Isaac ni conoce la escena.

Las mesas se miden por su color para no mezclar fondo. Los objetos pequeños se
miden dentro de su caja. En ambos casos la salida es una superficie visible:
no inventa el centro oculto ni una orientación completa para agarrar.
"""
import json
import sys
from pathlib import Path

import numpy as np

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import rclpy  # noqa: E402
import tf2_geometry_msgs  # noqa: F401,E402
from geometry_msgs.msg import PointStamped  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import Buffer, TransformException, TransformListener  # noqa: E402
from vision_msgs.msg import (  # noqa: E402
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

from camera_stream import (  # noqa: E402
    SynchronizedCameraFrames,
    color_array,
    depth_array,
)
from depth_geometry import colored_table_point, visible_object_point  # noqa: E402
from perception_core import (  # noqa: E402
    TABLE_CLASS_NAMES,
    TRANSPORT_OBJECT_CLASS_NAMES,
    bounded_box,
)

MAX_CAMERA_FRAMES = 120
TF_HISTORY_S = 120.0
# La cámara publica los tres canales con entrega garantizada. Para unirlos por
# fecha hay que conservar esa misma garantía: el perfil típico de video tolera
# pérdidas, pero una sola pieza faltante invalida la medición 3D completa.
EXACT_CAMERA_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
    reliability=ReliabilityPolicy.RELIABLE,
)


class SpatialLocalizer(Node):
    def __init__(self):
        super().__init__("spatial_localizer")
        self.frames = SynchronizedCameraFrames(MAX_CAMERA_FRAMES)
        # Grounding DINO vive en el servidor y puede responder mucho después
        # del cuadro. Conservar el historial evita usar la pose actual para
        # una imagen vieja cuando el robot ya se movió.
        self.tf_buffer = Buffer(
            cache_time=Duration(seconds=TF_HISTORY_S),
        )
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )
        self.table_detections_pub = self.create_publisher(
            Detection3DArray,
            "/g1/table_detections_3d",
            qos_profile_sensor_data,
        )
        self.object_detections_pub = self.create_publisher(
            Detection3DArray,
            "/g1/object_detections_3d",
            qos_profile_sensor_data,
        )
        self.status_pub = self.create_publisher(
            String,
            "/g1/perception/localization_status",
            10,
        )
        self.create_subscription(
            Image,
            "/g1/head_cam/image",
            lambda message: self.frames.add("color", message),
            EXACT_CAMERA_QOS,
        )
        self.create_subscription(
            Image,
            "/g1/head_cam/depth",
            lambda message: self.frames.add("depth", message),
            EXACT_CAMERA_QOS,
        )
        self.create_subscription(
            CameraInfo,
            "/g1/head_cam/camera_info",
            lambda message: self.frames.add("info", message),
            EXACT_CAMERA_QOS,
        )
        self.create_subscription(
            Detection2DArray,
            "/g1/open_vocabulary_detections",
            self.on_table_detections,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection2DArray,
            "/g1/object_detections",
            self.on_object_detections,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "localizador 3D listo; espera mesas y objetos detectados"
        )

    def publish_status(self, state: str, **fields):
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {"state": state, **fields},
                    ensure_ascii=False,
                )
            )
        )

    @staticmethod
    def class_name(detection) -> str | None:
        if not detection.results:
            return None
        best = max(
            detection.results,
            key=lambda result: result.hypothesis.score,
        )
        return best.hypothesis.class_id

    def on_table_detections(self, message: Detection2DArray):
        selected = [
            detection
            for detection in message.detections
            if self.class_name(detection) in TABLE_CLASS_NAMES
        ]
        self.localize_message(
            message,
            selected,
            self.localize_table,
            self.table_detections_pub,
            "mesa",
        )

    def on_object_detections(self, message: Detection2DArray):
        selected = [
            detection
            for detection in message.detections
            if self.class_name(detection) in TRANSPORT_OBJECT_CLASS_NAMES
        ]
        self.localize_message(
            message,
            selected,
            self.localize_object,
            self.object_detections_pub,
            "objeto",
        )

    def localize_message(
        self,
        message: Detection2DArray,
        detections,
        localizer,
        publisher,
        target_kind: str,
    ):
        if not detections:
            return
        frame = self.frames.complete(message.header)
        if frame is None:
            self.publish_status(
                "failed",
                target_kind=target_kind,
                error="no se conservó el cuadro sincronizado",
            )
            self.get_logger().warning(
                f"descarto {target_kind}: no se conservó su cuadro sincronizado"
            )
            return
        output = Detection3DArray()
        output.header.stamp = message.header.stamp
        output.header.frame_id = "map"
        for detection in detections:
            try:
                localized = localizer(detection, message.header, frame)
            except (ValueError, TransformException) as error:
                self.publish_status(
                    "failed",
                    target_kind=target_kind,
                    detection_id=detection.id,
                    error=str(error),
                )
                self.get_logger().warning(
                    f"no pude ubicar {detection.id}: {error}"
                )
                continue
            output.detections.append(localized)
            point = localized.bbox.center.position
            class_id = localized.results[0].hypothesis.class_id
            self.publish_status(
                "localized",
                target_kind=target_kind,
                detection_id=detection.id,
                class_id=class_id,
                x=round(point.x, 3),
                y=round(point.y, 3),
                z=round(point.z, 3),
            )
            self.get_logger().info(
                f"{class_id} ubicado en "
                f"({point.x:.2f}, {point.y:.2f}, {point.z:.2f})"
            )
        if output.detections:
            publisher.publish(output)

    @staticmethod
    def measurement_inputs(detection, frame):
        color_message = frame["color"]
        depth_message = frame["depth"]
        info = frame["info"]
        dimensions = {
            (color_message.width, color_message.height),
            (depth_message.width, depth_message.height),
            (info.width, info.height),
        }
        if len(dimensions) != 1:
            raise ValueError("color, profundidad y calibración no coinciden")
        center = detection.bbox.center.position
        box = bounded_box(
            center.x,
            center.y,
            detection.bbox.size_x,
            detection.bbox.size_y,
            color_message.width,
            color_message.height,
        )
        return color_message, depth_message, info, box

    def map_point(self, camera_point, header):
        stamped_point = PointStamped()
        stamped_point.header = header
        stamped_point.point.x = camera_point.right_m
        stamped_point.point.y = camera_point.down_m
        stamped_point.point.z = camera_point.forward_m
        return self.tf_buffer.transform(
            stamped_point,
            "map",
            timeout=Duration(seconds=2.0),
        )

    @staticmethod
    def make_detection(
        detection,
        header,
        map_point,
        class_id: str,
    ) -> Detection3D:
        result = Detection3D()
        result.header.stamp = header.stamp
        result.header.frame_id = "map"
        result.id = detection.id
        result.bbox.center.position = map_point.point
        result.bbox.center.orientation.w = 1.0
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = class_id
        if detection.results:
            hypothesis.hypothesis.score = max(
                item.hypothesis.score
                for item in detection.results
            )
        hypothesis.pose.pose.position = map_point.point
        hypothesis.pose.pose.orientation.w = 1.0
        result.results.append(hypothesis)
        return result

    def localize_table(self, detection, header, frame) -> Detection3D:
        color_message, depth_message, info, box = self.measurement_inputs(
            detection,
            frame,
        )
        camera_point = colored_table_point(
            color_array(color_message),
            depth_array(depth_message),
            np.asarray(info.k, dtype=np.float64).reshape(3, 3),
            box,
        )
        return self.make_detection(
            detection,
            header,
            self.map_point(camera_point, header),
            f"{camera_point.color}_table",
        )

    def localize_object(self, detection, header, frame) -> Detection3D:
        _color_message, depth_message, info, box = self.measurement_inputs(
            detection,
            frame,
        )
        camera_point = visible_object_point(
            depth_array(depth_message),
            np.asarray(info.k, dtype=np.float64).reshape(3, 3),
            box,
        )
        # Una caja 2D más profundidad sólo mide la cara visible. El tamaño cero
        # y la orientación identidad evitan presentar eso como la pose completa
        # que FoundationPose deberá aportar para el agarre real.
        return self.make_detection(
            detection,
            header,
            self.map_point(camera_point, header),
            "transport_object",
        )


def main():
    rclpy.init()
    node = SpatialLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
