#!/usr/bin/env python3
"""Convierte una mesa detectada en imagen en un punto 3D del mapa.

Corre en la Jetson y sólo usa interfaces transferibles al G1 real: color,
profundidad, calibración, la relación temporal entre cámara y mapa, y la
detección 2D. No consulta posiciones internas de Isaac ni conoce la escena.
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
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
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
from depth_geometry import colored_table_point  # noqa: E402
from perception_core import bounded_box  # noqa: E402

MAX_CAMERA_FRAMES = 120
TF_HISTORY_S = 120.0


class TableLocalizer(Node):
    def __init__(self):
        super().__init__("table_localizer")
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
        self.detections_pub = self.create_publisher(
            Detection3DArray,
            "/g1/table_detections_3d",
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
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/g1/head_cam/depth",
            lambda message: self.frames.add("depth", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/g1/head_cam/camera_info",
            lambda message: self.frames.add("info", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection2DArray,
            "/g1/open_vocabulary_detections",
            self.on_detections,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "localizador de mesas listo; espera detecciones visuales"
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

    def on_detections(self, message: Detection2DArray):
        frame = self.frames.complete(message.header)
        if frame is None:
            self.publish_status(
                "failed",
                error="no se conservó el cuadro sincronizado",
            )
            self.get_logger().warning(
                "descarto detección: no se conservó su cuadro sincronizado"
            )
            return
        output = Detection3DArray()
        output.header.stamp = message.header.stamp
        output.header.frame_id = "map"
        for detection in message.detections:
            try:
                localized = self.localize(detection, message.header, frame)
            except (ValueError, TransformException) as error:
                self.publish_status(
                    "failed",
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
                detection_id=detection.id,
                class_id=class_id,
                x=round(point.x, 3),
                y=round(point.y, 3),
                z=round(point.z, 3),
            )
            self.get_logger().info(
                f"{class_id} ubicada en "
                f"({point.x:.2f}, {point.y:.2f}, {point.z:.2f})"
            )
        if output.detections:
            self.detections_pub.publish(output)

    def localize(self, detection, header, frame) -> Detection3D:
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
        camera_point = colored_table_point(
            color_array(color_message),
            depth_array(depth_message),
            np.asarray(info.k, dtype=np.float64).reshape(3, 3),
            box,
        )
        stamped_point = PointStamped()
        stamped_point.header = header
        stamped_point.point.x = camera_point.right_m
        stamped_point.point.y = camera_point.down_m
        stamped_point.point.z = camera_point.forward_m
        map_point = self.tf_buffer.transform(
            stamped_point,
            "map",
            timeout=Duration(seconds=2.0),
        )

        result = Detection3D()
        result.header.stamp = header.stamp
        result.header.frame_id = "map"
        result.id = detection.id
        result.bbox.center.position = map_point.point
        result.bbox.center.orientation.w = 1.0
        # La profundidad produce un punto observado de la superficie, no el
        # volumen entero de la mesa. Un tamaño cero evita inventar dimensiones
        # que el sensor todavía no midió.
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = f"{camera_point.color}_table"
        if detection.results:
            hypothesis.hypothesis.score = (
                detection.results[0].hypothesis.score
            )
        hypothesis.pose.pose.position = map_point.point
        hypothesis.pose.pose.orientation.w = 1.0
        result.results.append(hypothesis)
        return result


def main():
    rclpy.init()
    node = TableLocalizer()
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
