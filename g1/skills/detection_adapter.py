#!/usr/bin/env python3
"""Adapta detecciones estándar a las salidas propias de la demo.

Este proceso no sabe qué motor produjo las cajas. Por eso seguirá funcionando
cuando el RT-DETR de compatibilidad sea reemplazado por Isaac ROS en el G1.
"""
import io
import json
import sys
from collections import OrderedDict
from pathlib import Path

import numpy as np
from PIL import Image as PILImage

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from perception_core import (  # noqa: E402
    CLASS_NAMES,
    TABLE_CLASS_NAMES,
    bounded_box,
    classify_table_color,
    legacy_detection,
    padded_box,
)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CompressedImage, Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from vision_msgs.msg import Detection2DArray  # noqa: E402

IMAGE_CACHE_SIZE = 60


def stamp_key(header) -> tuple[int, int]:
    return (header.stamp.sec, header.stamp.nanosec)


class DetectionAdapter(Node):
    def __init__(self):
        super().__init__("detection_adapter")
        self.images = OrderedDict()
        self.detections_pub = self.create_publisher(
            String,
            "/g1/detections",
            10,
        )
        self.clock_crop_pub = self.create_publisher(
            CompressedImage,
            "/g1/clock_crop/compressed",
            2,
        )
        self.create_subscription(
            Image,
            "/g1/head_cam/image",
            self.on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection2DArray,
            "/g1/object_detections",
            self.on_detections,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "adaptador listo; espera cajas en /g1/object_detections"
        )

    def on_image(self, msg: Image):
        if msg.encoding != "rgb8":
            return
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height,
            msg.width,
            3,
        ).copy()
        self.images[stamp_key(msg.header)] = (msg.header, rgb)
        while len(self.images) > IMAGE_CACHE_SIZE:
            self.images.popitem(last=False)

    def on_detections(self, msg: Detection2DArray):
        image_entry = self.images.get(stamp_key(msg.header))
        if image_entry is None:
            self.get_logger().warning(
                "llegó una detección cuyo cuadro ya no está en memoria"
            )
            return
        source_header, rgb = image_entry
        image_height, image_width = rgb.shape[:2]
        output = {}
        for detection in msg.detections:
            if not detection.results:
                continue
            best = max(
                detection.results,
                key=lambda result: result.hypothesis.score,
            ).hypothesis
            class_name = best.class_id
            if class_name not in CLASS_NAMES:
                continue
            box = bounded_box(
                detection.bbox.center.position.x,
                detection.bbox.center.position.y,
                detection.bbox.size_x,
                detection.bbox.size_y,
                image_width,
                image_height,
            )
            name = CLASS_NAMES[class_name]
            if class_name in TABLE_CLASS_NAMES:
                name = classify_table_color(rgb, box)
            output[name] = legacy_detection(
                class_name,
                best.score,
                box,
                image_width,
                image_height,
            )
            if class_name == "clock":
                self.publish_clock_crop(source_header, rgb, box)
        self.detections_pub.publish(
            String(data=json.dumps(output, ensure_ascii=False))
        )

    def publish_clock_crop(self, header, rgb: np.ndarray, box):
        crop_box = padded_box(box, rgb.shape[1], rgb.shape[0])
        crop = rgb[crop_box.y1:crop_box.y2, crop_box.x1:crop_box.x2]
        buffer = io.BytesIO()
        PILImage.fromarray(crop).save(buffer, format="JPEG", quality=90)
        message = CompressedImage()
        message.header = header
        message.format = "jpeg"
        message.data = buffer.getvalue()
        self.clock_crop_pub.publish(message)


def main():
    rclpy.init()
    node = DetectionAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
