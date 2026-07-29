#!/usr/bin/env python3
"""Detector neuronal compatible con la interfaz oficial de Isaac ROS.

La VM usa aquí RT-DETR en CPU porque su T4 no pertenece a las plataformas
admitidas por Isaac ROS 4.5. El G1 real podrá reemplazar este proceso por
`isaac_ros_rtdetr`: ambos publican `vision_msgs/Detection2DArray`.
"""
import json
import os
import threading
import time

import numpy as np
import torch
from PIL import Image as PILImage
from transformers import AutoImageProcessor, RTDetrForObjectDetection

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

DEFAULT_MODEL = "PekingU/rtdetr_r50vd"
DEFAULT_REVISION = "df939e661d8c52e80608d1ec566561aabd25a4e7"
TARGET_CLASSES = {"clock", "bottle", "diningtable", "dining table"}


class ObjectDetector(Node):
    def __init__(self):
        super().__init__("object_detector")
        self.model_name = os.environ.get("G1_OBJECT_DETECTOR_MODEL", DEFAULT_MODEL)
        self.model_revision = os.environ.get(
            "G1_OBJECT_DETECTOR_REVISION",
            DEFAULT_REVISION,
        )
        self.confidence = float(os.environ.get("G1_DETECTOR_CONFIDENCE", "0.70"))
        self.minimum_interval_s = float(
            os.environ.get("G1_DETECTOR_INTERVAL_S", "0.75")
        )
        torch.set_num_threads(int(os.environ.get("G1_DETECTOR_CPU_THREADS", "1")))

        self.get_logger().info(
            "cargando RT-DETR; la primera carga puede tardar unos segundos"
        )
        self.processor = AutoImageProcessor.from_pretrained(
            self.model_name,
            revision=self.model_revision,
            use_fast=True,
            local_files_only=True,
        )
        self.model = RTDetrForObjectDetection.from_pretrained(
            self.model_name,
            revision=self.model_revision,
            local_files_only=True,
        ).eval()

        self.detections_pub = self.create_publisher(
            Detection2DArray,
            "/g1/object_detections",
            qos_profile_sensor_data,
        )
        self.status_pub = self.create_publisher(
            String,
            "/g1/perception/status",
            10,
        )
        self.latest_image = None
        self.image_condition = threading.Condition()
        self.finished = False
        self.received_frames = 0
        self.processed_frames = 0
        self.create_subscription(
            Image,
            "/g1/head_cam/image",
            self.on_image,
            qos_profile_sensor_data,
        )
        self.worker = threading.Thread(target=self.run_worker, daemon=True)
        self.worker.start()
        self.get_logger().info(
            f"RT-DETR listo; umbral {self.confidence:.2f}, "
            f"intervalo mínimo {self.minimum_interval_s:.2f} s"
        )

    def on_image(self, msg: Image):
        if msg.encoding != "rgb8":
            return
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height,
            msg.width,
            3,
        ).copy()
        with self.image_condition:
            self.received_frames += 1
            # Guardar sólo el cuadro más nuevo evita acumular imágenes viejas
            # cuando la CPU simulada procesa más lento que la cámara.
            self.latest_image = (msg.header, rgb)
            self.image_condition.notify()

    def run_worker(self):
        last_started_at = 0.0
        while rclpy.ok() and not self.finished:
            with self.image_condition:
                self.image_condition.wait_for(
                    lambda: self.latest_image is not None or self.finished,
                    timeout=0.5,
                )
                if self.finished:
                    return
                if self.latest_image is None:
                    # Isaac tarda bastante en crear el primer cuadro después
                    # de un reinicio. Un plazo vencido significa "seguir
                    # esperando", no que haya una imagen para desempaquetar.
                    continue
                header, rgb = self.latest_image
                self.latest_image = None

            remaining = self.minimum_interval_s - (
                time.monotonic() - last_started_at
            )
            if remaining > 0:
                time.sleep(remaining)
                with self.image_condition:
                    if self.latest_image is not None:
                        header, rgb = self.latest_image
                        self.latest_image = None

            last_started_at = time.monotonic()
            started_at = time.monotonic()
            inputs = self.processor(
                images=PILImage.fromarray(rgb),
                return_tensors="pt",
            )
            with torch.inference_mode():
                outputs = self.model(**inputs)
            target_sizes = torch.tensor([[rgb.shape[0], rgb.shape[1]]])
            result = self.processor.post_process_object_detection(
                outputs,
                target_sizes=target_sizes,
                threshold=self.confidence,
            )[0]
            elapsed_ms = (time.monotonic() - started_at) * 1000.0
            if not rclpy.ok() or self.finished:
                return
            try:
                self.publish_detections(header, result)
            except Exception:
                # Durante un apagado ordenado el contexto ROS puede cerrarse
                # mientras termina una inferencia que ya estaba en curso.
                if not rclpy.ok() or self.finished:
                    return
                raise
            self.processed_frames += 1
            self.status_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "backend": "rtdetr_cpu_compatibility",
                            "model": self.model_name,
                            "latency_ms": round(elapsed_ms, 1),
                            "received_frames": self.received_frames,
                            "processed_frames": self.processed_frames,
                            "dropped_frames": (
                                self.received_frames - self.processed_frames
                            ),
                        }
                    )
                )
            )

    def publish_detections(self, header, result):
        message = Detection2DArray()
        message.header = header
        for index, (score, label, box) in enumerate(
            zip(result["scores"], result["labels"], result["boxes"])
        ):
            class_name = self.model.config.id2label[int(label)]
            if class_name not in TARGET_CLASSES:
                continue
            x1, y1, x2, y2 = [float(value) for value in box]
            detection = Detection2D()
            detection.header = header
            detection.id = f"{class_name}-{index}"
            detection.bbox.center.position.x = (x1 + x2) / 2
            detection.bbox.center.position.y = (y1 + y2) / 2
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_name
            hypothesis.hypothesis.score = float(score)
            detection.results.append(hypothesis)
            message.detections.append(detection)
        self.detections_pub.publish(message)

    def destroy_node(self):
        self.finished = True
        with self.image_condition:
            self.image_condition.notify_all()
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = ObjectDetector()
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
