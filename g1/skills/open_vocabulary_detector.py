#!/usr/bin/env python3
"""Búsqueda visual puntual en el servidor, sin bloquear callbacks de ROS."""
import io
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np
from PIL import Image as PILImage

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from agent.intelligence_client import (  # noqa: E402
    IntelligenceClient,
    RemoteIntelligenceError,
)
from model_trace import build_model_event  # noqa: E402
from open_vocabulary_core import parse_search_request  # noqa: E402
from visual_evidence import (  # noqa: E402
    MODEL_INPUT_TOPIC,
    VISUAL_EVIDENCE_TOPIC,
    image_ref,
)

import rclpy  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from vision_msgs.msg import (  # noqa: E402
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

MAX_FRAME_AGE_S = 2.0
FRAME_WAIT_TIMEOUT_S = 6.0
MODEL_EVENT_QOS = QoSProfile(
    depth=20,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class OpenVocabularyDetector(Node):
    def __init__(self):
        super().__init__("open_vocabulary_detector")
        self.client = IntelligenceClient()
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.worker = None
        self.busy = False
        self.finished = False
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            "/g1/open_vocabulary_detections",
            qos_profile_sensor_data,
        )
        self.evidence_pub = self.create_publisher(
            CompressedImage,
            VISUAL_EVIDENCE_TOPIC,
            2,
        )
        self.model_input_pub = self.create_publisher(
            CompressedImage,
            MODEL_INPUT_TOPIC,
            2,
        )
        self.model_event_pub = self.create_publisher(
            String,
            "/g1/model_events",
            MODEL_EVENT_QOS,
        )
        self.status_pub = self.create_publisher(
            String,
            "/g1/perception/search_status",
            10,
        )
        self.create_subscription(
            Image,
            "/g1/head_cam/image",
            self.on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/g1/perception/search_request",
            self.on_request,
            10,
        )
        self.publish_status("ready")
        self.get_logger().info(
            "búsqueda visual puntual lista; el modelo vive en el servidor"
        )

    def publish_status(self, state: str, **fields):
        if self.finished or not rclpy.ok():
            return
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {"state": state, **fields},
                    ensure_ascii=False,
                )
            )
        )

    def publish_model_event(self, event: dict):
        if self.finished or not rclpy.ok():
            return
        self.model_event_pub.publish(
            String(data=json.dumps(event, ensure_ascii=False))
        )

    def on_image(self, msg: Image):
        if msg.encoding != "rgb8":
            return
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height,
            msg.width,
            3,
        ).copy()
        with self.image_lock:
            self.latest_image = (
                msg.header,
                rgb,
                time.monotonic(),
            )

    def on_request(self, msg: String):
        try:
            request_id, target, labels = parse_search_request(msg.data)
        except ValueError as error:
            self.publish_status("failed", error=str(error))
            return
        if self.busy:
            self.publish_status(
                "rejected",
                request_id=request_id,
                target=target,
                error="ya hay una búsqueda en curso",
            )
            return
        with self.image_lock:
            previous_frame_key = self.frame_key(
                self.latest_image[0]
                if self.latest_image is not None
                else None
            )
        self.busy = True
        self.worker = threading.Thread(
            target=self.run_search,
            args=(request_id, target, labels, previous_frame_key),
            daemon=True,
        )
        self.worker.start()

    @staticmethod
    def frame_key(header):
        if header is None:
            return None
        return (
            int(header.stamp.sec),
            int(header.stamp.nanosec),
        )

    def wait_for_fresh_image(self, previous_frame_key=None):
        """Espera un cuadro posterior al pedido sin bloquear callbacks."""
        deadline = time.monotonic() + FRAME_WAIT_TIMEOUT_S
        while time.monotonic() < deadline and not self.finished:
            with self.image_lock:
                image_entry = self.latest_image
            if (
                image_entry is not None
                and time.monotonic() - image_entry[2] <= MAX_FRAME_AGE_S
                and self.frame_key(image_entry[0]) != previous_frame_key
            ):
                return image_entry[0], image_entry[1]
            time.sleep(0.05)
        raise ValueError("no llegó una imagen reciente")

    def run_search(
        self,
        request_id,
        target,
        labels,
        previous_frame_key=None,
    ):
        started_at = time.monotonic()
        frame_reference = None
        model_input_reference = None
        model_input_payload = None
        model_event_id = None
        self.publish_status(
            "running",
            request_id=request_id,
            target=target,
            labels=labels,
        )
        try:
            header, rgb = self.wait_for_fresh_image(previous_frame_key)
            buffer = io.BytesIO()
            PILImage.fromarray(rgb).save(buffer, format="JPEG", quality=90)
            jpeg = buffer.getvalue()
            frame_reference = image_ref(VISUAL_EVIDENCE_TOPIC, header)
            evidence = CompressedImage()
            evidence.header = header
            evidence.format = "jpeg"
            evidence.data = jpeg
            self.evidence_pub.publish(evidence)
            # El tablero recibe exactamente el mismo buffer que sale por HTTP.
            # Publicarlo antes de llamar al servidor prueba qué vio el modelo,
            # incluso si la red se corta y nunca llega una respuesta.
            model_input = CompressedImage()
            model_input.header = header
            model_input.format = "jpeg"
            model_input.data = jpeg
            self.model_input_pub.publish(model_input)
            model_input_reference = {
                **image_ref(MODEL_INPUT_TOPIC, header),
                "source_topic": "/g1/head_cam/image",
                "purpose": f"buscar {target} con Grounding DINO",
                "bytes": len(jpeg),
            }
            model_input_payload = {
                "target": target,
                "labels": list(labels),
                "image_width": int(rgb.shape[1]),
                "image_height": int(rgb.shape[0]),
            }
            running_event = build_model_event(
                task="detect_objects",
                state="running",
                input_summary=(
                    f"cuadro JPEG {rgb.shape[1]}x{rgb.shape[0]} y categorías "
                    + ", ".join(labels)
                ),
                input_ref=model_input_reference,
                input_payload=model_input_payload,
            )
            model_event_id = running_event["event_id"]
            self.publish_model_event(running_event)
            result = self.client.detect_objects(jpeg, labels)
            if (
                result["image_width"] != rgb.shape[1]
                or result["image_height"] != rgb.shape[0]
            ):
                raise RemoteIntelligenceError(
                    "el servidor respondió para otra resolución"
                )
            message = self.make_detection_message(
                header,
                request_id,
                result["detections"],
            )
            if self.finished or not rclpy.ok():
                return
            self.detections_pub.publish(message)
            validated_output = {
                "target": target,
                "detections": result["detections"],
                "image_width": result["image_width"],
                "image_height": result["image_height"],
            }
            self.publish_model_event(
                build_model_event(
                    event_id=model_event_id,
                    request_id=result.get("request_id"),
                    task="detect_objects",
                    state="succeeded",
                    input_summary=(
                        f"cuadro JPEG {rgb.shape[1]}x{rgb.shape[0]} y "
                        "categorías " + ", ".join(labels)
                    ),
                    input_ref=model_input_reference,
                    input_payload=model_input_payload,
                    model=result.get("model"),
                    raw_output=result["raw_output"],
                    validated_output=validated_output,
                    duration_s=round(
                        time.monotonic() - started_at,
                        3,
                    ),
                )
            )
            self.publish_status(
                "complete",
                request_id=request_id,
                target=target,
                frame_ref=frame_reference,
                count=len(message.detections),
                elapsed_s=round(time.monotonic() - started_at, 3),
                inference_s=result.get("inference_s"),
                network_total_s=result.get("elapsed_s"),
                model=result.get("model"),
            )
        except (RemoteIntelligenceError, OSError, ValueError) as error:
            if model_event_id is not None:
                self.publish_model_event(
                    build_model_event(
                        event_id=model_event_id,
                        request_id=getattr(error, "request_id", None),
                        task="detect_objects",
                        state="failed",
                        input_summary="cuadro JPEG y categorías de búsqueda",
                        input_ref=model_input_reference,
                        input_payload=model_input_payload,
                        raw_output=getattr(error, "raw_output", None),
                        duration_s=round(
                            time.monotonic() - started_at,
                            3,
                        ),
                        error=str(error),
                    )
                )
            fields = {
                "request_id": request_id,
                "target": target,
                "elapsed_s": round(time.monotonic() - started_at, 3),
                "error": str(error),
            }
            if frame_reference is not None:
                fields["frame_ref"] = frame_reference
            self.publish_status("failed", **fields)
        finally:
            self.busy = False

    @staticmethod
    def make_detection_message(header, request_id, detections):
        message = Detection2DArray()
        message.header = header
        for index, item in enumerate(detections):
            x1, y1, x2, y2 = item["box"]
            detection = Detection2D()
            detection.header = header
            detection.id = f"grounding_dino:{request_id}:{index}"
            detection.bbox.center.position.x = (x1 + x2) / 2
            detection.bbox.center.position.y = (y1 + y2) / 2
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = item["label"]
            hypothesis.hypothesis.score = item["confidence"]
            detection.results.append(hypothesis)
            message.detections.append(detection)
        return message

    def destroy_node(self):
        self.finished = True
        if self.worker is not None and self.worker.is_alive():
            self.worker.join(timeout=1.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = OpenVocabularyDetector()
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
