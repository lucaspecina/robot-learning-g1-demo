#!/usr/bin/env python3
"""Detector visual puntual para categorías descriptas con palabras."""
import io
import os
import threading
import time

DEFAULT_MODEL = "IDEA-Research/grounding-dino-tiny"
DEFAULT_REVISION = "a2bb814dd30d776dcf7e30523b00659f4f141c71"
DEFAULT_CONFIDENCE = 0.5
DEFAULT_TEXT_CONFIDENCE = 0.3
MAX_LABELS = 5
MAX_LABEL_LENGTH = 64


class InvalidDetectionRequestError(ValueError):
    pass


def validate_labels(labels) -> list[str]:
    """Limita y normaliza las categorías que llegan desde el robot."""
    if not isinstance(labels, list) or not 1 <= len(labels) <= MAX_LABELS:
        raise InvalidDetectionRequestError(
            f"labels debe contener entre 1 y {MAX_LABELS} elementos"
        )
    normalized = []
    for label in labels:
        if not isinstance(label, str):
            raise InvalidDetectionRequestError(
                "cada categoría debe ser una cadena"
            )
        value = " ".join(label.strip().lower().split())
        if not value or len(value) > MAX_LABEL_LENGTH:
            raise InvalidDetectionRequestError(
                f"cada categoría debe tener entre 1 y {MAX_LABEL_LENGTH} caracteres"
            )
        if value not in normalized:
            normalized.append(value)
    return normalized


def bounded_coordinates(box, image_width: int, image_height: int) -> list[float]:
    """Evita propagar cajas fuera de la imagen original."""
    x1, y1, x2, y2 = [float(value) for value in box]
    return [
        round(max(0.0, min(x1, image_width)), 1),
        round(max(0.0, min(y1, image_height)), 1),
        round(max(0.0, min(x2, image_width)), 1),
        round(max(0.0, min(y2, image_height)), 1),
    ]


class OpenVocabularyDetector:
    """Carga Grounding DINO una vez y serializa sus inferencias."""

    def __init__(
        self,
        model_name: str = None,
        model_revision: str = None,
        confidence: float = None,
        text_confidence: float = None,
    ):
        import torch
        from transformers import (
            AutoModelForZeroShotObjectDetection,
            AutoProcessor,
        )

        self.torch = torch
        self.model_name = model_name or os.environ.get(
            "OPEN_VOCAB_MODEL",
            DEFAULT_MODEL,
        )
        self.model_revision = model_revision or os.environ.get(
            "OPEN_VOCAB_MODEL_REVISION",
            DEFAULT_REVISION,
        )
        self.confidence = confidence or float(
            os.environ.get(
                "OPEN_VOCAB_CONFIDENCE",
                DEFAULT_CONFIDENCE,
            )
        )
        self.text_confidence = text_confidence or float(
            os.environ.get(
                "OPEN_VOCAB_TEXT_CONFIDENCE",
                DEFAULT_TEXT_CONFIDENCE,
            )
        )
        torch.set_num_threads(
            int(os.environ.get("OPEN_VOCAB_CPU_THREADS", "2"))
        )
        self.processor = AutoProcessor.from_pretrained(
            self.model_name,
            revision=self.model_revision,
            local_files_only=True,
        )
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(
            self.model_name,
            revision=self.model_revision,
            local_files_only=True,
        ).eval()
        # El servidor HTTP atiende en varios hilos, pero un único modelo no
        # debe ejecutar dos inferencias simultáneas y duplicar memoria.
        self.inference_lock = threading.Lock()

    def detect(self, image_bytes: bytes, labels) -> dict:
        from PIL import Image

        normalized_labels = validate_labels(labels)
        try:
            image = Image.open(io.BytesIO(image_bytes)).convert("RGB")
        except Exception as error:
            raise InvalidDetectionRequestError(
                "el JPEG no pudo abrirse como imagen"
            ) from error
        if image.width > 2048 or image.height > 2048:
            raise InvalidDetectionRequestError(
                "la imagen supera la resolución máxima"
            )

        inputs = self.processor(
            images=image,
            text=[normalized_labels],
            return_tensors="pt",
        )
        started_at = time.monotonic()
        with self.inference_lock, self.torch.inference_mode():
            outputs = self.model(**inputs)
        elapsed_s = time.monotonic() - started_at
        result = self.processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            threshold=self.confidence,
            text_threshold=self.text_confidence,
            target_sizes=[(image.height, image.width)],
        )[0]
        text_labels = result.get("text_labels", result.get("labels", []))
        detections = []
        for score, label, box in zip(
            result["scores"],
            text_labels,
            result["boxes"],
        ):
            coordinates = bounded_coordinates(
                box,
                image.width,
                image.height,
            )
            if coordinates[2] <= coordinates[0] or coordinates[3] <= coordinates[1]:
                continue
            detections.append(
                {
                    "label": str(label),
                    "confidence": round(float(score), 3),
                    "box": coordinates,
                }
            )
        return {
            "detections": detections,
            "image_width": image.width,
            "image_height": image.height,
            "model": self.model_name,
            "inference_s": round(elapsed_s, 3),
        }
