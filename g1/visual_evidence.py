#!/usr/bin/env python3
"""Memoria acotada para unir una decisión con el cuadro que la originó."""
from collections import OrderedDict
from dataclasses import dataclass


MIN_JPEG_BYTES = 100
MAX_JPEG_BYTES = 1_500_000
CLOCK_CROP_TOPIC = "/g1/clock_crop/compressed"
VISUAL_EVIDENCE_TOPIC = "/g1/perception/evidence/compressed"
MODEL_INPUT_TOPIC = "/g1/model_input/compressed"


def validate_jpeg(data: bytes) -> bytes:
    """Rechaza buffers truncados antes de enviarlos fuera del robot."""
    if not isinstance(data, bytes):
        raise ValueError("la evidencia visual no es un buffer de bytes")
    if not MIN_JPEG_BYTES <= len(data) <= MAX_JPEG_BYTES:
        raise ValueError(
            f"la evidencia visual tiene un tamaño inválido: {len(data)} bytes"
        )
    if not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
        raise ValueError("la evidencia visual no es un JPEG completo")
    return data


def is_complete_jpeg(data: bytes) -> bool:
    try:
        validate_jpeg(data)
    except ValueError:
        return False
    return True


def image_ref(topic: str, header) -> dict:
    """Conserva la hora de adquisición que define sensor_msgs/CompressedImage."""
    return {
        "topic": topic,
        "sec": int(header.stamp.sec),
        "nanosec": int(header.stamp.nanosec),
    }


def image_ref_key(reference: dict) -> tuple[str, int, int]:
    if not isinstance(reference, dict):
        raise ValueError("la referencia visual no es un objeto")
    topic = reference.get("topic")
    sec = reference.get("sec")
    nanosec = reference.get("nanosec")
    if (
        not isinstance(topic, str)
        or not topic.startswith("/")
        or not isinstance(sec, int)
        or isinstance(sec, bool)
        or sec < 0
        or not isinstance(nanosec, int)
        or isinstance(nanosec, bool)
        or not 0 <= nanosec < 1_000_000_000
    ):
        raise ValueError("la referencia visual tiene una fecha inválida")
    return (topic, sec, nanosec)


@dataclass(frozen=True)
class VisualEvidence:
    """JPEG exacto y datos suficientes para probar de qué cuadro proviene."""

    jpeg: bytes
    sec: int
    nanosec: int
    received_at: float
    source_topic: str
    kind: str
    description: str
    detail: str = "low"

    def __post_init__(self):
        validate_jpeg(self.jpeg)
        if self.sec < 0 or not 0 <= self.nanosec < 1_000_000_000:
            raise ValueError("la marca temporal de la evidencia es inválida")
        if not self.source_topic.startswith("/"):
            raise ValueError("el origen de la evidencia no es un topic ROS")
        if not self.kind or not self.description:
            raise ValueError("la evidencia visual no está explicada")
        if self.detail not in {"low", "high", "auto"}:
            raise ValueError("el nivel de detalle visual es inválido")

    @property
    def key(self) -> tuple[str, int, int]:
        return (self.source_topic, self.sec, self.nanosec)

    def reference(self) -> dict:
        return {
            "topic": self.source_topic,
            "sec": self.sec,
            "nanosec": self.nanosec,
            "kind": self.kind,
            "description": self.description,
            "detail": self.detail,
            "bytes": len(self.jpeg),
        }


class VisualEvidenceCache:
    """Conserva pocos JPEG; una imagen vieja nunca se reutiliza en otro paso."""

    def __init__(self, max_items: int = 24):
        if max_items <= 0:
            raise ValueError("la memoria de evidencia debe conservar elementos")
        self.max_items = max_items
        self._items = OrderedDict()

    def add(self, evidence: VisualEvidence):
        self._items[evidence.key] = evidence
        self._items.move_to_end(evidence.key)
        while len(self._items) > self.max_items:
            self._items.popitem(last=False)

    def get(
        self,
        reference: dict,
        *,
        now: float,
        max_age_s: float,
    ) -> VisualEvidence | None:
        if not isinstance(reference, dict) or max_age_s <= 0:
            return None
        try:
            key = image_ref_key(reference)
        except ValueError:
            return None
        evidence = self._items.get(key)
        if evidence is None:
            return None
        age_s = now - evidence.received_at
        if age_s < 0 or age_s > max_age_s:
            return None
        return evidence
