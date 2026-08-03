#!/usr/bin/env python3
"""Operaciones puras compartidas por la percepción visual."""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ImageBox:
    """Rectángulo en píxeles, limitado a los bordes de una imagen."""

    x1: int
    y1: int
    x2: int
    y2: int

    @property
    def width(self) -> int:
        return self.x2 - self.x1

    @property
    def height(self) -> int:
        return self.y2 - self.y1


CLASS_NAMES = {
    "clock": "reloj",
    "a digital wall clock": "reloj",
    "bottle": "botella",
    # El objeto de la demo es un cilindro liso: en seis cuadros RT-DETR lo
    # clasificó de forma estable como cup (0,49--0,62), no como bottle
    # (0,15--0,17). Ambos nombres representan el mismo rol físico transportable.
    "cup": "objeto",
    # El checkpoint fijado usa la etiqueta de Pascal VOC sin espacio. El
    # paquete acelerado puede usar la variante de COCO; aceptar ambas mantiene
    # estable el contrato ROS al cambiar el backend.
    "diningtable": "mesa",
    "dining table": "mesa",
    "a red table": "mesa",
    "a blue table": "mesa",
    "a table": "mesa",
}
TRANSPORT_OBJECT_CLASS_NAMES = {
    "bottle",
    "cup",
}
CLOCK_CLASS_NAMES = {"clock", "a digital wall clock"}
TABLE_CLASS_NAMES = {
    "diningtable",
    "dining table",
    "a red table",
    "a blue table",
    "a table",
}
SOURCE_TTL_S = {
    "rtdetr": 4.0,
    "grounding_dino": 8.0,
}


def color_pixel_counts(rgb_image: np.ndarray) -> dict[str, int]:
    """Cuenta píxeles rojos y azules como señal barata, no como detección."""
    if rgb_image.ndim != 3 or rgb_image.shape[2] != 3:
        raise ValueError("la imagen de color debe tener tres canales")
    rgb = rgb_image.astype(np.int16)
    red = rgb[:, :, 0]
    blue = rgb[:, :, 2]
    return {
        "red": int(np.count_nonzero((red > 80) & (red > blue * 1.35))),
        "blue": int(np.count_nonzero((blue > 80) & (blue > red * 1.35))),
    }


def bounded_box(
    center_x: float,
    center_y: float,
    size_x: float,
    size_y: float,
    image_width: int,
    image_height: int,
) -> ImageBox:
    """Convierte una caja central a límites enteros válidos."""
    x1 = max(0, round(center_x - size_x / 2))
    y1 = max(0, round(center_y - size_y / 2))
    x2 = min(image_width, round(center_x + size_x / 2))
    y2 = min(image_height, round(center_y + size_y / 2))
    return ImageBox(x1=x1, y1=y1, x2=x2, y2=y2)


def padded_box(
    box: ImageBox,
    image_width: int,
    image_height: int,
    horizontal_ratio: float = 0.15,
    vertical_ratio: float = 0.60,
) -> ImageBox:
    """Agrega contexto sin salirse de la imagen."""
    horizontal = max(4, round(box.width * horizontal_ratio))
    vertical = max(4, round(box.height * vertical_ratio))
    return ImageBox(
        x1=max(0, box.x1 - horizontal),
        y1=max(0, box.y1 - vertical),
        x2=min(image_width, box.x2 + horizontal),
        y2=min(image_height, box.y2 + vertical),
    )


def classify_table_color(rgb_image: np.ndarray, box: ImageBox) -> str:
    """Distingue el atributo rojo/azul dentro de una mesa ya detectada."""
    crop = rgb_image[box.y1:box.y2, box.x1:box.x2].astype(np.int16)
    if crop.size == 0:
        return "mesa"
    red = crop[:, :, 0]
    blue = crop[:, :, 2]
    red_fraction = float(((red > 80) & (red > blue * 1.35)).mean())
    blue_fraction = float(((blue > 80) & (blue > red * 1.35)).mean())
    if red_fraction >= 0.05 and red_fraction > blue_fraction * 1.5:
        return "mesa_roja"
    if blue_fraction >= 0.05 and blue_fraction > red_fraction * 1.5:
        return "mesa_azul"
    return "mesa"


def legacy_detection(
    class_name: str,
    score: float,
    box: ImageBox,
    image_width: int,
    image_height: int,
    source: str = "rtdetr",
) -> dict:
    """Construye la vista compacta que todavía consumen agente y tablero."""
    return {
        "cx": round((box.x1 + box.x2) / 2 / image_width, 3),
        "area": round(
            box.width * box.height / (image_width * image_height),
            4,
        ),
        "confidence": round(score, 3),
        "source": source,
        "class": class_name,
    }


def merge_source_detections(source_outputs: dict, now: float) -> dict:
    """Combina resultados recientes sin que una fuente borre a otra."""
    merged = {}
    for source, (received_at, detections) in source_outputs.items():
        if now - received_at <= SOURCE_TTL_S[source]:
            merged.update(detections)
    return merged
