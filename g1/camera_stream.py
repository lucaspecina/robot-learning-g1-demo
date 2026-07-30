#!/usr/bin/env python3
"""Unión exacta de color, profundidad y calibración por hora de captura."""
from collections import OrderedDict

import numpy as np


def stamp_key(message) -> tuple[int, int]:
    """Devuelve una clave estable tanto para un mensaje como para su cabecera."""
    header = getattr(message, "header", message)
    return (
        header.stamp.sec,
        header.stamp.nanosec,
    )


def color_array(message) -> np.ndarray:
    """Interpreta una imagen ROS rgb8 respetando el relleno de cada fila."""
    if message.encoding != "rgb8":
        raise ValueError(f"color inesperado: {message.encoding}")
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height,
        message.step,
    )
    return rows[:, :message.width * 3].reshape(
        message.height,
        message.width,
        3,
    )


def depth_array(message) -> np.ndarray:
    """Interpreta una imagen ROS 32FC1 en metros."""
    if message.encoding != "32FC1":
        raise ValueError(f"profundidad inesperada: {message.encoding}")
    item_size = np.dtype(np.float32).itemsize
    if message.step % item_size:
        raise ValueError("la fila de profundidad no está alineada")
    rows = np.frombuffer(message.data, dtype=np.float32).reshape(
        message.height,
        message.step // item_size,
    )
    return rows[:, :message.width]


class SynchronizedCameraFrames:
    """Conserva cuadros completos sin aproximar marcas de tiempo."""

    def __init__(self, max_frames: int = 120):
        if max_frames <= 0:
            raise ValueError("max_frames debe ser positivo")
        self.max_frames = max_frames
        self.frames = OrderedDict()

    def add(self, kind: str, message):
        if kind not in {"color", "depth", "info"}:
            raise ValueError(f"canal de cámara desconocido: {kind}")
        key = stamp_key(message)
        self.frames.setdefault(key, {})[kind] = message
        while len(self.frames) > self.max_frames:
            self.frames.popitem(last=False)

    def complete(self, message_or_header):
        frame = self.frames.get(stamp_key(message_or_header))
        if frame is None or set(frame) != {"color", "depth", "info"}:
            return None
        return frame

    def latest_complete(self):
        """Devuelve el cuadro completo más nuevo, si ya existe alguno."""
        for frame in reversed(self.frames.values()):
            if set(frame) == {"color", "depth", "info"}:
                return frame
        return None
