#!/usr/bin/env python3
"""Percepcion: encuentra objetos por color en lo que ve la camara.

Consume `/g1/head_cam/image` y publica en `/g1/detections` lo que reconoce, con
su posicion en la imagen y que tan grande se ve:

    {"botella": {"cx": 0.52, "area": 0.031}, "persona_roja": {...}}

  cx    posicion horizontal en la imagen, de 0 (borde izquierdo) a 1 (derecho).
        0.5 es "justo al frente" — es lo que usa el robot para girar hacia algo.
  area  fraccion de la imagen que ocupa. Sirve de estimador grosero de
        distancia: mas grande = mas cerca.

Por que deteccion por color y no un detector neuronal: en esta escena los
objetos SON colores planos y distinguibles, asi que el color resuelve el
problema completo con unas pocas cuentas, sin modelos ni GPU ni instalaciones.
Y lo importante es que **el contrato de salida es el mismo** que tendria un
detector de verdad: cuando haga falta reconocer objetos reales, se reemplaza
este nodo por uno con YOLO o un modelo con vision, publicando en el mismo
canal, y nada de lo que esta arriba se entera.

Uso (dentro del contenedor jetson):
    python3 detector.py
"""
import json
import io

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

# Qué buscamos. El color solo propone candidatos; área y forma evitan que un
# fondo entero se convierta en una detección válida, como ocurrió con el panel
# blanco del primer reloj.
TARGETS = {
    "persona_roja": {
        "color": np.array([190, 30, 30]),
        "tolerance": 70,
        "min_aspect": 0.10,
        "max_aspect": 0.85,
        "max_area": 0.40,
    },
    "persona_azul": {
        "color": np.array([30, 50, 190]),
        "tolerance": 70,
        "min_aspect": 0.10,
        "max_aspect": 0.85,
        "max_area": 0.40,
    },
    "botella": {
        "color": np.array([40, 140, 65]),
        "tolerance": 60,
        "min_aspect": 0.10,
        "max_aspect": 0.85,
        "max_area": 0.12,
    },
    # El display y la botella comparten tonos verdes. La forma ancha del reloj
    # y la forma alta de la botella los separan en esta escena controlada. En
    # el robot real este nodo se reemplaza por un detector visual entrenado.
    "reloj": {
        "color": np.array([40, 145, 80]),
        "tolerance": 70,
        "min_aspect": 1.50,
        "max_aspect": 5.00,
        "max_area": 0.15,
    },
}

MIN_AREA = 0.001


class ColorDetector(Node):
    def __init__(self):
        super().__init__("detector")
        self.pub = self.create_publisher(String, "/g1/detections", 10)
        self.clock_crop_pub = self.create_publisher(
            CompressedImage,
            "/g1/clock_crop/compressed",
            2,
        )
        self.create_subscription(Image, "/g1/head_cam/image", self.on_image, 1)
        self.get_logger().info("detector listo, escuchando /g1/head_cam/image")

    def on_image(self, msg: Image):
        if msg.encoding != "rgb8":
            return
        rgb_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height,
            msg.width,
            3,
        )
        img = rgb_image.astype(np.int16)

        detections = {}
        total_px = msg.height * msg.width

        for name, target in TARGETS.items():
            # Distancia de cada pixel al color buscado.
            distance = np.sqrt(((img - target["color"]) ** 2).sum(axis=2))
            mask = distance < target["tolerance"]
            pixel_count = int(mask.sum())
            area = pixel_count / total_px
            if area < MIN_AREA or area > target["max_area"]:
                continue

            rows = np.where(mask.any(axis=1))[0]
            columns = np.where(mask.any(axis=0))[0]
            box_width = int(columns[-1] - columns[0] + 1)
            box_height = int(rows[-1] - rows[0] + 1)
            aspect = box_width / box_height
            if not target["min_aspect"] <= aspect <= target["max_aspect"]:
                continue

            # El centro de la caja es estable aunque una cifra tenga más
            # segmentos encendidos que otra.
            center_x = float((columns[0] + columns[-1]) / 2.0 / msg.width)
            detections[name] = {
                "cx": round(center_x, 3),
                "area": round(area, 4),
                "aspect": round(aspect, 2),
            }
            if name == "reloj":
                self.publish_clock_crop(
                    msg,
                    rgb_image,
                    rows,
                    columns,
                )

        self.pub.publish(String(data=json.dumps(detections)))

    def publish_clock_crop(
        self,
        source_msg: Image,
        rgb_image: np.ndarray,
        rows: np.ndarray,
        columns: np.ndarray,
    ):
        # La llamada remota necesita el reloj, no el cuadro completo. El margen
        # conserva el panel alrededor de los segmentos y reduce ancho de banda.
        box_width = int(columns[-1] - columns[0] + 1)
        box_height = int(rows[-1] - rows[0] + 1)
        horizontal_padding = max(4, round(box_width * 0.15))
        vertical_padding = max(4, round(box_height * 0.60))
        x0 = max(0, int(columns[0]) - horizontal_padding)
        x1 = min(source_msg.width, int(columns[-1]) + horizontal_padding + 1)
        y0 = max(0, int(rows[0]) - vertical_padding)
        y1 = min(source_msg.height, int(rows[-1]) + vertical_padding + 1)
        crop = rgb_image[y0:y1, x0:x1]

        from PIL import Image as PILImage

        buffer = io.BytesIO()
        PILImage.fromarray(crop).save(
            buffer,
            format="JPEG",
            quality=90,
        )
        compressed = CompressedImage()
        compressed.header = source_msg.header
        compressed.format = "jpeg"
        compressed.data = buffer.getvalue()
        self.clock_crop_pub.publish(compressed)


def main():
    rclpy.init()
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
