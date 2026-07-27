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

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Que buscamos: nombre -> (color de referencia RGB, tolerancia)
# La tolerancia es cuanta distancia de color aceptamos; mas alta = mas permisivo.
TARGETS = {
    "persona_roja": (np.array([190, 30, 30]), 70),
    "persona_azul": (np.array([30, 50, 190]), 70),
    "botella": (np.array([40, 140, 65]), 60),
    "reloj": (np.array([242, 242, 230]), 35),
}

AREA_MINIMA = 0.002   # menos que esto es ruido, no un objeto


class ColorDetector(Node):
    def __init__(self):
        super().__init__("detector")
        self.pub = self.create_publisher(String, "/g1/detections", 10)
        self.create_subscription(Image, "/g1/head_cam/image", self.on_image, 1)
        self.get_logger().info("detector listo, escuchando /g1/head_cam/image")

    def on_image(self, msg: Image):
        if msg.encoding != "rgb8":
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        img = img.astype(np.int16)

        detecciones = {}
        total_px = msg.height * msg.width

        for nombre, (color, tolerancia) in TARGETS.items():
            # Distancia de cada pixel al color buscado.
            distancia = np.sqrt(((img - color) ** 2).sum(axis=2))
            mascara = distancia < tolerancia
            cuenta = int(mascara.sum())
            area = cuenta / total_px
            if area < AREA_MINIMA:
                continue

            # Centro horizontal de los pixeles encontrados, normalizado.
            columnas = np.where(mascara.any(axis=0))[0]
            cx = float(columnas.mean() / msg.width)
            detecciones[nombre] = {"cx": round(cx, 3), "area": round(area, 4)}

        self.pub.publish(String(data=json.dumps(detecciones)))


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
