#!/usr/bin/env python3
"""Servicio HTTP minimo que ocupa el lugar del agente en el servidor.

Todavia no piensa nada: responde un JSON con su reloj. Sirve para medir la
latencia real del enlace jetson <-> servidor, que es el que simula el wifi.
Cuando el agente de verdad exista, reemplaza a este stub sin cambiar la
direccion ni el puerto.

Uso (dentro del contenedor servidor):  python3 agente_stub.py
"""
import json
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

PUERTO = 8000


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        cuerpo = json.dumps({"ok": True, "t_servidor": time.time()}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(cuerpo)))
        self.end_headers()
        self.wfile.write(cuerpo)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        recibido = self.rfile.read(n)
        cuerpo = json.dumps({"ok": True, "bytes_recibidos": len(recibido)}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(cuerpo)))
        self.end_headers()
        self.wfile.write(cuerpo)

    def log_message(self, *args):
        pass  # sin ruido en la salida


if __name__ == "__main__":
    print(f"agente_stub escuchando en 0.0.0.0:{PUERTO}", flush=True)
    HTTPServer(("0.0.0.0", PUERTO), Handler).serve_forever()
