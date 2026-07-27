#!/usr/bin/env python3
"""Servicio HTTP minimo que ocupa el lugar del agente en el server.

Todavia no piensa nada: responde un JSON con su reloj. Sirve para medir la
latencia real del enlace jetson <-> server, que es el que simula el wifi.
Cuando el agente de verdad exista, reemplaza a este stub sin cambiar la
direccion ni el puerto.

Uso (dentro del contenedor server):  python3 agent_stub.py
"""
import json
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

PORT = 8000


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        body = json.dumps({"ok": True, "server_time": time.time()}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        received = self.rfile.read(n)
        body = json.dumps({"ok": True, "bytes_received": len(received)}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass  # sin ruido en la salida


if __name__ == "__main__":
    print(f"agent_stub escuchando en 0.0.0.0:{PORT}", flush=True)
    HTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
