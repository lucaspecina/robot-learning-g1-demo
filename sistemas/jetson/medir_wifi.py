#!/usr/bin/env python3
"""Mide el enlace jetson -> servidor (el "wifi" simulado) con pedidos HTTP.

Hace N pedidos GET al stub del agente y reporta la distribucion de latencias
y las fallas. Correrlo con distintos perfiles de red_degradar.sh dibuja la
curva de degradacion: que se siente en cada calidad de wifi.

Uso (dentro del contenedor jetson):  python3 medir_wifi.py [N]
"""
import sys
import time

import requests

URL = "http://172.30.0.20:8000/"
N = int(sys.argv[1]) if len(sys.argv) > 1 else 30
TIMEOUT_S = 2.0

lat_ms = []
fallas = 0
for _ in range(N):
    t0 = time.perf_counter()
    try:
        r = requests.get(URL, timeout=TIMEOUT_S)
        r.raise_for_status()
        lat_ms.append((time.perf_counter() - t0) * 1000)
    except Exception:
        fallas += 1
    time.sleep(0.05)

if lat_ms:
    lat_ms.sort()
    p50 = lat_ms[len(lat_ms) // 2]
    p95 = lat_ms[int(len(lat_ms) * 0.95) - 1]
    print(f"pedidos: {N}  ok: {len(lat_ms)}  fallas: {fallas}")
    print(f"latencia ms  p50: {p50:6.1f}   p95: {p95:6.1f}   max: {lat_ms[-1]:6.1f}")
else:
    print(f"pedidos: {N}  ok: 0  fallas: {fallas}  -> ENLACE MUERTO")
