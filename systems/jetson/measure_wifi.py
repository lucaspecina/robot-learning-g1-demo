#!/usr/bin/env python3
"""Mide el enlace jetson -> server (el "wifi" simulado) con pedidos HTTP.

Hace N pedidos GET al stub del agente y reporta la distribucion de latencias
y las fallas. Correrlo con distintos perfiles de degrade_network.sh dibuja la
curva de degradacion: que se siente en cada calidad de wifi.

Uso (dentro del contenedor jetson):  python3 measure_wifi.py [N]
"""
import sys
import time

import requests

URL = "http://172.30.0.20:8000/"
N = int(sys.argv[1]) if len(sys.argv) > 1 else 30
TIMEOUT_S = 2.0

latencies_ms = []
failures = 0
for _ in range(N):
    t0 = time.perf_counter()
    try:
        response = requests.get(URL, timeout=TIMEOUT_S)
        response.raise_for_status()
        latencies_ms.append((time.perf_counter() - t0) * 1000)
    except Exception:
        failures += 1
    time.sleep(0.05)

if latencies_ms:
    latencies_ms.sort()
    p50 = latencies_ms[len(latencies_ms) // 2]
    p95 = latencies_ms[int(len(latencies_ms) * 0.95) - 1]
    print(f"pedidos: {N}  ok: {len(latencies_ms)}  fallas: {failures}")
    print(f"latencia ms  p50: {p50:6.1f}   p95: {p95:6.1f}   max: {latencies_ms[-1]:6.1f}")
else:
    print(f"pedidos: {N}  ok: 0  fallas: {failures}  -> ENLACE MUERTO")
