#!/usr/bin/env bash
# Levanta los "sistemas" simulados: el contenedor jetson y el contenedor
# servidor, con sus redes y limites. Idempotente: si algo ya esta, lo salta.
#
# Uso (en la VM):  bash sistemas_up.sh
#
# Topologia:
#   Isaac (host, nativo) <--UDP DDS--> jetson (contenedor, red del host,
#        2 CPU / 8 GB)                  = "cable interno" del robot
#   jetson <--HTTP--> servidor (contenedor, red bridge aparte 172.30.0.0/16)
#        = "wifi", degradable con red_degradar.sh
set -euo pipefail
cd "$(dirname "$0")"

echo ">> Imagen de la jetson (build solo si cambio el Dockerfile)..."
sudo docker build -q -t jetson-sim ./jetson

echo ">> Red interna del wifi simulado..."
sudo docker network create --subnet 172.30.0.0/16 rednet_interna 2>/dev/null \
    && echo "   creada" || echo "   ya existia"

echo ">> Contenedor jetson (2 CPU, 8 GB, red del host)..."
if ! sudo docker ps --format '{{.Names}}' | grep -q '^jetson$'; then
    sudo docker rm -f jetson >/dev/null 2>&1 || true
    sudo docker run -d --name jetson \
        --network host \
        --cpus=2 --memory=8g \
        --cap-add=NET_ADMIN \
        -v /home/lucas/go2-lab:/workspace -w /workspace \
        jetson-sim >/dev/null
    echo "   lanzado"
else
    echo "   ya corria"
fi

echo ">> Contenedor servidor (red aparte, IP fija 172.30.0.20)..."
if ! sudo docker ps --format '{{.Names}}' | grep -q '^servidor$'; then
    sudo docker rm -f servidor >/dev/null 2>&1 || true
    sudo docker run -d --name servidor \
        --network rednet_interna --ip 172.30.0.20 \
        --cap-add=NET_ADMIN \
        -v /home/lucas/go2-lab/sistemas/servidor:/app -w /app \
        python:3.12-slim sleep infinity >/dev/null
    sudo docker exec servidor sh -c \
        "apt-get update -qq >/dev/null && apt-get install -y -qq iproute2 curl >/dev/null"
    echo "   lanzado"
else
    echo "   ya corria"
fi

echo ">> Stub del agente en el servidor (puerto 8000)..."
if ! sudo docker exec servidor sh -c "ls /tmp/agente.pid 2>/dev/null" >/dev/null 2>&1; then
    sudo docker exec -d servidor sh -c \
        "python3 /app/agente_stub.py > /tmp/agente.log 2>&1 & echo \$! > /tmp/agente.pid"
    echo "   lanzado"
else
    echo "   ya corria"
fi

echo ""
echo "== estado =="
sudo docker ps --format '  {{.Names}}: {{.Status}}'
echo ""
echo "Listo. La jetson habla ROS 2 con Isaac; el servidor atiende HTTP en 172.30.0.20:8000."
echo "Degradar el wifi:  sudo bash red_degradar.sh {limpio|wifi|wifi-malo|corte|ver}"
