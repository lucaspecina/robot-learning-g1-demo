#!/usr/bin/env bash
# Levanta los sistemas simulados: los contenedores jetson y server, con sus
# redes y limites. Idempotente: si algo ya esta, lo saltea.
#
# Uso (en la VM):  bash systems_up.sh
#
# Topologia:
#   Isaac (host, nativo) <--UDP DDS--> jetson (contenedor, red del host,
#        2 CPU / 8 GB)                  = "cable interno" del robot
#   jetson <--HTTP--> server (contenedor, red bridge aparte 172.30.0.0/16)
#        = "wifi", degradable con degrade_network.sh
set -euo pipefail
cd "$(dirname "$0")"

echo ">> Imagen de la jetson (build solo si cambio el Dockerfile)..."
sudo docker build -q -t jetson-sim ./jetson

echo ">> Red del wifi simulado..."
if sudo docker network inspect robotnet >/dev/null 2>&1; then
    echo "   ya existia"
else
    # Si la creacion falla (p.ej. otra red ocupa la misma subred) hay que
    # enterarse: sin la red, los contenedores no arrancan.
    sudo docker network create --subnet 172.30.0.0/16 robotnet >/dev/null
    echo "   creada"
fi

echo ">> Contenedor jetson (2 CPU, 8 GB, red del host)..."
if ! sudo docker ps --format '{{.Names}}' | grep -q '^jetson$'; then
    sudo docker rm -f jetson >/dev/null 2>&1 || true
    sudo docker run -d --restart unless-stopped --name jetson \
        --network host \
        --cpus=2 --memory=8g \
        --cap-add=NET_ADMIN \
        -v /home/lucas/go2-lab:/workspace -w /workspace \
        jetson-sim >/dev/null
    echo "   lanzado"
else
    echo "   ya corria"
fi

echo ">> Contenedor server (red aparte, IP fija 172.30.0.20)..."
if ! sudo docker ps --format '{{.Names}}' | grep -q '^server$'; then
    sudo docker rm -f server >/dev/null 2>&1 || true
    sudo docker run -d --restart unless-stopped --name server \
        --network robotnet --ip 172.30.0.20 \
        --cap-add=NET_ADMIN \
        -v /home/lucas/go2-lab/systems/server:/app -w /app \
        python:3.12-slim sleep infinity >/dev/null
    sudo docker exec server sh -c \
        "apt-get update -qq >/dev/null && apt-get install -y -qq iproute2 curl >/dev/null 2>&1"
    echo "   lanzado"
else
    echo "   ya corria"
fi

echo ">> Stub del agente en el server (puerto 8000)..."
if ! sudo docker exec server sh -c "ls /tmp/agent.pid 2>/dev/null" >/dev/null 2>&1; then
    sudo docker exec -d server sh -c \
        "python3 /app/agent_stub.py > /tmp/agent.log 2>&1 & echo \$! > /tmp/agent.pid"
    echo "   lanzado"
else
    echo "   ya corria"
fi

echo ""
echo "== estado =="
sudo docker ps --format '  {{.Names}}: {{.Status}}'
echo ""
echo "Listo. La jetson habla ROS 2 con Isaac; el server atiende HTTP en 172.30.0.20:8000."
echo "Degradar el wifi:  sudo bash degrade_network.sh {clean|wifi|wifi-bad|outage|show}"
