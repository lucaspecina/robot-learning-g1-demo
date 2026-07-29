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

echo ">> Imagen del servidor de inteligencia..."
sudo docker build -q -t intelligence-server ./server

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
jetson_image=$(sudo docker image inspect jetson-sim --format '{{.Id}}')
running_jetson_image=$(
    sudo docker inspect jetson --format '{{.Image}}' 2>/dev/null || true
)
if ! sudo docker ps --format '{{.Names}}' | grep -q '^jetson$' \
    || [ "$running_jetson_image" != "$jetson_image" ]; then
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
server_image=$(sudo docker image inspect intelligence-server --format '{{.Id}}')
running_server_image=$(
    sudo docker inspect server --format '{{.Image}}' 2>/dev/null || true
)
if ! sudo docker ps --format '{{.Names}}' | grep -q '^server$' \
    || [ "$running_server_image" != "$server_image" ]; then
    sudo docker rm -f server >/dev/null 2>&1 || true
    server_env_args=()
    if [ -f /home/lucas/go2-lab/.env ]; then
        # El archivo pertenece a la VM y Git lo ignora. Docker recibe las
        # variables sin copiar secretos a la imagen ni a este script.
        server_env_args=(--env-file /home/lucas/go2-lab/.env)
    fi
    sudo docker run -d --restart unless-stopped --name server \
        --network robotnet --ip 172.30.0.20 \
        --cap-add=NET_ADMIN \
        "${server_env_args[@]}" \
        intelligence-server >/dev/null
    echo "   lanzado"
else
    echo "   ya corria"
fi

echo ""
echo "== estado =="
sudo docker ps --format '  {{.Names}}: {{.Status}}'
echo ""
echo "Listo. La jetson habla ROS 2 con Isaac; el servidor de inteligencia atiende"
echo "HTTP en 172.30.0.20:8000."
echo "Degradar el wifi:  sudo bash degrade_network.sh {clean|wifi|wifi-bad|outage|show}"
