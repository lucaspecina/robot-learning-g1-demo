#!/usr/bin/env bash
# Enciende el laboratorio COMPLETO desde la laptop, con un solo comando:
#   VM de Azure → simulador Isaac (livestream) → Jupyter → túnel ssh.
# Correr desde Git Bash o WSL:  bash scripts/lab_up.sh
# La terminal queda siendo EL TÚNEL: dejarla abierta mientras trabajás.
#
# Seguridad: acá no hay secretos — la IP se consulta a Azure en vivo (no se
# hardcodea), la clave ssh vive en ~/.ssh, y el token de Jupyter solo sirve
# a través del túnel (el puerto 8888 de la VM escucha solo en localhost).
set -euo pipefail
RG=rg-go2-lab
VM=vm-go2-isaac
USER=lucas

echo ">> Encendiendo la VM (si ya está encendida, no hace nada)..."
az vm start -g "$RG" -n "$VM" --output none

IP=$(az vm show -g "$RG" -n "$VM" -d --query publicIps -o tsv)
echo ">> IP de la VM: $IP"

echo ">> Esperando a que el ssh responda..."
until ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new "$USER@$IP" true 2>/dev/null; do
    sleep 5; printf .
done
echo " ssh OK"

echo ">> Lanzando el simulador (si no está corriendo; primer boot ~2 min)..."
ssh "$USER@$IP" 'pgrep -f "isaac_go2_ros2.p[y]" >/dev/null || \
    (setsid nohup ~/go2-lab/launch_port.sh > ~/port_launch.log 2>&1 < /dev/null &); echo "   sim: ok"'

echo ">> Lanzando Jupyter (si no está corriendo)..."
ssh "$USER@$IP" 'ss -tln | grep -q 8888 || \
    (source /opt/ros/jazzy/setup.bash && setsid nohup ~/venvs/nb/bin/jupyter lab \
     --no-browser --port 8888 --ip 127.0.0.1 --ServerApp.token=go2lab \
     --notebook-dir ~/go2-lab/notebooks > ~/jupyter.log 2>&1 < /dev/null &); echo "   jupyter: ok"'

echo ""
echo "=================================================================="
echo "  Kernel para VSCode :  http://localhost:8888/?token=go2lab"
echo "  Cliente de Isaac   :  $IP   (dale ~2 min si recién arranca)"
echo "  Esta terminal ES el túnel — dejala abierta."
echo "=================================================================="
exec ssh -L 8888:localhost:8888 "$USER@$IP"
