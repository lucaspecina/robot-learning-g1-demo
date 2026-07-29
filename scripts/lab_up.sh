#!/usr/bin/env bash
# Enciende el laboratorio G1 COMPLETO desde la laptop, con un solo comando:
#   VM de Azure → demo G1 (Isaac + contenedores) → túnel del tablero + ssh.
# Correr desde Git Bash o WSL:  bash scripts/lab_up.sh
# La terminal queda siendo EL TÚNEL: dejarla abierta mientras trabajás.
#
# Seguridad: la IP se consulta a Azure en vivo y la clave ssh vive en ~/.ssh.
set -euo pipefail
RG=rg-go2-lab
VM=vm-go2-isaac
USER=lucas

echo ">> Encendiendo la VM (si ya está encendida, no hace nada)..."
az vm start -g "$RG" -n "$VM" --output none

IP=$(az vm show -g "$RG" -n "$VM" -d --query publicIps -o tsv | tr -d '\r')
echo ">> IP de la VM: $IP"

echo ">> Esperando a que el ssh responda..."
until ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new "$USER@$IP" true 2>/dev/null; do
    sleep 5; printf .
done
echo " ssh OK"

echo ">> Lanzando la demo G1 completa (primer arranque ~2 min)..."
ssh "$USER@$IP" 'cd ~/go2-lab/g1 && bash run_demo.sh up'

echo ""
echo "=================================================================="
echo "  Tablero             :  http://localhost:8080"
echo "  Cliente de Isaac    :  $IP:49100"
echo "  Esta terminal ES el túnel — dejala abierta."
echo "=================================================================="
exec ssh -L 8080:localhost:8080 "$USER@$IP"
