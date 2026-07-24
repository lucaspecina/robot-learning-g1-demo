#!/usr/bin/env bash
# Apaga el laboratorio completo: cierra procesos en la VM y la desasigna
# (deja de facturar compute; solo queda el disco).
# Correr desde Git Bash o WSL:  bash scripts/lab_down.sh
set -euo pipefail
RG=rg-go2-lab
VM=vm-go2-isaac
USER=lucas

IP=$(az vm show -g "$RG" -n "$VM" -d --query publicIps -o tsv 2>/dev/null || true)
if [ -n "${IP:-}" ]; then
    echo ">> Cerrando procesos en la VM..."
    ssh -o ConnectTimeout=10 "$USER@$IP" \
        'pkill -f "isaac_go2_ros2.p[y]"; pkill -f jupyter-lab; pkill -f "kit/ki[t]"; echo "   procesos cerrados"' || true
fi

echo ">> Apagando la VM (deallocate)..."
az vm deallocate -g "$RG" -n "$VM" --output none
echo "VM apagada. Facturación de compute: detenida."
