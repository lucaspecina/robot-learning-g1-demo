#!/usr/bin/env bash
# Apaga el laboratorio completo: cierra procesos en la VM y la desasigna
# (deja de facturar compute; solo queda el disco).
# Correr desde Git Bash o WSL:  bash scripts/lab_down.sh
set -euo pipefail
RG=rg-go2-lab
VM=vm-go2-isaac
USER=lucas

IP=$(az vm show -g "$RG" -n "$VM" -d --query publicIps -o tsv 2>/dev/null | tr -d '\r' || true)
if [ -n "${IP:-}" ]; then
    echo ">> Cerrando procesos en la VM..."
    ssh -o ConnectTimeout=10 "$USER@$IP" \
        'cd ~/go2-lab/g1 && bash run_demo.sh kill >/dev/null 2>&1 || true
         pkill -f "kit/ki[t]" 2>/dev/null || true
         echo "   procesos G1 cerrados"' || true
fi

echo ">> Apagando la VM (deallocate)..."
az vm deallocate -g "$RG" -n "$VM" --output none
echo "VM apagada. Facturación de compute: detenida."
