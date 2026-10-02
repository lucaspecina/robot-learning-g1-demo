#!/usr/bin/env bash
# Apaga el laboratorio completo: cierra procesos en la VM y la desasigna
# (deja de facturar compute; solo queda el disco).
# Correr desde Git Bash o WSL:  bash scripts/lab_down.sh
set -euo pipefail
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# Los nombres de los recursos de Azure no se versionan (el repo es público):
# viven en scripts/lab.env, que se arma copiando scripts/lab.env.example.
if [ ! -f "$SCRIPT_DIR/lab.env" ]; then
    echo "ERROR: falta scripts/lab.env (copiar scripts/lab.env.example y completar)." >&2
    exit 2
fi
source "$SCRIPT_DIR/lab.env"
RG=$AZURE_RESOURCE_GROUP
VM=$AZURE_VM
USER=lucas

# WSL y Windows guardan las claves SSH en lugares distintos. Al lanzar este
# script desde PowerShell reutilizamos el cliente que ya conoce la clave.
SSH=(ssh)
if grep -qi microsoft /proc/version 2>/dev/null && command -v ssh.exe >/dev/null; then
    SSH=(ssh.exe)
fi

IP=$(az vm show -g "$RG" -n "$VM" -d --query publicIps -o tsv 2>/dev/null | tr -d '\r' || true)
if [ -n "${IP:-}" ]; then
    echo ">> Cerrando procesos en la VM..."
    "${SSH[@]}" -o ConnectTimeout=10 "$USER@$IP" \
        'cd ~/go2-lab/g1 && bash run_demo.sh kill >/dev/null 2>&1 || true
         pkill -f "kit/ki[t]" 2>/dev/null || true
         echo "   procesos G1 cerrados"' || true
fi

echo ">> Apagando la VM (deallocate)..."
az vm deallocate -g "$RG" -n "$VM" --output none
echo "VM apagada. Facturación de compute: detenida."
