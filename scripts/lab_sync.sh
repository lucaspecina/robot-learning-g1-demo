#!/usr/bin/env bash
# Sincroniza la VM con el commit publicado de la rama local.
# No copia archivos sueltos ni mezcla cambios: ante cualquier diferencia,
# se detiene para que la versión ejecutada siga siendo auditable.
set -euo pipefail

AZURE_RESOURCE_GROUP=rg-go2-lab
AZURE_VM=vm-go2-isaac
SSH_USER=lucas
REMOTE_REPO=/home/lucas/go2-lab/robot-learning-g1-demo
ACTIVE_DEMO=/home/lucas/go2-lab/g1

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(git -C "$SCRIPT_DIR/.." rev-parse --show-toplevel)
LOCAL_BRANCH=$(git -C "$REPO_ROOT" branch --show-current)
LOCAL_HEAD=$(git -C "$REPO_ROOT" rev-parse HEAD)

if [ -z "$LOCAL_BRANCH" ]; then
    echo "ERROR: la copia de Windows no está sobre una rama." >&2
    exit 2
fi

REMOTE_HEAD=$(
    git -C "$REPO_ROOT" ls-remote origin "refs/heads/$LOCAL_BRANCH" \
        | awk 'NR == 1 {print $1}'
)
if [ -z "$REMOTE_HEAD" ]; then
    echo "ERROR: la rama $LOCAL_BRANCH todavía no existe en GitHub." >&2
    exit 2
fi
if [ "$LOCAL_HEAD" != "$REMOTE_HEAD" ]; then
    echo "ERROR: el commit local $LOCAL_HEAD todavía no está publicado." >&2
    echo "Hacé push de $LOCAL_BRANCH antes de actualizar la VM." >&2
    exit 3
fi
if [ -n "$(git -C "$REPO_ROOT" status --porcelain)" ]; then
    echo "AVISO: hay cambios locales sin commit; no se copiarán a la VM."
fi

SSH=(ssh)
if grep -qi microsoft /proc/version 2>/dev/null && command -v ssh.exe >/dev/null; then
    SSH=(ssh.exe)
fi

VM_IP=${1:-}
if [ -z "$VM_IP" ]; then
    VM_IP=$(
        az vm show \
            -g "$AZURE_RESOURCE_GROUP" \
            -n "$AZURE_VM" \
            -d \
            --query publicIps \
            -o tsv \
            | tr -d '\r'
    )
fi
if [ -z "$VM_IP" ]; then
    echo "ERROR: Azure no devolvió una IP para la VM." >&2
    exit 4
fi

"${SSH[@]}" "$SSH_USER@$VM_IP" bash -s -- \
    "$LOCAL_BRANCH" "$LOCAL_HEAD" "$REMOTE_REPO" "$ACTIVE_DEMO" <<'REMOTE'
set -euo pipefail
expected_branch=$1
expected_head=$2
remote_repo=$3
active_demo=$4

if [ ! -d "$remote_repo/.git" ]; then
    echo "ERROR: la VM no contiene el clon esperado: $remote_repo" >&2
    exit 10
fi
if [ -n "$(git -C "$remote_repo" status --porcelain)" ]; then
    echo "ERROR: el clon de la VM tiene cambios locales:" >&2
    git -C "$remote_repo" status --short >&2
    exit 11
fi

vm_branch=$(git -C "$remote_repo" branch --show-current)
if [ "$vm_branch" != "$expected_branch" ]; then
    echo "ERROR: Windows usa $expected_branch y la VM usa $vm_branch." >&2
    exit 12
fi

git -C "$remote_repo" fetch --prune origin "$expected_branch"
fetched_head=$(git -C "$remote_repo" rev-parse "origin/$expected_branch")
if [ "$fetched_head" != "$expected_head" ]; then
    echo "ERROR: GitHub cambió durante la sincronización." >&2
    exit 13
fi
git -C "$remote_repo" merge --ff-only "origin/$expected_branch"

actual_head=$(git -C "$remote_repo" rev-parse HEAD)
if [ "$actual_head" != "$expected_head" ]; then
    echo "ERROR: la VM quedó en $actual_head, no en $expected_head." >&2
    exit 14
fi
if [ "$(readlink -f "$active_demo")" != "$remote_repo/g1" ]; then
    echo "ERROR: la ruta activa no apunta al clon Git." >&2
    exit 15
fi

echo "VM sincronizada: $expected_branch @ $actual_head"
REMOTE
