#!/usr/bin/env bash
# Lanza el simulador unitree_mujoco (versión Python) con el venv del proyecto.
# Correr desde WSL:  bash scripts/sim.sh
# Config del sim: external/unitree_mujoco/simulate_python/config.py
#   (ROBOT="go2", DOMAIN_ID=1, INTERFACE="lo", USE_JOYSTICK=0)
set -euo pipefail
# Render por GPU vía WSLg: sin esto Mesa cae a llvmpipe (software, ~3 fps y
# la CPU al 500%). d3d12 = puente de WSLg a la GPU de Windows; el segundo var
# elige la NVIDIA (la laptop también tiene GPU integrada).
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
cd "$(dirname "$0")/../external/unitree_mujoco/simulate_python"
exec "$HOME/.venvs/go2/bin/python" unitree_mujoco.py
