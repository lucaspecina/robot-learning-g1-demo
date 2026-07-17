#!/usr/bin/env bash
# Lanza el simulador unitree_mujoco (versión Python) con el venv del proyecto.
# Correr desde WSL:  bash scripts/sim.sh
# Config del sim: external/unitree_mujoco/simulate_python/config.py
#   (ROBOT="go2", DOMAIN_ID=1, INTERFACE="lo", USE_JOYSTICK=0)
set -euo pipefail
cd "$(dirname "$0")/../external/unitree_mujoco/simulate_python"
exec "$HOME/.venvs/go2/bin/python" unitree_mujoco.py
