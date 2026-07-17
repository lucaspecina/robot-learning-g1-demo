#!/usr/bin/env bash
# Setup reproducible del entorno (correr desde WSL, en la raíz del repo):
#   bash scripts/setup.sh
# Idempotente: se puede re-correr sin romper nada.
set -euo pipefail
cd "$(dirname "$0")/.."

# 1. Clones externos (gitignored — este script los regenera)
mkdir -p external
[ -d external/unitree_mujoco ] || git clone --depth 1 https://github.com/unitreerobotics/unitree_mujoco.git external/unitree_mujoco
[ -d external/unitree_sdk2_python ] || git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git external/unitree_sdk2_python

# 2. Sin gamepad conectado, el sim Python muere si USE_JOYSTICK=1
sed -i 's/^USE_JOYSTICK = 1/USE_JOYSTICK = 0/' external/unitree_mujoco/simulate_python/config.py

# 3. uv (gestor de entornos, sin sudo)
command -v uv >/dev/null || { curl -LsSf https://astral.sh/uv/install.sh | sh; export PATH="$HOME/.local/bin:$PATH"; }

# 4. Venv en filesystem de WSL (rápido) con Python 3.10
#    (cyclonedds==0.10.2, pineado por el SDK, solo tiene wheels hasta cp310)
uv venv ~/.venvs/go2 --python 3.10
uv pip install --python ~/.venvs/go2/bin/python \
    -e external/unitree_sdk2_python mujoco pygame matplotlib ipykernel jupyterlab

echo
echo "Listo. Activar con:  source ~/.venvs/go2/bin/activate"
echo "Simulador:           bash scripts/sim.sh"
