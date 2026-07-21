#!/usr/bin/env bash
# Setup reproducible del entorno (correr desde WSL, en la raíz del repo):
#   bash scripts/setup.sh
# Idempotente: se puede re-correr sin romper nada.
set -euo pipefail
cd "$(dirname "$0")/.."

# 1. Clones externos (gitignored — este script los regenera, PINEADOS al
#    commit exacto con el que trabajamos, para reproducibilidad real)
clone_pinned() {  # repo_url destino sha
    if [ ! -d "$2" ]; then
        git clone "$1" "$2"
        git -C "$2" checkout --quiet "$3"
    fi
}
mkdir -p external
clone_pinned https://github.com/unitreerobotics/unitree_mujoco.git      external/unitree_mujoco      ae6a8403e272733e9996ef59990880330496177f
clone_pinned https://github.com/unitreerobotics/unitree_sdk2_python.git external/unitree_sdk2_python e4cd91f051aaa77a70600e3d2bf7f50889db1980
# rl_sar: trae las policies pre-entrenadas de Go2 que usa 03_walk_policy.py
clone_pinned https://github.com/fan-ziqi/rl_sar.git                     external/rl_sar              1fae490143f02f0098cfa90d15b9dd3e679cbd34
# unitree_rl_gym: referencia oficial (deploy de humanoides, pipeline de training)
clone_pinned https://github.com/unitreerobotics/unitree_rl_gym.git      external/unitree_rl_gym      276801e46c5d433564f24658bac64f254b7d2d4b

# 2. Ajustes al sim (idempotentes):
#    - sin gamepad conectado, el sim Python muere si USE_JOYSTICK=1
#    - SIMULATE_DT 0.002: con 0.005 los contactos se integran mal y las
#      policies RL tiemblan (roll std 0.93 -> 0.26 rad/s al bajarlo)
#    - sacamos cordones y escalera de la escena go2 (piso plano para empezar)
sed -i 's/^USE_JOYSTICK = 1/USE_JOYSTICK = 0/' external/unitree_mujoco/simulate_python/config.py
sed -i 's/^SIMULATE_DT = 0.005/SIMULATE_DT = 0.002/' external/unitree_mujoco/simulate_python/config.py
sed -i '/type="box"/d' external/unitree_mujoco/unitree_robots/go2/scene.xml

# 3. uv (gestor de entornos, sin sudo)
command -v uv >/dev/null || { curl -LsSf https://astral.sh/uv/install.sh | sh; export PATH="$HOME/.local/bin:$PATH"; }

# 4. Venv en filesystem de WSL (rápido) con Python 3.10
#    (cyclonedds==0.10.2, pineado por el SDK, solo tiene wheels hasta cp310)
uv venv ~/.venvs/go2 --python 3.10
# torch: el build de Linux en PyPI ya viene con CUDA (para correr policies y,
# a futuro, entrenar). pyyaml lo usan las configs de rl_sar.
uv pip install --python ~/.venvs/go2/bin/python \
    -e external/unitree_sdk2_python mujoco pygame matplotlib ipykernel jupyterlab \
    torch pyyaml

echo
echo "Listo. Activar con:  source ~/.venvs/go2/bin/activate"
echo "Simulador:           bash scripts/sim.sh"
