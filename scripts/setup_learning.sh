#!/usr/bin/env bash
# Sets up the learning environment (Mac, VM or workstation):
#   bash scripts/setup_learning.sh
# Idempotent: safe to run again.
set -euo pipefail
cd "$(dirname "$0")/.."

# unitree_lerobot is not on PyPI and its packaging skips its subpackages, so it
# is cloned at a pinned commit and installed editable. The commit is the tip of
# main on 2026-10-02; the one installed on the robot has not been checked yet.
UNITREE_LEROBOT_URL=https://github.com/unitreerobotics/unitree_lerobot.git
UNITREE_LEROBOT_SHA=41c2805742de879ddab2d8d6beaeaf215f876395
UNITREE_LEROBOT_DIR=external/unitree_lerobot

if [ ! -d "$UNITREE_LEROBOT_DIR/.git" ]; then
    # Robot meshes live in Git LFS and are not needed to convert data; neither
    # is the LeRobot submodule, which comes from PyPI instead.
    GIT_LFS_SKIP_SMUDGE=1 git clone --quiet "$UNITREE_LEROBOT_URL" "$UNITREE_LEROBOT_DIR"
fi
GIT_LFS_SKIP_SMUDGE=1 git -C "$UNITREE_LEROBOT_DIR" checkout --quiet "$UNITREE_LEROBOT_SHA"

uv sync

# On macOS some .pth files in the venv end up with the "hidden" file flag, and
# Python skips hidden .pth files since gh-113659. The
# editable install of this repo would silently stop being importable.
if [ "$(uname)" = "Darwin" ]; then
    chflags nohidden .venv/lib/python*/site-packages/*.pth
fi

echo "Entorno listo. Activar con: source .venv/bin/activate"
