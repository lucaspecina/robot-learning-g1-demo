"""Where the pipeline keeps its artifacts.

Everything lives under `data/` at the repository root, which Git ignores:
datasets and checkpoints are large and some recordings are internal.
"""

import os
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = REPO_ROOT / "data"
DATASETS_DIR = DATA_DIR / "datasets"
CHECKPOINTS_DIR = DATA_DIR / "checkpoints"

# Raw teleoperation recordings stay outside the repository (they are large and
# internal). Override with G1_RAW_RECORDINGS on machines where they live elsewhere.
RAW_RECORDINGS_DIR = Path(os.environ.get("G1_RAW_RECORDINGS", "~/Desktop/episodios")).expanduser()
