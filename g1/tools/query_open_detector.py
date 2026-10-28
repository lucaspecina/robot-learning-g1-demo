#!/usr/bin/env python3
"""Consulta el detector remoto con una imagen guardada, sin mover el robot."""
import argparse
import json
import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from agent.intelligence_client import IntelligenceClient  # noqa: E402
from open_vocabulary_core import SEARCH_TARGET_LABELS  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Busca una mesa en una imagen guardada."
    )
    parser.add_argument("image", type=Path)
    parser.add_argument("target", choices=sorted(SEARCH_TARGET_LABELS))
    args = parser.parse_args()

    result = IntelligenceClient().detect_objects(
        args.image.read_bytes(),
        SEARCH_TARGET_LABELS[args.target],
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
