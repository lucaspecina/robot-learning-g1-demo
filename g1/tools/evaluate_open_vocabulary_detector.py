#!/usr/bin/env python3
"""Prueba Grounding DINO sobre una imagen sin integrarlo al robot."""
import argparse
import json
import time
from pathlib import Path

import torch
from PIL import Image
from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

DEFAULT_MODEL = "IDEA-Research/grounding-dino-tiny"
DEFAULT_REVISION = "a2bb814dd30d776dcf7e30523b00659f4f141c71"


def main():
    parser = argparse.ArgumentParser(
        description="Busca objetos descriptos con texto en una imagen guardada."
    )
    parser.add_argument("image", type=Path)
    parser.add_argument(
        "--labels",
        nargs="+",
        default=["a red table", "a blue table"],
    )
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--revision", default=DEFAULT_REVISION)
    parser.add_argument("--threshold", type=float, default=0.4)
    parser.add_argument("--text-threshold", type=float, default=0.3)
    args = parser.parse_args()

    if not args.image.is_file():
        raise SystemExit(f"no existe la imagen: {args.image}")

    torch.set_num_threads(2)
    processor = AutoProcessor.from_pretrained(
        args.model,
        revision=args.revision,
    )
    model = AutoModelForZeroShotObjectDetection.from_pretrained(
        args.model,
        revision=args.revision,
    ).eval()
    image = Image.open(args.image).convert("RGB")
    inputs = processor(
        images=image,
        text=[args.labels],
        return_tensors="pt",
    )

    started_at = time.monotonic()
    with torch.inference_mode():
        outputs = model(**inputs)
    elapsed_ms = (time.monotonic() - started_at) * 1000.0
    result = processor.post_process_grounded_object_detection(
        outputs,
        inputs.input_ids,
        threshold=args.threshold,
        text_threshold=args.text_threshold,
        target_sizes=[(image.height, image.width)],
    )[0]

    detections = []
    text_labels = result.get("text_labels", result.get("labels", []))
    for score, label, box in zip(
        result["scores"],
        text_labels,
        result["boxes"],
    ):
        detections.append(
            {
                "label": str(label),
                "confidence": round(float(score), 3),
                "box": [round(float(value), 1) for value in box],
            }
        )

    print(
        json.dumps(
            {
                "model": args.model,
                "revision": args.revision,
                "labels": args.labels,
                "threshold": args.threshold,
                "text_threshold": args.text_threshold,
                "inference_ms": round(elapsed_ms, 1),
                "detections": detections,
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
