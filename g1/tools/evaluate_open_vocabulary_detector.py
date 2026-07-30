#!/usr/bin/env python3
"""Prueba Grounding DINO sobre una imagen sin integrarlo al robot."""
import argparse
import json
import time
from pathlib import Path

import torch
from PIL import Image, ImageDraw
from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

DEFAULT_MODEL = "IDEA-Research/grounding-dino-tiny"
DEFAULT_REVISION = "a2bb814dd30d776dcf7e30523b00659f4f141c71"


def main():
    parser = argparse.ArgumentParser(
        description="Busca objetos descriptos con texto en una imagen guardada."
    )
    parser.add_argument("images", type=Path, nargs="+")
    parser.add_argument(
        "--labels",
        nargs="+",
        default=["a red table", "a blue table"],
    )
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--revision", default=DEFAULT_REVISION)
    parser.add_argument("--threshold", type=float, default=0.4)
    parser.add_argument("--text-threshold", type=float, default=0.3)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()

    missing = [str(path) for path in args.images if not path.is_file()]
    if missing:
        raise SystemExit(f"no existen estas imágenes: {', '.join(missing)}")

    torch.set_num_threads(2)
    processor = AutoProcessor.from_pretrained(
        args.model,
        revision=args.revision,
    )
    model = AutoModelForZeroShotObjectDetection.from_pretrained(
        args.model,
        revision=args.revision,
    ).eval()
    reports = []
    for image_path in args.images:
        image = Image.open(image_path).convert("RGB")
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
        annotated = image.copy()
        draw = ImageDraw.Draw(annotated)
        text_labels = result.get("text_labels", result.get("labels", []))
        for score, label, box in zip(
            result["scores"],
            text_labels,
            result["boxes"],
        ):
            coordinates = [round(float(value), 1) for value in box]
            confidence = round(float(score), 3)
            detections.append(
                {
                    "label": str(label),
                    "confidence": confidence,
                    "box": coordinates,
                }
            )
            draw.rectangle(coordinates, outline="#20d878", width=2)
            draw.text(
                (coordinates[0] + 2, coordinates[1] + 2),
                f"{str(label)} {confidence:.2f}",
                fill="#20d878",
            )
        reports.append(
            {
                "image": str(image_path),
                "inference_ms": round(elapsed_ms, 1),
                "detections": detections,
            }
        )
        if args.output_dir is not None:
            args.output_dir.mkdir(parents=True, exist_ok=True)
            annotated.save(args.output_dir / image_path.name)

    print(
        json.dumps(
            {
                "model": args.model,
                "revision": args.revision,
                "labels": args.labels,
                "threshold": args.threshold,
                "text_threshold": args.text_threshold,
                "images": reports,
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
