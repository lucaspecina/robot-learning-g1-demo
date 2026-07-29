#!/usr/bin/env python3
"""Evalúa el RT-DETR elegido sobre imágenes guardadas.

Esta herramienta no forma parte del robot. Sirve para comparar modelos con
exactamente los mismos cuadros antes de elegir qué proceso desplegar en la
Jetson.
"""
import argparse
import json
import time
from pathlib import Path

import torch
from PIL import Image, ImageDraw
from transformers import AutoImageProcessor, RTDetrForObjectDetection

DEFAULT_MODEL = "PekingU/rtdetr_r50vd"
DEFAULT_REVISION = "df939e661d8c52e80608d1ec566561aabd25a4e7"


def main():
    parser = argparse.ArgumentParser(
        description="Mide detecciones, confianza y tiempo sobre imágenes."
    )
    parser.add_argument("images", type=Path, nargs="+")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--revision", default=DEFAULT_REVISION)
    parser.add_argument("--confidence", type=float, default=0.70)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--device",
        default="cuda" if torch.cuda.is_available() else "cpu",
    )
    args = parser.parse_args()

    missing = [str(path) for path in args.images if not path.is_file()]
    if missing:
        raise SystemExit(f"no existen estas imágenes: {', '.join(missing)}")

    processor = AutoImageProcessor.from_pretrained(
        args.model,
        revision=args.revision,
        use_fast=True,
    )
    model = RTDetrForObjectDetection.from_pretrained(
        args.model,
        revision=args.revision,
    ).eval().to(args.device)
    reports = []

    for image_path in args.images:
        image = Image.open(image_path).convert("RGB")
        inputs = {
            name: tensor.to(args.device)
            for name, tensor in processor(
                images=image,
                return_tensors="pt",
            ).items()
        }
        if args.device == "cuda":
            torch.cuda.synchronize()
        started_at = time.monotonic()
        with torch.inference_mode():
            outputs = model(**inputs)
        if args.device == "cuda":
            torch.cuda.synchronize()
        elapsed_ms = (time.monotonic() - started_at) * 1000.0
        target_sizes = torch.tensor(
            [[image.height, image.width]],
            device=args.device,
        )
        result = processor.post_process_object_detection(
            outputs,
            target_sizes=target_sizes,
            threshold=args.confidence,
        )[0]
        detections = []
        annotated = image.copy()
        draw = ImageDraw.Draw(annotated)
        for score, label, box in zip(
            result["scores"],
            result["labels"],
            result["boxes"],
        ):
            coordinates = [round(float(value), 1) for value in box]
            class_name = model.config.id2label[int(label)]
            confidence = round(float(score), 3)
            detections.append(
                {
                    "class": class_name,
                    "confidence": confidence,
                    "box": coordinates,
                }
            )
            draw.rectangle(coordinates, outline="#20d878", width=2)
            draw.text(
                (coordinates[0] + 2, coordinates[1] + 2),
                f"{class_name} {confidence:.2f}",
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
            output_path = args.output_dir / image_path.name
            annotated.save(output_path)

    print(
        json.dumps(
            {
                "model": args.model,
                "revision": args.revision,
                "device": args.device,
                "confidence_threshold": args.confidence,
                "images": reports,
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
