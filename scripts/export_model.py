#!/usr/bin/env python3
"""
export_model.py
---------------
Export weights/best.pt to NCNN or OpenVINO for faster inference on the Pi.

NCNN is recommended on Raspberry Pi (ARM64). OpenVINO is mainly tuned for Intel
CPUs but is supported here if you want to compare.

Usage
-----
    # Recommended on Pi 5
    python scripts/export_model.py --format ncnn

    # Alternative backend
    python scripts/export_model.py --format openvino

    # Custom weights / input size (must match config/model.yaml img_size)
    python scripts/export_model.py --weights weights/best.pt --imgsz 320

After export, set backend in config/model.yaml and run inference as usual:
    python inference.py --web --timing

Export dependencies (once):
    pip install -r requirements-export.txt

If export fails on the Pi (Python 3.13 / wheel issues), export on Colab or an
x86 machine and copy the generated *_ncnn_model/ or *_openvino_model/ folder
into weights/ on the Pi.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_WEIGHTS = PROJECT_ROOT / "weights" / "best.pt"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export YOLO .pt weights to NCNN or OpenVINO for Pi inference."
    )
    parser.add_argument(
        "--format",
        choices=("ncnn", "openvino"),
        default="ncnn",
        help="Export target (default: ncnn — best on Raspberry Pi)",
    )
    parser.add_argument(
        "--weights",
        type=Path,
        default=DEFAULT_WEIGHTS,
        help=f"PyTorch weights to export (default: {DEFAULT_WEIGHTS})",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=None,
        help="Square input size in pixels (default: read from config/model.yaml)",
    )
    return parser.parse_args()


def _load_img_size() -> int:
    import yaml

    cfg_path = PROJECT_ROOT / "config" / "model.yaml"
    with open(cfg_path, "r") as f:
        return int(yaml.safe_load(f)["img_size"])


def main() -> None:
    args = parse_args()
    weights = args.weights if args.weights.is_absolute() else PROJECT_ROOT / args.weights
    if not weights.is_file():
        print(
            f"[ERROR] Weights not found: {weights}\n"
            "Train first (train.py / Colab) or pass --weights to an existing .pt file.",
            file=sys.stderr,
        )
        sys.exit(1)

    imgsz = args.imgsz if args.imgsz is not None else _load_img_size()

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        print(
            "[ERROR] ultralytics is not installed. Run: pip install ultralytics",
            file=sys.stderr,
        )
        raise SystemExit(1) from exc

    print(f"[export] Loading {weights}")
    model = YOLO(str(weights))
    task = getattr(model, "task", None)
    print(f"[export] task={task}")
    if task and str(task).lower() == "segment":
        print(
            "[export] Segmentation checkpoint — regenerate the NCNN/OpenVINO folder "
            "from this .pt (do not keep an old detect export)."
        )
    print(f"[export] Exporting to {args.format} (imgsz={imgsz}) …")
    export_kwargs = {"format": args.format, "imgsz": imgsz}
    if args.format == "ncnn":
        export_kwargs["half"] = True
    exported = model.export(**export_kwargs)
    print(f"[export] Done: {exported}")
    print(
        f"\nNext: set backend: {args.format} in config/model.yaml, then run:\n"
        "  python inference.py --web --timing"
    )


if __name__ == "__main__":
    main()
