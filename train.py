"""
train.py
--------
Standalone training pipeline.  Run this on a machine with a GPU (or Google
Colab) — not on the Pi5 itself.

Steps performed
---------------
1. Read config/model.yaml  and config/dataset.yaml
2. Download the dataset (Roboflow) or resolve the local path
3. Build / load the YOLO model
4. Train and save the best weights to weights/best.pt
5. Print a short results summary

Usage
-----
    python train.py [--epochs N] [--batch N] [--model yolov8s]

Command-line arguments override the values in config/model.yaml so you can
run quick experiments without touching the config file.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).parent
WEIGHTS_DIR  = PROJECT_ROOT / "weights"
CONFIG_MODEL  = PROJECT_ROOT / "config" / "model.yaml"
CONFIG_DATASET = PROJECT_ROOT / "config" / "dataset.yaml"


def _load_yaml(path: Path) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train a YOLO model on the apple detection dataset."
    )
    parser.add_argument("--epochs", type=int,  default=None,
                        help="Override epochs from config/model.yaml")
    parser.add_argument("--batch",  type=int,  default=None,
                        help="Override batch_size from config/model.yaml")
    parser.add_argument("--model",  type=str,  default=None,
                        help="Override architecture (e.g. yolov8s)")
    parser.add_argument("--imgsz",  type=int,  default=None,
                        help="Override img_size")
    parser.add_argument("--resume", action="store_true",
                        help="Resume training from the weights path in config/model.yaml")
    parser.add_argument("--device", type=str, default=None,
                        help="Device override: 'cpu', '0', '0,1', 'mps' …")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # ------------------------------------------------------------------
    # Load configs
    # ------------------------------------------------------------------
    model_cfg   = _load_yaml(CONFIG_MODEL)
    dataset_cfg = _load_yaml(CONFIG_DATASET)   # used by dataset.py

    architecture = args.model  or model_cfg["architecture"]
    epochs       = args.epochs or int(model_cfg.get("epochs",     50))
    batch_size   = args.batch  or int(model_cfg.get("batch_size", 16))
    img_size     = args.imgsz  or int(model_cfg.get("img_size",  640))
    weights_path = str(model_cfg.get("weights", "")).strip()

    print("=" * 60)
    print("YOLO Apple Detection — Training")
    print("=" * 60)
    print(f"  Architecture : {architecture}")
    print(f"  Epochs       : {epochs}")
    print(f"  Batch size   : {batch_size}")
    print(f"  Image size   : {img_size}")
    print(f"  Resume from  : {weights_path or 'pretrained backbone'}")
    print("=" * 60)

    # ------------------------------------------------------------------
    # Resolve dataset
    # ------------------------------------------------------------------
    try:
        from src.dataset import get_dataset_yaml
        data_yaml = get_dataset_yaml(CONFIG_DATASET)
    except Exception as exc:
        print(f"\n[ERROR] Could not prepare dataset: {exc}")
        sys.exit(1)

    # ------------------------------------------------------------------
    # Load model
    # ------------------------------------------------------------------
    try:
        from ultralytics import YOLO  # type: ignore
    except ImportError:
        print("\n[ERROR] ultralytics not installed. Run: pip install ultralytics")
        sys.exit(1)

    if args.resume and weights_path and Path(weights_path).is_file():
        print(f"[Train] Resuming from {weights_path}")
        model = YOLO(weights_path)
    else:
        model_source = weights_path if weights_path and Path(weights_path).is_file() \
                       else f"{architecture}.pt"
        print(f"[Train] Loading model: {model_source}")
        model = YOLO(model_source)

    # ------------------------------------------------------------------
    # Train
    # ------------------------------------------------------------------
    train_kwargs: dict = dict(
        data=data_yaml,
        epochs=epochs,
        imgsz=img_size,
        batch=batch_size,
        project=str(PROJECT_ROOT / "runs" / "train"),
        name="apple",
        exist_ok=True,
        verbose=True,
    )
    if args.device:
        train_kwargs["device"] = args.device

    print("\n[Train] Starting training …\n")
    results = model.train(**train_kwargs)

    # ------------------------------------------------------------------
    # Copy best weights to weights/best.pt
    # ------------------------------------------------------------------
    WEIGHTS_DIR.mkdir(exist_ok=True)

    best_src = Path(results.save_dir) / "weights" / "best.pt"
    if best_src.is_file():
        dest = WEIGHTS_DIR / "best.pt"
        shutil.copy2(best_src, dest)
        print(f"\n[Train] Best weights saved to: {dest}")
        print(
            "\nNext step — update config/model.yaml:\n"
            f"  weights: weights/best.pt\n"
            "Then run inference.py on the Pi."
        )
    else:
        print(f"\n[WARN] best.pt not found at {best_src}. "
              "Check the runs/train/apple/weights/ directory manually.")

    # ------------------------------------------------------------------
    # Validation summary
    # ------------------------------------------------------------------
    print("\n[Train] Running validation on best weights …")
    val_model = YOLO(str(WEIGHTS_DIR / "best.pt"))
    metrics = val_model.val(data=data_yaml, imgsz=img_size, verbose=False)

    print("\n" + "=" * 60)
    print("Validation Results")
    print("=" * 60)
    try:
        print(f"  mAP50      : {metrics.box.map50:.4f}")
        print(f"  mAP50-95   : {metrics.box.map:.4f}")
        print(f"  Precision  : {metrics.box.mp:.4f}")
        print(f"  Recall     : {metrics.box.mr:.4f}")
    except AttributeError:
        print("  (Metrics object format may differ — check runs/train/apple/)")
    print("=" * 60)


if __name__ == "__main__":
    main()
