#!/usr/bin/env python3
"""
export_model.py
---------------
Export weights/*.pt for Pi inference.

Formats
-------
    ncnn      — CPU backend on Raspberry Pi (ARM64); fallback if Hailo isn't ready
    hailo     — Hailo-8L AI HAT+. HEF compile needs Linux x86_64 + Hailo DFC 3.x.
                On the Pi this writes Hailo-compatible ONNX. On the PC (DFC
                installed) it compiles the .hef Ultralytics can load on the Pi.
    openvino  — Intel-tuned alternative

Usage
-----
    # On the x86 Linux PC (Hailo DFC 3.x installed):
    python scripts/export_model.py --format hailo --weights weights/best.pt
    python scripts/export_model.py --format hailo --weights weights/gatebest.pt

    # CPU fallback on the Pi:
    python scripts/export_model.py --format ncnn
"""

from __future__ import annotations

import argparse
import platform
import shutil
import sys
from pathlib import Path

import yaml


def _find_project_root() -> Path:
    """Repo root = directory that contains config/model.yaml (scripts/ or repo root)."""
    here = Path(__file__).resolve().parent
    for candidate in (here, *here.parents):
        if (candidate / "config" / "model.yaml").is_file():
            return candidate
    if here.name == "scripts":
        return here.parent
    return here


PROJECT_ROOT = _find_project_root()
DEFAULT_WEIGHTS = PROJECT_ROOT / "weights" / "best.pt"
HAILO_ARCH = "hailo8l"
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export YOLO .pt weights to Hailo HEF, NCNN, or OpenVINO."
    )
    parser.add_argument(
        "--format",
        choices=("ncnn", "openvino", "hailo"),
        default="ncnn",
        help="Export target (default: ncnn). Use hailo on an x86 PC with DFC.",
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
    cfg_path = PROJECT_ROOT / "config" / "model.yaml"
    with open(cfg_path, "r") as f:
        return int(yaml.safe_load(f)["img_size"])


def _write_hailo_metadata(
    dest_dir: Path,
    *,
    task: str,
    imgsz: int,
    names: dict,
    onnx_name: str | None,
    hef_name: str | None,
) -> None:
    meta = {
        "task": task,
        "backend": "hailo",
        "hw_arch": HAILO_ARCH,
        "imgsz": int(imgsz),
        "names": {str(k): str(v) for k, v in dict(names).items()},
        "onnx": onnx_name,
        "hef": hef_name,
        "note": (
            "Compile the .hef on Linux x86_64 with Hailo DFC 3.x "
            "(python scripts/export_model.py --format hailo, or "
            "scripts/compile_hailo_hef.sh). Copy this folder back to the Pi "
            "and set backend: hailo."
        ),
    }
    with open(dest_dir / "metadata.yaml", "w") as f:
        yaml.safe_dump(meta, f, sort_keys=False)


def _pc_compile_help(weights: Path, dest_dir: Path, imgsz: int) -> str:
    rel = weights
    try:
        rel = weights.relative_to(PROJECT_ROOT)
    except ValueError:
        pass
    return (
        "\nHailo DFC cannot run on this machine "
        f"({platform.machine()}). Compile on Linux x86_64:\n"
        "  1. Install Hailo Dataflow Compiler 3.x (Hailo-8 / Hailo-8L) from\n"
        "     https://hailo.ai/developer-zone/\n"
        "  2. pip install ultralytics\n"
        "     pip install /path/to/hailo_dataflow_compiler-*.whl\n"
        f"  3. python scripts/export_model.py --format hailo --weights {rel}\n"
        "     or: bash scripts/compile_hailo_hef.sh\n"
        f"  4. Copy {dest_dir.name}/ (must contain a .hef) back to the Pi.\n"
        "  5. Set backend: hailo in config/model.yaml\n"
        f"ONNX compiler input (if using hailomz) is in {dest_dir}.\n"
        f"imgsz={imgsz} hw_arch={HAILO_ARCH}\n"
    )


def _export_hailo(weights: Path, imgsz: int) -> Path:
    from ultralytics import YOLO

    try:
        from src.hailo_runtime import hailo_export_dir, hailo_hef_path
    except ImportError:
        from hailo_runtime import hailo_export_dir, hailo_hef_path

    model = YOLO(str(weights))
    task = str(getattr(model, "task", "detect") or "detect")
    names = getattr(model, "names", {}) or {}
    dest_dir = hailo_export_dir(weights)
    dest_dir.mkdir(exist_ok=True)
    print(f"[export] task={task} arch={HAILO_ARCH} imgsz={imgsz}")

    try:
        from hailo_sdk_client import ClientRunner  # noqa: F401
    except ImportError:
        onnx_path = Path(
            model.export(
                format="onnx",
                imgsz=imgsz,
                opset=11,
                simplify=True,
                dynamic=False,
            )
        )
        dest = dest_dir / f"{weights.stem}.onnx"
        if dest.exists():
            dest.unlink()
        shutil.move(str(onnx_path), dest)
        extra = onnx_path.with_suffix(".onnx.data")
        if extra.is_file():
            shutil.move(str(extra), dest_dir / extra.name)
        hef = hailo_hef_path(dest_dir)
        _write_hailo_metadata(
            dest_dir,
            task=task,
            imgsz=imgsz,
            names=names,
            onnx_name=dest.name,
            hef_name=hef.name if hef else None,
        )
        print(f"[export] ONNX: {dest}")
        if hef is None:
            print(_pc_compile_help(weights, dest_dir, imgsz))
        else:
            print(f"[export] HEF already present: {hef.name}")
        return dest_dir

    print("[export] Hailo DFC found — compiling HEF (this takes several minutes) …")
    exported = Path(model.export(format="hailo", name=HAILO_ARCH, imgsz=imgsz))
    if exported.is_dir() and exported.resolve() != dest_dir.resolve():
        dest_dir.mkdir(exist_ok=True)
        for item in exported.iterdir():
            target = dest_dir / item.name
            if target.exists():
                if target.is_dir():
                    shutil.rmtree(target)
                else:
                    target.unlink()
            shutil.move(str(item), target)
        try:
            exported.rmdir()
        except OSError:
            pass
        exported = dest_dir
    hef = hailo_hef_path(Path(exported) if exported.is_dir() else exported.parent)
    _write_hailo_metadata(
        dest_dir,
        task=task,
        imgsz=imgsz,
        names=names,
        onnx_name=None,
        hef_name=hef.name if hef else None,
    )
    print(f"[export] Done: {exported}")
    print("Copy this folder onto the Pi if you compiled elsewhere, then:")
    print("  backend: hailo   # config/model.yaml")
    print("  python inference.py --web --timing")
    return dest_dir if dest_dir.is_dir() else exported


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
    except AttributeError as exc:
        print(
            "[ERROR] Ultralytics import failed (NumPy/matplotlib mismatch in this venv).\n"
            "In the compile env run:\n"
            "  pip install 'numpy>=1.23,<2' 'matplotlib>=3.8'",
            file=sys.stderr,
        )
        raise SystemExit(1) from exc

    print(f"[export] Loading {weights}")
    if args.format == "hailo":
        exported = _export_hailo(weights, imgsz)
        print(f"[export] Done: {exported}")
        return

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
