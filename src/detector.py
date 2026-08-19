"""
detector.py
-----------
Thin wrapper around Ultralytics YOLO that loads its configuration
from config/model.yaml.  Keeping YOLO behind this interface means
swapping model architecture, backend, or version never touches inference.py.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import List, NamedTuple, Optional

import numpy as np
import yaml


PROJECT_ROOT = Path(__file__).parent.parent
CONFIG_PATH = PROJECT_ROOT / "config" / "model.yaml"

BACKENDS = ("pytorch", "ncnn", "openvino", "hailo")


def _resolve_thread_count(configured: int) -> int:
    cpu = os.cpu_count() or 4
    if configured and configured > 0:
        return max(1, min(configured, cpu))
    return cpu


def _apply_thread_env(threads: int) -> None:
    """Keep BLAS/OpenMP from spawning extra pools beside NCNN."""
    os.environ.setdefault("OMP_NUM_THREADS", str(threads))
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")
    os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")


def _configure_ncnn_threads(model, threads: int) -> None:
    """Best-effort: pin the NCNN net to `threads` workers."""
    inner = getattr(model, "model", None)
    candidates = [
        inner,
        getattr(inner, "model", None),
        getattr(inner, "net", None),
        getattr(getattr(inner, "model", None), "net", None),
    ]
    for obj in candidates:
        if obj is None:
            continue
        opt = getattr(obj, "opt", None)
        if opt is not None and hasattr(opt, "num_threads"):
            opt.num_threads = threads
            return
        if hasattr(obj, "set_num_threads"):
            obj.set_num_threads(threads)
            return


class Detection(NamedTuple):
    """Single detected object returned per frame."""
    x1: int
    y1: int
    x2: int
    y2: int
    confidence: float
    class_id: int
    label: str
    # Mask centroid / area when the model is *-seg. -1 / 0 → use the bbox.
    cx: int = -1
    cy: int = -1
    mask_area: int = 0

    def center(self) -> tuple[int, int]:
        if self.cx >= 0 and self.cy >= 0:
            return self.cx, self.cy
        return (self.x1 + self.x2) // 2, (self.y1 + self.y2) // 2


def _load_config(path: Path = CONFIG_PATH) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _resolve_weights_path(weights_value: str) -> Path:
    weights_path = Path(weights_value)
    if not weights_path.is_absolute():
        weights_path = PROJECT_ROOT / weights_path
    return weights_path


def _exported_model_dir(weights_path: Path, backend: str) -> Path:
    """Map weights/best.pt -> weights/best_ncnn_model/ (Ultralytics convention)."""
    return weights_path.with_name(f"{weights_path.stem}_{backend}_model")


def _hailo_hef_path(export_dir: Path) -> Path | None:
    from src.hailo_runtime import hailo_hef_path

    return hailo_hef_path(export_dir)


def _pt_is_newer_than_export(weights_path: Path, export_dir: Path) -> bool:
    """True if the .pt was replaced after the last backend export."""
    if not weights_path.is_file() or not export_dir.is_dir():
        return False
    try:
        pt_mtime = weights_path.stat().st_mtime
        export_mtime = max(p.stat().st_mtime for p in export_dir.iterdir())
    except (OSError, ValueError):
        return False
    return pt_mtime > export_mtime + 1.0  # 1s slack for a copy still finishing


def _infer_task_from_checkpoint(weights_path: Path) -> Optional[str]:
    """Read Ultralytics task (detect/segment/…) from a .pt without constructing YOLO."""
    try:
        import torch

        ckpt = torch.load(weights_path, map_location="cpu", weights_only=False)
    except TypeError:
        try:
            import torch

            ckpt = torch.load(weights_path, map_location="cpu")
        except Exception:
            return None
    except Exception:
        return None
    if not isinstance(ckpt, dict):
        return None
    task = ckpt.get("task")
    if isinstance(task, str) and task:
        return task.lower()
    train_args = ckpt.get("train_args") or ckpt.get("args") or {}
    if isinstance(train_args, dict):
        t = train_args.get("task")
        if isinstance(t, str) and t:
            return t.lower()
    return None


def _read_export_task(export_dir: Path) -> Optional[str]:
    """Ultralytics writes metadata.yaml next to NCNN/OpenVINO exports."""
    meta = export_dir / "metadata.yaml"
    if not meta.is_file():
        return None
    try:
        with open(meta, "r") as f:
            data = yaml.safe_load(f) or {}
        task = data.get("task")
        if isinstance(task, str) and task:
            return task.lower()
    except Exception:
        return None
    return None


def mask_centroid_and_area(xy: np.ndarray) -> tuple[int, int, int]:
    """Centroid and pixel area of an (N, 2) mask polygon. Empty → (-1, -1, 0)."""
    if xy is None or len(xy) < 3:
        return -1, -1, 0
    pts = np.asarray(xy, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 2:
        return -1, -1, 0
    cx = int(round(float(pts[:, 0].mean())))
    cy = int(round(float(pts[:, 1].mean())))
    x, y = pts[:, 0], pts[:, 1]
    area = 0.5 * abs(float(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1))))
    return cx, cy, int(round(area))


def resolve_model_source(
    *,
    architecture: str,
    weights_value: str,
    backend: str,
) -> tuple[str, str]:
    """
    Resolve the path/string passed to YOLO() for the requested backend.

    Returns (model_source, human-readable description).
    """
    backend = backend.lower()
    if backend not in BACKENDS:
        raise ValueError(f"Unknown backend {backend!r}; choose from {BACKENDS}")

    if backend == "pytorch":
        if weights_value:
            weights_path = _resolve_weights_path(weights_value)
            if not weights_path.is_file():
                raise FileNotFoundError(
                    f"Weights file not found: {weights_path}\n"
                    "Run train.py first, or correct the 'weights' path in config/model.yaml."
                )
            return str(weights_path), f"pytorch weights: {weights_path}"
        return f"{architecture}.pt", "pytorch pretrained backbone"

    if not weights_value:
        raise ValueError(
            f"backend={backend!r} requires trained weights in config/model.yaml "
            "(a .pt file to export from)."
        )

    weights_path = _resolve_weights_path(weights_value)
    if not weights_path.is_file():
        raise FileNotFoundError(
            f"Source weights not found: {weights_path}\n"
            "Train first, then export with: python scripts/export_model.py "
            f"--format {backend}"
        )

    exported = _exported_model_dir(weights_path, backend)
    if not exported.is_dir():
        raise FileNotFoundError(
            f"Exported {backend} model not found: {exported}\n"
            f"Run: python scripts/export_model.py --format {backend}\n"
            "If export fails on the Pi, export on Colab/x86 and copy the folder to weights/."
        )
    if backend == "hailo":
        from src.hailo_runtime import hailo_unavailable_reason, hailo_hef_path

        hef = hailo_hef_path(exported)
        if hef is None:
            raise FileNotFoundError(hailo_unavailable_reason(exported))
        return str(hef), f"hailo HEF: {hef}"
    if _pt_is_newer_than_export(weights_path, exported):
        raise RuntimeError(
            f"weights {weights_path} is newer than the {backend} export at {exported}.\n"
            "The old export is still a detect net (or an older train) and will ignore the new .pt.\n"
            f"Re-export after swapping the .pt:\n"
            f"  python scripts/export_model.py --format {backend}"
        )
    return str(exported), f"{backend} export: {exported}"


class Detector:
    """
    Wraps Ultralytics YOLO for apple detection.

    Usage
    -----
    detector = Detector()                  # loads config/model.yaml
    detections = detector.detect(frame)    # frame is an OpenCV BGR ndarray
    """

    def __init__(
        self,
        config_path: Path = CONFIG_PATH,
        backend: Optional[str] = None,
        model_spec: Optional[dict] = None,
    ) -> None:
        self.config_path = config_path
        cfg = _load_config(config_path)
        self._base_cfg = cfg

        self.architecture: str = cfg["architecture"]
        self.backend: str = (backend or cfg.get("backend", "pytorch")).lower()
        self.confidence: float = float(cfg["confidence"])
        self.iou: float = float(cfg["iou"])
        self.img_size: int = int(cfg["img_size"])
        self.ncnn_threads: int = int(cfg.get("ncnn_threads", 0) or 0)

        self.model_id: str = ""
        self.track_label: str = "apple"
        self.task: str = "detect"
        self._model = None

        spec = model_spec or {}
        self.model_id = str(spec.get("id") or "")
        self._load_model(spec)

    def reload(self, model_spec: dict) -> None:
        """Hot-swap weights/task from a catalog entry."""
        self.model_id = str(model_spec.get("id") or self.model_id)
        self._load_model(model_spec)

    def _load_model(self, spec: dict) -> None:
        cfg = self._base_cfg
        weights_value: str = str(spec.get("weights") or cfg.get("weights") or "")
        self.architecture = str(spec.get("architecture") or cfg.get("architecture") or "yolov8n")
        self.track_label = str(spec.get("track_label") or "apple")

        yaml_task = str(spec.get("task") or cfg.get("task") or "auto").strip().lower()
        weights_path = (
            _resolve_weights_path(weights_value) if weights_value else None
        )
        pt_task = (
            _infer_task_from_checkpoint(weights_path)
            if weights_path is not None and weights_path.is_file()
            else None
        )
        if yaml_task in ("", "auto"):
            self.task = pt_task or "detect"
        else:
            self.task = yaml_task
            if pt_task and pt_task != yaml_task:
                print(
                    f"[Detector] config task={yaml_task!r} but checkpoint is "
                    f"{pt_task!r}; using config. Fix catalog if load fails."
                )

        model_source, description = resolve_model_source(
            architecture=self.architecture,
            weights_value=weights_value,
            backend=self.backend,
        )

        if self.backend != "pytorch":
            export_dir = Path(model_source)
            if export_dir.is_file():
                export_dir = export_dir.parent
            export_task = _read_export_task(export_dir)
            if export_task and export_task != self.task:
                raise RuntimeError(
                    f"Exported {self.backend} model is task={export_task!r} but "
                    f"config/checkpoint want task={self.task!r}.\n"
                    "Delete the old export folder and re-export from the new .pt:\n"
                    f"  python scripts/export_model.py --format {self.backend}"
                )

        if self.backend == "hailo":
            self._load_hailo(spec, model_source, description, weights_path)
            return

        threads = _resolve_thread_count(self.ncnn_threads)
        _apply_thread_env(threads)

        load_kw: dict = {}
        if self.task and self.task not in ("auto",):
            load_kw["task"] = self.task
        try:
            from ultralytics import YOLO  # type: ignore
            model = YOLO(model_source, **load_kw)
        except TypeError:
            from ultralytics import YOLO  # type: ignore
            model = YOLO(model_source)
        except ImportError as exc:
            raise ImportError(
                "ultralytics is not installed. Run: pip install ultralytics"
            ) from exc

        loaded_task = getattr(model, "task", None)
        if isinstance(loaded_task, str) and loaded_task:
            self.task = loaded_task

        _configure_ncnn_threads(model, threads)
        self._model = model

        tag = f" id={self.model_id}" if self.model_id else ""
        print(
            f"[Detector] Loaded {self.architecture} task={self.task}{tag} "
            f"backend={self.backend} ({description})  "
            f"imgsz={self.img_size}  threads={threads}"
        )
        warmup = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
        warmup_kw = dict(
            conf=self.confidence,
            iou=self.iou,
            imgsz=self.img_size,
            verbose=False,
            max_det=5,
        )
        if self.task == "segment":
            warmup_kw["retina_masks"] = False
        self._model(warmup, **warmup_kw)
        print("[Detector] Warmup complete")

    def _load_hailo(
        self,
        spec: dict,
        model_source: str,
        description: str,
        weights_path: Optional[Path],
    ) -> None:
        from src.hailo_runtime import HailoYOLO, class_names_from_export

        if isinstance(self._model, HailoYOLO):
            self._model.close()
            self._model = None

        export_dir = Path(model_source).parent
        names = class_names_from_export(export_dir)
        model = HailoYOLO(
            Path(model_source),
            names=names,
            confidence=self.confidence,
            img_size=self.img_size,
        )
        self._model = model
        self.img_size = model.img_size
        tag = f" id={self.model_id}" if self.model_id else ""
        print(
            f"[Detector] Loaded {self.architecture} task={self.task}{tag} "
            f"backend=hailo ({description})  imgsz={self.img_size}"
        )
        warmup = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
        self._model.predict(warmup)
        print("[Detector] Warmup complete")

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """
        Run inference on a single BGR frame.

        Parameters
        ----------
        frame : np.ndarray
            OpenCV BGR image.

        Returns
        -------
        List[Detection]
            All detections above the confidence threshold, sorted by
            descending confidence.
        """
        if self.backend == "hailo":
            return self._model.predict(frame, max_det=5)

        predict_kw = dict(
            conf=self.confidence,
            iou=self.iou,
            imgsz=self.img_size,
            verbose=False,
            max_det=5,
        )
        if self.task == "segment":
            predict_kw["retina_masks"] = False
        results = self._model(frame, **predict_kw)

        detections: List[Detection] = []
        for result in results:
            names = result.names  # {class_id: label}
            if result.boxes is None:
                continue
            mask_xy = None
            masks = getattr(result, "masks", None)
            if masks is not None:
                try:
                    mask_xy = masks.xy
                except Exception:
                    mask_xy = None
            for i, box in enumerate(result.boxes):
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                cx = cy = -1
                mask_area = 0
                if mask_xy is not None and i < len(mask_xy):
                    cx, cy, mask_area = mask_centroid_and_area(mask_xy[i])
                detections.append(
                    Detection(
                        x1=x1, y1=y1, x2=x2, y2=y2,
                        confidence=conf,
                        class_id=cls,
                        label=names.get(cls, str(cls)),
                        cx=cx,
                        cy=cy,
                        mask_area=mask_area,
                    )
                )

        detections.sort(key=lambda d: d.confidence, reverse=True)
        return detections
