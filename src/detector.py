"""
detector.py
-----------
Thin wrapper around Ultralytics YOLO that loads its configuration
from config/model.yaml.  Keeping YOLO behind this interface means
swapping model architecture or version never touches inference.py.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import List, NamedTuple

import numpy as np
import yaml


PROJECT_ROOT = Path(__file__).parent.parent
CONFIG_PATH = PROJECT_ROOT / "config" / "model.yaml"


class Detection(NamedTuple):
    """Single detected object returned per frame."""
    x1: int
    y1: int
    x2: int
    y2: int
    confidence: float
    class_id: int
    label: str


def _load_config(path: Path = CONFIG_PATH) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


class Detector:
    """
    Wraps Ultralytics YOLO for apple detection.

    Usage
    -----
    detector = Detector()                  # loads config/model.yaml
    detections = detector.detect(frame)    # frame is an OpenCV BGR ndarray
    """

    def __init__(self, config_path: Path = CONFIG_PATH) -> None:
        cfg = _load_config(config_path)

        self.architecture: str = cfg["architecture"]
        self.confidence: float = float(cfg["confidence"])
        self.iou: float = float(cfg["iou"])
        self.img_size: int = int(cfg["img_size"])

        weights_value: str = cfg.get("weights", "")
        if weights_value:
            weights_path = Path(weights_value)
            if not weights_path.is_absolute():
                weights_path = PROJECT_ROOT / weights_path
            weights_value = str(weights_path)
            if not weights_path.is_file():
                raise FileNotFoundError(
                    f"Weights file not found: {weights_value}\n"
                    "Run train.py first, or correct the 'weights' path in config/model.yaml."
                )
            model_source = weights_value
        else:
            # Use Ultralytics pretrained backbone (downloads on first run)
            model_source = f"{self.architecture}.pt"

        # Import deferred so the module can be imported without ultralytics
        # installed (useful for unit-testing stubs).
        try:
            from ultralytics import YOLO  # type: ignore
            self._model = YOLO(model_source)
        except ImportError as exc:
            raise ImportError(
                "ultralytics is not installed. Run: pip install ultralytics"
            ) from exc

        print(
            f"[Detector] Loaded {self.architecture} "
            f"({'custom weights: ' + model_source if weights_value else 'pretrained backbone'})"
            f"  imgsz={self.img_size}"
        )
        # Warm up torch/YOLO so the first live frame is not an outlier.
        warmup = np.zeros((480, 640, 3), dtype=np.uint8)
        self._model(warmup, conf=self.confidence, iou=self.iou, imgsz=self.img_size, verbose=False)
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
        results = self._model(
            frame,
            conf=self.confidence,
            iou=self.iou,
            imgsz=self.img_size,
            verbose=False,
            max_det=10,
        )

        detections: List[Detection] = []
        for result in results:
            names = result.names  # {class_id: label}
            if result.boxes is None:
                continue
            for box in result.boxes:
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                detections.append(
                    Detection(
                        x1=x1, y1=y1, x2=x2, y2=y2,
                        confidence=conf,
                        class_id=cls,
                        label=names.get(cls, str(cls)),
                    )
                )

        detections.sort(key=lambda d: d.confidence, reverse=True)
        return detections
