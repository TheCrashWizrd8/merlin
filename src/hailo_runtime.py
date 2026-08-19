"""
hailo_runtime.py
----------------
Hailo-8L helpers and HailoRT inference.

Compile (Linux x86_64 + Hailo DFC 3.x) produces a .hef in
weights/<stem>_hailo_model/.  The Pi runs that .hef with HailoRT
(hailo_platform). Ultralytics on this Pi cannot load .hef files.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import yaml

HAILO_ARCH = "hailo8l"
DFC_DOCS = "https://docs.ultralytics.com/integrations/hailo"
DEV_ZONE = "https://hailo.ai/developer-zone/"


def hailo_export_dir(weights_path: Path) -> Path:
    return weights_path.with_name(f"{weights_path.stem}_hailo_model")


def hailo_hef_path(export_dir: Path) -> Path | None:
    hefs = sorted(export_dir.glob("*.hef"))
    if hefs:
        return hefs[0]
    nested = sorted(export_dir.rglob("*.hef"))
    return nested[0] if nested else None


def hailo_unavailable_reason(export_dir: Path) -> str:
    return (
        f"No Hailo .hef in {export_dir}.\n"
        "Compile on a Linux x86_64 PC with Hailo Dataflow Compiler 3.x "
        f"(Hailo-8L). DFC wheel: {DEV_ZONE}  Ultralytics notes: {DFC_DOCS}\n"
        "  1. Copy this repo (or weights/*.pt) to the PC.\n"
        "  2. pip install ultralytics && pip install /path/to/hailo_dataflow_compiler-*.whl\n"
        "  3. python scripts/export_model.py --format hailo --weights weights/best.pt\n"
        "     python scripts/export_model.py --format hailo --weights weights/gatebest.pt\n"
        "     (or: bash scripts/compile_hailo_hef.sh)\n"
        "  4. Copy weights/*_hailo_model/ back onto the Pi.\n"
        "  5. Set backend: hailo in config/model.yaml"
    )


def package_status(export_dir: Path) -> tuple[bool, Optional[str]]:
    """True when Detector can load this catalog entry on HailoRT."""
    if hailo_hef_path(export_dir) is not None:
        return True, None
    return False, hailo_unavailable_reason(export_dir)


def class_names_from_export(export_dir: Path) -> dict[int, str]:
    meta = export_dir / "metadata.yaml"
    if not meta.is_file():
        return {}
    try:
        data = yaml.safe_load(meta.read_text()) or {}
    except Exception:
        return {}
    raw = data.get("names") or {}
    if isinstance(raw, dict):
        return {int(k): str(v) for k, v in raw.items()}
    if isinstance(raw, (list, tuple)):
        return {i: str(v) for i, v in enumerate(raw)}
    return {}


def letterbox_rgb(
    bgr: np.ndarray,
    imgsz: int,
    pad_value: int = 114,
    dst: np.ndarray | None = None,
) -> tuple[np.ndarray, float, tuple[float, float]]:
    """Resize with stride-free letterbox to imgsz×imgsz RGB uint8."""
    import cv2

    h, w = bgr.shape[:2]
    r = min(imgsz / h, imgsz / w)
    new_w, new_h = int(round(w * r)), int(round(h * r))
    if (new_w, new_h) != (w, h):
        resized = cv2.resize(bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    else:
        resized = bgr
    dw = (imgsz - new_w) / 2.0
    dh = (imgsz - new_h) / 2.0
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    padded = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(pad_value,) * 3
    )
    if padded.shape[0] != imgsz or padded.shape[1] != imgsz:
        padded = cv2.resize(padded, (imgsz, imgsz), interpolation=cv2.INTER_LINEAR)
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
    if dst is not None:
        dst[...] = rgb
        rgb = dst
    return rgb, r, (dw, dh)


def scale_hailo_box(
    ymin: float,
    xmin: float,
    ymax: float,
    xmax: float,
    *,
    imgsz: int,
    ratio: float,
    pad: tuple[float, float],
    frame_w: int,
    frame_h: int,
) -> tuple[int, int, int, int]:
    """Hailo NMS boxes are ymin,xmin,ymax,xmax in 0–1 of the letterboxed square."""
    dw, dh = pad
    if max(ymin, xmin, ymax, xmax) <= 1.5:
        ymin, xmin, ymax, xmax = (
            ymin * imgsz, xmin * imgsz, ymax * imgsz, xmax * imgsz
        )
    x1 = (xmin - dw) / ratio
    y1 = (ymin - dh) / ratio
    x2 = (xmax - dw) / ratio
    y2 = (ymax - dh) / ratio
    x1 = int(max(0, min(frame_w - 1, round(x1))))
    y1 = int(max(0, min(frame_h - 1, round(y1))))
    x2 = int(max(0, min(frame_w - 1, round(x2))))
    y2 = int(max(0, min(frame_h - 1, round(y2))))
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    return x1, y1, x2, y2


def detections_from_nms(
    nms_out,
    *,
    names: dict[int, str],
    confidence: float,
    imgsz: int,
    ratio: float,
    pad: tuple[float, float],
    frame_w: int,
    frame_h: int,
    max_det: int = 5,
) -> list:
    from src.detector import Detection

    per_class = _coerce_nms_by_class(nms_out)
    detections: list[Detection] = []
    for class_id, boxes in enumerate(per_class):
        for row in _box_rows(boxes):
            if len(row) < 5:
                continue
            ymin, xmin, ymax, xmax, score = (float(v) for v in row[:5])
            if score < confidence:
                continue
            x1, y1, x2, y2 = scale_hailo_box(
                ymin, xmin, ymax, xmax,
                imgsz=imgsz, ratio=ratio, pad=pad,
                frame_w=frame_w, frame_h=frame_h,
            )
            detections.append(
                Detection(
                    x1=x1, y1=y1, x2=x2, y2=y2,
                    confidence=score,
                    class_id=class_id,
                    label=names.get(class_id, str(class_id)),
                )
            )
    detections.sort(key=lambda d: d.confidence, reverse=True)
    return detections[:max_det]


def _box_rows(boxes) -> list[np.ndarray]:
    """Yield [ymin, xmin, ymax, xmax, score] rows for one class."""
    if boxes is None:
        return []
    if isinstance(boxes, np.ndarray):
        if boxes.size == 0:
            return []
        if boxes.dtype == object:
            rows: list[np.ndarray] = []
            for item in boxes.tolist():
                rows.extend(_box_rows(item))
            return rows
        if boxes.ndim == 1:
            return [boxes[:5]] if boxes.size >= 5 else []
        if boxes.ndim >= 2:
            flat = boxes.reshape(-1, boxes.shape[-1])
            return [row[:5] for row in flat if row.size >= 5]
        return []
    if isinstance(boxes, (list, tuple)):
        if not boxes:
            return []
        first = boxes[0]
        if isinstance(first, (list, tuple, np.ndarray)):
            inner = np.asarray(first)
            if inner.ndim == 0 or (inner.ndim == 1 and inner.size < 5):
                rows = []
                for item in boxes:
                    rows.extend(_box_rows(item))
                return rows
            if inner.ndim == 1 and inner.size >= 5:
                return [np.asarray(row, dtype=np.float32)[:5] for row in boxes]
        try:
            arr = np.asarray(boxes, dtype=np.float32)
        except ValueError:
            rows = []
            for item in boxes:
                rows.extend(_box_rows(item))
            return rows
        return _box_rows(arr)
    return []


def _coerce_nms_by_class(nms_out) -> list:
    """Normalize Hailo NMS-by-class output to a list, one tensor per class."""
    if nms_out is None:
        return []
    if isinstance(nms_out, dict):
        nms_out = next(iter(nms_out.values()))
    data = nms_out
    if isinstance(data, np.ndarray) and data.dtype == object:
        data = data.tolist()
    # Drop a leading batch dimension of 1 when the payload is per-class lists.
    while (
        isinstance(data, (list, tuple))
        and len(data) == 1
        and _is_per_class_list(data[0])
    ):
        data = data[0]
        if isinstance(data, np.ndarray) and data.dtype == object:
            data = data.tolist()
    if isinstance(data, np.ndarray) and data.dtype != object:
        return [data]
    if isinstance(data, (list, tuple)):
        return list(data)
    return [data]


def _is_per_class_list(item) -> bool:
    if isinstance(item, np.ndarray) and item.dtype == object:
        item = item.tolist()
    if not isinstance(item, (list, tuple)) or not item:
        return False
    first = item[0]
    if isinstance(first, np.ndarray):
        return first.ndim == 2 or first.size == 0 or (
            first.ndim == 1 and first.size != 5
        )
    if isinstance(first, (list, tuple)):
        arr = np.asarray(first, dtype=object)
        return arr.ndim >= 1
    return False


class HailoYOLO:
    """HailoRT runner for a YOLOv8 HEF with on-chip NMS."""

    def __init__(
        self,
        hef_path: Path,
        *,
        names: Optional[dict[int, str]] = None,
        confidence: float = 0.5,
        img_size: int = 640,
    ) -> None:
        from hailo_platform import (
            HEF,
            ConfigureParams,
            FormatType,
            HailoStreamInterface,
            InferVStreams,
            InputVStreamParams,
            OutputVStreamParams,
            VDevice,
        )

        self.hef_path = Path(hef_path)
        self.names = names or class_names_from_export(self.hef_path.parent)
        self.confidence = float(confidence)
        self.img_size = int(img_size)
        self._target = None
        self._infer = None
        self._activated = None

        hef = HEF(str(self.hef_path))
        input_info = hef.get_input_vstream_infos()[0]
        output_info = hef.get_output_vstream_infos()[0]
        self._input_name = input_info.name
        self._output_name = output_info.name
        shape = tuple(int(v) for v in input_info.shape)
        # NHWC (H, W, C) or (N, H, W, C)
        if len(shape) == 4:
            self.img_size = int(shape[1])
        elif len(shape) >= 2:
            self.img_size = int(shape[0])

        self._input = np.empty(
            (1, self.img_size, self.img_size, 3), dtype=np.uint8
        )

        self._target = VDevice()
        configure_params = ConfigureParams.create_from_hef(
            hef, interface=HailoStreamInterface.PCIe
        )
        network_group = self._target.configure(hef, configure_params)[0]
        input_vstreams_params = InputVStreamParams.make_from_network_group(
            network_group, format_type=FormatType.UINT8
        )
        output_vstreams_params = OutputVStreamParams.make_from_network_group(
            network_group, format_type=FormatType.FLOAT32
        )
        self._network_group = network_group
        self._ng_params = network_group.create_params()
        self._infer = InferVStreams(
            network_group, input_vstreams_params, output_vstreams_params
        )
        self._infer.__enter__()
        self._activated = network_group.activate(self._ng_params)
        self._activated.__enter__()

    def close(self) -> None:
        if self._activated is not None:
            try:
                self._activated.__exit__(None, None, None)
            except Exception:
                pass
            self._activated = None
        if self._infer is not None:
            try:
                self._infer.__exit__(None, None, None)
            except Exception:
                pass
            self._infer = None
        if self._target is not None:
            try:
                self._target.release()
            except Exception:
                pass
            self._target = None

    def predict(self, frame: np.ndarray, max_det: int = 5) -> list:
        if self._infer is None:
            raise RuntimeError("HailoYOLO is closed")
        h, w = frame.shape[:2]
        _, ratio, pad = letterbox_rgb(frame, self.img_size, dst=self._input[0])
        raw = self._infer.infer({self._input_name: self._input})
        nms = raw.get(self._output_name, raw)
        try:
            return detections_from_nms(
                nms,
                names=self.names,
                confidence=self.confidence,
                imgsz=self.img_size,
                ratio=ratio,
                pad=pad,
                frame_w=w,
                frame_h=h,
                max_det=max_det,
            )
        except (TypeError, ValueError) as exc:
            print(f"[HailoYOLO] NMS decode failed ({type(exc).__name__}: {exc})")
            return []


def load_hailo_yolo(
    hef_path: Path,
    *,
    names: Optional[dict[int, str]] = None,
    confidence: float = 0.5,
    img_size: int = 640,
) -> HailoYOLO:
    return HailoYOLO(
        hef_path, names=names, confidence=confidence, img_size=img_size
    )
