"""
hailo_runtime.py
----------------
Hailo-8L helpers and HailoRT inference.

Compile (Linux x86_64 + Hailo DFC 3.x) produces a .hef next to the
.pt: weights/<id>/hailo/, weights/<id>/<stem>_hailo_model/, or
weights/<id>/<stem>hailomodel/.  The Pi runs that .hef with HailoRT
(hailo_platform). Ultralytics on this Pi cannot load .hef files.
"""

from __future__ import annotations

import os
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

os.environ.setdefault("HAILORT_LOGGER_PATH", "NONE")
os.environ.setdefault("HAILORT_CONSOLE_LOGGER_LEVEL", "error")

_HAILO_DEVICE_ERROR_MARKERS = (
    "HAILO_OUT_OF_PHYSICAL_DEVICES",
    "OUT_OF_PHYSICAL_DEVICES",
    "NOT ENOUGH FREE DEVICES",
    "HAILO_DEVICE_IN_USE",
)

HAILO_ARCH = "hailo8l"
DFC_DOCS = "https://docs.ultralytics.com/integrations/hailo"
DEV_ZONE = "https://hailo.ai/developer-zone/"


def hailo_candidate_dirs(weights_path: Path) -> list[Path]:
    """Folders that may hold a compiled .hef for this weights path.

    Ultralytics/DFC usually writes ``best_hailo_model``; dropping a
    folder named ``besthailomodel`` (no underscores) is also accepted.
    """
    if weights_path.suffix.lower() == ".hef":
        return [weights_path.parent]
    if weights_path.is_dir():
        return [weights_path]
    stem = weights_path.stem
    parent = weights_path.parent
    return [
        parent / "hailo",
        parent / f"{stem}hailomodel",
        parent / f"{stem}_hailo_model",
        parent / f"{stem}_hailo",
    ]


def is_hailo_device_error(exc: BaseException) -> bool:
    """True when HailoRT could not open a free PCIe device."""
    text = f"{type(exc).__name__}: {exc}".upper()
    return any(marker in text for marker in _HAILO_DEVICE_ERROR_MARKERS)


def hailo_device_holders() -> list[str]:
    """Other processes that appear to have /dev/hailo0 open."""
    hailo = Path("/dev/hailo0")
    if not hailo.exists():
        return ["no /dev/hailo0"]
    me = os.getpid()
    holders: list[str] = []
    try:
        pids = Path("/proc").iterdir()
    except OSError:
        return []
    for pid_dir in pids:
        if not pid_dir.name.isdigit():
            continue
        pid = int(pid_dir.name)
        if pid == me:
            continue
        try:
            comm = (pid_dir / "comm").read_text().strip()
        except OSError:
            continue
        if not (
            comm.startswith("python")
            or comm.startswith("hailort")
            or comm in ("sub", "run.py")
        ):
            continue
        try:
            for fd in (pid_dir / "fd").iterdir():
                try:
                    target = os.readlink(fd)
                except OSError:
                    continue
                if "hailo" in target:
                    holders.append(f"{comm} pid={pid}")
                    break
        except OSError:
            continue
    return holders


def hailo_busy_hint(exc: BaseException | None = None) -> str:
    """One-line reason the Hailo-8L could not be claimed."""
    parts: list[str] = []
    if exc is not None:
        first = str(exc).strip().split("\n")[0]
        parts.append(first[:220] if first else type(exc).__name__)
    holders = hailo_device_holders()
    if holders == ["no /dev/hailo0"]:
        parts.append("Hailo driver has no /dev/hailo0")
    elif holders:
        parts.append("holding the chip: " + ", ".join(holders))
        parts.append("kill leftover ~/sub or hailortcli, then retry Hailo")
    else:
        parts.append(
            "Hailo reported 0 free devices — another process may still "
            "own the HAT, or the driver needs a reboot"
        )
    return " — ".join(parts)


def hailo_export_dir(weights_path: Path) -> Path:
    """Folder that should contain the compiled .hef.

    Accepts a .pt, a .hef, or a Hailo package directory. When several
    candidate folders have a .hef, the newest file wins.
    """
    candidates = hailo_candidate_dirs(weights_path)
    found: list[tuple[float, Path]] = []
    for folder in candidates:
        hef = hailo_hef_path(folder)
        if hef is None:
            continue
        try:
            mtime = hef.stat().st_mtime
        except OSError:
            mtime = 0.0
        found.append((mtime, folder))
    if found:
        found.sort(key=lambda item: item[0], reverse=True)
        return found[0][1]
    conventional = weights_path.with_name(f"{weights_path.stem}_hailo_model")
    return conventional if weights_path.suffix else candidates[0]


def hailo_hef_path(export_dir: Path) -> Path | None:
    if not export_dir.is_dir():
        return None
    hefs = list(export_dir.glob("*.hef"))
    if not hefs:
        hefs = list(export_dir.rglob("*.hef"))
    if not hefs:
        return None
    hefs.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return hefs[0]


def hailo_unavailable_reason(export_dir: Path) -> str:
    return (
        f"No Hailo .hef in {export_dir}.\n"
        "Compile on a Linux x86_64 PC with Hailo Dataflow Compiler 3.x "
        f"(Hailo-8L). DFC wheel: {DEV_ZONE}  Ultralytics notes: {DFC_DOCS}\n"
        "  1. Copy this repo (or weights/<id>/*.pt) to the PC.\n"
        "  2. pip install ultralytics && pip install /path/to/hailo_dataflow_compiler-*.whl\n"
        "  3. python scripts/export_model.py --format hailo --weights weights/detect/best.pt\n"
        "     python scripts/export_model.py --format hailo --weights weights/gate/gatebest.pt\n"
        "     (or: bash scripts/compile_hailo_hef.sh)\n"
        "  4. Copy weights/<id>/hailo/ or weights/<id>/*_hailo_model/ back onto the Pi.\n"
        "  5. Set backend: hailo in config/model.yaml"
    )


def package_status(export_dir: Path) -> tuple[bool, Optional[str]]:
    """True when Detector can load this catalog entry on HailoRT."""
    if hailo_hef_path(export_dir) is not None:
        return True, None
    return False, hailo_unavailable_reason(export_dir)


def class_names_from_export(export_dir: Path) -> dict[int, str]:
    data = _load_export_metadata(export_dir)
    raw = data.get("names") or {}
    if isinstance(raw, dict):
        return {int(k): str(v) for k, v in raw.items()}
    if isinstance(raw, (list, tuple)):
        return {i: str(v) for i, v in enumerate(raw)}
    return {}


def task_from_export(export_dir: Path) -> Optional[str]:
    """Task string from Ultralytics/Hailo metadata.yaml next to a .hef."""
    task = _load_export_metadata(export_dir).get("task")
    if isinstance(task, str) and task.strip():
        return task.strip().lower()
    return None


def _load_export_metadata(export_dir: Path) -> dict:
    meta = export_dir / "metadata.yaml"
    if not meta.is_file():
        return {}
    try:
        data = yaml.safe_load(meta.read_text()) or {}
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def letterbox_rgb(
    bgr: np.ndarray,
    imgsz: int,
    pad_value: int = 114,
    dst: np.ndarray | None = None,
) -> tuple[np.ndarray, float, tuple[float, float]]:
    """Resize with stride-free letterbox to imgsz×imgsz RGB uint8."""
    import cv2

    h, w = bgr.shape[:2]
    r = min(imgsz / float(h), imgsz / float(w))
    new_w, new_h = int(round(w * r)), int(round(h * r))
    dw = (imgsz - new_w) / 2.0
    dh = (imgsz - new_h) / 2.0
    top = int(round(dh - 0.1))
    left = int(round(dw - 0.1))
    if dst is None:
        out = np.empty((imgsz, imgsz, 3), dtype=np.uint8)
    else:
        out = dst
        if out.shape[0] != imgsz or out.shape[1] != imgsz or out.shape[2] != 3:
            raise ValueError("letterbox dst must be imgsz×imgsz×3")
    out[:] = pad_value
    if (new_w, new_h) != (w, h):
        resized = cv2.resize(bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    else:
        resized = bgr
    new_h = min(new_h, imgsz - top)
    new_w = min(new_w, imgsz - left)
    if resized.shape[0] != new_h or resized.shape[1] != new_w:
        resized = resized[:new_h, :new_w]
    roi = out[top : top + new_h, left : left + new_w]
    if roi.flags["C_CONTIGUOUS"]:
        cv2.cvtColor(resized, cv2.COLOR_BGR2RGB, dst=roi)
    else:
        roi[...] = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    return out, r, (dw, dh)


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
) -> tuple[int, int, int, int] | None:
    """Hailo NMS boxes are ymin,xmin,ymax,xmax in 0–1 of the letterboxed square."""
    dw, dh = pad
    if max(ymin, xmin, ymax, xmax) <= 1.5:
        ymin, xmin, ymax, xmax = (
            ymin * imgsz, xmin * imgsz, ymax * imgsz, xmax * imgsz
        )
    if ratio <= 1e-9:
        return None
    x1 = (xmin - dw) / ratio
    y1 = (ymin - dh) / ratio
    x2 = (xmax - dw) / ratio
    y2 = (ymax - dh) / ratio
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    # Letterbox pad / packed-NMS junk collapses to a point at the origin.
    if x2 <= 0.0 or y2 <= 0.0 or x1 >= frame_w or y1 >= frame_h:
        return None
    x1 = int(max(0, min(frame_w - 1, round(x1))))
    y1 = int(max(0, min(frame_h - 1, round(y1))))
    x2 = int(max(0, min(frame_w - 1, round(x2))))
    y2 = int(max(0, min(frame_h - 1, round(y2))))
    if (x2 - x1) < 2 or (y2 - y1) < 2:
        return None
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
            scaled = scale_hailo_box(
                ymin, xmin, ymax, xmax,
                imgsz=imgsz, ratio=ratio, pad=pad,
                frame_w=frame_w, frame_h=frame_h,
            )
            if scaled is None:
                continue
            x1, y1, x2, y2 = scaled
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


def detections_from_yolov8_raw(
    outputs,
    *,
    names: dict[int, str],
    confidence: float,
    imgsz: int,
    ratio: float,
    pad: tuple[float, float],
    frame_w: int,
    frame_h: int,
    max_det: int = 5,
    iou: float = 0.7,
) -> list:
    """Decode a Hailo YOLOv8/seg HEF that was compiled without on-chip NMS.

    Typical heads (NHWC): box DFL (H,W,64), class (H,W,nc), mask coeff
    (H,W,32) at strides 8/16/32, plus a 160×160 proto. Boxes are enough
    for tracking; proto is ignored.
    """
    from src.detector import Detection

    layers = _hwc_layers(outputs)
    if not layers:
        return []
    nc = max((len(names), 1))
    candidates: list[tuple[float, int, float, float, float, float]] = []
    for (h, w), group in layers.items():
        if h < 8 or w < 8:
            continue
        stride = imgsz / float(h)
        if stride < 7.5 or abs(stride - round(stride)) > 0.51:
            continue
        stride = float(round(stride))
        box = _layer_with_channels(group, 64)
        cls = _layer_with_channels(group, nc)
        if cls is None:
            cls = _best_class_map(group)
        if box is None or cls is None:
            continue
        if cls.shape[2] == 32:
            cls = _best_class_map(group)
            if cls is None:
                continue
        scores = _maybe_sigmoid(cls)
        peak = scores.max(axis=-1)
        keep = peak >= confidence
        if not np.any(keep):
            continue
        ltrb = _dfl_ltrb(box)
        ys, xs = np.nonzero(keep)
        for y, x in zip(ys.tolist(), xs.tolist()):
            cls_id = int(scores[y, x].argmax())
            score = float(scores[y, x, cls_id])
            cx = (x + 0.5) * stride
            cy = (y + 0.5) * stride
            l, t, r, b = (float(v) * stride for v in ltrb[y, x])
            candidates.append((score, cls_id, cx - l, cy - t, cx + r, cy + b))

    candidates.sort(key=lambda c: c[0], reverse=True)
    picked = _nms_xyxy(candidates, iou=iou, max_det=max_det)
    detections = []
    for score, cls_id, x1, y1, x2, y2 in picked:
        scaled = scale_hailo_box(
            y1, x1, y2, x2,
            imgsz=imgsz, ratio=ratio, pad=pad,
            frame_w=frame_w, frame_h=frame_h,
        )
        if scaled is None:
            continue
        sx1, sy1, sx2, sy2 = scaled
        detections.append(
            Detection(
                x1=sx1, y1=sy1, x2=sx2, y2=sy2,
                confidence=score,
                class_id=cls_id,
                label=names.get(cls_id, str(cls_id)),
            )
        )
    return detections


def _hwc_layers(outputs) -> dict[tuple[int, int], list[np.ndarray]]:
    if isinstance(outputs, dict):
        arrays = list(outputs.values())
    elif isinstance(outputs, (list, tuple)):
        arrays = list(outputs)
    else:
        arrays = [outputs]
    grouped: dict[tuple[int, int], list[np.ndarray]] = {}
    for raw in arrays:
        arr = _as_hwc(raw)
        if arr is None:
            continue
        grouped.setdefault((arr.shape[0], arr.shape[1]), []).append(arr)
    return grouped


def _as_hwc(raw) -> np.ndarray | None:
    arr = np.asarray(raw)
    if arr.dtype == object:
        return None
    arr = np.squeeze(arr)
    if arr.ndim == 2:
        arr = arr[..., None]
    if arr.ndim != 3:
        return None
    return arr.astype(np.float32, copy=False)


def _layer_with_channels(group: list[np.ndarray], channels: int) -> np.ndarray | None:
    for arr in group:
        if arr.shape[2] == channels:
            return arr
    return None


def _best_class_map(group: list[np.ndarray]) -> np.ndarray | None:
    """Class scores are the small-C map that is not DFL (64) or mask (32)."""
    ranked = sorted(
        (arr for arr in group if arr.shape[2] not in (32, 64)),
        key=lambda a: a.shape[2],
    )
    return ranked[0] if ranked else None


def _maybe_sigmoid(arr: np.ndarray) -> np.ndarray:
    if arr.size == 0:
        return arr
    lo = float(arr.min())
    hi = float(arr.max())
    if lo >= 0.0 and hi <= 1.01:
        return arr
    x = np.clip(arr, -30.0, 30.0)
    return 1.0 / (1.0 + np.exp(-x))


def _dfl_ltrb(box_hw64: np.ndarray, reg_max: int = 16) -> np.ndarray:
    """(H,W,64) DFL logits → (H,W,4) ltrb in grid cells."""
    h, w, c = box_hw64.shape
    x = box_hw64.reshape(h, w, 4, reg_max)
    x = x - x.max(axis=-1, keepdims=True)
    e = np.exp(x)
    p = e / np.clip(e.sum(axis=-1, keepdims=True), 1e-9, None)
    bins = np.arange(reg_max, dtype=np.float32)
    return np.tensordot(p, bins, axes=([-1], [0]))


def _nms_xyxy(
    candidates: list[tuple[float, int, float, float, float, float]],
    *,
    iou: float,
    max_det: int,
) -> list[tuple[float, int, float, float, float, float]]:
    kept: list[tuple[float, int, float, float, float, float]] = []
    for cand in candidates:
        _, _, x1, y1, x2, y2 = cand
        if any(_iou_xyxy((x1, y1, x2, y2), (k[2], k[3], k[4], k[5])) > iou for k in kept):
            continue
        kept.append(cand)
        if len(kept) >= max_det:
            break
    return kept


def _iou_xyxy(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if inter <= 0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    denom = area_a + area_b - inter
    return inter / denom if denom > 0 else 0.0


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
            # One box is 5 (or 6) values. A packed NMS buffer is much longer
            # and must not be sliced into a fake origin detection.
            return [boxes[:5]] if boxes.size in (5, 6) else []
        if boxes.ndim >= 2:
            # Hailo packed NMS is often (5, N) not (N, 5).
            if boxes.shape[0] in (5, 6) and boxes.shape[-1] not in (5, 6):
                boxes = np.moveaxis(boxes, 0, -1)
            if boxes.shape[-1] < 5 or boxes.shape[-1] > 6:
                return []
            flat = boxes.reshape(-1, boxes.shape[-1])
            return [row[:5] for row in flat]
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
    if isinstance(data, np.ndarray) and data.dtype != object and data.ndim == 3:
        # On-chip NMS tensor: (classes, 5, max_prop) or (classes, max_prop, 5).
        if data.shape[1] in (5, 6) and data.shape[-1] not in (5, 6):
            return [np.moveaxis(data[c], 0, -1) for c in range(data.shape[0])]
        if data.shape[-1] in (5, 6):
            return [data[c] for c in range(data.shape[0])]
    # InferVStreams: [ [cls0, cls1, ...] ] for batch=1.
    if (
        isinstance(data, (list, tuple))
        and len(data) == 1
        and isinstance(data[0], (list, tuple))
        and data[0]
        and isinstance(data[0][0], np.ndarray)
    ):
        return list(data[0])
    if (
        isinstance(data, (list, tuple))
        and data
        and isinstance(data[0], np.ndarray)
        and data[0].dtype != object
    ):
        return list(data)
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
    """HailoRT runner for a YOLOv8 HEF with on-chip NMS.

    The network group stays activated. Detect HEFs use on-chip NMS;
    segment HEFs (``nms: false``) are decoded on the host from the raw
    YOLOv8 heads. ``predict_pair`` letterboxes the second camera while
    the first infer runs.
    """

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
        self._prep = ThreadPoolExecutor(max_workers=1, thread_name_prefix="hailo-lb")

        try:
            hef = HEF(str(self.hef_path))
            input_info = hef.get_input_vstream_infos()[0]
            output_infos = list(hef.get_output_vstream_infos())
            nms_infos = [info for info in output_infos if "nms" in info.name.lower()]
            self._input_name = input_info.name
            self._output_name = (nms_infos[0].name if nms_infos else output_infos[0].name)
            self._decode_mode = "nms" if nms_infos else "yolov8_raw"
            shape = tuple(int(v) for v in input_info.shape)
            # NHWC (H, W, C) or (N, H, W, C)
            if len(shape) == 4:
                self.img_size = int(shape[1])
            elif len(shape) >= 2:
                self.img_size = int(shape[0])

            self._input = np.empty(
                (1, self.img_size, self.img_size, 3), dtype=np.uint8
            )
            self._input_b = np.empty_like(self._input)

            self._claim_device(
                hef,
                VDevice=VDevice,
                ConfigureParams=ConfigureParams,
                FormatType=FormatType,
                HailoStreamInterface=HailoStreamInterface,
                InferVStreams=InferVStreams,
                InputVStreamParams=InputVStreamParams,
                OutputVStreamParams=OutputVStreamParams,
            )
        except Exception:
            self.close()
            raise
        print(
            f"[HailoYOLO] {self.hef_path.parent.name}/{self.hef_path.name} "
            f"held on device  decode={self._decode_mode} "
            f"in={self._input_name} out={self._output_name} "
            f"outs={len(output_infos)} imgsz={self.img_size}"
        )

    def _claim_device(self, hef, **api) -> None:
        VDevice = api["VDevice"]
        last: Optional[BaseException] = None
        for attempt in range(2):
            try:
                self._target = VDevice()
                last = None
                break
            except Exception as exc:
                last = exc
                if attempt == 0 and is_hailo_device_error(exc):
                    print(f"[HailoYOLO] {hailo_busy_hint(exc)}; retrying once")
                    time.sleep(0.5)
                    continue
                raise
        if last is not None:
            raise last
        configure_params = api["ConfigureParams"].create_from_hef(
            hef, interface=api["HailoStreamInterface"].PCIe
        )
        network_group = self._target.configure(hef, configure_params)[0]
        # AUTO = HEF native UINT8 NHWC (no host convert). Queue 2 for vstream pipeline.
        input_vstreams_params = api["InputVStreamParams"].make_from_network_group(
            network_group, format_type=api["FormatType"].AUTO, queue_size=2
        )
        output_vstreams_params = api["OutputVStreamParams"].make_from_network_group(
            network_group, format_type=api["FormatType"].FLOAT32, queue_size=2
        )
        self._network_group = network_group
        self._ng_params = network_group.create_params()
        self._infer = api["InferVStreams"](
            network_group, input_vstreams_params, output_vstreams_params
        )
        self._infer.__enter__()
        self._activated = network_group.activate(self._ng_params)
        self._activated.__enter__()

    def close(self) -> None:
        if self._prep is not None:
            self._prep.shutdown(wait=False, cancel_futures=True)
            self._prep = None
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

    def _run_hef(self, batch: np.ndarray):
        """One frame through the activated HEF."""
        if self._infer is None:
            raise RuntimeError("HailoYOLO is closed")
        raw = self._infer.infer({self._input_name: batch})
        if self._decode_mode == "nms":
            if isinstance(raw, dict):
                return raw.get(self._output_name, next(iter(raw.values())))
            return raw
        return raw

    def _decode_outputs(self, raw, ratio, pad, frame_w, frame_h, max_det: int) -> list:
        try:
            if self._decode_mode == "nms":
                return detections_from_nms(
                    raw,
                    names=self.names,
                    confidence=self.confidence,
                    imgsz=self.img_size,
                    ratio=ratio,
                    pad=pad,
                    frame_w=frame_w,
                    frame_h=frame_h,
                    max_det=max_det,
                )
            return detections_from_yolov8_raw(
                raw,
                names=self.names,
                confidence=self.confidence,
                imgsz=self.img_size,
                ratio=ratio,
                pad=pad,
                frame_w=frame_w,
                frame_h=frame_h,
                max_det=max_det,
            )
        except (TypeError, ValueError) as exc:
            now = time.monotonic()
            if now - getattr(self, "_nms_log_at", 0.0) >= 5.0:
                self._nms_log_at = now
                print(f"[HailoYOLO] decode failed ({type(exc).__name__}: {exc})")
            return []

    def predict(self, frame: np.ndarray, max_det: int = 5) -> list:
        h, w = frame.shape[:2]
        _, ratio, pad = letterbox_rgb(frame, self.img_size, dst=self._input[0])
        raw = self._run_hef(self._input)
        return self._decode_outputs(raw, ratio, pad, w, h, max_det)

    def predict_pair(
        self,
        left: np.ndarray,
        right: np.ndarray,
        max_det: int = 5,
    ) -> tuple[list, list]:
        """Infer two frames; letterbox the second while the first HEF call runs."""
        hr, wr = right.shape[:2]
        fut = self._prep.submit(
            letterbox_rgb, right, self.img_size, 114, self._input_b[0]
        )
        dets_left = self.predict(left, max_det=max_det)
        _, ratio_r, pad_r = fut.result()
        raw_r = self._run_hef(self._input_b)
        dets_right = self._decode_outputs(raw_r, ratio_r, pad_r, wr, hr, max_det)
        return dets_left, dets_right


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
