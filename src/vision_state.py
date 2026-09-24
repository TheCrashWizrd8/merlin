"""
vision_state.py
---------------
Thread-safe detection / overlay payload for dashboard and future FOV compositing.
"""

from __future__ import annotations

import threading
import time
from dataclasses import asdict, is_dataclass
from typing import Any

_lock = threading.Lock()
_state: dict[str, Any] = {
    "ts": 0.0,
    "model_id": "",
    "backend": "",
    "running": False,
    "left": {"width": 0, "height": 0, "objects": [], "track": None},
    "right": {"width": 0, "height": 0, "objects": [], "track": None},
    "fov": {"width": 0, "height": 0, "objects": [], "track": None},
    "stereo": {"ok": False, "range_m": None, "reason": ""},
    "overlay": {
        "fov_from_stereo_ready": False,
        "note": "FOV boxes use a normalized left/right average until camera extrinsics are calibrated.",
    },
}


def _bbox_from_track(track) -> dict[str, Any] | None:
    if track is None or not getattr(track, "apple_detected", False):
        return None
    return {
        "label": getattr(track, "label", "apple"),
        "confidence": float(getattr(track, "confidence", 0.0) or 0.0),
        "x1": int(getattr(track, "bbox_x1", 0)),
        "y1": int(getattr(track, "bbox_y1", 0)),
        "x2": int(getattr(track, "bbox_x2", 0)),
        "y2": int(getattr(track, "bbox_y2", 0)),
        "cx": int(getattr(track, "target_x", 0)),
        "cy": int(getattr(track, "target_y", 0)),
    }


def _objects_from_detections(detections) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    if not detections:
        return out
    for det in detections:
        if is_dataclass(det):
            d = asdict(det)
        elif isinstance(det, dict):
            d = det
        else:
            continue
        try:
            out.append({
                "label": str(d.get("label", d.get("class_name", ""))),
                "confidence": float(d.get("confidence", d.get("conf", 0.0)) or 0.0),
                "x1": int(d.get("x1", d.get("bbox_x1", 0))),
                "y1": int(d.get("y1", d.get("bbox_y1", 0))),
                "x2": int(d.get("x2", d.get("bbox_x2", 0))),
                "y2": int(d.get("y2", d.get("bbox_y2", 0))),
            })
        except (TypeError, ValueError):
            continue
    return out


def _box_to_norm(box: dict[str, Any] | None, width: int, height: int) -> dict[str, Any] | None:
    if not box or width <= 0 or height <= 0:
        return None
    try:
        x1 = float(box["x1"]) / width
        y1 = float(box["y1"]) / height
        x2 = float(box["x2"]) / width
        y2 = float(box["y2"]) / height
    except (KeyError, TypeError, ValueError, ZeroDivisionError):
        return None
    cx = box.get("cx")
    cy = box.get("cy")
    ncx = (
        float(cx) / width
        if cx is not None
        else (x1 + x2) / 2.0
    )
    ncy = (
        float(cy) / height
        if cy is not None
        else (y1 + y2) / 2.0
    )
    return {
        "label": str(box.get("label") or ""),
        "confidence": float(box.get("confidence") or 0.0),
        "x1": x1,
        "y1": y1,
        "x2": x2,
        "y2": y2,
        "cx": ncx,
        "cy": ncy,
    }


def _norm_to_box(norm: dict[str, Any], width: int, height: int) -> dict[str, Any]:
    return {
        "label": str(norm.get("label") or ""),
        "confidence": float(norm.get("confidence") or 0.0),
        "x1": int(round(float(norm["x1"]) * width)),
        "y1": int(round(float(norm["y1"]) * height)),
        "x2": int(round(float(norm["x2"]) * width)),
        "y2": int(round(float(norm["y2"]) * height)),
        "cx": int(round(float(norm["cx"]) * width)),
        "cy": int(round(float(norm["cy"]) * height)),
    }


def _mean_norm(norms: list[dict[str, Any]]) -> dict[str, Any] | None:
    if not norms:
        return None
    n = float(len(norms))
    return {
        "label": str(norms[0].get("label") or ""),
        "confidence": max(float(item.get("confidence") or 0.0) for item in norms),
        "x1": sum(float(item["x1"]) for item in norms) / n,
        "y1": sum(float(item["y1"]) for item in norms) / n,
        "x2": sum(float(item["x2"]) for item in norms) / n,
        "y2": sum(float(item["y2"]) for item in norms) / n,
        "cx": sum(float(item["cx"]) for item in norms) / n,
        "cy": sum(float(item["cy"]) for item in norms) / n,
    }


def _project_views_to_fov(
    left: dict[str, Any],
    right: dict[str, Any],
    fov_w: int,
    fov_h: int,
) -> tuple[list[dict[str, Any]], dict[str, Any] | None]:
    """Map YOLO boxes onto FOV using a normalized average (extrinsics later)."""
    if fov_w <= 0 or fov_h <= 0:
        return [], None
    lw, lh = int(left.get("width") or 0), int(left.get("height") or 0)
    rw, rh = int(right.get("width") or 0), int(right.get("height") or 0)

    def _pair_objects() -> list[dict[str, Any]]:
        left_objs = list(left.get("objects") or [])
        right_objs = list(right.get("objects") or [])
        count = max(len(left_objs), len(right_objs))
        out: list[dict[str, Any]] = []
        for i in range(count):
            parts: list[dict[str, Any]] = []
            if i < len(left_objs):
                norm = _box_to_norm(left_objs[i], lw, lh)
                if norm:
                    parts.append(norm)
            if i < len(right_objs):
                norm = _box_to_norm(right_objs[i], rw, rh)
                if norm:
                    parts.append(norm)
            mean = _mean_norm(parts)
            if mean:
                out.append(_norm_to_box(mean, fov_w, fov_h))
        return out

    objects = _pair_objects()
    track_parts: list[dict[str, Any]] = []
    left_track = _box_to_norm(left.get("track"), lw, lh)
    right_track = _box_to_norm(right.get("track"), rw, rh)
    if left_track:
        track_parts.append(left_track)
    if right_track:
        track_parts.append(right_track)
    track_norm = _mean_norm(track_parts)
    track = _norm_to_box(track_norm, fov_w, fov_h) if track_norm else None
    if track is None and objects:
        track = dict(objects[0])
    return objects, track


def update_vision(
    *,
    model_id: str = "",
    backend: str = "",
    running: bool = False,
    left_shape: tuple[int, int] | None = None,
    right_shape: tuple[int, int] | None = None,
    fov_shape: tuple[int, int] | None = None,
    detections_left=None,
    detections_right=None,
    track_left=None,
    track_right=None,
    stereo_ok: bool = False,
    range_m: float | None = None,
    stereo_reason: str = "",
) -> None:
    with _lock:
        _state["ts"] = time.time()
        _state["model_id"] = model_id
        _state["backend"] = backend
        _state["running"] = bool(running)
        if left_shape:
            h, w = left_shape[:2]
            _state["left"]["width"] = int(w)
            _state["left"]["height"] = int(h)
        if right_shape:
            h, w = right_shape[:2]
            _state["right"]["width"] = int(w)
            _state["right"]["height"] = int(h)
        if fov_shape:
            h, w = fov_shape[:2]
            _state["fov"]["width"] = int(w)
            _state["fov"]["height"] = int(h)
        _state["left"]["objects"] = _objects_from_detections(detections_left)
        _state["right"]["objects"] = _objects_from_detections(detections_right)
        _state["left"]["track"] = _bbox_from_track(track_left)
        _state["right"]["track"] = _bbox_from_track(track_right)
        fov_w = int(_state["fov"]["width"] or _state["left"]["width"] or 0)
        fov_h = int(_state["fov"]["height"] or _state["left"]["height"] or 0)
        if fov_w and fov_h:
            _state["fov"]["width"] = fov_w
            _state["fov"]["height"] = fov_h
        objects, track = _project_views_to_fov(
            _state["left"], _state["right"], fov_w, fov_h
        )
        _state["fov"]["objects"] = objects
        _state["fov"]["track"] = track
        _state["stereo"] = {
            "ok": bool(stereo_ok),
            "range_m": range_m,
            "reason": stereo_reason or "",
        }
        _state["overlay"] = {
            "fov_from_stereo_ready": bool(track or objects),
            "note": (
                "FOV boxes are the normalized average of YOLO left/right "
                "(camera extrinsics / angle TBD)."
            ),
        }


def clear_vision() -> None:
    with _lock:
        _state["ts"] = time.time()
        _state["model_id"] = ""
        _state["backend"] = ""
        _state["running"] = False
        for key in ("left", "right", "fov"):
            _state[key] = {"width": 0, "height": 0, "objects": [], "track": None}
        _state["stereo"] = {"ok": False, "range_m": None, "reason": ""}
        _state["overlay"] = {
            "fov_from_stereo_ready": False,
            "note": "FOV boxes use a normalized left/right average until camera extrinsics are calibrated.",
        }


def vision_snapshot() -> dict[str, Any]:
    with _lock:
        return {
            "ts": _state["ts"],
            "model_id": _state["model_id"],
            "backend": _state["backend"],
            "running": _state["running"],
            "left": dict(_state["left"]),
            "right": dict(_state["right"]),
            "fov": dict(_state["fov"]),
            "stereo": dict(_state["stereo"]),
            "overlay": dict(_state["overlay"]),
        }
