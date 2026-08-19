"""Hailo package helpers (no Hailo hardware required)."""

from pathlib import Path

import numpy as np

from src.hailo_runtime import (
    detections_from_nms,
    hailo_hef_path,
    package_status,
)
from src.model_runtime import model_availability


def test_package_status_needs_hef(tmp_path: Path):
    ok, reason = package_status(tmp_path)
    assert not ok
    assert reason
    assert "x86_64" in reason
    assert "DFC" in reason or "hef" in reason.lower()


def test_package_status_with_hef(tmp_path: Path):
    (tmp_path / "best.hef").write_bytes(b"hef")
    ok, reason = package_status(tmp_path)
    assert ok
    assert reason is None
    assert hailo_hef_path(tmp_path).name == "best.hef"


def test_model_availability_hailo_detect_has_hef():
    avail, reason = model_availability(
        {"weights": "weights/best.pt"},
        "hailo",
    )
    assert avail
    assert reason is None


def test_model_availability_hailo_gate_without_hef():
    avail, reason = model_availability(
        {"weights": "weights/gatebest.pt"},
        "hailo",
    )
    assert not avail
    assert reason
    assert "hef" in reason.lower() or "x86" in reason.lower()


def test_nms_decode_normalized_boxes():
    names = {0: "apple", 1: "damaged_apple"}
    # ymin, xmin, ymax, xmax, score in 0–1 of the letterboxed square
    nms = [np.array([[0.1, 0.2, 0.5, 0.6, 0.9]]), np.zeros((0, 5))]
    dets = detections_from_nms(
        nms,
        names=names,
        confidence=0.5,
        imgsz=640,
        ratio=1.0,
        pad=(0.0, 0.0),
        frame_w=640,
        frame_h=640,
        max_det=5,
    )
    assert len(dets) == 1
    assert dets[0].label == "apple"
    assert dets[0].x1 == 128  # 0.2 * 640
    assert dets[0].y1 == 64   # 0.1 * 640


def test_nms_filters_confidence():
    nms = [np.array([[0.0, 0.0, 1.0, 1.0, 0.2]]), np.zeros((0, 5))]
    dets = detections_from_nms(
        nms,
        names={0: "apple"},
        confidence=0.5,
        imgsz=640,
        ratio=1.0,
        pad=(0.0, 0.0),
        frame_w=640,
        frame_h=640,
    )
    assert dets == []


def test_nms_inhomogeneous_two_classes():
    """Hailo NMS-by-class: each class has a different number of boxes."""
    cls0 = np.array([[0.1, 0.2, 0.5, 0.6, 0.9]], dtype=np.float32)
    cls1 = np.array(
        [[0.0, 0.0, 0.2, 0.2, 0.8], [0.3, 0.3, 0.4, 0.4, 0.7]],
        dtype=np.float32,
    )
    batched = np.empty((1,), dtype=object)
    batched[0] = [cls0, cls1]
    dets = detections_from_nms(
        {"out": batched},
        names={0: "apple", 1: "damaged_apple"},
        confidence=0.5,
        imgsz=640,
        ratio=1.0,
        pad=(0.0, 0.0),
        frame_w=640,
        frame_h=640,
        max_det=5,
    )
    assert [d.label for d in dets] == ["apple", "damaged_apple", "damaged_apple"]
    assert abs(dets[0].confidence - 0.9) < 1e-5

    # Same ragged tensors as a bare list (typical HailoRT NMS-by-class).
    dets2 = detections_from_nms(
        [cls0, cls1],
        names={0: "apple", 1: "damaged_apple"},
        confidence=0.5,
        imgsz=640,
        ratio=1.0,
        pad=(0.0, 0.0),
        frame_w=640,
        frame_h=640,
        max_det=5,
    )
    assert len(dets2) == 3
