"""Hailo package helpers (no Hailo hardware required)."""

from pathlib import Path

import numpy as np

from src.detector import resolve_model_source
from src.hailo_runtime import (
    detections_from_nms,
    hailo_busy_hint,
    hailo_export_dir,
    hailo_hef_path,
    is_hailo_device_error,
    package_status,
)
from src.model_runtime import model_availability


def test_hailo_export_dir_accepts_hef_path(tmp_path: Path):
    hef = tmp_path / "pack" / "best.hef"
    hef.parent.mkdir()
    hef.write_bytes(b"hef")
    assert hailo_export_dir(hef) == hef.parent
    assert hailo_export_dir(tmp_path / "best.pt") == tmp_path / "best_hailo_model"


def test_hailo_export_dir_prefers_besthailomodel(tmp_path: Path):
    import os
    import time

    compact = tmp_path / "besthailomodel"
    compact.mkdir()
    new_hef = compact / "best.hef"
    new_hef.write_bytes(b"new")
    conventional = tmp_path / "best_hailo_model"
    conventional.mkdir()
    old_hef = conventional / "best.hef"
    old_hef.write_bytes(b"old")
    now = time.time()
    os.utime(old_hef, (now - 60, now - 60))
    os.utime(new_hef, (now, now))
    assert hailo_export_dir(tmp_path / "best.pt") == compact
    assert hailo_hef_path(compact).name == "best.hef"


def test_hailo_export_dir_accepts_hailo_subdir(tmp_path: Path):
    hailo = tmp_path / "hailo"
    hailo.mkdir()
    (hailo / "model.hef").write_bytes(b"hef")
    assert hailo_export_dir(tmp_path / "best.pt") == hailo
    assert hailo_hef_path(hailo).name == "model.hef"


def test_resolve_model_source_hailo_from_pt_or_hef():
    source, desc = resolve_model_source(
        architecture="yolov8n",
        weights_value="weights/best.pt",
        backend="hailo",
    )
    assert source.endswith("best.hef")
    assert "besthailomodel" in source.replace("\\", "/")
    assert "hailo" in desc.lower()
    source2, _ = resolve_model_source(
        architecture="yolov8n",
        weights_value="weights/besthailomodel/best.hef",
        backend="hailo",
    )
    assert source2.endswith("best.hef")


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


def test_model_availability_direct_hef_path():
    avail, reason = model_availability(
        {"weights": "weights/besthailomodel/best.hef"},
        "hailo",
    )
    assert avail
    assert reason is None
    pytorch_ok, _ = model_availability(
        {"weights": "weights/besthailomodel/best.hef"},
        "pytorch",
    )
    assert not pytorch_ok


def test_model_availability_hailo_gate_uses_hef_if_present():
    avail, reason = model_availability(
        {"weights": "weights/gatebest.pt"},
        "hailo",
    )
    if avail:
        assert reason is None
    else:
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


def test_nms_drops_packed_header_and_pad_collapse():
    """Raw NMS buffers start with a count; that must not become a (0,0) apple."""
    names = {0: "apple"}
    packed = np.array([2.0, 0.1, 0.2, 0.3, 0.9], dtype=np.float32)
    dets = detections_from_nms(
        [packed, np.zeros((0, 5))],
        names=names,
        confidence=0.5,
        imgsz=640,
        ratio=0.4,
        pad=(0.0, 80.0),
        frame_w=1600,
        frame_h=1200,
    )
    assert dets == []

    tiny = [np.array([[0.0, 0.0, 0.002, 0.002, 0.99]], dtype=np.float32)]
    dets2 = detections_from_nms(
        tiny,
        names=names,
        confidence=0.5,
        imgsz=640,
        ratio=0.4,
        pad=(0.0, 80.0),
        frame_w=1600,
        frame_h=1200,
    )
    assert dets2 == []


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


def test_nms_hailo_packed_classes_5_proposals():
    """Native Hailo NMS tensor is (classes, 5, max_proposals)."""
    packed = np.zeros((2, 5, 4), dtype=np.float32)
    packed[0, :, 0] = [0.1, 0.2, 0.5, 0.6, 0.9]
    packed[1, :, 1] = [0.0, 0.0, 0.2, 0.2, 0.8]
    dets = detections_from_nms(
        packed,
        names={0: "apple", 1: "damaged_apple"},
        confidence=0.5,
        imgsz=640,
        ratio=1.0,
        pad=(0.0, 0.0),
        frame_w=640,
        frame_h=640,
        max_det=5,
    )
    assert [d.label for d in dets] == ["apple", "damaged_apple"]


def test_yolov8_raw_decodes_class_peak():
    from src.hailo_runtime import detections_from_yolov8_raw

    outputs = {}
    for h, tag in ((80, "s8"), (40, "s16"), (20, "s32")):
        box = np.zeros((h, h, 64), dtype=np.float32)
        for side in range(4):
            box[..., side * 16 + 2] = 10.0
        outputs[f"{tag}_box"] = box
        outputs[f"{tag}_cls"] = np.full((h, h, 1), -10.0, dtype=np.float32)
        outputs[f"{tag}_m"] = np.zeros((h, h, 32), dtype=np.float32)
    outputs["proto"] = np.zeros((160, 160, 32), dtype=np.float32)
    outputs["s8_cls"][40, 40, 0] = 8.0
    dets = detections_from_yolov8_raw(
        outputs,
        names={0: "gate"},
        confidence=0.5,
        imgsz=640,
        ratio=1.0,
        pad=(0.0, 0.0),
        frame_w=640,
        frame_h=640,
        max_det=5,
    )
    assert len(dets) == 1
    assert dets[0].label == "gate"
    assert dets[0].confidence > 0.9
    assert dets[0].x2 - dets[0].x1 >= 2
    assert dets[0].y2 - dets[0].y1 >= 2


def test_yolov8_raw_ignores_box_head_alone():
    from src.hailo_runtime import detections_from_yolov8_raw

    dets = detections_from_yolov8_raw(
        {"only_box": np.zeros((80, 80, 64), dtype=np.float32)},
        names={0: "gate"},
        confidence=0.5,
        imgsz=640,
        ratio=1.0,
        pad=(0.0, 0.0),
        frame_w=640,
        frame_h=640,
    )
    assert dets == []


def test_letterbox_writes_into_dst():
    from src.hailo_runtime import letterbox_rgb

    bgr = np.zeros((480, 640, 3), dtype=np.uint8)
    bgr[:] = (0, 0, 255)  # red in BGR
    dst = np.empty((640, 640, 3), dtype=np.uint8)
    out, ratio, pad = letterbox_rgb(bgr, 640, dst=dst)
    assert out is dst
    assert out.shape == (640, 640, 3)
    assert abs(ratio - 1.0) < 1e-6
    # Full-width 480-tall image: pad is top/bottom, centre pixel is red in RGB.
    assert tuple(int(v) for v in out[320, 320]) == (255, 0, 0)
    assert tuple(int(v) for v in out[0, 0]) == (114, 114, 114)


def test_is_hailo_device_error():
    assert is_hailo_device_error(
        RuntimeError("libhailort failed with error: 74 (HAILO_OUT_OF_PHYSICAL_DEVICES)")
    )
    assert is_hailo_device_error(
        RuntimeError(
            "Failed to create vdevice. there are not enough free devices. "
            "requested: 1, found: 0"
        )
    )
    assert not is_hailo_device_error(RuntimeError("file not found"))


def test_hailo_busy_hint_includes_error():
    hint = hailo_busy_hint(RuntimeError("HAILO_OUT_OF_PHYSICAL_DEVICES"))
    assert "HAILO_OUT_OF_PHYSICAL_DEVICES" in hint
    assert hailo_busy_hint()
