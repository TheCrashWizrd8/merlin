"""Unit tests for detector helpers (no Ultralytics / GPU required)."""

import os
from pathlib import Path

import numpy as np

from src.detector import Detection, mask_centroid_and_area, _pt_is_newer_than_export


def test_mask_centroid_square():
    xy = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.float32)
    cx, cy, area = mask_centroid_and_area(xy)
    assert cx == 5
    assert cy == 5
    assert 99 <= area <= 101


def test_mask_centroid_empty():
    assert mask_centroid_and_area(np.zeros((0, 2))) == (-1, -1, 0)
    assert mask_centroid_and_area(np.array([[1.0, 2.0]])) == (-1, -1, 0)


def test_detection_center_falls_back_to_bbox():
    d = Detection(0, 0, 10, 20, 0.9, 0, "apple")
    assert d.center() == (5, 10)


def test_detection_center_uses_mask():
    d = Detection(0, 0, 10, 20, 0.9, 0, "apple", cx=3, cy=7, mask_area=12)
    assert d.center() == (3, 7)


def test_pt_newer_than_export(tmp_path: Path):
    pt = tmp_path / "best.pt"
    export = tmp_path / "best_ncnn_model"
    export.mkdir()
    (export / "model.bin").write_bytes(b"old")
    pt.write_bytes(b"new")
    older = (export / "model.bin").stat().st_mtime - 10
    os.utime(export / "model.bin", (older, older))
    assert _pt_is_newer_than_export(pt, export)
