"""LinkBack USB camera: packet assembly, device specs, and the Camera-compatible wrapper."""

import numpy as np
import pytest

from src import camera as camera_mod
from src.camera import Camera, CameraError, make_camera
from src.linkback_camera import (
    AutoExposure,
    FRAME_BYTES,
    FRAME_H,
    FRAME_W,
    FrameAssembler,
    LinkbackCamera,
    is_linkback_spec,
    spec_port_path,
    yuy2_to_bgr,
)

PAYLOAD = 500  # 512-byte packet minus the 12-byte header


def _packets(frame: bytes, fid: int, *, eof: bool = True, payload: int = PAYLOAD) -> bytes:
    """One frame as UVC-style 512-byte packets (12-byte header, EOF on the last)."""
    out = bytearray()
    chunks = [frame[i : i + payload] for i in range(0, len(frame), payload)]
    for n, chunk in enumerate(chunks):
        flags = 0x8C | fid
        if eof and n == len(chunks) - 1:
            flags |= 0x02
        out += bytes([0x0C, flags]) + bytes(10) + chunk
    return bytes(out)


def test_spec_parsing():
    assert is_linkback_spec("linkback")
    assert is_linkback_spec(" LinkBack:3-1 ")
    assert not is_linkback_spec("/dev/v4l/by-path/usb-0:2:1.0-video-index0")
    assert not is_linkback_spec(2)
    assert spec_port_path("linkback") == ""
    assert spec_port_path("linkback:auto") == ""
    assert spec_port_path("linkback:3-2.1") == "3-2.1"


def test_assembler_emits_complete_frames():
    a = FrameAssembler(frame_bytes=1200)
    f1 = bytes(range(256)) * 4 + bytes(176)
    f2 = bytes(reversed(range(256))) * 4 + bytes(176)
    out = a.feed(_packets(f1, 0)) + a.feed(_packets(f2, 1))  # a short last packet ends each transfer
    assert out == [f1, f2]
    assert (a.frames_ok, a.frames_dropped) == (2, 0)


def test_assembler_handles_data_split_across_transfers():
    a = FrameAssembler(frame_bytes=1200)
    f1 = bytes([7]) * 1200
    raw = _packets(f1, 0)
    got = []
    for i in range(0, len(raw), 512):  # one packet per transfer
        got += a.feed(raw[i : i + 512])
    assert got == [f1]


def test_assembler_ends_frame_on_id_toggle_without_eof():
    a = FrameAssembler(frame_bytes=1200)
    f1, f2 = bytes([1]) * 1200, bytes([2]) * 1200
    out = a.feed(_packets(f1, 0, eof=False)) + a.feed(_packets(f2, 1, eof=False))
    assert out == [f1]  # second frame is not finished until the next toggle


def test_assembler_drops_short_and_corrupt_frames():
    a = FrameAssembler(frame_bytes=1200)
    good = bytes([9]) * 1200
    short = bytes([3]) * 700
    corrupt = _packets(bytes([4]) * 1200, 0)
    corrupt = corrupt[:512] + bytes([0x00, 0x00]) + corrupt[514:]  # header without EOH bit
    out = []
    for transfer in (_packets(short, 0), _packets(good, 1), corrupt, _packets(good, 1)):
        out += a.feed(transfer)
    assert out == [good, good]
    assert a.frames_dropped == 2


def test_yuy2_to_bgr_neutral_gray():
    raw = np.empty((FRAME_H, FRAME_W, 2), dtype=np.uint8)
    raw[..., 0] = 128
    raw[..., 1] = 128
    bgr = yuy2_to_bgr(raw.tobytes())
    assert bgr.shape == (FRAME_H, FRAME_W, 3)
    assert abs(int(bgr[10, 10, 0]) - int(bgr[10, 10, 2])) <= 2  # no colour cast


def _dark_raw(y: int = 20) -> np.ndarray:
    raw = np.full((FRAME_H, FRAME_W, 2), 128, dtype=np.uint8)
    raw[..., 0] = y
    return raw


def test_auto_exposure_brightens_dark_frames():
    ae = AutoExposure({"enabled": True, "target_luma": 100, "min_gamma": 0.4, "adapt": 1.0})
    out = ae.apply(_dark_raw(20))
    assert out[..., 0].mean() > 60
    assert (out[..., 1] == 128).all()  # chroma untouched


def test_auto_exposure_never_clips_highlights_or_touches_bright_frames():
    ae = AutoExposure({"enabled": True, "target_luma": 100, "adapt": 1.0})
    mixed = _dark_raw(20)
    mixed[:50, :50, 0] = 255
    assert (ae.apply(mixed)[:50, :50, 0] == 255).all()  # highlights stay at white, never exceed it
    assert (AutoExposure({"enabled": True, "target_luma": 100, "adapt": 1.0}).apply(_dark_raw(200))[..., 0] == 200).all()


def test_auto_exposure_is_limited_by_min_gamma_and_can_be_disabled():
    weak = AutoExposure({"enabled": True, "target_luma": 200, "min_gamma": 0.9, "adapt": 1.0})
    assert weak.apply(_dark_raw(10))[..., 0].max() < 30
    off = AutoExposure({"enabled": False})
    assert (off.apply(_dark_raw(20))[..., 0] == 20).all()


def test_make_camera_picks_the_right_class():
    assert isinstance(make_camera("linkback", 640, 480), LinkbackCamera)
    assert isinstance(make_camera("linkback:3-1", 640, 480), LinkbackCamera)
    assert isinstance(make_camera("/dev/video0", 640, 480), Camera)


def test_linkback_specs_bypass_v4l_resolution():
    assert camera_mod.assign_camera_device("linkback:3-1") == ("linkback:3-1", None)
    assert camera_mod.resolve_fov_capture_size("linkback", 0, 0) == (FRAME_W, FRAME_H)


def test_camera_reports_native_size_whatever_is_requested():
    cam = LinkbackCamera("linkback", width=640, height=480)
    assert cam.frame_size() == (FRAME_W, FRAME_H)
    assert not cam.is_open()


def test_open_failure_raises_camera_error(monkeypatch):
    def fake_run(self):
        self._fail("no LinkBack camera found (USB 3456:4321)")

    monkeypatch.setattr(LinkbackCamera, "_run", fake_run)
    with pytest.raises(CameraError, match="no LinkBack camera"):
        LinkbackCamera("linkback").open()


def test_frames_flow_through_grab_and_retrieve(monkeypatch):
    frame = bytes([100, 128]) * (FRAME_BYTES // 2)

    def fake_run(self):
        self._ready.set()
        assembler = FrameAssembler()
        self._publish(assembler, _packets(frame, 0))
        self._stop.wait(5)

    monkeypatch.setattr(LinkbackCamera, "_run", fake_run)
    cam = LinkbackCamera("linkback")
    cam.open()
    try:
        assert cam.grab()
        ok, bgr = cam.retrieve()
        assert ok and bgr.shape == (FRAME_H, FRAME_W, 3)
        assert not cam.grab()  # nothing newer than what was just retrieved
    finally:
        cam.release()
