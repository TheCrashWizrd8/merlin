"""Unit tests for USB capture-node assignment."""

import os
import time
import unittest
from unittest.mock import patch

from src.camera import (
    assign_camera_device,
    probe_max_mjpeg_size,
    resolve_camera_device,
    resolve_fov_capture_size,
    usb_capture_nodes,
    _parse_v4l2_mjpg_sizes,
)


_CAMS = [
    ("HD Camera A", ["/dev/video2", "/dev/video3"]),
    ("HD Camera B", ["/dev/video1", "/dev/video4"]),
]


class CameraMaxResolutionTests(unittest.TestCase):
    _V4L2_SAMPLE = """
[0]: 'MJPG' (Motion-JPEG, compressed)
\tSize: Discrete 640x480
\tSize: Discrete 1280x720
\tSize: Discrete 1280x1024
[1]: 'YUYV' (YUYV 4:2:2)
\tSize: Discrete 640x480
"""

    def test_parse_v4l2_mjpg_sizes(self):
        sizes = _parse_v4l2_mjpg_sizes(self._V4L2_SAMPLE)
        self.assertEqual(sizes, [(640, 480), (1280, 720), (1280, 1024)])

    def test_resolve_fov_capture_size_auto(self):
        with patch(
            "src.camera.probe_max_mjpeg_size",
            return_value=(1280, 1024),
        ):
            w, h = resolve_fov_capture_size("/dev/video3", 0, 0)
            self.assertEqual((w, h), (1280, 1024))

    def test_resolve_fov_capture_size_explicit(self):
        w, h = resolve_fov_capture_size("/dev/video3", 640, 480)
        self.assertEqual((w, h), (640, 480))

    def test_probe_max_mjpeg_size_from_v4l2(self):
        proc = type("P", (), {"returncode": 0, "stdout": self._V4L2_SAMPLE})()
        with patch("src.camera.subprocess.run", return_value=proc):
            with patch(
                "src.camera.resolve_camera_device",
                return_value=("/dev/video3", None),
            ):
                self.assertEqual(probe_max_mjpeg_size("/dev/video3"), (1280, 1024))


class CameraStablePathTests(unittest.TestCase):
    def test_resolve_by_path_when_present(self):
        stable = "/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1:1.0-video-index0"
        present = {stable, "/dev/video0"}
        with patch("src.camera.os.path.exists", side_effect=lambda p: p in present):
            path, note = resolve_camera_device(stable)
            self.assertEqual(path, stable)
            self.assertIsNone(note)

    def test_assign_uses_by_path_over_fallback(self):
        stable_l = "/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:2:1.0-video-index0"
        stable_r = "/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1:1.0-video-index0"
        present = {stable_l, stable_r, "/dev/video0", "/dev/video2"}
        with (
            patch("src.camera.list_usb_cameras", return_value=_CAMS),
            patch("src.camera.os.path.exists", side_effect=lambda p: p in present),
            patch(
                "src.camera.resolve_camera_device",
                side_effect=lambda d: (
                    (stable_l, None) if "2:1.0" in str(d) else (stable_r, None)
                ),
            ),
        ):
            left, _ = assign_camera_device(stable_l)
            right, _ = assign_camera_device(stable_r, exclude=(left,))
            self.assertEqual(left, stable_l)
            self.assertEqual(right, stable_r)


class CameraAssignTests(unittest.TestCase):
    def test_usb_capture_nodes_first_of_each_sorted(self):
        with patch("src.camera.list_usb_cameras", return_value=_CAMS):
            self.assertEqual(usb_capture_nodes(), ["/dev/video1", "/dev/video2"])

    def test_assign_missing_zero_uses_lowest_capture(self):
        present = {"/dev/video1", "/dev/video2", "/dev/video3", "/dev/video4"}
        with (
            patch("src.camera.list_usb_cameras", return_value=_CAMS),
            patch("src.camera.os.path.exists", side_effect=lambda p: p in present),
        ):
            left, note = assign_camera_device(0)
            self.assertEqual(left, "/dev/video1")
            self.assertIsNotNone(note)
            self.assertIn("missing", note)

            right, note_r = assign_camera_device(2, exclude=(left,))
            self.assertEqual(right, "/dev/video2")
            self.assertIsNone(note_r)

    def test_assign_does_not_steal_excluded_fov_node(self):
        present = {"/dev/video0"}
        missing = "/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0"
        with (
            patch(
                "src.camera.list_usb_cameras",
                return_value=[("USB 2.0 Camera", ["/dev/video0", "/dev/video1"])],
            ),
            patch("src.camera.os.path.exists", side_effect=lambda p: p in present),
            patch("src.camera.os.path.realpath", side_effect=lambda p: p),
        ):
            left, _ = assign_camera_device(missing, exclude=("/dev/video0",))
            self.assertNotEqual(left, "/dev/video0")

    def test_assign_fallback_does_not_return_excluded_realpath(self):
        present = {"/dev/video0"}
        stable = "/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:2:1.0-video-index0"
        with (
            patch(
                "src.camera.list_usb_cameras",
                return_value=[("UVC Camera", ["/dev/video0", "/dev/video1"])],
            ),
            patch("src.camera.os.path.exists", side_effect=lambda p: p in present or p == stable),
            patch(
                "src.camera.os.path.realpath",
                side_effect=lambda p: "/dev/video0" if "by-path" in str(p) or p == "/dev/video0" else p,
            ),
            patch("src.camera.resolve_camera_device", return_value=(stable, None)),
        ):
            left, _ = assign_camera_device(0, exclude=(stable,))
            self.assertNotEqual(os.path.realpath(str(left)), "/dev/video0")

    def test_assign_skips_metadata_node(self):
        cams = [
            ("HD Camera A", ["/dev/video0", "/dev/video1"]),
            ("HD Camera B", ["/dev/video3", "/dev/video4"]),
        ]
        present = {"/dev/video0", "/dev/video1", "/dev/video3", "/dev/video4"}
        with (
            patch("src.camera.list_usb_cameras", return_value=cams),
            patch("src.camera.os.path.exists", side_effect=lambda p: p in present),
        ):
            left, note = assign_camera_device(2)
            self.assertEqual(left, "/dev/video0")
            self.assertIsNotNone(note)
            right, note_r = assign_camera_device(1, exclude=(left,))
            self.assertEqual(right, "/dev/video3")
            self.assertIsNotNone(note_r)
            self.assertIn("using /dev/video3", note_r)

    def test_assign_keeps_existing_indexes(self):
        present = {"/dev/video1", "/dev/video2"}
        with (
            patch("src.camera.list_usb_cameras", return_value=_CAMS),
            patch("src.camera.os.path.exists", side_effect=lambda p: p in present),
        ):
            left, note = assign_camera_device(1)
            right, note_r = assign_camera_device(2, exclude=(left,))
            self.assertEqual(left, "/dev/video1")
            self.assertEqual(right, "/dev/video2")
            self.assertIsNone(note)
            self.assertIsNone(note_r)


class _FakeCam:
    def __init__(self) -> None:
        self.device = "/dev/video0"
        self.opened = True
        self.fail = False
        self.opens = 0

    def is_open(self) -> bool:
        return self.opened

    def grab(self) -> bool:
        return self.opened and not self.fail

    def retrieve(self):
        if self.fail or not self.opened:
            return False, None
        import numpy as np

        return True, np.zeros((8, 8, 3), dtype=np.uint8)

    def release(self) -> None:
        self.opened = False

    def open(self) -> None:
        self.opens += 1
        self.opened = True
        self.fail = False


class FrameGrabberReconnectTests(unittest.TestCase):
    def test_stale_clears_frame_and_reopens(self):
        from src.camera import FrameGrabber

        cam = _FakeCam()
        grabber = FrameGrabber(cam, name="left", stale_s=0.25)
        grabber.start()
        try:
            deadline = time.monotonic() + 2.0
            ok = False
            while time.monotonic() < deadline:
                ok, frame = grabber.peek()
                if ok and frame is not None:
                    break
                time.sleep(0.02)
            self.assertTrue(ok)
            cam.fail = True
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                ok, _ = grabber.peek()
                if not ok:
                    break
                time.sleep(0.02)
            self.assertFalse(ok)
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                if cam.opens >= 1 and grabber.peek()[0]:
                    break
                time.sleep(0.05)
            self.assertGreaterEqual(cam.opens, 1)
            ok, frame = grabber.peek()
            self.assertTrue(ok)
            self.assertIsNotNone(frame)
        finally:
            grabber.stop()

    def test_mark_down_does_not_keep_stale_frame(self):
        from src.camera import FrameGrabber

        cam = _FakeCam()
        grabber = FrameGrabber(cam, name="left", stale_s=5.0)
        grabber.start()
        try:
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                if grabber.peek()[0]:
                    break
                time.sleep(0.02)
            self.assertTrue(grabber.peek()[0])
        finally:
            grabber.stop()
        cam.fail = True
        grabber._mark_down("unplugged")
        ok, frame = grabber.peek()
        self.assertFalse(ok)
        self.assertIsNone(frame)


if __name__ == "__main__":
    unittest.main()
