"""Unit tests for USB capture-node assignment."""

import unittest
from unittest.mock import patch

from src.camera import assign_camera_device, usb_capture_nodes


_CAMS = [
    ("HD Camera A", ["/dev/video2", "/dev/video3"]),
    ("HD Camera B", ["/dev/video1", "/dev/video4"]),
]


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
            self.assertIn("capture", note_r)

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


if __name__ == "__main__":
    unittest.main()
