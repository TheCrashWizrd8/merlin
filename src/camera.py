"""
camera.py
---------
USB camera capture using OpenCV VideoCapture.

Wraps the camera in a context manager so it is always properly released:

    with Camera() as cam:
        for frame in cam:
            process(frame)

Or without context manager:

    cam = Camera(device=0, width=640, height=480)
    cam.open()
    ret, frame = cam.read()
    cam.release()

The Pi5 typically exposes the USB camera as /dev/video0 → device index 0.
If you have multiple cameras plugged in, pass the correct index or
/dev/videoN path to the constructor.
"""

from __future__ import annotations

import time
from typing import Generator, Optional, Tuple

import cv2
import numpy as np


class CameraError(RuntimeError):
    """Raised when the camera cannot be opened or a frame cannot be read."""


class Camera:
    """
    USB camera capture wrapper.

    Parameters
    ----------
    device : int | str
        OpenCV device index (0, 1, …) or a V4L2 device path ("/dev/video0").
    width : int
        Requested capture width in pixels.
    height : int
        Requested capture height in pixels.
    fps : int
        Requested frames per second.  The camera may not honour this exactly.
    backend : int
        OpenCV capture backend flag.  Defaults to cv2.CAP_V4L2 on Linux for
        lower latency.  Pass cv2.CAP_ANY to let OpenCV choose automatically.
    """

    def __init__(
        self,
        device: int | str = 0,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        backend: int = cv2.CAP_V4L2,
    ) -> None:
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.backend = backend
        self._cap: Optional[cv2.VideoCapture] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self) -> None:
        """Open the camera.  Raises CameraError if it cannot be opened."""
        self._cap = cv2.VideoCapture(self.device, self.backend)

        if not self._cap.isOpened():
            self._cap = cv2.VideoCapture(self.device)

        if not self._cap.isOpened():
            raise CameraError(
                f"Cannot open camera device '{self.device}'. "
                "Check that a USB camera is connected and try a different device index."
            )

        # Request MJPEG — needed for cameras like the DFRobot FIT0819 endoscope.
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)

        # Minimise internal buffer to reduce latency on Pi
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self._cap.get(cv2.CAP_PROP_FPS)
        print(
            f"[Camera] Opened device={self.device}  "
            f"resolution={actual_w}x{actual_h}  fps={actual_fps:.1f}"
        )

    def release(self) -> None:
        """Release the camera resource."""
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def is_open(self) -> bool:
        return self._cap is not None and self._cap.isOpened()

    # ------------------------------------------------------------------
    # Frame reading
    # ------------------------------------------------------------------

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Read a single frame.

        Returns
        -------
        (success, frame)
            success : bool  — False if the camera failed to deliver a frame.
            frame   : BGR ndarray or None on failure.
        """
        if self._cap is None:
            raise CameraError("Camera is not open. Call open() first.")
        return self._cap.read()

    def frame_size(self) -> Tuple[int, int]:
        """Return (width, height) of the actual capture resolution."""
        if self._cap is None:
            return self.width, self.height
        return (
            int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        )

    # ------------------------------------------------------------------
    # Generator interface
    # ------------------------------------------------------------------

    def frames(self) -> Generator[np.ndarray, None, None]:
        """
        Yield frames indefinitely until the camera fails.

        Usage
        -----
        for frame in cam.frames():
            ...
        """
        while self.is_open():
            ok, frame = self.read()
            if not ok or frame is None:
                break
            yield frame

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "Camera":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.release()
