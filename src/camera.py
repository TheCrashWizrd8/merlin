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

import re
import subprocess
import threading
import time
from typing import Generator, List, Optional, Tuple

import cv2
import numpy as np


def _v4l_device_path(device: int | str) -> Optional[str]:
    if isinstance(device, int) or (isinstance(device, str) and device.isdigit()):
        return f"/dev/video{int(device)}"
    if isinstance(device, str) and device.startswith("/dev/video"):
        return device
    return None


def force_v4l2_mjpg(device: int | str, width: int, height: int, fps: int = 30) -> None:
    """Ask the kernel for MJPEG at `width`x`height` before OpenCV opens the device."""
    path = _v4l_device_path(device)
    if path is None:
        return
    try:
        subprocess.run(
            [
                "v4l2-ctl",
                "-d", path,
                f"--set-fmt-video=width={width},height={height},pixelformat=MJPG",
            ],
            capture_output=True,
            text=True,
            timeout=3,
        )
        subprocess.run(
            ["v4l2-ctl", "-d", path, f"--set-parm={fps}"],
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return


def force_v4l2_low_latency(device: int | str) -> None:
    """Keep auto-exposure for brightness, but pin the frame rate.

    Aperture-priority + ``exposure_dynamic_framerate`` lets the shutter
    stretch and FPS collapse in the dark (looks like ~1 s lag).  Manual
    16 ms exposure was too dark indoors.
    """
    path = _v4l_device_path(device)
    if path is None:
        return
    try:
        subprocess.run(
            ["v4l2-ctl", "-d", path, "--set-ctrl=exposure_dynamic_framerate=0"],
            capture_output=True,
            text=True,
            timeout=3,
        )
        subprocess.run(
            ["v4l2-ctl", "-d", path, "--set-ctrl=auto_exposure=3"],
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return


# Pi ISP / decoder nodes are not USB cameras.
_PLATFORM_CAMERA_RE = re.compile(
    r"pispbe|rpi-hevc|bcm2835|unicam|rpivid",
    re.IGNORECASE,
)


def list_usb_cameras() -> List[tuple[str, List[str]]]:
    """
    Return USB (or other non-platform) V4L2 cameras as
    ``[(name, ["/dev/video0", ...]), ...]``.

    Uses ``v4l2-ctl --list-devices`` when available.  Platform ISP/HEVC
    nodes on the Pi are skipped so you only see real USB cameras.
    """
    try:
        proc = subprocess.run(
            ["v4l2-ctl", "--list-devices"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []
    if proc.returncode != 0 or not proc.stdout.strip():
        return []

    cameras: List[tuple[str, List[str]]] = []
    name = ""
    nodes: List[str] = []
    skip = False

    def _flush() -> None:
        if name and nodes and not skip:
            cameras.append((name, nodes[:]))

    for raw in proc.stdout.splitlines():
        line = raw.rstrip()
        if not line:
            _flush()
            name, nodes, skip = "", [], False
            continue
        if not line.startswith("\t") and line.endswith(":"):
            _flush()
            name = line[:-1].strip()
            nodes = []
            skip = bool(_PLATFORM_CAMERA_RE.search(name))
            continue
        path = line.strip()
        if path.startswith("/dev/video"):
            nodes.append(path)
    _flush()
    return cameras


def format_usb_camera_report() -> str:
    """Human-readable USB camera list for logs / errors."""
    cams = list_usb_cameras()
    if not cams:
        return (
            "No USB cameras found by v4l2-ctl "
            "(install v4l-utils, or the second camera is not enumerating)."
        )
    lines = [f"USB cameras seen by Linux ({len(cams)}):"]
    for name, nodes in cams:
        lines.append(f"  - {name}")
        for node in nodes:
            lines.append(f"      {node}")
    if len(cams) < 2:
        lines.append(
            "  Only one USB camera enumerated — the other is unplugged, "
            "unpowered, or not talking to USB (often a dead cable/camera)."
        )
    return "\n".join(lines)


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
        self.native_size: Tuple[int, int] = (width, height)
        self._resize = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self) -> None:
        """Open the camera.  Raises CameraError if it cannot be opened."""
        force_v4l2_mjpg(self.device, self.width, self.height, self.fps)
        force_v4l2_low_latency(self.device)

        self._cap = cv2.VideoCapture(self.device, self.backend)

        if not self._cap.isOpened():
            self._cap = cv2.VideoCapture(self.device)

        if not self._cap.isOpened():
            raise CameraError(
                f"Cannot open camera device '{self.device}'. "
                "Check that a USB camera is connected and try a different device index."
            )

        # Request MJPEG then size.  Many UVC cameras ignore CAP_PROP until
        # FOURCC is set; some still stay at native res (e.g. 1600×1200).
        fourcc_mjpg = cv2.VideoWriter_fourcc(*"MJPG")
        self._cap.set(cv2.CAP_PROP_FOURCC, fourcc_mjpg)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._cap.set(cv2.CAP_PROP_FOURCC, fourcc_mjpg)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        self._probe_native_size()
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        force_v4l2_low_latency(self.device)
        actual_w, actual_h = self.native_size
        actual_fps = self._cap.get(cv2.CAP_PROP_FPS)
        fourcc_int = int(self._cap.get(cv2.CAP_PROP_FOURCC))
        fourcc = "".join(
            chr((fourcc_int >> (8 * i)) & 0xFF) for i in range(4)
        ).strip("\x00")
        if self._resize:
            print(
                f"[Camera] Opened device={self.device}  native={actual_w}x{actual_h}  "
                f"forcing={self.width}x{self.height}  fps={actual_fps:.1f}  fourcc={fourcc or '?'}"
            )
            print(
                "[Camera] WARN: camera is still sending a large frame — JPEG decode "
                "will add latency. Check: v4l2-ctl -d /dev/video0 --get-fmt-video"
            )
        else:
            print(
                f"[Camera] Opened device={self.device}  "
                f"resolution={actual_w}x{actual_h}  fps={actual_fps:.1f}  fourcc={fourcc or '?'}"
            )
        if fourcc and fourcc != "MJPG":
            print(
                "[Camera] WARN: not MJPEG — USB decode may be slow. "
                "Try: v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=MJPG"
            )

    def _probe_native_size(self) -> None:
        """Read one frame so we know the real capture size (CAP_PROP often lies)."""
        if self._cap is None:
            return
        ok, frame = self._cap.read()
        if ok and frame is not None:
            h, w = frame.shape[:2]
            self.native_size = (w, h)
        else:
            self.native_size = (
                int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or self.width,
                int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or self.height,
            )
        self._resize = (
            self.native_size[0] != self.width or self.native_size[1] != self.height
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
        ok, frame = self._cap.read()
        return self._maybe_resize(ok, frame)

    def grab(self) -> bool:
        """Dequeue a camera buffer without JPEG-decoding it (cheap drain)."""
        if self._cap is None:
            raise CameraError("Camera is not open. Call open() first.")
        return bool(self._cap.grab())

    def retrieve(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Decode the last grabbed buffer into a BGR frame."""
        if self._cap is None:
            raise CameraError("Camera is not open. Call open() first.")
        ok, frame = self._cap.retrieve()
        return self._maybe_resize(ok, frame)

    def _maybe_resize(
        self, ok: bool, frame: Optional[np.ndarray]
    ) -> Tuple[bool, Optional[np.ndarray]]:
        if not ok or frame is None:
            return ok, frame
        if self._resize:
            frame = cv2.resize(
                frame,
                (self.width, self.height),
                interpolation=cv2.INTER_AREA,
            )
        return True, frame

    def frame_size(self) -> Tuple[int, int]:
        """Return (width, height) delivered to callers (working size)."""
        return self.width, self.height

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


class FrameGrabber:
    """
    Always keep the newest decoded frame.

    ``grab()`` + ``retrieve()`` run continuously so the UVC FIFO cannot
    fill with stale MJPEG.  Inference and the web preview both ``peek()``
    that latest copy — they do not wait for YOLO.
    """

    def __init__(self, camera: Camera) -> None:
        self._camera = camera
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._frame: Optional[np.ndarray] = None
        self._ok = False
        self._t_mono = 0.0
        self._running = False
        self._request = 0
        self._done = 0
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._loop,
            name=f"grab-{self._camera.device}",
            daemon=True,
        )
        self._thread.start()

    def _loop(self) -> None:
        while self._running:
            try:
                if not self._camera.grab():
                    time.sleep(0.002)
                    continue
                ok, frame = self._camera.retrieve()
            except Exception:
                time.sleep(0.002)
                continue
            copied = frame.copy() if ok and frame is not None else None
            with self._cond:
                self._ok = copied is not None
                self._frame = copied
                self._t_mono = time.monotonic()
                if self._request > self._done:
                    self._done = self._request
                self._cond.notify_all()

    def peek(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Newest decoded frame (copied). Never blocks."""
        with self._lock:
            if not self._ok or self._frame is None:
                return False, None
            return True, self._frame.copy()

    def request(self) -> int:
        """Ask for a new frame without blocking."""
        with self._cond:
            self._request += 1
            want = self._request
            self._cond.notify()
            return want

    def collect(self, want: int, timeout: float = 0.4) -> Tuple[bool, Optional[np.ndarray]]:
        """Wait until request `want` has been decoded (or a newer peek exists)."""
        with self._cond:
            if not self._cond.wait_for(
                lambda: self._done >= want or self._frame is not None,
                timeout=timeout,
            ):
                return False, None
            if not self._ok or self._frame is None:
                return False, None
            return True, self._frame.copy()

    def wait_latest(self, timeout: float = 0.4) -> Tuple[bool, Optional[np.ndarray]]:
        """Block until a newly decoded frame is ready (or timeout)."""
        return self.collect(self.request(), timeout=timeout)

    def latest(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Non-blocking: last decoded frame (may be stale). Prefer peek()."""
        return self.peek()

    @property
    def capture_mono(self) -> float:
        with self._lock:
            return self._t_mono

    def stop(self) -> None:
        self._running = False
        with self._cond:
            self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
