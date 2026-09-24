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

UVC cameras usually expose two nodes each (capture + metadata). Indexes
shuffle across boots — there may be no /dev/video0. Prefer /dev/videoN
paths; missing indexes are remapped to the first unused USB capture node.
"""

from __future__ import annotations

import os
import re
import subprocess
import threading
import time
from typing import Callable, Generator, List, Optional, Sequence, Tuple

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
import cv2
import numpy as np

try:
    cv2.setLogLevel(cv2.LOG_LEVEL_ERROR)
except Exception:
    try:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
    except Exception:
        pass


def _v4l_device_path(device: int | str) -> Optional[str]:
    if isinstance(device, int) or (isinstance(device, str) and device.isdigit()):
        return f"/dev/video{int(device)}"
    if isinstance(device, str) and device.startswith("/dev/video"):
        return device
    return None


_V4L_BY_PATH = "/dev/v4l/by-path"
_V4L_BY_ID = "/dev/v4l/by-id"


def _is_usb_by_path_name(name: str) -> bool:
    return "usb-" in name and "video-index" in name


def _capture_symlink_preferred(link: str) -> bool:
    """Prefer index0 (capture) over index1 (metadata)."""
    return link.endswith("video-index0")


def list_stable_usb_paths() -> List[dict]:
    """
    USB cameras by stable ``/dev/v4l/by-path/...`` symlinks (plug-order safe).

    Returns dicts with keys: by_path, video, name.
    """
    root = _V4L_BY_PATH
    if not os.path.isdir(root):
        return []
    seen_video: set[str] = set()
    out: List[dict] = []
    for entry in sorted(os.listdir(root)):
        if not _is_usb_by_path_name(entry):
            continue
        if not _capture_symlink_preferred(entry):
            continue
        by_path = os.path.join(root, entry)
        try:
            video = os.path.realpath(by_path)
        except OSError:
            continue
        if video in seen_video:
            continue
        seen_video.add(video)
        out.append({"by_path": by_path, "video": video, "name": entry})
    return out


def resolve_camera_device(device: int | str) -> tuple[str, Optional[str]]:
    """
    Map hardware.yaml value to an openable device path.

    Prefer ``/dev/v4l/by-path/...`` (USB physical port) or ``/dev/v4l/by-id/...``.
    Falls back to ``/dev/videoN`` or numeric index.
    """
    if device is None:
        return "", None
    text = str(device).strip()
    if not text:
        return "", None

    if text.startswith("/dev/v4l/by-path/") or text.startswith("/dev/v4l/by-id/"):
        if os.path.exists(text):
            return text, None
        return text, f"stable camera path missing: {text}"

    if "platform-" in text or text.startswith("usb-"):
        matches = [
            p["by_path"]
            for p in list_stable_usb_paths()
            if text in p["name"] or text in p["by_path"]
        ]
        if matches:
            return matches[0], None
        return text, f"no /dev/v4l/by-path match for '{text}'"

    requested = _v4l_device_path(device)
    if requested and os.path.exists(requested):
        return requested, None
    if requested:
        return requested, None
    return text, None


def _parse_v4l2_mjpg_sizes(text: str) -> List[Tuple[int, int]]:
    """Parse ``v4l2-ctl --list-formats-ext`` MJPEG size lines."""
    sizes: List[Tuple[int, int]] = []
    in_mjpg = False
    for line in text.splitlines():
        stripped = line.strip()
        fmt = re.match(r"\[\d+\]:\s*'(\w+)'", stripped)
        if fmt:
            in_mjpg = fmt.group(1) == "MJPG"
            continue
        if in_mjpg:
            match = re.search(r"Size:\s*Discrete\s+(\d+)x(\d+)", stripped)
            if match:
                sizes.append((int(match.group(1)), int(match.group(2))))
    return sizes


def probe_max_mjpeg_size(
    device: int | str,
    *,
    fallback: Tuple[int, int] = (1280, 720),
) -> Tuple[int, int]:
    """Largest MJPEG mode reported by v4l2-ctl for this device."""
    path, _ = resolve_camera_device(device)
    if not path:
        return fallback
    try:
        proc = subprocess.run(
            ["v4l2-ctl", "-d", path, "--list-formats-ext"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return fallback
    if proc.returncode != 0 or not proc.stdout.strip():
        return fallback
    sizes = _parse_v4l2_mjpg_sizes(proc.stdout)
    if not sizes:
        return fallback
    return max(sizes, key=lambda wh: wh[0] * wh[1])


def resolve_fov_capture_size(
    device: int | str,
    width: int,
    height: int,
) -> Tuple[int, int]:
    """
    Pick FOV capture resolution.

    ``width`` or ``height`` <= 0 selects the largest MJPEG mode from v4l2.
    """
    if width > 0 and height > 0:
        return width, height
    max_w, max_h = probe_max_mjpeg_size(device)
    return (width if width > 0 else max_w, height if height > 0 else max_h)


def _device_excluded(path: str, exclude: Sequence[str]) -> bool:
    try:
        real = os.path.realpath(path)
    except OSError:
        real = path
    for item in exclude:
        if not item:
            continue
        if path == item:
            return True
        try:
            if os.path.realpath(item) == real:
                return True
        except OSError:
            continue
    return False


def force_v4l2_mjpg(device: int | str, width: int, height: int, fps: int = 30) -> None:
    """Ask the kernel for MJPEG at `width`x`height` before OpenCV opens the device."""
    path, _ = resolve_camera_device(device)
    if not path:
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
    path, _ = resolve_camera_device(device)
    if not path:
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


def usb_capture_nodes() -> List[str]:
    """First ``/dev/videoN`` of each USB camera, sorted by index (capture, not metadata)."""
    nodes: List[str] = []
    for _name, paths in list_usb_cameras():
        for path in paths:
            if path.startswith("/dev/video"):
                nodes.append(path)
                break

    def _idx(path: str) -> int:
        match = re.search(r"(\d+)$", path)
        return int(match.group(1)) if match else 0

    return sorted(dict.fromkeys(nodes), key=_idx)


def assign_camera_device(
    device: int | str,
    exclude: Sequence[str] = (),
) -> tuple[str, Optional[str]]:
    """
    Map a configured stable path or index to an existing capture node.

    Returns ``(path, warning_or_None)``. Prefers ``/dev/v4l/by-path/...``
    from config; falls back to the first unused USB capture node.
    """
    capture = usb_capture_nodes()
    resolved, resolve_note = resolve_camera_device(device)

    if resolved and os.path.exists(resolved) and not _device_excluded(resolved, exclude):
        try:
            real = os.path.realpath(resolved)
        except OSError:
            real = resolved
        if real in capture or resolved.startswith("/dev/v4l/"):
            note = resolve_note
            return resolved, note

    for node in capture:
        if not _device_excluded(node, exclude) and os.path.exists(node):
            if resolved and resolved != node:
                why = "missing" if not os.path.exists(resolved) else "not available"
                return node, f"device {device} ({resolved}) is {why}; using {node}"
            return node, None

    fallback = resolved or _v4l_device_path(device) or str(device)
    if fallback and _device_excluded(str(fallback), exclude):
        return str(device), resolve_note
    return fallback, resolve_note


def format_usb_camera_report() -> str:
    """Human-readable USB camera list for logs / errors."""
    cams = list_usb_cameras()
    stable = list_stable_usb_paths()
    if not cams and not stable:
        return (
            "No USB cameras found by v4l2-ctl "
            "(install v4l-utils, or the second camera is not enumerating)."
        )
    lines = [f"USB cameras seen by Linux ({len(cams)}):"]
    for name, nodes in cams:
        lines.append(f"  - {name}")
        for node in nodes:
            lines.append(f"      {node}")
    if stable:
        lines.append("")
        lines.append(
            f"Stable by-path capture nodes ({len(stable)}) — use these in hardware.yaml:"
        )
        for row in stable:
            lines.append(f"  {row['video']}  ←  {row['by_path']}")
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
        if self._cap is not None:
            self.release()
        path, note = resolve_camera_device(self.device)
        if note:
            print(f"[camera] {note}")
        self.device = path or str(self.device)
        force_v4l2_mjpg(self.device, self.width, self.height, self.fps)
        force_v4l2_low_latency(self.device)

        # Open by path. Integer 0 means /dev/video0, which often does not exist
        # even when cameras are on /dev/video1 and /dev/video2.
        self._cap = cv2.VideoCapture(self.device, self.backend)

        if not self._cap.isOpened():
            self._cap = cv2.VideoCapture(self.device)

        if not self._cap.isOpened():
            nodes = usb_capture_nodes()
            hint = (
                f" Capture nodes: {', '.join(nodes)}."
                if nodes
                else " No USB capture nodes found."
            )
            raise CameraError(
                f"Cannot open camera device '{self.device}'.{hint} "
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

    def reopen(self) -> None:
        """Release and open again (USB unplug / V4L node recycle)."""
        self.release()
        self.open()

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
        if self._cap is None or not self._cap.isOpened():
            return False
        return bool(self._cap.grab())

    def retrieve(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Decode the last grabbed buffer into a BGR frame."""
        if self._cap is None or not self._cap.isOpened():
            return False, None
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


def camera_offline_frame(
    width: int = 640,
    height: int = 480,
    title: str = "Camera offline",
) -> np.ndarray:
    """Placeholder BGR frame so the dashboard still updates when a cam is down."""
    img = np.full((max(1, height), max(1, width), 3), 36, dtype=np.uint8)
    y = max(40, height // 2 - 16)
    for i, line in enumerate(title.split("\n")):
        cv2.putText(
            img,
            line,
            (20, y + i * 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (80, 80, 220),
            2,
            cv2.LINE_AA,
        )
    return img


def publish_camera_status(
    side: str,
    *,
    connected: bool,
    device: str = "",
    status: str = "",
    error: Optional[str] = None,
) -> None:
    if not side:
        return
    try:
        from src.sub_state import get_sub_state

        get_sub_state().update_camera(
            side,
            connected=connected,
            device=device,
            status=status,
            error=error,
        )
    except Exception:
        pass


_STALE_S = 1.0
_RECONNECT_BACKOFF_S = (0.4, 1.0, 2.0, 5.0)


class FrameGrabber:
    """
    Always keep the newest decoded frame.

    If grab/retrieve stops (unplug, USB stall), the last frame is dropped,
    a dashboard alert is published, and the V4L device is reopened with
    backoff until it comes back.
    """

    def __init__(
        self,
        camera: Camera,
        *,
        name: str = "cam",
        exclude: Sequence[str] | Callable[[], Sequence[str]] = (),
        stale_s: float = _STALE_S,
    ) -> None:
        self._camera = camera
        self._name = name
        self._exclude = exclude
        self._stale_s = max(0.2, float(stale_s))
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._frame: Optional[np.ndarray] = None
        self._ok = False
        self._t_mono = 0.0
        self._running = False
        self._request = 0
        self._done = 0
        self._thread: Optional[threading.Thread] = None
        self._alive = False
        self._error: Optional[str] = None
        self._reopen_log_at = 0.0

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._loop,
            name=f"grab-{self._name}-{self._camera.device}",
            daemon=True,
        )
        self._thread.start()

    def _exclude_now(self) -> Sequence[str]:
        if callable(self._exclude):
            try:
                return tuple(self._exclude() or ())
            except Exception:
                return ()
        return self._exclude

    def _mark_down(self, reason: str) -> None:
        with self._cond:
            self._ok = False
            self._frame = None
            self._cond.notify_all()
        was_alive = self._alive
        self._alive = False
        self._error = reason
        publish_camera_status(
            self._name,
            connected=False,
            device=str(self._camera.device),
            status="reconnecting",
            error=reason,
        )
        if was_alive:
            print(
                f"[camera] {self._name} lost ({self._camera.device}): {reason} "
                "— reconnecting"
            )

    def _mark_live(self) -> None:
        was_alive = self._alive
        self._alive = True
        self._error = None
        publish_camera_status(
            self._name,
            connected=True,
            device=str(self._camera.device),
            status="live",
            error=None,
        )
        if not was_alive:
            print(f"[camera] {self._name} live on {self._camera.device}")

    def _remap_if_needed(self) -> None:
        path = _v4l_device_path(self._camera.device)
        if path and os.path.exists(path):
            return
        new, note = assign_camera_device(self._camera.device, exclude=self._exclude_now())
        if new and new != str(self._camera.device):
            print(f"[camera] {self._name} remapped {self._camera.device} → {new}"
                  + (f" ({note})" if note else ""))
            self._camera.device = new

    def _reconnect(self, backoff_s: float) -> None:
        try:
            self._camera.release()
        except Exception:
            pass
        deadline = time.monotonic() + backoff_s
        while self._running and time.monotonic() < deadline:
            time.sleep(0.05)
        if not self._running:
            return
        self._remap_if_needed()
        try:
            self._camera.open()
        except Exception as exc:
            self._error = str(exc)
            publish_camera_status(
                self._name,
                connected=False,
                device=str(self._camera.device),
                status="reconnecting",
                error=str(exc),
            )
            now = time.monotonic()
            if now - self._reopen_log_at >= 10.0:
                self._reopen_log_at = now
                print(f"[camera] {self._name} reopen failed ({self._camera.device}): {exc}")

    def _loop(self) -> None:
        started = time.monotonic()
        last_ok = 0.0
        backoff_i = 0
        while self._running:
            if not self._camera.is_open():
                if last_ok or (time.monotonic() - started) >= self._stale_s:
                    self._mark_down(self._error or "device closed")
                self._reconnect(_RECONNECT_BACKOFF_S[backoff_i])
                backoff_i = min(backoff_i + 1, len(_RECONNECT_BACKOFF_S) - 1)
                continue
            try:
                grabbed = self._camera.grab()
                if not grabbed:
                    now = time.monotonic()
                    age_ok = last_ok > 0
                    reference = last_ok if age_ok else started
                    if now - reference >= self._stale_s:
                        self._mark_down("no frames")
                        self._reconnect(_RECONNECT_BACKOFF_S[backoff_i])
                        backoff_i = min(backoff_i + 1, len(_RECONNECT_BACKOFF_S) - 1)
                    else:
                        time.sleep(0.01)
                    continue
                ok, frame = self._camera.retrieve()
            except Exception as exc:
                self._mark_down(str(exc) or "grab error")
                self._reconnect(_RECONNECT_BACKOFF_S[backoff_i])
                backoff_i = min(backoff_i + 1, len(_RECONNECT_BACKOFF_S) - 1)
                continue
            copied = frame.copy() if ok and frame is not None else None
            if copied is None:
                time.sleep(0.01)
                continue
            last_ok = time.monotonic()
            backoff_i = 0
            with self._cond:
                self._ok = True
                self._frame = copied
                self._t_mono = last_ok
                if self._request > self._done:
                    self._done = self._request
                self._cond.notify_all()
            if not self._alive:
                self._mark_live()

    def peek(self, copy: bool = True) -> Tuple[bool, Optional[np.ndarray]]:
        """Newest decoded frame. Never blocks.

        Returns False once the camera is stale — the last good frame is
        not reused. copy=False is safe for YOLO: the grabber replaces the
        array on the next decode instead of writing in place.
        """
        with self._lock:
            ok = self._ok
            frame = self._frame
        if not ok or frame is None:
            return False, None
        return True, frame.copy() if copy else frame

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
        """Non-blocking: last decoded frame. Prefer peek()."""
        return self.peek()

    @property
    def capture_mono(self) -> float:
        with self._lock:
            return self._t_mono

    @property
    def alive(self) -> bool:
        return self._alive

    def stop(self) -> None:
        self._running = False
        with self._cond:
            self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
