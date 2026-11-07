"""
linkback_camera.py
------------------
Capture from the "lem01camera" USB camera (VID 3456, PID 4321, vendor app
"LinkBack"). Linux has no driver for it: it shows vendor-class interfaces
("iAP Interface", "com.linkback.protocol") instead of UVC, so no /dev/video
node appears.

How it streams (found by reverse engineering the Android app, no handshake):

* claim interfaces 0 and 1, select alternate setting 1 on interface 1;
* read bulk IN 0x82 — the camera starts sending at once;
* every 512-byte packet starts with a UVC-style payload header
  (byte 0 = header length, 0x0C; byte 1 = flags: bit0 frame-ID toggle,
  bit1 end-of-frame, bit7 end-of-header);
* the payload is raw YUY2, 450x450 (405000 bytes per frame), about 27 fps.

The stream is ~10 MB/s, so it must be read with several large queued bulk
transfers. Reading 512 bytes at a time loses data (horizontal band slips).

Both cameras of a pair share the same VID/PID/serial, so they are told apart
by USB port path ("3-1", "3-2.1", ...), like /dev/v4l/by-path.

Device specs accepted by ``is_linkback_spec``:

    linkback          first LinkBack camera not already in use
    linkback:3-1      the camera on USB port path 3-1

``LinkbackCamera`` mirrors the parts of ``camera.Camera`` that
``camera.FrameGrabber`` uses, so it drops in anywhere a Camera does.
"""

from __future__ import annotations

import glob
import os
import threading
import time
from pathlib import Path
from typing import Generator, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

_HARDWARE_PATH = Path(__file__).resolve().parent.parent / "config" / "hardware.yaml"

LINKBACK_VID = 0x3456
LINKBACK_PID = 0x4321
FRAME_W = 450
FRAME_H = 450
FRAME_BYTES = FRAME_W * FRAME_H * 2  # YUY2

_PACKET = 512
_EP_IN = 0x82
_INTERFACES = (0, 1)
_STREAM_IFACE = 1
_STREAM_ALT = 1
_QUEUED_TRANSFERS = 32
_TRANSFER_BYTES = 32768
_NEW_FRAME_WAIT_S = 0.1

_SPEC_PREFIX = "linkback"

_claim_lock = threading.Lock()
_claimed: dict[str, "LinkbackCamera"] = {}


def is_linkback_spec(device: object) -> bool:
    """True for ``linkback`` or ``linkback:<usb port path>`` device specs."""
    if not isinstance(device, str):
        return False
    text = device.strip().lower()
    return text == _SPEC_PREFIX or text.startswith(_SPEC_PREFIX + ":")


def spec_port_path(device: object) -> str:
    """Port path from a spec ("" = any camera)."""
    if not is_linkback_spec(device):
        return ""
    _, _, path = str(device).strip().partition(":")
    path = path.strip()
    return "" if path.lower() == "auto" else path


def list_linkback_devices() -> List[dict]:
    """LinkBack cameras on the bus, from sysfs (needs no USB permissions)."""
    found: List[dict] = []
    for vid_file in glob.glob("/sys/bus/usb/devices/*/idVendor"):
        base = os.path.dirname(vid_file)
        try:
            with open(vid_file) as f:
                vid = int(f.read().strip(), 16)
            with open(os.path.join(base, "idProduct")) as f:
                pid = int(f.read().strip(), 16)
        except (OSError, ValueError):
            continue
        if (vid, pid) != (LINKBACK_VID, LINKBACK_PID):
            continue
        path = os.path.basename(base)
        serial = ""
        try:
            with open(os.path.join(base, "serial")) as f:
                serial = f.read().strip()
        except OSError:
            pass
        found.append({"port_path": path, "serial": serial, "sysfs": base})
    return sorted(found, key=lambda d: d["port_path"])


def format_linkback_report() -> str:
    cams = list_linkback_devices()
    if not cams:
        return "No LinkBack (3456:4321) USB cameras found."
    lines = [f"LinkBack USB cameras ({len(cams)}):"]
    for cam in cams:
        lines.append(f"  linkback:{cam['port_path']}   (serial {cam['serial'] or '?'})")
    return "\n".join(lines)


class FrameAssembler:
    """Turn raw bulk data (whole 512-byte packets) into complete YUY2 frames."""

    def __init__(self, frame_bytes: int = FRAME_BYTES) -> None:
        self.frame_bytes = frame_bytes
        self._buf = bytearray()
        self._fid: Optional[int] = None
        self._bad = False
        self.frames_ok = 0
        self.frames_dropped = 0

    def feed(self, data: bytes) -> List[bytes]:
        out: List[bytes] = []
        for off in range(0, len(data), _PACKET):
            pkt = data[off : off + _PACKET]
            if len(pkt) < 2:
                continue
            hlen, flags = pkt[0], pkt[1]
            if hlen < 2 or hlen > len(pkt) or not flags & 0x80:
                self._bad = True
                continue
            fid = flags & 1
            if self._fid is not None and fid != self._fid:
                self._finish(out)
            self._fid = fid
            if flags & 0x40:
                self._bad = True
            self._buf += pkt[hlen:]
            if flags & 0x02:
                self._finish(out)
        return out

    def _finish(self, out: List[bytes]) -> None:
        if self._buf or self._bad:
            if len(self._buf) == self.frame_bytes and not self._bad:
                out.append(bytes(self._buf))
                self.frames_ok += 1
            else:
                self.frames_dropped += 1
        self._buf.clear()
        self._bad = False


def yuy2_to_bgr(frame: bytes, width: int = FRAME_W, height: int = FRAME_H) -> np.ndarray:
    raw = np.frombuffer(frame, dtype=np.uint8).reshape(height, width, 2)
    return cv2.cvtColor(raw, cv2.COLOR_YUV2BGR_YUY2)


_EXPOSURE_FILE = Path(__file__).resolve().parent.parent / "config" / "linkback_exposure.yaml"

EXPOSURE_DEFAULTS = {
    "enabled": True,
    "contrast_stretch": True,
    "target_luma": 100.0,
    "min_gamma": 0.55,
    "saturation": 1.5,
    "adapt": 0.08,
}
# (min, max) for each numeric setting.
EXPOSURE_LIMITS = {
    "target_luma": (40.0, 200.0),
    "min_gamma": (0.2, 1.0),
    "saturation": (0.0, 3.0),
    "adapt": (0.01, 1.0),
}

_MAX_GAMMA = 2.2  # strongest darkening the exposure control may apply

_exposure_settings: Optional[dict] = None


def _clean_exposure(raw: dict) -> dict:
    out = dict(EXPOSURE_DEFAULTS)
    for key in ("enabled", "contrast_stretch"):
        if key in raw:
            out[key] = bool(raw[key])
    for key, (lo, hi) in EXPOSURE_LIMITS.items():
        if key in raw:
            try:
                out[key] = min(hi, max(lo, float(raw[key])))
            except (TypeError, ValueError):
                pass
    return out


def load_exposure_config(path: Path = _HARDWARE_PATH) -> dict:
    """Exposure settings: defaults < ``cameras.linkback_exposure`` < dashboard-saved file."""
    merged: dict = {}
    for source, section in ((path, True), (_EXPOSURE_FILE, False)):
        try:
            with open(source) as f:
                cfg = yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError):
            continue
        if section:
            cfg = (cfg.get("cameras") or {}).get("linkback_exposure") or {}
        if isinstance(cfg, dict):
            merged.update(cfg)
    return _clean_exposure(merged)


def get_exposure_settings() -> dict:
    """Current settings, shared by every LinkBack camera (replaced, never edited in place)."""
    global _exposure_settings
    if _exposure_settings is None:
        _exposure_settings = load_exposure_config()
    return _exposure_settings


def set_exposure_settings(update: dict, *, save: bool = False) -> dict:
    """Apply a partial update live to all cameras; ``save`` keeps it across restarts."""
    global _exposure_settings
    merged = dict(get_exposure_settings())
    merged.update({k: v for k, v in update.items() if k in EXPOSURE_DEFAULTS})
    _exposure_settings = _clean_exposure(merged)
    if save:
        with open(_EXPOSURE_FILE, "w") as f:
            yaml.safe_dump(_exposure_settings, f, sort_keys=True)
    return dict(_exposure_settings)


def reset_exposure_settings(*, save: bool = True) -> dict:
    """Back to the hardware.yaml values (removes the saved override)."""
    global _exposure_settings
    if save:
        try:
            _EXPOSURE_FILE.unlink()
        except OSError:
            pass
    _exposure_settings = None
    return dict(get_exposure_settings())


class AutoExposure:
    """Software exposure for the camera's dark, flat picture.

    The sensor's own exposure is set through the vendor protocol, which is not
    known, so the picture is corrected in software, on the YUY2 bytes directly:

    * contrast stretch: the darkest ~1% of pixels go to black and the brightest
      to white, which removes the grey veil;
    * a slowly adapting gamma moves the average brightness toward ``target_luma``
      without clipping highlights (lifting shadows no more than ``min_gamma``
      allows; a lower target darkens instead);
    * ``saturation`` scales the colour (chroma) around neutral.
    """

    def __init__(self, cfg: Optional[dict] = None) -> None:
        # No cfg = follow the shared, dashboard-adjustable settings on every frame.
        self._follow_shared = cfg is None
        self._read(get_exposure_settings() if cfg is None else _clean_exposure(cfg))
        self.gamma = 1.0
        self.low = 0.0
        self.high = 255.0
        self._primed = False

    def _read(self, cfg: dict) -> None:
        self.enabled = cfg["enabled"]
        self.target_luma = cfg["target_luma"]
        self.min_gamma = cfg["min_gamma"]
        self.adapt = cfg["adapt"]
        self.stretch = cfg["contrast_stretch"]
        self.saturation = cfg["saturation"]

    def apply(self, raw: np.ndarray) -> np.ndarray:
        """Correct the (H, W, 2) YUY2 array in place and return it."""
        if self._follow_shared:
            self._read(get_exposure_settings())
        if not self.enabled:
            return raw
        luma = raw[::4, ::4, 0]
        if self.stretch:
            lo, hi = (float(v) for v in np.percentile(luma, (1.0, 99.5)))
            hi = max(hi, lo + 48.0)
            if not self._primed:
                self.low, self.high, self._primed = lo, hi, True
            self.low += self.adapt * (lo - self.low)
            self.high += self.adapt * (hi - self.high)
        else:
            self.low, self.high = 0.0, 255.0
        span = max(1.0, self.high - self.low)
        stretched = np.clip((luma.astype(np.float32) - self.low) / span, 0.0, 1.0)
        mean = min(0.99, max(0.01, float(stretched.mean())))
        target = self.target_luma / 255.0
        # Below 1 lifts shadows (never past min_gamma), above 1 darkens; 255 stays 255.
        wanted = min(_MAX_GAMMA, max(self.min_gamma, float(np.log(target) / np.log(mean))))
        self.gamma += self.adapt * (wanted - self.gamma)
        x = np.clip((np.arange(256, dtype=np.float32) - self.low) / span, 0.0, 1.0)
        raw[..., 0] = np.round((x ** self.gamma) * 255.0).astype(np.uint8)[raw[..., 0]]
        if self.saturation != 1.0:
            c = np.clip((np.arange(256, dtype=np.float32) - 128.0) * self.saturation + 128.0, 0, 255)
            raw[..., 1] = np.round(c).astype(np.uint8)[raw[..., 1]]
        return raw


class LinkbackCamera:
    """Threaded USB reader exposing the same calls ``camera.Camera`` does."""

    def __init__(
        self,
        device: int | str = "linkback",
        width: int = FRAME_W,
        height: int = FRAME_H,
        fps: int = 30,
        **_ignored,
    ) -> None:
        # Requested size is ignored: the sensor only outputs 450x450.
        self.device = str(device).strip()
        self.width = FRAME_W
        self.height = FRAME_H
        self.fps = fps
        self.native_size: Tuple[int, int] = (FRAME_W, FRAME_H)
        self.port_path = ""
        self._cond = threading.Condition()
        self._latest: Optional[bytes] = None
        self._seq = 0
        self._taken = 0
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._dead = False
        self._ready = threading.Event()
        self._open_error: Optional[str] = None
        self.frames_ok = 0
        self.frames_dropped = 0
        self.exposure = AutoExposure()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self) -> None:
        from src.camera import CameraError

        self.release()
        self._stop.clear()
        self._dead = False
        self._ready.clear()
        self._open_error = None
        self._latest = None
        self._seq = 0
        self._taken = 0
        self._thread = threading.Thread(
            target=self._run, name=f"linkback-{self.device}", daemon=True
        )
        self._thread.start()
        if not self._ready.wait(timeout=5.0) or self._open_error:
            error = self._open_error or "timed out opening the camera"
            self.release()
            raise CameraError(error)

    def release(self) -> None:
        thread = self._thread
        self._stop.set()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=4.0)
        self._thread = None
        with _claim_lock:
            if _claimed.get(self.port_path) is self:
                del _claimed[self.port_path]
        with self._cond:
            self._latest = None
            self._cond.notify_all()

    def reopen(self) -> None:
        self.release()
        self.open()

    def is_open(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive() and not self._dead

    def remap_device(self, exclude: Sequence[str] = ()) -> None:
        """FrameGrabber hook: LinkBack cameras are found by USB port, not V4L node."""
        return None

    # ------------------------------------------------------------------
    # Frame reading
    # ------------------------------------------------------------------

    def grab(self) -> bool:
        """Wait briefly for a frame newer than the last one retrieved."""
        deadline = time.monotonic() + _NEW_FRAME_WAIT_S
        with self._cond:
            while self._seq == self._taken:
                if not self.is_open():
                    return False
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._cond.wait(remaining)
            return self._latest is not None

    def retrieve(self) -> Tuple[bool, Optional[np.ndarray]]:
        with self._cond:
            raw = self._latest
            self._taken = self._seq
        if raw is None:
            return False, None
        pixels = np.frombuffer(raw, dtype=np.uint8).reshape(FRAME_H, FRAME_W, 2).copy()
        pixels = self.exposure.apply(pixels)
        return True, cv2.cvtColor(pixels, cv2.COLOR_YUV2BGR_YUY2)

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        from src.camera import CameraError

        if self._thread is None:
            raise CameraError("Camera is not open. Call open() first.")
        if not self.grab():
            return False, None
        return self.retrieve()

    def frame_size(self) -> Tuple[int, int]:
        return self.width, self.height

    def frames(self) -> Generator[np.ndarray, None, None]:
        while self.is_open():
            ok, frame = self.read()
            if not ok or frame is None:
                break
            yield frame

    def __enter__(self) -> "LinkbackCamera":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.release()

    # ------------------------------------------------------------------
    # USB thread
    # ------------------------------------------------------------------

    def _fail(self, message: str) -> None:
        self._open_error = message
        self._ready.set()

    def _run(self) -> None:
        try:
            import usb1
        except ImportError:
            self._fail("python-libusb1 is not installed (pip install libusb1)")
            return

        want = spec_port_path(self.device)
        ctx = usb1.USBContext()
        handle = None
        claimed_ifaces: List[int] = []
        transfers: List = []
        in_flight = [0]
        try:
            handle = self._open_handle(usb1, ctx, want)
            if handle is None:
                return
            try:
                for iface in _INTERFACES:
                    handle.claimInterface(iface)
                    claimed_ifaces.append(iface)
                handle.setInterfaceAltSetting(_STREAM_IFACE, _STREAM_ALT)
            except usb1.USBError as exc:
                self._fail(f"cannot start LinkBack camera {self.port_path}: {exc}")
                return

            assembler = FrameAssembler()

            def on_transfer(transfer) -> None:
                in_flight[0] -= 1
                status = transfer.getStatus()
                if status == usb1.TRANSFER_COMPLETED:
                    n = transfer.getActualLength()
                    if n:
                        self._publish(assembler, bytes(transfer.getBuffer()[:n]))
                elif status in (usb1.TRANSFER_NO_DEVICE, usb1.TRANSFER_ERROR, usb1.TRANSFER_STALL):
                    self._dead = True
                if not self._stop.is_set() and not self._dead:
                    try:
                        transfer.submit()
                        in_flight[0] += 1
                    except usb1.USBError:
                        self._dead = True

            for _ in range(_QUEUED_TRANSFERS):
                transfer = handle.getTransfer()
                transfer.setBulk(_EP_IN, _TRANSFER_BYTES, callback=on_transfer, timeout=0)
                transfer.submit()
                in_flight[0] += 1
                transfers.append(transfer)

            with _claim_lock:
                _claimed[self.port_path] = self
            print(f"[camera] LinkBack camera streaming on linkback:{self.port_path}", flush=True)
            self._ready.set()

            while not self._stop.is_set() and not self._dead:
                ctx.handleEventsTimeout(0.1)

            for transfer in transfers:
                try:
                    transfer.cancel()
                except usb1.USBError:
                    pass
            end = time.monotonic() + 2.0
            while in_flight[0] > 0 and time.monotonic() < end:
                ctx.handleEventsTimeout(0.05)
        except usb1.USBError as exc:
            self._dead = True
            self._fail(f"LinkBack camera error: {exc}")
        finally:
            self._dead = True
            for transfer in transfers:
                try:
                    transfer.close()
                except Exception:
                    pass
            if handle is not None:
                try:
                    handle.setInterfaceAltSetting(_STREAM_IFACE, 0)
                except Exception:
                    pass
                for iface in claimed_ifaces:
                    try:
                        handle.releaseInterface(iface)
                    except Exception:
                        pass
                try:
                    handle.close()
                except Exception:
                    pass
            try:
                ctx.close()
            except Exception:
                pass
            with self._cond:
                self._cond.notify_all()

    def _open_handle(self, usb1, ctx, want: str):
        seen: List[str] = []
        candidates = []
        for dev in ctx.getDeviceList(skip_on_error=True):
            if (dev.getVendorID(), dev.getProductID()) != (LINKBACK_VID, LINKBACK_PID):
                continue
            ports = ".".join(str(p) for p in dev.getPortNumberList())
            candidates.append((f"{dev.getBusNumber()}-{ports}", dev))
        # Lowest port first, so "linkback" resolves the same way on every start.
        for path, dev in sorted(candidates, key=lambda c: c[0]):
            seen.append(path)
            if want and path != want:
                continue
            with _claim_lock:
                owner = _claimed.get(path)
                if owner is not None and owner is not self:
                    continue
            try:
                handle = dev.open()
            except usb1.USBErrorAccess:
                self._fail(
                    f"permission denied opening LinkBack camera {path} — install "
                    "config/udev/99-linkback-camera.rules into /etc/udev/rules.d/"
                )
                return None
            except usb1.USBError as exc:
                self._fail(f"cannot open LinkBack camera {path}: {exc}")
                return None
            self.port_path = path
            return handle
        if want and want not in seen:
            self._fail(
                f"no LinkBack camera on port {want}"
                + (f" (found: {', '.join(seen)})" if seen else "")
            )
        elif seen:
            self._fail("all LinkBack cameras are already in use")
        else:
            self._fail("no LinkBack camera found (USB 3456:4321)")
        return None

    def _publish(self, assembler: FrameAssembler, data: bytes) -> None:
        frames = assembler.feed(data)
        self.frames_ok = assembler.frames_ok
        self.frames_dropped = assembler.frames_dropped
        if not frames:
            return
        with self._cond:
            self._latest = frames[-1]
            self._seq += 1
            self._cond.notify_all()
