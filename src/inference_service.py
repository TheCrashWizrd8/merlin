"""
inference_service.py
--------------------
Start / stop / switch YOLO from the dashboard (``~/sub`` / ``run.py`` /
``sub_server.py``).

Both USB cameras stay open for the JPEG stream (side-by-side when stereo
is configured). Starting a model loads the backend and runs detect on
**each** camera every frame; stopping unloads the net and leaves the
live stereo preview running.
"""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import Any, Optional

from src.model_runtime import (
    catalog_snapshot,
    init_model_runtime,
    shutdown_model_runtime,
)

_rate_log_at: dict[str, float] = {}
_PREVIEW_EYE_MAX_W = 640  # two-up mosaic matches the 1280 JPEG stream
_PREVIEW_PERIOD_S = 0.08


def _rate_print(key: str, msg: str, interval_s: float = 5.0) -> None:
    now = time.monotonic()
    if now - _rate_log_at.get(key, 0.0) < interval_s:
        return
    _rate_log_at[key] = now
    print(msg)

_service: Optional["InferenceService"] = None
_service_lock = threading.Lock()


def get_inference_service() -> "InferenceService":
    global _service
    with _service_lock:
        if _service is None:
            _service = InferenceService()
        return _service


def _output_for_view(output, track):
    from dataclasses import replace

    from src.tracker import TrackResult

    if not isinstance(track, TrackResult) or not track.apple_detected:
        return replace(
            output,
            apple_detected=False,
            target_x=0,
            target_y=0,
            bbox_x1=0,
            bbox_y1=0,
            bbox_x2=0,
            bbox_y2=0,
            confidence=0.0,
        )
    return replace(
        output,
        apple_detected=True,
        target_x=track.target_x,
        target_y=track.target_y,
        bbox_x1=track.bbox_x1,
        bbox_y1=track.bbox_y1,
        bbox_x2=track.bbox_x2,
        bbox_y2=track.bbox_y2,
        bbox_width=track.bbox_width,
        bbox_height=track.bbox_height,
        bbox_area=track.bbox_area,
        frame_area=track.frame_area,
        confidence=track.confidence,
        error_x=track.error_x,
        error_y=track.error_y,
    )


def _scale_view(output, scale: float):
    if output is None or scale == 1.0:
        return output
    from dataclasses import replace

    return replace(
        output,
        target_x=int(round(output.target_x * scale)),
        target_y=int(round(output.target_y * scale)),
        bbox_x1=int(round(output.bbox_x1 * scale)),
        bbox_y1=int(round(output.bbox_y1 * scale)),
        bbox_x2=int(round(output.bbox_x2 * scale)),
        bbox_y2=int(round(output.bbox_y2 * scale)),
        bbox_width=int(round(output.bbox_width * scale)),
        bbox_height=int(round(output.bbox_height * scale)),
    )


def _preview_eye(frame, display, output, track, view_id: str, max_w: int = _PREVIEW_EYE_MAX_W):
    """Downscale one camera before drawing boxes so the mosaic never hits 3200px."""
    import cv2

    h, w = frame.shape[:2]
    scale = 1.0
    if w > max_w:
        scale = max_w / float(w)
        frame = cv2.resize(
            frame,
            (max_w, max(1, int(round(h * scale)))),
            interpolation=cv2.INTER_AREA,
        )
    else:
        frame = frame.copy()
    if output is not None and track is not None:
        drawn = _output_for_view(output, track)
        if scale != 1.0:
            drawn = _scale_view(drawn, scale)
        frame = display.draw(
            frame,
            drawn,
            count_fps=False,
            copy=False,
            hud=False,
            gauges=False,
            view_id=view_id,
        )
    return frame


def _pi_load_note() -> str:
    """CPU temp + Pi throttle flags for the 30s [yolo] timing line."""
    bits: list[str] = []
    try:
        raw = Path("/sys/class/thermal/thermal_zone0/temp").read_text().strip()
        bits.append(f"cpu={int(raw) / 1000.0:.0f}C")
    except (OSError, ValueError):
        pass
    try:
        import subprocess

        r = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True,
            text=True,
            timeout=0.3,
        )
        token = (r.stdout or "").strip().split("=")[-1]
        n = int(token, 16)
        names = []
        if n & 1:
            names.append("undervolt")
        if n & 2:
            names.append("freq-cap")
        if n & 4:
            names.append("throttled")
        if n & 8:
            names.append("soft-temp")
        if n & 0x10000:
            names.append("uv-hist")
        if n & 0x20000:
            names.append("cap-hist")
        if n & 0x40000:
            names.append("thr-hist")
        if n & 0x80000:
            names.append("st-hist")
        if names:
            bits.append("throttle=" + ",".join(names))
        elif n:
            bits.append(f"throttled={token}")
    except Exception:
        pass
    return ("  " + " ".join(bits)) if bits else ""


class _CameraRig:
    """FOV pilot cam + stereo pair for YOLO, shared by preview and inference."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.cam = None
        self.cam_right = None
        self.cam_fov = None
        self.grabber = None
        self.grabber_right = None
        self.grabber_fov = None
        self._fov_dedicated = False
        self.use_stereo = False
        self.left_dev: int | str | None = None
        self.right_dev: int | str | None = None
        self.fov_dev: int | str | None = None
        self.width = 640
        self.height = 480
        self.fov_width = 0
        self.fov_height = 0

    def open(
        self,
        preferred_left: int | str | None = None,
        *,
        left_device: int | str | None = None,
        right_device: int | str | None = None,
        force_stereo: bool = False,
        force_mono: bool = False,
        width: int = 640,
        height: int = 480,
        fov_width: int | None = None,
        fov_height: int | None = None,
    ) -> None:
        from src.camera import (
            Camera,
            CameraError,
            FrameGrabber,
            assign_camera_device,
            format_usb_camera_report,
            resolve_fov_capture_size,
        )
        from src.stereo import load_stereo_config

        with self._lock:
            if self.grabber is not None:
                return

            stereo_cfg = load_stereo_config()
            use_stereo = bool(stereo_cfg.enabled)
            if force_stereo:
                use_stereo = True
            if force_mono:
                use_stereo = False
            self.width = int(width)
            self.height = int(height)
            self.fov_width = int(
                fov_width if fov_width is not None else stereo_cfg.fov_width
            )
            self.fov_height = int(
                fov_height if fov_height is not None else stereo_cfg.fov_height
            )
            left_dev = left_device if left_device is not None else stereo_cfg.left_device
            if preferred_left is not None and left_device is None and not use_stereo:
                left_dev = preferred_left
            right_dev = (
                right_device if right_device is not None else stereo_cfg.right_device
            )
            fov_dev = stereo_cfg.fov_device
            if fov_dev is None:
                fov_dev = left_dev
            print(format_usb_camera_report())

            # Reserve the FOV camera first so a missing stereo node cannot
            # steal it and lock it at the YOLO 640×480 size.
            fov_exclude: tuple[str, ...] = ()
            if fov_dev is not None:
                fov_reserved, _ = assign_camera_device(fov_dev)
                fov_exclude = (str(fov_reserved),)

            if use_stereo:
                left_dev, n1 = assign_camera_device(left_dev, exclude=fov_exclude)
                right_dev, n2 = assign_camera_device(
                    right_dev, exclude=(str(left_dev),) + fov_exclude
                )
                for note in (n1, n2):
                    if note:
                        print(f"[yolo] {note}")
                cam = Camera(device=left_dev, width=self.width, height=self.height)
                try:
                    cam.open()
                except CameraError as exc:
                    print(f"[yolo] Left camera failed ({exc}); will retry")
                cam_right = Camera(device=right_dev, width=self.width, height=self.height)
                try:
                    cam_right.open()
                except CameraError as exc:
                    print(f"[yolo] Right camera failed ({exc}); will retry")
            else:
                # One USB camera: open the FOV/pilot node at max MJPEG.
                # preferred_left=0 used to steal it and lock it at YOLO 640×480.
                cam_src = fov_dev if fov_dev is not None else left_dev
                left_dev, note = assign_camera_device(cam_src)
                if note:
                    print(f"[yolo] {note}")
                fov_dev = left_dev
                fov_exclude = (str(left_dev),)
                self.fov_width, self.fov_height = resolve_fov_capture_size(
                    left_dev, self.fov_width, self.fov_height
                )
                print(f"[yolo] FOV capture {self.fov_width}x{self.fov_height}")
                cam = Camera(
                    device=left_dev,
                    width=self.fov_width,
                    height=self.fov_height,
                )
                try:
                    cam.open()
                except CameraError as exc:
                    print(f"[yolo] Camera failed ({exc}); will retry")
                cam_right = None

            cam_fov = None
            grabber_fov = None

            def _same_cam(a, b) -> bool:
                if a is None or b is None:
                    return False
                try:
                    return os.path.realpath(str(a)) == os.path.realpath(str(b))
                except OSError:
                    return str(a) == str(b)

            fov_dedicated = not _same_cam(fov_dev, left_dev)
            if fov_dedicated:
                fov_dev, fov_note = assign_camera_device(fov_dev)
                if fov_note:
                    print(f"[yolo] {fov_note}")
                self.fov_width, self.fov_height = resolve_fov_capture_size(
                    fov_dev, self.fov_width, self.fov_height
                )
                print(f"[yolo] FOV capture {self.fov_width}x{self.fov_height}")
                cam_fov = Camera(
                    device=fov_dev,
                    width=self.fov_width,
                    height=self.fov_height,
                )
                try:
                    cam_fov.open()
                except CameraError as exc:
                    print(f"[yolo] FOV camera failed ({exc}); falling back to left")
                    cam_fov = None
                    fov_dedicated = False
                    fov_dev = left_dev

            self.cam = cam
            self.cam_right = cam_right
            self.cam_fov = cam_fov

            def _exclude(*peers, extra=()):
                out = list(extra)
                for peer in peers:
                    if peer is None:
                        continue
                    device = getattr(peer, "device", None)
                    if device:
                        out.append(str(device))
                return tuple(out)

            grabber = FrameGrabber(
                cam,
                name="left",
                exclude=lambda: _exclude(
                    self.cam_right, self.cam_fov, extra=fov_exclude
                ),
            )
            grabber.start()
            grabber_right = None
            if cam_right is not None:
                grabber_right = FrameGrabber(
                    cam_right,
                    name="right",
                    exclude=lambda: _exclude(
                        self.cam, self.cam_fov, extra=fov_exclude
                    ),
                )
                grabber_right.start()

            if cam_fov is not None:
                grabber_fov = FrameGrabber(
                    cam_fov,
                    name="fov",
                    exclude=lambda: _exclude(self.cam, self.cam_right),
                )
                grabber_fov.start()
            else:
                grabber_fov = grabber

            self.grabber = grabber
            self.grabber_right = grabber_right
            self.grabber_fov = grabber_fov
            self._fov_dedicated = bool(fov_dedicated and cam_fov is not None)
            self.use_stereo = bool(use_stereo and cam_right is not None)
            self.left_dev = left_dev
            self.right_dev = right_dev if self.use_stereo else None
            self.fov_dev = fov_dev
            if self.use_stereo:
                print(
                    f"[yolo] Stereo YOLO left={left_dev} right={right_dev} "
                    f"fov={fov_dev}{' (dedicated)' if self._fov_dedicated else ''}"
                )
            else:
                print(f"[yolo] Mono YOLO on {left_dev} fov={fov_dev}")

    def close(self) -> None:
        with self._lock:
            grabber = self.grabber
            grabber_right = self.grabber_right
            grabber_fov = self.grabber_fov
            cam = self.cam
            cam_right = self.cam_right
            cam_fov = self.cam_fov
            self.grabber = self.grabber_right = self.grabber_fov = None
            self.cam = self.cam_right = self.cam_fov = None
            self.use_stereo = False
            self._fov_dedicated = False
        if grabber is not None:
            grabber.stop()
        if grabber_right is not None:
            grabber_right.stop()
        if grabber_fov is not None and grabber_fov is not grabber:
            grabber_fov.stop()
        if cam is not None:
            try:
                cam.release()
            except Exception:
                pass
        if cam_right is not None:
            try:
                cam_right.release()
            except Exception:
                pass
        if cam_fov is not None and cam_fov is not cam:
            try:
                cam_fov.release()
            except Exception:
                pass

    def peek(self, copy: bool = True):
        left_ok, left = False, None
        right_ok, right = False, None
        grabber = self.grabber
        grabber_right = self.grabber_right
        if grabber is not None:
            left_ok, left = grabber.peek(copy=copy)
        if grabber_right is not None:
            right_ok, right = grabber_right.peek(copy=copy)
        return left_ok, left, right_ok, right

    def peek_fov(self, copy: bool = True):
        grabber = self.grabber_fov or self.grabber
        if grabber is None:
            return False, None
        return grabber.peek(copy=copy)


class InferenceService:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._rig = _CameraRig()
        self._stop = threading.Event()
        self._preview_stop = threading.Event()
        self._preview_thread: Optional[threading.Thread] = None
        self._thread: Optional[threading.Thread] = None
        self._owned = False
        self._run_gen = 0
        self._pending: Optional[tuple[str, str]] = None
        self._model_id = ""
        self._backend = ""
        self._error: Optional[str] = None
        self._state = "idle"
        self._preview_device: int | str = 0
        self._want_preview = False
        self._hud: dict = {
            "lock": threading.Lock(),
            "output": None,
            "track_left": None,
            "track_right": None,
            "rig_reversed": False,
            "loop_fps": None,
        }
        self._timing = False
        self._print_on_detect = False
        self._quiet = False
        self._imgsz: Optional[int] = None
        self._control_profile = "config"
        self._tracker_strategy = "best_confidence"
        self._rig_kw: dict[str, Any] = {}

    def configure(
        self,
        *,
        timing: bool = False,
        print_on_detect: bool = False,
        quiet: bool = False,
        imgsz: Optional[int] = None,
        control_profile: str = "config",
        tracker_strategy: str = "best_confidence",
        width: int = 640,
        height: int = 480,
        fov_width: int | None = None,
        fov_height: int | None = None,
        left_device: int | str | None = None,
        right_device: int | str | None = None,
        force_stereo: bool = False,
        force_mono: bool = False,
    ) -> None:
        self._timing = bool(timing)
        self._print_on_detect = bool(print_on_detect)
        self._quiet = bool(quiet)
        self._imgsz = imgsz
        self._control_profile = control_profile
        self._tracker_strategy = tracker_strategy
        self._rig_kw = {
            "left_device": left_device,
            "right_device": right_device,
            "force_stereo": force_stereo,
            "force_mono": force_mono,
            "width": int(width),
            "height": int(height),
            "fov_width": fov_width,
            "fov_height": fov_height,
        }

    def enable_idle_preview(self, device: int | str = 0) -> None:
        self._preview_device = device
        self._want_preview = True
        try:
            self._rig.open(preferred_left=device, **self._rig_kw)
        except Exception as exc:
            print(f"[yolo] Camera preview unavailable: {exc}")
            return
        self._ensure_preview()

    def disable_idle_preview(self) -> None:
        self._want_preview = False
        self._stop_preview()
        self._rig.close()

    def _ensure_preview(self) -> None:
        if self._preview_thread is not None and self._preview_thread.is_alive():
            return
        self._preview_stop.clear()
        self._preview_thread = threading.Thread(
            target=self._preview_loop, name="cam-preview", daemon=True
        )
        self._preview_thread.start()

    def _stop_preview(self) -> None:
        self._preview_stop.set()
        thread = self._preview_thread
        if thread is not None:
            thread.join(timeout=2.0)
            self._preview_thread = None
        self._preview_stop.clear()

    def _preview_loop(self) -> None:
        from src.camera import camera_offline_frame
        from src.display import Display
        from src.web_stream import encoder_busy, set_latest_frame, yolo_channel_active

        display = Display(headless=True)
        try:
            while not self._preview_stop.is_set():
                t0 = time.monotonic()
                if encoder_busy():
                    self._preview_stop.wait(0.02)
                    continue
                ok_fov, fov = self._rig.peek_fov(copy=False)
                ok_l, left, ok_r, right = self._rig.peek(copy=False)
                with self._hud["lock"]:
                    output = self._hud.get("output")
                    track_left = self._hud.get("track_left")
                    track_right = self._hud.get("track_right")
                    loop_fps = self._hud.get("loop_fps")
                if not ok_fov or fov is None:
                    fov = camera_offline_frame(
                        self._rig.fov_width,
                        self._rig.fov_height,
                        "FOV\noffline — reconnecting",
                    )
                set_latest_frame(fov, channel="fov")

                if yolo_channel_active("yolo_left"):
                    if not ok_l or left is None:
                        left_out = camera_offline_frame(640, 480, "LEFT\noffline — reconnecting")
                    else:
                        left_out = _preview_eye(left, display, output, track_left, "left")
                    set_latest_frame(
                        left_out,
                        channel="yolo_left",
                        hud=output,
                        overlay_fps=loop_fps,
                    )

                if yolo_channel_active("yolo_right") and self._rig.use_stereo:
                    if not ok_r or right is None:
                        right_out = camera_offline_frame(640, 480, "RIGHT\noffline — reconnecting")
                    else:
                        right_out = _preview_eye(right, display, output, track_right, "right")
                    set_latest_frame(
                        right_out,
                        channel="yolo_right",
                        hud=output,
                        overlay_fps=loop_fps,
                    )

                self._preview_stop.wait(
                    max(0.0, _PREVIEW_PERIOD_S - (time.monotonic() - t0))
                )
        finally:
            display.close()

    def snapshot(self) -> dict[str, Any]:
        runtime = None
        try:
            from src.model_runtime import get_model_runtime

            runtime = get_model_runtime()
        except Exception:
            runtime = None
        with self._lock:
            model_id = self._model_id
            backend = self._backend
            state = self._state
            err = self._error
            owned = self._owned
            pending = self._pending
        if runtime is not None:
            model_id = runtime.active_id or model_id
            backend = runtime.backend or backend
            st = runtime.status_snapshot()
            if st.get("state"):
                state = st["state"]
                err = st.get("error")
        running = owned or (runtime is not None and state in ("ready", "loading"))
        if pending:
            state = "loading"
        status = {
            "state": state,
            "error": err,
            "label": "",
            "backend": backend,
        }
        if runtime is not None:
            status.update(runtime.status_snapshot())
            if pending:
                status["state"] = "loading"
        snap = catalog_snapshot(
            active_id=model_id,
            status=status,
            backend=backend,
            running=running,
        )
        snap["owned"] = owned
        if pending:
            snap["pending"] = True
            snap["pending_id"] = pending[0]
            snap["pending_backend"] = pending[1]
        return snap

    def start(self, model_id: str, backend: str) -> dict[str, Any]:
        model_id = str(model_id).strip()
        backend = str(backend or "").strip().lower()
        if not model_id:
            return {"ok": False, "error": "missing id"}
        if not backend:
            return {"ok": False, "error": "missing backend"}

        queued = False
        with self._lock:
            live = self._thread is not None and self._thread.is_alive()
            if live:
                self._owned = True
                if (model_id, backend) != (self._model_id, self._backend):
                    self._pending = (model_id, backend)
                    self._state = "loading"
                    self._error = None
                    queued = True
            else:
                self._run_gen += 1
                gen = self._run_gen
                self._model_id = model_id
                self._backend = backend
                self._state = "loading"
                self._error = None
                self._stop.clear()
                self._owned = True
                self._thread = threading.Thread(
                    target=self._run,
                    args=(model_id, backend, gen),
                    name="yolo-service",
                    daemon=True,
                )
                self._thread.start()
                queued = True
        snap = self.snapshot()
        snap["ok"] = True
        if queued:
            snap["pending"] = True
            snap["pending_id"] = model_id
            snap["pending_backend"] = backend
        return snap

    def stop(self, *, keep_preview: bool = True, preserve_control_mode: bool = False) -> dict[str, Any]:
        self._stop.set()
        with self._lock:
            self._run_gen += 1
            thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0 if not keep_preview else 30.0)
        with self._lock:
            self._owned = False
            self._thread = None
            self._pending = None
            self._state = "idle"
            self._error = None
        shutdown_model_runtime()
        self._clear_hud()
        if not keep_preview:
            try:
                self._rig.close()
            except Exception:
                pass
            return {"ok": True}
        if keep_preview and self._want_preview:
            try:
                self._rig.open(preferred_left=self._preview_device, **self._rig_kw)
            except Exception as exc:
                print(f"[yolo] Camera preview unavailable: {exc}")
            self._ensure_preview()
        if not preserve_control_mode:
            from src.control_source import set_mode
            from src.sub_state import get_sub_state

            get_sub_state().set_control_mode("manual")
            set_mode("manual")
        snap = self.snapshot()
        snap["ok"] = True
        return snap

    def _clear_hud(self) -> None:
        with self._hud["lock"]:
            self._hud["output"] = None
            self._hud["track_left"] = None
            self._hud["track_right"] = None
            self._hud["rig_reversed"] = False
            self._hud["loop_fps"] = None
            self._hud["detections_left"] = None
            self._hud["detections_right"] = None
            self._hud["stereo_result"] = None
        from src.vision_state import clear_vision

        clear_vision()

    def _run(self, model_id: str, backend: str, gen: int = 0) -> None:
        from dataclasses import replace

        from src.control_source import SDT, get_current_sdt, set_mode
        from src.controller import Controller
        from src.hardware import StubOutput
        from src.hardware import from_config as hardware_from_config
        from src.stereo import (
            fuse_tracks,
            load_stereo_config,
            match_max_dy_px,
            pair_tracks,
            triangulate_with_swap,
        )
        from src.sub_state import get_sub_state
        from src.tracker import Tracker

        stereo_cfg = load_stereo_config()
        hardware: object = StubOutput({})

        try:
            self._rig.open(preferred_left=self._preview_device, **self._rig_kw)
            self._ensure_preview()
            use_stereo = self._rig.use_stereo

            with self._lock:
                self._state = "loading"
            print(f"[yolo] Loading {model_id!r} backend={backend}")
            runtime = init_model_runtime(
                backend=backend,
                start_id=model_id,
                imgsz_override=self._imgsz,
                lazy=False,
            )
            if runtime.active_id != model_id:
                result = runtime.select(model_id, backend=backend)
                if not result.get("ok"):
                    if runtime.active_id:
                        print(
                            f"[yolo] {result.get('error')}; "
                            f"keeping {runtime.active_id}/{runtime.backend}"
                        )
                    else:
                        raise RuntimeError(
                            result.get("error")
                            or f"select {model_id}/{backend} failed"
                        )
            elif runtime.backend != backend:
                print(
                    f"[yolo] {backend} unavailable for {model_id!r}; "
                    f"running backend={runtime.backend}"
                )

            tracker = Tracker(strategy=self._tracker_strategy)
            tracker_right = Tracker(strategy=self._tracker_strategy) if use_stereo else None
            controller = Controller.from_hardware_config(profile=self._control_profile)
            try:
                hardware = hardware_from_config()
            except Exception as exc:
                print(f"[yolo] Hardware init failed; stub: {exc}")
                hardware = StubOutput({})

            yolo_state = get_sub_state()
            yolo_state.set_control_mode("auto")
            yolo_state.recompute_effective()
            set_mode("auto")

            with self._lock:
                self._state = "ready"
                self._error = None
                self._model_id = runtime.active_id
                self._backend = runtime.backend
            cams = (
                f"left={self._rig.left_dev} right={self._rig.right_dev}"
                if use_stereo
                else f"mono={self._rig.left_dev}"
            )
            infer_note = (
                "infer both cameras every frame"
                if use_stereo
                else "infer FOV every frame"
            )
            print(
                f"[yolo] Running model={runtime.active_id!r} "
                f"backend={runtime.backend} track={runtime.track_label!r} "
                f"{cams} ({infer_note})"
            )

            stereo_swap_streak = 0
            stereo_rig_reversed = False
            loop_n = 0
            n_prev = 0
            t_prev = time.monotonic()
            t_frame = t_prev
            hud_fps = 0.0

            while not self._stop.is_set():
                if gen and gen != self._run_gen:
                    break
                pending = None
                with self._lock:
                    pending = self._pending
                    self._pending = None
                if pending is not None:
                    pid, pbe = pending
                    with self._lock:
                        self._state = "loading"
                    print(f"[yolo] Switching to {pid!r} / {pbe}")
                    try:
                        result = runtime.select(pid, backend=pbe)
                        if not result.get("ok"):
                            raise RuntimeError(
                                result.get("error") or f"select {pid}/{pbe} failed"
                            )
                        with self._lock:
                            self._model_id = runtime.active_id
                            self._backend = runtime.backend
                            self._state = "ready"
                            self._error = None
                    except Exception as exc:
                        with self._lock:
                            self._state = "error"
                            self._error = str(exc)
                        print(f"[yolo] Switch failed: {exc}")

                ok, frame, ok_r, frame_right = self._rig.peek(copy=False)
                if not ok:
                    frame = None
                if not ok_r:
                    frame_right = None
                if frame is None and frame_right is None:
                    self._stop.wait(0.05)
                    continue

                try:
                    if use_stereo and frame is not None and frame_right is not None:
                        detections, detections_right = runtime.detect_pair(
                            frame, frame_right
                        )
                    elif frame is not None:
                        detections = runtime.detect(frame)
                        detections_right = []
                    else:
                        detections = []
                        detections_right = runtime.detect(frame_right)
                except Exception as exc:
                    _rate_print("detect", f"[yolo] detect failed: {exc}")
                    self._stop.wait(0.05)
                    continue

                track_label = runtime.track_label
                track_left = None
                if frame is not None:
                    track_left = tracker.update(
                        detections, frame.shape, apple_label=track_label
                    )
                track = track_left
                track_right = None
                stereo_result = None
                if (
                    use_stereo
                    and tracker_right is not None
                    and frame_right is not None
                ):
                    track_right = tracker_right.update(
                        detections_right, frame_right.shape, apple_label=track_label
                    )
                    if track_left is not None and frame is not None:
                        stereo_result, stereo_swapped = triangulate_with_swap(
                            track_left,
                            track_right,
                            frame.shape[1],
                            frame.shape[0],
                            stereo_cfg,
                        )
                        if stereo_swapped:
                            stereo_swap_streak += 1
                            if stereo_swap_streak >= 5 and not stereo_rig_reversed:
                                stereo_rig_reversed = True
                                with self._hud["lock"]:
                                    self._hud["rig_reversed"] = True
                        else:
                            stereo_swap_streak = 0
                        paired = pair_tracks(
                            track_left,
                            track_right,
                            match_max_dy_px(stereo_cfg, frame.shape[0]),
                        )
                        track = fuse_tracks(track_left, track_right, paired)
                    else:
                        track = track_right
                elif track is None:
                    self._stop.wait(0.01)
                    continue

                from src.telemetry_context import TelemetryContext

                telemetry = TelemetryContext.from_sub_state()
                if stereo_result is not None:
                    output = controller.compute(
                        track,
                        telemetry=telemetry,
                        range_m=stereo_result.range_m if stereo_result.ok else None,
                        stereo_ok=stereo_result.ok,
                        stereo_note="" if stereo_result.ok else stereo_result.reason,
                    )
                else:
                    output = controller.compute(track, telemetry=telemetry)
                sdt = get_current_sdt(
                    SDT(output.steering_servo, output.drive_motor, output.camera_tilt_servo)
                )
                output = replace(
                    output,
                    steering_servo=sdt.s,
                    drive_motor=sdt.d,
                    camera_tilt_servo=sdt.t,
                )
                try:
                    hardware.apply(output)
                except Exception as exc:
                    _rate_print("hw", f"[yolo] hardware.apply failed: {exc}")

                now = time.monotonic()
                dt = now - t_frame
                t_frame = now
                if dt > 1e-4:
                    inst = 1.0 / dt
                    hud_fps = inst if hud_fps <= 0 else (0.2 * inst + 0.8 * hud_fps)

                with self._hud["lock"]:
                    self._hud["output"] = output
                    self._hud["track_left"] = track_left
                    self._hud["track_right"] = track_right
                    self._hud["loop_fps"] = hud_fps
                    self._hud["detections_left"] = detections
                    self._hud["detections_right"] = detections_right
                    self._hud["stereo_result"] = stereo_result

                from src.vision_state import update_vision

                ok_fov, fov_frame = self._rig.peek_fov(copy=False)
                update_vision(
                    model_id=runtime.active_id,
                    backend=runtime.backend,
                    running=True,
                    left_shape=frame.shape if frame is not None else None,
                    right_shape=frame_right.shape if frame_right is not None else None,
                    fov_shape=fov_frame.shape if ok_fov and fov_frame is not None else None,
                    detections_left=detections,
                    detections_right=detections_right,
                    track_left=track_left,
                    track_right=track_right,
                    stereo_ok=bool(stereo_result and stereo_result.ok),
                    range_m=stereo_result.range_m if stereo_result and stereo_result.ok else None,
                    stereo_reason=stereo_result.reason if stereo_result and not stereo_result.ok else "",
                )

                if self._print_on_detect and getattr(output, "apple_detected", False):
                    print(output.pretty())
                if self._timing:
                    loop_n += 1
                    if now - t_prev >= 30.0:
                        window = now - t_prev
                        fps = ((loop_n - n_prev) / window) if window > 0 else 0.0
                        print(f"[yolo] n={loop_n}  ~{fps:.1f} fps{_pi_load_note()}")
                        t_prev = now
                        n_prev = loop_n

        except Exception as exc:
            print(f"[yolo] Service failed: {exc}")
            with self._lock:
                self._state = "error"
                self._error = str(exc)
        finally:
            self._clear_hud()
            try:
                hardware.close()
            except Exception:
                pass
            superseded = False
            with self._lock:
                superseded = bool(gen) and gen != self._run_gen
                if not superseded:
                    self._owned = False
                    self._thread = None
                    if self._state != "error":
                        self._state = "idle"
            if not superseded:
                shutdown_model_runtime()
                print("[yolo] Inference stopped")
