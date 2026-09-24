"""
display.py
----------
Annotates an OpenCV frame with detection and control information and
shows it in a window.

Overlays drawn
--------------
  - Frame centre crosshair (thin white lines)
  - Apple bounding box (green when detected, red label when not)
  - Error vector line: frame centre → apple centre (yellow arrow)
  - HUD text: FPS, confidence, error_x / error_y, servo values
  - Mini bar gauges for steering, drive, and tilt
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import replace
from datetime import datetime
from typing import Optional

import cv2
import numpy as np

from src.controller import ControlOutput


# Colour palette (BGR)
COL_CROSSHAIR  = (200, 200, 200)   # light grey
COL_BBOX       = (0, 220, 60)      # green
COL_BBOX_NONE  = (60, 60, 200)     # blue tint when no apple
COL_VECTOR     = (0, 200, 255)     # yellow-orange
COL_TEXT       = (255, 255, 255)   # white
COL_TEXT_DIM   = (150, 150, 150)
COL_BAR_BG     = (50, 50, 50)
COL_BAR_FG     = (0, 200, 120)
COL_BAR_NEG    = (60, 80, 220)

FONT           = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE     = 0.45
FONT_THICKNESS = 1
# Overlay sizes were tuned for 640×480. Scale them with frame height.
_BASE_H = 480.0


def _overlay_metrics(frame_h: int) -> dict:
    """Font / line sizes that stay readable at 1600×1200 as well as 640×480."""
    s = max(1.0, float(frame_h) / _BASE_H)
    thick = max(1, int(round(FONT_THICKNESS * s)))
    return {
        "s": s,
        "font": FONT_SCALE * s,
        "thick": thick,
        "line_h": max(16, int(round(18 * s))),
        "pad": max(6, int(round(8 * s))),
        "bbox": max(2, int(round(2 * s))),
        "cross": max(1, int(round(1 * s))),
        "dot": max(4, int(round(4 * s))),
        "arrow": max(2, int(round(2 * s))),
        "bar_w": max(120, int(round(120 * s))),
        "bar_h": max(10, int(round(10 * s))),
        "gap_extra": max(50, int(round(50 * s))),
    }


class Display:
    """
    Annotated camera view.

    Parameters
    ----------
    window_name : str
        Title of the OpenCV window.
    fps_history : int
        Number of frames used for the rolling FPS average.
    headless : bool
        If True, imshow / waitKey calls are skipped (useful for SSH
        sessions without X forwarding).  Annotated frames are still
        returned from draw() so they can be saved or streamed.
    """

    def __init__(
        self,
        window_name: str = "Apple Tracker",
        fps_history: int = 30,
        headless: bool = False,
    ) -> None:
        self.window_name = window_name
        self.headless = headless
        self._timestamps: deque = deque(maxlen=fps_history)
        self._window_created = False
        # Display-only bbox EMA (does not affect control).
        self._box_ema: dict[str, tuple[float, ...]] = {}
        self._box_alpha = 0.28

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def draw(
        self,
        frame: np.ndarray,
        output: ControlOutput,
        count_fps: bool = True,
        copy: bool = True,
        overlay_fps: float | None = None,
        hud: bool = True,
        gauges: bool = True,
        view_id: str = "main",
        smooth_box: bool = True,
    ) -> np.ndarray:
        """Draw overlays onto `frame` and return the annotated image.

        ``smooth_box`` only affects drawing. Tracker / controller still
        see the raw box.
        """
        if count_fps:
            self._timestamps.append(time.monotonic())
        annotated = frame.copy() if copy else frame

        h, w = annotated.shape[:2]
        cx, cy = w // 2, h // 2
        m = _overlay_metrics(h)
        drawn = (
            self._smoothed_for_display(view_id, output)
            if smooth_box
            else output
        )

        self._draw_crosshair(annotated, cx, cy, m)

        if drawn.apple_detected:
            self._draw_bbox(annotated, drawn, m)
            self._draw_error_vector(
                annotated, cx, cy, drawn.target_x, drawn.target_y, m
            )

        if hud or gauges:
            self.overlay_hud(
                annotated,
                output,
                overlay_fps=overlay_fps,
                gauges=gauges,
                hud=hud,
                supersample=1,
            )

        return annotated

    def overlay_hud(
        self,
        img: np.ndarray,
        output: ControlOutput,
        *,
        overlay_fps: float | None = None,
        gauges: bool = True,
        hud: bool = True,
        supersample: int = 1,
    ) -> np.ndarray:
        """Burn HUD/gauges onto the stream-sized frame.

        Call this *after* the video is resized for JPEG so text is 1:1
        with stream pixels (sharp without a 3× buffer that tanks FPS).
        """
        if not hud and not gauges:
            return img
        h, w = img.shape[:2]
        ss = max(1, int(supersample))
        if overlay_fps is None:
            self._timestamps.append(time.monotonic())
        fps_shown = self.fps if overlay_fps is None else overlay_fps
        if ss == 1:
            m = _overlay_metrics(h)
            if hud:
                self._draw_hud(img, output, w, h, m, fps_shown=fps_shown)
            if gauges:
                self._draw_gauges(img, output, w, h, m)
            return img

        ch, cw = h * ss, w * ss
        layer = np.zeros((ch, cw, 3), dtype=np.uint8)
        m = _overlay_metrics(ch)
        if hud:
            self._draw_hud(layer, output, cw, ch, m, fps_shown=fps_shown)
        if gauges:
            self._draw_gauges(layer, output, cw, ch, m)
        small = cv2.resize(layer, (w, h), interpolation=cv2.INTER_AREA)
        alpha = small.max(axis=2).astype(np.float32) / 255.0
        alpha = np.clip(alpha * 1.35, 0.0, 1.0)[..., None]
        blended = small.astype(np.float32) * alpha + img.astype(np.float32) * (1.0 - alpha)
        np.copyto(img, blended.astype(np.uint8))
        return img

    def _smoothed_for_display(
        self, view_id: str, output: ControlOutput
    ) -> ControlOutput:
        if not output.apple_detected:
            self._box_ema.pop(view_id, None)
            return output
        cur = (
            float(output.bbox_x1),
            float(output.bbox_y1),
            float(output.bbox_x2),
            float(output.bbox_y2),
            float(output.target_x),
            float(output.target_y),
        )
        prev = self._box_ema.get(view_id)
        if prev is None:
            self._box_ema[view_id] = cur
            return output
        a = self._box_alpha
        sm = tuple(a * c + (1.0 - a) * p for c, p in zip(cur, prev))
        self._box_ema[view_id] = sm
        return replace(
            output,
            bbox_x1=int(round(sm[0])),
            bbox_y1=int(round(sm[1])),
            bbox_x2=int(round(sm[2])),
            bbox_y2=int(round(sm[3])),
            target_x=int(round(sm[4])),
            target_y=int(round(sm[5])),
        )

    def show(self, annotated: np.ndarray) -> bool:
        """
        Display the annotated frame.

        Returns
        -------
        bool
            False if the user pressed 'q' and wants to quit.
        """
        if self.headless:
            return True

        try:
            if not self._window_created:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                self._window_created = True

            cv2.imshow(self.window_name, annotated)
            key = cv2.waitKey(1) & 0xFF
            return key != ord("q")
        except cv2.error as exc:
            # Auto-fallback for headless environments where HighGUI is unavailable.
            self.headless = True
            self._window_created = False
            print(f"[display] GUI unavailable; switching to headless mode ({exc})")
            return True

    def close(self) -> None:
        if not self.headless and self._window_created:
            cv2.destroyWindow(self.window_name)
            self._window_created = False

    @property
    def fps(self) -> float:
        """Rolling average FPS based on recent draw() calls."""
        if len(self._timestamps) < 2:
            return 0.0
        elapsed = self._timestamps[-1] - self._timestamps[0]
        if elapsed <= 0:
            return 0.0
        return (len(self._timestamps) - 1) / elapsed

    # ------------------------------------------------------------------
    # Drawing helpers
    # ------------------------------------------------------------------

    def _draw_crosshair(self, img: np.ndarray, cx: int, cy: int, m: dict) -> None:
        h, w = img.shape[:2]
        cv2.line(img, (0, cy), (w, cy), COL_CROSSHAIR, m["cross"], cv2.LINE_AA)
        cv2.line(img, (cx, 0), (cx, h), COL_CROSSHAIR, m["cross"], cv2.LINE_AA)
        cv2.circle(img, (cx, cy), max(6, int(round(6 * m["s"]))), COL_CROSSHAIR, m["cross"], cv2.LINE_AA)

    def _draw_bbox(self, img: np.ndarray, output: ControlOutput, m: dict) -> None:
        # Use actual bounding box endpoints when available
        if output.bbox_x2 > output.bbox_x1 and output.bbox_y2 > output.bbox_y1:
            x1, y1 = output.bbox_x1, output.bbox_y1
            x2, y2 = output.bbox_x2, output.bbox_y2
        else:
            pad = max(10, int(round(10 * m["s"])))
            x1 = output.target_x - pad
            y1 = output.target_y - pad
            x2 = output.target_x + pad
            y2 = output.target_y + pad

        cv2.rectangle(img, (x1, y1), (x2, y2), COL_BBOX, m["bbox"])
        cv2.circle(img, (output.target_x, output.target_y), m["dot"], COL_BBOX, -1)
        label = f"{output.chosen_label or 'apple'} {output.confidence:.2f}"
        (tw, th), _ = cv2.getTextSize(label, FONT, m["font"], m["thick"])
        cap_h = th + max(6, int(round(6 * m["s"])))
        cv2.rectangle(img, (x1, y1 - cap_h), (x1 + tw + 4, y1), COL_BBOX, -1)
        cv2.putText(
            img, label, (x1 + 2, y1 - max(4, int(round(4 * m["s"])))),
            FONT, m["font"], (0, 0, 0), m["thick"], cv2.LINE_AA,
        )

    def _draw_error_vector(
        self, img: np.ndarray, cx: int, cy: int, tx: int, ty: int, m: dict
    ) -> None:
        cv2.arrowedLine(
            img, (cx, cy), (tx, ty),
            COL_VECTOR, m["arrow"], cv2.LINE_AA, tipLength=0.15,
        )

    def _draw_hud(
        self,
        img: np.ndarray,
        output: ControlOutput,
        w: int,
        h: int,
        m: dict,
        fps_shown: float | None = None,
    ) -> None:
        # Prefer controller-filtered ratio (smoothed; matches D)
        size_f = getattr(output, "size_ratio_filtered", 0.0) or 0.0
        size_r = getattr(output, "size_ratio_raw", 0.0) or 0.0
        if size_f <= 0.0 and output.frame_area > 0 and output.bbox_area > 0:
            size_f = output.bbox_area / output.frame_area
        prox = getattr(output, "proximity_t", 0.0)
        prox_tag = ""
        if getattr(output, "stereo_ok", False) and getattr(output, "range_m", None) is not None:
            prox_tag = " rng"
            if getattr(output, "stereo_note", "") == "hold":
                prox_tag = " rng hold"
        elif "size" in (getattr(output, "approach_note", "") or ""):
            prox_tag = " sz"
        lines = [
            datetime.now().strftime("Clk:  %H:%M:%S.%f")[:-3],
            f"FPS: {(self.fps if fps_shown is None else fps_shown):5.1f}",
            f"Apple: {'YES' if output.apple_detected else 'NO '}",
            f"Conf:  {output.confidence:.3f}",
            f"Size:  {size_f:.3f} (r{size_r:.3f})",
            f"Prox:  {prox:.3f}{prox_tag}",
        ]
        range_m = getattr(output, "range_m", None)
        if range_m is not None:
            lines.append(f"Range: {range_m:.2f} m")
        elif getattr(output, "stereo_note", ""):
            lines.append(f"Stereo:{output.stereo_note}")
        depth_used = getattr(output, "depth_m_used", None)
        if depth_used is not None:
            lines.append(f"Depth: {depth_used:.2f}m")
        note = getattr(output, "approach_note", "") or ""
        if note:
            lines.append(f"Note:  {note}")
        lines.extend([
            f"err_x: {output.error_x:+.4f}",
            f"err_y: {output.error_y:+.4f}",
            f"Steer: {output.steering_servo:+.3f}",
            f"Drive: {output.drive_motor:+.3f}",
            f"Tilt:  {output.camera_tilt_servo:+.3f}",
        ])
        x, y = m["pad"], m["line_h"]
        for line in lines:
            # Shadow
            cv2.putText(img, line, (x + 1, y + 1), FONT, m["font"],
                        (0, 0, 0), m["thick"] + 1, cv2.LINE_AA)
            cv2.putText(img, line, (x, y), FONT, m["font"],
                        COL_TEXT, m["thick"], cv2.LINE_AA)
            y += m["line_h"]

    def _draw_gauges(
        self, img: np.ndarray, output: ControlOutput, w: int, h: int, m: dict
    ) -> None:
        """
        Draw three horizontal bar gauges at the bottom of the frame.
        Centre = 0.  Green = positive, blue = negative.
        """
        gauge_specs = [
            ("Steer", output.steering_servo),
            ("Drive", output.drive_motor),
            ("Tilt",  output.camera_tilt_servo),
        ]
        bar_w = m["bar_w"]
        bar_h = m["bar_h"]
        margin = m["pad"]
        gap = bar_w + m["gap_extra"]
        start_x = margin
        base_y = h - margin - bar_h

        for i, (label, value) in enumerate(gauge_specs):
            ox = start_x + i * gap
            # Background
            cv2.rectangle(img, (ox, base_y), (ox + bar_w, base_y + bar_h),
                          COL_BAR_BG, -1)
            # Fill
            centre = ox + bar_w // 2
            fill_px = int(value * (bar_w // 2))
            if fill_px >= 0:
                cv2.rectangle(img, (centre, base_y),
                              (centre + fill_px, base_y + bar_h), COL_BAR_FG, -1)
            else:
                cv2.rectangle(img, (centre + fill_px, base_y),
                              (centre, base_y + bar_h), COL_BAR_NEG, -1)
            # Centre tick
            cv2.line(img, (centre, base_y - 2), (centre, base_y + bar_h + 2),
                     COL_TEXT_DIM, m["cross"])
            # Label
            cv2.putText(img, f"{label}: {value:+.2f}",
                        (ox, base_y - max(4, int(round(4 * m["s"])))), FONT, m["font"],
                        COL_TEXT, m["thick"], cv2.LINE_AA)
