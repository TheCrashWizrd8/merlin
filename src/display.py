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

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def draw(
        self,
        frame: np.ndarray,
        output: ControlOutput,
        count_fps: bool = True,
        copy: bool = True,
    ) -> np.ndarray:
        """Draw overlays onto `frame` and return the annotated image."""
        if count_fps:
            self._timestamps.append(time.monotonic())
        annotated = frame.copy() if copy else frame

        h, w = annotated.shape[:2]
        cx, cy = w // 2, h // 2

        self._draw_crosshair(annotated, cx, cy)

        if output.apple_detected:
            self._draw_bbox(annotated, output)
            self._draw_error_vector(annotated, cx, cy, output.target_x, output.target_y)

        self._draw_hud(annotated, output, w, h)
        self._draw_gauges(annotated, output, w, h)

        return annotated

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

    def _draw_crosshair(self, img: np.ndarray, cx: int, cy: int) -> None:
        h, w = img.shape[:2]
        cv2.line(img, (0, cy), (w, cy), COL_CROSSHAIR, 1, cv2.LINE_AA)
        cv2.line(img, (cx, 0), (cx, h), COL_CROSSHAIR, 1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 6, COL_CROSSHAIR, 1, cv2.LINE_AA)

    def _draw_bbox(self, img: np.ndarray, output: ControlOutput) -> None:
        # Use actual bounding box endpoints when available
        if output.bbox_x2 > output.bbox_x1 and output.bbox_y2 > output.bbox_y1:
            x1, y1 = output.bbox_x1, output.bbox_y1
            x2, y2 = output.bbox_x2, output.bbox_y2
        else:
            x1 = output.target_x - 10
            y1 = output.target_y - 10
            x2 = output.target_x + 10
            y2 = output.target_y + 10

        cv2.rectangle(img, (x1, y1), (x2, y2), COL_BBOX, 2)
        cv2.circle(img, (output.target_x, output.target_y), 4, COL_BBOX, -1)
        label = f"apple {output.confidence:.2f}"
        (tw, th), _ = cv2.getTextSize(label, FONT, FONT_SCALE, FONT_THICKNESS)
        cv2.rectangle(img, (x1, y1 - th - 6), (x1 + tw + 4, y1), COL_BBOX, -1)
        cv2.putText(img, label, (x1 + 2, y1 - 4), FONT, FONT_SCALE,
                    (0, 0, 0), FONT_THICKNESS, cv2.LINE_AA)

    def _draw_error_vector(
        self, img: np.ndarray, cx: int, cy: int, tx: int, ty: int
    ) -> None:
        cv2.arrowedLine(
            img, (cx, cy), (tx, ty),
            COL_VECTOR, 2, cv2.LINE_AA, tipLength=0.15,
        )

    def _draw_hud(
        self, img: np.ndarray, output: ControlOutput, w: int, h: int
    ) -> None:
        # Prefer controller-filtered ratio (smoothed; matches D)
        size_f = getattr(output, "size_ratio_filtered", 0.0) or 0.0
        size_r = getattr(output, "size_ratio_raw", 0.0) or 0.0
        if size_f <= 0.0 and output.frame_area > 0 and output.bbox_area > 0:
            size_f = output.bbox_area / output.frame_area
        lines = [
            datetime.now().strftime("Clk:  %H:%M:%S.%f")[:-3],
            f"FPS: {self.fps:5.1f}",
            f"Apple: {'YES' if output.apple_detected else 'NO '}",
            f"Conf:  {output.confidence:.3f}",
            f"Size:  {size_f:.3f} (r{size_r:.3f})",
            f"Prox:  {getattr(output, 'proximity_t', 0.0):.3f}",
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
        x, y = 8, 18
        for line in lines:
            # Shadow
            cv2.putText(img, line, (x + 1, y + 1), FONT, FONT_SCALE,
                        (0, 0, 0), FONT_THICKNESS + 1, cv2.LINE_AA)
            cv2.putText(img, line, (x, y), FONT, FONT_SCALE,
                        COL_TEXT, FONT_THICKNESS, cv2.LINE_AA)
            y += 18

    def _draw_gauges(
        self, img: np.ndarray, output: ControlOutput, w: int, h: int
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
        bar_w = 120
        bar_h = 10
        margin = 8
        gap = bar_w + 50
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
                     COL_TEXT_DIM, 1)
            # Label
            cv2.putText(img, f"{label}: {value:+.2f}",
                        (ox, base_y - 4), FONT, FONT_SCALE,
                        COL_TEXT, FONT_THICKNESS, cv2.LINE_AA)
