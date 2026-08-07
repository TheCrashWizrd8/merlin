"""
tracker.py
----------
Takes a list of Detection objects and a frame shape, picks the best
apple candidate, and returns its pixel coordinates together with the
normalised error from the frame centre.

Error convention
----------------
  error_x: -1.0 = apple at left edge,  0.0 = centred,  +1.0 = right edge
  error_y: -1.0 = apple at top edge,   0.0 = centred,  +1.0 = bottom edge

The controller uses these errors to decide how much to steer and tilt.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

from src.detector import Detection


@dataclass
class TrackResult:
    """Output of the tracker for a single frame."""
    apple_detected: bool
    # Pixel coordinates of the chosen apple's bounding-box centre
    target_x: int = 0
    target_y: int = 0
    # Bounding box of the chosen detection (for drawing)
    bbox_x1: int = 0
    bbox_y1: int = 0
    bbox_x2: int = 0
    bbox_y2: int = 0
    # Bbox size for distance estimation (larger = closer)
    bbox_width: int = 0
    bbox_height: int = 0
    bbox_area: int = 0
    frame_area: int = 0  # w*h of frame, for normalising bbox_area
    # Class label of the chosen detection (e.g. "apple", "damaged_apple")
    chosen_label: str = ""
    # Normalised errors relative to the frame centre (-1.0 … +1.0)
    error_x: float = 0.0
    error_y: float = 0.0
    confidence: float = 0.0


def _frame_centre(frame_shape: Tuple[int, int]) -> Tuple[int, int]:
    """Return (cx, cy) of the frame given (height, width)."""
    h, w = frame_shape[:2]
    return w // 2, h // 2


def _normalise_error(pixel_error: int, half_span: int) -> float:
    """
    Map a pixel offset from centre to the range [-1.0, +1.0].
    Clamps to the range in case the bounding box exceeds the frame.
    """
    if half_span == 0:
        return 0.0
    return max(-1.0, min(1.0, pixel_error / half_span))


class Tracker:
    """
    Selects the most relevant apple detection each frame and computes
    the steering / tilt error signals.

    Selection strategy
    ------------------
    By default the highest-confidence detection is chosen ("best_confidence").
    Set strategy="closest_to_centre" to prefer the apple nearest the frame
    centre instead — useful once the sub is already tracking and the target
    fills the frame.
    """

    STRATEGIES = ("best_confidence", "closest_to_centre")

    def __init__(self, strategy: str = "best_confidence") -> None:
        if strategy not in self.STRATEGIES:
            raise ValueError(
                f"Unknown strategy '{strategy}'. Choose from {self.STRATEGIES}"
            )
        self.strategy = strategy

    def update(
        self,
        detections: List[Detection],
        frame_shape: Tuple[int, int],
        apple_label: str = "apple",
    ) -> TrackResult:
        """
        Process detections for one frame.

        Parameters
        ----------
        detections : List[Detection]
            Raw detections from Detector.detect().
        frame_shape : Tuple[int, int]
            (height, width[, channels]) — i.e. frame.shape from OpenCV.
        apple_label : str
            The class label to track. Defaults to "apple".
            Override if the Roboflow dataset uses a different label string.

        Returns
        -------
        TrackResult
        """
        h, w = frame_shape[:2]
        cx_frame, cy_frame = w // 2, h // 2

        # Filter to apples only
        apples = [d for d in detections if d.label.lower() == apple_label.lower()]

        if not apples:
            return TrackResult(apple_detected=False)

        chosen = self._select(apples, cx_frame, cy_frame)

        # Bounding-box centre
        target_x = (chosen.x1 + chosen.x2) // 2
        target_y = (chosen.y1 + chosen.y2) // 2

        # Bbox size for distance estimation (larger apple = closer)
        bbox_w = chosen.x2 - chosen.x1
        bbox_h = chosen.y2 - chosen.y1
        bbox_area = bbox_w * bbox_h
        frame_area = w * h

        # Pixel error from frame centre (positive = apple is right/below centre)
        px_error_x = target_x - cx_frame
        px_error_y = target_y - cy_frame

        error_x = _normalise_error(px_error_x, w // 2)
        error_y = _normalise_error(px_error_y, h // 2)

        return TrackResult(
            apple_detected=True,
            target_x=target_x,
            target_y=target_y,
            bbox_x1=chosen.x1,
            bbox_y1=chosen.y1,
            bbox_x2=chosen.x2,
            bbox_y2=chosen.y2,
            bbox_width=bbox_w,
            bbox_height=bbox_h,
            bbox_area=bbox_area,
            frame_area=frame_area,
            chosen_label=chosen.label,
            error_x=error_x,
            error_y=error_y,
            confidence=chosen.confidence,
        )

    def _select(
        self,
        apples: List[Detection],
        cx_frame: int,
        cy_frame: int,
    ) -> Detection:
        if self.strategy == "best_confidence":
            # detections are already sorted by descending confidence from Detector
            return apples[0]

        # closest_to_centre
        def dist_sq(d: Detection) -> float:
            cx = (d.x1 + d.x2) / 2
            cy = (d.y1 + d.y2) / 2
            return (cx - cx_frame) ** 2 + (cy - cy_frame) ** 2

        return min(apples, key=dist_sq)
