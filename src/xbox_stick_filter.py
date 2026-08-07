"""
xbox_stick_filter.py
--------------------
Stabilize noisy Bluetooth stick reads: optional EMA + short hold on dropouts
so smoothing does not snap to zero between good samples.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass


@dataclass
class StickFilterConfig:
    smoothing_alpha: float = 0.0
    stick_hold_ms: float = 120.0
    release_alpha: float = 0.55  # decay when stick returned to centre


@dataclass
class _PairState:
    x: float = 0.0
    y: float = 0.0
    hold_until: float = 0.0


class StickFilter:
    def __init__(self, cfg: StickFilterConfig, *, active_threshold: float = 0.08) -> None:
        self._cfg = cfg
        self._active_threshold = active_threshold
        self._left = _PairState()
        self._right = _PairState()

    def reset(self) -> None:
        self._left = _PairState()
        self._right = _PairState()

    def filter(self, lx: float, ly: float, rx: float, ry: float) -> tuple[float, float, float, float]:
        now = time.monotonic()
        lx, ly = self._filter_pair(lx, ly, self._left, now)
        rx, ry = self._filter_pair(rx, ry, self._right, now)
        return lx, ly, rx, ry

    def _filter_pair(self, x: float, y: float, state: _PairState, now: float) -> tuple[float, float]:
        raw_mag = math.hypot(x, y)
        alpha = max(0.0, min(1.0, self._cfg.smoothing_alpha))

        if raw_mag >= self._active_threshold:
            if alpha > 0.0:
                state.x = alpha * x + (1.0 - alpha) * state.x
                state.y = alpha * y + (1.0 - alpha) * state.y
            else:
                state.x, state.y = x, y
            state.hold_until = now + self._cfg.stick_hold_ms / 1000.0
            return state.x, state.y

        if now < state.hold_until and math.hypot(state.x, state.y) >= self._active_threshold:
            return state.x, state.y

        if alpha > 0.0:
            decay = max(0.0, min(1.0, self._cfg.release_alpha))
            state.x *= 1.0 - decay
            state.y *= 1.0 - decay
            if math.hypot(state.x, state.y) < 0.02:
                state.x = state.y = 0.0
        else:
            state.x = state.y = 0.0
        state.hold_until = 0.0
        return state.x, state.y
