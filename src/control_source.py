"""
control_source.py
-----------------
Abstraction for where S/D/T commands come from.
Supports: auto (inference), manual (web sliders), future: RC controller.

Use get_current_sdt() to decide what to send to hardware.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Callable, Optional

# Module-level state (thread-safe for web + inference)
_lock = threading.Lock()
_mode: str = "auto"
_manual_s: float = 0.0
_manual_d: float = 0.0
_manual_t: float = 0.0
# Optional RC source (callable returning (s, d, t) or None)
_rc_source: Optional[Callable[[], tuple[float, float, float] | None]] = None


@dataclass
class SDT:
    """Steer, drive, tilt tuple."""
    s: float
    d: float
    t: float


def set_mode(mode: str) -> None:
    """Set control mode: 'auto' or 'manual'."""
    global _mode
    with _lock:
        _mode = "manual" if mode == "manual" else "auto"


def get_mode() -> str:
    with _lock:
        return _mode


def set_manual(s: float, d: float, t: float) -> None:
    """Update manual control values (clamped -1..1)."""
    global _manual_s, _manual_d, _manual_t
    s = max(-1.0, min(1.0, float(s)))
    d = max(-1.0, min(1.0, float(d)))
    t = max(-1.0, min(1.0, float(t)))
    with _lock:
        _manual_s, _manual_d, _manual_t = s, d, t


def get_manual() -> SDT:
    with _lock:
        return SDT(_manual_s, _manual_d, _manual_t)


def set_rc_source(callback: Callable[[], tuple[float, float, float] | None] | None) -> None:
    """Register RC controller source. Callback returns (s,d,t) or None when disconnected."""
    global _rc_source
    with _lock:
        _rc_source = callback


def get_current_sdt(
    auto_sdt: SDT,
) -> SDT:
    """
    Return the S/D/T to send to hardware based on mode and sources.
    When mode is 'auto', returns auto_sdt (from inference).
    When mode is 'manual', returns manual values from web (or RC if registered and active).
    """
    with _lock:
        if _mode == "auto":
            return auto_sdt
        if _rc_source is not None:
            rc = _rc_source()
            if rc is not None:
                return SDT(rc[0], rc[1], rc[2])
        return SDT(_manual_s, _manual_d, _manual_t)
