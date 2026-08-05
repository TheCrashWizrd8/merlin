"""
xbox_controller.py
------------------
Reads Xbox / gamepad input on the Pi and maps sticks/triggers to sub actuators.
Uses SDL2 GameController when available (correct Xbox Bluetooth mapping on Linux).
Falls back to raw joystick axes. Runs as a background thread.

Mapping (default):
  Left stick X  → aft_steer_y
  Left stick Y  → aft_steer_z (inverted)
  Right stick Y → thruster_x (inverted: up = forward)
  Right stick X → fin_left / fin_right (differential)
  LT / RT       → ballast drain / fill bias
"""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import Any

import yaml

from src.sub_state import SubActuators, XboxState, get_sub_state

CONFIG_PATH = Path(__file__).parent.parent / "config" / "hardware.yaml"
DEADZONE = 0.12
SCAN_INTERVAL_S = 2.0


def _headless_sdl_env() -> None:
    """Allow pygame/SDL on Pi without audio card or X11 display."""
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")


def _load_xbox_config() -> dict[str, Any]:
    try:
        with open(CONFIG_PATH) as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        cfg = {}
    xbox = cfg.get("xbox") or {}
    return {
        "deadzone": float(xbox.get("deadzone", DEADZONE)),
        "device_index": xbox.get("device_index"),  # None = first Xbox / gamepad found
        "poll_hz": float(xbox.get("poll_hz", 30.0)),
    }


def _apply_deadzone(v: float, dz: float = DEADZONE) -> float:
    if abs(v) < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


def _norm_axis(v: float) -> float:
    """Normalize SDL axis (-32768..32767 or -1..1) to -1..1."""
    if -1.1 <= v <= 1.1:
        return max(-1.0, min(1.0, float(v)))
    return max(-1.0, min(1.0, v / 32767.0))


def _norm_trigger(v: float) -> float:
    if 0.0 <= v <= 1.1:
        return max(0.0, min(1.0, float(v)))
    return max(0.0, min(1.0, (v + 32767.0) / (2.0 * 32767.0)))


def _is_xbox_name(name: str) -> bool:
    n = name.lower()
    return any(k in n for k in ("xbox", "microsoft", "360", "one", "series"))


def map_xbox_to_actuators(xbox: XboxState, deadzone: float = DEADZONE) -> SubActuators:
    """Convert Xbox stick/trigger state to sub actuator commands."""
    ls_x = _apply_deadzone(xbox.left_stick_x, deadzone)
    ls_y = _apply_deadzone(xbox.left_stick_y, deadzone)
    rs_x = _apply_deadzone(xbox.right_stick_x, deadzone)
    rs_y = _apply_deadzone(xbox.right_stick_y, deadzone)

    thruster = -rs_y
    fin_diff = rs_x
    fin_left = max(-1.0, min(1.0, -fin_diff))
    fin_right = max(-1.0, min(1.0, fin_diff))

    return SubActuators(
        aft_steer_y=ls_x,
        aft_steer_z=-ls_y,
        thruster_x=thruster,
        fin_left=fin_left,
        fin_right=fin_right,
    )


def map_xbox_ballast(xbox: XboxState) -> float:
    """LT drains (-1), RT fills (+1)."""
    lt = xbox.triggers.get("lt", 0.0)
    rt = xbox.triggers.get("rt", 0.0)
    if lt > 0.05 and rt > 0.05:
        return 0.0
    if lt > rt:
        return -lt
    if rt > lt:
        return rt
    return 0.0


class _GamepadBackend:
    """SDL2 GameController (preferred) or raw Joystick fallback."""

    def __init__(self, device_index: int | None) -> None:
        self._device_index = device_index
        self._pygame = None
        self._controller = None
        self._joystick = None
        self._use_controller_api = False
        self._name = ""

    @property
    def name(self) -> str:
        return self._name

    @property
    def connected(self) -> bool:
        return self._controller is not None or self._joystick is not None

    def open(self) -> bool:
        self.close()
        _headless_sdl_env()
        try:
            import pygame
            from pygame._sdl2 import controller as gc
        except ImportError:
            return False

        if self._pygame is None:
            pygame.init()
            pygame.joystick.init()
            gc.init()
            self._pygame = pygame

        idx = self._pick_device_index(pygame, gc)
        if idx is None:
            return False

        if gc.is_gamecontroller(idx):
            try:
                pad = gc.Controller(idx)
                self._controller = pad
                self._use_controller_api = True
                self._name = pad.name
                return True
            except Exception:
                pass

        try:
            joy = pygame.joystick.Joystick(idx)
            joy.init()
            self._joystick = joy
            self._use_controller_api = False
            self._name = joy.get_name()
            return True
        except Exception:
            return False

    def _pick_device_index(self, pygame, gc) -> int | None:
        if self._device_index is not None:
            if self._device_index < pygame.joystick.get_count():
                return int(self._device_index)
            return None

        count = pygame.joystick.get_count()
        xbox_idx = None
        fallback = None
        for i in range(count):
            try:
                name = pygame.joystick.Joystick(i).get_name()
            except Exception:
                continue
            if fallback is None:
                fallback = i
            if _is_xbox_name(name):
                xbox_idx = i
                break
        return xbox_idx if xbox_idx is not None else fallback

    def read(self) -> XboxState:
        if not self.connected:
            return XboxState(connected=False)

        pygame = self._pygame
        pygame.event.pump()

        if self._use_controller_api and self._controller is not None:
            return self._read_controller(self._controller)
        if self._joystick is not None:
            return self._read_joystick(self._joystick)
        return XboxState(connected=False)

    def _read_controller(self, pad) -> XboxState:
        from pygame._sdl2.controller import ControllerAxis

        def axis(a) -> float:
            try:
                return _norm_axis(float(pad.axis(a)))
            except Exception:
                return 0.0

        lt = _norm_trigger(axis(ControllerAxis.TRIGGERLEFT))
        rt = _norm_trigger(axis(ControllerAxis.TRIGGERRIGHT))

        buttons = {}
        for i in range(16):
            try:
                buttons[f"b{i}"] = bool(pad.button(i))
            except Exception:
                break

        return XboxState(
            connected=True,
            name=self._name,
            left_stick_x=axis(ControllerAxis.LEFTX),
            left_stick_y=axis(ControllerAxis.LEFTY),
            right_stick_x=axis(ControllerAxis.RIGHTX),
            right_stick_y=axis(ControllerAxis.RIGHTY),
            triggers={"lt": lt, "rt": rt},
            buttons=buttons,
            last_update=time.time(),
        )

    def _read_joystick(self, joy) -> XboxState:
        def axis(i: int) -> float:
            try:
                return _norm_axis(float(joy.get_axis(i)))
            except Exception:
                return 0.0

        n = joy.get_numaxes()
        # USB Xbox: 0,1 LS | 2,5 triggers | 3,4 RS
        # BT Xbox (xpadneo): often 0,1 LS | 3,4 RS | 2,5 or 4,5 triggers
        if n >= 6:
            lt = _norm_trigger((joy.get_axis(2) + 1.0) / 2.0)
            rt = _norm_trigger((joy.get_axis(5) + 1.0) / 2.0)
            rs_x, rs_y = axis(3), axis(4)
        elif n >= 4:
            lt = rt = 0.0
            rs_x, rs_y = axis(2), axis(3)
        else:
            lt = rt = 0.0
            rs_x = rs_y = 0.0

        buttons = {}
        for i in range(min(joy.get_numbuttons(), 12)):
            try:
                buttons[f"b{i}"] = bool(joy.get_button(i))
            except Exception:
                pass

        return XboxState(
            connected=True,
            name=self._name,
            left_stick_x=axis(0),
            left_stick_y=axis(1),
            right_stick_x=rs_x,
            right_stick_y=rs_y,
            triggers={"lt": lt, "rt": rt},
            buttons=buttons,
            last_update=time.time(),
        )

    def close(self) -> None:
        self._controller = None
        self._joystick = None
        self._use_controller_api = False
        self._name = ""


class XboxController:
    """Poll gamepad in a background thread (USB or Bluetooth Xbox)."""

    def __init__(self, poll_hz: float | None = None, device_index: int | None = None) -> None:
        cfg = _load_xbox_config()
        self.deadzone = cfg["deadzone"]
        self.poll_interval = 1.0 / (poll_hz or cfg["poll_hz"])
        self._device_index = device_index if device_index is not None else cfg["device_index"]
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._state = get_sub_state()
        self._pad = _GamepadBackend(self._device_index)
        self._last_scan = 0.0

    def _try_connect(self) -> bool:
        ok = self._pad.open()
        if ok:
            print(f"[xbox] Connected: {self._pad.name}")
        return ok

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            now = time.monotonic()
            if not self._pad.connected:
                if now - self._last_scan >= SCAN_INTERVAL_S:
                    self._last_scan = now
                    if not self._try_connect():
                        self._state.update_xbox(XboxState(connected=False))
                        time.sleep(SCAN_INTERVAL_S)
                        continue

            try:
                xbox = self._pad.read()
            except Exception as exc:
                print(f"[xbox] Read error: {exc}")
                self._pad.close()
                self._state.update_xbox(XboxState(connected=False))
                time.sleep(1.0)
                continue

            if not xbox.connected:
                self._pad.close()
                self._state.update_xbox(XboxState(connected=False))
                time.sleep(1.0)
                continue

            self._state.update_xbox(xbox)
            actuators = map_xbox_to_actuators(xbox, self.deadzone)
            self._state.set_xbox_actuators(actuators)

            if self._state.get_control_mode() == "xbox":
                ballast = map_xbox_ballast(xbox)
                self._state.set_ballast_command(ballast if abs(ballast) > 0.05 else 0.0)
                self._state.recompute_effective()

            time.sleep(self.poll_interval)

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        _headless_sdl_env()
        self._thread = threading.Thread(target=self._poll_loop, daemon=True, name="xbox")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._pad.close()
        if self._pad._pygame is not None:
            try:
                self._pad._pygame.joystick.quit()
                self._pad._pygame.quit()
            except Exception:
                pass
            self._pad._pygame = None


_xbox: XboxController | None = None
_xbox_lock = threading.Lock()


def get_xbox_controller(autostart: bool = False) -> XboxController:
    global _xbox
    with _xbox_lock:
        if _xbox is None:
            _xbox = XboxController()
            if autostart:
                _xbox.start()
        return _xbox
