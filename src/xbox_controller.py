"""
xbox_controller.py
------------------
Reads Xbox / gamepad input on the Pi and maps sticks/buttons to sub actuators.
Mapping is defined in config/xbox_mapping.yaml (edit without touching code).

Uses SDL2 GameController when available. Runs as a background thread.
"""

from __future__ import annotations

import os
import sys
import threading
import time
from pathlib import Path
from typing import Any

import yaml

from src.sub_state import SubActuators, XboxState, get_sub_state
from src.xbox_mapping import load_mapping_config, map_xbox_ballast, map_xbox_to_actuators
from src.xbox_stick_filter import StickFilter, StickFilterConfig

CONFIG_PATH = Path(__file__).parent.parent / "config" / "hardware.yaml"
DEADZONE = 0.18
SCAN_INTERVAL_S = 2.0
TRIGGER_DEADZONE = 0.08


def _headless_sdl_env() -> None:
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
        "enabled": bool(xbox.get("enabled", True)),
        "deadzone": float(xbox.get("deadzone", DEADZONE)),
        "trigger_deadzone": float(xbox.get("trigger_deadzone", TRIGGER_DEADZONE)),
        "device_index": xbox.get("device_index"),
        "poll_hz": float(xbox.get("poll_hz", 30.0)),
        "scan_interval_s": float(xbox.get("scan_interval_s", SCAN_INTERVAL_S)),
        "mapping_file": xbox.get("mapping_file"),
        "smoothing_alpha": float(xbox.get("smoothing_alpha", 0.0)),
        "stick_hold_ms": float(xbox.get("stick_hold_ms", 120.0)),
        "release_alpha": float(xbox.get("release_alpha", 0.55)),
    }


def is_xbox_enabled() -> bool:
    """True when gamepad polling is enabled in config/hardware.yaml."""
    return bool(_load_xbox_config()["enabled"])


def list_gamepads() -> list[dict[str, Any]]:
    """
    Enumerate connected gamepads without starting the poll thread.
    Run: python -m src.xbox_controller
    """
    _headless_sdl_env()
    try:
        import pygame
        from pygame._sdl2 import controller as gc
    except ImportError:
        return []

    pygame.init()
    pygame.joystick.init()
    gc.init()

    pads: list[dict[str, Any]] = []
    for i in range(pygame.joystick.get_count()):
        try:
            name = pygame.joystick.Joystick(i).get_name()
        except Exception:
            name = f"joystick {i}"
        pads.append({
            "index": i,
            "name": name,
            "is_xbox": _is_xbox_name(name),
            "is_gamecontroller": _is_sdl_controller(gc, i),
        })
    return pads


def _read_pad_buttons(joy) -> dict[str, bool]:
    """Named Xbox buttons + d-pad hat for mapping layer."""
    buttons: dict[str, bool] = {}
    for i in range(joy.get_numbuttons()):
        buttons[str(i)] = bool(joy.get_button(i))
    for i, name in ((0, "a"), (1, "b"), (2, "x"), (3, "y"), (4, "lb"), (5, "rb")):
        buttons[name] = buttons.get(str(i), False)
    if joy.get_numhats() > 0:
        hx, hy = joy.get_hat(0)
        buttons["dpad_up"] = hy > 0
        buttons["dpad_down"] = hy < 0
        buttons["dpad_left"] = hx < 0
        buttons["dpad_right"] = hx > 0
    return buttons


def _is_xbox_name(name: str) -> bool:
    n = name.lower()
    return any(k in n for k in ("xbox", "microsoft", "360", "one", "series"))


def _is_sdl_controller(gc, index: int) -> bool:
    """pygame 2.6+ uses is_controller; older builds used is_gamecontroller."""
    fn = getattr(gc, "is_controller", None) or getattr(gc, "is_gamecontroller", None)
    if fn is None:
        return False
    try:
        return bool(fn(index))
    except Exception:
        return False


def _norm_axis(v: float) -> float:
    if -1.1 <= v <= 1.1:
        return max(-1.0, min(1.0, float(v)))
    return max(-1.0, min(1.0, v / 32767.0))


def _norm_trigger(v: float) -> float:
    if 0.0 <= v <= 1.1:
        return max(0.0, min(1.0, float(v)))
    return max(0.0, min(1.0, (v + 32767.0) / (2.0 * 32767.0)))


class _GamepadBackend:
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

        if _is_sdl_controller(gc, idx):
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
        # pygame 2.6+: ControllerAxis removed; get_axis() returns raw int16.
        # Reading via as_joystick() matches the Linux Xbox mapping we already handle.
        try:
            return self._read_joystick(pad.as_joystick())
        except Exception:
            pass

        def axis(i: int) -> float:
            try:
                return _norm_axis(float(pad.get_axis(i)))
            except Exception:
                return 0.0

        # SDL_GameControllerAxis: 0=LX 1=LY 2=RX 3=RY 4=LT 5=RT
        lt = _norm_trigger(axis(4))
        rt = _norm_trigger(axis(5))

        return XboxState(
            connected=True,
            name=self._name,
            left_stick_x=axis(0),
            left_stick_y=axis(1),
            right_stick_x=axis(2),
            right_stick_y=axis(3),
            triggers={"lt": lt, "rt": rt},
            buttons={},
            last_update=time.time(),
        )

    def _read_joystick(self, joy) -> XboxState:
        def axis(i: int) -> float:
            try:
                return _norm_axis(float(joy.get_axis(i)))
            except Exception:
                return 0.0

        n = joy.get_numaxes()
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

        return XboxState(
            connected=True,
            name=self._name,
            left_stick_x=axis(0),
            left_stick_y=axis(1),
            right_stick_x=rs_x,
            right_stick_y=rs_y,
            triggers={"lt": lt, "rt": rt},
            buttons=_read_pad_buttons(joy),
            last_update=time.time(),
        )

    def close(self) -> None:
        self._controller = None
        self._joystick = None
        self._use_controller_api = False
        self._name = ""


class XboxController:
    def __init__(self, poll_hz: float | None = None, device_index: int | None = None) -> None:
        cfg = _load_xbox_config()
        self.deadzone = cfg["deadzone"]
        self.trigger_deadzone = cfg["trigger_deadzone"]
        self.poll_interval = 1.0 / (poll_hz or cfg["poll_hz"])
        self.scan_interval = float(cfg["scan_interval_s"])
        self._device_index = device_index if device_index is not None else cfg["device_index"]
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._state = get_sub_state()
        self._pad = _GamepadBackend(self._device_index)
        self._last_scan = 0.0
        self._mapping = load_mapping_config()
        self._stick_filter = StickFilter(
            StickFilterConfig(
                smoothing_alpha=cfg.get("smoothing_alpha", 0.0),
                stick_hold_ms=cfg.get("stick_hold_ms", 120.0),
                release_alpha=cfg.get("release_alpha", 0.55),
            ),
            active_threshold=max(0.05, self.deadzone * 0.45),
        )
        self._ballast_active = False

    def _apply_stick_filter(self, xbox: XboxState) -> XboxState:
        lx, ly, rx, ry = self._stick_filter.filter(
            xbox.left_stick_x,
            xbox.left_stick_y,
            xbox.right_stick_x,
            xbox.right_stick_y,
        )
        return XboxState(
            connected=xbox.connected,
            name=xbox.name,
            left_stick_x=lx,
            left_stick_y=ly,
            right_stick_x=rx,
            right_stick_y=ry,
            triggers=dict(xbox.triggers),
            buttons=dict(xbox.buttons),
            last_update=xbox.last_update,
        )

    @property
    def connected(self) -> bool:
        return self._pad.connected

    def connect(self) -> bool:
        """Try to open the gamepad once. Returns True if a pad was found."""
        return self._try_connect()

    def _try_connect(self) -> bool:
        ok = self._pad.open()
        if ok:
            print(f"[xbox] Connected: {self._pad.name}")
        return ok

    def _on_disconnect(self) -> None:
        self._stick_filter.reset()
        self._ballast_active = False
        self._state.update_xbox(XboxState(connected=False))
        self._state.set_xbox_actuators(SubActuators())
        self._state.recompute_effective()

    def _drain_hotplug_events(self) -> bool:
        """Return True if a connect/disconnect event was handled."""
        if self._pad._pygame is None:
            return False

        pygame = self._pad._pygame
        changed = False
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                changed = True
                if not self._pad.connected:
                    self._try_connect()
            elif event.type == pygame.JOYDEVICEREMOVED:
                if self._pad.connected:
                    print("[xbox] Controller disconnected")
                    self._pad.close()
                    self._on_disconnect()
                changed = True
        return changed

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            self._drain_hotplug_events()

            now = time.monotonic()
            if not self._pad.connected:
                if now - self._last_scan >= self.scan_interval:
                    self._last_scan = now
                    if not self._try_connect():
                        self._on_disconnect()
                        time.sleep(self.scan_interval)
                        continue

            try:
                xbox = self._pad.read()
            except Exception as exc:
                print(f"[xbox] Read error: {exc}")
                self._pad.close()
                self._on_disconnect()
                time.sleep(1.0)
                continue

            if not xbox.connected:
                self._pad.close()
                self._on_disconnect()
                time.sleep(1.0)
                continue

            self._state.update_xbox(xbox)
            filtered = self._apply_stick_filter(xbox)
            actuators = map_xbox_to_actuators(filtered, self.deadzone, self._mapping)
            self._state.set_xbox_actuators(actuators)

            if self._state.get_control_mode() == "xbox":
                fore, aft = map_xbox_ballast(
                    filtered, self._mapping, trigger_deadzone=self.trigger_deadzone
                )
                if fore != 0.0 or aft != 0.0:
                    self._state.set_ballast_commands(fore, aft)
                    self._ballast_active = True
                elif self._ballast_active:
                    self._state.set_ballast_commands(0.0, 0.0)
                    self._ballast_active = False
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


def connect_xbox(*, autostart: bool = True) -> XboxController:
    """
    Simple entry point: start background polling and auto-connect when a pad appears.
    Used by sub_server.py and inference.py --sub (respects xbox.enabled in config).
    """
    ctrl = get_xbox_controller(autostart=autostart)
    if autostart and not (ctrl._thread and ctrl._thread.is_alive()):
        ctrl.start()
    return ctrl


def _cli_main() -> int:
    """Built-in connection check — no separate script required."""
    print("[xbox] Scanning for gamepads …")
    pads = list_gamepads()
    if not pads:
        print("[xbox] No gamepads found.")
        print("       Plug in via USB or pair over Bluetooth, then run again.")
        print("       Linux: ensure your user can read /dev/input/* (input group).")
        return 1

    for pad in pads:
        tags = []
        if pad["is_xbox"]:
            tags.append("Xbox")
        if pad["is_gamecontroller"]:
            tags.append("GameController API")
        suffix = f" ({', '.join(tags)})" if tags else ""
        print(f"  [{pad['index']}] {pad['name']}{suffix}")

    if not is_xbox_enabled():
        print("[xbox] Note: xbox.enabled is false in config/hardware.yaml")

    ctrl = connect_xbox(autostart=True)
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and not ctrl.connected:
        time.sleep(0.2)

    if not ctrl.connected:
        print("[xbox] Pad(s) listed but could not open — try power-cycling the controller.")
        return 1

    print(f"[xbox] Connected: {ctrl._pad.name}")
    print("[xbox] Mapping: config/xbox_mapping.yaml")
    print("[xbox] Move sticks / press buttons (Ctrl+C to quit):")
    try:
        while True:
            snap = get_sub_state().control_snapshot()
            xbox = snap["xbox"]
            mapped = snap["xbox_mapped"]
            ls = xbox["sticks"]["left"]
            rs = xbox["sticks"]["right"]
            tr = xbox["triggers"]
            btns = xbox.get("buttons") or {}
            pressed = [k for k in ("a", "b", "lb", "dpad_up", "dpad_down") if btns.get(k)]
            line = (
                f"  LS ({ls['x']:+.2f},{ls['y']:+.2f})  RS ({rs['x']:+.2f},{rs['y']:+.2f})  "
                f"LT {tr.get('lt', 0):.2f}  thr {mapped['thrusterX']:+.2f}  "
                f"fins L{mapped['finLeft']:+.2f}/R{mapped['finRight']:+.2f}  "
                f"B fore{snap['ballast_fore_command']:+.2f}/aft{snap['ballast_aft_command']:+.2f}"
            )
            if pressed:
                line += "  [" + ",".join(pressed) + "]"
            print("\r" + line + " " * 4, end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        ctrl.stop()
    return 0


if __name__ == "__main__":
    sys.exit(_cli_main())
