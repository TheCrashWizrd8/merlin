"""
hardware.py
-----------
Sends ControlOutput (steering, drive, camera tilt) to physical hardware.

Supports:
  - stub: no hardware; values are logged only
  - sub:   route YOLO output into sub_state (ESP bridge sends S2/B to ESP32)
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path

import yaml

from src.controller import ControlOutput


CONFIG_PATH = Path(__file__).parent.parent / "config" / "hardware.yaml"


def _load_config(path: Path = CONFIG_PATH) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


class HardwareOutput(ABC):
    """Base for sending control values to hardware."""

    @abstractmethod
    def apply(self, output: ControlOutput) -> None:
        """Send steering, drive, and camera tilt to hardware."""
        pass

    def close(self) -> None:
        """Release resources (optional)."""
        pass


class StubOutput(HardwareOutput):
    """No hardware; only logs values (steering, drive, tilt)."""

    def __init__(self, config: dict) -> None:
        self._config = config
        self._last: dict[str, float] | None = None

    def apply(self, output: ControlOutput) -> None:
        current = {
            "steer": output.steering_servo,
            "drive": output.drive_motor,
            "tilt": output.camera_tilt_servo,
        }
        if current != self._last:
            self._last = current.copy()


class SubBridgeOutput(HardwareOutput):
    """
    Route ControlOutput to the sub vehicle stack via esp_bridge (no second serial port).

    Maps YOLO steering/drive/tilt → sub actuators (S2 … X …) in sub_state.
    The ESP bridge thread reads telemetry (TEL …) and writes S2/B commands.
    """

    def __init__(self, config: dict) -> None:
        self._config = config

    def apply(self, output: ControlOutput) -> None:
        from src.control_source import get_mode as yolo_mode, get_manual
        from src.sub_control import yolo_to_sub_motion
        from src.sub_state import get_sub_state
        from src.telemetry_context import TelemetryContext

        state = get_sub_state()
        mode = state.get_control_mode()
        telemetry = TelemetryContext.from_sub_state()

        if mode == "auto":
            motion = yolo_to_sub_motion(output, telemetry=telemetry)
            state.set_auto_actuators(motion.actuators)
            state.set_ballast_commands(motion.ballast_fore, motion.ballast_aft)
        elif mode == "manual" and yolo_mode() == "manual":
            manual = get_manual()
            motion = yolo_to_sub_motion(
                ControlOutput(
                    steering_servo=manual.s,
                    drive_motor=manual.d,
                    camera_tilt_servo=manual.t,
                    apple_detected=False,
                    target_x=0,
                    target_y=0,
                    error_x=0.0,
                    error_y=0.0,
                    confidence=0.0,
                    timestamp=output.timestamp,
                ),
                telemetry=telemetry,
            )
            state.set_manual_actuators(motion.actuators)
        # xbox mode: xbox_controller thread updates xbox_actuators; manual sliders unchanged
        state.recompute_effective()

    def close(self) -> None:
        pass


def from_config(config_path: Path = CONFIG_PATH) -> HardwareOutput:
    """
    Build the configured hardware output (stub or sub).
    """
    config = _load_config(config_path)
    interface = (config.get("interface") or "stub").strip().lower()

    if interface == "stub":
        return StubOutput(config)
    if interface == "sub":
        return SubBridgeOutput(config)
    raise ValueError(
        f"Unknown hardware interface '{interface}'. "
        "Use one of: stub, sub (in config/hardware.yaml)."
    )
