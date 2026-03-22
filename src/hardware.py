"""
hardware.py
-----------
Sends ControlOutput (steering, drive, camera tilt) to physical hardware.

Supports:
  - stub:   no hardware; values are logged only
  - pca9685: Adafruit PCA9685 I2C PWM board (3 channels: steer, drive, tilt)
  - serial:  UART to a microcontroller (e.g. Arduino) that drives the servos/motor

All three outputs are always applied: steering servo, drive motor, camera tilt servo.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any

import yaml

from src.controller import ControlOutput


CONFIG_PATH = Path(__file__).parent.parent / "config" / "hardware.yaml"


def _load_config(path: Path = CONFIG_PATH) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _value_to_pulse_us(
    value: float,
    centre: int,
    min_pwm: int,
    max_pwm: int,
    inverted: bool,
) -> int:
    """
    Map normalised value in [-1, 1] to PWM pulse width in microseconds.
    value=0 -> centre, value=1 -> max_pwm, value=-1 -> min_pwm.
    """
    if inverted:
        value = -value
    half = (max_pwm - min_pwm) / 2.0
    pulse = centre + value * half
    return int(max(min_pwm, min(max_pwm, round(pulse))))


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
            # Log only when values change to avoid flooding
            # (inference.py already prints the full table; optional extra log here)
            pass


class PCA9685Output(HardwareOutput):
    """Drive steering servo, drive motor (ESC), and camera tilt servo via PCA9685."""

    def __init__(self, config: dict) -> None:
        self._config = config
        self._pca: Any = None
        self._channels: dict[str, dict] = {
            "steering_servo": config["steering_servo"],
            "drive_motor": config["drive_motor"],
            "camera_tilt_servo": config["camera_tilt_servo"],
        }
        try:
            import board
            import busio
            from adafruit_pca9685 import PCA9685
            i2c = busio.I2C(board.SCL, board.SDA)
            pca_cfg = config.get("pca9685", {})
            addr = pca_cfg.get("i2c_address", 0x40)
            self._pca = PCA9685(i2c, address=addr)
            self._pca.frequency = pca_cfg.get("pwm_frequency", 50)
            self._pca_ref = self._pca
        except ImportError as e:
            raise ImportError(
                "PCA9685 support requires adafruit-circuitpython-pca9685 and adafruit-blinka. "
                "Install with: pip install adafruit-circuitpython-pca9685 adafruit-blinka"
            ) from e

    def _set_channel_pwm_us(self, channel: int, pulse_us: int) -> None:
        """Set a PCA9685 channel to a pulse width in microseconds (50 Hz assumed)."""
        # 50 Hz -> period 20 ms = 20000 us. duty_cycle 0-65535 = pulse_us/20000 * 65535
        duty = int(pulse_us / 20000.0 * 65535)
        duty = max(0, min(65535, duty))
        self._pca.channels[channel].duty_cycle = duty

    def apply(self, output: ControlOutput) -> None:
        if self._pca is None:
            return
        values = [
            (output.steering_servo, self._channels["steering_servo"]),
            (output.drive_motor, self._channels["drive_motor"]),
            (output.camera_tilt_servo, self._channels["camera_tilt_servo"]),
        ]
        for val, ch_cfg in values:
            pulse = _value_to_pulse_us(
                val,
                ch_cfg["centre_pwm"],
                ch_cfg["min_pwm"],
                ch_cfg["max_pwm"],
                ch_cfg.get("inverted", False),
            )
            self._set_channel_pwm_us(ch_cfg["channel"], pulse)

    def close(self) -> None:
        if getattr(self, "_pca_ref", None) is not None:
            try:
                self._pca_ref.deinit()
            except Exception:
                pass


class SerialOutput(HardwareOutput):
    """
    Send steering, drive, and tilt as text over serial for a microcontroller.

    Protocol: one line per update: "S <steer> D <drive> T <tilt>\n"
    Values are -1.0 to 1.0. The MCU converts to PWM.
    """

    def __init__(self, config: dict, port_override: str | None = None) -> None:
        self._config = config
        ser_cfg = config.get("serial", {})
        self._port = port_override or ser_cfg.get("port", "/dev/ttyUSB0")
        self._baud = ser_cfg.get("baud_rate", 115200)
        self._verbose = ser_cfg.get("verbose", True)
        self._serial = None

    def _open(self) -> None:
        if self._serial is not None:
            return
        try:
            import serial
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                timeout=0.01,
                write_timeout=0.05,
            )
        except ImportError:
            raise ImportError(
                "Serial output requires pyserial. Install with: pip install pyserial"
            ) from None
        except Exception as e:
            raise RuntimeError(f"Cannot open serial port {self._port}: {e}") from e

    def apply(self, output: ControlOutput) -> None:
        self._open()
        line = (
            f"S {output.steering_servo:.3f} "
            f"D {output.drive_motor:.3f} "
            f"T {output.camera_tilt_servo:.3f}\n"
        )
        if self._verbose:
            print(line, end="", flush=True)
        try:
            self._serial.write(line.encode("ascii"))
            self._serial.flush()
        except Exception as e:
            print(f"[hardware] Serial write failed: {e}")

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None


def from_config(
    config_path: Path = CONFIG_PATH,
    serial_port_override: str | None = None,
) -> HardwareOutput:
    """
    Build the configured hardware output (stub, pca9685, or serial).
    All three outputs (steering, drive, camera tilt) are sent by the returned instance.
    """
    config = _load_config(config_path)
    interface = (config.get("interface") or "stub").strip().lower()

    if interface == "stub":
        return StubOutput(config)
    if interface == "pca9685":
        return PCA9685Output(config)
    if interface == "serial":
        return SerialOutput(config, port_override=serial_port_override)
    raise ValueError(
        f"Unknown hardware interface '{interface}'. "
        "Use one of: stub, pca9685, serial (in config/hardware.yaml)."
    )
