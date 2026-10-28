"""A connected Xbox pad is the default control source and recovers fast from glitches."""

import pytest

from src.sub_state import get_sub_state
from src.xbox_controller import XboxController


class _FakePad:
    name = "Fake Pad"
    _pygame = None
    _joystick = None
    _controller = None

    def __init__(self, opens: bool = True, read_error: Exception | None = None) -> None:
        self.opens = opens
        self.read_error = read_error
        self.connected = False

    def open(self) -> bool:
        self.connected = self.opens
        return self.opens

    def close(self) -> None:
        self.connected = False

    def read(self):
        raise self.read_error


@pytest.fixture(autouse=True)
def _reset_state():
    state = get_sub_state()
    state.set_servo_cal_active(False)
    state.set_control_mode("manual")
    yield state
    state.set_servo_cal_active(False)
    state.set_control_mode("manual")


def _controller(**pad_kwargs) -> XboxController:
    ctrl = XboxController()
    ctrl._pad = _FakePad(**pad_kwargs)
    return ctrl


def test_connect_takes_over_from_manual(_reset_state):
    ctrl = _controller()
    assert ctrl.connect() is True
    assert _reset_state.get_control_mode() == "xbox"


def test_connect_leaves_yolo_auto_mode_alone(_reset_state):
    _reset_state.set_control_mode("auto")
    _controller().connect()
    assert _reset_state.get_control_mode() == "auto"


def test_connect_leaves_servo_calibration_alone(_reset_state):
    _reset_state.set_servo_cal_active(True)
    _controller().connect()
    assert _reset_state.get_control_mode() == "manual"


def test_failed_connect_changes_nothing(_reset_state):
    assert _controller(opens=False).connect() is False
    assert _reset_state.get_control_mode() == "manual"


def test_reconnect_after_dropout_returns_to_xbox(_reset_state):
    ctrl = _controller()
    ctrl.connect()
    ctrl._on_disconnect()
    assert _reset_state.get_control_mode() == "manual"  # fail-safe while the pad is gone
    ctrl._try_connect()
    assert _reset_state.get_control_mode() == "xbox"


def test_read_error_fails_safe_and_rescans_at_once(_reset_state):
    ctrl = _controller(read_error=RuntimeError("bluetooth glitch"))
    ctrl.connect()
    ctrl._last_scan = 12345.0
    ctrl._poll_once()
    assert _reset_state.get_control_mode() == "manual"
    assert _reset_state.xbox_actuators.thruster_x == 0.0
    assert ctrl._last_scan == 0.0  # next poll retries immediately, not after scan_interval
