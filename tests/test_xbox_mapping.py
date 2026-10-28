"""Tests for config/xbox_mapping.yaml → actuator mapping."""

from src.sub_state import XboxState
from src.xbox_controller import scale_trigger_axis, sdl_trigger_axis
from src.xbox_mapping import (
    check_emergency_stop,
    linked_flap_default_enabled,
    load_mapping_config,
    map_xbox_ballast,
    map_xbox_to_actuators,
    update_linked_flap_toggle,
    xbox_input_active,
)


def _xbox(**kwargs) -> XboxState:
    base = dict(
        connected=True,
        left_stick_x=0.0,
        left_stick_y=0.0,
        right_stick_x=0.0,
        right_stick_y=0.0,
        triggers={"lt": 0.0, "rt": 0.0},
        buttons={},
    )
    base.update(kwargs)
    return XboxState(**base)


def test_load_mapping_includes_new_sections():
    cfg = load_mapping_config()
    assert "flaps" in cfg
    assert "modes" in cfg
    assert cfg["thruster"].get("forward_trigger") == "rt"
    assert cfg["ballast"].get("up_button") == "rb"


def test_sdl_trigger_release_is_off_and_half_is_half():
    assert sdl_trigger_axis(0.0) == 0.0
    assert abs(sdl_trigger_axis(16384) - 0.5) < 0.02
    assert sdl_trigger_axis(32767) == 1.0
    assert sdl_trigger_axis(-32768) == 0.0


def test_trigger_resting_at_zero_is_off():
    pressed, rest = scale_trigger_axis(1.0, None)
    assert pressed == 1.0
    released, rest = scale_trigger_axis(0.0, rest)
    assert released == 0.0


def test_trigger_resting_at_minus_one_is_off():
    rest_v, rest = scale_trigger_axis(-1.0, None)
    assert rest_v == 0.0
    full, rest = scale_trigger_axis(1.0, rest)
    assert full == 1.0
    half, rest = scale_trigger_axis(0.0, rest)
    assert abs(half - 0.5) < 0.02


def test_rt_lt_thruster_mapping():
    xbox = _xbox(triggers={"lt": 0.0, "rt": 0.9})
    act = map_xbox_to_actuators(xbox, deadzone=0.05, trigger_deadzone=0.05)
    assert act.thruster_x > 0.8

    xbox = _xbox(triggers={"lt": 0.9, "rt": 0.0})
    act = map_xbox_to_actuators(xbox, deadzone=0.05, trigger_deadzone=0.05)
    assert act.thruster_x < -0.8

    xbox = _xbox(triggers={"lt": 1.0, "rt": 0.4})
    act = map_xbox_to_actuators(xbox, deadzone=0.05, trigger_deadzone=0.05)
    assert act.thruster_x > 0.2


def test_rb_lb_ballast_both_tanks():
    xbox = _xbox(buttons={"rb": True})
    fore, aft = map_xbox_ballast(xbox)
    assert fore == 1.0
    assert aft == 1.0

    xbox = _xbox(buttons={"lb": True})
    fore, aft = map_xbox_ballast(xbox)
    assert fore == -1.0
    assert aft == -1.0


def test_rb_lb_with_dpad_tank_select():
    xbox = _xbox(buttons={"rb": True, "dpad_up": True})
    fore, aft = map_xbox_ballast(xbox)
    assert fore == 1.0
    assert aft == 0.0

    xbox = _xbox(buttons={"lb": True, "dpad_down": True})
    fore, aft = map_xbox_ballast(xbox)
    assert fore == 0.0
    assert aft == -1.0


def test_right_stick_flaps_up_down():
    xbox = _xbox(right_stick_y=-1.0)
    act = map_xbox_to_actuators(xbox, deadzone=0.05, linked_flap_enabled=False)
    assert act.fin_left > 0.8
    assert act.fin_right > 0.8

    xbox = _xbox(right_stick_y=1.0)
    act = map_xbox_to_actuators(xbox, deadzone=0.05, linked_flap_enabled=False)
    assert act.fin_left < -0.8
    assert act.fin_right < -0.8


def test_right_stick_flaps_roll():
    xbox = _xbox(right_stick_x=-1.0)
    act = map_xbox_to_actuators(xbox, deadzone=0.05, linked_flap_enabled=False)
    assert act.fin_left > 0.8
    assert act.fin_right < -0.8

    xbox = _xbox(right_stick_x=1.0)
    act = map_xbox_to_actuators(xbox, deadzone=0.05, linked_flap_enabled=False)
    assert act.fin_left < -0.8
    assert act.fin_right > 0.8


def test_linked_flap_opposes_aft_steer_pitch():
    assert linked_flap_default_enabled() is True

    # Left stick up → aft_steer_z negative → both fins deflect opposite (also −)
    xbox = _xbox(left_stick_y=-0.9)
    act = map_xbox_to_actuators(xbox, deadzone=0.05, linked_flap_enabled=True)
    assert act.aft_steer_z < -0.8
    assert act.fin_left < -0.8
    assert act.fin_right < -0.8


def test_linked_flap_opposes_aft_steer_roll():
    # Left stick right → aft_steer_y positive → differential fins oppose
    xbox = _xbox(left_stick_x=0.9)
    act = map_xbox_to_actuators(xbox, deadzone=0.05, linked_flap_enabled=True)
    assert act.aft_steer_y > 0.8
    assert act.fin_left > 0.8
    assert act.fin_right < -0.8


def test_linked_flap_rt_only_does_not_move_fins():
    xbox = _xbox(triggers={"rt": 1.0})
    act = map_xbox_to_actuators(xbox, deadzone=0.05, trigger_deadzone=0.05)
    assert act.thruster_x > 0.9
    assert abs(act.fin_left) < 0.05
    assert abs(act.fin_right) < 0.05


def test_linked_flap_follows_aft_not_thrust():
    # Full thrust + aft pitch — fins same as aft pitch alone
    xbox_steer = _xbox(left_stick_y=-0.9)
    xbox_both = _xbox(left_stick_y=-0.9, triggers={"rt": 1.0})
    act_steer = map_xbox_to_actuators(
        xbox_steer, deadzone=0.05, trigger_deadzone=0.05, linked_flap_enabled=True
    )
    act_both = map_xbox_to_actuators(
        xbox_both, deadzone=0.05, trigger_deadzone=0.05, linked_flap_enabled=True
    )
    assert act_both.thruster_x > 0.9
    assert act_steer.fin_left == act_both.fin_left
    assert act_steer.fin_right == act_both.fin_right


def test_linked_flap_off_uses_stick_only():
    xbox = _xbox(right_stick_y=-1.0, left_stick_x=0.9)
    act = map_xbox_to_actuators(
        xbox, deadzone=0.05, trigger_deadzone=0.05, linked_flap_enabled=False
    )
    assert act.fin_left > 0.8
    assert act.fin_right > 0.8
    assert act.aft_steer_y > 0.8


def test_linked_flap_toggle_edge_detect():
    xbox = _xbox(buttons={"dpad_left": True})
    enabled, pressed = update_linked_flap_toggle(
        xbox, enabled=True, button_was_pressed=False
    )
    assert enabled is False
    assert pressed is True

    enabled, pressed = update_linked_flap_toggle(
        xbox, enabled=enabled, button_was_pressed=pressed
    )
    assert enabled is False

    xbox = _xbox(buttons={"dpad_left": False})
    enabled, pressed = update_linked_flap_toggle(
        xbox, enabled=enabled, button_was_pressed=pressed
    )
    assert pressed is False

    xbox = _xbox(buttons={"dpad_left": True})
    enabled, pressed = update_linked_flap_toggle(
        xbox, enabled=enabled, button_was_pressed=pressed
    )
    assert enabled is True


def test_stick_drift_does_not_count_as_active():
    xbox = _xbox(left_stick_x=0.06, left_stick_y=-0.05, right_stick_x=0.04)
    act = map_xbox_to_actuators(xbox, deadzone=0.18, linked_flap_enabled=True)
    assert act.thruster_x == 0.0
    assert act.fin_left == 0.0
    assert not xbox_input_active(xbox, act, actuator_threshold=0.12)


def test_deliberate_stick_counts_as_active():
    xbox = _xbox(left_stick_y=-0.9, triggers={"lt": 0.0, "rt": 0.0})
    act = map_xbox_to_actuators(xbox, deadzone=0.18, linked_flap_enabled=True)
    assert xbox_input_active(xbox, act, actuator_threshold=0.12)


def test_left_stick_aft_steer_axes():
    # Horizontal wins when both components are present: only aft Y moves.
    xbox = _xbox(left_stick_x=0.8, left_stick_y=-0.5)
    act = map_xbox_to_actuators(xbox, deadzone=0.05, linked_flap_enabled=False)
    assert act.aft_steer_y > 0.7
    assert abs(act.aft_steer_z) < 0.05


def test_left_stick_axes_move_one_aft_servo():
    left = map_xbox_to_actuators(
        _xbox(left_stick_x=-0.9, left_stick_y=0.35),
        deadzone=0.05,
        linked_flap_enabled=False,
    )
    assert left.aft_steer_y < -0.8
    assert abs(left.aft_steer_z) < 0.05

    up = map_xbox_to_actuators(
        _xbox(left_stick_x=0.4, left_stick_y=-0.9),
        deadzone=0.05,
        linked_flap_enabled=False,
    )
    assert up.aft_steer_z < -0.8
    assert abs(up.aft_steer_y) < 0.05


def test_aft_stick_axes_independent_without_legacy_cross_fallback():
    """If aft_steer_x is omitted, aft Y must not fall back to aft_steer_y axis (same as Z)."""
    cfg = load_mapping_config()
    sticks = dict(cfg.get("sticks") or {})
    left = dict(sticks.get("left") or {})
    left.pop("aft_steer_x", None)
    left.pop("aft_y_stick_axis", None)
    left["aft_steer_y"] = "y"
    left["aft_steer_z"] = "y"
    sticks["left"] = left
    cfg = dict(cfg)
    cfg["sticks"] = sticks
    x_only = map_xbox_to_actuators(
        _xbox(left_stick_x=0.9, left_stick_y=0.0),
        deadzone=0.05,
        config=cfg,
        linked_flap_enabled=False,
    )
    y_only = map_xbox_to_actuators(
        _xbox(left_stick_x=0.0, left_stick_y=-0.9),
        deadzone=0.05,
        config=cfg,
        linked_flap_enabled=False,
    )
    assert abs(x_only.aft_steer_y) > 0.7
    assert abs(x_only.aft_steer_z) < 0.05
    assert abs(y_only.aft_steer_z) > 0.7
    assert abs(y_only.aft_steer_y) < 0.05


def test_aft_stick_per_axis_deadzone_avoids_cross_bleed():
    """Mostly-horizontal stick must not deflect aft Z from circular deadzone scaling."""
    xbox = _xbox(left_stick_x=0.92, left_stick_y=0.08)
    act = map_xbox_to_actuators(xbox, deadzone=0.18, linked_flap_enabled=False)
    assert act.aft_steer_y > 0.55
    assert abs(act.aft_steer_z) < 0.05


def test_emergency_stop_edge_detect():
    prev = False
    triggered, prev = check_emergency_stop(
        _xbox(buttons={"b": True}), button_was_pressed=prev
    )
    assert triggered is True
    assert prev is True

    triggered, prev = check_emergency_stop(
        _xbox(buttons={"b": True}), button_was_pressed=prev
    )
    assert triggered is False

    triggered, prev = check_emergency_stop(
        _xbox(buttons={"b": False}), button_was_pressed=prev
    )
    assert triggered is False
    assert prev is False


def test_emergency_stop_button_does_not_trigger_override():
    xbox = _xbox(buttons={"b": True}, triggers={"lt": 0.0, "rt": 0.0})
    act = map_xbox_to_actuators(xbox, deadzone=0.18, linked_flap_enabled=True)
    assert not xbox_input_active(xbox, act, actuator_threshold=0.12)
