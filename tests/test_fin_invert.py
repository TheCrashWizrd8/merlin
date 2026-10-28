"""Fin R invert_command on S2 encoding (matched travel, all modes)."""

from src.esp_bridge import format_actuator_command
from src.servo_center import (
    fin_right_inverted_matched_command,
    tick_delta_for_command,
    wire_fin_commands_for_s2,
)
from src.sub_state import SubActuators


def test_matched_invert_equal_tick_travel():
    c4, c5 = 221, 258
    v_l = -0.9
    v_r = fin_right_inverted_matched_command(v_l, left_center=c4, right_center=c5)
    d_l = tick_delta_for_command(v_l, c4)
    d_r = tick_delta_for_command(v_r, c5)
    assert abs(d_l) > 1.0
    assert abs(d_r - (-d_l)) < 0.01
    assert abs(abs(d_r) - abs(d_l)) < 0.01


def test_invert_pitch_same_speed_not_naive_negate():
    from src.servo_center import read_servo_center_ticks, read_servo_travel_ticks

    centers = read_servo_center_ticks()
    travel = read_servo_travel_ticks()
    fl, fr = wire_fin_commands_for_s2(-0.9, -0.9, invert_fin_r=True)
    assert fl == -0.9
    assert fr != 0.9  # not same sign as naive (non-invert) wire
    c4, c5 = centers[4], centers[5]
    l_min, l_max = travel[4]
    r_min, r_max = travel[5]
    d_l = tick_delta_for_command(fl, c4, min_tick=l_min, max_tick=l_max)
    d_r = tick_delta_for_command(fr, c5, min_tick=r_min, max_tick=r_max)
    assert abs(abs(d_r) - abs(d_l)) < 0.02


def test_invert_roll_free_and_linked_same_wire():
    # Right-stick roll (free fins) and linked-flap roll share logical L/R signs.
    fl, fr = wire_fin_commands_for_s2(0.8, -0.8, invert_fin_r=True)
    assert fl == 0.8
    assert fr == 0.8


def test_invert_flaps_up_matched():
    from src.servo_center import read_servo_center_ticks, read_servo_travel_ticks

    centers = read_servo_center_ticks()
    travel = read_servo_travel_ticks()
    fl, fr = wire_fin_commands_for_s2(0.5, 0.5, invert_fin_r=True)
    assert fl == 0.5
    assert fr != -0.5  # not a naive negate of fin L
    c4, c5 = centers[4], centers[5]
    l_min, l_max = travel[4]
    r_min, r_max = travel[5]
    d_l = tick_delta_for_command(fl, c4, min_tick=l_min, max_tick=l_max)
    d_r = tick_delta_for_command(fr, c5, min_tick=r_min, max_tick=r_max)
    if abs(fr) < 0.99:
        assert abs(abs(d_r) - abs(d_l)) < 0.05


def test_format_actuator_uses_wire_in_all_modes():
    act = SubActuators(aft_steer_y=0.0, aft_steer_z=-0.5, fin_left=0.8, fin_right=-0.8, thruster_x=0.0)
    line = format_actuator_command(act)
    assert "0.800 0.800" in line
