"""
sub_control.py
--------------
Helpers for sub control mode and YOLO → sub actuator auto mapping.
"""

from __future__ import annotations

from src.controller import ControlOutput
from src.sub_state import SubActuators


def yolo_to_sub_actuators(output: ControlOutput) -> SubActuators:
    """
    Mirror car YOLO outputs to sub axes (mirror-car mapping from plan).
    Fins stay neutral in auto unless configured otherwise.
    """
    return SubActuators(
        aft_steer_y=output.steering_servo,
        aft_steer_z=output.camera_tilt_servo,
        thruster_x=output.drive_motor,
        fin_left=0.0,
        fin_right=0.0,
    )


def parse_actuator_payload(data: dict) -> SubActuators:
    """Parse actuator fields from API JSON (camelCase or snake_case)."""
    def g(*keys, default=0.0):
        for k in keys:
            if k in data:
                return float(data[k])
        return default

    return SubActuators(
        aft_steer_y=g("aftSteerY", "aft_steer_y"),
        aft_steer_z=g("aftSteerZ", "aft_steer_z"),
        thruster_x=g("thrusterX", "thruster_x"),
        fin_left=g("finLeft", "fin_left"),
        fin_right=g("finRight", "fin_right"),
    )


def clamp_actuators(act: SubActuators) -> SubActuators:
    def c(v: float) -> float:
        return max(-1.0, min(1.0, float(v)))

    return SubActuators(
        aft_steer_y=c(act.aft_steer_y),
        aft_steer_z=c(act.aft_steer_z),
        thruster_x=c(act.thruster_x),
        fin_left=c(act.fin_left),
        fin_right=c(act.fin_right),
    )
