"""
sub_control.py
--------------
Sub actuator mapping and JSON parsing helpers.
"""

from __future__ import annotations

from typing import Any

from src.sub_state import SubActuators


def yolo_to_sub_actuators(
    output: Any,
    telemetry: Any | None = None,
) -> SubActuators:
    """Layered auto mapping: fins → aft steer → thruster (see sub_motion.py)."""
    from src.sub_motion import plan_sub_motion
    from src.telemetry_context import TelemetryContext

    if telemetry is not None and not isinstance(telemetry, TelemetryContext):
        telemetry = None
    result = plan_sub_motion(output, telemetry=telemetry)
    return result.actuators


def yolo_to_sub_motion(
    output: Any,
    telemetry: Any | None = None,
):
    """Full motion plan including ballast trim."""
    from src.sub_motion import plan_sub_motion
    from src.telemetry_context import TelemetryContext

    if telemetry is not None and not isinstance(telemetry, TelemetryContext):
        telemetry = None
    return plan_sub_motion(output, telemetry=telemetry)


def parse_actuator_payload(data: dict[str, Any]) -> SubActuators:
    def g(*keys: str, default: float = 0.0) -> float:
        for k in keys:
            if k in data and data[k] is not None:
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
