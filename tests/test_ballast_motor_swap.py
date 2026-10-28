"""Pi-side fore/aft ballast motor swap on ESP B line."""

from src.esp_bridge import (
    format_ballast_command,
    transform_ballast_motor_line,
    wire_ballast_commands_for_esp,
)


def test_wire_ballast_commands_swap(monkeypatch):
    monkeypatch.setattr(
        "src.esp_bridge._ballast_swap_motors",
        True,
        raising=False,
    )
    assert wire_ballast_commands_for_esp(1.0, -0.5) == (-0.5, 1.0)


def test_format_ballast_command_swap(monkeypatch):
    monkeypatch.setattr(
        "src.esp_bridge._ballast_swap_motors",
        True,
        raising=False,
    )
    assert format_ballast_command(1.0, 0.0) == "B 0.000 1.000\n"


def test_transform_test_b_tank_swap(monkeypatch):
    monkeypatch.setattr(
        "src.esp_bridge._ballast_swap_motors",
        True,
        raising=False,
    )
    assert transform_ballast_motor_line("TEST B fore fill") == "TEST B aft fill"
