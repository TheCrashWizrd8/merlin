from src.servo_center import (
    DEFAULT_CENTER_TICK,
    FIN_CHANNELS,
    deg_to_tick,
    read_servo_center_ticks,
    tick_to_deg,
    write_servo_center_tick,
)


def test_deg_to_tick_neutral():
    assert deg_to_tick(90.0) == 375
    assert deg_to_tick(96.5) == DEFAULT_CENTER_TICK


def test_read_has_four_fin_channels():
    ticks = read_servo_center_ticks()
    assert set(ticks.keys()) == set(FIN_CHANNELS)
    for t in ticks.values():
        assert 150 <= t <= 600


def test_write_single_channel(tmp_path, monkeypatch):
    import src.servo_center as sc

    yaml_path = tmp_path / "pins.yaml"
    yaml_path.write_text("pca9685:\n  servo_center_ticks:\n    '2': 400\n")
    monkeypatch.setattr(sc, "PINS_PATH", yaml_path)
    write_servo_center_tick(3, 388)
    ticks = read_servo_center_ticks()
    assert ticks[2] == 400
    assert ticks[3] == 388


def test_tick_to_deg_roundtrip():
    for deg in (0.0, 90.0, 96.5, 180.0):
        tick = deg_to_tick(deg)
        assert abs(tick_to_deg(tick) - deg) < 0.6
