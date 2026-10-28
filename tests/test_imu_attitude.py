"""MPU6050 hull-frame remapping."""

from pathlib import Path

from src.imu_attitude import (
    choose_level_mount,
    map_reported_attitude,
    remap_attitude,
    reset_imu_mapper,
    tilt_deg,
)


def test_mount1_levels_sideways_chip():
    pitch, roll = remap_attitude(0.0, 90.0, mount=1)
    assert abs(pitch) < 1.0
    assert abs(roll) < 1.0


def test_mount2_levels_opposite_sideways_chip():
    pitch, roll = remap_attitude(0.0, -90.0, mount=2)
    assert abs(pitch) < 1.0
    assert abs(roll) < 1.0


def test_mount0_is_identity():
    pitch, roll = remap_attitude(8.0, -5.0, mount=0)
    assert abs(pitch - 8.0) < 1e-6
    assert abs(roll + 5.0) < 1e-6


def test_mount5_swaps_pitch_and_roll():
    pitch, roll = remap_attitude(10.0, 0.0, mount=5)
    assert abs(pitch) < 0.2
    assert abs(roll - 10.0) < 0.2


def test_choose_level_mount_sideways_roll():
    assert choose_level_mount(0.0, 90.0) == 1
    assert choose_level_mount(0.0, -90.0) == 2


def test_choose_level_mount_flat_stays_zero():
    assert choose_level_mount(0.0, 0.0) == 0


def test_choose_level_mount_sideways_pitch():
    assert choose_level_mount(90.0, 0.0) in (3, 4)
    assert choose_level_mount(-90.0, 0.0) in (3, 4)


def test_yaw90_after_level_keeps_rest_flat():
    pitch, roll = remap_attitude(0.0, 90.0, mount=1, yaw_mount=5)
    assert tilt_deg(pitch, roll) < 1.0


def test_map_reported_levels_sideways_without_config(tmp_path: Path):
    reset_imu_mapper()
    cfg = tmp_path / "hardware.yaml"
    cfg.write_text("imu:\n  mount: auto\n  yaw: 0\n", encoding="utf-8")
    last = (99.0, 99.0)
    for _ in range(8):
        last = map_reported_attitude(0.0, 90.0, path=cfg)
    assert tilt_deg(*last) < 2.0
    reset_imu_mapper()


def test_map_reported_does_not_invent_90_on_flat(tmp_path: Path):
    reset_imu_mapper()
    cfg = tmp_path / "hardware.yaml"
    cfg.write_text("imu:\n  mount: 1\n  yaw: 0\n", encoding="utf-8")
    last = (99.0, 99.0)
    for _ in range(8):
        last = map_reported_attitude(0.2, -0.3, path=cfg)
    assert tilt_deg(*last) < 5.0
    reset_imu_mapper()


def test_choose_level_mount_does_not_use_yaw_swap():
    assert choose_level_mount(0.0, 90.0) not in (5, 6)
    assert choose_level_mount(90.0, 0.0) not in (5, 6)


def test_map_reported_relocks_after_zero_then_sideways(tmp_path: Path):
    reset_imu_mapper()
    cfg = tmp_path / "hardware.yaml"
    cfg.write_text("imu:\n  mount: auto\n  yaw: 0\n", encoding="utf-8")
    for _ in range(8):
        map_reported_attitude(0.0, 0.0, path=cfg)
    last = map_reported_attitude(0.0, 90.0, path=cfg)
    assert tilt_deg(*last) < 5.0
    reset_imu_mapper()
