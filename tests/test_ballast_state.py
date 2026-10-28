"""Ballast pot calibration sync between ESP telemetry and dashboard state."""

from src.sub_state import SubState


def test_update_ballast_cal_negative_clears_adc():
    state = SubState()
    state.update_ballast_cal("fore", bottom_adc=500, top_adc=3500, valid=True)
    state.update_ballast_cal("fore", bottom_adc=-1, top_adc=-1, valid=False)
    snap = state.telemetry_snapshot()["ballast"]["fore"]
    assert snap["cal_bottom_adc"] is None
    assert snap["cal_top_adc"] is None
    assert snap["cal_valid"] is False


def test_clear_ballast_cal():
    state = SubState()
    state.update_ballast_cal("aft", bottom_adc=100, top_adc=3000, valid=True)
    state.clear_ballast_cal("aft")
    snap = state.telemetry_snapshot()["ballast"]["aft"]
    assert snap["cal_bottom_adc"] is None
    assert snap["cal_top_adc"] is None
    assert snap["cal_valid"] is False


def test_tel_ballast_fore_aft_updates_adc_and_level():
    from src.esp_bridge import parse_telemetry_line

    state = SubState()
    assert parse_telemetry_line("TEL ballast fore 0.512 2090 1680 0 STOP", state)
    assert parse_telemetry_line("TEL ballast aft 0.250 1024 820 0 STOP", state)
    snap = state.telemetry_snapshot()["ballast"]
    assert snap["fore"]["adc"] == 2090
    assert snap["fore"]["mv_milli"] == 1680
    assert abs(snap["fore"]["level"] - 0.512) < 0.001
    assert snap["aft"]["adc"] == 1024
    assert snap["aft"]["mv_milli"] == 820
    assert abs(snap["aft"]["level"] - 0.25) < 0.001


def test_ballast_tel_is_applied_as_sent():
    from src.esp_bridge import parse_telemetry_line

    state = SubState()
    assert parse_telemetry_line("TEL ballastcal fore 500 3500 1", state)
    assert parse_telemetry_line("TEL ballast fore 0.200 558 450 0 STOP", state)
    fore = state.telemetry_snapshot()["ballast"]["fore"]
    assert fore["adc"] == 558
    assert fore["mv_milli"] == 450
    # (558 - 500) / (3500 - 500) ≈ 0.0193
    assert fore["level"] is not None and fore["level"] < 0.03


def test_calibrated_level_recomputed_from_adc_on_pi():
    from src.esp_bridge import parse_telemetry_line

    state = SubState()
    assert parse_telemetry_line("TEL ballastcal aft 500 3500 1", state)
    assert parse_telemetry_line("TEL ballast aft 0.999 3000 2400 0 STOP", state)
    aft = state.telemetry_snapshot()["ballast"]["aft"]
    assert aft["adc"] == 3000
    # (3000 - 500) / (3500 - 500) = 0.833...
    assert 0.82 < aft["level"] < 0.85


def test_tel_ballast_legacy_line_without_mv():
    from src.esp_bridge import parse_telemetry_line

    state = SubState()
    assert parse_telemetry_line("TEL ballast fore 0.400 1800 0 STOP", state)
    snap = state.telemetry_snapshot()["ballast"]["fore"]
    assert snap["adc"] == 1800
    assert snap["mv_milli"] is None


def test_tel_ballast_negative_level_is_unwired():
    from src.esp_bridge import parse_telemetry_line

    state = SubState()
    assert parse_telemetry_line("TEL ballast fore -1.000 0 0 STOP", state)
    assert state.telemetry_snapshot()["ballast"]["fore"]["level"] is None


def test_wait_ballast_cal_cleared_true_after_tel():
    from src.esp_bridge import parse_telemetry_line
    from src.sub_web import ballast_tank_cal_cleared, wait_ballast_cal_cleared

    state = SubState()
    state.update_ballast_cal("fore", bottom_adc=500, top_adc=3500, valid=True)
    assert not ballast_tank_cal_cleared(state.telemetry_snapshot()["ballast"]["fore"])
    assert parse_telemetry_line("TEL ballastcal fore -1 -1 0", state)
    assert wait_ballast_cal_cleared(state, ["fore"], timeout_s=0.2)


def test_ballastcal_telemetry_parse_clears_on_negative():
    from src.esp_bridge import parse_telemetry_line

    state = SubState()
    state.update_ballast_cal("fore", bottom_adc=69, top_adc=89, valid=False)
    assert parse_telemetry_line("TEL ballastcal fore -1 -1 0", state)
    snap = state.telemetry_snapshot()["ballast"]["fore"]
    assert snap["cal_bottom_adc"] is None
    assert snap["cal_top_adc"] is None
    assert snap["cal_valid"] is False
