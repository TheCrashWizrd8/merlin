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


def test_ballastcal_telemetry_parse_clears_on_negative():
    from src.esp_bridge import parse_telemetry_line

    state = SubState()
    state.update_ballast_cal("fore", bottom_adc=69, top_adc=89, valid=False)
    assert parse_telemetry_line("TEL ballastcal fore -1 -1 0", state)
    snap = state.telemetry_snapshot()["ballast"]["fore"]
    assert snap["cal_bottom_adc"] is None
    assert snap["cal_top_adc"] is None
    assert snap["cal_valid"] is False
