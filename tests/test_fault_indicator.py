"""Fault indicator logic (no GPIO hardware required)."""

from src.fault_indicator import fault_active_from_tel


def test_no_fault_when_idle():
    assert fault_active_from_tel(leak=False, esp_fault="NONE") is False


def test_leak_triggers():
    assert fault_active_from_tel(leak=True, esp_fault="NONE") is True


def test_esp_fault_triggers():
    assert fault_active_from_tel(leak=False, esp_fault="LEAK") is True


def test_unknown_fault_does_not_trigger():
    assert fault_active_from_tel(leak=False, esp_fault="UNKNOWN") is False


def test_sources_can_disable_leak():
    sources = {"leak": False, "esp_fault": True}
    assert fault_active_from_tel(leak=True, esp_fault="NONE", sources=sources) is False
