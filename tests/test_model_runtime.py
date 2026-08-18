"""Catalog helpers (no Ultralytics required)."""

from src.model_runtime import (
    catalog_snapshot,
    load_model_catalog,
    model_availability,
    resolve_active_model_id,
)


def test_load_catalog_has_seg():
    catalog = load_model_catalog()
    assert "seg" in catalog
    assert catalog["seg"]["weights"] == "weights/best.pt"


def test_resolve_active_model_defaults_to_config():
    catalog = load_model_catalog()
    snap = catalog_snapshot()
    assert snap["active_id"] == "seg"
    assert len(snap["models"]) >= 1


def test_model_availability_missing_file():
    avail, reason = model_availability(
        {"weights": "weights/does_not_exist_ever.pt"},
        "ncnn",
    )
    assert not avail
    assert reason


def test_resolve_requested_over_config():
    catalog = load_model_catalog()
    cfg = {"active_model": "seg"}
    assert resolve_active_model_id(catalog, cfg, "detect") == "detect"
