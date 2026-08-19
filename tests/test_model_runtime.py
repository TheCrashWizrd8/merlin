"""Catalog helpers (no Ultralytics required)."""

from src.model_runtime import (
    catalog_snapshot,
    first_available_model_id,
    load_model_catalog,
    model_availability,
    resolve_active_model_id,
)


def test_load_catalog_has_deployed_models():
    catalog = load_model_catalog()
    assert "detect" in catalog
    assert "gate" in catalog
    assert "seg" not in catalog  # placeholders removed from yaml


def test_resolve_active_model_defaults_to_config():
    catalog = load_model_catalog()
    snap = catalog_snapshot()
    assert snap["active_id"] in catalog
    assert len(snap["models"]) >= 1


def test_model_availability_missing_file():
    avail, reason = model_availability(
        {"weights": "weights/does_not_exist_ever.pt"},
        "ncnn",
    )
    assert not avail
    assert reason


def test_resolve_prefers_available_over_configured():
    catalog = {
        "gate": {"weights": "weights/missing_gate.pt", "task": "segment"},
        "detect": {"weights": "weights/best.pt", "task": "detect"},
    }
    cfg = {"active_model": "gate", "backend": "ncnn"}
    # best.pt exists on repo Pi; ncnn folder may exist
    picked = resolve_active_model_id(catalog, cfg, backend="ncnn")
    assert picked == "detect"


def test_first_available_skips_missing():
    catalog = {
        "future": {"weights": "weights/nope.pt"},
        "detect": {"weights": "weights/best.pt"},
    }
    assert first_available_model_id(catalog, "pytorch") == "detect"
