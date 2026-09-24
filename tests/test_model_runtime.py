"""Catalog helpers (no Ultralytics required)."""

from pathlib import Path

from src.model_runtime import (
    backends_for_weights,
    catalog_snapshot,
    choose_backend,
    discover_models,
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


def test_discover_includes_disk_weights():
    models = discover_models()
    ids = {m["id"] for m in models}
    assert "detect" in ids or "best" in ids
    for m in models:
        assert "backends" in m
        assert "pytorch" in m["backends"]


def test_detect_is_main_hailo_model():
    models = discover_models()
    detect = next(m for m in models if m["id"] == "detect")
    assert detect["main"] is True
    assert detect["default_backend"] == "hailo"
    assert detect["backends"]["hailo"]["available"] is True
    assert models[0]["id"] == "detect"


def test_backends_pytorch_when_pt_exists():
    info = backends_for_weights("weights/best.pt")
    assert info["pytorch"]["available"] is True
    assert info["hailo"]["available"] is True


def test_backends_hef_is_hailo_only():
    info = backends_for_weights("weights/besthailomodel/best.hef")
    assert info["hailo"]["available"] is True
    assert info["pytorch"]["available"] is False


def test_resolve_active_model_defaults_to_config():
    catalog = load_model_catalog()
    snap = catalog_snapshot()
    assert len(snap["models"]) >= 1
    assert "running" in snap


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
    picked = resolve_active_model_id(catalog, cfg, backend="ncnn")
    assert picked == "detect"


def test_first_available_skips_missing():
    catalog = {
        "future": {"weights": "weights/nope.pt"},
        "detect": {"weights": "weights/best.pt"},
    }
    assert first_available_model_id(catalog, "pytorch") == "detect"


def test_choose_backend_gate_hailo_when_hef_present():
    catalog = load_model_catalog()
    chosen = choose_backend(catalog["gate"], requested="hailo", global_backend="hailo")
    hailo_ok, _ = model_availability(catalog["gate"], "hailo")
    if hailo_ok:
        assert chosen == "hailo"
    else:
        assert chosen == "ncnn"


def test_choose_backend_strict_does_not_fallback():
    entry = {"weights": "weights/does_not_exist_ever.pt"}
    assert choose_backend(entry, requested="hailo", strict=True) == ""


def test_choose_backend_park_occupant_skips_hailo():
    catalog = load_model_catalog()
    chosen = choose_backend(catalog["detect"], requested="hailo", allow_hailo=False)
    assert chosen != "hailo"
    assert chosen in ("ncnn", "pytorch", "openvino")


def test_choose_backend_detect_prefers_hailo():
    catalog = load_model_catalog()
    chosen = choose_backend(catalog["detect"], global_backend="hailo")
    assert chosen == "hailo"


def test_discover_models_returns_independent_copies():
    first = discover_models()
    assert first
    first[0]["id"] = "mutated"
    second = discover_models()
    assert second[0]["id"] != "mutated"


def test_discover_includes_extra_hailo_package():
    models = discover_models()
    ids = {m["id"] for m in models}
    assert "detect" in ids
    assert "gate" in ids
    detect = next(m for m in models if m["id"] == "detect")
    assert detect["backends"]["hailo"]["available"] is True
    extra = next((m for m in models if m["id"] == "best_hailo_model"), None)
    if extra is not None:
        assert extra["backends"]["hailo"]["available"] is True
        assert extra["main"] is False


def test_discover_nested_folder_uses_dir_name(tmp_path: Path):
    from src.model_runtime import invalidate_discover_cache

    weights = tmp_path / "weights"
    model_dir = weights / "lobster"
    model_dir.mkdir(parents=True)
    (model_dir / "best.pt").write_bytes(b"pt")
    (model_dir / "model.yaml").write_text(
        "label: Lobster\ntask: detect\ntrack_label: lobster\ndefault_backend: pytorch\n",
        encoding="utf-8",
    )
    cfg = tmp_path / "model.yaml"
    cfg.write_text("models: {}\n", encoding="utf-8")
    invalidate_discover_cache()
    models = discover_models(config_path=cfg, weights_dir=weights)
    lobster = next(m for m in models if m["id"] == "lobster")
    assert lobster["label"] == "Lobster"
    assert lobster["track_label"] == "lobster"
    assert lobster["task"] == "detect"
    assert lobster["backends"]["pytorch"]["available"] is True


def test_discover_skips_template_and_export_dirs(tmp_path: Path):
    from src.model_runtime import invalidate_discover_cache

    weights = tmp_path / "weights"
    (weights / "_template").mkdir(parents=True)
    (weights / "_template" / "best.pt").write_bytes(b"pt")
    pack = weights / "besthailomodel"
    pack.mkdir()
    (pack / "best.hef").write_bytes(b"hef")
    cfg = tmp_path / "model.yaml"
    cfg.write_text("models: {}\n", encoding="utf-8")
    invalidate_discover_cache()
    models = discover_models(config_path=cfg, weights_dir=weights)
    ids = {m["id"] for m in models}
    assert "_template" not in ids
    assert "besthailomodel" not in ids


def test_discover_catalog_overlays_sidecar(tmp_path: Path):
    from src.model_runtime import invalidate_discover_cache

    weights = tmp_path / "weights"
    model_dir = weights / "detect"
    model_dir.mkdir(parents=True)
    (model_dir / "best.pt").write_bytes(b"pt")
    (model_dir / "model.yaml").write_text(
        "label: From sidecar\ntask: detect\ntrack_label: fruit\n",
        encoding="utf-8",
    )
    cfg = tmp_path / "model.yaml"
    cfg.write_text(
        "models:\n  detect:\n    label: From catalog\n"
        "    track_label: apple\n    main: true\n",
        encoding="utf-8",
    )
    invalidate_discover_cache()
    models = discover_models(config_path=cfg, weights_dir=weights)
    detect = next(m for m in models if m["id"] == "detect")
    assert detect["label"] == "From catalog"
    assert detect["track_label"] == "apple"
    assert detect["main"] is True


def test_discover_hef_only_reads_segment_metadata(tmp_path: Path):
    from src.model_runtime import invalidate_discover_cache

    weights = tmp_path / "weights"
    model_dir = weights / "newgate"
    model_dir.mkdir(parents=True)
    (model_dir / "gate.hef").write_bytes(b"hef")
    (model_dir / "metadata.yaml").write_text(
        "task: segment\nnames:\n  0: gate\n",
        encoding="utf-8",
    )
    cfg = tmp_path / "model.yaml"
    cfg.write_text("models: {}\n", encoding="utf-8")
    invalidate_discover_cache()
    models = discover_models(config_path=cfg, weights_dir=weights)
    newgate = next(m for m in models if m["id"] == "newgate")
    assert newgate["task"] == "segment"
    assert newgate["track_label"] == "gate"
    assert newgate["default_backend"] == "hailo"


def test_choose_backend_can_park_on_cpu():
    models = discover_models()
    detect = next(m for m in models if m["id"] == "detect")
    cpu = choose_backend(detect, allow_hailo=False)
    assert cpu in ("ncnn", "openvino", "pytorch")
    assert choose_backend(detect, requested="hailo", strict=True) == "hailo"
