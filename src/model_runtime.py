"""
model_runtime.py
----------------
Loadable YOLO model catalog (config/model.yaml) and thread-safe switching
while inference is running. The sub dashboard selects models in Auto mode.
"""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import Any, Optional

import yaml

from src.detector import (
    BACKENDS,
    CONFIG_PATH,
    Detector,
    _exported_model_dir,
    _resolve_weights_path,
)

BACKEND_PREFERENCE = ("hailo", "ncnn", "openvino", "pytorch")

PROJECT_ROOT = Path(__file__).parent.parent
WEIGHTS_DIR = PROJECT_ROOT / "weights"

# Sibling export folders Ultralytics/DFC write next to a .pt — not models.
_EXPORT_DIR_NAMES = frozenset({"hailo", "ncnn", "openvino", "__pycache__", "runs"})
_EXPORT_DIR_SUFFIXES = (
    "_hailo_model",
    "_ncnn_model",
    "_openvino_model",
    "hailomodel",
    "_hailo",
)

_runtime: Optional["ModelRuntime"] = None
_runtime_lock = threading.Lock()
_DISCOVER_TTL_S = 5.0
_discover_lock = threading.Lock()
_discover_cache: dict[str, tuple[float, list[dict]]] = {}


def invalidate_discover_cache() -> None:
    with _discover_lock:
        _discover_cache.clear()


def _load_yaml(path: Path = CONFIG_PATH) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def load_model_catalog(config_path: Path = CONFIG_PATH) -> dict[str, dict]:
    cfg = _load_yaml(config_path)
    raw = cfg.get("models") or {}
    if not isinstance(raw, dict):
        return {}
    catalog: dict[str, dict] = {}
    for model_id, entry in raw.items():
        if not isinstance(entry, dict):
            continue
        catalog[str(model_id)] = dict(entry)
    return catalog


def resolve_active_model_id(
    catalog: dict[str, dict],
    cfg: dict,
    requested_id: str = "",
    backend: str = "",
) -> str:
    if not backend:
        backend = str(cfg.get("backend") or "pytorch").lower()

    def available(model_id: str) -> bool:
        entry = catalog.get(model_id)
        if not entry:
            return False
        ok, _ = model_availability(entry, backend)
        return ok

    if requested_id and available(requested_id):
        return requested_id
    if requested_id and requested_id in catalog and not available(requested_id):
        print(
            f"[ModelRuntime] Requested model {requested_id!r} unavailable; "
            "picking another catalog entry."
        )

    active = str(cfg.get("active_model") or "").strip()
    if active and available(active):
        return active
    if active and active in catalog and not available(active):
        print(
            f"[ModelRuntime] active_model={active!r} unavailable; "
            "picking first available entry."
        )

    for model_id, entry in catalog.items():
        if model_availability(entry, backend)[0]:
            return model_id

    if requested_id and requested_id in catalog:
        return requested_id
    if active and active in catalog:
        return active
    if catalog:
        return next(iter(catalog))
    return ""


def first_available_model_id(
    catalog: dict[str, dict],
    backend: str,
) -> str:
    for model_id, entry in catalog.items():
        if model_availability(entry, backend)[0]:
            return model_id
    for model_id, entry in catalog.items():
        if choose_backend(entry, global_backend=backend):
            return model_id
    return ""


def choose_backend(
    entry: dict,
    requested: str = "",
    global_backend: str = "",
    *,
    strict: bool = False,
    allow_hailo: bool = True,
) -> str:
    """Pick a runnable backend for one catalog entry.

    An explicit ``requested`` backend wins when it is available. With
    ``strict=True`` (dashboard clicks) that is the only candidate — no
    silent Hailo → NCNN fallback. ``allow_hailo=False`` is for parking
    the previous Hailo occupant on CPU.
    """
    requested = str(requested or "").strip().lower()

    def _ok(backend: str) -> bool:
        if backend not in BACKENDS:
            return False
        if not allow_hailo and backend == "hailo":
            return False
        return model_availability(entry, backend)[0]

    if requested:
        if _ok(requested):
            return requested
        if strict:
            return ""

    seen: list[str] = []
    for candidate in (
        str(entry.get("default_backend") or "").strip().lower(),
        str(global_backend or "").strip().lower(),
        *BACKEND_PREFERENCE,
    ):
        if candidate and candidate not in seen:
            seen.append(candidate)
    for backend in seen:
        if _ok(backend):
            return backend
    return ""


def _normalize_weights(weights: str) -> str:
    return str(weights).replace("\\", "/").lstrip("./")


def backends_for_weights(weights_value: str) -> dict[str, dict]:
    """Which runtimes can actually load this .pt / .hef (or its exports) on disk."""
    result: dict[str, dict] = {}
    for backend in BACKENDS:
        ok, reason = model_availability({"weights": weights_value}, backend)
        result[backend] = {"available": ok, "unavailable_reason": reason}
    return result


def discover_models(
    config_path: Path = CONFIG_PATH,
    weights_dir: Path | None = None,
) -> list[dict]:
    """
    Catalog = YAML labels overlaid on ``weights/<model_id>/``.

    Drop a new folder (``.pt`` plus optional ``model.yaml`` sidecar) and it
    appears on the dashboard. Flat legacy files under ``weights/*.pt`` still
    work.
    """
    wdir = Path(weights_dir) if weights_dir is not None else WEIGHTS_DIR
    key = f"{config_path}|{wdir}"
    now = time.monotonic()
    with _discover_lock:
        hit = _discover_cache.get(key)
        if hit is not None and now - hit[0] < _DISCOVER_TTL_S:
            return [dict(item) for item in hit[1]]
    models = _discover_models_uncached(config_path, wdir)
    with _discover_lock:
        _discover_cache[key] = (now, models)
    return [dict(item) for item in models]


def _is_export_dir_name(name: str) -> bool:
    if name.startswith("_") or name in _EXPORT_DIR_NAMES:
        return True
    return any(name.endswith(suffix) for suffix in _EXPORT_DIR_SUFFIXES)


def _rel_weights(path: Path) -> str:
    try:
        return path.resolve().relative_to(PROJECT_ROOT.resolve()).as_posix()
    except ValueError:
        try:
            return path.relative_to(PROJECT_ROOT).as_posix()
        except ValueError:
            return path.as_posix().replace("\\", "/")


def _resolved(path: Path) -> Path:
    try:
        return path.resolve()
    except OSError:
        return path


def _load_sidecar(model_dir: Path) -> dict:
    path = model_dir / "model.yaml"
    if not path.is_file():
        return {}
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except (OSError, yaml.YAMLError):
        return {}
    return dict(data) if isinstance(data, dict) else {}


def _primary_pt(model_dir: Path, sidecar: dict) -> Path | None:
    named = str(sidecar.get("weights") or "").strip()
    if named:
        candidate = Path(named)
        if not candidate.is_absolute():
            candidate = model_dir / Path(named).name
        if candidate.is_file() and candidate.suffix.lower() == ".pt":
            return candidate
    pts = sorted(p for p in model_dir.glob("*.pt") if p.is_file())
    for preferred in ("model.pt", "best.pt"):
        for pt in pts:
            if pt.name == preferred:
                return pt
    return pts[0] if pts else None


def _infer_hef_fields(hef: Path) -> dict:
    """task / track_label from metadata.yaml next to a .hef."""
    from src.hailo_runtime import class_names_from_export, task_from_export

    folder = hef.parent
    out: dict[str, str] = {"default_backend": "hailo"}
    task = task_from_export(folder)
    if task:
        out["task"] = task
    names = class_names_from_export(folder)
    if names:
        out["track_label"] = str(next(iter(names.values())))
    return out


def _primary_hef(model_dir: Path) -> Path | None:
    from src.hailo_runtime import hailo_hef_path

    for folder in (model_dir / "hailo", model_dir):
        found = hailo_hef_path(folder) if folder.is_dir() else None
        if found is not None:
            return found
    try:
        children = list(model_dir.iterdir())
    except OSError:
        return None
    for child in sorted(children):
        if child.is_dir() and _is_export_dir_name(child.name):
            found = hailo_hef_path(child)
            if found is not None:
                return found
    return None


def _merge_entry(model_id: str, sidecar: dict, catalog_entry: dict | None) -> dict:
    """Sidecar fills defaults; config/model.yaml overlays the same id."""
    entry: dict = {
        "label": model_id,
        "task": "auto",
        "track_label": model_id,
        "default_backend": "",
        "main": False,
    }
    if sidecar:
        entry.update({k: v for k, v in sidecar.items() if k != "id" and v not in (None, "")})
    if catalog_entry:
        entry.update({k: v for k, v in catalog_entry.items() if v not in (None, "")})
    return entry


def _model_record(model_id: str, weights: str, entry: dict) -> dict:
    backends = backends_for_weights(str(entry.get("weights") or weights))
    any_ok = any(b["available"] for b in backends.values())
    return {
        "id": model_id,
        "label": str(entry.get("label") or model_id),
        "task": str(entry.get("task") or "auto"),
        "track_label": str(entry.get("track_label") or "apple"),
        "weights": str(entry.get("weights") or weights),
        "available": any_ok,
        "unavailable_reason": None if any_ok else "no runnable backend on disk",
        "backends": backends,
        "main": bool(entry.get("main")),
        "default_backend": str(entry.get("default_backend") or ""),
    }


def _discover_models_uncached(config_path: Path, weights_dir: Path) -> list[dict]:
    catalog = load_model_catalog(config_path)
    by_weights: dict[str, tuple[str, dict]] = {}
    by_resolved: dict[Path, tuple[str, dict]] = {}
    for model_id, entry in catalog.items():
        raw = str(entry.get("weights") or "")
        key = _normalize_weights(raw)
        if key:
            by_weights[key] = (model_id, entry)
        if raw:
            by_resolved[_resolved(_resolve_weights_path(raw))] = (model_id, entry)

    seen: set[str] = set()
    seen_files: set[Path] = set()
    models: list[dict] = []

    def _claim(model_id: str, path: Path, entry: dict, weights: str) -> None:
        resolved = _resolved(path)
        if model_id in seen or resolved in seen_files:
            return
        seen.add(model_id)
        seen_files.add(resolved)
        models.append(_model_record(model_id, weights, entry))

    if weights_dir.is_dir():
        for child in sorted(weights_dir.iterdir()):
            if not child.is_dir() or _is_export_dir_name(child.name):
                continue
            sidecar = _load_sidecar(child)
            model_id = str(sidecar.get("id") or child.name).strip() or child.name
            catalog_entry = catalog.get(model_id)
            pt = _primary_pt(child, sidecar)
            hef = None if pt is not None else _primary_hef(child)
            source = pt or hef
            if source is None:
                continue
            if hef is not None:
                for key, value in _infer_hef_fields(hef).items():
                    sidecar.setdefault(key, value)
            resolved = _resolved(source)
            mapped = by_resolved.get(resolved) or by_weights.get(
                _normalize_weights(_rel_weights(source))
            )
            if mapped:
                model_id, catalog_entry = mapped
            entry = _merge_entry(model_id, sidecar, catalog_entry)
            if catalog_entry and catalog_entry.get("weights"):
                catalog_path = _resolve_weights_path(str(catalog_entry["weights"]))
                if catalog_path.is_file() or catalog_path.is_symlink():
                    entry["weights"] = str(catalog_entry["weights"])
                    _claim(model_id, catalog_path, entry, str(catalog_entry["weights"]))
                    continue
            rel = _rel_weights(source)
            entry["weights"] = rel
            _claim(model_id, source, entry, rel)

        for pt in sorted(p for p in weights_dir.glob("*.pt") if p.is_file()):
            resolved = _resolved(pt)
            if resolved in seen_files:
                continue
            rel = _rel_weights(pt)
            key = _normalize_weights(rel)
            mapped = by_resolved.get(resolved) or by_weights.get(key)
            if mapped:
                model_id, catalog_entry = mapped
            else:
                model_id = pt.stem
                catalog_entry = catalog.get(model_id)
            entry = _merge_entry(model_id, {}, catalog_entry)
            if not catalog_entry:
                entry["weights"] = rel
            _claim(model_id, pt, entry, str(entry.get("weights") or rel))

    covered_hefs = _covered_hef_paths(models)

    hefs: list[Path] = []
    if weights_dir.is_dir():
        hefs.extend(sorted(weights_dir.glob("*.hef")))
        for dirpath, dirnames, filenames in os.walk(weights_dir):
            dirnames[:] = [
                name for name in dirnames if name not in {"__pycache__", "runs"}
            ]
            folder = Path(dirpath)
            if folder == weights_dir or _is_export_dir_name(folder.name):
                for name in filenames:
                    if name.lower().endswith(".hef"):
                        hefs.append(folder / name)
    for hef in hefs:
        resolved = _resolved(hef)
        if resolved in covered_hefs or resolved in seen_files:
            continue
        rel = _rel_weights(hef)
        try:
            parts = list(hef.relative_to(weights_dir).parts)
        except ValueError:
            parts = [hef.parent.name, hef.name]
        if parts and _is_export_dir_name(parts[0]):
            continue
        model_id = next(
            (part for part in parts if not _is_export_dir_name(part) and not part.lower().endswith(".hef")),
            hef.stem,
        )
        if model_id in seen:
            continue
        seen.add(model_id)
        seen_files.add(resolved)
        covered_hefs.add(resolved)
        inferred = _infer_hef_fields(hef)
        backends = backends_for_weights(rel)
        models.append({
            "id": model_id,
            "label": f"{model_id} (Hailo)",
            "task": inferred.get("task") or "detect",
            "track_label": inferred.get("track_label") or "apple",
            "weights": rel,
            "available": True,
            "unavailable_reason": None,
            "backends": backends,
            "main": False,
            "default_backend": "hailo",
        })

    for model_id, entry in catalog.items():
        if model_id in seen:
            continue
        weights = str(entry.get("weights") or "")
        models.append(_model_record(model_id, weights, entry))

    models.sort(
        key=lambda m: (
            0 if m.get("main") else 1,
            0 if (m.get("backends") or {}).get("hailo", {}).get("available") else 1,
            str(m.get("id") or ""),
        )
    )
    return models


def _covered_hef_paths(models: list[dict]) -> set[Path]:
    """HEF files already claimed by a discovered catalog entry."""
    covered: set[Path] = set()
    try:
        from src.hailo_runtime import hailo_export_dir, hailo_hef_path
    except Exception:
        return covered
    for item in models:
        weights = str(item.get("weights") or "")
        if not weights:
            continue
        path = _resolve_weights_path(weights)
        hef: Path | None = None
        if path.suffix.lower() == ".hef" and path.is_file():
            hef = path
        else:
            found = hailo_hef_path(hailo_export_dir(path))
            if found is not None:
                hef = found
        if hef is None:
            continue
        try:
            covered.add(hef.resolve())
        except OSError:
            covered.add(hef)
    return covered


def model_availability(
    entry: dict,
    backend: str,
) -> tuple[bool, Optional[str]]:
    weights = str(entry.get("weights") or "").strip()
    if not weights:
        return False, "no weights path in catalog"
    weights_path = _resolve_weights_path(weights)
    if backend == "pytorch":
        if weights_path.suffix.lower() == ".hef":
            return False, "pytorch cannot load a .hef — use hailo"
        if not weights_path.is_file():
            return False, f"missing {weights}"
        return True, None
    if backend == "hailo":
        from src.hailo_runtime import hailo_export_dir, package_status

        if weights_path.suffix.lower() == ".hef":
            if weights_path.is_file():
                return True, None
            return False, f"missing {weights}"
        export_dir = hailo_export_dir(weights_path)
        return package_status(export_dir)
    if not weights_path.is_file():
        return False, f"missing {weights}"
    exported = _exported_model_dir(weights_path, backend)
    if not exported.is_dir():
        return False, f"run: python scripts/export_model.py --weights {weights} --format {backend}"
    return True, None


def catalog_snapshot(
    *,
    config_path: Path = CONFIG_PATH,
    active_id: str = "",
    status: Optional[dict] = None,
    backend: str = "",
    running: bool = False,
) -> dict[str, Any]:
    models = discover_models(config_path)
    if not backend:
        cfg = _load_yaml(config_path)
        backend = str(cfg.get("backend") or "pytorch").lower()
    return {
        "active_id": active_id,
        "status": status or {"state": "idle", "error": None},
        "backend": backend,
        "running": running,
        "models": models,
    }


def models_dashboard_snapshot() -> dict[str, Any]:
    """SSE/API snapshot; safe when inference has not started."""
    try:
        from src.inference_service import get_inference_service

        service = get_inference_service()
        return service.snapshot()
    except Exception:
        pass
    try:
        from src.sub_state import get_sub_state

        state = get_sub_state()
        active_id = state.get_yolo_model_id()
        status = state.get_yolo_model_status()
    except Exception:
        active_id = ""
        status = {"state": "idle", "error": None}
    runtime = get_model_runtime()
    running = False
    backend = ""
    if runtime is not None:
        active_id = runtime.active_id or active_id
        status = runtime.status_snapshot()
        running = True
        backend = str(getattr(runtime, "backend", "") or "")
    return catalog_snapshot(
        active_id=active_id,
        status=status,
        backend=backend,
        running=running,
    )


def get_model_runtime() -> Optional["ModelRuntime"]:
    return _runtime


def shutdown_model_runtime() -> None:
    global _runtime
    with _runtime_lock:
        rt = _runtime
        _runtime = None
    if rt is not None:
        rt.close()


def init_model_runtime(
    *,
    config_path: Path = CONFIG_PATH,
    backend: Optional[str] = None,
    imgsz_override: Optional[int] = None,
    start_id: Optional[str] = None,
    lazy: bool = False,
) -> "ModelRuntime":
    global _runtime
    with _runtime_lock:
        if _runtime is not None:
            if backend:
                _runtime._backend_override = backend
            if start_id:
                result = _runtime.select(start_id, backend=backend)
                if not result.get("ok"):
                    if not _runtime.active_id:
                        raise RuntimeError(
                            result.get("error")
                            or f"select {start_id}/{backend} failed"
                        )
                    print(
                        f"[ModelRuntime] {result.get('error')}; "
                        f"keeping {_runtime.active_id}/{_runtime.backend}"
                    )
            return _runtime
        _runtime = ModelRuntime(
            config_path=config_path,
            backend=backend,
            imgsz_override=imgsz_override,
            start_id=start_id,
            lazy=lazy,
        )
        return _runtime


class ModelRuntime:
    """Thread-safe catalog: every runnable model is loaded, one is active.

    Hailo-8L only holds one HEF at a time. Preload puts that HEF on the
    start model; every other catalog entry loads on NCNN/PyTorch. If the
    chip is busy or missing, the Hailo load falls back to CPU instead of
    taking the YOLO service down.
    """

    def __init__(
        self,
        config_path: Path = CONFIG_PATH,
        backend: Optional[str] = None,
        imgsz_override: Optional[int] = None,
        start_id: Optional[str] = None,
        lazy: bool = False,
    ) -> None:
        self.config_path = config_path
        self._backend_override = backend
        self._imgsz_override = imgsz_override
        self._lock = threading.RLock()
        self._cfg = _load_yaml(config_path)
        self._catalog = self._refresh_catalog()
        self._active_id = ""
        self._track_label = "apple"
        self._detector: Optional[Detector] = None
        self._loaded: dict[str, Detector] = {}
        self._status: dict[str, Any] = {"state": "idle", "error": None, "task": None}

        if lazy:
            self._sync_state()
            return

        try:
            from src.sub_state import get_sub_state

            requested = start_id or get_sub_state().get_yolo_model_id()
        except Exception:
            requested = start_id or ""

        global_backend = str(
            self._backend_override or self._cfg.get("backend") or "pytorch"
        ).lower()
        start = resolve_active_model_id(
            self._catalog,
            self._cfg,
            requested,
            backend=global_backend,
        )
        if start_id:
            start = start_id
        if start and start not in self._catalog:
            start = ""
        self._preload_all(start, global_backend)
        if start and start in self._loaded:
            self._activate(start)
        elif self._loaded:
            self._activate(next(iter(self._loaded)))
        else:
            from src.hailo_runtime import hailo_busy_hint

            raise RuntimeError(
                "No YOLO model could be loaded. "
                + hailo_busy_hint()
            )

    @property
    def active_id(self) -> str:
        return self._active_id

    @property
    def track_label(self) -> str:
        return self._track_label

    @property
    def backend(self) -> str:
        if self._detector is not None:
            return self._detector.backend
        return str(self._backend_override or self._cfg.get("backend") or "pytorch").lower()

    @property
    def img_size(self) -> int:
        if self._detector is None:
            return int(self._cfg.get("img_size") or 640)
        return self._detector.img_size

    @img_size.setter
    def img_size(self, value: int) -> None:
        size = int(value)
        for detector in self._loaded.values():
            detector.img_size = size

    def status_snapshot(self) -> dict[str, Any]:
        with self._lock:
            snap = dict(self._status)
            snap["loaded"] = self._loaded_summary()
            return snap

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            status = dict(self._status)
            status["loaded"] = self._loaded_summary()
            active_id = self._active_id
            loaded = {mid: det.backend for mid, det in self._loaded.items()}
        snap = catalog_snapshot(
            config_path=self.config_path,
            active_id=active_id,
            status=status,
            backend=str(
                self._detector.backend
                if self._detector
                else (self._backend_override or self._cfg.get("backend") or "pytorch")
            ).lower(),
        )
        for item in snap.get("models") or []:
            mid = item.get("id")
            item["loaded"] = mid in loaded
            if mid in loaded:
                item["loaded_backend"] = loaded[mid]
        snap["loaded_ids"] = list(loaded)
        return snap

    def _loaded_summary(self) -> list[dict[str, str]]:
        return [
            {"id": mid, "backend": det.backend}
            for mid, det in self._loaded.items()
        ]

    def _refresh_catalog(self) -> dict[str, dict]:
        yaml_cat = load_model_catalog(self.config_path)
        discovered = discover_models(self.config_path)
        merged: dict[str, dict] = {}
        for item in discovered:
            merged[item["id"]] = {
                "label": item["label"],
                "weights": item["weights"],
                "task": item["task"],
                "track_label": item["track_label"],
                "main": item.get("main", False),
                "default_backend": item.get("default_backend") or "",
            }
        for model_id, entry in yaml_cat.items():
            merged.setdefault(model_id, dict(entry))
            merged[model_id].update(entry)
        return merged

    def _sync_state(self) -> None:
        try:
            from src.sub_state import get_sub_state

            state = get_sub_state()
            state.set_yolo_model_id(self._active_id)
            status = dict(self._status)
            status["loaded"] = self._loaded_summary()
            state.set_yolo_model_status(status)
        except Exception:
            pass

    def _entry_for(self, model_id: str) -> dict:
        self._catalog = self._refresh_catalog()
        if model_id not in self._catalog:
            raise KeyError(f"Unknown model {model_id!r}")
        return {"id": model_id, **self._catalog[model_id]}

    def _global_backend(self) -> str:
        return str(
            self._backend_override or self._cfg.get("backend") or "pytorch"
        ).lower()

    def _hailo_occupant(self) -> Optional[str]:
        for mid, det in self._loaded.items():
            if det.backend == "hailo":
                return mid
        return None

    def _park_hailo_occupant(self, occupant: str) -> None:
        """Free the Hailo-8L. Reload the previous model on CPU if possible."""
        old = self._loaded.pop(occupant, None)
        if old is not None:
            old.close()
            print(f"[ModelRuntime] Released Hailo from {occupant!r}")
            time.sleep(0.25)
        entry = self._catalog.get(occupant) or {}
        cpu = choose_backend(entry, allow_hailo=False)
        if not cpu:
            print(
                f"[ModelRuntime] {occupant!r} has no CPU backend; left unloaded"
            )
            return
        try:
            self._ensure_loaded(occupant, requested=cpu)
        except Exception as exc:
            print(
                f"[ModelRuntime] Could not keep {occupant!r} on {cpu}: {exc}"
            )

    def _activate(self, model_id: str) -> None:
        detector = self._loaded[model_id]
        entry = self._catalog.get(model_id) or {}
        self._detector = detector
        self._active_id = model_id
        self._track_label = str(
            entry.get("track_label") or detector.track_label or "apple"
        )
        self._status = {
            "state": "ready",
            "error": None,
            "task": detector.task,
            "label": str(entry.get("label") or model_id),
            "backend": detector.backend,
            "loaded": self._loaded_summary(),
        }
        self._sync_state()
        print(
            f"[ModelRuntime] Active model={model_id!r} backend={detector.backend} "
            f"task={detector.task} track_label={self._track_label!r} "
            f"loaded={','.join(self._loaded) or 'none'}"
        )

    def _preload_all(self, start_id: str, global_backend: str) -> None:
        order: list[str] = []
        if start_id and start_id in self._catalog:
            order.append(start_id)
        mains = [mid for mid, e in self._catalog.items() if e.get("main") and mid not in order]
        order.extend(mains)
        for mid in self._catalog:
            if mid not in order:
                order.append(mid)
        for mid in order:
            entry = self._catalog.get(mid) or {}
            if mid == start_id:
                requested = global_backend
            else:
                cpu = choose_backend(entry, allow_hailo=False)
                if not cpu:
                    hailo_ok = choose_backend(entry, requested="hailo", strict=True)
                    if hailo_ok:
                        print(
                            f"[ModelRuntime] {mid!r} is Hailo-only; "
                            "will load when you select Hailo"
                        )
                    else:
                        print(f"[ModelRuntime] {mid!r} has no runnable backend; skip")
                    continue
                requested = cpu
            try:
                self._ensure_loaded(mid, requested=requested)
            except Exception as exc:
                print(f"[ModelRuntime] Could not load {mid!r}: {exc}")

    def _ensure_loaded(self, model_id: str, requested: str = "") -> Detector:
        entry = self._entry_for(model_id)
        chosen = choose_backend(
            entry,
            requested=requested,
            global_backend=self._global_backend(),
            strict=bool(requested),
        )
        if not chosen:
            raise FileNotFoundError(f"model {model_id!r} has no runnable backend")

        existing = self._loaded.get(model_id)
        if existing is not None and existing.backend == chosen:
            return existing

        if chosen == "hailo":
            occupant = self._hailo_occupant()
            if occupant and occupant != model_id:
                if str(requested).lower() == "hailo":
                    self._park_hailo_occupant(occupant)
                else:
                    fallback = choose_backend(entry, allow_hailo=False)
                    if fallback:
                        print(
                            f"[ModelRuntime] Hailo is holding {occupant!r}; "
                            f"loading {model_id!r} on {fallback}"
                        )
                        chosen = fallback
                    else:
                        print(
                            f"[ModelRuntime] Hailo is holding {occupant!r}; "
                            f"deferring {model_id!r} until you select it"
                        )
                        raise RuntimeError(
                            f"Hailo already running {occupant!r}; "
                            f"select {model_id!r} to swap"
                        )

        if existing is not None:
            existing.close()
            self._loaded.pop(model_id, None)

        self._status = {
            "state": "loading",
            "error": None,
            "task": entry.get("task"),
            "label": entry.get("label", model_id),
            "backend": chosen,
            "loaded": self._loaded_summary(),
        }
        self._sync_state()

        spec = dict(entry)
        spec["id"] = model_id
        spec["backend"] = chosen
        try:
            detector = Detector(
                config_path=self.config_path,
                backend=chosen,
                model_spec=spec,
            )
        except Exception as exc:
            if chosen != "hailo":
                raise
            from src.hailo_runtime import hailo_busy_hint

            print(f"[ModelRuntime] Hailo failed for {model_id!r}: {hailo_busy_hint(exc)}")
            cpu = choose_backend(entry, allow_hailo=False)
            if not cpu:
                raise RuntimeError(hailo_busy_hint(exc)) from exc
            print(f"[ModelRuntime] Falling back to {cpu} for {model_id!r}")
            chosen = cpu
            spec["backend"] = cpu
            detector = Detector(
                config_path=self.config_path,
                backend=cpu,
                model_spec=spec,
            )
        if self._imgsz_override:
            detector.img_size = int(self._imgsz_override)
        self._loaded[model_id] = detector
        print(
            f"[ModelRuntime] Loaded {model_id!r} backend={detector.backend} "
            f"task={detector.task}"
        )
        return detector

    def close(self) -> None:
        with self._lock:
            for detector in self._loaded.values():
                detector.close()
            self._loaded.clear()
            self._detector = None
            self._status = {"state": "idle", "error": None, "task": None, "label": ""}
            self._sync_state()

    def _load_model(self, model_id: str, backend: Optional[str] = None) -> None:
        wanted = str(backend or "")
        if wanted == "hailo":
            occupant = self._hailo_occupant()
            if occupant not in (None, model_id):
                self._park_hailo_occupant(occupant)
        self._ensure_loaded(model_id, requested=wanted)
        self._activate(model_id)

    def apply_pending(self) -> None:
        """Reload if sub_state requested a different model."""
        try:
            from src.sub_state import get_sub_state

            wanted = get_sub_state().get_yolo_model_id()
        except Exception:
            return
        if not wanted or wanted == self._active_id:
            return
        with self._lock:
            if wanted == self._active_id:
                return
            try:
                self._load_model(wanted)
            except Exception as exc:
                self._status = {
                    "state": "error",
                    "error": str(exc),
                    "task": None,
                    "label": self._catalog.get(wanted, {}).get("label", wanted),
                    "loaded": self._loaded_summary(),
                }
                self._sync_state()
                print(f"[ModelRuntime] Failed to load {wanted!r}: {exc}")

    def select(self, model_id: str, backend: Optional[str] = None) -> dict[str, Any]:
        model_id = str(model_id).strip()
        self._catalog = self._refresh_catalog()
        if model_id not in self._catalog:
            return {"ok": False, "error": f"Unknown model {model_id!r}"}

        entry = self._catalog[model_id]
        requested = str(backend or "").strip().lower()
        chosen = choose_backend(
            entry,
            requested=requested,
            global_backend=self._global_backend(),
            strict=bool(requested),
        )
        if not chosen:
            snap = self.snapshot()
            snap["ok"] = False
            snap["error"] = (
                f"{requested} is not available for {model_id!r}"
                if requested
                else f"model {model_id!r} has no runnable backend"
            )
            return snap

        try:
            from src.sub_state import get_sub_state

            get_sub_state().set_yolo_model_id(model_id)
        except Exception:
            pass

        with self._lock:
            try:
                self._load_model(model_id, backend=chosen)
                ok = True
                err = None
            except Exception as exc:
                ok = False
                err = str(exc)

        snap = self.snapshot()
        snap["ok"] = ok
        snap["error"] = err
        return snap

    def detect(self, frame):
        with self._lock:
            if self._detector is None:
                return []
            return self._detector.detect(frame)

    def detect_pair(self, left, right):
        with self._lock:
            if self._detector is None:
                return [], []
            return self._detector.detect_pair(left, right)
