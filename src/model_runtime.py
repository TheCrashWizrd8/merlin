"""
model_runtime.py
----------------
Loadable YOLO model catalog (config/model.yaml) and thread-safe switching
while inference is running. The sub dashboard selects models in Auto mode.
"""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Any, Optional

import yaml

from src.detector import CONFIG_PATH, Detector, _resolve_weights_path, _exported_model_dir

PROJECT_ROOT = Path(__file__).parent.parent

_runtime: Optional["ModelRuntime"] = None


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
    return ""


def model_availability(
    entry: dict,
    backend: str,
) -> tuple[bool, Optional[str]]:
    weights = str(entry.get("weights") or "").strip()
    if not weights:
        return False, "no weights path in catalog"
    weights_path = _resolve_weights_path(weights)
    if not weights_path.is_file():
        return False, f"missing {weights}"
    if backend == "pytorch":
        return True, None
    if backend == "hailo":
        from src.hailo_runtime import hailo_export_dir, package_status

        return package_status(hailo_export_dir(weights_path))
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
) -> dict[str, Any]:
    cfg = _load_yaml(config_path)
    catalog = load_model_catalog(config_path)
    if not backend:
        backend = str(cfg.get("backend") or "pytorch").lower()
    if not active_id:
        active_id = resolve_active_model_id(catalog, cfg, backend=backend)
    models: list[dict] = []
    for model_id, entry in catalog.items():
        avail, reason = model_availability(entry, backend)
        models.append({
            "id": model_id,
            "label": str(entry.get("label") or model_id),
            "task": str(entry.get("task") or "auto"),
            "track_label": str(entry.get("track_label") or "apple"),
            "weights": str(entry.get("weights") or ""),
            "available": avail,
            "unavailable_reason": reason,
        })
    return {
        "active_id": active_id,
        "status": status or {"state": "idle", "error": None},
        "backend": backend,
        "models": models,
    }


def models_dashboard_snapshot() -> dict[str, Any]:
    """SSE/API snapshot; safe when inference has not started."""
    try:
        from src.sub_state import get_sub_state

        state = get_sub_state()
        active_id = state.get_yolo_model_id()
        status = state.get_yolo_model_status()
    except Exception:
        active_id = ""
        status = {"state": "idle", "error": None}
    runtime = get_model_runtime()
    if runtime is not None:
        active_id = runtime.active_id or active_id
        status = runtime.status_snapshot()
    return catalog_snapshot(active_id=active_id, status=status)


def get_model_runtime() -> Optional["ModelRuntime"]:
    return _runtime


def init_model_runtime(
    *,
    config_path: Path = CONFIG_PATH,
    backend: Optional[str] = None,
    imgsz_override: Optional[int] = None,
) -> "ModelRuntime":
    global _runtime
    _runtime = ModelRuntime(
        config_path=config_path,
        backend=backend,
        imgsz_override=imgsz_override,
    )
    return _runtime


class ModelRuntime:
    """Thread-safe wrapper: one Detector, hot-swapped from the catalog."""

    def __init__(
        self,
        config_path: Path = CONFIG_PATH,
        backend: Optional[str] = None,
        imgsz_override: Optional[int] = None,
    ) -> None:
        self.config_path = config_path
        self._backend_override = backend
        self._imgsz_override = imgsz_override
        self._lock = threading.RLock()
        self._cfg = _load_yaml(config_path)
        self._catalog = load_model_catalog(config_path)
        self._active_id = ""
        self._track_label = "apple"
        self._detector: Optional[Detector] = None
        self._status: dict[str, Any] = {"state": "idle", "error": None, "task": None}

        try:
            from src.sub_state import get_sub_state

            requested = get_sub_state().get_yolo_model_id()
        except Exception:
            requested = ""

        start_id = resolve_active_model_id(
            self._catalog,
            self._cfg,
            requested,
            backend=str(
                self._backend_override or self._cfg.get("backend") or "pytorch"
            ).lower(),
        )
        if start_id:
            try:
                self._load_model(start_id)
            except (FileNotFoundError, RuntimeError) as exc:
                fallback = first_available_model_id(
                    self._catalog,
                    str(
                        self._backend_override
                        or self._cfg.get("backend")
                        or "pytorch"
                    ).lower(),
                )
                if fallback and fallback != start_id:
                    print(
                        f"[ModelRuntime] Could not load {start_id!r} ({exc}); "
                        f"falling back to {fallback!r}"
                    )
                    self._load_model(fallback)
                else:
                    raise
        else:
            # Fallback: legacy single weights entry in model.yaml
            self._detector = Detector(
                config_path=config_path,
                backend=backend,
            )
            if imgsz_override:
                self._detector.img_size = int(imgsz_override)
            self._active_id = "default"
            self._track_label = str(self._cfg.get("track_label") or "apple")
            self._status = {
                "state": "ready",
                "error": None,
                "task": self._detector.task,
                "label": "Default",
            }
            self._sync_state()

    @property
    def active_id(self) -> str:
        return self._active_id

    @property
    def track_label(self) -> str:
        return self._track_label

    @property
    def img_size(self) -> int:
        if self._detector is None:
            return int(self._cfg.get("img_size") or 640)
        return self._detector.img_size

    @img_size.setter
    def img_size(self, value: int) -> None:
        if self._detector is not None:
            self._detector.img_size = int(value)

    def status_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._status)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            status = dict(self._status)
            active_id = self._active_id
        return catalog_snapshot(
            config_path=self.config_path,
            active_id=active_id,
            status=status,
            backend=str(
                self._detector.backend
                if self._detector
                else (self._backend_override or self._cfg.get("backend") or "pytorch")
            ).lower(),
        )

    def _sync_state(self) -> None:
        try:
            from src.sub_state import get_sub_state

            state = get_sub_state()
            state.set_yolo_model_id(self._active_id)
            state.set_yolo_model_status(dict(self._status))
        except Exception:
            pass

    def _entry_for(self, model_id: str) -> dict:
        if model_id not in self._catalog:
            raise KeyError(f"Unknown model {model_id!r}")
        return {"id": model_id, **self._catalog[model_id]}

    def _load_model(self, model_id: str) -> None:
        entry = self._entry_for(model_id)
        backend = (
            self._backend_override
            or self._cfg.get("backend")
            or "pytorch"
        )
        avail, reason = model_availability(entry, str(backend).lower())
        if not avail:
            raise FileNotFoundError(reason or f"model {model_id!r} unavailable")

        self._status = {
            "state": "loading",
            "error": None,
            "task": entry.get("task"),
            "label": entry.get("label", model_id),
        }
        self._sync_state()

        spec = dict(entry)
        spec["id"] = model_id
        if self._detector is None:
            detector = Detector(
                config_path=self.config_path,
                backend=self._backend_override,
                model_spec=spec,
            )
        else:
            self._detector.reload(spec)
            detector = self._detector

        if self._imgsz_override:
            detector.img_size = int(self._imgsz_override)

        self._detector = detector
        self._active_id = model_id
        self._track_label = str(entry.get("track_label") or "apple")
        self._status = {
            "state": "ready",
            "error": None,
            "task": detector.task,
            "label": str(entry.get("label") or model_id),
        }
        self._sync_state()
        print(
            f"[ModelRuntime] Active model={model_id!r} task={detector.task} "
            f"track_label={self._track_label!r}"
        )

    def apply_pending(self) -> None:
        """Reload if sub_state requested a different model."""
        try:
            from src.sub_state import get_sub_state

            wanted = get_sub_state().get_yolo_model_id()
        except Exception:
            return
        if not wanted or wanted == self._active_id:
            return
        entry = self._catalog.get(wanted)
        if entry is None:
            return
        backend = str(
            self._backend_override or self._cfg.get("backend") or "pytorch"
        ).lower()
        avail, reason = model_availability(entry, backend)
        if not avail:
            self._status = {
                "state": "error",
                "error": reason or f"model {wanted!r} unavailable",
                "task": entry.get("task"),
                "label": entry.get("label", wanted),
            }
            self._sync_state()
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
                }
                self._sync_state()
                print(f"[ModelRuntime] Failed to load {wanted!r}: {exc}")

    def select(self, model_id: str) -> dict[str, Any]:
        model_id = str(model_id).strip()
        if model_id not in self._catalog:
            return {"ok": False, "error": f"Unknown model {model_id!r}"}

        entry = self._catalog[model_id]
        backend = str(
            self._backend_override or self._cfg.get("backend") or "pytorch"
        ).lower()
        avail, reason = model_availability(entry, backend)
        if not avail:
            snap = self.snapshot()
            snap["ok"] = False
            snap["error"] = reason or f"model {model_id!r} unavailable"
            return snap

        try:
            from src.sub_state import get_sub_state

            get_sub_state().set_yolo_model_id(model_id)
        except Exception:
            pass

        with self._lock:
            try:
                self._load_model(model_id)
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
        self.apply_pending()
        with self._lock:
            if self._detector is None:
                raise RuntimeError("No YOLO model loaded")
            return self._detector.detect(frame)
