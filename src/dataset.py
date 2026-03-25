"""
dataset.py
----------
Dataset download and path resolution helper.

Reads config/dataset.yaml and returns the local path to the Ultralytics-
compatible data.yaml file that train.py passes to model.train().

Swapping datasets for production
---------------------------------
Set  source: local  in config/dataset.yaml and point  local_path  to the
directory that already contains a data.yaml.  Typical workflow: unzip or mount
your YOLO dataset in Google Colab (or copy locally), then use local.

Roboflow (optional)
--------------------
If source is roboflow, set ROBOFLOW_API_KEY (env or .env). Never commit keys.

The .env file is loaded automatically if python-dotenv is installed.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

import yaml


CONFIG_PATH = Path(__file__).parent.parent / "config" / "dataset.yaml"
PROJECT_ROOT = Path(__file__).parent.parent


def _load_env() -> None:
    """Load .env file from project root if python-dotenv is available."""
    try:
        from dotenv import load_dotenv  # type: ignore
        load_dotenv(PROJECT_ROOT / ".env")
    except ImportError:
        pass


def _load_config(path: Path = CONFIG_PATH) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def get_dataset_yaml(config_path: Path = CONFIG_PATH) -> str:
    """
    Resolve or download the dataset and return the path to its data.yaml.

    Parameters
    ----------
    config_path : Path
        Path to the dataset config (defaults to config/dataset.yaml).

    Returns
    -------
    str
        Absolute path to the data.yaml file consumed by Ultralytics train().

    Raises
    ------
    FileNotFoundError
        If source is "local" and the data.yaml cannot be found.
    EnvironmentError
        If source is "roboflow" and ROBOFLOW_API_KEY is not set.
    """
    _load_env()
    cfg = _load_config(config_path)
    source: str = cfg.get("source", "local").lower()

    if source == "local":
        return _resolve_local(cfg)
    elif source == "roboflow":
        return _download_roboflow(cfg)
    else:
        raise ValueError(
            f"Unknown dataset source '{source}'. "
            "Expected 'roboflow' or 'local' in config/dataset.yaml."
        )


# ---------------------------------------------------------------------------
# Local source
# ---------------------------------------------------------------------------

def _resolve_local(cfg: dict) -> str:
    local_path = PROJECT_ROOT / cfg["local_path"]

    # Accept either the directory itself (containing data.yaml) or a
    # direct path to the data.yaml file.
    if local_path.is_dir():
        data_yaml = local_path / "data.yaml"
    elif local_path.suffix == ".yaml":
        data_yaml = local_path
    else:
        data_yaml = local_path / "data.yaml"

    if not data_yaml.is_file():
        raise FileNotFoundError(
            f"Local dataset data.yaml not found at: {data_yaml}\n"
            "Check the 'local_path' setting in config/dataset.yaml."
        )

    print(f"[Dataset] Using local dataset: {data_yaml}")
    return str(data_yaml)


# ---------------------------------------------------------------------------
# Roboflow source
# ---------------------------------------------------------------------------

def _download_roboflow(cfg: dict) -> str:
    api_key = cfg.get("api_key") or os.environ.get("ROBOFLOW_API_KEY", "")
    if not api_key:
        raise EnvironmentError(
            "Roboflow API key not found.\n"
            "Set the ROBOFLOW_API_KEY environment variable or add it to a .env file.\n"
            "You can also set 'api_key' directly in config/dataset.yaml (not recommended)."
        )

    try:
        from roboflow import Roboflow  # type: ignore
    except ImportError as exc:
        raise ImportError(
            "roboflow is not installed. Run: pip install roboflow"
        ) from exc

    workspace: str = cfg["workspace"]
    project_name: str = cfg["project"]
    version_num: int = int(cfg["version"])
    fmt: str = cfg.get("format", "yolov8")

    print(
        f"[Dataset] Downloading from Roboflow: "
        f"{workspace}/{project_name} v{version_num} ({fmt})"
    )

    rf = Roboflow(api_key=api_key)
    project = rf.workspace(workspace).project(project_name)
    version = project.version(version_num)

    # Download into data/<project_name>-<version>/
    download_dir = str(PROJECT_ROOT / "data")
    dataset = version.download(fmt, location=download_dir)

    data_yaml = Path(dataset.location) / "data.yaml"
    if not data_yaml.is_file():
        raise FileNotFoundError(
            f"Roboflow download completed but data.yaml not found at: {data_yaml}"
        )

    print(f"[Dataset] Dataset ready: {data_yaml}")
    return str(data_yaml)


# ---------------------------------------------------------------------------
# CLI helper
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    path = get_dataset_yaml()
    print(f"data.yaml path: {path}")
