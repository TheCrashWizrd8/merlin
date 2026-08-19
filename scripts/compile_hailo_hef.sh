#!/usr/bin/env bash
# Compile Hailo-8L HEF files. Run this on a Linux x86_64 PC with Hailo DFC 3.x.
#
# Preferred: Ultralytics format=hailo (needs hailo_sdk_client from the DFC wheel).
# Fallback: hailomz (Hailo Model Zoo) against the ONNX in weights/*_hailo_model/.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

ARCH="$(uname -m)"
if [[ "$ARCH" != "x86_64" ]]; then
  cat <<EOF
This script compiles .hef files. It must run on Linux x86_64, not $ARCH.

On the PC:
  1. Copy this project (or at least weights/best.pt and weights/gatebest.pt).
  2. Install Hailo Dataflow Compiler 3.x from https://hailo.ai/developer-zone/
     (Hailo-8 / Hailo-8L — matches HailoRT 4.23 on the Pi).
  3. pip install ultralytics
     pip install /path/to/hailo_dataflow_compiler-*.whl
  4. bash scripts/compile_hailo_hef.sh
  5. Copy weights/best_hailo_model/ and weights/gatebest_hailo_model/
     back to the Pi (they must contain a .hef).
  6. On the Pi: set backend: hailo in config/model.yaml

ONNX-only folders on the Pi are compiler input, not a runnable Hailo model.
EOF
  exit 1
fi

python_bin="${PYTHON:-python3}"
if [[ -x "$ROOT/.venv/bin/python" ]]; then
  python_bin="$ROOT/.venv/bin/python"
fi

if "$python_bin" -c "from hailo_sdk_client import ClientRunner" >/dev/null 2>&1; then
  echo "Using Ultralytics + Hailo DFC (hailo8l)"
  "$python_bin" "$ROOT/scripts/export_model.py" --format hailo --weights "$ROOT/weights/best.pt"
  "$python_bin" "$ROOT/scripts/export_model.py" --format hailo --weights "$ROOT/weights/gatebest.pt"
  echo "Copy weights/*_hailo_model/ (with .hef) to the Pi and set backend: hailo"
  exit 0
fi

if ! command -v hailomz >/dev/null 2>&1; then
  cat <<EOF
Neither hailo_sdk_client nor hailomz is available.

Install Hailo DFC 3.x from https://hailo.ai/developer-zone/ then either:
  pip install /path/to/hailo_dataflow_compiler-*.whl
  python scripts/export_model.py --format hailo --weights weights/best.pt
or install hailo-model-zoo so hailomz is on PATH, put 200–1024 calib JPEGs
in ./calib (or set CALIB_PATH), and re-run this script.
EOF
  exit 1
fi

CALIB="${CALIB_PATH:-$ROOT/calib}"
if [[ ! -d "$CALIB" ]]; then
  echo "Calibration folder missing: $CALIB"
  echo "Put 200+ representative JPEGs there (or set CALIB_PATH)."
  exit 1
fi

DETECT_ONNX="$ROOT/weights/best_hailo_model/best.onnx"
GATE_ONNX="$ROOT/weights/gatebest_hailo_model/gatebest.onnx"
if [[ ! -f "$DETECT_ONNX" || ! -f "$GATE_ONNX" ]]; then
  echo "ONNX missing. On the Pi first run:"
  echo "  python scripts/export_model.py --format hailo --weights weights/best.pt"
  echo "  python scripts/export_model.py --format hailo --weights weights/gatebest.pt"
  echo "Then copy those *_hailo_model folders here."
  exit 1
fi

echo "Using hailomz (hw-arch hailo8l)"
hailomz compile yolov8n \
  --ckpt "$DETECT_ONNX" \
  --hw-arch hailo8l \
  --classes 2 \
  --calib-path "$CALIB"

hailomz compile yolov8sseg \
  --ckpt "$GATE_ONNX" \
  --hw-arch hailo8l \
  --classes 1 \
  --calib-path "$CALIB"

echo
echo "Copy the compiled .hef files into:"
echo "  weights/best_hailo_model/     (detect)"
echo "  weights/gatebest_hailo_model/ (gate)"
echo "on the Pi, then set backend: hailo"
