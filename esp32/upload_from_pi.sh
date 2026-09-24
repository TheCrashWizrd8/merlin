#!/bin/bash
# Build and upload an ESP32 sketch from the Raspberry Pi.
# Run from the yolo-project root:
#   bash esp32/upload_from_pi.sh
#   bash esp32/upload_from_pi.sh scan
#   bash esp32/upload_from_pi.sh [BOARD] [PORT]
#   bash esp32/upload_from_pi.sh [BOARD] [PORT] [SKETCH]
#   bash esp32/upload_from_pi.sh --sketch sub_hw_test
# Prerequisites: Arduino CLI and ESP32 board support installed (see esp32/README.md).

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_SKETCH="sub_rc"
SKETCH_NAME="${DEFAULT_SKETCH}"

if ! command -v arduino-cli &>/dev/null; then
  echo "arduino-cli not found. Install it first (see esp32/README.md)."
  exit 1
fi

cd "${PROJECT_ROOT}"

# --- Scan mode: just list ports and exit ---
if [ "${1:-}" = "scan" ] || [ "${1:-}" = "list" ] || [ "${1:-}" = "--list-ports" ]; then
  echo "=== Serial ports (arduino-cli board list) ==="
  arduino-cli board list
  echo ""
  echo "=== USB serial devices (/dev/ttyACM* /dev/ttyUSB*) ==="
  shopt -s nullglob 2>/dev/null || true
  for d in /dev/ttyACM* /dev/ttyUSB*; do
    echo "  $d"
  done
  shopt -u nullglob 2>/dev/null || true
  echo ""
  echo "Use one of the above ports for upload, e.g.:"
  echo "  bash esp32/upload_from_pi.sh esp32:esp32:esp32s3 /dev/ttyACM0"
  exit 0
fi

# --- Parse optional flags ---
FREE_PORT="${FREE_PORT:-0}"
while [ "${1:-}" = "--free-port" ] || [ "${1:-}" = "--free" ]; do
  FREE_PORT=1
  shift
done

if [ "${1:-}" = "--sketch" ]; then
  SKETCH_NAME="${2:-}"
  shift 2
fi

# --- Build and upload ---
BOARD="${1:-esp32:esp32:esp32s3}"
PORT="${2:-}"
if [ -n "${3:-}" ]; then
  SKETCH_NAME="${3}"
fi

if [ -z "${SKETCH_NAME}" ]; then
  echo "Missing sketch name."
  echo "Examples:"
  echo "  bash esp32/upload_from_pi.sh --sketch sub_hw_test"
  echo "  bash esp32/upload_from_pi.sh esp32:esp32:esp32s3 /dev/ttyACM0 sub_rc"
  exit 1
fi

SKETCH_PATH="esp32/${SKETCH_NAME}"
if [ ! -d "${SKETCH_PATH}" ]; then
  echo "Sketch folder not found: ${SKETCH_PATH}"
  echo "Available:"
  ls -1d esp32/*/ 2>/dev/null | sed 's|esp32/||;s|/$||' | awk '{print "  " $1}'
  exit 1
fi

# ESP32-S3 USB-Serial/JTAG drops at 921600. DebugLevel=none stops i2c-ng log flood
# (must be on the compile FQBN — it is a build flag, not upload-only).
UPLOAD_SPEED="${UPLOAD_SPEED:-115200}"
if [[ "${BOARD}" == esp32:esp32:esp32s3* ]]; then
  if [[ "${BOARD}" != *"UploadSpeed="* ]]; then
    BOARD="${BOARD}:UploadSpeed=${UPLOAD_SPEED},DebugLevel=none"
  elif [[ "${BOARD}" != *"DebugLevel="* ]]; then
    BOARD="${BOARD},DebugLevel=none"
  fi
fi

echo "Building sketch: ${SKETCH_PATH} (clean build, fqbn ${BOARD})"
BUILD_FLAGS=()
# ESP32-S3: ensure Serial is routed to USB CDC on boot.
if [[ "${BOARD}" == esp32:esp32:esp32s3* ]]; then
  BUILD_FLAGS+=(--build-property 'build.extra_flags=-DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1 -DESP32=1')
fi
arduino-cli compile --clean --fqbn "${BOARD}" "${BUILD_FLAGS[@]}" "${SKETCH_PATH}"

if [ -z "${PORT}" ]; then
  echo "Scanning for USB serial port (ttyACM / ttyUSB)..."
  for _ in 1 2 3; do
    PORT=$(arduino-cli board list 2>/dev/null | awk '/ttyACM|ttyUSB/{print $1; exit}')
    [ -n "${PORT}" ] && break
    [ -c /dev/ttyACM0 ] && PORT="/dev/ttyACM0" && break
    [ -c /dev/ttyUSB0 ] && PORT="/dev/ttyUSB0" && break
    echo "  Retrying port detection..."
    sleep 1
  done
  if [ -z "${PORT}" ]; then
    echo "No ESP32 port found. Plug in the board via USB and run:"
    echo "  bash esp32/upload_from_pi.sh scan"
    echo "Or pass the port explicitly:  bash esp32/upload_from_pi.sh esp32:esp32:esp32s3 /dev/ttyACM0"
    exit 1
  fi
  echo "Using port: ${PORT}"
else
  if [[ "${PORT}" == *"ttyAMA"* ]]; then
    echo "Warning: ${PORT} is the Pi's built-in UART, not USB. For ESP32 over USB use /dev/ttyACM0 or /dev/ttyUSB0."
    echo "Run:  bash esp32/upload_from_pi.sh scan"
  fi
fi

if command -v fuser &>/dev/null && [ -n "${PORT}" ]; then
  port_holders() {
    fuser "${PORT}" 2>/dev/null || true
  }
  holders=$(port_holders)
  if [ -n "${holders}" ]; then
    if [ "${FREE_PORT}" = "1" ]; then
      echo "Stopping processes on ${PORT} (${holders})..."
      for pat in run.py sub_server.py sub_server sub; do
        pkill -f "${pat}" 2>/dev/null || true
      done
      sleep 1
      for _ in 1 2 3 4 5 6 7 8 9 10; do
        holders=$(port_holders)
        [ -z "${holders}" ] && break
        sleep 0.5
      done
      holders=$(port_holders)
    fi
    if [ -n "${holders}" ]; then
      echo "Error: ${PORT} is still busy (${holders})."
      echo "  pkill -f 'run.py|sub'"
      echo "  pkill -f sub_server.py"
      echo "Or retry with:  bash esp32/upload_from_pi.sh --free-port ${BOARD} ${PORT} ${SKETCH_NAME}"
      exit 1
    fi
    echo "Port ${PORT} is free."
  fi
fi

UPLOAD_RETRIES="${UPLOAD_RETRIES:-3}"
echo "Uploading to ${PORT} at ${UPLOAD_SPEED} baud (USB-Serial/JTAG)..."
upload_ok=0
for attempt in $(seq 1 "${UPLOAD_RETRIES}"); do
  if [ "${attempt}" -gt 1 ]; then
    echo "Retry upload attempt ${attempt}/${UPLOAD_RETRIES} (hold BOOT if this keeps failing)..."
    sleep 2
  fi
  if arduino-cli upload -p "${PORT}" --fqbn "${BOARD}" "${SKETCH_PATH}"; then
    upload_ok=1
    break
  fi
done
if [ "${upload_ok}" -ne 1 ]; then
  echo "Upload failed after ${UPLOAD_RETRIES} attempts."
  echo "Tips: --free-port, UPLOAD_SPEED=57600, unplug/replug USB, hold BOOT during upload."
  exit 1
fi

echo "Done. ESP32 should be running; unplug and replug USB if the serial port was in use."
