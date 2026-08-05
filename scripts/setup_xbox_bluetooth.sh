#!/usr/bin/env bash
# Pair a Bluetooth Xbox controller on Raspberry Pi and install xpadneo if needed.
# Run: bash scripts/setup_xbox_bluetooth.sh

set -euo pipefail

echo "=== Xbox Bluetooth setup (Raspberry Pi) ==="
echo

# input group — required to read /dev/input/js*
if ! groups "$USER" | grep -q '\binput\b'; then
  echo "Adding $USER to group 'input' (log out/in after this)..."
  sudo usermod -aG input "$USER"
fi

# xpadneo — Xbox One / Series / Elite Bluetooth
if ! lsmod | grep -q hid_xpadneo; then
  echo
  echo "xpadneo driver not loaded."
  echo "Xbox One/Series controllers over Bluetooth need it on Linux."
  read -r -p "Install xpadneo now? [y/N] " ans
  if [[ "${ans,,}" == "y" ]]; then
    tmp=$(mktemp -d)
    git clone https://github.com/atar-axis/xpadneo.git "$tmp/xpadneo"
    (cd "$tmp/xpadneo" && sudo ./install.sh)
    rm -rf "$tmp"
    echo "Reboot recommended after first xpadneo install."
  fi
fi

echo
echo "=== Pair controller ==="
echo "1. Hold the Xbox sync button until the logo flashes"
echo "2. In bluetoothctl: scan on → pair <MAC> → trust <MAC> → connect <MAC>"
echo
read -r -p "Open interactive bluetoothctl now? [y/N] " ans
if [[ "${ans,,}" == "y" ]]; then
  bluetoothctl
fi

echo
echo "Verify with: python scripts/test_xbox.py --watch"
echo "Then run:    python sub_server.py   (no --no-xbox)"
