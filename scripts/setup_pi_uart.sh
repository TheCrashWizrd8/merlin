#!/usr/bin/env bash
# Enable Pi GPIO UART on pins 14 (TX) and 15 (RX) for ESP telemetry.
# Frees /dev/serial0 from the login console so sub_server / esp_bridge can use it.
#
# Run: sudo bash scripts/setup_pi_uart.sh
# Then reboot.

set -euo pipefail

if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
  echo "Run with sudo: sudo bash scripts/setup_pi_uart.sh"
  exit 1
fi

CONFIG=""
CMDLINE=""
for c in /boot/firmware/config.txt /boot/config.txt; do
  [[ -f "$c" ]] && CONFIG="$c" && break
done
for c in /boot/firmware/cmdline.txt /boot/cmdline.txt; do
  [[ -f "$c" ]] && CMDLINE="$c" && break
done

if [[ -z "$CONFIG" || -z "$CMDLINE" ]]; then
  echo "Could not find config.txt or cmdline.txt under /boot"
  exit 1
fi

echo "=== Pi GPIO UART (pins 14 TX / 15 RX) ==="
echo "config:  $CONFIG"
echo "cmdline: $CMDLINE"
echo

if ! grep -qE '^enable_uart=1' "$CONFIG"; then
  echo "Adding enable_uart=1 to $CONFIG"
  echo "enable_uart=1" >> "$CONFIG"
else
  echo "enable_uart=1 already set"
fi

if grep -qE 'console=serial0' "$CMDLINE"; then
  echo "Removing kernel console on serial0 (required for ESP link)"
  sed -i 's/console=serial0,[0-9]* //g' "$CMDLINE"
  sed -i 's/ console=serial0,[0-9]*//g' "$CMDLINE"
else
  echo "serial0 not used as kernel console — OK"
fi

REAL_USER="${SUDO_USER:-$USER}"
if [[ "$REAL_USER" != "root" ]]; then
  usermod -aG dialout "$REAL_USER" 2>/dev/null || true
  echo "Added $REAL_USER to group dialout (re-login if needed)"
fi

echo
echo "Wiring (3.3 V logic — cross TX/RX, common GND):"
echo "  Pi GPIO 14 (header pin  8) TX  →  ESP RX"
echo "  Pi GPIO 15 (header pin 10) RX  ←  ESP TX"
echo "  Pi GND                     ↔  ESP GND"
echo
echo "Software: config/hardware.yaml uses port /dev/serial0 @ 115200"
echo "Verify after reboot:"
echo "  ls -l /dev/serial0"
echo "  python scripts/test_pi_uart.py"
echo
echo "Reboot now for cmdline/config changes to apply."
