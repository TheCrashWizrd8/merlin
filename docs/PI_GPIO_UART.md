# Pi GPIO UART — ESP telemetry (pins 14 / 15)

Use the **40-pin header UART** when the ESP is wired to the Pi instead of USB.

## Wiring

| Pi (BCM) | Header pin | Signal | Connect to ESP |
|----------|------------|--------|----------------|
| GPIO 14  | 8          | **TX** (Pi sends) | ESP **RX** (PI_UART_RX, default GPIO 18) |
| GPIO 15  | 10         | **RX** (Pi receives) | ESP **TX** (PI_UART_TX, default GPIO 17) |
| GND      | 6, 9, …    | GND | ESP **GND** |

Both sides are **3.3 V** UART — do not tie Pi 5 V to ESP unless your board needs it for power.

Cross the data lines: **Pi TX → ESP RX**, **Pi RX ← ESP TX**.

## One-time Pi setup

The kernel login console must **not** use `serial0`, or it will corrupt ESP traffic.

```bash
sudo bash scripts/setup_pi_uart.sh
sudo reboot
```

This script:

- Sets `enable_uart=1` in `/boot/firmware/config.txt` (or `/boot/config.txt`)
- Removes `console=serial0,...` from `cmdline.txt`
- Adds your user to the `dialout` group

## Software config

`config/hardware.yaml`:

```yaml
serial:
  port: "/dev/serial0"
  baud_rate: 115200
```

On **Raspberry Pi 5**, `/dev/serial0` is usually `ttyAMA10`. Same pins 14/15 on the header.

USB ESP link: set `port: "/dev/ttyACM0"` instead.

## Test

```bash
# ESP powered and sending TEL lines
python scripts/test_pi_uart.py

# Full dashboard
python sub_server.py
```

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| Permission denied on `/dev/serial0` | `sudo usermod -aG dialout $USER`, re-login |
| Garbled / no data | Wrong baud (use 115200 both sides), or console still on serial0 — re-run setup + reboot |
| Only kernel boot text | Remove `console=serial0` from cmdline |
| Open OK, no TEL | Check wiring (TX/RX crossed), ESP firmware using same UART pins |

Protocol: same `TEL …` telemetry and `B` / `S2` commands as USB — see `docs/SUB_DASHBOARD.md`.

ESP firmware: `esp32/sub_rc/sub_rc.ino` with `USE_PI_UART 1` and `PI_UART_RX` / `PI_UART_TX` matching your wiring. Flash after changing pins.
