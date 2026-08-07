#!/usr/bin/env python3
"""
simulate_esp_serial.py
----------------------
Pretend to be the ESP32 over serial: sends TEL telemetry lines and prints
any B / S2 commands the Pi sends back.

Tests the full Pi serial path (esp_bridge -> sub_state -> dashboard).

Terminal 1:
  python scripts/simulate_esp_serial.py

Terminal 2:
  python sub_server.py --serial-port /tmp/sub_fake_pi --no-xbox

Open http://localhost:8080/sub/
"""

from __future__ import annotations

import argparse
import math
import os
import pty
import select
import shutil
import subprocess
import sys
import termios
import time


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Simulate ESP32 sub serial telemetry")
    p.add_argument(
        "--pi-port",
        default="/tmp/sub_fake_pi",
        help="Port path for sub_server --serial-port (default /tmp/sub_fake_pi)",
    )
    p.add_argument(
        "--esp-port",
        default="/tmp/sub_fake_esp",
        help="Port path this script opens (default /tmp/sub_fake_esp)",
    )
    p.add_argument("--hz", type=float, default=5.0, help="Telemetry rate in Hz")
    p.add_argument(
        "--leak-alarm",
        action="store_true",
        help="Toggle leak sensor 3 every 20s",
    )
    return p.parse_args()


def _clean_link(path: str) -> None:
    if os.path.islink(path) or os.path.exists(path):
        os.remove(path)


def setup_socat(pi_port: str, esp_port: str) -> subprocess.Popen | None:
    if not shutil.which("socat"):
        return None
    _clean_link(pi_port)
    _clean_link(esp_port)
    proc = subprocess.Popen(
        [
            "socat",
            f"PTY,link={pi_port},raw,echo=0",
            f"PTY,link={esp_port},raw,echo=0",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(0.4)
    if proc.poll() is not None:
        return None
    return proc


def setup_pty(pi_port: str, esp_port: str) -> tuple[int, int]:
    """Fallback: one PTY pair; Pi opens slave symlink, we keep both fds."""
    master, slave = pty.openpty()
    slave_path = os.ttyname(slave)
    _clean_link(pi_port)
    os.symlink(slave_path, pi_port)
    print(f"[sim-esp] PTY fallback: Pi -> {pi_port} -> {slave_path}")
    return master, slave


def open_port(path: str) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def build_telemetry(t: float, state: dict) -> list[str]:
    for key in ("fore", "aft"):
        lvl = state[f"{key}_level"]
        cmd = state[f"{key}_cmd"]
        lvl = max(0.0, min(1.0, lvl + cmd * 0.005))
        state[f"{key}_level"] = lvl
        state[f"{key}_moving"] = abs(cmd) > 0.05
        state[f"{key}_dir"] = "FILL" if cmd > 0.05 else "DRAIN" if cmd < -0.05 else "STOP"

    leak0 = state["leak0"]
    if state["leak_alarm"]:
        leak0 = int(t // 20) % 2 == 1
        state["leak0"] = leak0

    pitch = 5.0 * math.sin(t * 0.3)
    roll = 3.0 * math.sin(t * 0.17)
    state["yaw"] = (state["yaw"] + 1.2) % 360.0
    depth = 1.5 + 0.4 * math.sin(t * 0.1)
    battery = 12.6 - (t % 120.0) * 0.005
    thr = state.get("thruster", 0.0)
    hb = state.get("heartbeat", 0) + 1
    state["heartbeat"] = hb

    lines = [
        f"TEL battery {battery:.2f}",
        f"TEL gyro {pitch:.2f} {roll:.2f} {state['yaw']:.2f}",
        f"TEL depth {depth:.2f}",
        f"TEL leak {1 if leak0 else 0} 0 0 0",
    ]
    for key in ("fore", "aft"):
        lvl = state[f"{key}_level"]
        lines.append(
            f"TEL ballast {key} {lvl:.3f} {int(lvl * 4095)} "
            f"{1 if state[f'{key}_moving'] else 0} {state[f'{key}_dir']}"
        )
        cal = state[f"{key}_cal"]
        lines.append(
            f"TEL ballastcal {key} {cal['bottom']} {cal['top']} "
            f"{1 if cal['valid'] else 0}"
        )
    lines.extend([
        "TEL controls",
        f"{state.get('aft_y', 0.0):.3f}",
        f"{state.get('aft_z', 0.0):.3f}",
        f"{state.get('fin_l', 0.0):.3f}",
        f"{state.get('fin_r', 0.0):.3f}",
        f"{thr:.3f}",
        f"{state['fore_cmd']:.3f}",
        f"{state['aft_cmd']:.3f}",
        f"TEL thruster {thr:.3f} {int(abs(thr) * 255)}",
        "TEL status SIM",
        f"TEL fault {'LEAK' if leak0 else 'NONE'}",
        f"TEL heartbeat {hb}",
    ])
    return lines


def handle_command(line: str, state: dict) -> None:
    line = line.strip()
    if not line:
        return
    print(f"[sim-esp] << (Pi cmd) {line}")
    if line.startswith("B "):
        parts = line.split()
        try:
            if len(parts) >= 3:
                state["fore_cmd"] = float(parts[1])
                state["aft_cmd"] = float(parts[2])
            elif len(parts) >= 2:
                state["fore_cmd"] = state["aft_cmd"] = float(parts[1])
        except (IndexError, ValueError):
            pass
    elif line.startswith("CAL B "):
        parts = line.split()
        if len(parts) >= 4 and parts[2] in ("fore", "aft") and parts[3] in ("top", "bottom", "show"):
            tank = parts[2]
            which = parts[3]
            adc = int(state[f"{tank}_level"] * 4095)
            cal = state[f"{tank}_cal"]
            if which == "top":
                cal["top"] = adc
                print(f"[sim-esp] >> OK CAL B {tank} top {adc}")
            elif which == "bottom":
                cal["bottom"] = adc
                print(f"[sim-esp] >> OK CAL B {tank} bottom {adc}")
            elif which == "show":
                print(
                    f"[sim-esp] >> OK CAL B {tank} show "
                    f"{cal['bottom']} {cal['top']} {1 if cal['valid'] else 0}"
                )
            cal["valid"] = abs(cal["top"] - cal["bottom"]) >= 50
    elif line.startswith("S2 "):
        parts = line.split()
        try:
            state["aft_y"] = float(parts[1])
            state["aft_z"] = float(parts[2])
            fin_idx = parts.index("F")
            state["fin_l"] = float(parts[fin_idx + 1])
            state["fin_r"] = float(parts[fin_idx + 2])
            x_idx = parts.index("X")
            state["thruster"] = float(parts[x_idx + 1])
        except (IndexError, ValueError):
            state["last_cmd"] = line
    elif line.startswith("S ") and " D " in line:
        state["last_cmd"] = line


def _disable_echo(fd: int) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[3] &= ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def run_loop(fd: int, hz: float, state: dict, pi_port: str) -> None:
    interval = 1.0 / hz
    rx_buf = b""
    t0 = time.monotonic()
    next_tx = t0

    _disable_echo(fd)
    os.write(fd, b"sub_rc ready - waiting for B / S2 lines from Pi\n")
    print(f"[sim-esp] Waiting for Pi on {pi_port} ...")

    while True:
        now = time.monotonic()
        try:
            r, _, _ = select.select([fd], [], [], 0.05)
        except OSError:
            time.sleep(0.2)
            continue

        if fd in r:
            try:
                chunk = os.read(fd, 512)
            except OSError:
                time.sleep(0.2)
                continue
            if chunk:
                rx_buf += chunk
                while b"\n" in rx_buf:
                    raw, rx_buf = rx_buf.split(b"\n", 1)
                    handle_command(raw.decode("utf-8", errors="replace"), state)

        if now >= next_tx:
            next_tx = now + interval
            for line in build_telemetry(now - t0, state):
                os.write(fd, (line + "\n").encode("ascii"))
                if not line.startswith("TEL controls") and not line[0].isdigit() and "." not in line[:4]:
                    print(f"[sim-esp] >> (telemetry) {line}")


def main() -> int:
    args = parse_args()
    state = {
        "fore_level": 0.5,
        "aft_level": 0.4,
        "fore_cmd": 0.0,
        "aft_cmd": 0.0,
        "fore_cal": {"bottom": 500, "top": 3500, "valid": True},
        "aft_cal": {"bottom": 600, "top": 3400, "valid": True},
        "yaw": 0.0,
        "leak0": False,
        "leak_alarm": args.leak_alarm,
        "last_cmd": "",
        "aft_y": 0.0,
        "aft_z": 0.0,
        "fin_l": 0.0,
        "fin_r": 0.0,
        "thruster": 0.0,
        "heartbeat": 0,
    }

    socat_proc = setup_socat(args.pi_port, args.esp_port)
    slave_fd: int | None = None
    fd: int

    if socat_proc:
        print(f"[sim-esp] socat bridge: Pi={args.pi_port}  ESP={args.esp_port}")
        fd = open_port(args.esp_port)
        pi_port = args.pi_port
    else:
        master, slave_fd = setup_pty(args.pi_port, args.esp_port)
        fd = master
        pi_port = args.pi_port
        print("[sim-esp] socat not found; using PTY fallback")

    print("[sim-esp] In another terminal run:")
    print(f"  python sub_server.py --serial-port {pi_port} --no-xbox")
    print("[sim-esp] >> = telemetry to Pi  |  << = control from Pi")
    print("[sim-esp] Move sliders on dashboard to see B / S2 commands.")
    print("[sim-esp] Ctrl+C to stop.")

    try:
        run_loop(fd, args.hz, state, pi_port)
    except KeyboardInterrupt:
        print("\n[sim-esp] Stopped.")
    finally:
        os.close(fd)
        if slave_fd is not None:
            os.close(slave_fd)
        if socat_proc and socat_proc.poll() is None:
            socat_proc.terminate()
        for path in (args.pi_port, args.esp_port):
            if os.path.islink(path):
                os.remove(path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
