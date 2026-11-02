"""
fault_indicator.py
------------------
Pi GPIO → relay RCIN for leak/fault LEDs on the relay **NC** contact.

Updates on ``TEL leak`` / ``TEL fault`` only. Holds RCIN with ``gpioset`` or lgpio.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import yaml

CONFIG_PATH = _PROJECT_ROOT / "config" / "hardware.yaml"

_indicator: FaultIndicator | None = None
_indicator_lock = threading.Lock()


def _load_config() -> dict:
    try:
        with open(CONFIG_PATH) as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        cfg = {}
    return dict(cfg.get("fault_indicator") or {})


def fault_active_from_tel(
    *,
    leak: bool,
    esp_fault: str,
    sources: dict | None = None,
) -> bool:
    src = sources if sources is not None else (_load_config().get("sources") or {})
    if src.get("leak", True) and leak:
        return True
    if src.get("esp_fault", True):
        fault = str(esp_fault or "").strip().upper()
        if fault not in ("", "NONE", "UNKNOWN"):
            return True
    return False


def _gpio_chip_index(chip_name: str) -> int:
    name = chip_name.strip().lower()
    if name.startswith("gpiochip"):
        return int(name[len("gpiochip") :] or "0")
    return 0


def resolve_bcm_gpio(bcm: int, cfg: dict) -> tuple[str, int]:
    """
    Map header BCM number → (gpiochip name, line offset).
    libgpiod v2 has no ``gpiofind``; probe chips with ``gpioinfo``.
    """
    override_chip = str(cfg.get("gpio_chip") or "auto").strip().lower()
    if override_chip and override_chip != "auto":
        return override_chip, bcm
    if shutil.which("gpioinfo") and shutil.which("gpiodetect"):
        try:
            detect = subprocess.check_output(
                ["gpiodetect"], text=True, timeout=3
            )
            chip_lines = [ln for ln in detect.splitlines() if ln.strip()]
            rp1_chip: str | None = None
            chips: list[str] = []
            for ln in chip_lines:
                chip = ln.split()[0].rstrip(":")
                chips.append(chip)
                if "pinctrl-rp1" in ln.lower() or "rp1" in ln.lower():
                    rp1_chip = chip
            probe_order = []
            if rp1_chip:
                probe_order.append(rp1_chip)
            probe_order.extend(ch for ch in chips if ch not in probe_order)
            for chip in probe_order:
                ok = subprocess.run(
                    ["gpioinfo", "-c", chip, str(bcm)],
                    capture_output=True,
                    text=True,
                    timeout=3,
                )
                if ok.returncode == 0:
                    return chip, bcm
        except (subprocess.CalledProcessError, OSError, ValueError):
            pass
    if Path("/dev/gpiochip0").exists():
        return "gpiochip0", bcm
    return "gpiochip0", bcm


class FaultIndicator:
    def __init__(self, cfg: dict | None = None) -> None:
        cfg = cfg if cfg is not None else _load_config()
        self._enabled = bool(cfg.get("enabled", False))
        self._bcm = int(cfg.get("gpio", 18))
        self._chip_name, self._line_offset = resolve_bcm_gpio(self._bcm, cfg)
        idle = cfg.get("idle_rcin_level")
        fault = cfg.get("fault_rcin_level")
        self._idle_rcin: int | None = int(idle) if idle is not None else None
        self._fault_rcin: int | None = int(fault) if fault is not None else None
        self._hold_s = max(0.02, float(cfg.get("hold_refresh_s", 0.05)))
        self._sources = dict(cfg.get("sources") or {})
        self._leak = False
        self._sim_leak = False
        self._flash_hz = max(0.0, float(cfg.get("flash_hz", 0) or 0))
        self._flash_stop: threading.Event | None = None
        self._esp_fault = "NONE"
        self._fault_active = False
        self._held_level: int | None = None
        self._lock = threading.Lock()
        self._gpioset_proc: subprocess.Popen[bytes] | None = None
        self._lgpio_handle: int | None = None
        self._lgpio_chip_idx = _gpio_chip_index(self._chip_name)
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        backend = str(cfg.get("backend") or "auto").strip().lower()
        rc = dict(cfg.get("rc_pwm") or {})
        self._use_rc_pwm = backend == "rc_pwm"
        self._pwm_period_ns = int(float(rc.get("period_us", 20000)) * 1000)
        self._pwm_active_ns = int(float(rc.get("active_us", 2000)) * 1000)
        self._pwm_inactive_ns = int(float(rc.get("inactive_us", 1000)) * 1000)
        self._pwm_safe_start_s = float(rc.get("safe_start_s", 0.6))
        self._pwm_ok = False
        self._hw_chip: Path | None = None
        has_pinctrl = shutil.which("pinctrl") is not None
        has_gpioset = shutil.which("gpioset") is not None
        if self._use_rc_pwm:
            self._use_pinctrl = False
            self._use_gpioset = False
        elif backend == "pinctrl":
            self._use_pinctrl = has_pinctrl
            self._use_gpioset = False
        elif backend == "gpioset":
            self._use_pinctrl = False
            self._use_gpioset = has_gpioset
        else:
            # Pi OS: pinctrl drives BCM pins reliably; gpioset often silent-fails on Pi 5.
            self._use_pinctrl = has_pinctrl
            self._use_gpioset = has_gpioset and not has_pinctrl
        self._pinctrl_ok = False
        self._gpioset_gave_up = False
        self._gpioset_daemon = False
        self._boot_logged = False
        self._initialized = False
        self._fault_reason_logged = False
        self._consumer = "yolo_fault_indicator"

    @property
    def enabled(self) -> bool:
        return self._enabled

    def start(self) -> None:
        if not self._enabled:
            return
        with self._lock:
            self._initialized = True
            self._fault_active = False
            self._leak = False
            self._esp_fault = "NONE"
            idle_level = self._rcin_level_for_fault(False)
            self._apply_level(idle_level, log_change=False)
            if not self._use_rc_pwm and not self._use_pinctrl and (
                not self._use_gpioset or self._gpioset_gave_up
            ):
                self._start_lgpio_thread_if_needed()
        self._log_boot_once()

    def stop(self) -> None:
        self._stop_flash()
        self._release_rc_pwm()
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._release_gpioset()
        if self._lgpio_handle is not None:
            try:
                import lgpio

                lgpio.gpiochip_close(self._lgpio_handle)
            except Exception:
                pass
            self._lgpio_handle = None

    def apply_tel_leak(self, sensors: list[bool]) -> None:
        if not self._enabled:
            return
        self._ensure_initialized()
        with self._lock:
            self._leak = any(sensors)
            self._sync_output()

    def set_simulated_leak(self, active: bool) -> None:
        """Dashboard test button: behave as if a leak sensor tripped."""
        if not self._enabled:
            return
        self._ensure_initialized()
        with self._lock:
            self._sim_leak = bool(active)
            self._sync_output()

    def apply_tel_fault(self, fault: str) -> None:
        if not self._enabled:
            return
        self._ensure_initialized()
        with self._lock:
            self._esp_fault = str(fault).strip()
            self._sync_output()

    def _ensure_initialized(self) -> None:
        if not self._initialized:
            self.start()

    def _sync_output(self, *, log_change: bool = True) -> None:
        prev_fault = self._fault_active
        self._fault_active = fault_active_from_tel(
            leak=self._leak or self._sim_leak,
            esp_fault=self._esp_fault,
            sources=self._sources,
        )
        if self._fault_active and not prev_fault:
            print(
                f"[fault_indicator] fault ON — leak={self._leak} "
                f"esp_fault={self._esp_fault!r}",
                flush=True,
            )
            self._fault_reason_logged = True
        elif not self._fault_active and prev_fault:
            print("[fault_indicator] fault cleared — idle RCIN", flush=True)
        if self._fault_active and self._flash_hz > 0:
            self._start_flash()
            return
        self._stop_flash()
        level = self._rcin_level_for_fault(self._fault_active)
        self._apply_level(level, log_change=log_change)

    def _start_flash(self) -> None:
        if self._flash_stop is not None and not self._flash_stop.is_set():
            return
        stop = threading.Event()
        self._flash_stop = stop
        threading.Thread(
            target=self._flash_loop, args=(stop,), daemon=True, name="fault_flash"
        ).start()

    def _stop_flash(self) -> None:
        if self._flash_stop is not None:
            self._flash_stop.set()
            self._flash_stop = None

    def _flash_loop(self, stop: threading.Event) -> None:
        on = True
        while not stop.is_set():
            with self._lock:
                if stop.is_set():
                    return
                self._apply_level(self._rcin_level_for_fault(on), log_change=False)
            on = not on
            stop.wait(0.5 / self._flash_hz)

    def _rcin_level_for_fault(self, fault_active: bool) -> int:
        if self._idle_rcin is not None and self._fault_rcin is not None:
            return self._fault_rcin if fault_active else self._idle_rcin
        return 0 if fault_active else 1

    def _apply_level(self, level: int, *, log_change: bool = True) -> None:
        if level == self._held_level and self._output_alive():
            return
        prev = self._held_level
        self._held_level = level
        if self._use_rc_pwm:
            self._hold_with_rc_pwm(level)
        elif self._use_pinctrl:
            self._hold_with_pinctrl(level)
        elif self._use_gpioset and not self._gpioset_gave_up:
            self._hold_with_gpioset(level)
        else:
            if not self._open_lgpio():
                return
            self._write_lgpio(level)
            self._start_lgpio_thread_if_needed()
        if log_change and prev is not None and prev != level:
            if self._use_rc_pwm:
                where = f"BCM{self._bcm} rc_pwm"
            elif self._use_pinctrl:
                where = f"BCM{self._bcm} pinctrl"
            else:
                where = f"{self._chip_name} line {self._line_offset}"
            print(
                f"[fault_indicator] {where} level={level} "
                f"(fault={self._fault_active})",
                flush=True,
            )

    def _output_alive(self) -> bool:
        if self._use_rc_pwm:
            return self._pwm_ok
        if self._use_pinctrl:
            return self._pinctrl_ok
        if self._use_gpioset and not self._gpioset_gave_up:
            if self._gpioset_daemon:
                return True
            return self._gpioset_proc is not None and self._gpioset_proc.poll() is None
        return self._lgpio_handle is not None

    def _find_hw_pwm(self) -> Path | None:
        """Pi 5 PWM0 (reaches BCM18 as channel 2) once ``dtoverlay=pwm,pin=18,func=3`` is set."""
        for chip in sorted(Path("/sys/class/pwm").glob("pwmchip*")):
            try:
                if "1f00098000" in str((chip / "device").resolve()):
                    return chip
            except OSError:
                pass
        return None

    def _hold_with_hw_pwm(self, level: int) -> bool:
        chip = self._hw_chip or self._find_hw_pwm()
        if chip is None:
            return False
        chan = chip / "pwm2"
        width = self._pwm_active_ns if level else self._pwm_inactive_ns
        try:
            if self._hw_chip is None:
                if not chan.exists():
                    (chip / "export").write_text("2")
                # udev hands the new channel to the gpio group a moment after export
                for _ in range(150):
                    if (chan / "enable").exists() and os.access(chan / "enable", os.W_OK):
                        break
                    time.sleep(0.02)
                if shutil.which("pinctrl"):
                    # BCM18 -> PWM0_CHAN2 is alt function a3; other users of the pin can undo the overlay's mux
                    subprocess.run(
                        ["pinctrl", "set", str(self._bcm), "a3"],
                        capture_output=True,
                        timeout=5,
                    )
                (chan / "enable").write_text("0")
                (chan / "duty_cycle").write_text("0")
                (chan / "period").write_text(str(self._pwm_period_ns))
                # Safe-start: the switch needs to see an inactive pulse before it will activate.
                (chan / "duty_cycle").write_text(str(self._pwm_inactive_ns))
                (chan / "enable").write_text("1")
                if level and self._pwm_safe_start_s > 0:
                    time.sleep(self._pwm_safe_start_s)
                self._hw_chip = chip
                self._pwm_ok = True
            (chan / "duty_cycle").write_text(str(width))
            return True
        except OSError as exc:
            print(f"[fault_indicator] hardware pwm failed: {exc}", flush=True)
            self._hw_chip = None
            return False

    def _hold_with_rc_pwm(self, level: int) -> None:
        """Servo-style pulse train for an RC switch: long pulse = active, short = inactive.

        Pi 5 hardware PWM0 (the controller that reaches BCM18) is off in the stock
        device tree, so the pulses come from lgpio's tx_servo instead. The train
        stops when the process exits, which leaves the switch off (fault LEDs on).
        """
        if self._hold_with_hw_pwm(level):
            return
        width = int(round((self._pwm_active_ns if level else self._pwm_inactive_ns) / 1000))
        freq = max(1, int(round(1e9 / self._pwm_period_ns)))
        try:
            import lgpio

            if not self._pwm_ok:
                self._lgpio_handle = lgpio.gpiochip_open(self._lgpio_chip_idx)
                lgpio.gpio_claim_output(self._lgpio_handle, self._line_offset, 0)
                # Safe-start: the switch needs to see an inactive pulse before it will activate.
                lgpio.tx_servo(
                    self._lgpio_handle,
                    self._line_offset,
                    int(round(self._pwm_inactive_ns / 1000)),
                    freq,
                )
                if level and self._pwm_safe_start_s > 0:
                    time.sleep(self._pwm_safe_start_s)
                self._pwm_ok = True
            lgpio.tx_servo(self._lgpio_handle, self._line_offset, width, freq)
        except Exception as exc:
            self._pwm_ok = False
            print(f"[fault_indicator] rc_pwm failed: {exc}", flush=True)

    def _release_rc_pwm(self) -> None:
        if not self._use_rc_pwm or not self._pwm_ok:
            return
        try:
            if self._hw_chip is not None:
                (self._hw_chip / "pwm2" / "enable").write_text("0")
                (self._hw_chip / "unexport").write_text("2")
                self._hw_chip = None
            else:
                import lgpio

                lgpio.tx_servo(self._lgpio_handle, self._line_offset, 0)
        except Exception:
            pass
        self._pwm_ok = False

    def _hold_with_pinctrl(self, level: int) -> None:
        drive = "dh" if level else "dl"
        try:
            proc = subprocess.run(
                ["pinctrl", "set", str(self._bcm), "op", drive],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if proc.returncode == 0:
                self._pinctrl_ok = True
                return
            err = (proc.stderr or proc.stdout or "").strip()
            print(f"[fault_indicator] pinctrl failed: {err}", flush=True)
        except OSError as exc:
            print(f"[fault_indicator] pinctrl failed: {exc}", flush=True)
        self._use_pinctrl = False
        self._use_gpioset = shutil.which("gpioset") is not None
        if self._use_gpioset:
            self._hold_with_gpioset(level)
        else:
            self._open_lgpio()
            self._write_lgpio(level)
            self._start_lgpio_thread_if_needed()

    def _hold_with_gpioset(self, level: int) -> None:
        self._release_gpioset()
        args = [
            "gpioset",
            "-z",
            "-C",
            self._consumer,
            "-c",
            self._chip_name,
            f"{self._line_offset}={level}",
        ]
        try:
            proc = subprocess.run(
                args,
                capture_output=True,
                text=True,
                timeout=5,
            )
            if proc.returncode == 0:
                self._gpioset_daemon = True
                self._gpioset_proc = None
                return
            err = (proc.stderr or proc.stdout or "").strip()
            print(
                f"[fault_indicator] gpioset -z failed ({proc.returncode}): {err}",
                flush=True,
            )
        except OSError as exc:
            print(f"[fault_indicator] gpioset failed ({exc})", flush=True)

        # Fallback: blocking gpioset child (libgpiod v1 style)
        try:
            self._gpioset_daemon = False
            self._gpioset_proc = subprocess.Popen(
                ["gpioset", "-c", self._chip_name, f"{self._line_offset}={level}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            time.sleep(0.03)
            if self._gpioset_proc.poll() is None:
                return
        except OSError:
            pass

        self._gpioset_gave_up = True
        self._use_gpioset = False
        print(
            f"[fault_indicator] gpioset could not hold {self._chip_name} "
            f"line {self._line_offset} — trying lgpio",
            flush=True,
        )
        self._open_lgpio()
        self._write_lgpio(level)
        self._start_lgpio_thread_if_needed()

    def _release_gpioset(self) -> None:
        proc = self._gpioset_proc
        self._gpioset_proc = None
        if proc is not None:
            try:
                proc.terminate()
                proc.wait(timeout=1.0)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
        if self._gpioset_daemon:
            # Release daemonized line so the next gpioset can take over.
            subprocess.run(
                [
                    "gpioset",
                    "-z",
                    "-C",
                    self._consumer,
                    "-c",
                    self._chip_name,
                    f"{self._line_offset}=0",
                ],
                capture_output=True,
                timeout=3,
            )
            self._gpioset_daemon = False

    def _start_lgpio_thread_if_needed(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        if self._lgpio_handle is None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._lgpio_hold_loop,
            daemon=True,
            name="fault_indicator",
        )
        self._thread.start()

    def _open_lgpio(self) -> bool:
        if self._lgpio_handle is not None:
            return True
        try:
            import lgpio

            chip = lgpio.gpiochip_open(self._lgpio_chip_idx)
            idle = self._rcin_level_for_fault(False)
            lgpio.gpio_claim_output(chip, self._line_offset, l=idle)
            self._lgpio_handle = chip
            return True
        except Exception as exc:
            print(
                f"[fault_indicator] lgpio chip {self._lgpio_chip_idx} "
                f"line {self._line_offset} failed: {exc}",
                flush=True,
            )
            return False

    def _write_lgpio(self, level: int) -> None:
        handle = self._lgpio_handle
        if handle is None:
            return
        try:
            import lgpio

            lgpio.gpio_write(handle, self._line_offset, level)
        except Exception as exc:
            print(f"[fault_indicator] lgpio write failed: {exc}", flush=True)

    def _lgpio_hold_loop(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                if self._held_level is not None:
                    self._write_lgpio(self._held_level)
            self._stop.wait(self._hold_s)

    def _log_boot_once(self) -> None:
        if self._boot_logged:
            return
        self._boot_logged = True
        if self._use_rc_pwm:
            mode = "rc_pwm"
        elif self._use_pinctrl:
            mode = "pinctrl"
        elif self._use_gpioset and not self._gpioset_gave_up:
            mode = "gpioset"
        else:
            mode = "lgpio hold"
        idle = self._idle_rcin if self._idle_rcin is not None else "?"
        fault = self._fault_rcin if self._fault_rcin is not None else "?"
        alive = self._output_alive()
        print(
            f"[fault_indicator] BCM{self._bcm} ({mode}, idle={idle} fault={fault}, "
            f"drive_ok={alive})",
            flush=True,
        )
        if not alive:
            print(
                "[fault_indicator] WARNING: GPIO not held — NC LEDs may stay on. "
                "Install gpiod: sudo apt install gpiod",
                flush=True,
            )


def get_fault_indicator(autostart: bool = False) -> FaultIndicator:
    global _indicator
    with _indicator_lock:
        if _indicator is None:
            _indicator = FaultIndicator()
        if autostart:
            _indicator.start()
        return _indicator


def connect_fault_indicator(*, autostart: bool = True) -> FaultIndicator:
    ind = get_fault_indicator(autostart=False)
    if autostart:
        ind.start()
    return ind


def is_fault_indicator_enabled() -> bool:
    return bool(_load_config().get("enabled", False))


def set_simulated_leak(active: bool) -> None:
    if not is_fault_indicator_enabled():
        return
    get_fault_indicator(autostart=True).set_simulated_leak(active)


def notify_tel_leak(sensors: list[bool]) -> None:
    if not is_fault_indicator_enabled():
        return
    get_fault_indicator(autostart=True).apply_tel_leak(sensors)


def notify_tel_fault(fault: str) -> None:
    if not is_fault_indicator_enabled():
        return
    get_fault_indicator(autostart=True).apply_tel_fault(fault)
