"""
sub_web.py
------------
Flask routes and sub dashboard UI. Register with register_sub_routes(app).
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import yaml

from src.esp_bridge import get_esp_bridge
from src.sub_control import clamp_actuators, parse_actuator_payload
from src.servo_center import (
    centers_snapshot,
    format_center_command,
    format_travel_save_command,
    write_servo_center_tick,
    write_servo_travel_tick,
)
from src.sub_state import get_sub_state

_DASHBOARD_PATH = Path(__file__).parent / "sub_dashboard.html"
_PINS_YAML = Path(__file__).parent.parent / "config" / "pins.yaml"
_PINS_TTL_S = 30.0
_pins_cache: tuple[float, dict] | None = None


def ballast_tank_cal_cleared(tank_snap: dict) -> bool:
    return (
        not bool(tank_snap.get("cal_valid"))
        and tank_snap.get("cal_top_adc") is None
        and tank_snap.get("cal_bottom_adc") is None
    )


def wait_ballast_cal_cleared(state, tanks: list[str], timeout_s: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        ballast = state.telemetry_snapshot()["ballast"]
        if all(ballast_tank_cal_cleared(ballast[t]) for t in tanks):
            return True
        time.sleep(0.05)
    return False


def _invalidate_pins_cache() -> None:
    global _pins_cache
    _pins_cache = None

# Pin confirmation checklist (matches esp32/sub_rc/sub_rc.ino diagnostics)
_PIN_CHECKLIST: list[tuple[str, str]] = [
    ("PING", "OK PONG"),
    ("PINS", "OK PINS"),
    ("TEST S 1 0", "OK TEST servo"),
    ("TEST S 2 0", "OK TEST servo"),
    ("TEST S 3 0", "OK TEST servo"),
    ("TEST S 4 0", "OK TEST servo"),
    ("TEST S 5 0", "OK TEST servo"),
    ("TEST T 0", "OK TEST thruster"),
    ("TEST B fore stop", "OK TEST ballast"),
    ("TEST B aft stop", "OK TEST ballast"),
    ("TEST L", "OK TEST leaks"),
    ("TEST A", "OK TEST adc"),
]


def _expected_pins_snapshot() -> dict:
    global _pins_cache
    now = time.monotonic()
    if _pins_cache is not None and now - _pins_cache[0] < _PINS_TTL_S:
        return _pins_cache[1]
    try:
        with open(_PINS_YAML) as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        cfg = {}
    snap = {
        "sub": cfg.get("sub") or {},
        "esp32": cfg.get("esp32") or {},
        "pca9685": cfg.get("pca9685") or {},
        "l298n": cfg.get("l298n") or {},
    }
    _pins_cache = (now, snap)
    return snap


def _release_servo_cal_for_live_control(state, bridge) -> None:
    """ESP ignores S2 while CAL S hold is active — exit cal when user drives manually."""
    if state.is_servo_cal_active():
        bridge.send_raw("CAL S DONE\n")
        state.set_servo_cal_active(False)
    bridge.set_diag_mode(False)


def _recent_rx_contains(state, needle: str, since_ts: float) -> bool:
    for entry in reversed(state.get_serial_log(30)):
        if entry["dir"] != "rx":
            continue
        if entry["ts"] < since_ts:
            break
        if needle in entry["line"]:
            return True
    return False


def _serial_highlights(lines: list[dict]) -> list[dict]:
    """Recent ESC / OK / ERR / actuator lines for a small pinned log."""
    keys = ("OK ESC", "ERR ESC", "OK TEST thruster", "S2 ", "OK S2", "ERR S2")
    out: list[dict] = []
    for entry in reversed(lines):
        line = str(entry.get("line", ""))
        if not any(k in line for k in keys):
            continue
        out.append(entry)
        if len(out) >= 24:
            break
    out.reverse()
    return out


def _stream_payload(state, *, heavy: bool = True) -> dict:
    """Dashboard snapshot for SSE. Fast ticks omit serial, pins, and map tracks."""
    from src.model_runtime import models_dashboard_snapshot

    payload = {
        "telemetry": state.telemetry_snapshot(include_tracks=heavy),
        "control": state.control_snapshot(),
        "status": state.status_snapshot(),
        "version": state.get_version(),
    }
    if not heavy:
        return payload

    diag = state.diagnostics_snapshot()
    payload["models"] = models_dashboard_snapshot()
    payload["serial"] = {
        "lines": state.get_serial_log(80),
        "highlights": _serial_highlights(state.get_serial_log(120)),
    }
    payload["pins"] = {
        "expected": _expected_pins_snapshot(),
        "esp_pins_lines": diag["esp_pins_lines"],
        "esp_pins_map": diag["esp_pins_map"],
        "last_pong_ts": diag["last_pong_ts"],
    }
    payload["diagnostics"] = diag
    return payload


def _sse_event(event: str, payload: dict) -> str:
    return f"event: {event}\ndata: {json.dumps(payload, separators=(',', ':'))}\n\n"


def register_sub_routes(app, *, start_services: bool = True) -> None:
    """
    Add /sub/ dashboard and API routes to an existing Flask app.
    When start_services=True, starts ESP bridge and Xbox reader threads.
    """
    from flask import Response, jsonify, request

    state = get_sub_state()
    state.set_servo_cal_active(False)

    if start_services:
        bridge = get_esp_bridge(autostart=True)
        bridge.start()
        from src.xbox_controller import connect_xbox, is_xbox_enabled
        if is_xbox_enabled():
            connect_xbox()
        from src.gps_reader import connect_gps, is_gps_enabled
        if is_gps_enabled():
            connect_gps(esp_port=bridge.port)
        from src.fault_indicator import connect_fault_indicator, is_fault_indicator_enabled
        if is_fault_indicator_enabled():
            connect_fault_indicator()

    # ------------------------------------------------------------------
    # Dashboard
    # ------------------------------------------------------------------

    @app.route("/sub/")
    @app.route("/sub")
    def sub_dashboard():
        html = _DASHBOARD_PATH.read_text(encoding="utf-8")
        return Response(
            html,
            mimetype="text/html",
            headers={
                "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
                "Pragma": "no-cache",
                "Expires": "0",
            },
        )

    # ------------------------------------------------------------------
    # Status & aggregated telemetry
    # ------------------------------------------------------------------

    @app.route("/sub/api/status")
    def sub_api_status():
        return jsonify(state.status_snapshot())

    @app.route("/sub/api/telemetry")
    def sub_api_telemetry():
        return jsonify(state.telemetry_snapshot())

    @app.route("/sub/api/stream")
    def sub_api_stream():
        """Server-Sent Events feed — pushes dashboard updates on state change."""
        min_interval_s = 0.1  # gauges, including ballast pots
        heavy_interval_s = 0.5  # serial log, pins, GPS track, sonar plot
        heartbeat_s = 15.0

        def generate():
            since = -1
            last_push = 0.0
            last_heavy = 0.0
            last_heartbeat = time.monotonic()
            pending = True  # send a snapshot immediately on connect

            while True:
                version = state.wait_for_change(since, timeout=0.25)
                now = time.monotonic()

                if version != since:
                    pending = True

                if pending and (now - last_push >= min_interval_s):
                    heavy = (now - last_heavy) >= heavy_interval_s
                    payload = _stream_payload(state, heavy=heavy)
                    yield _sse_event("update", payload)
                    since = payload["version"]
                    last_push = now
                    pending = False
                    last_heartbeat = now
                    if heavy:
                        last_heavy = now
                elif pending:
                    time.sleep(min_interval_s - (now - last_push))
                elif now - last_heartbeat >= heartbeat_s:
                    yield ": heartbeat\n\n"
                    last_heartbeat = now

        return Response(
            generate(),
            mimetype="text/event-stream",
            headers={
                "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
                "Pragma": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",
            },
        )

    @app.route("/sub/api/telemetry/battery")
    def sub_api_battery():
        snap = state.telemetry_snapshot()
        return jsonify({"timestamp": snap["timestamp"], **snap["battery"]})

    @app.route("/sub/api/telemetry/gyro")
    def sub_api_gyro():
        snap = state.telemetry_snapshot()
        return jsonify({"timestamp": snap["timestamp"], **snap["gyro"]})

    @app.route("/sub/api/telemetry/depth")
    def sub_api_depth():
        snap = state.telemetry_snapshot()
        return jsonify({"timestamp": snap["timestamp"], **snap["depth"]})

    @app.route("/sub/api/telemetry/sonar")
    def sub_api_sonar():
        snap = state.telemetry_snapshot()
        return jsonify({"timestamp": snap["timestamp"], **snap["sonar"]})

    @app.route("/sub/api/telemetry/gps")
    def sub_api_gps():
        snap = state.telemetry_snapshot()
        return jsonify({"timestamp": snap["timestamp"], **snap["gps"]})

    @app.route("/sub/api/gps/clear", methods=["POST"])
    def sub_api_gps_clear():
        state.clear_gps_track()
        return jsonify({"ok": True})

    @app.route("/sub/api/telemetry/leaks")
    def sub_api_leaks():
        snap = state.telemetry_snapshot()
        leaks = snap["leaks"]
        return jsonify({
            "timestamp": snap["timestamp"],
            "connected": leaks["connected"],
            "sensors": leaks["sensors"],
            "triggered": leaks["triggered"],
        })

    @app.route("/sub/api/telemetry/ballast")
    def sub_api_ballast_status():
        snap = state.telemetry_snapshot()
        b = snap["ballast"]
        return jsonify({
            "timestamp": snap["timestamp"],
            "level": b["level"],
            "command": b["command"],
            "connected": b["connected"],
        })

    @app.route("/sub/api/serial")
    def sub_api_serial():
        limit = request.args.get("limit", 100, type=int)
        limit = max(1, min(500, limit))
        lines = state.get_serial_log(limit)
        return jsonify({
            "lines": lines,
            "highlights": _serial_highlights(state.get_serial_log(max(limit, 300))),
        })

    # ------------------------------------------------------------------
    # Control (read)
    # ------------------------------------------------------------------

    @app.route("/sub/api/control", methods=["GET"])
    def sub_api_control_get():
        return jsonify(state.control_snapshot())

    @app.route("/sub/api/actuators", methods=["GET"])
    def sub_api_actuators_get():
        snap = state.control_snapshot()
        return jsonify({
            "mode": snap["mode"],
            "effective": snap["effective"],
            "auto": snap["auto"],
            "manual": snap["manual"],
            "xbox_mapped": snap["xbox_mapped"],
            "timestamp": snap["timestamp"],
        })

    @app.route("/sub/api/xbox", methods=["GET"])
    def sub_api_xbox():
        snap = state.control_snapshot()
        return jsonify(snap["xbox"])

    # ------------------------------------------------------------------
    # Control (write)
    # ------------------------------------------------------------------

    @app.route("/sub/api/control", methods=["POST"])
    def sub_api_control_post():
        data = request.get_json(silent=True) or {}
        if "mode" in data:
            mode = str(data["mode"])
            if mode in ("xbox", "manual"):
                from src.inference_service import get_inference_service

                inf = get_inference_service()
                if inf.snapshot().get("running"):
                    inf.stop(preserve_control_mode=True)
            state.set_control_mode(mode)
            from src.control_source import set_mode as set_yolo_mode

            if mode == "auto":
                set_yolo_mode("auto")
            else:
                set_yolo_mode("manual")
            state.recompute_effective()
        if any(k in data for k in ("aftSteerY", "aft_steer_y", "thrusterX")):
            act = clamp_actuators(parse_actuator_payload(data))
            state.set_manual_actuators(act)
            state.set_control_mode("manual")
        state.recompute_effective()
        bridge = get_esp_bridge()
        if str(data.get("mode", state.get_control_mode())) == "manual" or any(
            k in data for k in ("aftSteerY", "aft_steer_y", "thrusterX")
        ):
            _release_servo_cal_for_live_control(state, bridge)
        else:
            bridge.set_diag_mode(False)
        return jsonify(state.control_snapshot())

    @app.route("/sub/api/models", methods=["GET"])
    def sub_api_models_get():
        from src.inference_service import get_inference_service
        return jsonify(get_inference_service().snapshot())

    @app.route("/sub/api/models/select", methods=["POST"])
    def sub_api_models_select():
        from src.inference_service import get_inference_service

        data = request.get_json(silent=True) or {}
        model_id = str(data.get("id", "")).strip()
        backend = str(data.get("backend", "")).strip().lower()
        if not model_id:
            return jsonify({"ok": False, "error": "missing id"}), 400
        result = get_inference_service().start(model_id, backend)
        code = 200 if result.get("ok") else 400
        return jsonify(result), code

    @app.route("/sub/api/models/stop", methods=["POST"])
    def sub_api_models_stop():
        from src.inference_service import get_inference_service

        data = request.get_json(silent=True) or {}
        preserve = bool(data.get("preserve_control_mode"))
        return jsonify(get_inference_service().stop(preserve_control_mode=preserve))

    @app.route("/sub/api/vision/detections")
    def sub_api_vision_detections():
        from src.vision_state import vision_snapshot

        return jsonify(vision_snapshot())

    @app.route("/sub/api/vision/overlay")
    def sub_api_vision_overlay():
        """Alias for detection payload used by FOV bbox overlay (extrinsics TBD)."""
        from src.vision_state import vision_snapshot

        return jsonify(vision_snapshot())

    @app.route("/sub/api/control/ballast", methods=["POST"])
    def sub_api_ballast_post():
        data = request.get_json(silent=True) or {}
        tank = str(data.get("tank", "both")).lower()
        if tank not in ("fore", "aft", "both"):
            tank = "both"
        if "fore" in data or "aft" in data:
            fore = float(data.get("fore", state.get_ballast_commands()[0]))
            aft = float(data.get("aft", state.get_ballast_commands()[1]))
            state.set_ballast_commands(fore, aft)
        elif "action" in data:
            action = str(data["action"]).lower()
            if action == "fill":
                state.set_ballast_command(1.0, tank=tank)
            elif action == "drain":
                state.set_ballast_command(-1.0, tank=tank)
            elif action in ("neutral", "stop", "hold"):
                state.set_ballast_command(0.0, tank=tank)
        elif "value" in data:
            state.set_ballast_command(float(data["value"]), tank=tank)
        get_esp_bridge().set_diag_mode(False)
        return jsonify(state.telemetry_snapshot()["ballast"])

    @app.route("/sub/api/ballast/calibrate", methods=["POST"])
    def sub_api_ballast_calibrate():
        data = request.get_json(silent=True) or {}
        tank = str(data.get("tank", "fore")).lower()
        end = str(data.get("end", "")).lower()
        if tank not in ("fore", "aft", "both"):
            return jsonify({"ok": False, "error": "tank must be fore, aft, or both"}), 400
        if end in ("clear", "reset"):
            tanks = ["fore", "aft"] if tank == "both" else [tank]
            cmd = "CAL B both clear" if tank == "both" else f"CAL B {tank} clear"
            bridge = get_esp_bridge()
            ok = bridge.send_raw(cmd)
            if not ok:
                return jsonify({
                    "ok": False,
                    "tank": tank,
                    "end": "clear",
                    "command": cmd,
                    "error": "serial write failed",
                }), 503
            if not wait_ballast_cal_cleared(state, tanks):
                diag = state.diagnostics_snapshot().get("last_diagnostic") or ""
                return jsonify({
                    "ok": False,
                    "tank": tank,
                    "end": "clear",
                    "command": cmd,
                    "error": "ESP still has ballast calibration. Reflash sub_rc, then Clear Cal again.",
                    "last_diagnostic": diag,
                    "ballast": state.telemetry_snapshot()["ballast"],
                }), 409
            for name in tanks:
                state.clear_ballast_cal(name)
            return jsonify({"ok": True, "tank": tank, "end": "clear", "command": cmd})
        if end not in ("top", "bottom"):
            return jsonify({"ok": False, "error": "end must be top, bottom, or clear"}), 400
        bridge = get_esp_bridge()
        ok = bridge.send_raw(f"CAL B {tank} {end}")
        return jsonify({"ok": ok, "tank": tank, "end": end, "command": f"CAL B {tank} {end}"})

    @app.route("/sub/api/ballast/calibrate/resume", methods=["POST"])
    def sub_api_ballast_calibrate_resume():
        return jsonify({"ok": True})

    @app.route("/sub/api/servo/centers", methods=["GET"])
    def sub_api_servo_centers_get():
        snap = centers_snapshot()
        snap["cal_active"] = state.is_servo_cal_active()
        return jsonify(snap)

    @app.route("/sub/api/servo/centers/tune", methods=["POST"])
    def sub_api_servo_centers_tune():
        data = request.get_json(silent=True) or {}
        try:
            ch = int(data["channel"])
            tick = int(data.get("centre_tick", data.get("tick", 0)))
        except (KeyError, TypeError, ValueError):
            return jsonify({"ok": False, "error": "channel and centre_tick required"}), 400
        bridge = get_esp_bridge()
        ok = bridge.tune_servo_center(ch, tick)
        if ok:
            state.set_servo_cal_active(True)
        else:
            state.set_servo_cal_active(False)
        return jsonify({
            "ok": ok,
            "channel": ch,
            "centre_tick": tick,
            "cal_active": state.is_servo_cal_active(),
            "error": None if ok else "ESP serial send failed — is ~/sub running? Reflash sub_rc for CAL S TUNE / TEST P.",
        })

    @app.route("/sub/api/servo/centers/done", methods=["POST"])
    def sub_api_servo_centers_done():
        state.set_servo_cal_active(False)
        ok = get_esp_bridge().send_raw("CAL S DONE")
        return jsonify({"ok": ok, "cal_active": False})

    @app.route("/sub/api/servo/centers/save", methods=["POST"])
    def sub_api_servo_centers_save():
        data = request.get_json(silent=True) or {}
        try:
            ch = int(data["channel"])
            tick = int(data.get("centre_tick", data.get("tick", 0)))
        except (KeyError, TypeError, ValueError):
            return jsonify({"ok": False, "error": "channel and centre_tick required"}), 400
        try:
            saved = write_servo_center_tick(ch, tick)
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        _invalidate_pins_cache()
        ok = get_esp_bridge().send_raw(format_center_command(ch, int(saved["centre_tick"])))
        return jsonify({"ok": ok, **saved})

    @app.route("/sub/api/servo/limits/tune", methods=["POST"])
    def sub_api_servo_limits_tune():
        data = request.get_json(silent=True) or {}
        try:
            ch = int(data["channel"])
            end = str(data.get("end", data.get("side", ""))).lower()
            tick = int(data.get("tick", data.get("travel_tick", 0)))
        except (KeyError, TypeError, ValueError):
            return jsonify({"ok": False, "error": "channel, end (min|max), and tick required"}), 400
        if end not in ("min", "max"):
            return jsonify({"ok": False, "error": "end must be min or max"}), 400
        bridge = get_esp_bridge()
        ok = bridge.tune_servo_travel(ch, end, tick)
        if ok:
            state.set_servo_cal_active(True)
        else:
            state.set_servo_cal_active(False)
        return jsonify({
            "ok": ok,
            "channel": ch,
            "end": end,
            "tick": tick,
            "cal_active": state.is_servo_cal_active(),
            "error": None if ok else "ESP serial send failed — reflash sub_rc for CAL S TUNE MIN|MAX.",
        })

    @app.route("/sub/api/servo/limits/save", methods=["POST"])
    def sub_api_servo_limits_save():
        data = request.get_json(silent=True) or {}
        try:
            ch = int(data["channel"])
            end = str(data.get("end", data.get("side", ""))).lower()
            tick = int(data.get("tick", data.get("travel_tick", 0)))
        except (KeyError, TypeError, ValueError):
            return jsonify({"ok": False, "error": "channel, end (min|max), and tick required"}), 400
        try:
            saved = write_servo_travel_tick(ch, end, tick)
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        _invalidate_pins_cache()
        bridge = get_esp_bridge()
        ok = bridge.send_raw(
            format_travel_save_command(ch, str(saved["end"]), int(saved["tick"]))
        )
        # Push both endpoints so ESP max/min stay in sync with centre.
        t_min, t_max = int(saved["min_tick"]), int(saved["max_tick"])
        ok = bridge.send_raw(format_travel_save_command(ch, "min", t_min)) and ok
        ok = bridge.send_raw(format_travel_save_command(ch, "max", t_max)) and ok
        return jsonify({"ok": ok, **saved})

    @app.route("/sub/api/control/actuators", methods=["POST"])
    def sub_api_actuators_post():
        data = request.get_json(silent=True) or {}
        act = clamp_actuators(parse_actuator_payload(data))
        state.set_manual_actuators(act)
        state.set_control_mode("manual")
        state.recompute_effective()
        _release_servo_cal_for_live_control(state, get_esp_bridge())
        return jsonify(state.control_snapshot())

    @app.route("/sub/api/serial", methods=["POST"])
    def sub_api_serial_send():
        data = request.get_json(silent=True) or {}
        line = str(data.get("line", "")).strip()
        if not line:
            return jsonify({"ok": False, "error": "empty line"}), 400
        bridge = get_esp_bridge()
        ok = bridge.send_raw(line)
        return jsonify({"ok": ok})

    # ------------------------------------------------------------------
    # Pin diagnostics
    # ------------------------------------------------------------------

    @app.route("/sub/api/pins")
    def sub_api_pins():
        diag = state.diagnostics_snapshot()
        return jsonify({
            "expected": _expected_pins_snapshot(),
            "esp_pins_lines": diag["esp_pins_lines"],
            "esp_pins_map": diag["esp_pins_map"],
            "last_pong_ts": diag["last_pong_ts"],
        })

    @app.route("/sub/api/diagnostics")
    def sub_api_diagnostics():
        return jsonify(state.diagnostics_snapshot())

    @app.route("/sub/api/test", methods=["POST"])
    def sub_api_test():
        data = request.get_json(silent=True) or {}
        cmd = str(data.get("command", "")).strip()
        if not cmd:
            return jsonify({"ok": False, "error": "empty command"}), 400
        bridge = get_esp_bridge()
        bridge.set_diag_mode(True, resume_after_s=3.5)
        ok = bridge.send_raw(cmd)
        return jsonify({"ok": ok, "command": cmd})

    @app.route("/sub/api/test/resume", methods=["POST"])
    def sub_api_test_resume():
        get_esp_bridge().set_diag_mode(False)
        return jsonify({"ok": True})

    @app.route("/sub/api/test/run", methods=["POST"])
    def sub_api_test_run():
        bridge = get_esp_bridge()
        bridge.set_diag_mode(True)
        results: list[dict] = []
        try:
            for cmd, expect in _PIN_CHECKLIST:
                t0 = time.time()
                if not bridge.send_raw(cmd):
                    results.append({"command": cmd, "expect": expect, "ok": False, "error": "send failed"})
                    continue
                wait = 0.8 if cmd == "PINS" else 0.45
                time.sleep(wait)
                ok = _recent_rx_contains(state, expect, t0 - 0.05)
                if cmd == "PINS" and not ok:
                    ok = len(state.diagnostics_snapshot()["esp_pins_lines"]) > 0
                results.append({"command": cmd, "expect": expect, "ok": ok})
        finally:
            bridge.set_diag_mode(False)
        return jsonify({
            "results": results,
            "all_ok": all(r["ok"] for r in results),
        })


def register_sub_dashboard(app, start_services: bool = True) -> None:
    """Alias used by web_stream, sub_server, and inference --sub."""
    register_sub_routes(app, start_services=start_services)
