"""
sub_web.py
------------
Flask routes and sub dashboard UI. Register with register_sub_routes(app).
"""

from __future__ import annotations

import time
from pathlib import Path

import yaml

from src.esp_bridge import get_esp_bridge
from src.sub_control import clamp_actuators, parse_actuator_payload
from src.sub_state import get_sub_state
from src.xbox_controller import get_xbox_controller

_DASHBOARD_HTML = (Path(__file__).parent / "sub_dashboard.html").read_text(encoding="utf-8")
_PINS_YAML = Path(__file__).parent.parent / "config" / "pins.yaml"

# Pin confirmation checklist (matches esp32/sub_rc/sub_rc.ino diagnostics)
_PIN_CHECKLIST: list[tuple[str, str]] = [
    ("PING", "OK PONG"),
    ("PINS", "OK PINS"),
    ("TEST S 0 0", "OK TEST S"),
    ("TEST S 1 0", "OK TEST S"),
    ("TEST S 2 0", "OK TEST S"),
    ("TEST S 3 0", "OK TEST S"),
    ("TEST T 0", "OK TEST T"),
    ("TEST B stop", "OK TEST B"),
    ("TEST L", "OK LEAK"),
    ("TEST A", "OK ADC"),
]


def _expected_pins_snapshot() -> dict:
    try:
        with open(_PINS_YAML) as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        cfg = {}
    return {
        "sub": cfg.get("sub") or {},
        "esp32": cfg.get("esp32") or {},
        "pca9685": cfg.get("pca9685") or {},
        "l298n": cfg.get("l298n") or {},
    }


def _recent_rx_contains(state, needle: str, since_ts: float) -> bool:
    for entry in reversed(state.get_serial_log(30)):
        if entry["dir"] != "rx":
            continue
        if entry["ts"] < since_ts:
            break
        if needle in entry["line"]:
            return True
    return False


def register_sub_routes(app, *, start_services: bool = True) -> None:
    """
    Add /sub/ dashboard and API routes to an existing Flask app.
    When start_services=True, starts ESP bridge and Xbox reader threads.
    """
    from flask import Response, jsonify, request

    state = get_sub_state()

    if start_services:
        get_esp_bridge(autostart=True).start()
        get_xbox_controller(autostart=True).start()

    # ------------------------------------------------------------------
    # Dashboard
    # ------------------------------------------------------------------

    @app.route("/sub/")
    @app.route("/sub")
    def sub_dashboard():
        return Response(_DASHBOARD_HTML, mimetype="text/html")

    # ------------------------------------------------------------------
    # Status & aggregated telemetry
    # ------------------------------------------------------------------

    @app.route("/sub/api/status")
    def sub_api_status():
        return jsonify(state.status_snapshot())

    @app.route("/sub/api/telemetry")
    def sub_api_telemetry():
        return jsonify(state.telemetry_snapshot())

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
        return jsonify({"lines": state.get_serial_log(limit)})

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
            state.set_control_mode(str(data["mode"]))
        if any(k in data for k in ("aftSteerY", "aft_steer_y", "thrusterX")):
            act = clamp_actuators(parse_actuator_payload(data))
            state.set_manual_actuators(act)
            state.set_control_mode("manual")
        state.recompute_effective()
        return jsonify(state.control_snapshot())

    @app.route("/sub/api/control/ballast", methods=["POST"])
    def sub_api_ballast_post():
        data = request.get_json(silent=True) or {}
        if "action" in data:
            action = str(data["action"]).lower()
            if action == "fill":
                state.set_ballast_command(1.0)
            elif action == "drain":
                state.set_ballast_command(-1.0)
            elif action in ("neutral", "stop", "hold"):
                state.set_ballast_command(0.0)
        elif "value" in data:
            state.set_ballast_command(float(data["value"]))
        return jsonify(state.telemetry_snapshot()["ballast"])

    @app.route("/sub/api/control/actuators", methods=["POST"])
    def sub_api_actuators_post():
        data = request.get_json(silent=True) or {}
        act = clamp_actuators(parse_actuator_payload(data))
        state.set_manual_actuators(act)
        state.set_control_mode("manual")
        state.recompute_effective()
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
        ok = bridge.send_raw(cmd)
        return jsonify({"ok": ok, "command": cmd})

    @app.route("/sub/api/test/run", methods=["POST"])
    def sub_api_test_run():
        bridge = get_esp_bridge()
        results: list[dict] = []
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
        return jsonify({
            "results": results,
            "all_ok": all(r["ok"] for r in results),
        })
