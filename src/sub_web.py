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
from src.sub_state import get_sub_state

_DASHBOARD_PATH = Path(__file__).parent / "sub_dashboard.html"
_PINS_YAML = Path(__file__).parent.parent / "config" / "pins.yaml"

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


def _stream_payload(state) -> dict:
    """Combined dashboard snapshot for SSE push."""
    from src.model_runtime import models_dashboard_snapshot

    diag = state.diagnostics_snapshot()
    return {
        "telemetry": state.telemetry_snapshot(),
        "control": state.control_snapshot(),
        "models": models_dashboard_snapshot(),
        "status": state.status_snapshot(),
        "serial": {"lines": state.get_serial_log(200)},
        "pins": {
            "expected": _expected_pins_snapshot(),
            "esp_pins_lines": diag["esp_pins_lines"],
            "esp_pins_map": diag["esp_pins_map"],
            "last_pong_ts": diag["last_pong_ts"],
        },
        "diagnostics": diag,
        "version": state.get_version(),
    }


def _sse_event(event: str, payload: dict) -> str:
    return f"event: {event}\ndata: {json.dumps(payload, separators=(',', ':'))}\n\n"


def register_sub_routes(app, *, start_services: bool = True) -> None:
    """
    Add /sub/ dashboard and API routes to an existing Flask app.
    When start_services=True, starts ESP bridge and Xbox reader threads.
    """
    from flask import Response, jsonify, request

    state = get_sub_state()

    if start_services:
        bridge = get_esp_bridge(autostart=True)
        bridge.start()
        from src.xbox_controller import connect_xbox, is_xbox_enabled
        if is_xbox_enabled():
            connect_xbox()
        from src.gps_reader import connect_gps, is_gps_enabled
        if is_gps_enabled():
            connect_gps(esp_port=bridge.port)

    # ------------------------------------------------------------------
    # Dashboard
    # ------------------------------------------------------------------

    @app.route("/sub/")
    @app.route("/sub")
    def sub_dashboard():
        html = _DASHBOARD_PATH.read_text(encoding="utf-8")
        return Response(html, mimetype="text/html")

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
        min_interval_s = 0.033  # ~30 Hz cap
        heartbeat_s = 15.0

        def generate():
            since = -1
            last_push = 0.0
            last_heartbeat = time.monotonic()
            pending = True  # send full snapshot immediately on connect

            while True:
                version = state.wait_for_change(since, timeout=0.25)
                now = time.monotonic()

                if version != since:
                    pending = True

                if pending and (now - last_push >= min_interval_s):
                    payload = _stream_payload(state)
                    yield _sse_event("update", payload)
                    since = payload["version"]
                    last_push = now
                    pending = False
                    last_heartbeat = now
                elif now - last_heartbeat >= heartbeat_s:
                    yield ": heartbeat\n\n"
                    last_heartbeat = now

        return Response(
            generate(),
            mimetype="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
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
            mode = str(data["mode"])
            state.set_control_mode(mode)
            if mode in ("auto", "manual"):
                from src.control_source import set_mode as set_yolo_mode
                set_yolo_mode(mode)
        if any(k in data for k in ("aftSteerY", "aft_steer_y", "thrusterX")):
            act = clamp_actuators(parse_actuator_payload(data))
            state.set_manual_actuators(act)
            state.set_control_mode("manual")
        state.recompute_effective()
        get_esp_bridge().set_diag_mode(False)
        return jsonify(state.control_snapshot())

    @app.route("/sub/api/models", methods=["GET"])
    def sub_api_models_get():
        from src.model_runtime import models_dashboard_snapshot
        return jsonify(models_dashboard_snapshot())

    @app.route("/sub/api/models/select", methods=["POST"])
    def sub_api_models_select():
        from src.model_runtime import get_model_runtime

        data = request.get_json(silent=True) or {}
        model_id = str(data.get("id", "")).strip()
        if not model_id:
            return jsonify({"ok": False, "error": "missing id"}), 400

        runtime = get_model_runtime()
        if runtime is not None:
            result = runtime.select(model_id)
            code = 200 if result.get("ok") else 400
            return jsonify(result), code

        state.set_yolo_model_id(model_id)
        from src.model_runtime import catalog_snapshot, load_model_catalog, CONFIG_PATH
        catalog = load_model_catalog(CONFIG_PATH)
        if model_id not in catalog:
            return jsonify({"ok": False, "error": f"Unknown model {model_id!r}"}), 400
        entry = catalog[model_id]
        state.set_yolo_model_status({
            "state": "pending",
            "error": None,
            "task": entry.get("task"),
            "label": entry.get("label", model_id),
        })
        snap = catalog_snapshot(active_id=model_id, status=state.get_yolo_model_status())
        snap["ok"] = True
        snap["pending"] = True
        return jsonify(snap)

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
        if tank not in ("fore", "aft"):
            return jsonify({"ok": False, "error": "tank must be fore or aft"}), 400
        if end not in ("top", "bottom"):
            return jsonify({"ok": False, "error": "end must be top or bottom"}), 400
        bridge = get_esp_bridge()
        ok = bridge.send_raw(f"CAL B {tank} {end}")
        return jsonify({"ok": ok, "tank": tank, "end": end, "command": f"CAL B {tank} {end}"})

    @app.route("/sub/api/ballast/calibrate/resume", methods=["POST"])
    def sub_api_ballast_calibrate_resume():
        return jsonify({"ok": True})

    @app.route("/sub/api/control/actuators", methods=["POST"])
    def sub_api_actuators_post():
        data = request.get_json(silent=True) or {}
        act = clamp_actuators(parse_actuator_payload(data))
        state.set_manual_actuators(act)
        state.set_control_mode("manual")
        state.recompute_effective()
        get_esp_bridge().set_diag_mode(False)
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
