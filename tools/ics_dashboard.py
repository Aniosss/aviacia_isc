from __future__ import annotations

import json
import math
import secrets
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlsplit

from ics_pid_controller import ClearWeatherILSController, ControlResult
from ics_protocol import ICSInputs


HTML_PATH = Path(__file__).with_name("ics_dashboard.html")


class DashboardState:
    def __init__(self, controller: ClearWeatherILSController) -> None:
        self.controller = controller
        self.points: list[dict[str, Any]] = []
        self.next_sequence = 0
        self.run_id = secrets.token_hex(8)
        self.lock = threading.Lock()

    def record(self, time_s: float, state: ICSInputs, result: ControlResult) -> None:
        point = {
            "seq": self.next_sequence,
            "t": round(time_s, 3),
            "roll": {
                "value": state.RollAngle,
                "setpoint": result.target_roll_deg,
                "output": result.aileron,
                "integral": self.controller.roll_pid.config.ki * self.controller.roll_pid.integral,
            },
            "pitch": {
                "value": state.PitchAngle,
                "setpoint": result.target_pitch_deg,
                "output": result.elevator,
                "integral": self.controller.pitch_pid.config.ki * self.controller.pitch_pid.integral,
            },
            "flare": {
                "value": state.PitchAngle,
                "setpoint": result.target_pitch_deg,
                "output": result.elevator if result.flare_active else 0.0,
                "integral": self.controller.pitch_pid.config.ki * self.controller.pitch_pid.integral,
            },
            "speed": {
                "value": state.IndicatedAirspeed,
                "setpoint": result.target_ias_kt,
                "output": result.throttle_left_rate,
                "integral": self.controller.speed_pid.config.ki * self.controller.speed_pid.integral,
            },
            "meta": {
                "ra": state.RadioAltitude,
                "vs": state.VerticalSpeed,
                "loc": result.loc_dots,
                "gs": result.gs_dots,
                "aoa": result.estimated_aoa_deg,
                "aoa_ref": result.reference_aoa_deg,
                "mach": result.mach,
                "alpha_margin": result.alpha_margin_deg,
                "vapp": result.vapp_kt,
                "target_ias": result.target_ias_kt,
                "flaps": result.flap_configuration,
                "flare_armed": result.flare_armed,
                "flare": result.flare_active,
                "flare_progress": result.flare_progress,
                "rudder_cmd": result.rudder,
                "throttle_cmd": result.throttle_norm,
                "throttle_target_angle": result.throttle_target_angle_deg,
                "warnings": list(result.envelope_warnings),
                "throttle_left": state.LeftThrottleAngle,
                "throttle_right": state.RightThrottleAngle,
                "thrust_left": state.EngLeftThrust,
                "thrust_right": state.EngRigntThrust,
                "active": state.AgentIsActive,
            },
        }
        with self.lock:
            self.points.append(point)
            self.next_sequence += 1

    def snapshot(self, since_sequence: int = -1) -> dict[str, Any]:
        with self.lock:
            start = max(0, since_sequence + 1)
            points = list(self.points[start:])
            gains = self._gains()
            last_sequence = self.next_sequence - 1
        return {
            "run_id": self.run_id,
            "points": points,
            "gains": gains,
            "last_sequence": last_sequence,
        }

    def update_gains(self, loop: str, values: dict[str, Any]) -> dict[str, float]:
        with self.lock:
            pid = self._pid(loop)
            for name in ("kp", "ki", "kd"):
                if name in values:
                    value = float(values[name])
                    if not math.isfinite(value):
                        raise ValueError(f"{name} must be finite")
                    setattr(pid.config, name, value)
            return self._gains()[loop]

    def _pid(self, loop: str):
        mapping = {
            "roll": self.controller.roll_pid,
            "pitch": self.controller.pitch_pid,
            "flare": self.controller.pitch_pid,
            "speed": self.controller.speed_pid,
        }
        try:
            return mapping[loop]
        except KeyError as exc:
            raise ValueError(f"unknown loop: {loop}") from exc

    def _gains(self) -> dict[str, dict[str, float]]:
        return {
            name: {key: getattr(self._pid(name).config, key) for key in ("kp", "ki", "kd")}
            for name in ("roll", "pitch", "flare", "speed")
        }


class DashboardServer:
    def __init__(self, state: DashboardState, host: str, port: int) -> None:
        self.state = state
        handler = self._handler_type()
        self.httpd = ThreadingHTTPServer((host, port), handler)
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    @property
    def port(self) -> int:
        return int(self.httpd.server_address[1])

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.httpd.shutdown()
        self.httpd.server_close()
        self.thread.join(timeout=2.0)

    def _handler_type(self):
        state = self.state

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def do_GET(self) -> None:
                request = urlsplit(self.path)
                if request.path == "/api/state":
                    try:
                        since = int(parse_qs(request.query).get("since", ["-1"])[0])
                    except ValueError:
                        self._json(400, {"error": "since must be an integer"})
                        return
                    self._json(200, state.snapshot(since))
                    return
                if request.path in {"/", "/index.html"}:
                    body = HTML_PATH.read_bytes()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Connection", "close")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    self.close_connection = True
                    return
                self.send_error(404)

            def do_POST(self) -> None:
                if self.path != "/api/gains":
                    self.send_error(404)
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    payload = json.loads(self.rfile.read(length))
                    gains = state.update_gains(str(payload["loop"]), payload)
                except (KeyError, ValueError, TypeError, json.JSONDecodeError) as exc:
                    self._json(400, {"error": str(exc)})
                    return
                self._json(200, gains)

            def log_message(self, format: str, *args: object) -> None:
                return

            def _json(self, status: int, payload: Any) -> None:
                body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "close")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                self.close_connection = True

        return Handler
