from __future__ import annotations

import sys
import unittest
from dataclasses import fields
from http.client import HTTPConnection
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from ics_dashboard import DashboardServer, DashboardState  # noqa: E402
from ics_pid_controller import ClearWeatherILSController, ControllerConfig  # noqa: E402
from ics_protocol import ICSInputs  # noqa: E402


def make_state(**overrides: object) -> ICSInputs:
    values = {item.name: 0.0 for item in fields(ICSInputs)}
    values.update({
        "AgentIsActive": 1,
        "GroundSpeed": 150.0,
        "GroundSpeedValid": 1,
        "VerticalSpeedValid": 1,
        "IndicatedAirspeed": 148.0,
        "RunwayHeading": 64.0,
        "MagneticHeading": 65.0,
        "RollAngle": 2.0,
        "PitchAngle": 3.0,
        "PitchAngleValid": 1,
        "RadioAltitude": 2800.0,
        "LeftThrottleAngle": 31.5,
        "RightThrottleAngle": 32.0,
        "EngLeftThrust": 8400.0,
        "EngRigntThrust": 8500.0,
    })
    values.update(overrides)
    return ICSInputs(**values)  # type: ignore[arg-type]


class DashboardStateTests(unittest.TestCase):
    def test_records_controller_values(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        state = make_state()
        result = controller.update(state, 0.05)
        dashboard = DashboardState(controller)

        dashboard.record(1.25, state, result)
        snapshot = dashboard.snapshot()

        self.assertEqual(len(snapshot["points"]), 1)
        point = snapshot["points"][0]
        self.assertEqual(point["seq"], 0)
        self.assertEqual(point["t"], 1.25)
        self.assertEqual(point["roll"]["value"], 2.0)
        self.assertEqual(point["roll"]["setpoint"], result.target_roll_deg)
        self.assertEqual(point["roll"]["output"], result.aileron)
        self.assertEqual(point["flare"]["output"], 0.0)
        self.assertEqual(point["meta"]["aoa_ref"], result.reference_aoa_deg)
        self.assertEqual(point["meta"]["flare_progress"], result.flare_progress)
        self.assertEqual(point["meta"]["throttle_left"], 31.5)
        self.assertEqual(point["meta"]["throttle_right"], 32.0)
        self.assertEqual(point["meta"]["thrust_left"], 8400.0)
        self.assertEqual(point["meta"]["thrust_right"], 8500.0)

    def test_returns_only_points_after_sequence(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        dashboard = DashboardState(controller)
        state = make_state()
        result = controller.update(state, 0.05)
        dashboard.record(1.0, state, result)
        dashboard.record(2.0, state, result)

        snapshot = dashboard.snapshot(since_sequence=0)

        self.assertEqual(snapshot["last_sequence"], 1)
        self.assertEqual([point["seq"] for point in snapshot["points"]], [1])

    def test_updates_live_pid_gains(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        dashboard = DashboardState(controller)

        updated = dashboard.update_gains("pitch", {"kp": 0.08, "ki": 0.002})

        self.assertEqual(updated["kp"], 0.08)
        self.assertEqual(updated["ki"], 0.002)
        self.assertEqual(controller.pitch_pid.config.kp, 0.08)

    def test_flare_view_updates_the_shared_pitch_pid_gains(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        dashboard = DashboardState(controller)

        updated = dashboard.update_gains("flare", {"kp": 0.16, "kd": 0.12})

        self.assertEqual(updated["kp"], 0.16)
        self.assertEqual(updated["kd"], 0.12)
        self.assertEqual(controller.pitch_pid.config.kp, 0.16)
        self.assertEqual(dashboard.snapshot()["gains"]["pitch"], updated)

    def test_rejects_non_finite_gain(self) -> None:
        dashboard = DashboardState(ClearWeatherILSController(ControllerConfig()))
        with self.assertRaises(ValueError):
            dashboard.update_gains("roll", {"kp": float("nan")})

    def test_http_11_response_closes_connection(self) -> None:
        state = DashboardState(ClearWeatherILSController(ControllerConfig()))
        server = DashboardServer(state, "127.0.0.1", 0)
        server.start()
        connection = HTTPConnection("127.0.0.1", server.port, timeout=2.0)
        try:
            connection.request("GET", "/api/state?since=-1")
            response = connection.getresponse()
            self.assertEqual(response.version, 11)
            self.assertEqual(response.status, 200)
            self.assertEqual(response.getheader("Connection"), "close")
            response.read()
            self.assertTrue(response.will_close)
        finally:
            connection.close()
            server.stop()


if __name__ == "__main__":
    unittest.main()
