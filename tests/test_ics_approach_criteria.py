from __future__ import annotations

import sys
import unittest
from dataclasses import fields
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from ics_approach_criteria import (  # noqa: E402
    ApproachCriteriaConfig,
    ApproachCriteriaMonitor,
)
from ics_protocol import ICSInputs  # noqa: E402


def make_state(**overrides: object) -> ICSInputs:
    values = {item.name: 0.0 for item in fields(ICSInputs)}
    values.update({
        "RadioAltitudeValid": 1,
        "RadioAltitude": 1000.0,
        "RunwayHeadingValid": 1,
        "RunwayHeading": 64.0,
        "TrkAngleMagneticValid": 1,
        "TrkAngleMagnetic": 64.2,
        "GSDeviationValid": 1,
    })
    values.update(overrides)
    return ICSInputs(**values)  # type: ignore[arg-type]


class ApproachCriteriaMonitorTests(unittest.TestCase):
    def test_passes_when_both_limits_hold_until_300_ft(self) -> None:
        monitor = ApproachCriteriaMonitor()
        monitor.observe(make_state(RadioAltitude=900.0), -3.1)
        monitor.observe(
            make_state(RadioAltitude=500.0, TrkAngleMagnetic=63.5),
            -2.6,
        )
        crossing = monitor.observe(make_state(RadioAltitude=299.8), -3.0)

        verdict = monitor.verdict()
        self.assertEqual(crossing.status, "COMPLETE")
        self.assertEqual(verdict.status, "PASS")
        self.assertEqual(verdict.sample_count, 2)
        self.assertAlmostEqual(verdict.max_course_error_deg or 0.0, 0.5)
        self.assertAlmostEqual(verdict.max_glideslope_error_deg or 0.0, 0.4)

    def test_uses_track_not_crabbed_heading_for_course(self) -> None:
        state = make_state(MagneticHeading=72.0, TrkAngleMagnetic=64.1)
        monitor = ApproachCriteriaMonitor()

        sample = monitor.observe(state, -3.0)

        self.assertAlmostEqual(sample.course_error_deg or 0.0, 0.1)

    def test_fails_when_limits_are_exceeded(self) -> None:
        monitor = ApproachCriteriaMonitor()
        monitor.observe(make_state(), -3.0)
        monitor.observe(make_state(TrkAngleMagnetic=65.0), -2.2)
        monitor.observe(make_state(RadioAltitude=300.0), -3.0)

        verdict = monitor.verdict()
        self.assertEqual(verdict.status, "FAIL")
        self.assertIn("course 1.000deg exceeds 0.7deg", verdict.reasons)
        self.assertIn("glideslope 0.800deg exceeds 0.5deg", verdict.reasons)

    def test_non_finite_required_telemetry_fails_the_run(self) -> None:
        monitor = ApproachCriteriaMonitor()
        monitor.observe(make_state(TrkAngleMagnetic=float("nan")), -3.0)
        monitor.observe(make_state(RadioAltitude=299.0), -3.0)

        verdict = monitor.verdict()
        self.assertEqual(verdict.status, "FAIL")
        self.assertIn("magnetic track non-finite", verdict.reasons)

    def test_numeric_telemetry_is_used_when_validity_flags_are_zero(self) -> None:
        monitor = ApproachCriteriaMonitor()
        sample = monitor.observe(
            make_state(
                RunwayHeadingValid=0,
                TrkAngleMagneticValid=0,
                GSDeviationValid=0,
            ),
            -3.0,
        )
        monitor.observe(make_state(RadioAltitude=299.0), -3.0)

        self.assertEqual(sample.status, "OK")
        self.assertEqual(monitor.verdict().status, "PASS")

    def test_waits_for_course_and_glideslope_capture(self) -> None:
        monitor = ApproachCriteriaMonitor()
        waiting = monitor.observe(make_state(TrkAngleMagnetic=65.0), 0.0)
        captured = monitor.observe(make_state(RadioAltitude=800.0), -3.0)
        monitor.observe(make_state(RadioAltitude=299.0), -3.0)

        verdict = monitor.verdict()
        self.assertEqual(waiting.status, "WAITING_CAPTURE")
        self.assertEqual(captured.status, "OK")
        self.assertEqual(verdict.status, "PASS")
        self.assertEqual(verdict.sample_count, 1)

    def test_numeric_radio_altitude_completes_even_when_valid_flag_is_zero(self) -> None:
        monitor = ApproachCriteriaMonitor()
        monitor.observe(make_state(RadioAltitude=500.0), -3.0)

        sample = monitor.observe(
            make_state(RadioAltitude=299.0, RadioAltitudeValid=0),
            -3.0,
        )

        self.assertEqual(sample.status, "COMPLETE")
        self.assertTrue(monitor.cutoff_reached)
        self.assertEqual(monitor.verdict().status, "PASS")

    def test_reports_incomplete_if_cutoff_is_not_reached(self) -> None:
        monitor = ApproachCriteriaMonitor(
            ApproachCriteriaConfig(cutoff_radio_altitude_ft=350.0)
        )
        monitor.observe(make_state(RadioAltitude=500.0), -3.0)

        verdict = monitor.verdict()
        self.assertEqual(verdict.status, "INCOMPLETE")
        self.assertIn("350-ft cutoff was not reached", verdict.reasons)


if __name__ == "__main__":
    unittest.main()
