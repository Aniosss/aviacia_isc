import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from ics_protocol import GearState, ICSInputs  # noqa: E402
from prepare_a31_stabilizer import (  # noqa: E402
    SafetyLimits,
    pulse_command,
    resolve_target_deg,
    safety_violation,
    target_reached,
    target_stable_for,
)


def make_state(**overrides: object) -> ICSInputs:
    values = {field.name: 0 for field in ICSInputs.__dataclass_fields__.values()}
    values.update(
        {
            "AgentIsActive": 1,
            "RadioAltitude": 2800.0,
            "IndicatedAirspeed": 150.0,
            "PitchAngle": 3.0,
            "RollAngle": 0.0,
            "VerticalSpeed": -500.0,
            "BodyNormAccel": 1.0,
            "StabilizerAngle": 0.0,
            "NoseGearStatus": GearState.UpLock,
            "LeftGearStatus": GearState.UpLock,
            "RightGearStatus": GearState.UpLock,
        }
    )
    values.update(overrides)
    return ICSInputs(**values)


class PrepareA31StabilizerTests(unittest.TestCase):
    def test_target_reached_uses_tolerance(self) -> None:
        self.assertTrue(target_reached(-1.95, -2.0, 0.1))
        self.assertFalse(target_reached(-1.8, -2.0, 0.1))

    def test_relative_target_is_resolved_from_measured_baseline(self) -> None:
        self.assertAlmostEqual(resolve_target_deg(-4.7, 1.0, None), -3.7)
        self.assertAlmostEqual(resolve_target_deg(-6.7, -1.0, None), -7.7)

    def test_absolute_target_remains_available_for_diagnostics(self) -> None:
        self.assertAlmostEqual(resolve_target_deg(-4.7, None, -3.7), -3.7)

    def test_pulse_direction_follows_angle_error(self) -> None:
        self.assertEqual(pulse_command(0.0, -2.0, 0.1, 0.25), -0.25)
        self.assertEqual(pulse_command(-4.0, -2.0, 0.1, 0.25), 0.25)
        self.assertEqual(pulse_command(-2.05, -2.0, 0.1, 0.25), 0.0)

    def test_target_must_remain_inside_band_for_stable_time(self) -> None:
        stable_since, stable = target_stable_for(
            -2.05, -2.0, 0.1, None, 10.0, 2.0
        )
        self.assertEqual(stable_since, 10.0)
        self.assertFalse(stable)

        stable_since, stable = target_stable_for(
            -2.02, -2.0, 0.1, stable_since, 12.0, 2.0
        )
        self.assertEqual(stable_since, 10.0)
        self.assertTrue(stable)

    def test_target_stability_resets_after_leaving_band(self) -> None:
        stable_since, stable = target_stable_for(
            -2.2, -2.0, 0.1, 10.0, 11.0, 2.0
        )
        self.assertIsNone(stable_since)
        self.assertFalse(stable)

    def test_nominal_state_is_safe(self) -> None:
        self.assertIsNone(safety_violation(make_state(), SafetyLimits()))

    def test_low_altitude_aborts(self) -> None:
        message = safety_violation(
            make_state(RadioAltitude=499.0),
            SafetyLimits(),
        )
        self.assertIn("radio altitude", message or "")

    def test_excessive_sink_rate_aborts(self) -> None:
        message = safety_violation(
            make_state(VerticalSpeed=-2600.0),
            SafetyLimits(),
        )
        self.assertIn("vertical speed", message or "")


if __name__ == "__main__":
    unittest.main()
