from __future__ import annotations

import sys
import unittest
from dataclasses import fields
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from ics_pid_controller import (  # noqa: E402
    ClearWeatherILSController,
    ControllerConfig,
    PID,
    PIDConfig,
    angle_error_deg,
)
from ics_protocol import (  # noqa: E402
    AIRBORNE_CONTROL_VALID_MASK,
    ControlModeState,
    GearState,
    ICSInputs,
)
from run_ics_pid import main_gear_contact, make_airborne_output  # noqa: E402


def make_state(**overrides: object) -> ICSInputs:
    values: dict[str, object] = {}
    for item in fields(ICSInputs):
        values[item.name] = 0.0
    values.update({
        "AgentIsActive": 1,
        "NoseGearStatus": GearState.UpLock,
        "LeftGearStatus": GearState.UpLock,
        "RightGearStatus": GearState.UpLock,
        "GroundSpeed": 150.0,
        "GroundSpeedValid": 1,
        "IndicatedAirspeed": 150.0,
        "IndicatedAirspeedValid": 1,
        "VerticalSpeedValid": 1,
        "RadioAltitude": 2800.0,
        "SlatsAngle": 28.0,
        "FlapsAngle": 36.0,
        "RunwayHeading": 64.0,
        "MagneticHeading": 64.0,
        "TrkAngleMagnetic": 64.0,
        "PitchAngle": 2.5,
        "PitchAngleValid": 1,
    })
    values.update(overrides)
    return ICSInputs(**values)  # type: ignore[arg-type]


class PIDTests(unittest.TestCase):
    def test_angle_error_wraps(self) -> None:
        self.assertEqual(angle_error_deg(2.0, 358.0), 4.0)
        self.assertEqual(angle_error_deg(358.0, 2.0), -4.0)

    def test_pid_clamps_and_does_not_wind_up(self) -> None:
        pid = PID(PIDConfig(2.0, 1.0, 0.0, -1.0, 1.0, -10.0, 10.0))
        for _ in range(20):
            self.assertEqual(pid.update(10.0, 0.0, 0.1), 1.0)
        self.assertEqual(pid.integral, 0.0)

    def test_negative_localizer_error_commands_negative_roll_via_positive_aileron(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        result = controller.update(make_state(LocDeviation=-0.155), 0.05)
        self.assertLess(result.target_roll_deg, 0.0)
        self.assertGreater(result.aileron, 0.0)

    def test_crosswind_crab_does_not_consume_localizer_intercept(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        result = controller.update(
            make_state(
                LocDeviation=-0.06975,
                MagneticHeading=59.5,
                TrkAngleMagnetic=63.4,
            ),
            0.05,
        )

        self.assertAlmostEqual(result.target_heading_deg, 59.5)
        self.assertAlmostEqual(result.heading_error_deg, -3.9)
        self.assertAlmostEqual(result.target_roll_deg, -2.73)

    def test_centered_localizer_holds_runway_track_while_crabbed(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        result = controller.update(
            make_state(
                LocDeviation=0.0,
                MagneticHeading=60.0,
                TrkAngleMagnetic=64.0,
            ),
            0.05,
        )

        self.assertAlmostEqual(result.heading_error_deg, 0.0)
        self.assertAlmostEqual(result.target_roll_deg, 0.0)

    def test_outputs_respect_limits(self) -> None:
        config = ControllerConfig()
        controller = ClearWeatherILSController(config)
        result = controller.update(
            make_state(LocDeviation=10.0, GSDeviation=10.0, RollAngle=-90.0, PitchAngle=40.0),
            0.05,
        )
        self.assertLessEqual(abs(result.aileron), config.roll_pid.output_max)
        self.assertLessEqual(abs(result.elevator), config.pitch_pid.output_max)
        self.assertLessEqual(abs(result.throttle_left_rate), config.throttle_rate_max_deg_per_s)
        self.assertLessEqual(abs(result.throttle_right_rate), config.throttle_rate_max_deg_per_s)

    def test_positive_glideslope_guidance_reduces_descent_rate(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        on_path = controller.update(make_state(GSDeviation=0.0), 0.05)
        below_path = controller.update(make_state(GSDeviation=-0.175), 0.05)
        self.assertGreater(below_path.target_vs_fpm, on_path.target_vs_fpm)

    def test_vertical_speed_correction_has_no_integral_memory(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        state = make_state(VerticalSpeed=-1200.0)
        first = controller.update(state, 0.05)
        for _ in range(100):
            last = controller.update(state, 0.05)
        self.assertAlmostEqual(last.vertical_correction_deg, first.vertical_correction_deg)

    def test_fast_descent_gets_stronger_pitch_correction(self) -> None:
        config = ControllerConfig()
        baseline = ClearWeatherILSController(config).update(make_state(), 0.05)
        fast_descent = ClearWeatherILSController(config).update(
            make_state(VerticalSpeed=baseline.target_vs_fpm - 500.0),
            0.05,
        )
        slow_descent = ClearWeatherILSController(config).update(
            make_state(VerticalSpeed=baseline.target_vs_fpm + 500.0),
            0.05,
        )

        self.assertAlmostEqual(fast_descent.vertical_correction_deg, 0.75)
        self.assertAlmostEqual(slow_descent.vertical_correction_deg, -0.5)

    def test_fast_descent_regression_commands_nose_up(self) -> None:
        config = ControllerConfig()
        controller = ClearWeatherILSController(config)
        controller.update(
            make_state(
                RadioAltitude=523.0,
                IndicatedAirspeed=150.5,
                VerticalSpeed=-789.0,
                PitchAngle=2.67,
            ),
            0.1,
        )

        result = controller.update(
            make_state(
                RadioAltitude=400.0,
                IndicatedAirspeed=153.0,
                VerticalSpeed=-1700.0,
                PitchAngle=-4.63,
            ),
            0.1,
        )

        self.assertGreater(result.target_pitch_deg, -4.63)
        self.assertGreater(result.elevator, 0.0)
        self.assertLessEqual(result.elevator, config.pitch_pid.output_max)

    def test_estimates_approach_angle_of_attack(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        result = controller.update(
            make_state(GroundSpeed=207.0, VerticalSpeed=-1082.0, PitchAngle=4.1),
            0.05,
        )

        self.assertAlmostEqual(result.flight_path_angle_deg, -2.956, places=2)
        self.assertAlmostEqual(result.estimated_aoa_deg, 7.056, places=2)
        self.assertAlmostEqual(result.reference_aoa_deg, 7.056, places=2)

    def test_adaptive_aoa_holds_its_reference_during_large_vertical_error(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        first = controller.update(make_state(PitchAngle=4.0, VerticalSpeed=0.0), 0.05)
        second = controller.update(make_state(PitchAngle=8.0, VerticalSpeed=1000.0), 0.05)

        self.assertAlmostEqual(first.reference_aoa_deg, 4.0)
        self.assertAlmostEqual(second.reference_aoa_deg, 4.0)

    def test_adaptive_aoa_moves_slowly_when_stabilized(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        first = controller.update(make_state(PitchAngle=4.0, VerticalSpeed=0.0), 0.05)
        stabilized = make_state(
            PitchAngle=first.target_flight_path_angle_deg + 5.0,
            VerticalSpeed=first.target_vs_fpm,
        )

        second = controller.update(stabilized, 0.1)

        self.assertAlmostEqual(second.estimated_aoa_deg, 5.0)
        self.assertGreater(second.reference_aoa_deg, first.reference_aoa_deg)
        self.assertLessEqual(second.reference_aoa_deg, first.reference_aoa_deg + 0.02)

    def test_adaptive_aoa_recovers_gradually_from_an_unstable_entry(self) -> None:
        config = ControllerConfig()
        controller = ClearWeatherILSController(config)
        state = make_state(
            GSDeviation=-0.119,
            GroundSpeed=150.0,
            VerticalSpeed=-793.0,
            PitchAngle=2.15,
        )

        first = controller.update(state, 0.1)
        second = controller.update(state, 0.1)

        self.assertGreater(abs(first.gs_dots), config.adaptive_aoa_max_gs_dots)
        self.assertAlmostEqual(first.reference_aoa_deg, first.estimated_aoa_deg)
        self.assertAlmostEqual(
            second.reference_aoa_deg - first.reference_aoa_deg,
            config.adaptive_aoa_recovery_rate_deg_per_s * 0.1,
        )

    def test_approach_target_vertical_speed_is_bounded(self) -> None:
        config = ControllerConfig()
        controller = ClearWeatherILSController(config)

        above = controller.update(make_state(GSDeviation=0.5), 0.05)
        below = controller.update(make_state(GSDeviation=-0.5), 0.05)

        self.assertEqual(above.target_vs_fpm, config.min_approach_target_vs_fpm)
        self.assertEqual(below.target_vs_fpm, config.max_approach_target_vs_fpm)

    def test_pitch_target_is_rate_limited_from_current_attitude(self) -> None:
        config = ControllerConfig()
        controller = ClearWeatherILSController(config)
        state = make_state(PitchAngle=6.8, VerticalSpeed=0.0)

        result = controller.update(state, 0.05)

        self.assertAlmostEqual(result.target_pitch_deg, 6.8 - 0.075)
        self.assertGreater(result.elevator, config.pitch_pid.output_min)

    def test_vapp_comes_from_weight_and_flaps(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig(landing_weight_kg=69277.0))

        full = controller.update(make_state(FlapsAngle=36.0), 0.05)
        flaps_3 = controller.update(make_state(FlapsAngle=27.0), 0.05)
        fallback = controller.update(make_state(FlapsAngle=0.0), 0.05)

        self.assertEqual(full.vapp_kt, 136.0)
        self.assertEqual(flaps_3.vapp_kt, 140.0)
        self.assertEqual(fallback.vapp_kt, 140.0)

    def test_speed_target_moves_gradually_from_entry_speed_to_vapp(self) -> None:
        config = ControllerConfig(target_ias_rate_kt_per_s=0.25)
        controller = ClearWeatherILSController(config)

        first = controller.update(
            make_state(IndicatedAirspeed=150.0, FlapsAngle=27.0),
            0.1,
        )
        second = controller.update(
            make_state(IndicatedAirspeed=150.0, FlapsAngle=27.0),
            0.1,
        )

        self.assertAlmostEqual(first.target_ias_kt, 149.975)
        self.assertAlmostEqual(second.target_ias_kt, 149.95)
        self.assertEqual(second.vapp_kt, 140.0)

    def test_speed_pid_outputs_throttle_rate_in_degrees_per_second(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        controller.update(
            make_state(
                IndicatedAirspeed=140.0,
                LeftThrottleAngle=20.0,
                RightThrottleAngle=20.0,
            ),
            0.05,
        )

        result = controller.update(
            make_state(
                IndicatedAirspeed=400.0,
                LeftThrottleAngle=20.0,
                RightThrottleAngle=20.0,
            ),
            0.05,
        )

        self.assertLess(result.throttle_left_rate, 0.0)
        self.assertLess(result.throttle_right_rate, 0.0)
        self.assertLessEqual(abs(result.throttle_left_rate), 8.0)
        self.assertLessEqual(abs(result.throttle_right_rate), 8.0)
        self.assertGreaterEqual(result.throttle_norm, 0.0)
        self.assertLessEqual(result.throttle_norm, 1.0)

    def test_throttle_position_loop_synchronizes_split_levers(self) -> None:
        config = ControllerConfig(throttle_position_gain_per_s=0.8)
        controller = ClearWeatherILSController(config)

        result = controller.update(
            make_state(
                IndicatedAirspeed=140.0,
                FlapsAngle=27.0,
                LeftThrottleAngle=5.0,
                RightThrottleAngle=25.0,
            ),
            0.1,
        )

        self.assertAlmostEqual(result.throttle_target_angle_deg, 15.0, places=1)
        self.assertEqual(result.throttle_left_rate / config.throttle_left_rate_sign, 8.0)
        self.assertEqual(result.throttle_right_rate / config.throttle_right_rate_sign, -8.0)

    def test_throttle_sync_boost_stays_off_for_small_lever_difference(self) -> None:
        config = ControllerConfig(
            throttle_position_gain_per_s=0.8,
            throttle_sync_boost_threshold_deg=1.0,
            throttle_sync_boost_gain_per_s=2.0,
        )
        controller = ClearWeatherILSController(config)

        result = controller.update(
            make_state(
                IndicatedAirspeed=140.0,
                FlapsAngle=27.0,
                LeftThrottleAngle=14.75,
                RightThrottleAngle=15.25,
            ),
            0.1,
        )

        self.assertAlmostEqual(result.throttle_left_rate, 0.2)
        self.assertAlmostEqual(result.throttle_right_rate, -0.2)

    def test_airborne_output_converges_split_throttles_with_symmetric_absolute_target(self) -> None:
        config = ControllerConfig(throttle_position_gain_per_s=0.8)
        controller = ClearWeatherILSController(config)
        state = make_state(
            IndicatedAirspeed=140.0,
            FlapsAngle=27.0,
            LeftThrottleAngle=5.0,
            RightThrottleAngle=25.0,
        )

        result = controller.update(state, 0.1)
        output = make_airborne_output(state, result)

        self.assertEqual(output.ControlValidMask, 0b11111)
        self.assertGreater(output.ThrottleLeftRate, 0.0)
        self.assertLess(output.ThrottleRightRate, 0.0)
        self.assertAlmostEqual(output.ThrottleLeft, 15.0 / config.throttle_forward_max_deg)
        self.assertEqual(output.ThrottleRight, output.ThrottleLeft)

    def test_speed_loop_integrates_absolute_normalized_throttle_command(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        controller.update(
            make_state(
                IndicatedAirspeed=140.0,
                LeftThrottleAngle=20.0,
                RightThrottleAngle=20.0,
            ),
            0.05,
        )
        state = make_state(
            IndicatedAirspeed=151.0,
            LeftThrottleAngle=20.0,
            RightThrottleAngle=20.0,
        )

        first = controller.update(state, 0.1)
        second = controller.update(state, 0.1)

        self.assertLess(first.throttle_norm, 20.0 / 55.7)
        self.assertLess(second.throttle_norm, first.throttle_norm)

    def test_speed_integral_does_not_wind_up_at_idle_stop(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        controller.update(make_state(IndicatedAirspeed=140.0), 0.05)
        fast = make_state(
            IndicatedAirspeed=170.0,
            LeftThrottleAngle=1.0,
            RightThrottleAngle=1.0,
        )
        for _ in range(100):
            at_idle = controller.update(fast, 0.1)

        recovery = controller.update(
            make_state(
                IndicatedAirspeed=130.0,
                LeftThrottleAngle=0.0,
                RightThrottleAngle=0.0,
            ),
            0.1,
        )

        self.assertEqual(at_idle.throttle_norm, 0.0)
        self.assertGreater(recovery.throttle_left_rate, 0.0)
        self.assertGreater(recovery.throttle_right_rate, 0.0)

    def test_estimates_mach_for_alpha_protection(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        result = controller.update(
            make_state(TrueAirspeedValid=1, TrueAirspeed=207.0, AirfieldTemp=15.0),
            0.05,
        )

        self.assertAlmostEqual(result.mach, 0.313, places=3)
        self.assertAlmostEqual(result.alpha_prot_deg, 12.818, places=3)

    def test_numeric_true_airspeed_is_used_when_valid_flag_is_zero(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())

        result = controller.update(
            make_state(TrueAirspeedValid=0, TrueAirspeed=207.0, AirfieldTemp=15.0),
            0.05,
        )

        self.assertAlmostEqual(result.mach, 0.313, places=3)

    def test_adaptive_aoa_uses_numeric_values_when_valid_flags_are_zero(self) -> None:
        config = ControllerConfig(
            adaptive_aoa_filter_tau_s=0.0,
            adaptive_aoa_rate_deg_per_s=10.0,
        )
        controller = ClearWeatherILSController(config)
        invalid_flags = {
            "PitchAngleValid": 0,
            "VerticalSpeedValid": 0,
            "GroundSpeedValid": 0,
        }
        initial = controller.update(
            make_state(
                PitchAngle=9.0,
                VerticalSpeed=0.0,
                GroundSpeed=140.0,
                **invalid_flags,
            ),
            0.1,
        )
        updated = controller.update(
            make_state(
                PitchAngle=5.0,
                VerticalSpeed=-700.0,
                GroundSpeed=140.0,
                **invalid_flags,
            ),
            0.1,
        )

        self.assertLess(updated.reference_aoa_deg, initial.reference_aoa_deg)

    def test_flare_linearly_blends_upstream_initial_vs_to_touchdown_rate(self) -> None:
        config = ControllerConfig()
        controller = ClearWeatherILSController(config)
        high = controller.update(
            make_state(RadioAltitude=201.0, VerticalSpeed=-1200.0),
            0.05,
        )
        entry = controller.update(
            make_state(RadioAltitude=150.0, VerticalSpeed=-1200.0),
            0.05,
        )
        midpoint = controller.update(
            make_state(RadioAltitude=77.5, VerticalSpeed=-800.0),
            0.05,
        )
        low = controller.update(
            make_state(RadioAltitude=5.0, VerticalSpeed=-400.0),
            0.05,
        )

        self.assertFalse(high.flare_active)
        self.assertTrue(entry.flare_active)
        self.assertEqual(entry.flare_progress, 0.0)
        self.assertAlmostEqual(entry.target_vs_fpm, config.flare_initial_vs_fpm)
        self.assertAlmostEqual(
            entry.flare_entry_vertical_speed_fpm,
            config.flare_initial_vs_fpm,
        )
        self.assertAlmostEqual(midpoint.flare_progress, 0.5)
        self.assertAlmostEqual(
            midpoint.target_vs_fpm,
            0.5 * (config.flare_initial_vs_fpm + config.touchdown_target_vs_fpm),
        )
        self.assertEqual(low.flare_progress, 1.0)
        self.assertAlmostEqual(low.target_vs_fpm, config.touchdown_target_vs_fpm)

    def test_flare_uses_upstream_vs_hold_with_attitude_and_rate_damping(self) -> None:
        config = ControllerConfig(flare_pitch_target_rate_deg_per_s=100.0)
        controller = ClearWeatherILSController(config)
        state = make_state(
            RadioAltitude=150.0,
            VerticalSpeed=-800.0,
            PitchAngle=2.0,
            BodyPitchRateValid=1,
            BodyPitchRate=1.0,
        )

        result = controller.update(state, 0.1)

        expected = (
            config.flare_pitch_base_deg
            + config.flare_vs_to_pitch_gain_deg_per_fpm
            * (config.flare_initial_vs_fpm - state.VerticalSpeed)
            - config.flare_pitch_attitude_damping_gain * state.PitchAngle
            - config.flare_pitch_rate_damping_gain * state.BodyPitchRate
        )
        self.assertAlmostEqual(result.target_pitch_deg, expected)

    def test_flare_uses_numeric_pitch_rate_when_valid_flag_is_zero(self) -> None:
        config = ControllerConfig(flare_pitch_target_rate_deg_per_s=100.0)
        controller = ClearWeatherILSController(config)
        state = make_state(
            RadioAltitude=150.0,
            VerticalSpeed=-800.0,
            PitchAngle=2.0,
            BodyPitchRateValid=0,
            BodyPitchRate=1.0,
        )

        result = controller.update(state, 0.1)

        expected = (
            config.flare_pitch_base_deg
            + config.flare_vs_to_pitch_gain_deg_per_fpm
            * (config.flare_initial_vs_fpm - state.VerticalSpeed)
            - config.flare_pitch_attitude_damping_gain * state.PitchAngle
            - config.flare_pitch_rate_damping_gain * state.BodyPitchRate
        )
        self.assertAlmostEqual(result.target_pitch_deg, expected)

    def test_flare_entry_does_not_drop_the_pitch_target(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        approach_state = make_state(
            RadioAltitude=250.0,
            VerticalSpeed=-800.0,
            GSDeviation=-0.119,
        )
        for _ in range(30):
            before = controller.update(approach_state, 0.1)

        entry = controller.update(
            make_state(
                RadioAltitude=198.0,
                VerticalSpeed=-800.0,
                GSDeviation=-0.119,
            ),
            0.1,
        )

        self.assertFalse(before.flare_active)
        self.assertTrue(entry.flare_active)
        self.assertGreaterEqual(entry.target_pitch_deg, before.target_pitch_deg)
        self.assertGreater(entry.elevator, 0.0)

    def test_flare_can_start_early_when_time_to_ground_is_short(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())

        result = controller.update(
            make_state(RadioAltitude=180.0, VerticalSpeed=-1000.0),
            0.05,
        )

        self.assertTrue(result.flare_active)
        self.assertEqual(result.flare_entry_radio_altitude_ft, 180.0)

    def test_flare_arms_internally_without_advertising_the_mode_to_ics(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())

        high = controller.update(
            make_state(RadioAltitude=450.0, VerticalSpeed=-800.0),
            0.05,
        )
        armed = controller.update(
            make_state(RadioAltitude=350.0, VerticalSpeed=-800.0),
            0.05,
        )
        output = make_airborne_output(
            make_state(RadioAltitude=350.0, VerticalSpeed=-800.0),
            armed,
        )

        self.assertFalse(high.flare_armed)
        self.assertFalse(high.flare_active)
        self.assertTrue(armed.flare_armed)
        self.assertFalse(armed.flare_active)
        self.assertEqual(output.ControlMode, ControlModeState.Approach)
        self.assertEqual(output.ModeFlareArm, 0)
        self.assertEqual(output.ModeFlare, 0)

    def test_flare_elevator_is_bounded_by_normal_pitch_pid_limits(self) -> None:
        config = ControllerConfig()
        controller = ClearWeatherILSController(config)
        result = controller.update(
            make_state(RadioAltitude=150.0, VerticalSpeed=-780.0, PitchAngle=-20.0),
            0.1,
        )
        for altitude in (130.0, 110.0, 90.0, 70.0, 50.0, 30.0, 10.0):
            result = controller.update(
                make_state(
                    RadioAltitude=altitude,
                    VerticalSpeed=-780.0,
                    PitchAngle=-20.0,
                ),
                0.1,
            )

        self.assertTrue(result.flare_active)
        self.assertEqual(result.elevator, config.pitch_pid.output_max)
        self.assertLessEqual(abs(result.elevator), config.pitch_pid.output_max)

    def test_low_altitude_flare_does_not_override_speed_loop_output(self) -> None:
        config = ControllerConfig()
        flare_controller = ClearWeatherILSController(config)
        approach_controller = ClearWeatherILSController(config)
        high_state = make_state(
            RadioAltitude=250.0,
            IndicatedAirspeed=150.0,
            VerticalSpeed=-800.0,
            LeftThrottleAngle=20.0,
            RightThrottleAngle=20.0,
        )
        flare_controller.update(high_state, 0.1)
        approach_controller.update(high_state, 0.1)

        flare = flare_controller.update(
            make_state(
                RadioAltitude=10.0,
                IndicatedAirspeed=170.0,
                VerticalSpeed=-800.0,
                LeftThrottleAngle=20.0,
                RightThrottleAngle=20.0,
            ),
            0.1,
        )
        approach = approach_controller.update(
            make_state(
                RadioAltitude=250.0,
                IndicatedAirspeed=170.0,
                VerticalSpeed=-800.0,
                LeftThrottleAngle=20.0,
                RightThrottleAngle=20.0,
            ),
            0.1,
        )

        self.assertTrue(flare.flare_active)
        self.assertFalse(approach.flare_active)
        self.assertEqual(flare.throttle_left_rate, approach.throttle_left_rate)
        self.assertEqual(flare.throttle_right_rate, approach.throttle_right_rate)
        self.assertEqual(flare.throttle_norm, approach.throttle_norm)

    def test_flare_keeps_pitch_pid_output_limits_from_config(self) -> None:
        config = ControllerConfig()
        config.pitch_pid = PIDConfig(1.0, 0.0, 0.0, -0.25, 0.25, -20.0, 20.0)
        controller = ClearWeatherILSController(config)

        result = controller.update(
            make_state(RadioAltitude=10.0, VerticalSpeed=-1200.0, PitchAngle=-10.0),
            0.1,
        )

        self.assertTrue(result.flare_active)
        self.assertEqual(result.elevator, config.pitch_pid.output_max)

    def test_flare_does_not_reset_pitch_pid_integral_state(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        state = make_state(RadioAltitude=210.0, VerticalSpeed=-800.0, PitchAngle=2.5)
        controller.update(state, 0.05)
        controller.pitch_pid.integral = 1.25

        result = controller.update(
            make_state(RadioAltitude=150.0, VerticalSpeed=-800.0, PitchAngle=2.5),
            0.05,
        )

        self.assertTrue(result.flare_active)
        self.assertGreaterEqual(controller.pitch_pid.integral, 1.25)
        self.assertLess(controller.pitch_pid.integral, 1.27)

    def test_flare_uses_the_same_ics_modes_as_high_altitude_approach(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        high_state = make_state(
            RadioAltitude=450.0,
            VerticalSpeed=-800.0,
            MagneticHeading=65.0,
        )
        high_result = controller.update(high_state, 0.05)
        high_output = make_airborne_output(high_state, high_result)

        flare_state = make_state(
            RadioAltitude=80.0,
            VerticalSpeed=-800.0,
            MagneticHeading=65.0,
        )
        result = controller.update(flare_state, 0.05)

        output = make_airborne_output(flare_state, result)
        mode_fields = (
            "ControlValidMask",
            "ControlMode",
            "ModeLocCapture",
            "ModeLocTrack",
            "ModeGSCapture",
            "ModeGSTrack",
            "ModeFlareArm",
            "ModeFlare",
            "ModeAlignArm",
            "ModeAlign",
            "ModeRolloutArm",
            "ModeRollout",
            "ModeTaxiArm",
            "ModeTaxi",
            "ModeSpeed",
            "ModeThrust",
        )

        self.assertTrue(result.flare_active)
        self.assertEqual(result.rudder, 0.0)
        for field_name in mode_fields:
            self.assertEqual(getattr(output, field_name), getattr(high_output, field_name))
        self.assertEqual(output.ControlValidMask, AIRBORNE_CONTROL_VALID_MASK)
        self.assertEqual(output.ControlMode, ControlModeState.Approach)
        self.assertEqual(output.ModeLocCapture, 0)
        self.assertEqual(output.ModeLocTrack, 0)
        self.assertEqual(output.ModeGSCapture, 0)
        self.assertEqual(output.ModeGSTrack, 0)
        self.assertEqual(output.ModeFlareArm, 0)
        self.assertEqual(output.ModeFlare, 0)
        self.assertEqual(output.ModeAlignArm, 0)
        self.assertEqual(output.ModeAlign, 0)
        self.assertEqual(output.ModeRolloutArm, 0)
        self.assertEqual(output.ModeRollout, 0)
        self.assertEqual(output.ModeTaxiArm, 0)
        self.assertEqual(output.ModeTaxi, 0)
        self.assertEqual(output.ModeSpeed, 1)
        self.assertEqual(output.ModeThrust, 1)
        self.assertEqual(output.ElevatorCmd, result.elevator)
        self.assertEqual(output.RudderCmd, result.rudder)
        self.assertEqual(output.ThrottleLeftRate, result.throttle_left_rate)
        self.assertEqual(output.ThrottleRightRate, result.throttle_right_rate)
        self.assertEqual(output.ThrottleRightRate, output.ThrottleLeftRate)
        self.assertEqual(output.ThrottleLeft, result.throttle_left_hold_norm)
        self.assertEqual(output.ThrottleRight, result.throttle_right_hold_norm)

    def test_main_gear_contact_stops_on_first_main_gear_wow(self) -> None:
        self.assertFalse(main_gear_contact(make_state(NoseGearWeightOnWheels=1)))
        self.assertTrue(main_gear_contact(make_state(LeftGearWeightOnWheels=1)))
        self.assertTrue(main_gear_contact(make_state(RightGearWeightOnWheels=1)))
        self.assertTrue(
            main_gear_contact(
                make_state(
                    LeftGearWeightOnWheels=1,
                    RightGearWeightOnWheels=1,
                )
            )
        )

    def test_positive_pitch_error_commands_positive_elevator(self) -> None:
        config = ControllerConfig()
        controller = ClearWeatherILSController(config)

        result = controller.update(
            make_state(RadioAltitude=250.0, VerticalSpeed=-800.0, PitchAngle=2.0),
            0.1,
        )

        self.assertFalse(result.flare_active)
        self.assertGreater(result.target_pitch_deg, 2.0)
        self.assertGreater(result.elevator, 0.0)
        self.assertLessEqual(abs(result.elevator), config.pitch_pid.output_max)

    def test_low_altitude_heading_error_keeps_rudder_zero(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())

        result = controller.update(
            make_state(
                RadioAltitude=80.0,
                VerticalSpeed=-800.0,
                RunwayHeading=64.0,
                MagneticHeading=65.0,
            ),
            0.05,
        )

        self.assertTrue(result.flare_active)
        self.assertEqual(result.rudder, 0.0)

    def test_reports_document_envelope_warnings(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        result = controller.update(
            make_state(
                IndicatedAirspeed=200.0,
                RollAngle=20.0,
                RadioAltitude=30.0,
                PitchAngle=8.2,
                VerticalSpeed=-600.0,
                BodyNormAccelValid=1,
                BodyNormAccel=2.2,
            ),
            0.05,
        )

        self.assertIn("VFE", result.envelope_warnings)
        self.assertIn("ROLL_LIMIT", result.envelope_warnings)
        self.assertIn("LOAD_FACTOR", result.envelope_warnings)
        self.assertIn("TOUCHDOWN_SPEED_HIGH", result.envelope_warnings)
        self.assertIn("TOUCHDOWN_VS", result.envelope_warnings)
        self.assertIn("TOUCHDOWN_PITCH", result.envelope_warnings)

    def test_small_unbiased_normal_acceleration_is_not_a_load_factor_warning(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())
        result = controller.update(
            make_state(BodyNormAccelValid=1, BodyNormAccel=-0.02),
            0.05,
        )

        self.assertNotIn("LOAD_FACTOR", result.envelope_warnings)

    def test_numeric_normal_acceleration_is_used_when_valid_flag_is_zero(self) -> None:
        controller = ClearWeatherILSController(ControllerConfig())

        result = controller.update(
            make_state(BodyNormAccelValid=0, BodyNormAccel=2.2),
            0.05,
        )

        self.assertIn("LOAD_FACTOR", result.envelope_warnings)

if __name__ == "__main__":
    unittest.main()
