from __future__ import annotations

import argparse
import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from dataclasses import asdict, fields
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from ics_altitude_authority_sweep import (  # noqa: E402
    LOG_COLUMNS,
    TEST_CONTROL_VALID_MASK,
    SafetyLimits,
    differential_surface,
    make_output,
    main,
    parse_altitudes,
    safety_reason,
    telemetry_row,
    validate_args,
)
from ics_protocol import ControlModeState, GearState, ICSInputs  # noqa: E402


def make_state(**overrides: object) -> ICSInputs:
    values: dict[str, object] = {item.name: 0.0 for item in fields(ICSInputs)}
    values.update(
        {
            "AgentIsActive": 1,
            "FlightPhaseValid": 1,
            "FlightPhase": 10,
            "RadioAltitudeValid": 1,
            "RadioAltitude": 350.0,
            "IndicatedAirspeedValid": 1,
            "IndicatedAirspeed": 150.0,
            "GroundSpeedValid": 1,
            "GroundSpeed": 145.0,
            "VerticalSpeedValid": 1,
            "VerticalSpeed": -750.0,
            "PitchAngleValid": 1,
            "PitchAngle": 3.0,
            "RollAngleValid": 1,
            "RollAngle": 0.5,
            "BodyPitchRateValid": 1,
            "BodyRollRateValid": 1,
            "BodyYawRateValid": 1,
            "BodyNormAccelValid": 1,
            "BodyLatAccelValid": 1,
            "NoseGearStatus": GearState.UpLock,
            "LeftGearStatus": GearState.UpLock,
            "RightGearStatus": GearState.UpLock,
            "LeftThrottleAngle": 20.0,
            "RightThrottleAngle": 20.0,
        }
    )
    values.update(overrides)
    return ICSInputs(**values)  # type: ignore[arg-type]


def safety_limits() -> SafetyLimits:
    return SafetyLimits(
        abort_radio_altitude_ft=100.0,
        max_abs_pitch_deg=15.0,
        max_abs_roll_deg=15.0,
        min_ias_kt=120.0,
        max_ias_kt=190.0,
        min_vertical_speed_fpm=-2000.0,
    )


def valid_args(**overrides: object) -> argparse.Namespace:
    values: dict[str, object] = {
        "pulse_seconds": 0.25,
        "recovery_seconds": 0.75,
        "initial_settle_seconds": 0.5,
        "rate_hz": 30.0,
        "arm_seconds": 2.2,
        "telemetry_timeout": 1.0,
        "max_wait_seconds": 90.0,
        "max_target_overshoot_ft": 20.0,
        "max_abs_pitch": 15.0,
        "max_abs_roll": 15.0,
        "wait_timeout": 10.0,
        "elevator_step": 0.5,
        "aileron_step": 5.0,
        "activation_min_ra_ft": 500.0,
        "landing_after_ft": None,
        "abort_ra_ft": 100.0,
        "altitudes": (400.0, 300.0, 200.0, 150.0),
        "min_ias": 120.0,
        "max_ias": 190.0,
        "min_vs_fpm": -2000.0,
        "bind_port": 3030,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(0.0, seconds)


class FakeTelemetrySocket:
    sender = ("127.0.0.1", 59557)

    def __init__(self, radio_altitude_ft: float = 541.0) -> None:
        self.radio_altitude_ft = radio_altitude_ft
        self.initial_pending = True
        self.telemetry_pending = False
        self.sent_packets: list[tuple[dict[str, object], float]] = []

    def setsockopt(self, *_args: object) -> None:
        pass

    def bind(self, _address: tuple[str, int]) -> None:
        pass

    def settimeout(self, _seconds: float) -> None:
        pass

    def setblocking(self, _enabled: bool) -> None:
        pass

    def sendto(self, packet: bytes, _sender: tuple[str, int]) -> int:
        decoded = json.loads(packet.decode("utf-8"))
        self.sent_packets.append((decoded, self.radio_altitude_ft))
        self.telemetry_pending = True
        return len(packet)

    def recvfrom(self, _size: int) -> tuple[bytes, tuple[str, int]]:
        if self.initial_pending:
            self.initial_pending = False
        elif self.telemetry_pending:
            self.telemetry_pending = False
            self.radio_altitude_ft -= 0.4
        else:
            raise BlockingIOError
        state = asdict(
            make_state(
                RadioAltitude=self.radio_altitude_ft,
                VerticalSpeed=-720.0,
            )
        )
        return json.dumps(state).encode("utf-8"), self.sender

    def close(self) -> None:
        pass


class AltitudeAuthoritySweepTests(unittest.TestCase):
    def test_parses_strictly_descending_altitudes(self) -> None:
        self.assertEqual(
            parse_altitudes("400, 300,200,150"), (400.0, 300.0, 200.0, 150.0)
        )

    def test_rejects_bad_altitude_sequences(self) -> None:
        for value in ("", "400,400", "300,400", "400,0", "400,nan"):
            with (
                self.subTest(value=value),
                self.assertRaises(argparse.ArgumentTypeError),
            ):
                parse_altitudes(value)

    def test_output_owns_only_pitch_and_roll_commands(self) -> None:
        output = make_output(ControlModeState.Approach, 0.5, -5.0, 0.36)

        self.assertEqual(output.ControlValidMask, TEST_CONTROL_VALID_MASK)
        self.assertEqual(output.ControlMode, ControlModeState.Approach)
        self.assertEqual(output.ModeAIReady, 1)
        self.assertEqual(output.ElevatorCmd, 0.5)
        self.assertEqual(output.AileronCmd, -5.0)
        self.assertEqual(output.RudderCmd, 0.0)
        self.assertEqual(output.ThrottleLeftRate, 0.0)
        self.assertEqual(output.ThrottleRightRate, 0.0)
        self.assertEqual(output.ThrottleLeft, 0.36)
        self.assertEqual(output.ThrottleRight, 0.36)
        self.assertEqual(output.ModeFlare, 0)
        self.assertEqual(output.ModeAlign, 0)

    def test_nominal_state_is_safe(self) -> None:
        self.assertIsNone(safety_reason(make_state(), safety_limits()))

    def test_safety_abort_conditions_are_detected(self) -> None:
        cases = (
            ({"AgentIsActive": 0}, "inactive"),
            ({"LeftGearWeightOnWheels": 1}, "weight-on-wheels"),
            ({"RadioAltitudeValid": 0}, "radio altitude became invalid"),
            ({"RadioAltitude": float("nan")}, "radio altitude became non-finite"),
            ({"RadioAltitude": 99.0}, "fell below"),
            ({"PitchAngleValid": 0}, "pitch angle became invalid"),
            ({"PitchAngle": float("nan")}, "pitch angle became non-finite"),
            ({"PitchAngle": 15.1}, "absolute pitch"),
            ({"RollAngleValid": 0}, "roll angle became invalid"),
            ({"RollAngle": float("nan")}, "roll angle became non-finite"),
            ({"RollAngle": -15.1}, "absolute roll"),
            ({"IndicatedAirspeedValid": 0}, "airspeed became invalid"),
            ({"IndicatedAirspeed": float("nan")}, "airspeed became non-finite"),
            ({"IndicatedAirspeed": 119.0}, "IAS left"),
            ({"VerticalSpeedValid": 0}, "vertical speed became invalid"),
            ({"VerticalSpeed": float("nan")}, "vertical speed became non-finite"),
            ({"VerticalSpeed": -2001.0}, "vertical speed fell below"),
        )
        for overrides, expected in cases:
            with self.subTest(overrides=overrides):
                reason = safety_reason(make_state(**overrides), safety_limits())
                self.assertIsNotNone(reason)
                self.assertIn(expected, reason or "")

    def test_default_safety_arguments_are_accepted(self) -> None:
        validate_args(argparse.ArgumentParser(add_help=False), valid_args())

    def test_landing_switch_must_match_a_non_final_target(self) -> None:
        parser = argparse.ArgumentParser(add_help=False)
        validate_args(parser, valid_args(landing_after_ft=400.0))
        for landing_after_ft in (350.0, 150.0, float("nan")):
            with self.subTest(landing_after_ft=landing_after_ft):
                with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    validate_args(
                        parser,
                        valid_args(landing_after_ft=landing_after_ft),
                    )

    def test_low_altitude_sweep_cannot_loosen_hard_safety_limits(self) -> None:
        unsafe_overrides = (
            {"elevator_step": 0.51},
            {"aileron_step": 5.01},
            {"pulse_seconds": 0.251},
            {"activation_min_ra_ft": 499.9},
            {"abort_ra_ft": 99.9},
            {"min_ias": 119.9},
            {"max_ias": 190.1},
            {"min_vs_fpm": -2000.1},
            {"max_abs_pitch": 15.1},
            {"max_abs_roll": 15.1},
            {"max_target_overshoot_ft": 20.1},
            {"telemetry_timeout": 1.01},
        )
        parser = argparse.ArgumentParser(add_help=False)
        for overrides in unsafe_overrides:
            with self.subTest(overrides=overrides):
                with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    validate_args(parser, valid_args(**overrides))

    def test_telemetry_row_matches_csv_schema_and_records_modes(self) -> None:
        output = make_output(ControlModeState.Approach, 0.5, 5.0, 0.36)
        row = telemetry_row(
            sequence=7,
            start_time=0.0,
            segment="pitch_300ft",
            axis="pitch",
            target_ra_ft=300.0,
            output=output,
            throttle_hold=0.36,
            state=make_state(RadioAltitude=299.5),
        )

        self.assertEqual(tuple(row), LOG_COLUMNS)
        self.assertEqual(row["sequence"], 7)
        self.assertEqual(row["control_mode"], "Approach")
        self.assertEqual(row["mode_ai_ready"], 1)
        self.assertEqual(row["agent_active"], 1)
        self.assertEqual(row["flight_phase"], 10)
        self.assertEqual(row["elevator_cmd_g"], 0.5)
        self.assertEqual(row["aileron_cmd_deg"], 5.0)
        self.assertEqual(row["ra_ft"], 299.5)

    def test_aileron_response_uses_differential_not_cancelling_average(self) -> None:
        row = {
            "aileron_left_deg": 4.0,
            "aileron_right_deg": -4.0,
        }
        self.assertEqual(
            differential_surface(row, "aileron_left_deg", "aileron_right_deg"),
            4.0,
        )

    def test_full_sweep_uses_one_continuous_approach_activation(self) -> None:
        fake_socket = FakeTelemetrySocket()
        fake_clock = FakeClock()
        with tempfile.TemporaryDirectory() as temp_dir:
            log_path = Path(temp_dir) / "sweep.csv"
            argv = [
                "ics_altitude_authority_sweep.py",
                "--log",
                str(log_path),
            ]
            with (
                patch(
                    "ics_altitude_authority_sweep.socket.socket",
                    return_value=fake_socket,
                ),
                patch(
                    "ics_altitude_authority_sweep.time.monotonic", fake_clock.monotonic
                ),
                patch("ics_altitude_authority_sweep.time.sleep", fake_clock.sleep),
                patch.object(sys, "argv", argv),
                redirect_stdout(io.StringIO()),
            ):
                exit_code = main()

            self.assertEqual(exit_code, 0)
            self.assertTrue(log_path.exists())

        packets = fake_socket.sent_packets
        compressed_modes: list[int] = []
        for packet, _radio_altitude_ft in packets:
            mode = int(packet["ControlMode"])
            if not compressed_modes or compressed_modes[-1] != mode:
                compressed_modes.append(mode)
        self.assertEqual(compressed_modes, [0, 1, 0])

        first_approach = next(
            (packet, altitude)
            for packet, altitude in packets
            if packet["ControlMode"] == 1
        )
        self.assertGreater(first_approach[1], 500.0)

        nonzero_commands: list[tuple[float, float]] = []
        previous = (0.0, 0.0)
        for packet, _radio_altitude_ft in packets:
            if packet["ControlMode"] != 1:
                continue
            command = (
                float(packet["ElevatorCmd"]),
                float(packet["AileronCmd"]),
            )
            if command != previous and command != (0.0, 0.0):
                nonzero_commands.append(command)
            previous = command
        self.assertEqual(
            nonzero_commands,
            [
                (0.5, 0.0),
                (0.0, 5.0),
                (0.5, 0.0),
                (0.0, -5.0),
                (0.5, 0.0),
                (0.0, 5.0),
                (0.5, 0.0),
                (0.0, -5.0),
            ],
        )
        approach_seen = False
        shutdown_seen = False
        for packet, _radio_altitude_ft in packets:
            mode = int(packet["ControlMode"])
            if mode == 1:
                approach_seen = True
                self.assertFalse(shutdown_seen)
                self.assertEqual(packet["ControlValidMask"], 3)
            elif not approach_seen:
                self.assertEqual(packet["ControlValidMask"], 3)
            else:
                shutdown_seen = True
                self.assertEqual(packet["ControlValidMask"], 0)
        self.assertTrue(shutdown_seen)

    def test_activation_is_refused_if_arming_drops_below_500_ft(self) -> None:
        fake_socket = FakeTelemetrySocket(radio_altitude_ft=520.0)
        fake_clock = FakeClock()
        with tempfile.TemporaryDirectory() as temp_dir:
            argv = [
                "ics_altitude_authority_sweep.py",
                "--log",
                str(Path(temp_dir) / "aborted.csv"),
            ]
            with (
                patch(
                    "ics_altitude_authority_sweep.socket.socket",
                    return_value=fake_socket,
                ),
                patch(
                    "ics_altitude_authority_sweep.time.monotonic", fake_clock.monotonic
                ),
                patch("ics_altitude_authority_sweep.time.sleep", fake_clock.sleep),
                patch.object(sys, "argv", argv),
                redirect_stdout(io.StringIO()),
            ):
                exit_code = main()

        self.assertEqual(exit_code, 4)
        self.assertFalse(
            any(
                packet["ControlMode"] == 1
                for packet, _altitude in fake_socket.sent_packets
            )
        )
        self.assertEqual(fake_socket.sent_packets[-1][0]["ControlMode"], 0)
        self.assertEqual(fake_socket.sent_packets[-1][0]["ControlValidMask"], 0)

    def test_optional_landing_switch_happens_once_after_400ft_test(self) -> None:
        fake_socket = FakeTelemetrySocket()
        fake_clock = FakeClock()
        with tempfile.TemporaryDirectory() as temp_dir:
            argv = [
                "ics_altitude_authority_sweep.py",
                "--landing-after-ft",
                "400",
                "--log",
                str(Path(temp_dir) / "landing-switch.csv"),
            ]
            with (
                patch(
                    "ics_altitude_authority_sweep.socket.socket",
                    return_value=fake_socket,
                ),
                patch(
                    "ics_altitude_authority_sweep.time.monotonic",
                    fake_clock.monotonic,
                ),
                patch("ics_altitude_authority_sweep.time.sleep", fake_clock.sleep),
                patch.object(sys, "argv", argv),
                redirect_stdout(io.StringIO()),
            ):
                exit_code = main()

        self.assertEqual(exit_code, 0)
        packets = fake_socket.sent_packets
        compressed_modes: list[int] = []
        for packet, _radio_altitude_ft in packets:
            mode = int(packet["ControlMode"])
            if not compressed_modes or compressed_modes[-1] != mode:
                compressed_modes.append(mode)
        self.assertEqual(compressed_modes, [0, 1, 2, 0])

        first_landing = next(
            (packet, altitude)
            for packet, altitude in packets
            if packet["ControlMode"] == 2
        )
        self.assertLess(first_landing[1], 400.0)
        self.assertGreater(first_landing[1], 300.0)

        command_transitions: list[tuple[int, float, float]] = []
        previous = (0, 0.0, 0.0)
        for packet, _radio_altitude_ft in packets:
            mode = int(packet["ControlMode"])
            if mode not in (1, 2):
                continue
            command = (
                mode,
                float(packet["ElevatorCmd"]),
                float(packet["AileronCmd"]),
            )
            if command != previous and command[1:] != (0.0, 0.0):
                command_transitions.append(command)
            previous = command
        self.assertEqual(
            command_transitions,
            [
                (1, 0.5, 0.0),
                (1, 0.0, 5.0),
                (2, 0.5, 0.0),
                (2, 0.0, -5.0),
                (2, 0.5, 0.0),
                (2, 0.0, 5.0),
                (2, 0.5, 0.0),
                (2, 0.0, -5.0),
            ],
        )
        for packet, _radio_altitude_ft in packets:
            if packet["ControlMode"] in (1, 2):
                self.assertEqual(packet["ControlValidMask"], 3)
                self.assertEqual(packet["ModeAIReady"], 1)
                self.assertEqual(packet["ModeFlare"], 0)
                self.assertEqual(packet["ModeAlign"], 0)
                self.assertEqual(packet["ModeRollout"], 0)


if __name__ == "__main__":
    unittest.main()
