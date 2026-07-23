#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import socket
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ics_protocol import ControlModeState, ICSInputs, ICSOutputs


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TARGET_ALTITUDES_FT = (400.0, 300.0, 200.0, 150.0)
MIN_TEST_ACTIVATION_RA_FT = 500.0
MAX_ELEVATOR_STEP_G = 0.5
MAX_AILERON_STEP_DEG = 5.0
MAX_PULSE_SECONDS = 0.25
MIN_ABORT_RA_FT = 100.0
MAX_TARGET_OVERSHOOT_FT = 20.0
MAX_TELEMETRY_TIMEOUT_S = 1.0
ELEVATOR_VALID_MASK = 1 << 0
AILERON_VALID_MASK = 1 << 1
TEST_CONTROL_VALID_MASK = ELEVATOR_VALID_MASK | AILERON_VALID_MASK
THROTTLE_FORWARD_MAX_DEG = 55.7

LOG_COLUMNS = (
    "sequence",
    "time_s",
    "segment",
    "axis",
    "target_ra_ft",
    "event",
    "control_valid_mask",
    "control_mode",
    "mode_ai_ready",
    "elevator_cmd_g",
    "aileron_cmd_deg",
    "throttle_hold_norm",
    "agent_active",
    "flight_phase_valid",
    "flight_phase",
    "ra_valid",
    "ra_ft",
    "ias_valid",
    "ias_kt",
    "ground_speed_kt",
    "vs_valid",
    "vs_fpm",
    "pitch_valid",
    "pitch_deg",
    "roll_valid",
    "roll_deg",
    "body_pitch_rate_valid",
    "body_pitch_rate_deg_s",
    "body_roll_rate_valid",
    "body_roll_rate_deg_s",
    "body_yaw_rate_valid",
    "body_yaw_rate_deg_s",
    "body_norm_accel_valid",
    "body_norm_accel_g",
    "body_lat_accel_valid",
    "body_lat_accel_g",
    "stabilizer_deg",
    "elevator_left_deg",
    "elevator_right_deg",
    "aileron_left_deg",
    "aileron_right_deg",
    "rudder_deg",
    "flaps_deg",
    "slats_deg",
    "left_throttle_deg",
    "right_throttle_deg",
    "nose_gear_wow",
    "left_gear_wow",
    "right_gear_wow",
)


class SafetyAbort(RuntimeError):
    pass


@dataclass(frozen=True)
class SafetyLimits:
    abort_radio_altitude_ft: float
    max_abs_pitch_deg: float
    max_abs_roll_deg: float
    min_ias_kt: float
    max_ias_kt: float
    min_vertical_speed_fpm: float


def parse_altitudes(raw: str) -> tuple[float, ...]:
    try:
        values = tuple(float(item.strip()) for item in raw.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "altitudes must be a comma-separated list of numbers"
        ) from exc
    if not values or any(not math.isfinite(value) or value <= 0.0 for value in values):
        raise argparse.ArgumentTypeError("altitudes must be positive finite values")
    if any(current <= following for current, following in zip(values, values[1:])):
        raise argparse.ArgumentTypeError("altitudes must be strictly descending")
    return values


def clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


def main_gear_contact(state: ICSInputs) -> bool:
    return bool(state.LeftGearWeightOnWheels or state.RightGearWeightOnWheels)


def safety_reason(state: ICSInputs, limits: SafetyLimits) -> str | None:
    if state.AgentIsActive != 1:
        return "ICS became inactive"
    if main_gear_contact(state):
        return "main-gear weight-on-wheels detected"
    if not state.RadioAltitudeValid:
        return "radio altitude became invalid"
    if not math.isfinite(state.RadioAltitude):
        return "radio altitude became non-finite"
    if state.RadioAltitude < limits.abort_radio_altitude_ft:
        return (
            "radio altitude fell below "
            f"{limits.abort_radio_altitude_ft:g} ft ({state.RadioAltitude:.1f})"
        )
    if not state.PitchAngleValid:
        return "pitch angle became invalid"
    if not math.isfinite(state.PitchAngle):
        return "pitch angle became non-finite"
    if abs(state.PitchAngle) > limits.max_abs_pitch_deg:
        return (
            f"absolute pitch exceeded {limits.max_abs_pitch_deg:g} deg "
            f"({state.PitchAngle:.2f})"
        )
    if not state.RollAngleValid:
        return "roll angle became invalid"
    if not math.isfinite(state.RollAngle):
        return "roll angle became non-finite"
    if abs(state.RollAngle) > limits.max_abs_roll_deg:
        return (
            f"absolute roll exceeded {limits.max_abs_roll_deg:g} deg "
            f"({state.RollAngle:.2f})"
        )
    if not state.IndicatedAirspeedValid:
        return "indicated airspeed became invalid"
    if not math.isfinite(state.IndicatedAirspeed):
        return "indicated airspeed became non-finite"
    if not limits.min_ias_kt <= state.IndicatedAirspeed <= limits.max_ias_kt:
        return (
            f"IAS left {limits.min_ias_kt:g}..{limits.max_ias_kt:g} kt "
            f"({state.IndicatedAirspeed:.1f})"
        )
    if not state.VerticalSpeedValid:
        return "vertical speed became invalid"
    if not math.isfinite(state.VerticalSpeed):
        return "vertical speed became non-finite"
    if state.VerticalSpeed < limits.min_vertical_speed_fpm:
        return (
            f"vertical speed fell below {limits.min_vertical_speed_fpm:g} fpm "
            f"({state.VerticalSpeed:.0f})"
        )
    return None


def receive_first(
    sock: socket.socket,
    timeout_s: float,
) -> tuple[ICSInputs, tuple[str, int]] | None:
    sock.settimeout(timeout_s)
    try:
        data, sender = sock.recvfrom(65535)
    except socket.timeout:
        return None
    return ICSInputs.from_json_bytes(data), sender


def make_output(
    control_mode: ControlModeState,
    elevator_g: float,
    aileron_deg: float,
    throttle_hold: float,
) -> ICSOutputs:
    return ICSOutputs(
        ControlValidMask=TEST_CONTROL_VALID_MASK,
        ControlMode=control_mode,
        ElevatorCmd=elevator_g,
        AileronCmd=aileron_deg,
        # These fields are outside TEST_CONTROL_VALID_MASK. They carry the
        # measured shared position only to protect simulator builds that ignore
        # the mask from interpreting their default zero as an idle command.
        ThrottleLeft=throttle_hold,
        ThrottleRight=throttle_hold,
        ModeAIReady=1,
    )


def telemetry_row(
    sequence: int,
    start_time: float,
    segment: str,
    axis: str,
    target_ra_ft: float | None,
    output: ICSOutputs,
    throttle_hold: float,
    state: ICSInputs,
) -> dict[str, Any]:
    return {
        "sequence": sequence,
        "time_s": time.monotonic() - start_time,
        "segment": segment,
        "axis": axis,
        "target_ra_ft": "" if target_ra_ft is None else target_ra_ft,
        "event": "",
        "control_valid_mask": output.ControlValidMask,
        "control_mode": output.ControlMode.name,
        "mode_ai_ready": output.ModeAIReady,
        "elevator_cmd_g": output.ElevatorCmd,
        "aileron_cmd_deg": output.AileronCmd,
        "throttle_hold_norm": throttle_hold,
        "agent_active": state.AgentIsActive,
        "flight_phase_valid": state.FlightPhaseValid,
        "flight_phase": state.FlightPhase,
        "ra_valid": state.RadioAltitudeValid,
        "ra_ft": state.RadioAltitude,
        "ias_valid": state.IndicatedAirspeedValid,
        "ias_kt": state.IndicatedAirspeed,
        "ground_speed_kt": state.GroundSpeed,
        "vs_valid": state.VerticalSpeedValid,
        "vs_fpm": state.VerticalSpeed,
        "pitch_valid": state.PitchAngleValid,
        "pitch_deg": state.PitchAngle,
        "roll_valid": state.RollAngleValid,
        "roll_deg": state.RollAngle,
        "body_pitch_rate_valid": state.BodyPitchRateValid,
        "body_pitch_rate_deg_s": state.BodyPitchRate,
        "body_roll_rate_valid": state.BodyRollRateValid,
        "body_roll_rate_deg_s": state.BodyRollRate,
        "body_yaw_rate_valid": state.BodyYawRateValid,
        "body_yaw_rate_deg_s": state.BodyYawRate,
        "body_norm_accel_valid": state.BodyNormAccelValid,
        "body_norm_accel_g": state.BodyNormAccel,
        "body_lat_accel_valid": state.BodyLatAccelValid,
        "body_lat_accel_g": state.BodyLatAccel,
        "stabilizer_deg": state.StabilizerAngle,
        "elevator_left_deg": state.ElevatorLeftAngle,
        "elevator_right_deg": state.ElevatorRightAngle,
        "aileron_left_deg": state.AileronLeftAngle,
        "aileron_right_deg": state.AileronRightAngle,
        "rudder_deg": state.RudderAngle,
        "flaps_deg": state.FlapsAngle,
        "slats_deg": state.SlatsAngle,
        "left_throttle_deg": state.LeftThrottleAngle,
        "right_throttle_deg": state.RightThrottleAngle,
        "nose_gear_wow": state.NoseGearWeightOnWheels,
        "left_gear_wow": state.LeftGearWeightOnWheels,
        "right_gear_wow": state.RightGearWeightOnWheels,
    }


class ContinuousSweepSession:
    def __init__(
        self,
        sock: socket.socket,
        sender: tuple[str, int],
        rate_hz: float,
        telemetry_timeout_s: float,
        safety_limits: SafetyLimits,
        throttle_hold: float,
        start_time: float,
        rows: list[dict[str, Any]],
    ) -> None:
        self.sock = sock
        self.sender = sender
        self.period_s = 1.0 / rate_hz
        self.telemetry_timeout_s = telemetry_timeout_s
        self.safety_limits = safety_limits
        self.throttle_hold = throttle_hold
        self.start_time = start_time
        self.rows = rows

    def run_duration(
        self,
        segment: str,
        axis: str,
        target_ra_ft: float | None,
        output: ICSOutputs,
        duration_s: float,
    ) -> list[dict[str, Any]]:
        return self._run(
            segment=segment,
            axis=axis,
            target_ra_ft=target_ra_ft,
            output=output,
            duration_s=duration_s,
            wait_timeout_s=None,
        )

    def wait_for_altitude(
        self,
        target_ra_ft: float,
        output: ICSOutputs,
        wait_timeout_s: float,
    ) -> list[dict[str, Any]]:
        return self._run(
            segment=f"wait_{target_ra_ft:g}ft",
            axis="neutral",
            target_ra_ft=target_ra_ft,
            output=output,
            duration_s=None,
            wait_timeout_s=wait_timeout_s,
        )

    def _run(
        self,
        segment: str,
        axis: str,
        target_ra_ft: float | None,
        output: ICSOutputs,
        duration_s: float | None,
        wait_timeout_s: float | None,
    ) -> list[dict[str, Any]]:
        if (duration_s is None) == (wait_timeout_s is None):
            raise ValueError("provide exactly one of duration_s or wait_timeout_s")
        segment_start = len(self.rows)
        packet = output.to_json_bytes()
        now = time.monotonic()
        deadline = now + (
            duration_s if duration_s is not None else wait_timeout_s or 0.0
        )
        next_send = now
        last_telemetry_at = now
        self.sock.setblocking(False)
        try:
            while True:
                now = time.monotonic()
                if now >= next_send:
                    self.sock.sendto(packet, self.sender)
                    next_send = now + self.period_s

                for _ in range(128):
                    try:
                        data, telemetry_sender = self.sock.recvfrom(65535)
                    except BlockingIOError:
                        break
                    if telemetry_sender != self.sender:
                        continue
                    state = ICSInputs.from_json_bytes(data)
                    last_telemetry_at = time.monotonic()
                    row = telemetry_row(
                        sequence=len(self.rows),
                        start_time=self.start_time,
                        segment=segment,
                        axis=axis,
                        target_ra_ft=target_ra_ft,
                        output=output,
                        throttle_hold=self.throttle_hold,
                        state=state,
                    )
                    reason = safety_reason(state, self.safety_limits)
                    if reason is not None:
                        row["event"] = f"SAFETY_ABORT: {reason}"
                        self.rows.append(row)
                        raise SafetyAbort(reason)
                    self.rows.append(row)
                    if (
                        duration_s is None
                        and target_ra_ft is not None
                        and state.RadioAltitude <= target_ra_ft
                    ):
                        return self.rows[segment_start:]

                now = time.monotonic()
                if duration_s is not None and now >= deadline:
                    return self.rows[segment_start:]
                if duration_s is None and now >= deadline:
                    reason = f"timed out waiting for {target_ra_ft:g} ft"
                    self._mark_last_event(reason)
                    raise SafetyAbort(reason)
                if now - last_telemetry_at > self.telemetry_timeout_s:
                    reason = "telemetry timeout"
                    self._mark_last_event(reason)
                    raise SafetyAbort(reason)
                time.sleep(min(0.005, max(0.0, next_send - now)))
        finally:
            self.sock.setblocking(True)

    def _mark_last_event(self, reason: str) -> None:
        if self.rows:
            self.rows[-1]["event"] = f"SAFETY_ABORT: {reason}"


def average_surface(row: dict[str, Any], left_key: str, right_key: str) -> float:
    return 0.5 * (float(row[left_key]) + float(row[right_key]))


def differential_surface(row: dict[str, Any], left_key: str, right_key: str) -> float:
    return 0.5 * (float(row[left_key]) - float(row[right_key]))


def print_pitch_summary(
    target_ra_ft: float,
    baseline: dict[str, Any],
    pulse_rows: list[dict[str, Any]],
    recovery_rows: list[dict[str, Any]],
) -> None:
    if not pulse_rows:
        print(f"RA {target_ra_ft:g}: pitch pulse received no telemetry")
        return
    pitch0 = float(baseline["pitch_deg"])
    vs0 = float(baseline["vs_fpm"])
    elevator0 = average_surface(baseline, "elevator_left_deg", "elevator_right_deg")
    end = pulse_rows[-1]
    pitch_deltas = [float(row["pitch_deg"]) - pitch0 for row in pulse_rows]
    rates = [float(row["body_pitch_rate_deg_s"]) for row in pulse_rows]
    nz_values = [float(row["body_norm_accel_g"]) for row in pulse_rows]
    elevator_end = average_surface(end, "elevator_left_deg", "elevator_right_deg")
    recovery_pitch = (
        float(recovery_rows[-1]["pitch_deg"])
        if recovery_rows
        else float(end["pitch_deg"])
    )
    print(
        f"RA {target_ra_ft:g} pitch cmd={float(end['elevator_cmd_g']):+.2f}g "
        f"actual={float(baseline['ra_ft']):.1f}->"
        f"{float(end['ra_ft']):.1f}ft "
        f"dPitchEnd={pitch_deltas[-1]:+.3f}deg "
        f"dPitchPeak={max(abs(value) for value in pitch_deltas):.3f}deg "
        f"q={min(rates):+.3f}..{max(rates):+.3f}deg/s "
        f"dVS={float(end['vs_fpm']) - vs0:+.0f}fpm "
        f"dElev={elevator_end - elevator0:+.3f}deg "
        f"nz={min(nz_values):+.3f}..{max(nz_values):+.3f}g "
        f"recoveryPitch={recovery_pitch:+.3f}deg"
    )


def print_roll_summary(
    target_ra_ft: float,
    baseline: dict[str, Any],
    pulse_rows: list[dict[str, Any]],
    recovery_rows: list[dict[str, Any]],
) -> None:
    if not pulse_rows:
        print(f"RA {target_ra_ft:g}: roll pulse received no telemetry")
        return
    roll0 = float(baseline["roll_deg"])
    aileron0 = differential_surface(
        baseline,
        "aileron_left_deg",
        "aileron_right_deg",
    )
    end = pulse_rows[-1]
    roll_deltas = [float(row["roll_deg"]) - roll0 for row in pulse_rows]
    rates = [float(row["body_roll_rate_deg_s"]) for row in pulse_rows]
    lat_values = [float(row["body_lat_accel_g"]) for row in pulse_rows]
    aileron_end = differential_surface(
        end,
        "aileron_left_deg",
        "aileron_right_deg",
    )
    recovery_roll = (
        float(recovery_rows[-1]["roll_deg"])
        if recovery_rows
        else float(end["roll_deg"])
    )
    print(
        f"RA {target_ra_ft:g} roll cmd={float(end['aileron_cmd_deg']):+.2f}deg "
        f"actual={float(baseline['ra_ft']):.1f}->"
        f"{float(end['ra_ft']):.1f}ft "
        f"dRollEnd={roll_deltas[-1]:+.3f}deg "
        f"dRollPeak={max(abs(value) for value in roll_deltas):.3f}deg "
        f"p={min(rates):+.3f}..{max(rates):+.3f}deg/s "
        f"dAilDiff={aileron_end - aileron0:+.3f}deg "
        f"ny={min(lat_values):+.3f}..{max(lat_values):+.3f}g "
        f"recoveryRoll={recovery_roll:+.3f}deg"
    )


def send_off(
    sock: socket.socket,
    sender: tuple[str, int],
    rate_hz: float,
    duration_s: float = 0.5,
) -> None:
    packet = ICSOutputs(
        ControlValidMask=0, ControlMode=ControlModeState.Off
    ).to_json_bytes()
    period_s = 1.0 / rate_hz
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        sock.sendto(packet, sender)
        time.sleep(period_s)


def validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    positive_names = (
        "pulse_seconds",
        "recovery_seconds",
        "initial_settle_seconds",
        "rate_hz",
        "arm_seconds",
        "telemetry_timeout",
        "max_wait_seconds",
        "max_target_overshoot_ft",
        "max_abs_pitch",
        "max_abs_roll",
        "wait_timeout",
    )
    for name in positive_names:
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be a positive finite number")
    if args.arm_seconds < 2.0:
        parser.error("--arm-seconds must be at least 2 seconds")
    if not math.isfinite(args.elevator_step) or args.elevator_step <= 0.0:
        parser.error("--elevator-step must be a positive finite number")
    if args.elevator_step > MAX_ELEVATOR_STEP_G:
        parser.error(f"--elevator-step must not exceed {MAX_ELEVATOR_STEP_G:g} g")
    if not math.isfinite(args.aileron_step) or args.aileron_step <= 0.0:
        parser.error("--aileron-step must be a positive finite number")
    if args.aileron_step > MAX_AILERON_STEP_DEG:
        parser.error(f"--aileron-step must not exceed {MAX_AILERON_STEP_DEG:g} deg")
    if args.pulse_seconds > MAX_PULSE_SECONDS:
        parser.error(f"--pulse-seconds must not exceed {MAX_PULSE_SECONDS:g} seconds")
    if (
        not math.isfinite(args.activation_min_ra_ft)
        or args.activation_min_ra_ft < MIN_TEST_ACTIVATION_RA_FT
    ):
        parser.error(
            "--activation-min-ra-ft must be finite and at least "
            f"{MIN_TEST_ACTIVATION_RA_FT:g} ft"
        )
    if not math.isfinite(args.abort_ra_ft) or args.abort_ra_ft < MIN_ABORT_RA_FT:
        parser.error(
            f"--abort-ra-ft must be finite and at least {MIN_ABORT_RA_FT:g} ft"
        )
    if min(args.altitudes) <= args.abort_ra_ft:
        parser.error("every test altitude must be above --abort-ra-ft")
    if max(args.altitudes) >= args.activation_min_ra_ft:
        parser.error("the highest test altitude must be below --activation-min-ra-ft")
    if args.landing_after_ft is not None:
        if not math.isfinite(args.landing_after_ft):
            parser.error("--landing-after-ft must be finite")
        matching_targets = [
            altitude
            for altitude in args.altitudes
            if math.isclose(altitude, args.landing_after_ft, abs_tol=1e-6)
        ]
        if not matching_targets:
            parser.error("--landing-after-ft must exactly match one test altitude")
        if math.isclose(
            args.landing_after_ft,
            args.altitudes[-1],
            abs_tol=1e-6,
        ):
            parser.error(
                "--landing-after-ft must leave at least one lower test altitude"
            )
    if (
        not math.isfinite(args.min_ias)
        or not math.isfinite(args.max_ias)
        or args.min_ias >= args.max_ias
    ):
        parser.error("--min-ias and --max-ias must be finite and ordered")
    if args.min_ias < 120.0 or args.max_ias > 190.0:
        parser.error("IAS safety limits may only be made stricter than 120..190 kt")
    if not math.isfinite(args.min_vs_fpm):
        parser.error("--min-vs-fpm must be finite")
    if args.min_vs_fpm < -2000.0:
        parser.error("--min-vs-fpm must not be below -2000 fpm")
    if args.max_abs_pitch > 15.0 or args.max_abs_roll > 15.0:
        parser.error("attitude safety limits must not exceed 15 deg")
    if args.max_target_overshoot_ft > MAX_TARGET_OVERSHOOT_FT:
        parser.error(
            f"--max-target-overshoot-ft must not exceed {MAX_TARGET_OVERSHOOT_FT:g} ft"
        )
    if args.telemetry_timeout > MAX_TELEMETRY_TIMEOUT_S:
        parser.error(
            f"--telemetry-timeout must not exceed {MAX_TELEMETRY_TIMEOUT_S:g} seconds"
        )
    if not 1 <= args.bind_port <= 65535:
        parser.error("--bind-port must be in 1..65535")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Activate ICS once above 500 ft, then measure pitch and roll authority "
            "during one continuous descent."
        )
    )
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--bind-port", type=int, default=3030)
    parser.add_argument("--wait-timeout", type=float, default=10.0)
    parser.add_argument(
        "--altitudes", type=parse_altitudes, default=DEFAULT_TARGET_ALTITUDES_FT
    )
    parser.add_argument("--elevator-step", type=float, default=0.5)
    parser.add_argument("--aileron-step", type=float, default=5.0)
    parser.add_argument("--pulse-seconds", type=float, default=0.25)
    parser.add_argument("--recovery-seconds", type=float, default=0.75)
    parser.add_argument("--initial-settle-seconds", type=float, default=0.5)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--arm-seconds", type=float, default=2.2)
    parser.add_argument("--activation-min-ra-ft", type=float, default=500.0)
    parser.add_argument(
        "--landing-after-ft",
        type=float,
        help=(
            "after completing this target in Approach, switch once to Landing "
            "for all lower targets"
        ),
    )
    parser.add_argument("--abort-ra-ft", type=float, default=100.0)
    parser.add_argument("--max-target-overshoot-ft", type=float, default=20.0)
    parser.add_argument("--telemetry-timeout", type=float, default=1.0)
    parser.add_argument("--max-wait-seconds", type=float, default=90.0)
    parser.add_argument("--max-abs-pitch", type=float, default=15.0)
    parser.add_argument("--max-abs-roll", type=float, default=15.0)
    parser.add_argument("--min-ias", type=float, default=120.0)
    parser.add_argument("--max-ias", type=float, default=190.0)
    parser.add_argument("--min-vs-fpm", type=float, default=-2000.0)
    parser.add_argument("--log", type=Path)
    args = parser.parse_args()
    validate_args(parser, args)

    limits = SafetyLimits(
        abort_radio_altitude_ft=args.abort_ra_ft,
        max_abs_pitch_deg=args.max_abs_pitch,
        max_abs_roll_deg=args.max_abs_roll,
        min_ias_kt=args.min_ias,
        max_ias_kt=args.max_ias,
        min_vertical_speed_fpm=args.min_vs_fpm,
    )
    log_path = args.log or (
        ROOT / "logs" / time.strftime("ics_altitude_authority_%Y%m%d_%H%M%S.csv")
    )
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind_ip, args.bind_port))
    sender: tuple[str, int] | None = None
    rows: list[dict[str, Any]] = []
    commands_started = False
    exit_code = 0
    start_time = time.monotonic()

    try:
        print(f"waiting on udp://{args.bind_ip}:{args.bind_port}")
        first = receive_first(sock, args.wait_timeout)
        if first is None:
            print("no telemetry received; not sending")
            return 2
        state, sender = first
        print(
            f"from={sender} active={state.AgentIsActive} phase={state.FlightPhase} "
            f"ra={state.RadioAltitude:.1f}ft ias={state.IndicatedAirspeed:.1f}kt "
            f"vs={state.VerticalSpeed:.0f}fpm pitch={state.PitchAngle:.2f}deg "
            f"roll={state.RollAngle:.2f}deg"
        )
        roll_commands = tuple(
            args.aileron_step if index % 2 == 0 else -args.aileron_step
            for index in range(len(args.altitudes))
        )
        print(
            f"targets={','.join(f'{value:g}' for value in args.altitudes)}ft "
            f"pitch={args.elevator_step:+.2f}g/{args.pulse_seconds:g}s "
            "roll="
            f"{','.join(f'{value:+.2f}' for value in roll_commands)}deg/"
            f"{args.pulse_seconds:g}s"
        )
        if args.landing_after_ft is not None:
            print(
                "mode test: Approach through "
                f"{args.landing_after_ft:g}ft, then one switch to Landing"
            )
        reason = safety_reason(state, limits)
        if reason is not None:
            print(f"precondition failed: {reason}")
            return 3
        if state.RadioAltitude <= args.activation_min_ra_ft:
            print(
                "precondition failed: start above "
                f"{args.activation_min_ra_ft:g} ft for one valid ICS activation"
            )
            return 3

        throttle_hold = clamp(
            0.5
            * (state.LeftThrottleAngle + state.RightThrottleAngle)
            / THROTTLE_FORWARD_MAX_DEG,
            0.0,
            1.0,
        )
        arm_output = make_output(ControlModeState.Off, 0.0, 0.0, throttle_hold)
        active_mode = ControlModeState.Approach
        neutral_output = make_output(active_mode, 0.0, 0.0, throttle_hold)
        rows.append(
            telemetry_row(
                sequence=0,
                start_time=start_time,
                segment="initial",
                axis="neutral",
                target_ra_ft=None,
                output=arm_output,
                throttle_hold=throttle_hold,
                state=state,
            )
        )
        session = ContinuousSweepSession(
            sock=sock,
            sender=sender,
            rate_hz=args.rate_hz,
            telemetry_timeout_s=args.telemetry_timeout,
            safety_limits=limits,
            throttle_hold=throttle_hold,
            start_time=start_time,
            rows=rows,
        )

        print(
            f"arming for one activation above {args.activation_min_ra_ft:g} ft: "
            f"ModeAIReady=1 / ControlMode=Off for {args.arm_seconds:g}s"
        )
        commands_started = True
        arm_rows = session.run_duration(
            "arm",
            "neutral",
            None,
            arm_output,
            args.arm_seconds,
        )
        if not arm_rows:
            raise SafetyAbort("no telemetry received during arming")
        activation_ra = float(arm_rows[-1]["ra_ft"])
        if activation_ra <= args.activation_min_ra_ft:
            raise SafetyAbort(
                "radio altitude reached "
                f"{activation_ra:.1f} ft before the required above-"
                f"{args.activation_min_ra_ft:g}-ft Off -> Approach activation"
            )

        print(f"activating once at RA={activation_ra:.1f}ft: Off -> Approach")
        session.run_duration(
            "activate",
            "neutral",
            None,
            neutral_output,
            0.25,
        )
        session.run_duration(
            "initial_settle",
            "neutral",
            None,
            neutral_output,
            args.initial_settle_seconds,
        )

        for index, target_ra_ft in enumerate(args.altitudes):
            wait_rows = session.wait_for_altitude(
                target_ra_ft,
                neutral_output,
                args.max_wait_seconds,
            )
            if not wait_rows:
                raise SafetyAbort(f"no telemetry while waiting for {target_ra_ft:g} ft")
            baseline = wait_rows[-1]
            actual_ra = float(baseline["ra_ft"])
            if actual_ra < target_ra_ft - args.max_target_overshoot_ft:
                raise SafetyAbort(
                    f"overshot {target_ra_ft:g} ft target by "
                    f"{target_ra_ft - actual_ra:.1f} ft"
                )

            pitch_output = make_output(
                active_mode,
                args.elevator_step,
                0.0,
                throttle_hold,
            )
            pitch_rows = session.run_duration(
                f"pitch_{target_ra_ft:g}ft",
                "pitch",
                target_ra_ft,
                pitch_output,
                args.pulse_seconds,
            )
            pitch_recovery_rows = session.run_duration(
                f"pitch_recovery_{target_ra_ft:g}ft",
                "neutral",
                target_ra_ft,
                neutral_output,
                args.recovery_seconds,
            )
            print_pitch_summary(
                target_ra_ft,
                baseline,
                pitch_rows,
                pitch_recovery_rows,
            )

            roll_baseline = (pitch_recovery_rows or pitch_rows or [baseline])[-1]
            roll_sign = 1.0 if index % 2 == 0 else -1.0
            roll_output = make_output(
                active_mode,
                0.0,
                roll_sign * args.aileron_step,
                throttle_hold,
            )
            roll_rows = session.run_duration(
                f"roll_{target_ra_ft:g}ft",
                "roll",
                target_ra_ft,
                roll_output,
                args.pulse_seconds,
            )
            roll_recovery_rows = session.run_duration(
                f"roll_recovery_{target_ra_ft:g}ft",
                "neutral",
                target_ra_ft,
                neutral_output,
                args.recovery_seconds,
            )
            print_roll_summary(
                target_ra_ft,
                roll_baseline,
                roll_rows,
                roll_recovery_rows,
            )

            if (
                args.landing_after_ft is not None
                and active_mode == ControlModeState.Approach
                and math.isclose(
                    target_ra_ft,
                    args.landing_after_ft,
                    abs_tol=1e-6,
                )
            ):
                active_mode = ControlModeState.Landing
                neutral_output = make_output(active_mode, 0.0, 0.0, throttle_hold)
                switch_rows = session.run_duration(
                    f"switch_to_landing_after_{target_ra_ft:g}ft",
                    "neutral",
                    target_ra_ft,
                    neutral_output,
                    0.25,
                )
                if not switch_rows:
                    raise SafetyAbort("no telemetry during Approach -> Landing switch")
                switch_ra = float(switch_rows[-1]["ra_ft"])
                print(f"switching once at RA={switch_ra:.1f}ft: Approach -> Landing")

        if args.landing_after_ft is None:
            print("all altitude targets completed in one continuous Approach session")
        else:
            print(
                "all altitude targets completed with one Approach -> Landing transition"
            )
    except SafetyAbort as exc:
        print(f"SAFETY ABORT: {exc}")
        exit_code = 4
    except KeyboardInterrupt:
        print("stopped by user")
        exit_code = 130
    finally:
        if sender is not None and commands_started:
            try:
                send_off(sock, sender, args.rate_hz)
            except OSError:
                pass
        sock.close()
        if rows:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            with log_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=LOG_COLUMNS)
                writer.writeheader()
                writer.writerows(rows)
            print(f"log={log_path.resolve()}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
