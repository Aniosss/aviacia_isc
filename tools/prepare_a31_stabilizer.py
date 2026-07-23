#!/usr/bin/env python3
"""Prepare A.3.1 immediately after the required ICS handshake.

ICS has no direct stabilizer-position command. This tool therefore applies
short, bounded elevator pulses and closes the loop around StabilizerAngle.
"""

from __future__ import annotations

import argparse
import math
import socket
import time
import winsound
from dataclasses import dataclass

from ics_protocol import ControlModeState, ICSInputs, ICSOutputs


ELEVATOR_VALID_MASK = 1


@dataclass(frozen=True)
class SafetyLimits:
    min_ra_ft: float = 500.0
    min_ias_kt: float = 110.0
    max_ias_kt: float = 220.0
    max_abs_pitch_deg: float = 12.0
    max_abs_roll_deg: float = 20.0
    min_vertical_speed_fpm: float = -2500.0
    min_normal_accel_g: float = -0.4
    max_normal_accel_g: float = 2.0


def target_reached(current_deg: float, target_deg: float, tolerance_deg: float) -> bool:
    return abs(target_deg - current_deg) <= tolerance_deg


def resolve_target_deg(
    baseline_deg: float,
    offset_deg: float | None,
    absolute_target_deg: float | None,
) -> float:
    if offset_deg is not None:
        return baseline_deg + offset_deg
    if absolute_target_deg is not None:
        return absolute_target_deg
    raise ValueError("either offset or absolute stabilizer target is required")


def target_stable_for(
    current_deg: float,
    target_deg: float,
    tolerance_deg: float,
    stable_since: float | None,
    now: float,
    stable_seconds: float,
) -> tuple[float | None, bool]:
    """Track continuous time inside the target band."""
    if not target_reached(current_deg, target_deg, tolerance_deg):
        return None, False
    if stable_since is None:
        stable_since = now
    return stable_since, now - stable_since >= stable_seconds


def pulse_command(
    current_deg: float,
    target_deg: float,
    tolerance_deg: float,
    magnitude_g: float,
) -> float:
    """Return a bounded pulse whose sign moves the measured angle to target."""
    error_deg = target_deg - current_deg
    if abs(error_deg) <= tolerance_deg:
        return 0.0
    return math.copysign(magnitude_g, error_deg)


def safety_violation(state: ICSInputs, limits: SafetyLimits) -> str | None:
    if state.AgentIsActive != 1:
        return "AgentIsActive is not 1"
    checks = (
        ("RadioAltitude", state.RadioAltitude),
        ("IndicatedAirspeed", state.IndicatedAirspeed),
        ("PitchAngle", state.PitchAngle),
        ("RollAngle", state.RollAngle),
        ("VerticalSpeed", state.VerticalSpeed),
        ("BodyNormAccel", state.BodyNormAccel),
        ("StabilizerAngle", state.StabilizerAngle),
    )
    for name, value in checks:
        if not math.isfinite(value):
            return f"{name} is non-finite"
    if state.RadioAltitude < limits.min_ra_ft:
        return f"radio altitude below {limits.min_ra_ft:g} ft ({state.RadioAltitude:.1f})"
    if not limits.min_ias_kt <= state.IndicatedAirspeed <= limits.max_ias_kt:
        return (
            f"IAS outside {limits.min_ias_kt:g}..{limits.max_ias_kt:g} kt "
            f"({state.IndicatedAirspeed:.1f})"
        )
    if abs(state.PitchAngle) > limits.max_abs_pitch_deg:
        return f"|pitch| above {limits.max_abs_pitch_deg:g} deg ({state.PitchAngle:.2f})"
    if abs(state.RollAngle) > limits.max_abs_roll_deg:
        return f"|roll| above {limits.max_abs_roll_deg:g} deg ({state.RollAngle:.2f})"
    if state.VerticalSpeed < limits.min_vertical_speed_fpm:
        return (
            f"vertical speed below {limits.min_vertical_speed_fpm:g} fpm "
            f"({state.VerticalSpeed:.0f})"
        )
    if not limits.min_normal_accel_g <= state.BodyNormAccel <= limits.max_normal_accel_g:
        return (
            "normal acceleration outside "
            f"{limits.min_normal_accel_g:g}..{limits.max_normal_accel_g:g} g "
            f"({state.BodyNormAccel:.2f})"
        )
    return None


def receive_state(
    sock: socket.socket,
    timeout_s: float,
) -> tuple[ICSInputs, tuple[str, int]] | None:
    sock.settimeout(timeout_s)
    try:
        data, sender = sock.recvfrom(65535)
    except socket.timeout:
        return None
    return ICSInputs.from_json_bytes(data), sender


def throttle_hold(state: ICSInputs, forward_max_deg: float) -> float:
    average_deg = 0.5 * (state.LeftThrottleAngle + state.RightThrottleAngle)
    return max(0.0, min(1.0, average_deg / forward_max_deg))


def output_packet(
    state: ICSInputs,
    *,
    mode: ControlModeState,
    elevator_g: float,
    elevator_valid: bool,
    forward_max_deg: float,
) -> ICSOutputs:
    hold = throttle_hold(state, forward_max_deg)
    return ICSOutputs(
        ControlValidMask=ELEVATOR_VALID_MASK if elevator_valid else 0,
        ControlMode=mode,
        ElevatorCmd=elevator_g,
        # Protect simulator builds that ignore ControlValidMask from reading
        # default zero as an idle-throttle command.
        ThrottleLeft=hold,
        ThrottleRight=hold,
        ModeAIReady=1,
    )


def send_for(
    sock: socket.socket,
    sender: tuple[str, int],
    packet: ICSOutputs,
    duration_s: float,
    rate_hz: float,
) -> None:
    deadline = time.monotonic() + duration_s
    period_s = 1.0 / rate_hz
    payload = packet.to_json_bytes()
    while time.monotonic() < deadline:
        sock.sendto(payload, sender)
        time.sleep(period_s)


def deactivate(
    sock: socket.socket,
    sender: tuple[str, int],
    rate_hz: float,
) -> None:
    off = ICSOutputs(ControlValidMask=0, ControlMode=ControlModeState.Off)
    send_for(sock, sender, off, 0.5, rate_hz)


def beep() -> None:
    try:
        winsound.Beep(1200, 300)
        winsound.Beep(1600, 500)
    except RuntimeError:
        winsound.MessageBeep()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "After the required 2.2-second ICS handshake, read the current "
            "StabilizerAngle, apply the requested relative offset, and wait "
            "for both IOS stabilizer faults."
        )
    )
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--bind-port", type=int, default=3030)
    parser.add_argument("--wait-timeout", type=float, default=60.0)
    parser.add_argument("--delay", type=float, default=2.2)
    target_group = parser.add_mutually_exclusive_group()
    target_group.add_argument(
        "--offset-deg",
        type=float,
        help=(
            "target relative to StabilizerAngle measured after the handshake; "
            "defaults to -1 deg"
        ),
    )
    target_group.add_argument(
        "--target-deg",
        type=float,
        help="legacy absolute target angle",
    )
    parser.add_argument("--tolerance-deg", type=float, default=0.10)
    parser.add_argument(
        "--stable-seconds",
        type=float,
        default=2.0,
        help="continuous time required inside the target band before prompting for faults",
    )
    parser.add_argument("--pulse-g", type=float, default=0.25)
    parser.add_argument("--pulse-seconds", type=float, default=0.25)
    parser.add_argument("--settle-seconds", type=float, default=0.75)
    parser.add_argument("--control-timeout", type=float, default=90.0)
    parser.add_argument("--fault-timeout", type=float, default=30.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--throttle-forward-max-deg", type=float, default=55.7)
    parser.add_argument(
        "--enable-safety",
        action="store_true",
        help=(
            "enable altitude, speed, attitude, sink-rate and load-factor "
            "abort limits; disabled by default"
        ),
    )
    parser.add_argument(
        "--send",
        action="store_true",
        help="actually send ICS commands; without this flag the tool is a dry run",
    )
    args = parser.parse_args()
    if args.offset_deg is None and args.target_deg is None:
        args.offset_deg = -1.0
    if args.delay < 2.2:
        parser.error("--delay must be at least 2.2 s for the Off -> Approach handshake")
    if not 0.01 <= args.tolerance_deg <= 0.5:
        parser.error("--tolerance-deg must be within 0.01..0.5")
    if not 0.01 <= args.pulse_g <= 0.5:
        parser.error("--pulse-g must be within 0.01..0.5 g")
    if not 0.05 <= args.pulse_seconds <= 0.25:
        parser.error("--pulse-seconds must be within 0.05..0.25 s")
    if not 0.1 <= args.settle_seconds <= 2.0:
        parser.error("--settle-seconds must be within 0.1..2.0 s")
    if not 0.5 <= args.stable_seconds <= 10.0:
        parser.error("--stable-seconds must be within 0.5..10 s")
    return args


def main() -> int:
    args = parse_args()
    limits = SafetyLimits()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind_ip, args.bind_port))

    print(
        f"waiting for spawn telemetry on udp://{args.bind_ip}:{args.bind_port} "
        f"for {args.wait_timeout:g}s"
    )
    received = receive_state(sock, args.wait_timeout)
    if received is None:
        print("no telemetry received; nothing sent")
        return 2
    state, sender = received
    print(
        f"spawn from={sender} ra={state.RadioAltitude:.1f}ft "
        f"pitch={state.PitchAngle:+.2f}deg stabilizer={state.StabilizerAngle:+.3f}deg "
        f"faults={state.FaultLeftStab}/{state.FaultRightStab}"
    )
    if state.FaultLeftStab or state.FaultRightStab:
        print("deactivate both stabilizer faults in IOS before running this tool")
        return 3
    if args.enable_safety:
        violation = safety_violation(state, limits)
        if violation is not None:
            print(f"safety check failed: {violation}")
            return 4
    if not args.send:
        dry_target_deg = resolve_target_deg(
            state.StabilizerAngle,
            args.offset_deg,
            args.target_deg,
        )
        print(
            f"DRY RUN: would wait {args.delay:g}s, then move StabilizerAngle "
            f"from the measured baseline to about {dry_target_deg:+.2f}deg "
            f"with <= {args.pulse_g:g}g / "
            f"{args.pulse_seconds:g}s pulses, hold it for "
            f"{args.stable_seconds:g}s, then wait for both IOS faults"
        )
        return 0

    start = time.monotonic()
    next_status = start
    print(f"arming in ControlMode=Off; target action at t={args.delay:g}s after spawn")
    while True:
        elapsed = time.monotonic() - start
        if elapsed >= args.delay:
            break
        latest = receive_state(sock, min(0.1, args.delay - elapsed))
        if latest is not None:
            state, sender = latest
        if args.enable_safety:
            violation = safety_violation(state, limits)
            if violation is not None:
                print(f"aborting before target time: {violation}")
                return 5
        packet = output_packet(
            state,
            mode=ControlModeState.Off,
            elevator_g=0.0,
            elevator_valid=True,
            forward_max_deg=args.throttle_forward_max_deg,
        )
        sock.sendto(packet.to_json_bytes(), sender)
        if time.monotonic() >= next_status:
            print(
                f"t={elapsed:5.1f}/{args.delay:g}s "
                f"stabilizer={state.StabilizerAngle:+.3f}deg"
            )
            next_status = time.monotonic() + 1.0

    approach = output_packet(
        state,
        mode=ControlModeState.Approach,
        elevator_g=0.0,
        elevator_valid=True,
        forward_max_deg=args.throttle_forward_max_deg,
    )
    print("activating ControlMode: Off -> Approach")
    send_for(sock, sender, approach, 0.2, args.rate_hz)

    baseline_deg = state.StabilizerAngle
    target_deg = resolve_target_deg(
        baseline_deg,
        args.offset_deg,
        args.target_deg,
    )
    print(
        f"relative target prepared: baseline={baseline_deg:+.3f}deg "
        f"target={target_deg:+.3f}deg "
        f"delta={target_deg - baseline_deg:+.3f}deg"
    )

    control_deadline = time.monotonic() + args.control_timeout
    last_error = abs(target_deg - state.StabilizerAngle)
    direction_multiplier = 1.0
    worsening_pulses = 0
    stable_since: float | None = None
    while True:
        if args.enable_safety:
            violation = safety_violation(state, limits)
            if violation is not None:
                print(f"safety abort while positioning stabilizer: {violation}")
                deactivate(sock, sender, args.rate_hz)
                return 6
        if time.monotonic() >= control_deadline:
            print(
                f"target timeout: stabilizer={state.StabilizerAngle:+.3f}deg, "
                f"wanted={target_deg:+.3f}deg"
            )
            deactivate(sock, sender, args.rate_hz)
            return 7

        now = time.monotonic()
        stable_since, stable = target_stable_for(
            state.StabilizerAngle,
            target_deg,
            args.tolerance_deg,
            stable_since,
            now,
            args.stable_seconds,
        )
        if stable:
            break
        if stable_since is not None:
            latest = receive_state(sock, 0.1)
            if latest is not None:
                state, sender = latest
            neutral = output_packet(
                state,
                mode=ControlModeState.Approach,
                elevator_g=0.0,
                elevator_valid=False,
                forward_max_deg=args.throttle_forward_max_deg,
            )
            sock.sendto(neutral.to_json_bytes(), sender)
            continue

        command = direction_multiplier * pulse_command(
            state.StabilizerAngle,
            target_deg,
            args.tolerance_deg,
            args.pulse_g,
        )
        pulse = output_packet(
            state,
            mode=ControlModeState.Approach,
            elevator_g=command,
            elevator_valid=True,
            forward_max_deg=args.throttle_forward_max_deg,
        )
        print(
            f"pulse elevator={command:+.3f}g "
            f"stabilizer={state.StabilizerAngle:+.3f}deg "
            f"target={target_deg:+.3f}deg"
        )
        send_for(sock, sender, pulse, args.pulse_seconds, args.rate_hz)

        settle_deadline = time.monotonic() + args.settle_seconds
        while time.monotonic() < settle_deadline:
            latest = receive_state(sock, min(0.1, settle_deadline - time.monotonic()))
            if latest is not None:
                state, sender = latest
            neutral = output_packet(
                state,
                mode=ControlModeState.Approach,
                elevator_g=0.0,
                elevator_valid=False,
                forward_max_deg=args.throttle_forward_max_deg,
            )
            sock.sendto(neutral.to_json_bytes(), sender)

        error = abs(target_deg - state.StabilizerAngle)
        if error > last_error + 0.02:
            worsening_pulses += 1
        else:
            worsening_pulses = 0
        if worsening_pulses >= 2:
            direction_multiplier *= -1.0
            worsening_pulses = 0
            print("measured angle moved away from target; reversing pulse direction")
        last_error = error

    print(
        f"TARGET STABLE: StabilizerAngle={state.StabilizerAngle:+.3f}deg "
        f"for {args.stable_seconds:g}s. "
        "Activate STABILIZER #1 and #2 ACTUATOR FAULT in IOS now."
    )
    beep()

    fault_deadline = time.monotonic() + args.fault_timeout
    next_correction_at = time.monotonic()
    while not (state.FaultLeftStab and state.FaultRightStab):
        if time.monotonic() >= fault_deadline:
            print(
                "fault confirmation timeout: "
                f"faults={state.FaultLeftStab}/{state.FaultRightStab}"
            )
            deactivate(sock, sender, args.rate_hz)
            return 8
        latest = receive_state(sock, 0.1)
        if latest is not None:
            state, sender = latest
        if args.enable_safety:
            violation = safety_violation(state, limits)
            if violation is not None:
                print(f"safety abort while waiting for faults: {violation}")
                deactivate(sock, sender, args.rate_hz)
                return 9
        if state.FaultLeftStab or state.FaultRightStab:
            neutral = output_packet(
                state,
                mode=ControlModeState.Approach,
                elevator_g=0.0,
                elevator_valid=False,
                forward_max_deg=args.throttle_forward_max_deg,
            )
            sock.sendto(neutral.to_json_bytes(), sender)
            continue
        correction = pulse_command(
            state.StabilizerAngle,
            target_deg,
            args.tolerance_deg,
            args.pulse_g,
        )
        if correction != 0.0 and time.monotonic() >= next_correction_at:
            packet = output_packet(
                state,
                mode=ControlModeState.Approach,
                elevator_g=direction_multiplier * correction,
                elevator_valid=True,
                forward_max_deg=args.throttle_forward_max_deg,
            )
            send_for(
                sock,
                sender,
                packet,
                args.pulse_seconds,
                args.rate_hz,
            )
            next_correction_at = time.monotonic() + args.settle_seconds
        else:
            neutral = output_packet(
                state,
                mode=ControlModeState.Approach,
                elevator_g=0.0,
                elevator_valid=False,
                forward_max_deg=args.throttle_forward_max_deg,
            )
            sock.sendto(neutral.to_json_bytes(), sender)

    deactivate(sock, sender, args.rate_hz)
    if not target_reached(
        state.StabilizerAngle,
        target_deg,
        args.tolerance_deg,
    ):
        print(
            f"A.3.1 NOT PREPARED: faults activated at "
            f"stabilizer={state.StabilizerAngle:+.3f}deg, "
            f"wanted={target_deg:+.3f}±{args.tolerance_deg:.3f}deg. "
            "Reset the spawn and both faults, then repeat."
        )
        return 10
    print(
        f"A.3.1 PREPARED: stabilizer={state.StabilizerAngle:+.3f}deg "
        f"faults={state.FaultLeftStab}/{state.FaultRightStab}. "
        "Start run_ics_pid.py in a new command."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
