#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import socket
import time
from pathlib import Path

from ics_protocol import ControlModeState, ICSInputs, ICSOutputs


ELEVATOR_VALID_MASK = 1
THROTTLE_FORWARD_MAX_DEG = 55.7


class SafetyAbort(RuntimeError):
    pass


def parse_step(raw: str) -> float:
    try:
        step = float(raw)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("step must be a number") from exc
    if not math.isfinite(step) or step <= 0.0:
        raise argparse.ArgumentTypeError("step must be a positive finite value")
    return step


def clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


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


def safety_reason(state: ICSInputs, min_ra_ft: float) -> str | None:
    if state.AgentIsActive != 1:
        return "ICS became inactive"
    if state.RadioAltitudeValid and state.RadioAltitude < min_ra_ft:
        return f"radio altitude fell below {min_ra_ft:g} ft"
    if state.PitchAngleValid and abs(state.PitchAngle) > 15.0:
        return f"absolute pitch exceeded 15 deg ({state.PitchAngle:.2f})"
    if state.RollAngleValid and abs(state.RollAngle) > 15.0:
        return f"absolute roll exceeded 15 deg ({state.RollAngle:.2f})"
    if state.IndicatedAirspeedValid and not 120.0 <= state.IndicatedAirspeed <= 190.0:
        return f"IAS left 120..190 kt ({state.IndicatedAirspeed:.1f})"
    if state.VerticalSpeedValid and state.VerticalSpeed < -2000.0:
        return f"vertical speed fell below -2000 fpm ({state.VerticalSpeed:.0f})"
    return None


def exchange(
    sock: socket.socket,
    sender: tuple[str, int],
    output: ICSOutputs,
    duration_s: float,
    rate_hz: float,
    label: str,
    command_g: float,
    min_ra_ft: float,
    start_time: float,
) -> list[dict[str, float | str]]:
    period_s = 1.0 / rate_hz
    packet = output.to_json_bytes()
    deadline = time.monotonic() + duration_s
    next_send = time.monotonic()
    rows: list[dict[str, float | str]] = []
    sock.setblocking(False)
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                sock.sendto(packet, sender)
                next_send += period_s

            for _ in range(128):
                try:
                    data, telemetry_sender = sock.recvfrom(65535)
                except BlockingIOError:
                    break
                if telemetry_sender != sender:
                    continue
                state = ICSInputs.from_json_bytes(data)
                reason = safety_reason(state, min_ra_ft)
                if reason is not None:
                    raise SafetyAbort(reason)
                rows.append({
                    "time_s": time.monotonic() - start_time,
                    "segment": label,
                    "elevator_cmd_g": command_g,
                    "ra_ft": state.RadioAltitude,
                    "ias_kt": state.IndicatedAirspeed,
                    "vs_fpm": state.VerticalSpeed,
                    "pitch_deg": state.PitchAngle,
                    "pitch_rate_deg_s": state.BodyPitchRate,
                    "normal_accel_g": state.BodyNormAccel,
                    "elevator_left_deg": state.ElevatorLeftAngle,
                    "elevator_right_deg": state.ElevatorRightAngle,
                    "stabilizer_deg": state.StabilizerAngle,
                })
            time.sleep(min(0.005, max(0.0, next_send - time.monotonic())))
    finally:
        sock.setblocking(True)
    return rows


def make_output(control_mode: ControlModeState, elevator_g: float, throttle_hold: float) -> ICSOutputs:
    return ICSOutputs(
        ControlValidMask=ELEVATOR_VALID_MASK,
        ControlMode=control_mode,
        ElevatorCmd=elevator_g,
        ThrottleLeft=throttle_hold,
        ThrottleRight=throttle_hold,
        ModeAIReady=1,
    )


def print_step_result(
    step: float,
    baseline: dict[str, float | str],
    pulse_rows: list[dict[str, float | str]],
    recovery_rows: list[dict[str, float | str]],
) -> None:
    if not pulse_rows:
        print(f"step={step:+.2f}g no telemetry samples")
        return
    pitch0 = float(baseline["pitch_deg"])
    vs0 = float(baseline["vs_fpm"])
    elevator0 = 0.5 * (
        float(baseline["elevator_left_deg"]) + float(baseline["elevator_right_deg"])
    )
    final = pulse_rows[-1]
    pitch_peak_delta = max(abs(float(row["pitch_deg"]) - pitch0) for row in pulse_rows)
    pitch_rate_min = min(float(row["pitch_rate_deg_s"]) for row in pulse_rows)
    pitch_rate_max = max(float(row["pitch_rate_deg_s"]) for row in pulse_rows)
    normal_accel_min = min(float(row["normal_accel_g"]) for row in pulse_rows)
    normal_accel_max = max(float(row["normal_accel_g"]) for row in pulse_rows)
    final_elevator = 0.5 * (
        float(final["elevator_left_deg"]) + float(final["elevator_right_deg"])
    )
    print(
        f"step={step:+.2f}g pulse_samples={len(pulse_rows)} "
        f"dPitchEnd={float(final['pitch_deg']) - pitch0:+.3f}deg "
        f"dPitchPeak={pitch_peak_delta:.3f}deg "
        f"qRange={pitch_rate_min:+.3f}..{pitch_rate_max:+.3f}deg/s "
        f"dVS={float(final['vs_fpm']) - vs0:+.0f}fpm "
        f"dElev={final_elevator - elevator0:+.3f}deg "
        f"nzRange={normal_accel_min:+.3f}..{normal_accel_max:+.3f}g"
    )
    if recovery_rows:
        recovered = recovery_rows[-1]
        print(
            f"after recovery pitch={float(recovered['pitch_deg']):+.3f}deg "
            f"q={float(recovered['pitch_rate_deg_s']):+.3f}deg/s "
            f"vs={float(recovered['vs_fpm']):+.0f}fpm; "
            "reset the aircraft to the same position before testing another step"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Measure one ICS elevator command from a repeatable airborne initial state."
    )
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--bind-port", type=int, default=3030)
    parser.add_argument("--wait-timeout", type=float, default=10.0)
    parser.add_argument("--step", type=parse_step, default=0.5)
    parser.add_argument("--pulse-seconds", type=float, default=0.25)
    parser.add_argument("--settle-seconds", type=float, default=1.5)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--arm-seconds", type=float, default=2.2)
    parser.add_argument("--min-ra-ft", type=float, default=1000.0)
    parser.add_argument("--log", type=Path)
    args = parser.parse_args()
    if args.arm_seconds < 2.0:
        parser.error("--arm-seconds must be at least 2.0")
    if args.pulse_seconds <= 0.0 or args.settle_seconds <= 0.0 or args.rate_hz <= 0.0:
        parser.error("pulse, settle, and rate values must be positive")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind_ip, args.bind_port))
    sender: tuple[str, int] | None = None
    all_rows: list[dict[str, float | str]] = []
    log_path = args.log or Path("logs") / time.strftime("ics_elevator_sweep_%Y%m%d_%H%M%S.csv")
    start_time = time.monotonic()
    try:
        print(f"waiting on udp://{args.bind_ip}:{args.bind_port}")
        first = receive_first(sock, args.wait_timeout)
        if first is None:
            print("no telemetry received; not sending")
            return 2
        state, sender = first
        reason = safety_reason(state, args.min_ra_ft)
        if reason is not None:
            print(f"precondition failed: {reason}")
            return 3
        throttle_hold = clamp(
            0.5 * (state.LeftThrottleAngle + state.RightThrottleAngle) / THROTTLE_FORWARD_MAX_DEG,
            0.0,
            1.0,
        )
        print(
            f"from={sender} ra={state.RadioAltitude:.1f}ft ias={state.IndicatedAirspeed:.1f}kt "
            f"pitch={state.PitchAngle:.2f}deg throttle_hold={throttle_hold:.3f}"
        )

        armed = make_output(ControlModeState.Off, 0.0, throttle_hold)
        all_rows.extend(exchange(
            sock, sender, armed, args.arm_seconds, args.rate_hz, "arm", 0.0,
            args.min_ra_ft, start_time,
        ))
        neutral = make_output(ControlModeState.Approach, 0.0, throttle_hold)
        all_rows.extend(exchange(
            sock, sender, neutral, 0.25, args.rate_hz, "activate", 0.0,
            args.min_ra_ft, start_time,
        ))

        settle_rows = exchange(
            sock, sender, neutral, args.settle_seconds, args.rate_hz, "settle", 0.0,
            args.min_ra_ft, start_time,
        )
        all_rows.extend(settle_rows)
        if not settle_rows:
            raise SafetyAbort("telemetry stopped during settle")
        pulse = make_output(ControlModeState.Approach, args.step, throttle_hold)
        pulse_rows = exchange(
            sock, sender, pulse, args.pulse_seconds, args.rate_hz, "pulse", args.step,
            args.min_ra_ft, start_time,
        )
        all_rows.extend(pulse_rows)
        recovery_rows = exchange(
            sock, sender, neutral, args.settle_seconds, args.rate_hz, "recovery", 0.0,
            args.min_ra_ft, start_time,
        )
        all_rows.extend(recovery_rows)
        print_step_result(args.step, settle_rows[-1], pulse_rows, recovery_rows)
    except SafetyAbort as exc:
        print(f"SAFETY ABORT: {exc}")
        return 4
    finally:
        if sender is not None:
            off = ICSOutputs(ControlValidMask=0, ControlMode=ControlModeState.Off)
            try:
                packet = off.to_json_bytes()
                for _ in range(15):
                    sock.sendto(packet, sender)
                    time.sleep(1.0 / 30.0)
            except OSError:
                pass
        sock.close()
        if all_rows:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            with log_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(all_rows[0]))
                writer.writeheader()
                writer.writerows(all_rows)
            print(f"log={log_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
