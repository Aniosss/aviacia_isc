#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import socket
import time
from pathlib import Path

from ics_approach_criteria import ApproachCriteriaConfig, ApproachCriteriaMonitor
from ics_dashboard import DashboardServer, DashboardState
from ics_flight_envelope import LandingFlapConfiguration, measured_landing_flaps
from ics_pid_controller import ClearWeatherILSController, ControllerConfig, ControlResult
from ics_protocol import AIRBORNE_CONTROL_VALID_MASK, ControlModeState, ICSInputs, ICSOutputs


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config" / "ics_clear_weather_pid.json"
TERMINAL_PHASE_RADIO_ALTITUDE_FT = 80.0


def receive(sock: socket.socket, timeout_s: float) -> tuple[ICSInputs, tuple[str, int]]:
    sock.settimeout(timeout_s)
    data, sender = sock.recvfrom(65535)
    return ICSInputs.from_json_bytes(data), sender


def send_for(
    sock: socket.socket,
    sender: tuple[str, int],
    output: ICSOutputs,
    duration_s: float,
    rate_hz: float,
) -> None:
    period = 1.0 / rate_hz
    deadline = time.monotonic() + duration_s
    packet = output.to_json_bytes()
    while time.monotonic() < deadline:
        sock.sendto(packet, sender)
        time.sleep(period)


def deactivate(sock: socket.socket, sender: tuple[str, int], rate_hz: float) -> None:
    output = ICSOutputs(ControlValidMask=0, ControlMode=ControlModeState.Off)
    send_for(sock, sender, output, 0.5, rate_hz)


def make_airborne_output(state: ICSInputs, result: ControlResult) -> ICSOutputs:
    return ICSOutputs(
        ControlValidMask=AIRBORNE_CONTROL_VALID_MASK,
        ControlMode=ControlModeState.Approach,
        ElevatorCmd=result.elevator,
        AileronCmd=result.aileron,
        RudderCmd=result.rudder,
        ThrottleLeftRate=result.throttle_left_rate,
        ThrottleRightRate=result.throttle_right_rate,
        # Absolute throttle positions are not part of the documented airborne
        # command set, but some builds ignore the validity mask. Send the same
        # target on both channels so position and rate control agree.
        ThrottleLeft=result.throttle_left_hold_norm,
        ThrottleRight=result.throttle_right_hold_norm,
        ModeAIReady=1,
        # Keep every airborne mode flag identical to the high-altitude PID
        # approach. Flare and align remain controller-internal phases only;
        # advertising them to ICS changes the simulator's longitudinal law.
        ModeLocCapture=0,
        ModeLocTrack=0,
        ModeGSCapture=0,
        ModeGSTrack=0,
        ModeSpeed=1,
        ModeThrust=1,
        QualityLateralError=abs(result.loc_dots),
        QualityHeadingError=abs(result.heading_error_deg),
        QualitySpeedError=abs(result.target_ias_kt - state.IndicatedAirspeed),
    )


def main_gear_contact(state: ICSInputs) -> bool:
    return bool(state.LeftGearWeightOnWheels or state.RightGearWeightOnWheels)


def main() -> int:
    parser = argparse.ArgumentParser(description="Clear-weather baseline ILS PID for the ICS simulator.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--bind-port", type=int, default=3030)
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--console-rate-hz", type=float, default=0.2)
    parser.add_argument("--send", action="store_true", help="Perform the handshake and send live commands.")
    parser.add_argument("--allow-inactive", action="store_true")
    parser.add_argument("--log", type=Path)
    parser.add_argument("--landing-weight-kg", type=float)
    parser.add_argument(
        "--landing-flaps",
        choices=[item.value for item in LandingFlapConfiguration],
        help="Required landing configuration; overrides landing_flap_fallback from the config.",
    )
    parser.add_argument("--dashboard", action="store_true", help="Open the live PID dashboard server.")
    parser.add_argument(
        "--check-a11-criteria",
        action="store_true",
        help=(
            "Evaluate the A.1.1 course/glideslope limits above the radio-altitude "
            "cutoff and stop commands when the cutoff is reached."
        ),
    )
    parser.add_argument("--criteria-cutoff-ra-ft", type=float, default=300.0)
    parser.add_argument("--criteria-max-course-error-deg", type=float, default=0.7)
    parser.add_argument("--criteria-max-glideslope-error-deg", type=float, default=0.5)
    parser.add_argument("--dashboard-host", default="127.0.0.1")
    parser.add_argument("--dashboard-port", type=int, default=8765)
    parser.add_argument(
        "--dashboard-hold-seconds",
        type=float,
        default=300.0,
        help="Keep the dashboard available after the run ends; use 0 to disable.",
    )
    args = parser.parse_args()
    if (
        args.duration <= 0.0
        or args.rate_hz <= 0.0
        or args.console_rate_hz <= 0.0
        or args.dashboard_hold_seconds < 0.0
        or args.criteria_cutoff_ra_ft <= 0.0
        or args.criteria_max_course_error_deg <= 0.0
        or args.criteria_max_glideslope_error_deg <= 0.0
        or not all(
            math.isfinite(value)
            for value in (
                args.criteria_cutoff_ra_ft,
                args.criteria_max_course_error_deg,
                args.criteria_max_glideslope_error_deg,
            )
        )
    ):
        parser.error("duration, rates, cutoff, and criteria limits must be positive")

    config = ControllerConfig.from_json(args.config)
    if args.landing_flaps is not None:
        config.landing_flap_fallback = args.landing_flaps
    if args.landing_weight_kg is not None:
        if args.landing_weight_kg <= 0.0:
            parser.error("landing weight must be positive")
        config.landing_weight_kg = args.landing_weight_kg
    controller = ClearWeatherILSController(config)
    criteria_monitor = None
    if args.check_a11_criteria:
        criteria_monitor = ApproachCriteriaMonitor(
            ApproachCriteriaConfig(
                cutoff_radio_altitude_ft=args.criteria_cutoff_ra_ft,
                max_course_error_deg=args.criteria_max_course_error_deg,
                max_glideslope_error_deg=args.criteria_max_glideslope_error_deg,
                target_glideslope_deg=config.glideslope_angle_deg,
            )
        )
        print(
            "A.1.1 criteria enabled: "
            f"cutoff={args.criteria_cutoff_ra_ft:g}ft "
            f"course<={args.criteria_max_course_error_deg:g}deg "
            f"glideslope<={args.criteria_max_glideslope_error_deg:g}deg"
        )
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind_ip, args.bind_port))
    print(f"waiting on udp://{args.bind_ip}:{args.bind_port}")
    try:
        state, sender = receive(sock, 10.0)
    except socket.timeout:
        print("no telemetry received")
        return 2

    required_flaps = LandingFlapConfiguration(config.landing_flap_fallback)
    measured_flaps = measured_landing_flaps(state.FlapsAngle)
    print(
        f"from={sender} active={state.AgentIsActive} ra={state.RadioAltitude:.1f} "
        f"loc={state.LocDeviation:.4f} gs={state.GSDeviation:.4f} "
        f"flaps={state.FlapsAngle:.1f}/{measured_flaps.value if measured_flaps else 'NOT_LANDING'}"
    )
    if measured_flaps != required_flaps:
        message = (
            f"landing flaps must be {required_flaps.value}; "
            f"telemetry reports {state.FlapsAngle:.1f} deg"
        )
        if args.send:
            print(message)
            return 8
        print(f"DRY RUN warning: {message}")
    if args.send:
        if state.RadioAltitude <= 400.0:
            print("radio altitude must be above 400 ft for airborne activation")
            return 3
        if state.AgentIsActive != 1 and not args.allow_inactive:
            print("AgentIsActive is not 1; enable ICS in IOS")
            return 4
        ready_off = ICSOutputs(
            ControlValidMask=AIRBORNE_CONTROL_VALID_MASK,
            ControlMode=ControlModeState.Off,
            ModeAIReady=1,
        )
        print("arming ModeAIReady=1 / ControlMode=Off for 2.2s")
        send_for(sock, sender, ready_off, 2.2, args.rate_hz)
        approach = ICSOutputs(
            ControlValidMask=AIRBORNE_CONTROL_VALID_MASK,
            ControlMode=ControlModeState.Approach,
            ModeAIReady=1,
        )
        print("activating ControlMode: Off -> Approach")
        send_for(sock, sender, approach, 0.2, args.rate_hz)
    else:
        print("DRY RUN: commands are calculated but not sent")

    dashboard_state: DashboardState | None = None
    dashboard_server: DashboardServer | None = None
    if args.dashboard:
        dashboard_state = DashboardState(controller)
        dashboard_server = DashboardServer(
            dashboard_state,
            args.dashboard_host,
            args.dashboard_port,
        )
        dashboard_server.start()
        print(f"dashboard=http://{args.dashboard_host}:{dashboard_server.port}")

    log_path = args.log or ROOT / "logs" / f"ics_pid_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    columns = (
        "time_s", "active", "flight_phase", "control_mode", "ra_ft", "ias_kt", "ground_speed_kt", "vs_fpm",
        "latitude_deg", "longitude_deg", "heading_deg", "track_magnetic_deg",
        "roll_deg", "pitch_deg", "body_pitch_rate_deg_s",
        "flight_path_angle_deg", "estimated_aoa_deg", "mach",
        "lateral_deviation", "runway_heading_deg", "loc_ddm", "gs_ddm", "loc_dots", "gs_dots",
        "target_heading_deg", "target_roll_deg",
        "target_vs_fpm", "target_flight_path_angle_deg", "target_pitch_deg",
        "vertical_correction_deg", "configured_aoa_deg", "reference_aoa_deg",
        "flaps_angle_deg", "slats_angle_deg", "flap_configuration", "landing_weight_kg",
        "target_ias_kt", "vapp_kt", "vsr1_kt", "vfe_kt", "alpha_prot_deg", "alpha_sw_deg",
        "alpha_margin_deg", "roll_limit_deg", "flare_armed", "flare_active", "flare_progress",
        "flare_entry_ra_ft", "flare_entry_vs_fpm",
        "body_norm_accel_g", "left_throttle_angle_deg", "right_throttle_angle_deg",
        "stabilizer_angle_deg", "elevator_left_angle_deg", "elevator_right_angle_deg",
        "aileron_left_angle_deg", "aileron_right_angle_deg", "rudder_angle_deg",
        "wind_direction_true_deg", "wind_speed_kt",
        "nose_gear_wow", "left_gear_wow", "right_gear_wow", "main_gear_touchdown",
        "eng_left_thrust", "eng_right_thrust", "envelope_warnings",
        "touchdown_vs_limit_fpm", "touchdown_speed_min_kt", "touchdown_speed_max_kt",
        "touchdown_pitch_limit_deg",
        "aileron_cmd_deg", "elevator_cmd", "rudder_cmd_deg",
        "throttle_left_rate_cmd_deg_s", "throttle_right_rate_cmd_deg_s",
        "throttle_cmd_norm", "throttle_target_angle_deg",
        "throttle_left_position_cmd_norm", "throttle_right_position_cmd_norm",
        "roll_kp", "roll_ki", "roll_kd", "roll_i_term",
        "pitch_kp", "pitch_ki", "pitch_kd", "pitch_i_term",
        "speed_kp", "speed_ki", "speed_kd", "speed_i_term",
        "criteria_course_error_deg", "criteria_glideslope_error_deg", "criteria_status",
    )
    start = time.monotonic()
    previous = start
    next_send = start
    next_print = start
    latest_sender = sender
    exit_code = 0
    terminal_inactive_reported = False
    try:
        with log_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            while time.monotonic() - start < args.duration:
                try:
                    state, latest_sender = receive(sock, 0.5)
                except socket.timeout:
                    print("telemetry timeout; deactivating")
                    exit_code = 5
                    break
                now = time.monotonic()
                touchdown_detected = main_gear_contact(state)
                terminal_phase = bool(
                    touchdown_detected
                    or state.RadioAltitude <= TERMINAL_PHASE_RADIO_ALTITUDE_FT
                )
                if args.send and state.AgentIsActive != 1 and not terminal_phase:
                    print(
                        "ICS became inactive "
                        f"ra={state.RadioAltitude:.1f}ft phase={state.FlightPhase} "
                        f"vs={state.VerticalSpeed:.0f}fpm pitch={state.PitchAngle:.2f}deg "
                        f"wow={state.NoseGearWeightOnWheels}/"
                        f"{state.LeftGearWeightOnWheels}/"
                        f"{state.RightGearWeightOnWheels}; deactivating"
                    )
                    exit_code = 6
                    break
                if (
                    args.send
                    and state.AgentIsActive != 1
                    and terminal_phase
                    and not terminal_inactive_reported
                ):
                    print(
                        "ICS became inactive in terminal phase "
                        f"ra={state.RadioAltitude:.1f}ft phase={state.FlightPhase} "
                        f"vs={state.VerticalSpeed:.0f}fpm pitch={state.PitchAngle:.2f}deg "
                        f"wow={state.NoseGearWeightOnWheels}/"
                        f"{state.LeftGearWeightOnWheels}/"
                        f"{state.RightGearWeightOnWheels}; continuing to touchdown"
                    )
                    terminal_inactive_reported = True
                dt_s = now - previous
                previous = now
                result = controller.update(state, dt_s)
                if dashboard_state is not None:
                    dashboard_state.record(now - start, state, result)
                criteria_sample = None
                criteria_cutoff_reached = False
                if criteria_monitor is not None:
                    criteria_sample = criteria_monitor.observe(
                        state,
                        result.flight_path_angle_deg,
                    )
                    criteria_cutoff_reached = criteria_monitor.cutoff_reached
                if (
                    args.send
                    and not touchdown_detected
                    and not criteria_cutoff_reached
                    and now >= next_send
                ):
                    output = make_airborne_output(state, result)
                    sock.sendto(output.to_json_bytes(), latest_sender)
                    next_send = now + 1.0 / args.rate_hz
                writer.writerow({
                    "time_s": now - start,
                    "active": state.AgentIsActive,
                    "flight_phase": state.FlightPhase,
                    "control_mode": ControlModeState.Approach.name,
                    "ra_ft": state.RadioAltitude,
                    "ias_kt": state.IndicatedAirspeed,
                    "ground_speed_kt": state.GroundSpeed,
                    "vs_fpm": state.VerticalSpeed,
                    "latitude_deg": state.Latitude,
                    "longitude_deg": state.Longitude,
                    "heading_deg": state.MagneticHeading,
                    "track_magnetic_deg": state.TrkAngleMagnetic,
                    "roll_deg": state.RollAngle,
                    "pitch_deg": state.PitchAngle,
                    "body_pitch_rate_deg_s": state.BodyPitchRate,
                    "flight_path_angle_deg": result.flight_path_angle_deg,
                    "estimated_aoa_deg": result.estimated_aoa_deg,
                    "mach": result.mach,
                    "lateral_deviation": state.LateralDeviation,
                    "runway_heading_deg": state.RunwayHeading,
                    "loc_ddm": state.LocDeviation,
                    "gs_ddm": state.GSDeviation,
                    "loc_dots": result.loc_dots,
                    "gs_dots": result.gs_dots,
                    "target_heading_deg": result.target_heading_deg,
                    "target_roll_deg": result.target_roll_deg,
                    "target_vs_fpm": result.target_vs_fpm,
                    "target_flight_path_angle_deg": result.target_flight_path_angle_deg,
                    "target_pitch_deg": result.target_pitch_deg,
                    "vertical_correction_deg": result.vertical_correction_deg,
                    "configured_aoa_deg": controller.config.approach_aoa_deg,
                    "reference_aoa_deg": result.reference_aoa_deg,
                    "flaps_angle_deg": state.FlapsAngle,
                    "slats_angle_deg": state.SlatsAngle,
                    "flap_configuration": result.flap_configuration,
                    "landing_weight_kg": controller.config.landing_weight_kg,
                    "target_ias_kt": result.target_ias_kt,
                    "vapp_kt": result.vapp_kt,
                    "vsr1_kt": result.vsr1_kt,
                    "vfe_kt": result.vfe_kt,
                    "alpha_prot_deg": result.alpha_prot_deg,
                    "alpha_sw_deg": result.alpha_sw_deg,
                    "alpha_margin_deg": result.alpha_margin_deg,
                    "roll_limit_deg": result.roll_limit_deg,
                    "flare_armed": int(result.flare_armed),
                    "flare_active": int(result.flare_active),
                    "flare_progress": result.flare_progress,
                    "flare_entry_ra_ft": result.flare_entry_radio_altitude_ft,
                    "flare_entry_vs_fpm": result.flare_entry_vertical_speed_fpm,
                    "body_norm_accel_g": state.BodyNormAccel,
                    "left_throttle_angle_deg": state.LeftThrottleAngle,
                    "right_throttle_angle_deg": state.RightThrottleAngle,
                    "stabilizer_angle_deg": state.StabilizerAngle,
                    "elevator_left_angle_deg": state.ElevatorLeftAngle,
                    "elevator_right_angle_deg": state.ElevatorRightAngle,
                    "aileron_left_angle_deg": state.AileronLeftAngle,
                    "aileron_right_angle_deg": state.AileronRightAngle,
                    "rudder_angle_deg": state.RudderAngle,
                    "wind_direction_true_deg": state.WindDirectionTrue,
                    "wind_speed_kt": state.WindSpeed,
                    "nose_gear_wow": state.NoseGearWeightOnWheels,
                    "left_gear_wow": state.LeftGearWeightOnWheels,
                    "right_gear_wow": state.RightGearWeightOnWheels,
                    "main_gear_touchdown": int(touchdown_detected),
                    "eng_left_thrust": state.EngLeftThrust,
                    "eng_right_thrust": state.EngRigntThrust,
                    "envelope_warnings": ";".join(result.envelope_warnings),
                    "touchdown_vs_limit_fpm": result.touchdown_vertical_speed_limit_fpm,
                    "touchdown_speed_min_kt": result.touchdown_speed_min_kt,
                    "touchdown_speed_max_kt": result.touchdown_speed_max_kt,
                    "touchdown_pitch_limit_deg": result.touchdown_pitch_limit_deg,
                    "aileron_cmd_deg": result.aileron,
                    "elevator_cmd": result.elevator,
                    "rudder_cmd_deg": result.rudder,
                    "throttle_left_rate_cmd_deg_s": result.throttle_left_rate,
                    "throttle_right_rate_cmd_deg_s": result.throttle_right_rate,
                    "throttle_cmd_norm": result.throttle_norm,
                    "throttle_target_angle_deg": result.throttle_target_angle_deg,
                    "throttle_left_position_cmd_norm": result.throttle_left_hold_norm,
                    "throttle_right_position_cmd_norm": result.throttle_right_hold_norm,
                    "roll_kp": controller.roll_pid.config.kp,
                    "roll_ki": controller.roll_pid.config.ki,
                    "roll_kd": controller.roll_pid.config.kd,
                    "roll_i_term": controller.roll_pid.config.ki * controller.roll_pid.integral,
                    "pitch_kp": controller.pitch_pid.config.kp,
                    "pitch_ki": controller.pitch_pid.config.ki,
                    "pitch_kd": controller.pitch_pid.config.kd,
                    "pitch_i_term": controller.pitch_pid.config.ki * controller.pitch_pid.integral,
                    "speed_kp": controller.speed_pid.config.kp,
                    "speed_ki": controller.speed_pid.config.ki,
                    "speed_kd": controller.speed_pid.config.kd,
                    "speed_i_term": controller.speed_pid.config.ki * controller.speed_pid.integral,
                    "criteria_course_error_deg": (
                        criteria_sample.course_error_deg if criteria_sample else ""
                    ),
                    "criteria_glideslope_error_deg": (
                        criteria_sample.glideslope_error_deg if criteria_sample else ""
                    ),
                    "criteria_status": criteria_sample.status if criteria_sample else "",
                })
                if criteria_cutoff_reached:
                    print(
                        "criteria cutoff reached at "
                        f"ra={state.RadioAltitude:.1f}ft; stopping commands"
                    )
                    break
                if now >= next_print:
                    print(
                        f"t={now-start:5.1f} ra={state.RadioAltitude:7.1f} ias={state.IndicatedAirspeed:6.1f} "
                        f"/{result.target_ias_kt:.1f} {result.flap_configuration} "
                        f"loc={result.loc_dots:+.3f} gs={result.gs_dots:+.3f} "
                        f"roll={state.RollAngle:+.2f}/{result.target_roll_deg:+.2f} "
                        f"pitch={state.PitchAngle:+.2f}/{result.target_pitch_deg:+.2f} "
                        f"mode={ControlModeState.Approach.name} flare={result.flare_progress:.2f} "
                        f"aoa={result.estimated_aoa_deg:.2f}/{result.reference_aoa_deg:.2f} "
                        f"thr={state.LeftThrottleAngle:.1f}/{state.RightThrottleAngle:.1f} "
                        f"T={state.EngLeftThrust:.1f}/{state.EngRigntThrust:.1f} "
                        f"dT={state.EngLeftThrust-state.EngRigntThrust:+.1f} "
                        f"env={','.join(result.envelope_warnings) or 'OK'} "
                        f"cmd=({result.aileron:+.2f}deg,{result.elevator:+.3f},"
                        f"{result.rudder:+.2f}deg,"
                        f"{result.throttle_left_rate:+.2f}/{result.throttle_right_rate:+.2f}deg/s,"
                        f"{result.throttle_norm:.3f})"
                    )
                    next_print = now + 1.0 / args.console_rate_hz
                if touchdown_detected:
                    speed_ok = (
                        result.touchdown_speed_min_kt
                        < state.IndicatedAirspeed
                        < result.touchdown_speed_max_kt
                    )
                    vertical_ok = abs(state.VerticalSpeed) <= result.touchdown_vertical_speed_limit_fpm
                    pitch_ok = 0.0 < state.PitchAngle < result.touchdown_pitch_limit_deg
                    print(
                        "touchdown "
                        f"ias={state.IndicatedAirspeed:.1f}kt speed_ok={int(speed_ok)} "
                        f"vs={state.VerticalSpeed:.0f}fpm vertical_ok={int(vertical_ok)} "
                        f"pitch={state.PitchAngle:.2f}deg pitch_ok={int(pitch_ok)}"
                    )
                    print(
                        "first main-gear contact detected; stopping commands and deactivating"
                    )
                    break
    except KeyboardInterrupt:
        print("stopped by user")
        exit_code = 130
    finally:
        if args.send:
            deactivate(sock, latest_sender, args.rate_hz)
        sock.close()
        if criteria_monitor is not None:
            verdict = criteria_monitor.verdict()
            print(verdict.summary())
            if exit_code == 0 and verdict.status == "FAIL":
                exit_code = 9
            elif exit_code == 0 and verdict.status == "INCOMPLETE":
                exit_code = 10
        print(f"log={log_path}")
        if (
            dashboard_server is not None
            and exit_code != 130
            and args.dashboard_hold_seconds > 0.0
        ):
            print(
                f"dashboard remains available for {args.dashboard_hold_seconds:g}s; "
                "press Ctrl+C to close"
            )
            try:
                time.sleep(args.dashboard_hold_seconds)
            except KeyboardInterrupt:
                pass
        if dashboard_server is not None:
            dashboard_server.stop()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
