#!/usr/bin/env python3
import argparse
import socket
import time

from ics_protocol import ALL_CONTROL_VALID_MASK, ControlModeState, ICSInputs, ICSOutputs


def mode_from_name(name: str) -> ControlModeState:
    try:
        return ControlModeState[name]
    except KeyError as exc:
        choices = ", ".join(mode.name for mode in ControlModeState)
        raise argparse.ArgumentTypeError(f"Unknown mode {name!r}; choose one of: {choices}") from exc


def summarize(inputs: ICSInputs) -> str:
    return (
        f"active={inputs.AgentIsActive} phase={inputs.FlightPhase} "
        f"ra={inputs.RadioAltitude:.1f} ias={inputs.IndicatedAirspeed:.1f} "
        f"pitch={inputs.PitchAngle:.3f} q={inputs.BodyPitchRate:.3f} "
        f"nz={inputs.BodyNormAccel:.3f} roll={inputs.RollAngle:.3f} "
        f"hdg={inputs.MagneticHeading:.3f} "
        f"elevL={inputs.ElevatorLeftAngle:.3f} elevR={inputs.ElevatorRightAngle:.3f} "
        f"ailL={inputs.AileronLeftAngle:.3f} ailR={inputs.AileronRightAngle:.3f} "
        f"rudder={inputs.RudderAngle:.3f}"
    )


def receive_inputs(sock: socket.socket, timeout: float) -> tuple[ICSInputs, tuple[str, int]] | None:
    sock.settimeout(timeout)
    try:
        data, sender = sock.recvfrom(65535)
    except socket.timeout:
        return None
    return ICSInputs.from_json_bytes(data), sender


def drain_latest_inputs(sock: socket.socket, timeout: float) -> ICSInputs | None:
    deadline = time.time() + timeout
    latest: ICSInputs | None = None
    while time.time() < deadline:
        received = receive_inputs(sock, max(0.01, deadline - time.time()))
        if received is None:
            break
        latest, _ = received
    return latest


def main() -> int:
    parser = argparse.ArgumentParser(description="Activate airborne ICS and send a short control pulse.")
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--bind-port", type=int, default=3030)
    parser.add_argument("--wait-timeout", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--mode", type=mode_from_name, default=ControlModeState.Approach)
    parser.add_argument("--mask", type=int, default=ALL_CONTROL_VALID_MASK)
    parser.add_argument("--aileron", type=float, default=0.05)
    parser.add_argument("--rudder", type=float, default=0.0)
    parser.add_argument("--elevator", type=float, default=0.0)
    parser.add_argument("--allow-inactive", action="store_true", help="Send even when AgentIsActive is 0.")
    parser.add_argument("--arm-seconds", type=float, default=2.2, help="Hold AI-ready/Off before switching to the requested mode.")
    parser.add_argument("--settle-seconds", type=float, default=0.5, help="Send neutral commands after the pulse.")
    args = parser.parse_args()
    if args.arm_seconds < 2.0:
        parser.error("--arm-seconds must be at least 2.0 for ICS activation")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind_ip, args.bind_port))

    print(f"waiting for ICS telemetry on udp://{args.bind_ip}:{args.bind_port} for {args.wait_timeout:g}s")
    first = receive_inputs(sock, args.wait_timeout)
    if first is None:
        print("no telemetry received; not sending")
        return 2

    inputs, sender = first
    print(f"telemetry from={sender} {summarize(inputs)}")
    if inputs.AgentIsActive != 1 and not args.allow_inactive:
        print("AgentIsActive is not 1; not sending control. Press ICS in IOS, then rerun.")
        return 3

    output = ICSOutputs(
        ControlValidMask=args.mask,
        ControlMode=args.mode,
        ElevatorCmd=args.elevator,
        AileronCmd=args.aileron,
        RudderCmd=args.rudder,
        ModeAIReady=1,
    )
    neutral = ICSOutputs(ControlValidMask=0, ControlMode=ControlModeState.Off)
    armed_off = ICSOutputs(
        ControlValidMask=args.mask,
        ControlMode=ControlModeState.Off,
        ModeAIReady=1,
    )

    period = 1.0 / args.rate_hz
    arm_deadline = time.time() + args.arm_seconds
    print(f"arming ModeAIReady=1 / ControlMode=Off for {args.arm_seconds:g}s")
    while time.time() < arm_deadline:
        sock.sendto(armed_off.to_json_bytes(), sender)
        time.sleep(period)

    # Airborne ICS only engages on the rising ControlMode transition Off -> Approach.
    approach = ICSOutputs(
        ControlValidMask=args.mask,
        ControlMode=ControlModeState.Approach,
        ModeAIReady=1,
    )
    transition_deadline = time.time() + 0.2
    print("activating ControlMode: Off -> Approach")
    while time.time() < transition_deadline:
        sock.sendto(approach.to_json_bytes(), sender)
        time.sleep(period)

    deadline = time.time() + args.duration
    sent = 0
    print(
        f"sending pulse to={sender} duration={args.duration:g}s rate={args.rate_hz:g}Hz "
        f"mode={args.mode.name} mask={args.mask} aileron={args.aileron} rudder={args.rudder} elevator={args.elevator}"
    )
    while time.time() < deadline:
        sock.sendto(output.to_json_bytes(), sender)
        sent += 1
        time.sleep(period)

    settle_deadline = time.time() + args.settle_seconds
    while time.time() < settle_deadline:
        sock.sendto(neutral.to_json_bytes(), sender)
        sent += 1
        time.sleep(period)

    after = drain_latest_inputs(sock, 1.0)
    if after is not None:
        inputs = after
        print(f"after {summarize(inputs)}")
    print(f"done sent={sent}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
