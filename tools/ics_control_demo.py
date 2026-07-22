#!/usr/bin/env python3
import socket
import time

from ics_protocol import ALL_CONTROL_VALID_MASK, ControlModeState, ICSInputs, ICSOutputs


STEPS = (
    ("aileron left", dict(AileronCmd=-1.0)),
    ("aileron right", dict(AileronCmd=1.0)),
    ("elevator down", dict(ElevatorCmd=-1.0)),
    ("elevator up", dict(ElevatorCmd=1.0)),
    ("rudder left", dict(RudderCmd=-1.0)),
    ("rudder right", dict(RudderCmd=1.0)),
    ("throttles full", dict(ThrottleLeft=1.0, ThrottleRight=1.0)),
    ("airbrake", dict(AirbrakeCmd=1.0)),
    ("wheel/brakes", dict(NoseWheelTillerCmd=1.0, BrakeLeftCmd=1.0, BrakeRightCmd=1.0)),
)


def main() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", 3030))
    sock.settimeout(5.0)

    try:
        data, sender = sock.recvfrom(65535)
    except socket.timeout:
        print("no telemetry")
        return 2

    telemetry = ICSInputs.from_json_bytes(data)
    print(f"from={sender} active={telemetry.AgentIsActive} phase={telemetry.FlightPhase}")
    period = 1.0 / 20.0

    for label, values in STEPS:
        mode = ControlModeState.Taxi if label == "wheel/brakes" else ControlModeState.Landing
        output = ICSOutputs(
            ControlValidMask=ALL_CONTROL_VALID_MASK,
            ControlMode=mode,
            **values,
        )
        print(label, flush=True)
        deadline = time.time() + 1.5
        while time.time() < deadline:
            sock.sendto(output.to_json_bytes(), sender)
            time.sleep(period)

    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
