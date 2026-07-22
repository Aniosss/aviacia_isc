#!/usr/bin/env python3
import argparse
import json
import socket
import time

from ics_protocol import ICSInputs


def compact_state(message: ICSInputs) -> str:
    fields = [
        ("active", "AgentIsActive"),
        ("phase", "FlightPhase"),
        ("lat", "Latitude"),
        ("lon", "Longitude"),
        ("ra", "RadioAltitude"),
        ("ias", "IndicatedAirspeed"),
        ("gs", "GroundSpeed"),
        ("vs", "VerticalSpeed"),
        ("pitch", "PitchAngle"),
        ("roll", "RollAngle"),
        ("hdg", "MagneticHeading"),
        ("loc", "LocDeviation"),
        ("glide", "GSDeviation"),
    ]
    parts = []
    for label, key in fields:
        value = getattr(message, key)
        if isinstance(value, float):
            value = round(value, 4)
        parts.append(f"{label}={value}")
    return " ".join(parts)


def make_probe_payload(sequence: int) -> bytes:
    payload = {
        "Source": "CodexIcsProbe",
        "Type": "heartbeat",
        "Sequence": sequence,
        "UnixTime": time.time(),
        "Note": "UDP connectivity probe; no control commands",
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Receive Statistics:ICS UDP telemetry and send a harmless heartbeat.")
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--bind-port", type=int, default=3030)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--print-interval", type=float, default=5.0, help="Seconds between console status lines.")
    parser.add_argument("--reply-interval", type=float, default=5.0, help="Seconds between heartbeat replies.")
    parser.add_argument("--reply", action="store_true", help="Send a heartbeat JSON back to the sender.")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind_ip, args.bind_port))
    sock.settimeout(1.0)

    print(f"listening udp://{args.bind_ip}:{args.bind_port} for {args.seconds:g}s")
    deadline = time.time() + args.seconds
    packets = 0
    decode_errors = 0
    sender: tuple[str, int] | None = None
    last_message: ICSInputs | None = None
    last_print = 0.0
    last_reply = 0.0

    while time.time() < deadline:
        now = time.time()
        try:
            data, sender = sock.recvfrom(65535)
        except socket.timeout:
            if last_message is not None and now - last_print >= args.print_interval:
                print(
                    f"#{packets} from={sender} packets={packets} decode_errors={decode_errors} "
                    f"{compact_state(last_message)}"
                )
                last_print = now
            continue

        packets += 1
        try:
            message = ICSInputs.from_json_bytes(data)
        except Exception as exc:
            decode_errors += 1
            if now - last_print >= args.print_interval:
                print(f"#{packets} from={sender} decode_errors={decode_errors} last_error={type(exc).__name__}: {exc}")
                last_print = now
            continue

        last_message = message
        if now - last_print >= args.print_interval:
            print(
                f"#{packets} from={sender} packets={packets} decode_errors={decode_errors} "
                f"{compact_state(message)}"
            )
            last_print = now

        if args.reply and now - last_reply >= args.reply_interval:
            sock.sendto(make_probe_payload(packets), sender)
            last_reply = now

    print(f"done packets={packets} last_sender={sender}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
