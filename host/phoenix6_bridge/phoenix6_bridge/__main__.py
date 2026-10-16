# Copyright (c) 2026 CANBusRP2040 contributors
# SPDX-License-Identifier: Apache-2.0
"""Command line entry point: python -m phoenix6_bridge --port /dev/ttyACM1"""

from __future__ import annotations

import argparse
import logging
import signal
import sys


def main() -> int:
    parser = argparse.ArgumentParser(
        prog="phoenix6_bridge",
        description="Run Phoenix 6 requests from a microcontroller on Talon FX devices.",
    )
    parser.add_argument("--port", required=True, help="serial port of the client, e.g. /dev/ttyACM1")
    parser.add_argument(
        "--canbus",
        default="",
        help='CAN bus for Phoenix 6: a SocketCAN interface such as "can0", a CANivore name, '
        'or "" for the default bus',
    )
    parser.add_argument("--baudrate", type=int, default=115200, help="ignored by USB CDC ACM ports")
    parser.add_argument("--link-timeout", type=float, default=0.5, help="seconds (default 0.5)")
    parser.add_argument(
        "--skip-version-check",
        action="store_true",
        help="do not compare schema.json with the installed phoenix6 package",
    )
    parser.add_argument("-v", "--verbose", action="count", default=0)
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose > 1 else logging.INFO if args.verbose else logging.WARNING,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    from .bridge import Bridge, SerialTransport, check_installed
    from .schema import load_schema, schema_hash

    schema = load_schema()
    if not args.skip_version_check:
        try:
            check_installed(schema)
        except RuntimeError as e:
            print(e, file=sys.stderr)
            return 1

    try:
        transport = SerialTransport(args.port, args.baudrate)
    except OSError as e:
        print(e, file=sys.stderr)
        return 1
    bridge = Bridge(transport, canbus=args.canbus, schema=schema, link_timeout=args.link_timeout)

    # Stop the motors on SIGTERM too (e.g. systemd), not only on Ctrl+C
    stopping = False

    def on_sigterm(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, on_sigterm)
    print(
        f"Phoenix 6 {schema['phoenix6_version']} bridge on {args.port}, "
        f"CAN bus {args.canbus or '(default)'}, schema 0x{schema_hash(schema):08x}",
        file=sys.stderr,
    )
    try:
        bridge.run(should_stop=lambda: stopping)
    except KeyboardInterrupt:
        pass
    finally:
        transport.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
