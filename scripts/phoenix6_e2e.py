#!/usr/bin/env python3
# Copyright (c) 2026 CANBusRP2040 contributors
# SPDX-License-Identifier: Apache-2.0
"""
End-to-end test of the Phoenix 6 bridge without hardware.

Runs the native_sim build of tests/e2e/phoenix6_sim (the firmware's Phoenix 6
client) and connects its second pseudo terminal to the host bridge, which
executes the requests on simulated Talon FX devices (CTR_TARGET=Simulation).

  west build -b native_sim/native/64 tests/e2e/phoenix6_sim -d build/e2e
  python3 scripts/phoenix6_e2e.py build/e2e/zephyr/zephyr.exe
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BRIDGE_DIR = ROOT / "host" / "phoenix6_bridge"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("exe", type=Path, help="zephyr.exe of tests/e2e/phoenix6_sim")
    parser.add_argument("--python", default=sys.executable, help="interpreter with phoenix6")
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()

    app = subprocess.Popen(
        [str(args.exe)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1
    )
    output: list[str] = []
    pty_found = threading.Event()
    pty: list[str] = []

    def reader() -> None:
        assert app.stdout is not None
        for line in app.stdout:
            output.append(line)
            print("[firmware]", line, end="", flush=True)
            m = re.search(r"uart_1 connected to pseudotty: (\S+)", line)
            if m:
                pty.append(m.group(1))
                pty_found.set()

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()

    if not pty_found.wait(10):
        app.kill()
        print("the firmware did not open its bridge pseudo terminal", file=sys.stderr)
        return 1

    env = dict(os.environ, CTR_TARGET="Simulation", PYTHONPATH=str(BRIDGE_DIR))
    bridge = subprocess.Popen(
        [args.python, "-m", "phoenix6_bridge", "--port", pty[0], "--canbus", "sim", "-v"],
        cwd=BRIDGE_DIR,
        env=env,
    )

    try:
        app.wait(timeout=args.timeout)
    except subprocess.TimeoutExpired:
        app.kill()
        print("timeout", file=sys.stderr)
    finally:
        bridge.terminate()
        try:
            bridge.wait(timeout=10)
        except subprocess.TimeoutExpired:
            bridge.kill()
    thread.join(timeout=5)

    passed = any("PROJECT EXECUTION SUCCESSFUL" in line for line in output)
    print("E2E", "PASSED" if passed else "FAILED")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
