"""Move one HL-2915 a few degrees and back.

Use this before any ids are assigned. The factory id is 1.

    python examples/wiggle.py --port /dev/ttyUSB0 --degrees 15
"""

from __future__ import annotations

import argparse
import math
import time

from reachy_mini_pi.controller import ReachyMiniMotorController
from reachy_mini_pi.motors import counts_to_radians, radians_to_counts
from reachy_mini_pi.protocol import (
    ADDR_GOAL_POSITION,
    ADDR_PRESENT_POSITION,
    ADDR_TORQUE_ENABLE,
    from_le,
    to_le,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--id", type=int, default=1)
    parser.add_argument("--degrees", type=float, default=15.0)
    parser.add_argument("--baud", type=int, default=1_000_000)
    args = parser.parse_args()

    controller = ReachyMiniMotorController(args.port, baudrate=args.baud)
    try:
        if not controller.ping(args.id):
            raise SystemExit(f"Servo id {args.id} did not answer on {args.port}.")
        raw = from_le(controller.read_bytes(args.id, ADDR_PRESENT_POSITION, 2))
        start = counts_to_radians(raw)
        goal = start + math.radians(args.degrees)
        print(f"Present {start:.3f} rad. Moving {args.degrees:.1f} degrees and back.")
        controller.write_bytes(args.id, ADDR_TORQUE_ENABLE, b"\x01")
        controller.write_bytes(args.id, ADDR_GOAL_POSITION, to_le(radians_to_counts(goal)))
        time.sleep(1.0)
        controller.write_bytes(args.id, ADDR_GOAL_POSITION, to_le(radians_to_counts(start)))
        time.sleep(1.0)
        controller.write_bytes(args.id, ADDR_TORQUE_ENABLE, b"\x00")
        print("Back at the start, torque off.")
    finally:
        controller.close()


if __name__ == "__main__":
    main()
