"""Small sine wave on every joint, same shape as the stock motor-controller demo.

All nine servos must already have Reachy Mini ids (10 through 18).

    python examples/sin.py --port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse
import math
import time

from reachy_mini_pi.controller import ReachyMiniMotorController


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--degrees", type=float, default=10.0)
    args = parser.parse_args()

    amplitude = math.radians(args.degrees)
    controller = ReachyMiniMotorController(args.port, baudrate=args.baud)
    try:
        controller.enable_torque()
        started = time.time()
        while True:
            angle = amplitude * math.sin(2.0 * math.pi * 0.25 * (time.time() - started))
            controller.set_all_goal_positions([angle] * 9)
            print([round(value, 3) for value in controller.read_all_positions()])
            time.sleep(0.02)
    finally:
        controller.disable_torque()
        controller.close()


if __name__ == "__main__":
    main()
