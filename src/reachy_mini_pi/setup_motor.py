"""Assign Reachy Mini ids to HL-2915 servos, one servo at a time.

The target id, homing offset, and travel limits come from
`motors_config.py`. The motor ships as id 1 at 1_000_000 baud. This talks
to that one servo and writes the id you set for the named joint. Position
mode on this servo is HLS mode 0.
"""

from __future__ import annotations

import argparse
import time

from reachy_mini_pi.controller import ReachyMiniMotorController
from reachy_mini_pi.motors import (
    ANGLE_LIMITS,
    FACTORY_BAUDRATE,
    FACTORY_ID,
    HOMING_OFFSET,
    MOTOR_IDS,
    MOTOR_NAMES,
    UNLOAD_CONDITION,
)
from reachy_mini_pi.protocol import (
    ADDR_ID,
    ADDR_LOCK,
    ADDR_MAX_ANGLE,
    ADDR_MIN_ANGLE,
    ADDR_OFFSET,
    ADDR_OPERATING_MODE,
    ADDR_TORQUE_ENABLE,
    ADDR_UNLOAD,
    HLS_MODE_POSITION,
    encode_position,
    to_le,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="for example /dev/ttyUSB0")
    parser.add_argument("--motor", required=True, choices=MOTOR_NAMES)
    parser.add_argument(
        "--from-id",
        type=int,
        default=FACTORY_ID,
        help="id the servo answers as right now (factory default 1)",
    )
    parser.add_argument("--baud", type=int, default=FACTORY_BAUDRATE)
    parser.add_argument(
        "--center",
        action="store_true",
        help="call the servo's midpoint calibration instead of writing the stock offset. "
        "Hold the joint at its neutral pose first.",
    )
    args = parser.parse_args()

    target_id = MOTOR_IDS[args.motor]
    controller = ReachyMiniMotorController(args.port, baudrate=args.baud, timeout=0.05)
    try:
        if not controller.ping(args.from_id):
            found = controller.scan()
            raise SystemExit(
                f"No servo answered at id {args.from_id} on {args.port}. "
                f"Servos that did answer: {found or 'none'}. "
                "Plug in only the servo you are assigning, and check 12 V power."
            )
        print(f"Found id {args.from_id}. Configuring it as {args.motor} (id {target_id}).")
        _unlock(controller, args.from_id)
        if args.from_id != target_id:
            controller.write_bytes(args.from_id, ADDR_ID, bytes((target_id,)))
            time.sleep(0.05)
        live_id = target_id if controller.ping(target_id) else args.from_id
        if live_id != target_id:
            raise SystemExit(
                f"The servo did not take id {target_id}. Power-cycle it and run the "
                "command again; if it still answers as "
                f"{args.from_id}, the id write did not stick."
            )
        _unlock(controller, live_id)
        if args.center:
            controller.calibrate_center(live_id)
            print("Calibrated the current shaft angle as count 2048.")
        else:
            offset = HOMING_OFFSET[args.motor]
            controller.write_bytes(live_id, ADDR_OFFSET, to_le(encode_position(offset)))
            print(f"Writing homing offset {offset} counts.")
        lower, upper = ANGLE_LIMITS[args.motor]
        controller.write_bytes(live_id, ADDR_MIN_ANGLE, to_le(lower))
        controller.write_bytes(live_id, ADDR_MAX_ANGLE, to_le(upper))
        controller.write_bytes(live_id, ADDR_OPERATING_MODE, bytes((HLS_MODE_POSITION,)))
        controller.write_bytes(live_id, ADDR_UNLOAD, bytes((UNLOAD_CONDITION,)))
        controller.write_bytes(live_id, ADDR_LOCK, b"\x01")
        controller.write_bytes(live_id, ADDR_TORQUE_ENABLE, b"\x00")
        print(
            f"{args.motor} is id {live_id}, position mode, "
            f"travel {lower}..{upper}, torque off."
        )
        print("Unplug it before configuring the next servo.")
    finally:
        controller.close()


def scan_main() -> None:
    parser = argparse.ArgumentParser(description="List HL-2915 ids on the adapter.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=FACTORY_BAUDRATE)
    args = parser.parse_args()
    controller = ReachyMiniMotorController(args.port, baudrate=args.baud, timeout=0.005)
    try:
        found = controller.scan()
    finally:
        controller.close()
    if not found:
        print("No servos answered. Check 12 V power, the jumper, and TX/RX.")
        return
    from reachy_mini_pi.motors import ID_TO_NAME

    for servo_id in found:
        name = ID_TO_NAME.get(servo_id, "not a Reachy Mini id")
        print(f"id {servo_id}: {name}")


def _unlock(controller: ReachyMiniMotorController, servo_id: int) -> None:
    controller.write_bytes(servo_id, ADDR_TORQUE_ENABLE, b"\x00")
    controller.write_bytes(servo_id, ADDR_LOCK, b"\x00")


if __name__ == "__main__":
    main()
