"""Joint names, ids, and the radian conversion the kinematics expect.

The nine joints stay in the same order as `reachy_mini_motor_controller`:

    body_rotation, stewart_1..6, right_antenna, left_antenna

Positions on the wire are encoder counts. Positions in this API are
radians, with 0 at the calibrated center (count 2048).
"""

from __future__ import annotations

import math

from reachy_mini_pi.motors_config import MOTORS
from reachy_mini_pi.protocol import (
    CENTER_COUNTS,
    COUNTS_PER_REV,
    HLS_MODE_CURRENT,
    HLS_MODE_POSITION,
    HLS_MODE_SPEED,
    decode_position,
    encode_position,
)

# Filled from motors_config.py. Edit ids there, then run setup_motor.
MOTOR_IDS: dict[str, int] = {name: cfg.id for name, cfg in MOTORS.items()}
HOMING_OFFSET: dict[str, int] = {name: cfg.offset for name, cfg in MOTORS.items()}
ANGLE_LIMITS: dict[str, tuple[int, int]] = {
    name: (cfg.min_count, cfg.max_count) for name, cfg in MOTORS.items()
}
JOINT_SIGN: dict[str, int] = {name: cfg.sign for name, cfg in MOTORS.items()}

MOTOR_NAMES: list[str] = list(MOTOR_IDS)
ID_TO_NAME: dict[int, str] = {servo_id: name for name, servo_id in MOTOR_IDS.items()}

STEWART_NAMES = MOTOR_NAMES[1:7]
ANTENNA_NAMES = MOTOR_NAMES[7:9]
BODY_NAME = MOTOR_NAMES[0]

# Voltage, overheat, and overcurrent each drop torque when they trip.
UNLOAD_CONDITION = 0x01 | 0x04 | 0x08

FACTORY_ID = 1
FACTORY_BAUDRATE = 1_000_000


def radians_to_counts(radians: float, sign: int = 1) -> int:
    """Convert a joint angle in radians to an HLS position register value."""
    if sign not in (1, -1):
        raise ValueError(f"sign must be +1 or -1, got {sign}")
    delta = int(round(float(radians) * sign * COUNTS_PER_REV / (2.0 * math.pi)))
    return encode_position(CENTER_COUNTS + delta)


def counts_to_radians(raw: int, sign: int = 1) -> float:
    """Convert an HLS position register value to radians."""
    if sign not in (1, -1):
        raise ValueError(f"sign must be +1 or -1, got {sign}")
    delta = decode_position(raw) - CENTER_COUNTS
    return sign * delta * (2.0 * math.pi) / COUNTS_PER_REV


def stewart_mode_to_hls(dynamixel_mode: int) -> int:
    """Map a Dynamixel operating-mode number onto an HLS mode.

    The Reachy Mini daemon still speaks Dynamixel mode numbers for the
    Stewart platform: 0 current, 1 velocity, 3 position, 5 current-limited
    position. HLS position mode is 0, and its constant-current mode is 2.
    """
    if dynamixel_mode in (3, 4, 5):
        return HLS_MODE_POSITION
    if dynamixel_mode == 1:
        return HLS_MODE_SPEED
    if dynamixel_mode == 0:
        return HLS_MODE_CURRENT
    if dynamixel_mode == 16:
        raise ValueError("PWM operating mode is not supported on HL-2915")
    raise ValueError(f"unsupported operating mode {dynamixel_mode}")


def body_mode_to_hls(mode: int) -> int:
    """Map the yaw-motor mode the daemon actually writes.

    `set_body_rotation_operating_mode(0)` is how the daemon selects position
    control on the yaw servo. On HLS, position control is also mode 0, so
    that call is passed through. Any other value uses the Stewart mapping.
    """
    if mode == 0:
        return HLS_MODE_POSITION
    return stewart_mode_to_hls(mode)


def hls_mode_to_reported(hls_mode: int) -> int:
    """Mode number `_infer_control_mode` in the stock daemon understands.

    3 means position (motors enabled). 1 means the current/speed mode the
    daemon treats as gravity compensation.
    """
    if hls_mode == HLS_MODE_POSITION:
        return 3
    if hls_mode in (HLS_MODE_SPEED, HLS_MODE_CURRENT):
        return 1
    return 3
