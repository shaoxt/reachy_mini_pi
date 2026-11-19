"""Edit this file to assign a bus id to each joint.

An HL-2915 leaves the factory as id 1. Change `id` here, plug in that one
servo, then write the id onto it:

    reachy-mini-pi-setup-motor --port /dev/ttyUSB0 --motor stewart_1

Ids must be unique and in 0..253. Keep the nine joints in this order.
`sign` is +1 or -1 (set -1 if that joint runs backwards). `offset` is the
homing offset in counts. `min_count` and `max_count` are the travel limits.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MotorConfig:
    """Settings written to one servo and used by the control loop."""

    id: int
    sign: int = 1
    offset: int = 0
    min_count: int = 0
    max_count: int = 4095


# Body yaw, the six Stewart legs, then right antenna and left antenna.
MOTORS: dict[str, MotorConfig] = {
    "body_rotation": MotorConfig(id=10, offset=0, min_count=0, max_count=4095),
    "stewart_1": MotorConfig(id=11, offset=1024, min_count=1502, max_count=2958),
    "stewart_2": MotorConfig(id=12, offset=-1024, min_count=1138, max_count=2844),
    "stewart_3": MotorConfig(id=13, offset=1024, min_count=1502, max_count=2958),
    "stewart_4": MotorConfig(id=14, offset=-1024, min_count=1138, max_count=2594),
    "stewart_5": MotorConfig(id=15, offset=1024, min_count=1252, max_count=2958),
    "stewart_6": MotorConfig(id=16, offset=-1024, min_count=1138, max_count=2594),
    "right_antenna": MotorConfig(id=17, offset=0, min_count=0, max_count=4095),
    "left_antenna": MotorConfig(id=18, offset=0, min_count=0, max_count=4095),
}

JOINT_ORDER: tuple[str, ...] = (
    "body_rotation",
    "stewart_1",
    "stewart_2",
    "stewart_3",
    "stewart_4",
    "stewart_5",
    "stewart_6",
    "right_antenna",
    "left_antenna",
)


def validate_motors(motors: dict[str, MotorConfig]) -> None:
    """Raise ValueError when ids, signs, or joint order cannot be used."""
    names = list(motors)
    if names != list(JOINT_ORDER):
        raise ValueError(
            "MOTORS must list exactly these joints, in this order: "
            + ", ".join(JOINT_ORDER)
        )
    seen: dict[int, str] = {}
    for name, cfg in motors.items():
        if not isinstance(cfg.id, int) or isinstance(cfg.id, bool):
            raise ValueError(f"{name} id must be an integer, got {cfg.id!r}")
        if cfg.id < 0 or cfg.id > 253:
            raise ValueError(f"{name} id {cfg.id} is outside 0..253")
        if cfg.id in seen:
            raise ValueError(
                f"duplicate id {cfg.id} on {name} and {seen[cfg.id]}"
            )
        seen[cfg.id] = name
        if cfg.sign not in (1, -1):
            raise ValueError(f"{name} sign must be +1 or -1, got {cfg.sign}")
        if cfg.min_count >= cfg.max_count:
            raise ValueError(
                f"{name} min_count {cfg.min_count} must be below max_count {cfg.max_count}"
            )


validate_motors(MOTORS)
