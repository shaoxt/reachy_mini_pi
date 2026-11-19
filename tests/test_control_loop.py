"""Control loop against an in-memory bus. No serial port and no hardware."""

import time
from datetime import timedelta

import pytest

from reachy_mini_pi.control_loop import ReachyMiniPyControlLoop
from reachy_mini_pi.controller import ServoError
from reachy_mini_pi.motors import MOTOR_IDS
from reachy_mini_pi.protocol import ADDR_KI, ADDR_KP
from tests.memory_bus import MemoryServoBus


def _loop(bus: MemoryServoBus) -> ReachyMiniPyControlLoop:
    return ReachyMiniPyControlLoop(
        "",
        timedelta(milliseconds=10),
        allowed_retries=2,
        bus=bus,
    )


def test_enabled_body_yaw_is_written_and_read_back():
    bus = MemoryServoBus(list(MOTOR_IDS.values()))
    loop = _loop(bus)
    try:
        loop.enable_torque()
        loop.set_body_rotation(0.2)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            if abs(loop.get_last_position().body_yaw - 0.2) < 1e-3:
                break
            time.sleep(0.01)
        assert abs(loop.get_last_position().body_yaw - 0.2) < 1e-3
        assert loop.is_torque_enabled()
        assert loop.get_stewart_platform_operating_mode() == 3
    finally:
        loop.close()


def test_goal_is_not_written_while_torque_is_off():
    bus = MemoryServoBus(list(MOTOR_IDS.values()))
    loop = _loop(bus)
    try:
        loop.set_body_rotation(0.4)
        time.sleep(0.08)
        assert abs(loop.get_last_position().body_yaw) < 1e-6
        assert not loop.is_torque_enabled()
    finally:
        loop.close()


def test_dynamixel_scale_pid_is_not_written():
    bus = MemoryServoBus(list(MOTOR_IDS.values()))
    loop = _loop(bus)
    try:
        loop.async_write_pid_gains(10, 300, 0, 0)
        assert bus.mem[10][ADDR_KP] == 0
        loop.async_write_pid_gains(10, 12, 3, 4)
        assert bus.mem[10][ADDR_KP] == 12
        assert bus.mem[10][ADDR_KP + 1] == 4  # Kd sits next to Kp
        assert bus.mem[10][ADDR_KI] == 3
    finally:
        loop.close()


def test_missing_servo_names_the_joint():
    ids = [servo_id for servo_id in MOTOR_IDS.values() if servo_id != 11]
    bus = MemoryServoBus(ids)
    with pytest.raises(ServoError, match="stewart_1"):
        _loop(bus)
