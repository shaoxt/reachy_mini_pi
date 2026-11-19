"""Packet codec and joint-unit conversion."""

import math

import pytest

from reachy_mini_pi.motors import (
    MOTOR_IDS,
    body_mode_to_hls,
    counts_to_radians,
    radians_to_counts,
    stewart_mode_to_hls,
)
from reachy_mini_pi.motors_config import MOTORS, MotorConfig, validate_motors
from reachy_mini_pi.protocol import (
    CENTER_COUNTS,
    HLS_MODE_CURRENT,
    HLS_MODE_POSITION,
    HLS_MODE_SPEED,
    build_packet,
    decode_current_ma,
    decode_position,
    encode_current_ma,
    encode_position,
    ping_packet,
    pop_packet,
    sync_write_packet,
)


def test_ping_packet_matches_the_known_frame():
    # id 1, length 2, instruction ping. Checksum is the inverted sum 0xFB.
    assert ping_packet(1) == bytes.fromhex("ff ff 01 02 01 fb")


def test_pop_packet_skips_noise_and_a_bad_checksum():
    good = ping_packet(1)
    bad = b"\xff\xff\x01\x02\x01\x00"
    buf = bytearray(b"\x00\x13" + bad + good)
    assert pop_packet(buf) == good
    assert pop_packet(buf) is None


def test_sync_write_frame_round_trips_through_the_parser():
    packet = sync_write_packet(42, [(10, b"\x00\x08"), (11, b"\x00\x08")])
    assert packet[:2] == b"\xff\xff"
    assert packet[2] == 0xFE
    assert packet[4] == 0x83
    # Rebuilding the checksum from the body must match the last byte.
    assert packet[-1] == (~sum(packet[2:-1])) & 0xFF


def test_radians_round_trip_within_one_count():
    for angle in (-4.0, -1.2, -0.1, 0.0, 0.1, 1.5):
        recovered = counts_to_radians(radians_to_counts(angle))
        assert abs(recovered - angle) < (2 * math.pi / 4096)


def test_negative_direction_uses_bit_15():
    assert encode_position(-100) == (100 | 0x8000)
    assert decode_position(100 | 0x8000) == -100
    assert encode_position(CENTER_COUNTS) == CENTER_COUNTS


def test_reversed_joint_sign():
    forward = radians_to_counts(0.2, sign=1)
    reverse = radians_to_counts(0.2, sign=-1)
    assert forward != reverse
    assert abs(counts_to_radians(reverse, sign=-1) - 0.2) < 1e-3
    assert abs(counts_to_radians(forward, sign=1) - 0.2) < 1e-3


def test_current_conversion_uses_6_5_milliamp_steps():
    assert encode_current_ma(0) == 0
    assert decode_current_ma(encode_current_ma(65)) == 65
    assert decode_current_ma(encode_current_ma(-13)) == -13
    assert encode_current_ma(100_000) == 2047


def test_operating_mode_map():
    assert stewart_mode_to_hls(3) == HLS_MODE_POSITION
    assert stewart_mode_to_hls(5) == HLS_MODE_POSITION
    assert stewart_mode_to_hls(0) == HLS_MODE_CURRENT
    assert stewart_mode_to_hls(1) == HLS_MODE_SPEED
    # The daemon writes 0 on the yaw motor to select position control.
    assert body_mode_to_hls(0) == HLS_MODE_POSITION
    assert body_mode_to_hls(3) == HLS_MODE_POSITION


def test_build_packet_length_counts_instruction_and_checksum():
    packet = build_packet(4, 0x03, b"\x2a\x00\x08")
    assert packet[3] == 2 + 3


def test_motor_config_ids_are_unique_and_reach_the_driver():
    assert list(MOTOR_IDS) == list(MOTORS)
    assert list(MOTOR_IDS.values()) == list(range(10, 19))
    assert len(set(MOTOR_IDS.values())) == 9


def test_duplicate_motor_id_is_rejected():
    bad = dict(MOTORS)
    bad["stewart_2"] = MotorConfig(id=MOTORS["stewart_1"].id)
    with pytest.raises(ValueError, match="duplicate id"):
        validate_motors(bad)
