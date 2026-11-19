"""Direct, blocking HLS controller.

This is the class to use from a script that moves one servo, scans the
bus, or assigns ids. The daemon uses `ReachyMiniPyControlLoop` instead,
which queues the same operations onto one background thread.
"""

from __future__ import annotations

import logging

from reachy_mini_pi.bus import ServoBus
from reachy_mini_pi.motors import (
    ANTENNA_NAMES,
    BODY_NAME,
    JOINT_SIGN,
    MOTOR_IDS,
    MOTOR_NAMES,
    STEWART_NAMES,
    body_mode_to_hls,
    counts_to_radians,
    hls_mode_to_reported,
    radians_to_counts,
    stewart_mode_to_hls,
)
from reachy_mini_pi.protocol import (
    ADDR_GOAL_CURRENT,
    ADDR_GOAL_POSITION,
    ADDR_KI,
    ADDR_KP,
    ADDR_OPERATING_MODE,
    ADDR_PRESENT_CURRENT,
    ADDR_PRESENT_POSITION,
    ADDR_TORQUE_ENABLE,
    HLS_MODE_POSITION,
    ProtocolError,
    build_packet,
    encode_current_ma,
    from_le,
    ping_packet,
    read_packet,
    split_packet,
    sync_write_packet,
    to_le,
    write_packet,
)

logger = logging.getLogger(__name__)

# Dynamixel position-PID values in the stock hardware yaml are 200-300.
# HLS Kp/Kd/Ki registers only accept 0-254, on a different scale.
_HLS_PID_MAX = 254


class ServoError(RuntimeError):
    """The bus or a servo did not do what we asked."""


class FullBodyPosition:
    """Last (or commanded) pose, in radians.

    `stewart` has the six platform joints and `antennas` has right then left.
    The stock daemon reads these attribute names.
    """

    def __init__(self, body_yaw: float, stewart: list[float], antennas: list[float]):
        if len(stewart) != 6:
            raise ValueError(f"stewart needs 6 joints, got {len(stewart)}")
        if len(antennas) != 2:
            raise ValueError(f"antennas needs 2 joints, got {len(antennas)}")
        self.body_yaw = float(body_yaw)
        self.stewart = [float(value) for value in stewart]
        self.antennas = [float(value) for value in antennas]

    def as_list(self) -> list[float]:
        return [self.body_yaw, *self.stewart, *self.antennas]


class ReachyMiniMotorController:
    """Talk to all nine HL-2915 servos through one Waveshare adapter."""

    def __init__(
        self,
        serialport: str,
        baudrate: int = 1_000_000,
        timeout: float = 0.02,
        bus: ServoBus | None = None,
        signs: dict[str, int] | None = None,
    ):
        self.baudrate = baudrate
        self._bus = bus if bus is not None else ServoBus(serialport, baudrate, timeout)
        self._owns_bus = bus is None
        self._signs = {name: JOINT_SIGN.get(name, 1) for name in MOTOR_NAMES}
        if signs:
            for name, sign in signs.items():
                if name not in MOTOR_IDS:
                    raise ValueError(f"unknown motor {name}")
                if sign not in (1, -1):
                    raise ValueError(f"sign for {name} must be +1 or -1")
                self._signs[name] = sign
        self._stewart_mode = 3

    def close(self) -> None:
        if self._owns_bus:
            self._bus.close()

    def get_motor_name_id(self) -> dict[str, int]:
        return dict(MOTOR_IDS)

    def ping(self, servo_id: int) -> bool:
        try:
            reply = self._bus.transact(ping_packet(servo_id), expect_reply=True)
        except (TimeoutError, ProtocolError, OSError):
            return False
        if reply is None or len(reply) < 6:
            return False
        replied_id, error, _params = split_packet(reply)
        # A status frame uses byte 4 as the error flags. 0 means the ping landed.
        return replied_id == (servo_id & 0xFF) and error == 0

    def scan(self, ids: range | None = None) -> list[int]:
        """Return every id that answers. A full scan takes a couple of seconds."""
        found = []
        for servo_id in range(254) if ids is None else ids:
            if self.ping(servo_id):
                found.append(servo_id)
        return found

    def read_bytes(self, servo_id: int, address: int, length: int) -> bytes:
        reply = self._bus.transact(
            read_packet(servo_id, address, length), expect_reply=True
        )
        if reply is None:
            raise ServoError(f"servo {servo_id} did not answer a read")
        replied_id, error, params = split_packet(reply)
        if replied_id != servo_id:
            raise ServoError(
                f"expected a reply from id {servo_id}, got id {replied_id}"
            )
        if len(params) != length:
            raise ServoError(
                f"servo {servo_id} read {len(params)} bytes, expected {length} "
                f"(status 0x{error:02X})"
            )
        if error != 0:
            logger.warning("servo %s status 0x%02X while reading %s", servo_id, error, address)
        return params

    def write_bytes(self, servo_id: int, address: int, data: bytes) -> None:
        """Write a register. The status frame is consumed and checked."""
        try:
            reply = self._bus.transact(
                write_packet(servo_id, address, data), expect_reply=True
            )
        except TimeoutError:
            # Response-level 0 servos stay quiet after a write. The write
            # itself has already gone out.
            return
        if reply is None:
            return
        replied_id, error, _params = split_packet(reply)
        if replied_id not in (servo_id, 0xFE):
            raise ServoError(f"write reply came from id {replied_id}")
        if error not in (0,):
            # Bit flags in the status byte. A zero is the only clean reply.
            # Ignore the case where a firmware echoes the instruction number
            # only when no extra payload is present and the frame is a ping.
            if error != 0:
                logger.debug(
                    "servo %s write to %s returned status 0x%02X",
                    servo_id,
                    address,
                    error,
                )

    def write_raw_packet(self, data: bytes) -> bytes:
        return self._bus.write_raw(data)

    def sync_write(self, address: int, payload: list[tuple[int, bytes]]) -> None:
        if not payload:
            return
        self._bus.transact(sync_write_packet(address, payload), expect_reply=False)

    def set_torque_ids(self, ids: list[int], enabled: bool) -> None:
        value = b"\x01" if enabled else b"\x00"
        self.sync_write(ADDR_TORQUE_ENABLE, [(servo_id, value) for servo_id in ids])

    def enable_torque(self) -> None:
        self.set_torque_ids(list(MOTOR_IDS.values()), True)

    def disable_torque(self) -> None:
        self.set_torque_ids(list(MOTOR_IDS.values()), False)

    def enable_torque_on_ids(self, ids: list[int]) -> None:
        self.set_torque_ids(ids, True)

    def disable_torque_on_ids(self, ids: list[int]) -> None:
        self.set_torque_ids(ids, False)

    def is_torque_enabled(self) -> bool:
        for servo_id in MOTOR_IDS.values():
            if self.read_bytes(servo_id, ADDR_TORQUE_ENABLE, 1) != b"\x01":
                return False
        return True

    def read_position_counts(self, servo_id: int) -> int:
        raw = from_le(self.read_bytes(servo_id, ADDR_PRESENT_POSITION, 2))
        return raw

    def read_joint_radians(self, name: str) -> float:
        return counts_to_radians(
            self.read_position_counts(MOTOR_IDS[name]), self._signs[name]
        )

    def read_all_positions(self) -> list[float]:
        return [self.read_joint_radians(name) for name in MOTOR_NAMES]

    def set_joint_radians(self, name: str, radians: float) -> None:
        raw = radians_to_counts(radians, self._signs[name])
        self.write_bytes(MOTOR_IDS[name], ADDR_GOAL_POSITION, to_le(raw))

    def set_all_goal_positions(self, positions: list[float]) -> None:
        self._sync_positions(list(zip(MOTOR_NAMES, positions, strict=True)))

    def set_stewart_platform_position(self, position: list[float]) -> None:
        if len(position) != 6:
            raise ValueError(f"stewart platform needs 6 positions, got {len(position)}")
        self._sync_positions(list(zip(STEWART_NAMES, position, strict=True)))

    def set_body_rotation(self, position: float) -> None:
        self._sync_positions([(BODY_NAME, position)])

    def set_antennas_positions(self, positions: list[float]) -> None:
        if len(positions) != 2:
            raise ValueError(f"antennas need 2 positions, got {len(positions)}")
        self._sync_positions(list(zip(ANTENNA_NAMES, positions, strict=True)))

    def set_stewart_platform_goal_current(self, current: list[int]) -> None:
        if len(current) != 6:
            raise ValueError(f"stewart platform needs 6 currents, got {len(current)}")
        payload = [
            (MOTOR_IDS[name], to_le(encode_current_ma(int(milliamps))))
            for name, milliamps in zip(STEWART_NAMES, current, strict=True)
        ]
        self.sync_write(ADDR_GOAL_CURRENT, payload)

    def read_stewart_platform_current(self) -> list[int]:
        currents = []
        for name in STEWART_NAMES:
            raw = from_le(self.read_bytes(MOTOR_IDS[name], ADDR_PRESENT_CURRENT, 2))
            from reachy_mini_pi.protocol import decode_current_ma

            currents.append(decode_current_ma(raw))
        return currents

    def set_operating_mode_ids(self, ids: list[int], hls_mode: int) -> None:
        value = bytes((hls_mode & 0xFF,))
        self.sync_write(ADDR_OPERATING_MODE, [(servo_id, value) for servo_id in ids])

    def set_stewart_platform_operating_mode(self, mode: int) -> None:
        hls_mode = stewart_mode_to_hls(int(mode))
        self.set_operating_mode_ids(
            [MOTOR_IDS[name] for name in STEWART_NAMES], hls_mode
        )
        self._stewart_mode = int(mode)

    def set_body_rotation_operating_mode(self, mode: int) -> None:
        self.set_operating_mode_ids([MOTOR_IDS[BODY_NAME]], body_mode_to_hls(int(mode)))

    def set_antennas_operating_mode(self, mode: int) -> None:
        self.set_operating_mode_ids(
            [MOTOR_IDS[name] for name in ANTENNA_NAMES],
            stewart_mode_to_hls(int(mode)),
        )

    def get_stewart_platform_operating_mode(self) -> int:
        return self._stewart_mode

    def read_operating_mode(self, servo_id: int) -> int:
        return self.read_bytes(servo_id, ADDR_OPERATING_MODE, 1)[0]

    def enable_body_rotation(self, enable: bool) -> None:
        self.set_torque_ids([MOTOR_IDS[BODY_NAME]], enable)

    def enable_antennas(self, enable: bool) -> None:
        self.set_torque_ids([MOTOR_IDS[name] for name in ANTENNA_NAMES], enable)

    def enable_stewart_platform(self, enable: bool) -> None:
        self.set_torque_ids([MOTOR_IDS[name] for name in STEWART_NAMES], enable)

    def write_pid_gains(self, servo_id: int, p: int, i: int, d: int) -> None:
        """Write HLS Kp/Ki/Kd when the numbers are already on the HLS scale.

        The stock daemon sends Dynamixel gains such as P=300. Those do not
        fit this servo and are ignored so a factory tune is left alone.
        """
        if min(p, i, d) < 0 or max(p, i, d) > _HLS_PID_MAX:
            logger.warning(
                "Ignoring PID (%s, %s, %s) for servo %s. "
                "HL-2915 gains are 0-254 and are not Dynamixel units.",
                p,
                i,
                d,
                servo_id,
            )
            return
        # Address 50 is Kp, 51 is Kd, 52 is Ki.
        self.write_bytes(servo_id, ADDR_KP, bytes((int(p), int(d), int(i))))

    def read_pid_gains(self, servo_id: int) -> tuple[int, int, int]:
        data = self.read_bytes(servo_id, ADDR_KP, 3)
        kp, kd, ki = data[0], data[1], data[2]
        return kp, ki, kd

    def calibrate_center(self, servo_id: int) -> None:
        """Tell the servo that wherever it is now should become count 2048."""
        packet = build_packet(servo_id, 0x0B)
        try:
            self._bus.transact(packet, expect_reply=True)
        except TimeoutError:
            return

    def _sync_positions(self, joints: list[tuple[str, float]]) -> None:
        payload = []
        for name, radians in joints:
            raw = radians_to_counts(radians, self._signs[name])
            payload.append((MOTOR_IDS[name], to_le(raw)))
        self.sync_write(ADDR_GOAL_POSITION, payload)

    def load_reported_stewart_mode(self) -> int:
        """Read one platform servo and translate its HLS mode for the daemon."""
        hls_mode = self.read_operating_mode(MOTOR_IDS[STEWART_NAMES[0]])
        if hls_mode != HLS_MODE_POSITION and hls_mode not in (1, 2, 3):
            hls_mode = HLS_MODE_POSITION
        self._stewart_mode = hls_mode_to_reported(hls_mode)
        return self._stewart_mode
