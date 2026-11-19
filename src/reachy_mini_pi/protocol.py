"""Feetech HLS packet codec.

HL-2915 servos speak the FT-SCS / HLS protocol: half-duplex UART, 8N1,
little-endian multi-byte values, and a checksum that is the inverted sum
of every byte after the 0xFF 0xFF header.
"""

from __future__ import annotations

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_REG_WRITE = 0x04
INST_ACTION = 0x05
INST_CALIBRATE = 0x0B
INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83

BROADCAST_ID = 0xFE

# HLS memory map. Addresses match the official HLSCL SDK header.
ADDR_ID = 5
ADDR_BAUD_RATE = 6
ADDR_MIN_ANGLE = 9
ADDR_MAX_ANGLE = 11
ADDR_UNLOAD = 19
ADDR_OFFSET = 31
ADDR_OPERATING_MODE = 33
ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_GOAL_CURRENT = 44
ADDR_GOAL_SPEED = 46
ADDR_TORQUE_LIMIT = 48
ADDR_KP = 50
ADDR_KD = 51
ADDR_KI = 52
ADDR_LOCK = 55
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED = 58
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_STATUS = 65
ADDR_MOVING = 66
ADDR_PRESENT_CURRENT = 69

# HLS operating modes (address 33). These are not Dynamixel mode numbers.
HLS_MODE_POSITION = 0
HLS_MODE_SPEED = 1
HLS_MODE_CURRENT = 2
HLS_MODE_PWM = 3

# One turn is 4096 counts. 2048 is the calibrated center, same resolution
# the Reachy Mini kinematics already assume for the XL330.
COUNTS_PER_REV = 4096
CENTER_COUNTS = 2048

# Address 44 counts in 6.5 mA steps. BIT15 is the sign.
CURRENT_MA_PER_COUNT = 6.5

BAUD_CODES = {
    1_000_000: 0,
    500_000: 1,
    250_000: 2,
    128_000: 3,
    115_200: 4,
    76_800: 5,
    57_600: 6,
    38_400: 7,
}


class ProtocolError(RuntimeError):
    """A frame was malformed or its checksum did not match."""


def checksum(body: bytes) -> int:
    """Inverted sum of the bytes that follow the header."""
    return (~sum(body)) & 0xFF


def build_packet(servo_id: int, instruction: int, params: bytes = b"") -> bytes:
    """Build one instruction frame.

    Length counts the instruction, the parameters, and the checksum.
    """
    length = len(params) + 2
    body = bytes((servo_id & 0xFF, length, instruction & 0xFF)) + params
    return b"\xff\xff" + body + bytes((checksum(body),))


def ping_packet(servo_id: int) -> bytes:
    return build_packet(servo_id, INST_PING)


def read_packet(servo_id: int, address: int, length: int) -> bytes:
    return build_packet(servo_id, INST_READ, bytes((address & 0xFF, length & 0xFF)))


def write_packet(servo_id: int, address: int, data: bytes) -> bytes:
    return build_packet(servo_id, INST_WRITE, bytes((address & 0xFF,)) + data)


def sync_write_packet(address: int, payload: list[tuple[int, bytes]]) -> bytes:
    """Broadcast one register block to several servos.

    Every entry in `payload` must carry the same number of data bytes.
    A sync write to the broadcast id does not produce a status frame.
    """
    if not payload:
        raise ValueError("sync write needs at least one servo")
    width = len(payload[0][1])
    if width == 0 or any(len(data) != width for _, data in payload):
        raise ValueError("sync write data blocks must be the same non-zero length")
    raw = bytearray((address & 0xFF, width))
    for servo_id, data in payload:
        raw.append(servo_id & 0xFF)
        raw.extend(data)
    return build_packet(BROADCAST_ID, INST_SYNC_WRITE, bytes(raw))


def checksum_ok(packet: bytes) -> bool:
    if len(packet) < 6 or packet[0] != 0xFF or packet[1] != 0xFF:
        return False
    return packet[-1] == checksum(packet[2:-1])


def pop_packet(buf: bytearray) -> bytes | None:
    """Remove and return the first valid frame in `buf`.

    Bytes before the header are discarded. A frame whose checksum fails
    is skipped so a single noise byte cannot stall the parser.
    """
    while True:
        start = buf.find(b"\xff\xff")
        if start < 0:
            if buf and buf[-1] == 0xFF:
                del buf[:-1]
            else:
                buf.clear()
            return None
        if start:
            del buf[:start]
        if len(buf) < 4:
            return None
        length = buf[3]
        total = 4 + length
        if length < 2 or total > 128:
            del buf[:2]
            continue
        if len(buf) < total:
            return None
        packet = bytes(buf[:total])
        if not checksum_ok(packet):
            del buf[:2]
            continue
        del buf[:total]
        return packet


def split_packet(packet: bytes) -> tuple[int, int, bytes]:
    """Return (id, instruction_or_error, parameters)."""
    if not checksum_ok(packet):
        raise ProtocolError(f"bad checksum: {packet.hex()}")
    return packet[2], packet[4], packet[5:-1]


def encode_position(counts: int) -> int:
    """Pack a signed count the way HLS goal/present position registers do.

    BIT15 is the direction bit. The lower 15 bits are the magnitude.
    """
    counts = int(counts)
    if counts < 0:
        magnitude = min(0x7FFF, -counts)
        return magnitude | 0x8000
    return min(0x7FFF, counts)


def decode_position(raw: int) -> int:
    raw = int(raw) & 0xFFFF
    if raw & 0x8000:
        return -(raw & 0x7FFF)
    return raw


def encode_current_ma(milliamps: int) -> int:
    """Convert milliamps to the signed goal-current register."""
    counts = int(round(milliamps / CURRENT_MA_PER_COUNT))
    counts = max(-2047, min(2047, counts))
    if counts < 0:
        return (-counts) | 0x8000
    return counts


def decode_current_ma(raw: int) -> int:
    raw = int(raw) & 0xFFFF
    magnitude = raw & 0x7FFF
    sign = -1 if raw & 0x8000 else 1
    return int(round(sign * magnitude * CURRENT_MA_PER_COUNT))


def to_le(value: int, width: int = 2) -> bytes:
    return int(value & ((1 << (8 * width)) - 1)).to_bytes(width, "little")


def from_le(data: bytes) -> int:
    return int.from_bytes(data, "little")
