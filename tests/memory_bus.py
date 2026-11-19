"""In-memory HL-2915 bus for tests. Speaks the same frames as a real servo."""

from __future__ import annotations

from reachy_mini_pi.protocol import (
    ADDR_GOAL_POSITION,
    ADDR_PRESENT_POSITION,
    INST_PING,
    INST_READ,
    INST_SYNC_WRITE,
    INST_WRITE,
    build_packet,
    split_packet,
)


class MemoryServoBus:
    """Nine (or fewer) servos whose registers live in bytearrays."""

    def __init__(self, ids: list[int]):
        self.mem = {}
        for servo_id in ids:
            memory = bytearray(90)
            memory[ADDR_PRESENT_POSITION] = 0x00
            memory[ADDR_PRESENT_POSITION + 1] = 0x08  # 2048, the center
            self.mem[servo_id] = memory
        self.writes: list[tuple[int, int, bytes]] = []

    def transact(self, packet: bytes, expect_reply: bool) -> bytes | None:
        servo_id, instruction, params = split_packet(packet)
        if instruction == INST_SYNC_WRITE:
            self._sync_write(params)
            return None
        if servo_id not in self.mem:
            raise TimeoutError(f"no servo {servo_id}")
        if instruction == INST_PING:
            return build_packet(servo_id, 0)
        if instruction == INST_READ:
            address, length = params[0], params[1]
            data = bytes(self.mem[servo_id][address : address + length])
            return build_packet(servo_id, 0, data)
        if instruction == INST_WRITE:
            address = params[0]
            data = bytes(params[1:])
            self._write(servo_id, address, data)
            return build_packet(servo_id, 0)
        if expect_reply:
            return build_packet(servo_id, 0)
        return None

    def write_raw(self, packet: bytes) -> bytes:
        reply = self.transact(packet, expect_reply=True)
        return b"" if reply is None else reply

    def close(self) -> None:
        return None

    def _sync_write(self, params: bytes) -> None:
        address = params[0]
        width = params[1]
        rest = params[2:]
        step = width + 1
        if width == 0 or len(rest) % step != 0:
            raise ValueError("malformed sync write")
        for offset in range(0, len(rest), step):
            servo_id = rest[offset]
            data = bytes(rest[offset + 1 : offset + 1 + width])
            if servo_id in self.mem:
                self._write(servo_id, address, data)

    def _write(self, servo_id: int, address: int, data: bytes) -> None:
        memory = self.mem[servo_id]
        memory[address : address + len(data)] = data
        self.writes.append((servo_id, address, data))
        if address <= ADDR_GOAL_POSITION < address + len(data):
            start = ADDR_GOAL_POSITION - address
            goal = data[start : start + 2]
            if len(goal) == 2:
                memory[ADDR_PRESENT_POSITION : ADDR_PRESENT_POSITION + 2] = goal
