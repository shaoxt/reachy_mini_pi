"""Serial port transport for the Waveshare Bus Servo Adapter.

The adapter owns the half-duplex direction pin. The Pi just writes and
reads a normal UART. Jumper A is the GPIO UART. Jumper B is the USB port.
"""

from __future__ import annotations

import time

from reachy_mini_pi.protocol import pop_packet

try:
    import serial
except ImportError:  # pragma: no cover - exercised only on a bare environment
    serial = None  # type: ignore[assignment]


class ServoBus:
    """One owner of the adapter's UART."""

    def __init__(self, port: str, baudrate: int = 1_000_000, timeout: float = 0.02):
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: pip install pyserial")
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        try:
            self._ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
                exclusive=True,
            )
        except PermissionError as exc:
            raise PermissionError(
                f"Cannot open {port}. Add your user to the dialout group "
                "and log in again: sudo usermod -aG dialout $USER"
            ) from exc
        self._buf = bytearray()

    def transact(self, packet: bytes, expect_reply: bool) -> bytes | None:
        """Send one frame. Return the status frame, or None for a broadcast."""
        self._ser.reset_input_buffer()
        self._buf.clear()
        self._ser.write(packet)
        self._ser.flush()
        if not expect_reply:
            return None
        return self._read_packet()

    def write_raw(self, packet: bytes) -> bytes:
        """Send bytes that are already a full frame and collect one reply."""
        reply = self.transact(packet, expect_reply=True)
        return b"" if reply is None else reply

    def close(self) -> None:
        if self._ser.is_open:
            self._ser.close()

    def _read_packet(self) -> bytes:
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            waiting = self._ser.in_waiting
            chunk = self._ser.read(waiting or 1)
            if chunk:
                self._buf.extend(chunk)
                packet = pop_packet(self._buf)
                if packet is not None:
                    return packet
        raise TimeoutError(
            f"No servo reply on {self.port} at {self.baudrate} baud. "
            "Check 12 V servo power, the adapter jumper, and TX/RX."
        )
