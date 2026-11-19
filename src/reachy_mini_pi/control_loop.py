"""Background control loop with the same methods the Reachy Mini daemon calls.

The daemon constructs this object, then from its own thread calls
`set_stewart_platform_position`, `enable_torque`, and `get_last_position`.
Those calls return immediately. One thread here owns the UART, so a read
can never cut through the middle of a write.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass
from datetime import timedelta

from reachy_mini_pi.controller import FullBodyPosition, ReachyMiniMotorController, ServoError
from reachy_mini_pi.motors import (
    ANTENNA_NAMES,
    BODY_NAME,
    MOTOR_IDS,
    MOTOR_NAMES,
    STEWART_NAMES,
)
from reachy_mini_pi.protocol import ADDR_TORQUE_ENABLE

logger = logging.getLogger(__name__)

_STALE_S = 0.5


@dataclass
class ControlLoopStats:
    """Counters the daemon prints into its status report."""

    read_failures: int
    last_period_s: float

    def __str__(self) -> str:
        return (
            f"read_failures={self.read_failures} "
            f"last_period_s={self.last_period_s:.4f}"
        )


def _seconds(value: timedelta | float) -> float:
    if isinstance(value, timedelta):
        return value.total_seconds()
    return float(value)


class ReachyMiniPyControlLoop:
    """50 Hz-style loop over the nine HL-2915 servos.

    `read_position_loop_period` is a `datetime.timedelta`, matching the
    stock `ReachyMiniPyControlLoop` constructor. `voltage_rampup_timeout`
    is accepted and ignored: the HL-2915 is a 12 V servo, and this board
    does not do the XL330's PWM voltage cap.
    """

    def __init__(
        self,
        serialport: str,
        read_position_loop_period: timedelta | float,
        allowed_retries: int = 5,
        stats_pub_period: timedelta | float | None = None,
        voltage_rampup_timeout: timedelta | float | None = None,
        bus: object | None = None,
        baudrate: int = 1_000_000,
        signs: dict[str, int] | None = None,
    ):
        del stats_pub_period, voltage_rampup_timeout
        if allowed_retries < 1:
            raise ValueError("allowed_retries must be at least 1")
        self._period = _seconds(read_position_loop_period)
        if self._period <= 0:
            raise ValueError("read_position_loop_period must be positive")
        self._allowed_retries = allowed_retries
        self._controller = ReachyMiniMotorController(
            serialport,
            baudrate=baudrate,
            bus=bus,  # type: ignore[arg-type]
            signs=signs,
        )
        self._lock = threading.Lock()
        self._goals = {name: 0.0 for name in MOTOR_NAMES}
        self._torque = {servo_id: False for servo_id in MOTOR_IDS.values()}
        self._currents = [0] * 6
        self._stewart_mode = 3
        self._body_mode = 0
        self._dirty_torque = False
        self._dirty_mode = False
        self._last_position = FullBodyPosition(0.0, [0.0] * 6, [0.0, 0.0])
        self._last_ok = 0.0
        self._read_failures = 0
        self._last_period_s = self._period
        self._requests: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        try:
            self._connect()
        except Exception:
            self._controller.close()
            raise
        self._thread = threading.Thread(
            target=self._run, name="reachy-mini-pi-motors", daemon=True
        )
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._controller.close()

    def get_motor_name_id(self) -> dict[str, int]:
        return self._controller.get_motor_name_id()

    def get_last_position(self) -> FullBodyPosition:
        with self._lock:
            if self._last_ok == 0.0:
                raise ServoError("No motor position has been read yet")
            if time.monotonic() - self._last_ok > _STALE_S:
                raise ServoError("No response from the motors")
            position = self._last_position
        return FullBodyPosition(position.body_yaw, position.stewart, position.antennas)

    def get_stats(self) -> ControlLoopStats:
        with self._lock:
            return ControlLoopStats(self._read_failures, self._last_period_s)

    def is_torque_enabled(self) -> bool:
        with self._lock:
            return all(self._torque.values())

    def enable_torque(self) -> None:
        self._set_torque(list(MOTOR_IDS.values()), True)

    def disable_torque(self) -> None:
        self._set_torque(list(MOTOR_IDS.values()), False)

    def enable_torque_on_ids(self, ids: list[int]) -> None:
        self._set_torque(ids, True)

    def disable_torque_on_ids(self, ids: list[int]) -> None:
        self._set_torque(ids, False)

    def enable_body_rotation(self, enable: bool) -> None:
        self._set_torque([MOTOR_IDS[BODY_NAME]], enable)

    def enable_antennas(self, enable: bool) -> None:
        self._set_torque([MOTOR_IDS[name] for name in ANTENNA_NAMES], enable)

    def enable_stewart_platform(self, enable: bool) -> None:
        self._set_torque([MOTOR_IDS[name] for name in STEWART_NAMES], enable)

    def set_all_goal_positions(self, positions: list[float] | FullBodyPosition) -> None:
        if isinstance(positions, FullBodyPosition):
            values = positions.as_list()
        else:
            values = list(positions)
        if len(values) != 9:
            raise ValueError(f"expected 9 joint positions, got {len(values)}")
        with self._lock:
            for name, value in zip(MOTOR_NAMES, values, strict=True):
                self._goals[name] = float(value)

    def set_stewart_platform_position(self, position: list[float]) -> None:
        if len(position) != 6:
            raise ValueError(f"stewart platform needs 6 positions, got {len(position)}")
        with self._lock:
            for name, value in zip(STEWART_NAMES, position, strict=True):
                self._goals[name] = float(value)

    def set_body_rotation(self, position: float) -> None:
        with self._lock:
            self._goals[BODY_NAME] = float(position)

    def set_antennas_positions(self, positions: list[float]) -> None:
        if len(positions) != 2:
            raise ValueError(f"antennas need 2 positions, got {len(positions)}")
        with self._lock:
            for name, value in zip(ANTENNA_NAMES, positions, strict=True):
                self._goals[name] = float(value)

    def set_stewart_platform_goal_current(self, current: list[int]) -> None:
        if len(current) != 6:
            raise ValueError(f"stewart platform needs 6 currents, got {len(current)}")
        with self._lock:
            self._currents = [int(value) for value in current]

    def set_stewart_platform_operating_mode(self, mode: int) -> None:
        with self._lock:
            self._stewart_mode = int(mode)
            self._dirty_mode = True

    def set_body_rotation_operating_mode(self, mode: int) -> None:
        with self._lock:
            self._body_mode = int(mode)
            self._dirty_mode = True

    def set_antennas_operating_mode(self, mode: int) -> None:
        # Antennas stay in position mode unless a caller asks otherwise.
        # The stock daemon never calls this; the synchronous controller does.
        self._on_thread(
            lambda: self._controller.set_antennas_operating_mode(int(mode))
        )

    def get_stewart_platform_operating_mode(self) -> int:
        with self._lock:
            return self._stewart_mode

    def async_read_raw_bytes(self, servo_id: int, address: int, length: int) -> list[int]:
        data = self._on_thread(
            lambda: self._controller.read_bytes(int(servo_id), int(address), int(length))
        )
        return list(data)

    def async_write_raw_bytes(self, servo_id: int, address: int, data: list[int]) -> None:
        payload = bytes(int(value) & 0xFF for value in data)
        self._on_thread(
            lambda: self._controller.write_bytes(int(servo_id), int(address), payload)
        )

    def async_read_pid_gains(self, servo_id: int) -> tuple[int, int, int]:
        return self._on_thread(lambda: self._controller.read_pid_gains(int(servo_id)))

    def async_write_pid_gains(self, servo_id: int, p: int, i: int, d: int) -> None:
        self._on_thread(
            lambda: self._controller.write_pid_gains(int(servo_id), int(p), int(i), int(d))
        )

    def write_raw_packet(self, data: bytes) -> bytes:
        packet = bytes(data)
        return self._on_thread(lambda: self._controller.write_raw_packet(packet))

    def _connect(self) -> None:
        missing = []
        for name, servo_id in MOTOR_IDS.items():
            if not self._ping_with_retries(servo_id):
                missing.append(f"{name} (id {servo_id})")
        if missing:
            raise ServoError(
                "These servos did not answer: "
                + ", ".join(missing)
                + ". An HL-2915 ships as id 1 at 1000000 baud. "
                "Plug in one servo at a time and run reachy-mini-pi-setup-motor."
            )
        for name, servo_id in MOTOR_IDS.items():
            self._goals[name] = self._read_joint_with_retries(name)
            torque = self._controller.read_bytes(servo_id, ADDR_TORQUE_ENABLE, 1)
            self._torque[servo_id] = torque == b"\x01"
        self._stewart_mode = self._controller.load_reported_stewart_mode()
        self._publish_positions(time.monotonic())

    def _ping_with_retries(self, servo_id: int) -> bool:
        for _ in range(self._allowed_retries):
            if self._controller.ping(servo_id):
                return True
        return False

    def _read_joint_with_retries(self, name: str) -> float:
        last_error: Exception | None = None
        for _ in range(self._allowed_retries):
            try:
                return self._controller.read_joint_radians(name)
            except (ServoError, TimeoutError, OSError) as exc:
                last_error = exc
        raise ServoError(f"could not read {name}: {last_error}")

    def _set_torque(self, ids: list[int], enabled: bool) -> None:
        with self._lock:
            for servo_id in ids:
                if servo_id not in self._torque:
                    raise ValueError(f"unknown servo id {servo_id}")
                self._torque[servo_id] = enabled
            self._dirty_torque = True

    def _run(self) -> None:
        while not self._stop.is_set():
            started = time.perf_counter()
            try:
                self._drain()
                self._apply()
                self._read_positions()
            except Exception:
                logger.exception("motor control step failed")
                with self._lock:
                    self._read_failures += 1
            elapsed = time.perf_counter() - started
            with self._lock:
                self._last_period_s = elapsed
            if self._stop.wait(max(0.0, self._period - elapsed)):
                break

    def _drain(self) -> None:
        while True:
            try:
                job = self._requests.get_nowait()
            except queue.Empty:
                return
            job()

    def _apply(self) -> None:
        with self._lock:
            torque = dict(self._torque)
            goals = dict(self._goals)
            stewart_mode = self._stewart_mode
            body_mode = self._body_mode
            currents = list(self._currents)
            write_torque = self._dirty_torque
            write_mode = self._dirty_mode
            self._dirty_torque = False
            self._dirty_mode = False

        if write_mode:
            self._controller.set_stewart_platform_operating_mode(stewart_mode)
            self._controller.set_body_rotation_operating_mode(body_mode)
        if write_torque:
            enabled = [servo_id for servo_id, on in torque.items() if on]
            disabled = [servo_id for servo_id, on in torque.items() if not on]
            if enabled:
                self._controller.set_torque_ids(enabled, True)
            if disabled:
                self._controller.set_torque_ids(disabled, False)

        # Writing a goal position turns torque back on, so a coasting servo
        # must be left out of the position packet.
        if stewart_mode == 0:
            if any(torque[MOTOR_IDS[name]] for name in STEWART_NAMES):
                self._controller.set_stewart_platform_goal_current(currents)
            names = [BODY_NAME, *ANTENNA_NAMES]
        else:
            names = list(MOTOR_NAMES)
        joints = [
            (name, goals[name]) for name in names if torque[MOTOR_IDS[name]]
        ]
        if joints:
            self._controller._sync_positions(joints)

    def _read_positions(self) -> None:
        values: dict[str, float] = {}
        for name in MOTOR_NAMES:
            last_error: Exception | None = None
            for _ in range(min(self._allowed_retries, 2)):
                try:
                    values[name] = self._controller.read_joint_radians(name)
                    last_error = None
                    break
                except (ServoError, TimeoutError, OSError) as exc:
                    last_error = exc
            if last_error is not None:
                with self._lock:
                    self._read_failures += 1
                logger.warning("position read failed for %s: %s", name, last_error)
                return
        self._publish_positions(time.monotonic(), values)

    def _publish_positions(
        self, when: float, values: dict[str, float] | None = None
    ) -> None:
        with self._lock:
            source = values if values is not None else self._goals
            self._last_position = FullBodyPosition(
                source[BODY_NAME],
                [source[name] for name in STEWART_NAMES],
                [source[name] for name in ANTENNA_NAMES],
            )
            self._last_ok = when

    def _on_thread(self, fn):
        if self._thread is not None and threading.current_thread() is self._thread:
            return fn()
        box: dict = {}

        def job() -> None:
            try:
                box["result"] = fn()
            except Exception as exc:
                box["error"] = exc

        self._requests.put(job)
        # The loop drains the queue at the start of every period. Wait a
        # little longer than one period so a slow read still lands.
        deadline = time.monotonic() + max(1.0, self._period * 4)
        while "result" not in box and "error" not in box:
            if time.monotonic() > deadline:
                raise ServoError("motor control thread did not answer")
            time.sleep(0.001)
        if "error" in box:
            raise box["error"]
        return box["result"]
