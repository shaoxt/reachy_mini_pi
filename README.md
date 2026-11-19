# Reachy Mini on a Raspberry Pi 4B

This is the motor half of Reachy Mini, rewritten for different hardware:

| Stock Reachy Mini | This repo |
| --- | --- |
| Dynamixel XL330 / XC330 | Feetech **HL-2915-C002** |
| USB2Dynamixel-style adapter | Waveshare **Bus Servo Adapter (A)** ([the board you linked](https://www.amazon.com/dp/B0CJ6TP3TP)) |
| Rust `reachy_mini_motor_controller` | This Python package |

The head still has the same nine joints, in the same order, in radians:

`body_rotation`, `stewart_1` … `stewart_6`, `right_antenna`, `left_antenna`

`ReachyMiniPyControlLoop` has the same methods the Reachy Mini daemon already calls (`set_body_rotation`, `set_stewart_platform_position`, `enable_torque`, `get_last_position`, …). Joint 0 is still the calibrated center of the servo.

## What this hardware actually is

The HL-2915-C002 is a 12 V serial-bus servo:

- one wire carries a half-duplex UART conversation
- factory id **1**, factory baud **1 000 000**, 8 data bits, no parity, 1 stop bit
- one turn is 4096 counts, and 2048 is the center (about 0.088° per count)
- it reports position, current, voltage, and temperature

The Waveshare board is the adapter for that wire. It powers the servos and turns the Pi's normal UART into the single-wire bus. The Pi does not bit-bang PWM, and it does not use I2C.

A full Reachy Mini needs all nine servos on **one** bus, each with its own id. Out of the box every servo is id 1, so they have to be numbered before they are daisy-chained.

## Power

Use a **12 V** supply on the adapter's DC jack or the green screw terminal. The HL-2915 wants 9–14 V, and this adapter is rated 9–12.6 V, so 12 V is the match.

Each C002 draws about 140 mA with no load and up to 600 mA stalled. Nine of them need a supply that can deliver **at least 5 A**. Servo power comes from that 12 V supply. The Pi's 5 V pin cannot feed these motors. Do share a ground between the Pi and the adapter.

## Wiring

**USB (simplest on a Pi 4B).** Put the adapter jumper on **B**. Plug the adapter into a Pi USB port. The serial device is usually `/dev/ttyUSB0` or `/dev/ttyACM0`.

**GPIO UART.** Put the jumper on **A**. On the Pi 4B header:

| Pi pin | Adapter |
| --- | --- |
| Pin 8, GPIO14 TXD | TX |
| Pin 10, GPIO15 RXD | RX |
| Pin 6 GND | GND |

Waveshare's own guide labels this RX-to-RX and TX-to-TX, because of how the silkscreen is drawn. If a scan finds nothing, swap TX and RX once. Servo power still comes from the 12 V jack, not from these pins.

Use the Pi's PL011 UART. The mini-UART drops bytes at 1 Mbaud. In `/boot/firmware/config.txt`:

```
enable_uart=1
dtoverlay=disable-bt
```

Remove `console=serial0,115200` from `/boot/firmware/cmdline.txt`, reboot, and the port is `/dev/serial0`.

```bash
sudo usermod -aG dialout $USER
```

Log out and back in so the group takes effect.

The servo plug is GND, VCC, Signal. Match the adapter's silkscreen. A reversed plug will not rotate the servo.

## Install

On the Pi:

```bash
cd reachy_mini_pi
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

## Bring-up, one servo at a time

1. Power the adapter with 12 V. Connect **one** servo.
2. See that it answers. A new servo is id 1:

```bash
reachy-mini-pi-scan --port /dev/ttyUSB0
```

3. Move it 15° and back, so you know the direction before it is mounted:

```bash
python examples/wiggle.py --port /dev/ttyUSB0 --id 1 --degrees 15
```

4. Give it a Reachy Mini id. Unplug the others while you do this. The names and ids are fixed:

Change the `id` for a joint in `src/reachy_mini_pi/motors_config.py` before you assign it. The defaults match the stock robot:

| Joint | Id | Homing offset (counts) |
| --- | --- | --- |
| body_rotation | 10 | 0 |
| stewart_1 | 11 | +1024 |
| stewart_2 | 12 | -1024 |
| stewart_3 | 13 | +1024 |
| stewart_4 | 14 | -1024 |
| stewart_5 | 15 | +1024 |
| stewart_6 | 16 | -1024 |
| right_antenna | 17 | 0 |
| left_antenna | 18 | 0 |

```bash
reachy-mini-pi-setup-motor --port /dev/ttyUSB0 --motor stewart_1
```

That writes the id from `motors_config.py`, plus the offset, the travel limits, and HLS position mode. Torque is left off. The offset numbers are the same ones the stock robot stores in its Dynamixels, so a horn mounted the same way still has joint 0 in the right place.

If the horn is already on and you can hold the joint at the pose that should be zero, use `--center` instead. That tells the servo "where you are right now is count 2048" and does not also apply the stock offset.

5. Repeat for the other eight servos, then daisy-chain them and scan again. You want ids 10 through 18.

6. A small motion on all nine:

```bash
python examples/sin.py --port /dev/ttyUSB0 --degrees 10
```

Ctrl-C turns torque off.

If a mounted joint runs the wrong way, set that joint's `sign` to `-1` in `src/reachy_mini_pi/motors_config.py`. That flips the radian command without rewriting the servo.

## Using it from the Reachy Mini daemon

The daemon currently imports the Dynamixel controller:

```python
from reachy_mini_motor_controller import ReachyMiniPyControlLoop
```

Point that import at this package:

```python
from reachy_mini_pi import ReachyMiniPyControlLoop
```

The constructor is the same (`serialport`, `read_position_loop_period`, `allowed_retries`). Pass the adapter's port (`/dev/ttyUSB0` or `/dev/serial0`) instead of the Dynamixel port.

Two Dynamixel details are intentionally not copied across:

- The daemon's hardware-error reader uses Dynamixel register 70 and register 144. On this servo, status is register 65 and voltage is register 62 (0.1 V per count). `async_read_raw_bytes` will read whatever address you give it.
- The yaml PID values (P=300) are Dynamixel units. HL-2915 gains are 0–254 on a different scale, so those writes are ignored and the factory tune is left alone. Gains you pass in the 0–254 range are written to Kp, Kd, Ki.

Stewart operating modes still use the Dynamixel numbers the daemon sends: 3 and 5 are position, 0 is current (gravity compensation), 1 is constant speed. The yaw motor is special only because the daemon writes mode 0 to mean "position", and position mode on this servo really is 0.

`get_last_position()` returns radians with `.body_yaw`, `.stewart` (6 values), and `.antennas` (right, then left). A read that stays silent for half a second raises, which is what makes the daemon stop instead of commanding a pose it can no longer see.

## Tests

```bash
pytest
```

The tests run against an in-memory servo bus. They do not open a serial port.
