# ESP32-C3 timestamped controller receiver

This ESP-IDF project receives orientation samples from two ESP-NOW controllers
and exposes them through the ESP32-C3 native USB Serial/JTAG CDC port. The
first valid sender after boot is the right slot and the second is the left
slot. Pairing remains in RAM until reboot.

The normal USB stream is intended for the companion three-sphere camera/IMU
tracker in `../pose`. It replaces the previous HadesVR UART stream so sequence
numbers and controller timestamps survive transport to the Kalman filter.

## Fusion USB frame

Every accepted ESP-NOW sample produces one 31-byte little-endian frame:

| Offset | Size | Field |
| --- | ---: | --- |
| 0 | 2 | Magic (`A5 5A`) |
| 2 | 1 | Fusion protocol version (`1`) |
| 3 | 1 | Controller slot (`0` right, `1` left) |
| 4 | 1 | Payload length (`24`) |
| 5 | 24 | Original controller tracker packet |
| 29 | 2 | CRC16-CCITT-FALSE over bytes 2–28 |

The embedded tracker packet is unchanged:

| Offset | Size | Field |
| --- | ---: | --- |
| 0 | 1 | Packet ID (`1`) |
| 1 | 1 | Protocol version (`1`) |
| 2 | 2 | Sequence number |
| 4 | 4 | Low 32 bits of the microsecond sample timestamp |
| 8 | 8 | Quaternion W, X, Y, Z as signed 16-bit values |
| 16 | 6 | Acceleration X, Y, Z as signed 16-bit values |
| 22 | 2 | Status flags |

Quaternion values use 32767 counts per unit and acceleration uses 2048 counts
per g. The USB baud-rate argument is retained for host serial APIs, although
native USB CDC does not use a physical UART baud.

Frame types `0x80` and `0x81` use the same fixed 31-byte envelope for host
calibration commands and receiver calibration status. Their payload length may
be shorter than 24 bytes; unused payload bytes remain zero and the CRC still
covers bytes 2-28. Normal right/left sample frames remain unchanged.

## Wireless controller calibration

The receiver can run a controller's complete gyro, six-face accelerometer, and
magnetometer calibration without connecting that controller to the computer.
Close the pose tracker or any other program using the receiver serial port,
remove the diagnostic jumper, and run:

```sh
python calibrate_controller.py /dev/ttyACM0 --slot right
```

Use `--slot left` for the second controller. The first controller seen after a
receiver reboot is the right slot and the second is the left slot, including
controllers advertising because they do not yet have calibration in NVS. The
tool asks for Enter before each capture and allows magnetometer capture to end
after 10 seconds; otherwise it runs for 30 seconds. Ctrl-C cancels without
replacing a previously saved calibration.

## Native USB diagnostic mode

Connect GPIO 0 directly to GPIO 1 before reset to replace binary output with a
human-readable right/left summary at up to 10 lines per second. Remove the
jumper and reset before using the pose tracker.

## Build, flash, and test

Activate an ESP-IDF 6.0 environment:

```sh
idf.py build
idf.py -p /dev/ttyACM0 flash
```

Run the platform-independent protocol tests with:

```sh
cc -std=c11 -Wall -Wextra -Werror -Imain \
  tests/test_receiver_protocol.c main/receiver_protocol.c \
  -o /tmp/vrrecv_test
/tmp/vrrecv_test
```

Only one process can open the native USB port at a time. Close serial monitors
before starting `vr-led-tracker`.

To inspect the decoded binary samples directly, install `pyserial` and run:

```sh
python check_receiver.py /dev/ttyACM0
```

Use `--slot right` or `--slot left` to filter a controller, and `--raw` to also
show the signed 16-bit values before quaternion and acceleration scaling.
