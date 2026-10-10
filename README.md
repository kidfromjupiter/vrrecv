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

### Learned sensor axes

Build and flash **both** controller and receiver firmware and use the updated
`calibrate_controller.py`; calibration commands/status now use protocol v2.
Orientation samples, the USB envelope, and magnitude diagnostics keep their
existing formats/versions. Old calibration tools cannot start the new wizard.

Body directions are **+X right, +Y front, +Z top**, a right-handed frame.
The first six poses now refer to these physical controller directions, not to
MPU package markings. The wizard determines a signed MPU axis permutation and
native accelerometer offsets/scales from the opposite faces. That same MPU
map applies to both acceleration and gyro. Gyro bias is measured while still;
gyro scale remains the nominal sensitivity.

After figure-eight magnetometer calibration, follow eight stationary holds:
right/front/top, left/front/top, right/back/top, left/back/top,
right/front/bottom, left/front/bottom, right/back/bottom, and left/back/bottom.
Point each indicated corner upward so its three directions point equally
upward. No precise heading is required; vary the front's compass direction
between holds while staying near the same location. Each hold gathers 100
fresh paired readings, with a five-second limit and a short settling delay.

The alignment solver compares all 24 proper signed magnetometer maps using
the consistency of the normalized gravity–magnetic-field dot product. Poor
or ambiguous alignment fails calibration and leaves the previous value active.
Movement or incorrect accelerometer poses return to the same prompt for a
manual retry, preserving completed steps. An invalid combined six-face fit
repeats all six faces. Ctrl-C still cancels. Magnetic overflow, insufficient
samples, and sensor errors abort without replacing saved calibration. The host prints both saved maps
as `body <- sensor` assignments when the wizard succeeds.

NVS v2 stores both maps and a learned-map flag together with the coefficients.
A valid v1 record remains usable with the configured fallback maps until the
new wizard succeeds. Those maps are explicitly labeled legacy. Native offsets
and scales are applied before mapping, exactly once. Only per-axis scaling
and mounting at 90-degree increments are supported; arbitrary rotation
matrices and full soft-iron compensation are not estimated.

Acceptance limits live in `../vrcontroller/main/tracker_config.h`: face dominance 0.95,
pose cosine 0.94, accelerometer standard deviation 0.02 g, residual gyro rate
5 degrees/second, and magnetic standard deviation 1 µT during a hold. The
magnetometer map score must be at most 0.05, and the runner-up at least
`max(1.5 * best, best + 0.03)`. These are initial defaults requiring hardware
validation.

## Magnetometer calibration quality check

Flash the updated controller and receiver firmware, remove the receiver's
GPIO 0–1 diagnostic jumper, and close any other program using its USB port.
Check the saved calibration without repeating the wizard:

```sh
python check_magnetometer.py /dev/ttyACM0 --slot right --duration 30
```

Use `--slot left` for the second controller. Press Enter when ready, then
rotate through all orientations near one location for the entire capture.
The tool compares raw and calibrated magnetic-field magnitude in µT, reporting
minimum, maximum, mean, population standard deviation, and peak-to-peak range.
It also reports standard deviation and peak-to-peak range as percentages of
the mean, plus their relative reduction after calibration. Negative reduction
means variation increased. A zero raw variation or mean makes the relative
comparison unavailable.

Lower calibrated variation indicates more consistent magnitude during this
rotation test; the report does not assign a pass/fail grade or establish heading
accuracy. Changing magnetic surroundings or incomplete rotation coverage can
affect the result. Finite readings outside the fusion range (10–100 µT) remain
in the statistics and are counted. Stale, duplicate, nonfinite, uncalibrated,
overflow, and sensor-error readings are excluded and reported. At least 20
usable pairs are required. The duration defaults to 30 seconds and accepts
5–300 seconds.

Diagnostic data is streamed only during a check, at up to 10 Hz. Fusion
continues at its normal rate; diagnostic sends can replace some orientation
transmissions while the check runs. The normal orientation packet format stays
unchanged. Ctrl-C sends a stop command, and the controller automatically stops
streaming when the requested duration expires or calibration starts.

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

### Magnetometer diagnostic transport and tests

USB frame type `0x82` carries a slot byte followed by an 8-byte magnetometer
command (ESP-NOW packet ID 5). Type `0x83` carries a slot byte followed by a
20-byte sample (ESP-NOW packet ID 4). Both use the existing 31-byte envelope,
version 1, zero padding, and CRC. Orientation and calibration frames are
unchanged. Controller and receiver diagnostic codecs must be updated together.
The diagnostic packet layout is documented in `../vrcontroller/README.md`.

```sh
cc -std=c11 -Wall -Wextra -Werror -DTEST_RECEIVER -Imain \
  tests/test_magnetometer_protocol.c main/magnetometer_protocol.c \
  main/receiver_protocol.c -lm -o /tmp/vrrecv_mag_test
/tmp/vrrecv_mag_test
python3 -m unittest discover -s tests -v
```

### Calibration v2 map reports

Controller calibration packet IDs 2/3 now use version 2; their sizes stay
8/12 bytes. Packet ID 6 is a 12-byte map report using calibration version 2.
USB frame type `0x84` carries a slot byte followed by that report, for a
13-byte payload in the unchanged 31-byte envelope. The report layout and
new corner-stage/result codes are documented in `../vrcontroller/README.md`.
The receiver validates reports before pairing/forwarding. The host caches
reports by slot/request and queries the saved maps again after successful
calibration, so a missing terminal report can be recovered. Both firmware
repositories and the host tool must be updated together; ordinary pose and
magnetometer diagnostic packets stay compatible.

```sh
cc -std=c11 -Wall -Wextra -Werror -DTEST_RECEIVER -Imain \
  tests/test_calibration_protocol.c main/receiver_protocol.c \
  -o /tmp/vrrecv_calibration_protocol_test
/tmp/vrrecv_calibration_protocol_test
python3 -m unittest discover -s tests -v
```
