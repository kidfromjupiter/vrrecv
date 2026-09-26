# VR receiver protocol core

ESP-IDF project for the transport-independent parts of a two-controller VR
receiver. The protocol module decodes 24-byte tracker samples, maintains right
and left controller state, and builds the 67-byte UART frame expected by the
HadesVR SteamVR driver.

No input transport is currently configured. The firmware entry point is an
empty stub, so a flashed device does not receive controller samples or emit
HadesVR frames. A future transport can decode incoming samples with
`receiver_packet_decode`, apply them to an explicitly selected controller slot
with `receiver_state_apply_sample`, and construct output with
`receiver_build_hades_frame`.

## Tracker packet format

Tracker samples use protocol version 1 and contain:

- Packet ID and version: 2 bytes
- Sequence number: 2 bytes
- Microsecond timestamp: 4 bytes
- Quaternion: four little-endian signed 16-bit values
- Acceleration: three little-endian signed 16-bit values
- Status flags: 2 bytes

Quaternion values are scaled by 32767 and acceleration values by 2048 per g.

## Build and test

Build and flash with an ESP-IDF 6.0 environment active:

```sh
idf.py build
idf.py flash
```

Run the platform-independent protocol tests with:

```sh
cc -std=c11 -Wall -Wextra -Werror -Imain \
  tests/test_receiver_protocol.c main/receiver_protocol.c \
  -o /tmp/vrrecv_test
/tmp/vrrecv_test
```
