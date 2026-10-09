#!/usr/bin/env python3
"""Print decoded controller samples from the receiver USB serial stream."""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass

try:
    import serial
except ImportError:
    print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
    raise SystemExit(2)


MAGIC = b"\xa5\x5a"
PROTOCOL_VERSION = 1
PAYLOAD_SIZE = 24
FRAME_SIZE = 31


def crc16_ccitt(data: bytes | bytearray) -> int:
    """CRC16-CCITT-FALSE used by receiver_build_fusion_frame()."""
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Sample:
    slot: int
    sequence: int
    timestamp_us: int
    quaternion_raw: tuple[int, int, int, int]
    acceleration_raw: tuple[int, int, int]
    status: int


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.frames = 0
        self.invalid_bytes = 0
        self.crc_errors = 0

    def feed(self, data: bytes) -> list[Sample]:
        self.buffer.extend(data)
        samples: list[Sample] = []

        while True:
            start = self.buffer.find(MAGIC)
            if start < 0:
                keep = 1 if self.buffer[-1:] == MAGIC[:1] else 0
                discarded = len(self.buffer) - keep
                self.invalid_bytes += discarded
                if discarded:
                    del self.buffer[:discarded]
                break

            if start:
                self.invalid_bytes += start
                del self.buffer[:start]

            if len(self.buffer) < FRAME_SIZE:
                break

            frame = self.buffer[:FRAME_SIZE]
            if (
                frame[2] != PROTOCOL_VERSION
                or frame[3] not in (0, 1)
                or frame[4] != PAYLOAD_SIZE
            ):
                self.invalid_bytes += 1
                del self.buffer[0]
                continue

            expected_crc = struct.unpack_from("<H", frame, 29)[0]
            if crc16_ccitt(frame[2:29]) != expected_crc:
                self.crc_errors += 1
                self.invalid_bytes += 1
                del self.buffer[0]
                continue

            packet_id, packet_version, sequence, timestamp_us, *values = struct.unpack(
                "<BBHI4h3hH", frame[5:29]
            )
            if packet_id != 1 or packet_version != 1:
                self.invalid_bytes += FRAME_SIZE
                del self.buffer[:FRAME_SIZE]
                continue

            samples.append(
                Sample(
                    slot=frame[3],
                    sequence=sequence,
                    timestamp_us=timestamp_us,
                    quaternion_raw=tuple(values[:4]),
                    acceleration_raw=tuple(values[4:7]),
                    status=values[7],
                )
            )
            self.frames += 1
            del self.buffer[:FRAME_SIZE]

        return samples


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Decode and print the vrrecv fusion stream."
    )
    parser.add_argument("device", nargs="?", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument(
        "--slot",
        choices=("right", "left", "both"),
        default="both",
        help="controller slot to print (default: both)",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="also print the signed 16-bit sensor values",
    )
    return parser.parse_args()


def vector_norm(values: tuple[float, ...]) -> float:
    return math.sqrt(sum(value * value for value in values))


def main() -> int:
    args = parse_args()
    selected_slot = {"right": 0, "left": 1}.get(args.slot)
    parser = FrameParser()
    previous_sequence: dict[int, int] = {}
    previous_timestamp: dict[int, int] = {}
    started = time.monotonic()

    try:
        port = serial.Serial(args.device, args.baud, timeout=0.1)
    except serial.SerialException as exc:
        print(f"Cannot open {args.device}: {exc}", file=sys.stderr)
        return 1

    print(f"Listening on {args.device} at {args.baud} baud; press Ctrl-C to stop.")
    print("slot  sequence  timestamp_us  dt_us   gap  quaternion (w x y z)"
          "                     |q|     acceleration_g (x y z)       |a|   status")

    try:
        while True:
            data = port.read(max(1, min(port.in_waiting, 4096)))
            for sample in parser.feed(data):
                if selected_slot is not None and sample.slot != selected_slot:
                    continue

                prior_sequence = previous_sequence.get(sample.slot)
                prior_timestamp = previous_timestamp.get(sample.slot)
                gap = 0 if prior_sequence is None else (
                    (sample.sequence - prior_sequence - 1) & 0xFFFF
                )
                dt_us = 0 if prior_timestamp is None else (
                    (sample.timestamp_us - prior_timestamp) & 0xFFFFFFFF
                )
                previous_sequence[sample.slot] = sample.sequence
                previous_timestamp[sample.slot] = sample.timestamp_us

                quaternion = tuple(value / 32767.0 for value in sample.quaternion_raw)
                acceleration = tuple(value / 2048.0 for value in sample.acceleration_raw)
                slot_name = "right" if sample.slot == 0 else "left "
                print(
                    f"{slot_name} {sample.sequence:8d} {sample.timestamp_us:13d} "
                    f"{dt_us:6d} {gap:5d}  "
                    f"[{quaternion[0]:+7.4f} {quaternion[1]:+7.4f} "
                    f"{quaternion[2]:+7.4f} {quaternion[3]:+7.4f}] "
                    f"{vector_norm(quaternion):6.3f}  "
                    f"[{acceleration[0]:+7.3f} {acceleration[1]:+7.3f} "
                    f"{acceleration[2]:+7.3f}] {vector_norm(acceleration):6.3f}  "
                    f"0x{sample.status:04x}",
                    flush=True,
                )
                if args.raw:
                    print(
                        f"      raw q={sample.quaternion_raw} "
                        f"a={sample.acceleration_raw}",
                        flush=True,
                    )
    except KeyboardInterrupt:
        elapsed = max(time.monotonic() - started, 1e-9)
        print(
            f"\nStopped: {parser.frames} frames in {elapsed:.1f}s "
            f"({parser.frames / elapsed:.1f} frames/s), "
            f"CRC errors={parser.crc_errors}, invalid bytes={parser.invalid_bytes}"
        )
    except serial.SerialException as exc:
        print(f"\nSerial error: {exc}", file=sys.stderr)
        return 1
    finally:
        port.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
