#!/usr/bin/env python3
"""Run a controller's full sensor calibration through vrrecv."""

from __future__ import annotations

import argparse
import select
import struct
import sys
import time
from dataclasses import dataclass


MAGIC = b"\xa5\x5a"
USB_VERSION = 1
FRAME_SIZE = 31
PAYLOAD_CAPACITY = 24
FRAME_CALIBRATION_COMMAND = 0x80
FRAME_CALIBRATION_STATUS = 0x81

PACKET_CALIBRATION_COMMAND = 2
PACKET_CALIBRATION_STATUS = 3
PACKET_VERSION = 1

ACTION_QUERY = 1
ACTION_START_FULL = 2
ACTION_CONFIRM = 3
ACTION_FINISH_MAG = 4
ACTION_CANCEL = 5

STAGE_IDLE = 0
STAGE_WAIT_STILL = 1
STAGE_CAPTURE_STILL = 2
STAGE_WAIT_POS_X = 3
STAGE_CAPTURE_POS_X = 4
STAGE_WAIT_NEG_X = 5
STAGE_CAPTURE_NEG_X = 6
STAGE_WAIT_POS_Y = 7
STAGE_CAPTURE_POS_Y = 8
STAGE_WAIT_NEG_Y = 9
STAGE_CAPTURE_NEG_Y = 10
STAGE_WAIT_POS_Z = 11
STAGE_CAPTURE_POS_Z = 12
STAGE_WAIT_NEG_Z = 13
STAGE_CAPTURE_NEG_Z = 14
STAGE_WAIT_MAG = 15
STAGE_CAPTURE_MAG = 16
STAGE_SAVING = 17
STAGE_COMPLETE = 18
STAGE_FAILED = 19
STAGE_CANCELLED = 20

RESULT_OK = 1
RESULT_UNAVAILABLE = 11
STATUS_CAN_FINISH = 0x02

WAIT_PROMPTS = {
    STAGE_WAIT_STILL: "Keep the controller completely still on a stable surface.",
    STAGE_WAIT_POS_X: "Place the controller with MPU +X upward.",
    STAGE_WAIT_NEG_X: "Place the controller with MPU -X upward.",
    STAGE_WAIT_POS_Y: "Place the controller with MPU +Y upward.",
    STAGE_WAIT_NEG_Y: "Place the controller with MPU -Y upward.",
    STAGE_WAIT_POS_Z: "Place the controller with MPU +Z upward.",
    STAGE_WAIT_NEG_Z: "Place the controller with MPU -Z upward.",
    STAGE_WAIT_MAG: "Prepare to move the controller through broad figure-eights in every orientation.",
}

CAPTURE_NAMES = {
    STAGE_CAPTURE_STILL: "Capturing stationary gyro bias",
    STAGE_CAPTURE_POS_X: "Capturing +X face",
    STAGE_CAPTURE_NEG_X: "Capturing -X face",
    STAGE_CAPTURE_POS_Y: "Capturing +Y face",
    STAGE_CAPTURE_NEG_Y: "Capturing -Y face",
    STAGE_CAPTURE_POS_Z: "Capturing +Z face",
    STAGE_CAPTURE_NEG_Z: "Capturing -Z face",
    STAGE_SAVING: "Saving calibration",
}

RESULT_NAMES = {
    0: "none",
    1: "ok",
    2: "controller busy",
    3: "invalid calibration stage",
    4: "sensor read failed",
    5: "controller moved during stationary capture",
    6: "wrong or unstable accelerometer face",
    7: "not enough magnetometer samples",
    8: "magnetic field is outside the valid range",
    9: "could not save calibration",
    10: "cancelled",
    11: "controller slot unavailable",
    12: "operation timed out",
    13: "ESP-NOW transport failed",
}


def crc16_ccitt(data: bytes | bytearray) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_usb_frame(frame_type: int, payload: bytes) -> bytes:
    if len(payload) > PAYLOAD_CAPACITY:
        raise ValueError("USB payload is too large")
    frame = bytearray(FRAME_SIZE)
    frame[0:2] = MAGIC
    frame[2] = USB_VERSION
    frame[3] = frame_type
    frame[4] = len(payload)
    frame[5 : 5 + len(payload)] = payload
    struct.pack_into("<H", frame, 29, crc16_ccitt(frame[2:29]))
    return bytes(frame)


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0
        self.invalid_bytes = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buffer.extend(data)
        frames: list[tuple[int, bytes]] = []
        while True:
            start = self.buffer.find(MAGIC)
            if start < 0:
                keep = 1 if self.buffer[-1:] == MAGIC[:1] else 0
                self.invalid_bytes += len(self.buffer) - keep
                if len(self.buffer) > keep:
                    del self.buffer[: len(self.buffer) - keep]
                break
            if start:
                self.invalid_bytes += start
                del self.buffer[:start]
            if len(self.buffer) < FRAME_SIZE:
                break
            candidate = self.buffer[:FRAME_SIZE]
            if candidate[2] != USB_VERSION or candidate[4] > PAYLOAD_CAPACITY:
                self.invalid_bytes += 1
                del self.buffer[0]
                continue
            expected = struct.unpack_from("<H", candidate, 29)[0]
            if crc16_ccitt(candidate[2:29]) != expected:
                self.crc_errors += 1
                self.invalid_bytes += 1
                del self.buffer[0]
                continue
            size = candidate[4]
            frames.append((candidate[3], bytes(candidate[5 : 5 + size])))
            del self.buffer[:FRAME_SIZE]
        return frames


@dataclass(frozen=True)
class CalibrationStatus:
    slot: int
    stage: int
    result: int
    flags: int
    elapsed_seconds: int
    request_id: int
    sample_count: int


def encode_command(slot: int, action: int, expected_stage: int, request_id: int) -> bytes:
    packet = struct.pack(
        "<BBBBH2x",
        PACKET_CALIBRATION_COMMAND,
        PACKET_VERSION,
        action,
        expected_stage,
        request_id,
    )
    return build_usb_frame(FRAME_CALIBRATION_COMMAND, bytes([slot]) + packet)


def decode_status(payload: bytes) -> CalibrationStatus | None:
    if len(payload) != 13 or payload[1:3] != bytes(
        [PACKET_CALIBRATION_STATUS, PACKET_VERSION]
    ):
        return None
    stage, result, flags, elapsed, request_id, samples = struct.unpack_from(
        "<BBBBHH", payload, 3
    )
    return CalibrationStatus(
        payload[0], stage, result, flags, elapsed, request_id, samples
    )


class CalibrationClient:
    def __init__(self, port: object, slot: int, request_id: int) -> None:
        self.port = port
        self.slot = slot
        self.request_id = request_id
        self.parser = FrameParser()
        self.current_stage = STAGE_IDLE

    def send(self, action: int, expected_stage: int = STAGE_IDLE) -> None:
        self.port.write(encode_command(self.slot, action, expected_stage, self.request_id))
        self.port.flush()

    def statuses(self, timeout: float) -> list[CalibrationStatus]:
        deadline = time.monotonic() + timeout
        found: list[CalibrationStatus] = []
        while time.monotonic() < deadline and not found:
            data = self.port.read(max(1, min(getattr(self.port, "in_waiting", 0), 4096)))
            for frame_type, payload in self.parser.feed(data):
                if frame_type != FRAME_CALIBRATION_STATUS:
                    continue
                status = decode_status(payload)
                if status is not None and status.slot == self.slot:
                    found.append(status)
        return found

    def wait_for_change(self, stage: int, action: int, timeout: float = 12.0) -> CalibrationStatus:
        deadline = time.monotonic() + timeout
        next_send = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                self.send(action, stage)
                next_send = now + 1.0
            for status in self.statuses(0.25):
                if status.request_id != self.request_id:
                    continue
                if status.stage in (STAGE_FAILED, STAGE_CANCELLED, STAGE_COMPLETE):
                    return status
                if status.stage != stage:
                    return status
        raise TimeoutError("controller did not advance to the next calibration stage")

    def wait_next(self, timeout: float = 15.0) -> CalibrationStatus:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for status in self.statuses(0.5):
                if status.request_id == self.request_id:
                    return status
        raise TimeoutError("controller stopped reporting calibration status")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate a wireless controller through vrrecv."
    )
    parser.add_argument("device", nargs="?", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--slot", choices=("right", "left"), required=True)
    parser.add_argument(
        "--discovery-timeout",
        type=float,
        default=15.0,
        help="seconds to wait for the selected slot (default: 15)",
    )
    return parser.parse_args()


def discover(client: CalibrationClient, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    print("Waiting for the selected controller slot...", flush=True)
    while time.monotonic() < deadline:
        client.send(ACTION_QUERY)
        for status in client.statuses(1.0):
            if status.request_id != client.request_id:
                continue
            if status.result != RESULT_UNAVAILABLE:
                return
    raise TimeoutError("selected controller was not discovered")


def run_calibration(client: CalibrationClient) -> int:
    status = client.wait_for_change(STAGE_IDLE, ACTION_START_FULL)
    last_capture = -1
    finish_requested = False
    mag_hint_shown = False

    while True:
        client.current_stage = status.stage
        if status.stage == STAGE_COMPLETE and status.result == RESULT_OK:
            print("Calibration saved successfully.")
            return 0
        if status.stage in (STAGE_FAILED, STAGE_CANCELLED):
            reason = RESULT_NAMES.get(status.result, f"error {status.result}")
            print(f"Calibration failed: {reason}.", file=sys.stderr)
            return 1

        if status.stage in WAIT_PROMPTS:
            print(f"\n{WAIT_PROMPTS[status.stage]}")
            input("Press Enter when ready: ")
            status = client.wait_for_change(status.stage, ACTION_CONFIRM)
            continue

        if status.stage == STAGE_CAPTURE_MAG:
            if status.elapsed_seconds != last_capture:
                print(
                    f"Magnetometer: {status.elapsed_seconds:2d}s, "
                    f"{status.sample_count} samples",
                    flush=True,
                )
                last_capture = status.elapsed_seconds
            if not mag_hint_shown and status.flags & STATUS_CAN_FINISH:
                print("Press Enter to finish early, or keep moving for 30 seconds.")
                mag_hint_shown = True
            readable, _, _ = select.select([sys.stdin], [], [], 0)
            if readable:
                sys.stdin.readline()
                finish_requested = True
            if finish_requested and status.flags & STATUS_CAN_FINISH:
                status = client.wait_for_change(
                    STAGE_CAPTURE_MAG, ACTION_FINISH_MAG, timeout=8.0
                )
            else:
                status = client.wait_next(timeout=3.0)
            continue

        name = CAPTURE_NAMES.get(status.stage)
        if name is not None and status.stage != last_capture:
            print(f"{name}...", flush=True)
            last_capture = status.stage
        status = client.wait_next(timeout=45.0 if status.stage == STAGE_SAVING else 15.0)


def main() -> int:
    args = parse_args()
    try:
        import serial
    except ImportError:
        print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
        return 2

    slot = 0 if args.slot == "right" else 1
    request_id = int(time.monotonic_ns()) & 0xFFFF or 1
    try:
        port = serial.Serial(args.device, args.baud, timeout=0.1)
    except serial.SerialException as exc:
        print(f"Cannot open {args.device}: {exc}", file=sys.stderr)
        return 1

    client = CalibrationClient(port, slot, request_id)
    try:
        discover(client, args.discovery_timeout)
        print(f"Starting full calibration for the {args.slot} controller.")
        return run_calibration(client)
    except KeyboardInterrupt:
        print("\nCancelling calibration...", file=sys.stderr)
        client.send(ACTION_CANCEL, client.current_stage)
        client.statuses(1.0)
        return 130
    except TimeoutError as exc:
        print(f"Calibration timed out: {exc}", file=sys.stderr)
        client.send(ACTION_CANCEL, client.current_stage)
        return 1
    finally:
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
