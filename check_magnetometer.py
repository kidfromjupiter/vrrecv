#!/usr/bin/env python3
"""Measure saved magnetometer calibration through the vrrecv native USB port."""

from __future__ import annotations

import argparse
import math
import statistics
import struct
import sys
import time
from collections import Counter
from dataclasses import dataclass, field

from calibrate_controller import (
    ACTION_QUERY, CalibrationClient, FrameParser, RESULT_NAMES, RESULT_UNAVAILABLE,
    STAGE_IDLE, build_usb_frame,
)

FRAME_MAG_COMMAND = 0x82
FRAME_MAG_SAMPLE = 0x83
ACTION_START = 1
ACTION_STOP = 2
FRESH = 0x01
SENSOR_ERROR = 0x02
OVERFLOW = 0x04
OUT_OF_RANGE = 0x08
CALIBRATED = 0x10


@dataclass(frozen=True)
class MagSample:
    slot: int
    request_id: int
    sequence: int
    flags: int
    timestamp_us: int
    raw_ut: float
    calibrated_ut: float


def encode_command(slot: int, action: int, request_id: int, duration: int = 0) -> bytes:
    packet = struct.pack("<BBBBHH", 5, 1, action, 0, request_id, duration)
    return build_usb_frame(FRAME_MAG_COMMAND, bytes([slot]) + packet)


def decode_sample(payload: bytes) -> MagSample | None:
    if len(payload) != 21 or payload[0] not in (0, 1) or payload[1:3] != b"\x04\x01":
        return None
    request, sequence, flags, timestamp, raw, corrected = struct.unpack_from("<HHHIff", payload, 3)
    if request == 0 or flags & ~0x1F:
        return None
    return MagSample(payload[0], request, sequence, flags, timestamp, raw, corrected)


def magnitude_stats(values: list[float]) -> dict[str, float]:
    mean = statistics.fmean(values)
    stddev = statistics.pstdev(values, mu=mean)
    span = max(values) - min(values)
    return {
        "min": min(values), "max": max(values), "mean": mean, "stddev": stddev,
        "span": span,
        "cv": 100 * stddev / mean if mean > 0 else math.nan,
        "span_pct": 100 * span / mean if mean > 0 else math.nan,
    }


@dataclass
class Measurement:
    raw: list[float] = field(default_factory=list)
    calibrated: list[float] = field(default_factory=list)
    exclusions: Counter = field(default_factory=Counter)
    packets: int = 0
    gaps: int = 0
    out_of_range: int = 0
    previous_sequence: int | None = None
    previous_timestamp: int | None = None

    def add(self, sample: MagSample) -> None:
        self.packets += 1
        if self.previous_sequence is not None:
            step = (sample.sequence - self.previous_sequence) & 0xFFFF
            if step == 0 or step >= 0x8000:
                self.exclusions["duplicate/out-of-order"] += 1
                self.exclusions["total"] += 1
                return
            self.gaps += step - 1
        self.previous_sequence = sample.sequence
        reasons = []
        if not sample.flags & FRESH:
            reasons.append("stale/not-ready")
        if sample.flags & SENSOR_ERROR:
            reasons.append("sensor-error")
        if sample.flags & OVERFLOW:
            reasons.append("overflow")
        if not sample.flags & CALIBRATED:
            reasons.append("uncalibrated")
        if not (math.isfinite(sample.raw_ut) and math.isfinite(sample.calibrated_ut)):
            reasons.append("nonfinite")
        elif sample.raw_ut < 0 or sample.calibrated_ut < 0:
            reasons.append("negative-magnitude")
        if sample.flags & FRESH and self.previous_timestamp is not None:
            step = (sample.timestamp_us - self.previous_timestamp) & 0xFFFFFFFF
            if step == 0 or step >= 0x80000000:
                reasons.append("stale-timestamp")
        if sample.flags & FRESH and "stale-timestamp" not in reasons:
            self.previous_timestamp = sample.timestamp_us
        if reasons:
            self.exclusions.update(reasons)
            self.exclusions["total"] += 1
            return
        if sample.flags & OUT_OF_RANGE:
            self.out_of_range += 1
        self.raw.append(sample.raw_ut)
        self.calibrated.append(sample.calibrated_ut)


def print_report(measurement: Measurement, parser: FrameParser) -> bool:
    count = len(measurement.raw)
    print(f"\nUsable pairs: {count}; packets: {measurement.packets}; sequence gaps: {measurement.gaps}")
    print(f"Calibrated out-of-range readings retained: {measurement.out_of_range}")
    excluded = dict(measurement.exclusions)
    print(f"Exclusions (reasons may overlap): {excluded or 'none'}")
    print(f"USB CRC errors: {parser.crc_errors}; invalid bytes: {parser.invalid_bytes}")
    if count < 20:
        print("Insufficient data: at least 20 usable paired readings are required.", file=sys.stderr)
        return False
    raw = magnitude_stats(measurement.raw)
    calibrated = magnitude_stats(measurement.calibrated)
    print(f"{'Metric':<28} {'Raw':>12} {'Calibrated':>12}")
    for key, label in (
        ("min", "Minimum (uT)"), ("max", "Maximum (uT)"), ("mean", "Mean (uT)"),
        ("stddev", "Std deviation (uT)"), ("span", "Peak-to-peak (uT)"),
        ("cv", "Std deviation / mean (%)"), ("span_pct", "Peak-to-peak / mean (%)"),
    ):
        print(f"{label:<28} {raw[key]:12.3f} {calibrated[key]:12.3f}")
    for key, label in (("cv", "stddev/mean"), ("span_pct", "peak-to-peak/mean")):
        if math.isfinite(raw[key]) and raw[key] > 0 and math.isfinite(calibrated[key]):
            improvement = 100 * (1 - calibrated[key] / raw[key])
            print(f"Relative reduction in {label}: {improvement:.1f}% (negative means increased variation).")
        else:
            print(f"Relative reduction in {label}: unavailable (raw variation or mean is zero).")
    print("Lower variation during rotation indicates better magnitude consistency; this does not grade heading accuracy.")
    return True


def read_frames(port: object, parser: FrameParser) -> list[tuple[int, bytes]]:
    return parser.feed(port.read(max(1, min(getattr(port, "in_waiting", 0), 4096))))


def require_saved_calibration(port: object, slot: int, request_id: int, timeout: float = 15) -> None:
    client = CalibrationClient(port, slot, request_id)
    deadline = time.monotonic() + timeout
    print("Waiting for the selected controller's saved calibration...", flush=True)
    while time.monotonic() < deadline:
        client.send(ACTION_QUERY)
        statuses = client.statuses(1)
        if client.protocol_mismatch:
            raise RuntimeError("Old calibration protocol; flash both updated firmwares supporting v2.")
        for status in statuses:
            if status.request_id != request_id or status.result == RESULT_UNAVAILABLE:
                continue
            if status.result != 0:
                raise RuntimeError(RESULT_NAMES.get(status.result, "Controller query failed"))
            if status.stage != STAGE_IDLE:
                raise RuntimeError("Controller is calibrating; wait for it to finish before checking.")
            if not status.flags & 1:
                raise RuntimeError("No saved calibration; run calibrate_controller.py first.")
            return
    raise TimeoutError("Selected slot unavailable. Remove the diagnostic jumper, check power, and flash both updated firmwares supporting calibration v2.")


def run_measurement(port: object, slot: int, duration: int, request_id: int) -> int:
    parser = FrameParser()
    measurement = Measurement()
    started = time.monotonic()
    deadline = started + duration
    next_start = started
    next_progress = started + 1
    last_packet = None
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if last_packet is None and now >= next_start:
                port.write(encode_command(slot, ACTION_START, request_id, duration))
                port.flush()
                next_start = now + 1
            for frame_type, payload in read_frames(port, parser):
                if frame_type != FRAME_MAG_SAMPLE:
                    continue
                sample = decode_sample(payload)
                if sample is None or sample.slot != slot or sample.request_id != request_id:
                    continue
                last_packet = time.monotonic()
                measurement.add(sample)
            now = time.monotonic()
            if now - (started if last_packet is None else last_packet) >= 5:
                raise TimeoutError("No diagnostic packets for five seconds. Flash both updated firmwares and check the selected slot.")
            if now >= next_progress:
                print(f"Rotating: {min(now - started, duration):.0f}/{duration}s; usable pairs={len(measurement.raw)}", flush=True)
                next_progress = now + 1
        if last_packet is None:
            raise TimeoutError("No diagnostic packets received. Flash both updated firmwares.")
        return 0 if print_report(measurement, parser) else 1
    finally:
        try:
            port.write(encode_command(slot, ACTION_STOP, request_id))
            port.flush()
        except Exception as exc:
            print(f"Could not send stop: {exc}; controller streaming expires automatically.", file=sys.stderr)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("device", nargs="?", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--slot", choices=("right", "left"), required=True)
    parser.add_argument("--duration", type=int, choices=range(5, 301), default=30, metavar="5..300")
    return parser.parse_args()


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
        with serial.Serial(args.device, args.baud, timeout=0.1) as port:
            require_saved_calibration(port, slot, request_id)
            print("Rotate through all orientations near one location, away from changing magnetic sources.")
            input(f"Press Enter to start the {args.duration}-second check: ")
            return run_measurement(port, slot, args.duration, request_id)
    except KeyboardInterrupt:
        print("\nCheck interrupted.", file=sys.stderr)
        return 130
    except (TimeoutError, RuntimeError, serial.SerialException, EOFError) as exc:
        print(f"Magnetometer check failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
