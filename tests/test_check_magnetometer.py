import contextlib
import io
import math
import struct
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1]))
import check_magnetometer as M
from calibrate_controller import FrameParser, build_usb_frame, STAGE_CAPTURE_MAG, CalibrationStatus


def sample(sequence=0, timestamp=123, raw=50.0, calibrated=45.0, flags=M.FRESH | M.CALIBRATED):
    return M.MagSample(0, 12, sequence, flags, timestamp, raw, calibrated)


def frame(sequence=0, slot=0, request=12):
    packet = struct.pack('<BBHHHIff', 4, 1, request, sequence, M.FRESH | M.CALIBRATED,
                         (sequence + 1) * 100000, 50.0 + sequence % 2, 45.0)
    return build_usb_frame(M.FRAME_MAG_SAMPLE, bytes([slot]) + packet)


class FakePort:
    def __init__(self, clock, emit=True):
        self.clock = clock
        self.emit = emit
        self.writes = []
        self.sequence = 0
        self.in_waiting = 0

    def write(self, data):
        self.writes.append(data)

    def flush(self):
        pass

    def read(self, _):
        self.clock.now += 0.1
        self.sequence += 1
        if not self.emit:
            return b''
        return (build_usb_frame(0, bytes(24)) + frame(self.sequence, slot=1) +
                frame(self.sequence, request=99) + frame(self.sequence))


class Clock:
    now = 0.0
    def __call__(self):
        return self.now


class MagnetometerTests(unittest.TestCase):
    def test_encoding_and_fragmented_noisy_frames(self):
        command = M.encode_command(1, M.ACTION_START, 12, 30)
        parser = FrameParser()
        self.assertEqual(parser.feed(command[:4]), [])
        decoded = parser.feed(command[4:])
        self.assertEqual(decoded, [(M.FRAME_MAG_COMMAND, b'\x01\x05\x01\x01\x00\x0c\x00\x1e\x00')])
        bad = bytearray(frame()); bad[29] ^= 1
        decoded = parser.feed(b'noise' + bad + frame())
        self.assertEqual(len(decoded), 1)
        self.assertEqual(M.decode_sample(decoded[0][1]), sample(timestamp=100000, calibrated=45))
        self.assertEqual(parser.crc_errors, 1)
        self.assertIsNone(M.decode_sample(b''))
        self.assertIsNone(M.decode_sample(bytes([3]) + decoded[0][1][1:]))

    def test_statistics(self):
        stats = M.magnitude_stats([40, 50, 60])
        self.assertEqual(stats['mean'], 50)
        self.assertEqual(stats['span'], 20)
        self.assertEqual(stats['span_pct'], 40)
        self.assertAlmostEqual(stats['stddev'], math.sqrt(200 / 3))
        self.assertEqual(M.magnitude_stats([50] * 20)['cv'], 0)
        self.assertTrue(math.isnan(M.magnitude_stats([0] * 20)['cv']))

    def test_exclusions_range_and_wrap(self):
        measurement = M.Measurement()
        measurement.add(sample(65535, 0xFFFFFFF0))
        measurement.add(sample(0, 100))
        measurement.add(sample(0, 100))
        measurement.add(sample(2, 200, flags=M.CALIBRATED | M.OVERFLOW))
        measurement.add(sample(3, 300, raw=math.nan))
        measurement.add(sample(4, 400, calibrated=120, flags=M.FRESH | M.CALIBRATED | M.OUT_OF_RANGE))
        measurement.add(sample(5, 400))
        self.assertEqual(measurement.raw, [50, 50, 50])
        self.assertEqual(measurement.calibrated, [45, 45, 120])
        self.assertEqual(measurement.gaps, 1)
        self.assertEqual(measurement.out_of_range, 1)
        self.assertEqual(measurement.exclusions['duplicate/out-of-order'], 1)
        self.assertEqual(measurement.exclusions['stale-timestamp'], 1)
        self.assertEqual(measurement.exclusions['nonfinite'], 1)
        self.assertEqual(measurement.exclusions['total'], 4)

    def test_sensor_error_and_uncalibrated(self):
        measurement = M.Measurement()
        measurement.add(sample(flags=M.FRESH | M.SENSOR_ERROR))
        self.assertEqual(len(measurement.raw), 0)
        self.assertEqual(measurement.exclusions['sensor-error'], 1)
        self.assertEqual(measurement.exclusions['uncalibrated'], 1)

    def test_report_and_zero_variance(self):
        measurement = M.Measurement(raw=[50] * 20, calibrated=[45] * 20)
        with contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertTrue(M.print_report(measurement, FrameParser()))
        self.assertIn('unavailable', output.getvalue())
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertFalse(M.print_report(M.Measurement(), FrameParser()))

    def test_timed_check_filters_other_traffic_and_stops(self):
        clock = Clock(); port = FakePort(clock)
        with patch.object(M.time, 'monotonic', clock), contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(M.run_measurement(port, 0, 5, 12), 0)
        self.assertIn('Calibrated', output.getvalue())
        self.assertEqual(port.writes[0], M.encode_command(0, M.ACTION_START, 12, 5))
        self.assertEqual(port.writes[-1], M.encode_command(0, M.ACTION_STOP, 12))

    def test_timeout_retries_and_stops(self):
        clock = Clock(); port = FakePort(clock, emit=False)
        with patch.object(M.time, 'monotonic', clock), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(TimeoutError):
                M.run_measurement(port, 0, 30, 12)
        self.assertGreater(len(port.writes), 2)
        self.assertEqual(port.writes[-1], M.encode_command(0, M.ACTION_STOP, 12))

    def test_interruption_stops(self):
        port = FakePort(Clock())
        with patch.object(port, 'read', side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                M.run_measurement(port, 0, 30, 12)
        self.assertEqual(port.writes[-1], M.encode_command(0, M.ACTION_STOP, 12))

    def test_discovery_checks_saved_calibration(self):
        ready = CalibrationStatus(0, M.STAGE_IDLE, 0, 1, 0, 12, 0)
        missing = CalibrationStatus(0, M.STAGE_IDLE, 0, 0, 0, 12, 0)
        busy = CalibrationStatus(0, STAGE_CAPTURE_MAG, 0, 1, 0, 12, 0)
        with patch.object(M.CalibrationClient, 'send'), contextlib.redirect_stdout(io.StringIO()):
            with patch.object(M.CalibrationClient, 'statuses', return_value=[ready]):
                M.require_saved_calibration(object(), 0, 12)
            transport = CalibrationStatus(0, 19, 13, 0, 0, 12, 0)
            with patch.object(M.CalibrationClient, 'statuses', return_value=[transport]):
                with self.assertRaisesRegex(RuntimeError, 'transport failed'):
                    M.require_saved_calibration(object(), 0, 12)
            for status in [missing, busy]:
                with patch.object(M.CalibrationClient, 'statuses', return_value=[status]):
                    with self.assertRaises(RuntimeError):
                        M.require_saved_calibration(object(), 0, 12)

    def test_unavailable_discovery_times_out(self):
        clock = Clock()
        def unavailable(_):
            clock.now += 1
            return [CalibrationStatus(0, 19, M.RESULT_UNAVAILABLE, 0, 0, 12, 0)]
        with patch.object(M.time, 'monotonic', clock), patch.object(M.CalibrationClient, 'send'), \
                patch.object(M.CalibrationClient, 'statuses', side_effect=unavailable), \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(TimeoutError):
                M.require_saved_calibration(object(), 0, 12, timeout=2)


if __name__ == '__main__':
    unittest.main()
