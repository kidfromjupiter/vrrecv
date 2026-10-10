import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).parents[1]))
import calibrate_controller as MODULE


class CalibrationToolTests(unittest.TestCase):
    def test_command_frame_round_trip(self):
        frame = MODULE.encode_command(
            1, MODULE.ACTION_CONFIRM, MODULE.STAGE_WAIT_POS_Y, 0x1234
        )
        parser = MODULE.FrameParser()
        self.assertEqual(parser.feed(frame[:7]), [])
        decoded = parser.feed(frame[7:])
        self.assertEqual(len(decoded), 1)
        frame_type, payload = decoded[0]
        self.assertEqual(frame_type, MODULE.FRAME_CALIBRATION_COMMAND)
        self.assertEqual(payload[0], 1)
        self.assertEqual(
            payload[1:7],
            bytes([2, 1, 3, MODULE.STAGE_WAIT_POS_Y, 0x34, 0x12]),
        )

    def test_parser_recovers_after_bad_crc_and_noise(self):
        bad = bytearray(MODULE.build_usb_frame(0, bytes(24)))
        bad[29] ^= 1
        good = MODULE.build_usb_frame(1, bytes(24))
        parser = MODULE.FrameParser()
        self.assertEqual(parser.feed(b"noise" + bad + good), [(1, bytes(24))])
        self.assertEqual(parser.crc_errors, 1)

    def test_decode_calibration_status(self):
        radio_status = bytes(
            [3, 1, 16, 0, 3, 12, 0xCD, 0xAB, 0x41, 0x01, 0, 0]
        )
        status = MODULE.decode_status(bytes([0]) + radio_status)
        self.assertIsNotNone(status)
        assert status is not None
        self.assertEqual(status.slot, 0)
        self.assertEqual(status.stage, MODULE.STAGE_CAPTURE_MAG)
        self.assertEqual(status.flags, 3)
        self.assertEqual(status.elapsed_seconds, 12)
        self.assertEqual(status.request_id, 0xABCD)
        self.assertEqual(status.sample_count, 321)


if __name__ == "__main__":
    unittest.main()
