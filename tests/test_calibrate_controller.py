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
            bytes([2, 2, 3, MODULE.STAGE_WAIT_POS_Y, 0x34, 0x12]),
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
            [3, 2, 16, 0, 3, 12, 0xCD, 0xAB, 0x41, 0x01, 0, 0]
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


class AxisCalibrationTests(unittest.TestCase):
    def report_payload(self, request_id=0x1234, flags=5):
        import struct
        return bytes([0,6,2]) + struct.pack('<H6bBB',request_id,2,-1,3,-1,-2,3,flags,0)

    def test_map_report_and_legacy_maps(self):
        import contextlib
        import io
        report=MODULE.decode_map_report(self.report_payload())
        self.assertIsNotNone(report)
        self.assertEqual(report.imu_axis_map,(2,-1,3))
        self.assertEqual(report.mag_axis_map,(-1,-2,3))
        with contextlib.redirect_stdout(io.StringIO()) as output:
            MODULE.print_map_report(report)
        self.assertIn('X<-+Y',output.getvalue())
        self.assertIn('learned',output.getvalue())
        legacy=MODULE.decode_map_report(self.report_payload(flags=1))
        with contextlib.redirect_stdout(io.StringIO()) as output:
            MODULE.print_map_report(legacy)
        self.assertIn('legacy configured',output.getvalue())
        invalid=bytearray(self.report_payload()); invalid[5]=0
        self.assertIsNone(MODULE.decode_map_report(invalid))
        invalid=bytearray(self.report_payload()); invalid[12]=1
        self.assertIsNone(MODULE.decode_map_report(invalid))

    def test_all_corner_stages_and_physical_prompts(self):
        import struct
        for corner in range(8):
            wait=21+2*corner; capture=22+2*corner
            self.assertIn(wait,MODULE.WAIT_PROMPTS)
            self.assertIn(capture,MODULE.CAPTURE_NAMES)
            encoded=MODULE.encode_command(0,MODULE.ACTION_CONFIRM,wait,7)
            self.assertEqual(encoded[9],wait)
            payload=bytes([0,3,2])+struct.pack('<BBBBHH',capture,0,5,1,7,100)+bytes(2)
            self.assertEqual(MODULE.decode_status(payload).stage,capture)
        self.assertIn('right',MODULE.WAIT_PROMPTS[MODULE.STAGE_WAIT_POS_X])
        self.assertNotIn('MPU',MODULE.WAIT_PROMPTS[MODULE.STAGE_WAIT_POS_X])
        self.assertNotEqual(MODULE.STAGE_CAPTURE_CORNER_0,MODULE.STAGE_COMPLETE)

    def test_reject_old_status_version(self):
        payload=bytes([0,3,1,16,0,3,1,7,0,100,0,0,0])
        self.assertIsNone(MODULE.decode_status(payload))

    def test_cache_reports_with_status_in_same_read(self):
        class Port:
            in_waiting=4096
            def read(self,_):
                return MODULE.build_usb_frame(MODULE.FRAME_CALIBRATION_MAP_REPORT,payload)+status
        payload=self.report_payload()
        status=MODULE.build_usb_frame(MODULE.FRAME_CALIBRATION_STATUS,bytes([0,3,2,18,1,5,0,0x34,0x12,0,0,0,0]))
        client=MODULE.CalibrationClient(Port(),0,0x1234)
        found=client.statuses(0.1)
        self.assertEqual(found[0].stage,MODULE.STAGE_COMPLETE)
        self.assertEqual(client.map_reports[0x1234].mag_axis_map,(-1,-2,3))

    def test_fresh_report_query_retries_and_filters_slot(self):
        import struct
        from unittest.mock import patch
        class Port:
            in_waiting=4096
            reads=0
            writes=[]
            def write(self,data): self.writes.append(data)
            def flush(self): pass
            def read(self,_):
                self.reads+=1
                clock[0]+=0.3
                if self.reads<5: return b''
                packet=bytes([6,2])+struct.pack('<H6bBB',8,2,-1,3,-1,-2,3,5,0)
                return (MODULE.build_usb_frame(MODULE.FRAME_CALIBRATION_MAP_REPORT,bytes([1])+packet)+
                        MODULE.build_usb_frame(MODULE.FRAME_CALIBRATION_MAP_REPORT,bytes([0])+packet))
        clock=[0]
        port=Port(); client=MODULE.CalibrationClient(port,0,7)
        with patch.object(MODULE.time,'monotonic',side_effect=lambda:clock[0]):
            report=client.read_map_report(timeout=5)
        self.assertEqual(report.request_id,8)
        self.assertGreaterEqual(len(port.writes),2)
        self.assertEqual(port.writes[0],MODULE.encode_command(0,MODULE.ACTION_QUERY,0,8))


    def test_wizard_runs_every_body_face_and_corner(self):
        import contextlib
        import io
        from unittest.mock import patch
        waits = [1,3,5,7,9,11,13,15] + list(range(21,36,2))
        report = MODULE.decode_map_report(self.report_payload())
        class Client:
            current_stage=0
            next_index=0
            confirmations=[]
            def wait_for_change(self, stage, action):
                if action == MODULE.ACTION_START_FULL:
                    return self.status(1)
                self.confirmations.append(stage)
                return self.status(stage+1)
            def status(self, stage):
                return MODULE.CalibrationStatus(0,stage,1 if stage==18 else 0,5,0,0x1234,100)
            def wait_next(self, **kwargs):
                self.next_index+=1
                if self.next_index < len(waits): return self.status(waits[self.next_index])
                if self.next_index == len(waits): return self.status(17)
                return self.status(18)
            def read_map_report(self): return report
        client=Client()
        with patch('builtins.input', return_value=''), patch.object(MODULE.select,'select',return_value=([],[],[])), \
                contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(MODULE.run_calibration(client),0)
        self.assertEqual(client.confirmations,waits)
        self.assertIn('Saved learned maps',output.getvalue())

    def test_discovery_detects_old_firmware(self):
        import contextlib
        import io
        from unittest.mock import patch
        class Client:
            protocol_mismatch=True
            request_id=7
            def send(self, action): pass
            def statuses(self, timeout): return []
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, 'flash both'):
                MODULE.discover(Client(),1)


if __name__ == "__main__":
    unittest.main()
