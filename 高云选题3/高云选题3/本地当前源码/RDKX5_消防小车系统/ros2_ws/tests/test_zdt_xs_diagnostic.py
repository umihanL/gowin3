"""Run the real read-only CLI against captured-frame fixtures, never a UART."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision.zdt_xs_protocol import parse_response, validate_request, READ_COMMANDS


class CapturedReadOnlyLink:
    def __init__(self, firmware=206, config_error=False, options_hex='00 06'):
        self.firmware = firmware
        self.config_error = config_error
        self.options_hex = options_hex
        self.writes = []
        self.closed = False

    def __enter__(self): return self
    def __exit__(self, *_exc): self.closed = True

    def exchange(self, request):
        address, command = validate_request(request)
        if command not in READ_COMMANDS:
            raise AssertionError('read-only diagnostic attempted control')
        self.writes.append(request)
        frames = {
            0x1F: b'\x01\x1f' + self.firmware.to_bytes(2, 'big') + bytes.fromhex('23 14 6b'),
            0x42: bytes.fromhex('01 42 21 15 19 02 02 02 00 10 01 00 04 b0 0b b8 13 88 05 07 01 00 01 01 00 08 08 98 07 d0 00 08 6b'),
            0x1A: bytes.fromhex('01 1a ' + self.options_hex + ' 6b'),
            0x24: bytes.fromhex('01 24 2c 72 6b'),
            0x36: bytes.fromhex('01 36 00 00 00 00 00 6b'),
            0x33: bytes.fromhex('01 33 00 00 01 00 00 6b'),
            0x34: bytes.fromhex('01 34 01 00 00 80 00 6b'),
            0x35: bytes.fromhex('01 35 00 00 00 6b'),
            0x37: bytes.fromhex('01 37 00 00 00 00 00 6b'),
            0x3A: bytes.fromhex('01 3a 03 6b'),
        }
        if self.config_error:
            frames[0x42] = bytes.fromhex('01 42 ee 6b')
        return parse_response(frames[command], address, command).require_success()


class ZDTDiagnosticTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('check_zdt_xs_test_cli', ROOT / 'tools/check_zdt_xs.py')
        self.cli = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.cli)

    def run_cli(self, link):
        output = io.StringIO()
        with patch.object(self.cli, 'ZDTSerialTransport', return_value=link), contextlib.redirect_stdout(output):
            rc = self.cli.main(['--read-only', '--port', '/dev/fakeZDT'])
        return rc, json.loads(output.getvalue())

    def test_real_capture_profile_reports_verified_parameters_but_sends_only_reads(self):
        link = CapturedReadOnlyLink()
        rc, report = self.run_cli(link)
        self.assertEqual(rc, 0); self.assertIsNone(report['error'])
        self.assertTrue(report['motion_configuration_valid'])
        self.assertFalse(report['readings']['READ_OPTIONS']['decoded']['layout_verified'])
        self.assertTrue(report['normalized_options']['layout_verified'])
        self.assertEqual(report['normalized_options']['raw_bytes_hex'], '00 06')
        self.assertEqual(report['motion_parameters']['microsteps'], 16)
        self.assertFalse(report['motion_parameters']['speed_div10'])
        self.assertEqual(report['target_position_deg'], 360.0)
        self.assertEqual(report['realtime_target_position_deg'], -180.0)
        self.assertEqual(report['position_error_deg'], 0.0)
        self.assertFalse(report['motion_commands_sent']); self.assertFalse(report['enable_commands_sent'])
        self.assertFalse(report['configuration_modified']); self.assertTrue(link.closed)
        self.assertEqual(len(link.writes), 10)
        self.assertTrue(all(r[1] in READ_COMMANDS for r in link.writes))

    def test_unknown_firmware_keeps_successful_communication_read_only(self):
        link = CapturedReadOnlyLink(firmware=207)
        rc, report = self.run_cli(link)
        self.assertEqual(rc, 0); self.assertIsNone(report['error'])
        self.assertFalse(report['motion_configuration_valid'])
        self.assertEqual(report['normalized_options']['layout_verification_reason'],
                         'extended_options_unverified_device_version')
        self.assertEqual(len(link.writes), 10); self.assertTrue(link.closed)

    def test_actual_0406_readback_reports_bit10_and_preserves_all_feedback(self):
        link = CapturedReadOnlyLink(options_hex='04 06')
        rc, report = self.run_cli(link)
        self.assertEqual(rc, 0); self.assertIsNone(report['error'])
        self.assertFalse(report['motion_configuration_valid'])
        self.assertEqual(report['normalized_options']['undocumented_bit_indices'], [10])
        self.assertIn('0x0406', report['motion_configuration_reason'])
        self.assertIn('bit(s) 10', report['motion_configuration_reason'])
        self.assertIn('READ_STATUS', report['readings'])
        self.assertTrue(report['readings']['READ_STATUS']['decoded']['enabled'])
        self.assertFalse(report['motion_commands_sent']); self.assertFalse(report['enable_commands_sent'])
        self.assertFalse(report['configuration_modified']); self.assertTrue(link.closed)
        self.assertEqual(len(link.writes), 10)
        self.assertTrue(all(r[1] in READ_COMMANDS for r in link.writes))

    def test_config_rejection_reports_error_without_retry_or_control(self):
        link = CapturedReadOnlyLink(config_error=True)
        rc, report = self.run_cli(link)
        self.assertEqual(rc, 1); self.assertIn('invalid command format', report['error'])
        self.assertEqual([r[1] for r in link.writes], [0x1F, 0x42])
        self.assertTrue(link.closed)


if __name__ == '__main__':
    unittest.main(verbosity=2)
