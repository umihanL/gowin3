"""The diagnostic must not open a port by default or transmit actuator commands."""
import contextlib
import importlib.util
import io
from pathlib import Path
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('check_pd42',
    Path(__file__).resolve().parents[1] / 'tools/check_pd42.py')
check = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(check)


class PD42DiagnosticTests(unittest.TestCase):
    def test_no_flag_does_not_open_serial(self):
        with patch.object(check, 'PD42SerialTransport') as connection, \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(check.main([]), 0)
        connection.assert_not_called()

    def test_port_requires_explicit_read_only_and_unicast(self):
        for args in (['--port', '/dev/ttyS1'], ['--read-only'], ['--address', '0']):
            with self.subTest(args=args), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                check.main(args)

    def test_readout_only_contains_read_commands(self):
        from types import SimpleNamespace
        requests = []
        class ReadConnection:
            def __init__(self, *args, **kwargs): pass
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def exchange(self, packet):
                requests.append(packet)
                return SimpleNamespace(value=0, data=b'\x00', ok=True)
        with patch.object(check, 'PD42SerialTransport', ReadConnection):
            values = check.read_feedback('/dev/ttyS1', 1, 115200)
        self.assertEqual(len(values), 9)
        self.assertTrue(all(packet[2] < 0xF0 for packet in requests))
        self.assertEqual(values['READ_ENABLED']['raw_data_hex'], '00')


if __name__ == '__main__':
    unittest.main()
