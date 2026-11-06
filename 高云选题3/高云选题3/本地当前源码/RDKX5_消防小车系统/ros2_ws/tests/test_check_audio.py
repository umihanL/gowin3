"""Audio acceptance tooling: prevent accidental capture/control and false passes."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

MODULE = Path(__file__).resolve().parents[1] / 'tools/check_audio.py'
SPEC = importlib.util.spec_from_file_location('check_audio', MODULE)
audio = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audio)


class AudioDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.devices = [dict(index=0, name='HDMI', input_channels=0),
            dict(index=1, name='USB Mic (hw:2,0)', input_channels=1),
            dict(index=2, name='default', input_channels=1)]

    def test_default_does_not_capture_or_play(self):
        with patch.object(audio, 'enumerate_devices', return_value={'sounddevice': self.devices}), \
             patch.object(audio, 'record_audio') as record, patch.object(audio, 'playback') as play, \
             contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(audio.main([]), 0)
        record.assert_not_called()
        play.assert_not_called()
        data = json.loads(output.getvalue())
        self.assertFalse(data['command_published'])
        self.assertFalse(data['configuration_modified'])

    def test_capture_requires_explicit_duration_for_asr_and_save(self):
        for args in (['--asr'], ['--save-wav', 'capture.wav'],
                     ['--record-seconds', 'nan'], ['--record-seconds', '16'],
                     ['--record-seconds', '3', '--play-text', '测试'],
                     ['--start-delay', '3'], ['--record-seconds', '3', '--start-delay', '31']):
            with self.subTest(args=args), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                audio.parse_args(args)

    def test_ambiguous_device_and_output_only_device_refused(self):
        for selector in (None, '', 'HDMI', '0', 'missing'):
            with self.subTest(selector=selector), self.assertRaises(ValueError):
                audio.select_input(self.devices, selector)
        duplicate = self.devices + [dict(index=3, name='USB Mic (hw:3,0)', input_channels=1)]
        with self.assertRaises(ValueError): audio.select_input(duplicate, 'USB Mic')
        self.assertIsNone(audio.stable_input_hint(duplicate[1], duplicate))
        self.assertEqual(audio.select_input(self.devices, 'USB Mic')['index'], 1)
        self.assertEqual(audio.stable_input_hint(self.devices[1], self.devices), 'USB Mic')
        self.assertIsNone(audio.stable_input_hint(self.devices[2], self.devices))

    def test_silence_and_clipping_are_reported_without_nan(self):
        silent = audio.pcm_metrics(b'\0\0' * 16000)
        self.assertEqual(silent['duration_seconds'], 1)
        self.assertFalse(silent['signal_present'])
        self.assertIsNone(silent['rms_dbfs'])
        clipped = audio.pcm_metrics(struct.pack('<hhhh', 32767, -32768, 0, 0))
        self.assertEqual(clipped['clipped_fraction'], .5)
        self.assertEqual(clipped['zero_fraction'], .5)
        self.assertEqual(clipped['peak_dbfs'], 0)
        json.dumps(silent, allow_nan=False)

    def test_overflow_rejects_asr_instead_of_accepting_broken_audio(self):
        with patch.object(audio, 'enumerate_devices', return_value={'sounddevice': self.devices}), \
             patch.object(audio, 'record_audio', return_value=(b'\0\0' * 16000, 1, {})), \
             patch.object(audio, 'asr_check') as asr, contextlib.redirect_stdout(io.StringIO()) as output:
            code = audio.main(['--record-seconds', '1', '--input-device', 'USB Mic', '--asr'])
        self.assertEqual(code, 1)
        asr.assert_not_called()
        self.assertFalse(json.loads(output.getvalue())['capture']['usable_for_asr'])

    def test_explicit_wav_never_overwrites_existing_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'evidence.wav'
            path.write_bytes(b'previous capture')
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                audio.parse_args(['--record-seconds', '1', '--save-wav', str(path)])
            self.assertEqual(path.read_bytes(), b'previous capture')

    def test_tool_does_not_import_or_publish_ros_or_parse_commands(self):
        import ast
        tree = ast.parse(MODULE.read_text(encoding='utf-8'))
        imports = [node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)]
        imports += [name.name for node in ast.walk(tree) if isinstance(node, ast.Import) for name in node.names]
        self.assertFalse(any(name and name.startswith(('rclpy', 'std_msgs', 'rescue_vision.voice',
            'rescue_vision.task_', 'rescue_vision.local_actuator')) for name in imports))


if __name__ == '__main__':
    unittest.main()
