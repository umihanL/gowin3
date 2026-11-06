import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
import wave

ROOT = Path(__file__).resolve().parents[1]


def module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'tools'/(name+'.py'))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


prepare = module('prepare_sensevoice')
benchmark = module('benchmark_asr')


class AsrToolTests(unittest.TestCase):
    def test_valid_existing_asset_never_downloads(self):
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder)/'model'
            target.write_bytes(b'good')
            def forbidden(*args, **kwargs):
                raise AssertionError('network must not run')
            status = prepare.verified_download('https://test', target, 4,
                hashlib.sha256(b'good').hexdigest(), opener=forbidden)
            self.assertEqual(status, 'verified_existing')

    def test_corrupt_existing_requires_explicit_repair_and_bad_download_preserves_it(self):
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder)/'model'; target.write_bytes(b'old!')
            sha = hashlib.sha256(b'good').hexdigest()
            with self.assertRaises(ValueError):
                prepare.verified_download('https://test', target, 4, sha,
                    opener=lambda *a, **k: io.BytesIO(b'good'))
            with self.assertRaises(ValueError):
                prepare.verified_download('https://test', target, 4, sha, repair=True,
                    opener=lambda *a, **k: io.BytesIO(b'bad!'))
            self.assertEqual(target.read_bytes(), b'old!')
            self.assertEqual(list(Path(folder).glob('*.download')), [])
            prepare.verified_download('https://test', target, 4, sha, repair=True,
                opener=lambda *a, **k: io.BytesIO(b'good'))
            self.assertEqual(target.read_bytes(), b'good')

    def test_oversized_download_and_workspace_escape_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder)/'model'
            with self.assertRaises(ValueError):
                prepare.verified_download('https://test', target, 4, '0'*64,
                    opener=lambda *a, **k: io.BytesIO(b'too large'))
            self.assertFalse(target.exists())
            with self.assertRaises(ValueError):
                prepare.workspace_path(folder, '../escape')

    def test_cer_normalizes_punctuation_and_reports_empty_reference_unknown(self):
        metric = benchmark.character_error_rate('你好，ROS２！', '你 好 ROS2')
        self.assertEqual(metric['cer'], 0)
        self.assertEqual(benchmark.character_error_rate('消防员', '消防人')['distance'], 1)
        self.assertIsNone(benchmark.character_error_rate('，', '你好')['cer'])

    def test_file_format_must_match_real_input_rate(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'sample.wav'
            for rate, channels in ((48000, 1), (16000, 2)):
                with wave.open(str(path), 'wb') as audio:
                    audio.setnchannels(channels); audio.setsampwidth(2); audio.setframerate(rate)
                    audio.writeframes(b'\0\0'*rate*channels)
                with self.assertRaises(ValueError):
                    benchmark.read_pcm(path)
            with wave.open(str(path), 'wb') as audio:
                audio.setnchannels(1); audio.setsampwidth(2); audio.setframerate(16000)
                audio.writeframes(b'\0\0'*16000)
            self.assertEqual(len(benchmark.read_pcm(path)), 32000)

    def test_benchmark_uses_adapter_and_disables_fallback_without_commands(self):
        seen = {}
        class Recognizer:
            def SetWords(self, enabled): seen['words'] = enabled
            def AcceptWaveform(self, raw): seen['bytes'] = seen.get('bytes', 0)+len(raw); return False
            def FinalResult(self): return json.dumps({'text': '你好', 'confidence': None})
        class Backend:
            engine = 'test-only'; confidence_available = False
            def create_recognizer(self, rate): seen['rate'] = rate; return Recognizer()
        def loader(config, workspace): seen['config'] = config; return Backend()
        result = benchmark.run_backend('sensevoice', b'\0\0'*16000, ROOT,
            {'asr_fallback_backend': 'vosk'}, 1, '你好', loader=loader)
        self.assertIsNone(seen['config']['asr_fallback_backend'])
        self.assertEqual(seen['config']['asr_threads'], 1)
        self.assertEqual(seen['bytes'], 32000)
        self.assertEqual(seen['rate'], 16000)
        self.assertFalse(result['command_published'])
        self.assertEqual(result['single_sample_cer']['cer'], 0)


if __name__ == '__main__':
    unittest.main()
