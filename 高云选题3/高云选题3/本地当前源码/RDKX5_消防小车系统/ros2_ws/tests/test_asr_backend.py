"""Real adapter code with fake native engines; no audio, downloads or hardware."""
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision.asr_backend import SenseVoiceBackend, load_asr_backend


class NativeRecognizer:
    def __init__(self, text='分析一下当前场景'):
        self.text = text
        self.streams = []
        self.decode_calls = 0

    def create_stream(self):
        stream = types.SimpleNamespace(result=types.SimpleNamespace(text=self.text))
        def accept(rate, samples):
            stream.rate = rate
            stream.samples = samples.copy()
        stream.accept_waveform = accept
        self.streams.append(stream)
        return stream

    def decode_stream(self, stream):
        self.decode_calls += 1


class FakeVAD:
    def __init__(self, intervals):
        self.queue = []
        self.intervals = intervals
        self.chunks = []
        self.flush_calls = 0

    def accept_waveform(self, samples):
        self.chunks.append(samples.copy())

    def empty(self):
        return not self.queue

    @property
    def front(self):
        return self.queue[0]

    def pop(self):
        self.queue.pop(0)

    def flush(self):
        self.flush_calls += 1
        self.queue.extend(types.SimpleNamespace(start=start, samples=np.zeros(end-start))
                          for start, end in self.intervals)


class SenseVoiceSessionTests(unittest.TestCase):
    def backend(self, intervals=((0, 16000),), native=None):
        self.native = native or NativeRecognizer()
        self.vads = []
        def factory():
            vad = FakeVAD(intervals); self.vads.append(vad); return vad
        return SenseVoiceBackend('/fake/model', native_recognizer=self.native, vad_factory=factory)

    def test_pcm_scale_rate_full_capture_and_no_fabricated_confidence(self):
        recognizer = self.backend().create_recognizer(16000)
        raw = np.array([-32768, -1000, 0, 1000, 32767] * 100, dtype='<i2').tobytes()
        self.assertFalse(recognizer.AcceptWaveform(raw[:200]))
        self.assertFalse(recognizer.AcceptWaveform(raw[200:]))
        self.assertEqual(json.loads(recognizer.Result())['text'], '')
        result = json.loads(recognizer.FinalResult())
        self.assertEqual(result['text'], '分析一下当前场景')
        self.assertIsNone(result['confidence'])
        self.assertFalse(result['confidence_available'])
        self.assertEqual(result['result'], [])
        self.assertTrue(result['vad_speech_detected'])
        self.assertEqual(result['speech_guard'], 'silero_vad')
        self.assertEqual(recognizer.pcm_bytes, raw)
        self.assertEqual(self.native.streams[0].rate, 16000)
        self.assertEqual(self.native.streams[0].samples.dtype, np.float32)
        np.testing.assert_array_equal(self.native.streams[0].samples, np.frombuffer(raw, '<i2') / np.float32(32768.))
        self.assertEqual(result['audio_s'], 500/16000)
        self.assertGreaterEqual(result['decode_s'], 0.)
        self.assertGreaterEqual(result['vad_s'], 0.)
        self.assertEqual(len(self.vads[0].chunks[-1]), 512)

    def test_no_vad_speech_never_decodes_noise_or_silence(self):
        for raw in (b'\0\0' * 16000, np.full(16000, 20000, dtype='<i2').tobytes()):
            recognizer = self.backend(intervals=()).create_recognizer()
            recognizer.AcceptWaveform(raw)
            result = json.loads(recognizer.FinalResult())
            self.assertEqual(result['text'], '')
            self.assertEqual(result['reason'], 'no_speech')
            self.assertFalse(result['vad_speech_detected'])
            self.assertEqual(result['speech_s'], 0.)
            self.assertEqual(self.native.decode_calls, 0)
            self.assertEqual(self.vads[0].flush_calls, 1)

    def test_empty_audio_skips_decoder_and_vad(self):
        result = json.loads(self.backend().create_recognizer().FinalResult())
        self.assertEqual(result['reason'], 'no_audio')
        self.assertEqual(self.native.decode_calls, 0)
        self.assertEqual(self.vads, [])

    def test_vad_intervals_clip_merge_and_trim_without_duplicate_samples(self):
        # Overlap and out-of-range VAD edges must not duplicate or invent audio.
        recognizer = self.backend(intervals=((4000, 7000), (6000, 8000), (12000, 18000))).create_recognizer()
        raw = np.arange(16000, dtype='<i2').tobytes()
        recognizer.AcceptWaveform(raw)
        result = json.loads(recognizer.FinalResult())
        self.assertEqual(result['segments'], [dict(start_s=.25, end_s=.5), dict(start_s=.75, end_s=1.)])
        self.assertEqual(result['speech_s'], .5)
        # 150 ms pads touch and merge to [1600, 16000], preserving original samples.
        np.testing.assert_array_equal(self.native.streams[0].samples,
            np.arange(1600, 16000, dtype=np.float32)/32768.)
        self.assertLessEqual(result['decode_audio_s'], result['audio_s'])

    def test_final_is_cached_and_input_is_bounded_without_truncation(self):
        recognizer = self.backend().create_recognizer()
        raw = bytearray(b'\x01\0' * (16000*30))
        recognizer.AcceptWaveform(raw); raw[:] = b'\0' * len(raw)
        self.assertNotEqual(recognizer.pcm_bytes[:2], bytes(raw[:2]))
        with self.assertRaisesRegex(ValueError, 'exceeds 30'): recognizer.AcceptWaveform(b'\0\0')
        first = recognizer.FinalResult()
        self.assertEqual(recognizer.FinalResult(), first)
        self.assertEqual(recognizer.Result(), first)
        self.assertEqual(self.native.decode_calls, 1)
        with self.assertRaisesRegex(RuntimeError, 'finalized'): recognizer.AcceptWaveform(b'\0\0')

    def test_failure_is_cached_and_never_retried_with_missing_audio(self):
        native = NativeRecognizer()
        def fail(stream): native.decode_calls += 1; raise RuntimeError('native_decode_failed')
        native.decode_stream = fail
        recognizer = self.backend(native=native).create_recognizer()
        recognizer.AcceptWaveform(b'\x01\0' * 16000)
        for _ in range(2):
            with self.assertRaisesRegex(RuntimeError, 'native_decode_failed'): recognizer.FinalResult()
        self.assertEqual(native.decode_calls, 1)
        self.assertEqual(len(recognizer.pcm_bytes), 32000)

    def test_invalid_format_and_rate_fail_before_decode(self):
        backend = self.backend()
        for rate in (48000, True, 16000., 0):
            with self.assertRaises(ValueError): backend.create_recognizer(rate)
        recognizer = backend.create_recognizer()
        with self.assertRaises(ValueError): recognizer.AcceptWaveform(b'\x01')
        with self.assertRaises(TypeError): recognizer.AcceptWaveform([1, 2])
        self.assertEqual(self.native.decode_calls, 0)


class BackendLoadingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.model = self.root / 'sense'; self.model.mkdir()
        for name in ('model.int8.onnx', 'tokens.txt'): (self.model/name).write_bytes(b'fake')
        self.vad = self.root / 'silero_vad.onnx'; self.vad.write_bytes(b'fake')
        self.calls = []
        self.native = NativeRecognizer()
        def load(**kwargs): self.calls.append(kwargs); return self.native
        self.configs = []
        def vadconfig():
            config = types.SimpleNamespace(silero_vad=types.SimpleNamespace())
            self.configs.append(config); return config
        self.sherpa = types.SimpleNamespace(OfflineRecognizer=types.SimpleNamespace(from_sense_voice=load),
            VadModelConfig=vadconfig, VoiceActivityDetector=lambda config, **kw: FakeVAD(((0, 16000),)))
        self.modules = patch.dict(sys.modules, {'sherpa_onnx': self.sherpa})
        self.modules.start(); self.addCleanup(self.modules.stop)

    def config(self):
        return dict(asr_backend='sensevoice', sensevoice_model_dir='sense',
                    silero_vad_model_path='silero_vad.onnx', asr_threads=2)

    def test_native_factory_settings_and_model_cached_across_sessions(self):
        backend = load_asr_backend(self.config(), self.root)
        for _ in range(2):
            session = backend.create_recognizer(); session.AcceptWaveform(b'\x01\0' * 16000); session.FinalResult()
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(self.calls[0], dict(model=str(self.model/'model.int8.onnx'),
            tokens=str(self.model/'tokens.txt'), language='zh', use_itn=True, num_threads=2, provider='cpu'))
        self.assertEqual(self.configs[0].sample_rate, 16000)
        self.assertEqual(self.configs[0].silero_vad.window_size, 512)
        self.assertEqual(self.configs[0].silero_vad.model, str(self.vad))
        self.assertFalse(backend.confidence_available)

    def test_missing_vad_is_explicit_failure_not_energy_substitution(self):
        self.vad.unlink()
        with self.assertRaisesRegex(FileNotFoundError, 'Mandatory Silero VAD'):
            load_asr_backend(self.config(), self.root)
        self.assertEqual(self.calls, [])

    def test_vosk_fallback_requires_explicit_configuration_and_reports_reason(self):
        legacy = self.root/'vosk/am'; legacy.mkdir(parents=True); (legacy/'final.mdl').write_bytes(b'fake')
        vosk = types.SimpleNamespace(Model=lambda path: path, KaldiRecognizer=lambda model, rate: (model, rate))
        self.vad.unlink()
        config = dict(self.config(), model_dir='vosk')
        with patch.dict(sys.modules, {'vosk': vosk}):
            with self.assertRaises(FileNotFoundError): load_asr_backend(config, self.root)
            backend = load_asr_backend(dict(config, asr_fallback_backend='vosk'), self.root)
            self.assertEqual(backend.asr_backend, 'vosk')
            self.assertTrue(backend.confidence_available)
            self.assertEqual(backend.fallback_from, 'sensevoice')
            self.assertIn('Mandatory Silero VAD', backend.fallback_reason)
            self.assertEqual(backend.create_recognizer()[1], 16000)
            original = load_asr_backend(dict(model_dir='vosk'), self.root)
            self.assertEqual(original.asr_backend, 'vosk')
            self.assertIsNone(original.fallback_from)

    def test_invalid_threads_or_backend_do_not_silently_fallback(self):
        for changes in ({'asr_threads': True}, {'asr_threads': 0}, {'asr_threads': 9},
                        {'asr_backend': 'unknown'}, {'asr_fallback_backend': 'cloud'}):
            with self.assertRaises(ValueError):
                load_asr_backend(dict(self.config(), **changes), self.root)


if __name__ == '__main__': unittest.main()
