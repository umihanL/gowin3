"""Exercise real process lifecycle without ROS, model weights, audio or vehicle I/O."""
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
import struct

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision.voice_tts_process import ProcessNeuralTTS, model_paths, audio_pcm16


def fake_tts_worker(connection, config, workspace):
    if config.get('startup_hangs'):
        time.sleep(60.)
    connection.send(('ready', dict(sample_rate=16000, speakers=5)))
    try:
        while True:
            request, text = connection.recv()
            if text == 'hang':
                time.sleep(60.)
            elif text == 'error':
                connection.send(('error', 'native model failure'))
                break
            connection.send(('audio', request, text.encode(), dict(duration_s=.1)))
    except (EOFError, BrokenPipeError):
        pass
    finally:
        connection.close()


class VoiceTTSProcessTests(unittest.TestCase):
    def make(self, **config):
        backend = ProcessNeuralTTS(config, ROOT, worker_target=fake_tts_worker)
        self.addCleanup(backend.close)
        return backend

    def test_repeated_requests_reuse_cached_worker(self):
        backend = self.make()
        self.assertEqual(backend.synthesize('first')[0], b'first')
        pid = backend._process.pid
        for index in range(5):
            text = 'message ' + str(index)
            self.assertEqual(backend.synthesize(text)[0], text.encode())
            self.assertEqual(backend._process.pid, pid)

    def test_prepare_loads_once_without_generating_and_first_speech_reuses_it(self):
        backend = self.make()
        metadata = backend.prepare()
        self.assertEqual(metadata['sample_rate'], 16000)
        pid = backend._process.pid
        self.assertEqual(backend._request, 0)
        self.assertEqual(backend.prepare(), metadata)
        self.assertEqual(backend._process.pid, pid)
        self.assertEqual(backend._request, 0)
        self.assertEqual(backend.synthesize('first speech')[0], b'first speech')
        self.assertEqual(backend._process.pid, pid)

    def test_prepare_timeout_can_retry_later(self):
        backend = self.make(tts_synthesis_timeout_s=1., startup_hangs=True)
        with self.assertRaises(TimeoutError):
            backend.prepare()
        self.assertIsNone(backend._process)
        backend.config['startup_hangs'] = False
        self.assertEqual(backend.prepare()['sample_rate'], 16000)
        self.assertEqual(backend.synthesize('retry')[0], b'retry')

    def test_close_interrupts_background_prepare(self):
        backend = self.make(startup_hangs=True)
        errors = []
        def run():
            try:
                backend.prepare()
            except Exception as exc:
                errors.append(exc)
        thread = threading.Thread(target=run)
        thread.start(); time.sleep(.1)
        backend.close(); thread.join(2.)
        self.assertFalse(thread.is_alive())
        self.assertIsNone(backend._process)
        self.assertTrue(errors)

    def test_hung_native_synthesis_times_out_and_next_request_recovers(self):
        backend = self.make(tts_synthesis_timeout_s=1.)
        backend.synthesize('prime')
        old = backend._process
        with self.assertRaises(TimeoutError):
            backend.synthesize('hang')
        self.assertFalse(old.is_alive())
        self.assertIsNone(backend._process)
        self.assertEqual(backend.synthesize('recover')[0], b'recover')

    def test_native_error_does_not_wedge_future_speech(self):
        backend = self.make()
        with self.assertRaisesRegex(RuntimeError, 'native model failure'):
            backend.synthesize('error')
        self.assertEqual(backend.synthesize('recover')[0], b'recover')

    def test_close_interrupts_hung_synthesis(self):
        backend = self.make()
        backend.synthesize('prime')
        old = backend._process
        errors = []
        def run():
            try:
                backend.synthesize('hang')
            except Exception as exc:
                errors.append(exc)
        thread = threading.Thread(target=run)
        thread.start(); time.sleep(.1)
        backend.close(); thread.join(2.)
        self.assertFalse(thread.is_alive())
        self.assertFalse(old.is_alive())
        self.assertTrue(errors)
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            backend.synthesize('after close')

    def test_startup_timeout_reclaims_worker(self):
        backend = self.make(tts_synthesis_timeout_s=1., startup_hangs=True)
        with self.assertRaises(TimeoutError):
            backend.synthesize('start')
        self.assertIsNone(backend._process)

    def test_empty_and_excessive_text_rejected_before_start(self):
        backend = self.make()
        for value in ('', ' ', 'a'*351, None):
            with self.assertRaises(ValueError):
                backend.synthesize(value)
        self.assertIsNone(backend._process)

    def test_model_requires_normalization_assets(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for name in ('model.onnx', 'tokens.txt', 'lexicon.txt'):
                (root/name).write_text('fixture')
            config = {'tts_model_dir': str(root)}
            with self.assertRaisesRegex(FileNotFoundError, 'normalization'):
                model_paths(config, root)
            for name in ('phone.fst', 'date.fst', 'number.fst'):
                (root/name).write_text('fixture')
            self.assertEqual(Path(model_paths(config, root)['model']), (root/'model.onnx').resolve())

    def test_matcha_requires_both_acoustic_model_and_vocoder(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for name in ('model-steps-3.onnx', 'tokens.txt', 'lexicon.txt', 'phone.fst', 'date.fst', 'number.fst'):
                (root/name).write_text('fixture')
            config = {'tts_backend': 'sherpa_matcha', 'tts_model_dir': str(root)}
            with self.assertRaisesRegex(FileNotFoundError, 'vocoder'):
                model_paths(config, root)
            (root/'vocos-22khz-univ.onnx').write_text('fixture')
            paths = model_paths(config, root)
            self.assertIn('acoustic_model', paths)
            self.assertIn('vocoder', paths)
            self.assertNotIn('model', paths)


class VoiceTTSVolumeTests(unittest.TestCase):
    def pcm(self, waveform, **config):
        pcm, metadata = audio_pcm16(waveform, config)
        return struct.unpack('<' + 'h'*(len(pcm)//2), pcm), metadata

    def test_default_preserves_original_waveform(self):
        pcm, metadata = self.pcm([0., .25, -.25])
        self.assertEqual(pcm, (0, 8191, -8191))
        self.assertEqual(metadata['audio_gain'], 1.)

    def test_quiet_speech_reaches_target_without_clipping(self):
        pcm, metadata = self.pcm([0., .247, -.247, .058], tts_normalize_peak=.85, tts_max_gain=4.)
        self.assertAlmostEqual(metadata['audio_gain'], .85/.247, places=5)
        self.assertAlmostEqual(metadata['output_peak'], .85, places=6)
        self.assertLess(max(abs(v) for v in pcm), 32767)
        self.assertAlmostEqual(pcm[1]/32767, .85, places=4)

    def test_gain_is_bounded_for_very_quiet_speech(self):
        pcm, metadata = self.pcm([.01, -.01], tts_normalize_peak=.85, tts_max_gain=4.)
        self.assertEqual(metadata['audio_gain'], 4.)
        self.assertAlmostEqual(metadata['output_peak'], .04, places=6)

    def test_silence_and_near_silence_are_never_amplified(self):
        for waveform in ([0., 0.], [1.e-5, -1.e-5]):
            pcm, metadata = self.pcm(waveform, tts_normalize_peak=.85, tts_max_gain=4.)
            self.assertEqual(metadata['audio_gain'], 1.)
            self.assertEqual(pcm, (0, 0))

    def test_hot_input_is_attenuated_to_target(self):
        pcm, metadata = self.pcm([1.2, -1.2], tts_normalize_peak=.85, tts_max_gain=4.)
        self.assertLess(metadata['audio_gain'], 1.)
        self.assertAlmostEqual(metadata['output_peak'], .85, places=6)
        self.assertLess(max(abs(v) for v in pcm), 32767)

    def test_nonfinite_audio_and_invalid_limits_are_rejected(self):
        for waveform in ([float('nan')], [float('inf')], []):
            with self.assertRaises(ValueError):
                self.pcm(waveform, tts_normalize_peak=.85)
        for config in ({'tts_normalize_peak': 1.1}, {'tts_max_gain': 20},
                       {'tts_normalize_peak': float('nan')}):
            with self.assertRaises(ValueError):
                self.pcm([.2], **config)


if __name__ == '__main__':
    unittest.main()
