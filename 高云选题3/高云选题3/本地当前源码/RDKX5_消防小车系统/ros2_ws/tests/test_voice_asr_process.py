"""Actual spawned worker recovery without native models, audio, ROS or motion."""
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision.voice_asr_process import ProcessASRBackend
from rescue_vision.voice_capture import StreamingASR


def fake_native_worker(connection, config, workspace):
    # Native-like calls intentionally block in the child so only process
    # termination can recover them; a thread cancellation event is insufficient.
    time.sleep(config.get('startup_delay', 0.))
    connection.send(('ready', dict(asr_backend='fixture', engine='fixture', confidence_available=True)))
    audio = b''
    try:
        while True:
            session, method, args = connection.recv()
            if method == 'create':
                time.sleep(config.get('create_delay', 0.))
                audio = b''
                value = None
            elif method == 'SetWords':
                value = None
            elif method == 'AcceptWaveform':
                audio += args[0]
                if audio == b'BLOCK_INPUT':
                    time.sleep(60.)
                value = False
            elif method == 'FinalResult':
                if audio == b'BLOCK_FINAL':
                    time.sleep(60.)
                value = json.dumps(dict(text=config.get('response_text', audio.decode()), result=[dict(conf=.99)]))
            else:
                value = '{}'
            connection.send(('ok', session, value))
    except (EOFError, BrokenPipeError):
        pass
    finally:
        connection.close()


class VoiceASRProcessTests(unittest.TestCase):
    def make_backend(self, timeout=5.):
        backend = ProcessASRBackend({}, ROOT, worker_target=fake_native_worker,
            operation_timeout_s=timeout, startup_timeout_s=10.)
        self.addCleanup(backend.close)
        return backend

    def decode(self, backend, text):
        decoder = StreamingASR(backend.create_recognizer(), 'diagnostic', .65)
        decoder.submit(text.encode()); decoder.finish(); decoder.start()
        self.assertTrue(decoder.done.wait(3.))
        self.assertIsNone(decoder.error)
        self.assertEqual(decoder.combined()['text'], text)
        decoder.cancel()
        return decoder

    def test_eight_recordings_have_distinct_sessions_but_reuse_one_cached_model(self):
        backend = self.make_backend()
        pid = backend._process.pid
        for index in range(8):
            self.decode(backend, 'question-' + str(index))
            self.assertTrue(backend.ready)
            self.assertEqual(backend._process.pid, pid)
        self.assertEqual(backend._session, 8)

    def test_cancel_hung_native_final_releases_thread_and_next_capture_recovers(self):
        backend = self.make_backend()
        old_pid = backend._process.pid
        first = StreamingASR(backend.create_recognizer(), 'diagnostic', .65)
        first.submit(b'BLOCK_FINAL'); first.finish(); first.start()
        self.assertFalse(first.done.wait(.25))
        started = time.monotonic(); first.cancel()
        self.assertLess(time.monotonic()-started, 2.)
        self.assertTrue(first.done.wait(.5))
        self.assertEqual(first.results, [])
        self.assertFalse(backend.ready)
        self.decode(backend, 'second question')
        self.assertNotEqual(backend._process.pid, old_pid)

    def test_hung_native_input_can_be_reclaimed_without_waiting_for_final(self):
        backend = self.make_backend()
        first = StreamingASR(backend.create_recognizer(), 'diagnostic', .65)
        first.submit(b'BLOCK_INPUT'); first.start()
        self.assertFalse(first.done.wait(.25))
        first.cancel()
        self.assertTrue(first.done.wait(.5))
        self.decode(backend, 'after input cancellation')

    def test_operation_timeout_reclaims_child_then_new_session_can_complete(self):
        backend = self.make_backend(timeout=.2)
        first = StreamingASR(backend.create_recognizer(), 'diagnostic', .65)
        first.submit(b'BLOCK_FINAL'); first.finish(); first.start()
        self.assertTrue(first.done.wait(2.))
        self.assertIn('asr_worker_timeout', str(first.error))
        self.assertFalse(backend.ready)
        self.decode(backend, 'after timeout')

    def test_late_cancel_from_expired_session_cannot_kill_new_worker(self):
        backend = self.make_backend()
        old = backend.create_recognizer()
        old.Cancel()
        current = backend.create_recognizer()
        new_pid = backend._process.pid
        old.Cancel()
        self.assertTrue(backend.ready)
        self.assertEqual(backend._process.pid, new_pid)
        current.AcceptWaveform(b'new utterance')
        self.assertEqual(json.loads(current.FinalResult())['text'], 'new utterance')
        with self.assertRaisesRegex(RuntimeError, 'asr_session_expired'):
            old.FinalResult()
        self.assertTrue(backend.ready)

    def test_close_blocks_future_capture_and_terminates_native_worker(self):
        backend = self.make_backend()
        process = backend._process
        backend.close()
        self.assertFalse(process.is_alive())
        with self.assertRaisesRegex(RuntimeError, 'asr_backend_closed'):
            backend.create_recognizer()

    def test_cancelled_startup_terminates_child_before_load_deadline(self):
        created = []
        discard = ProcessASRBackend._discard_worker
        def track(backend, *args, **kwargs):
            if backend._process is not None:
                created.append(backend._process)
            return discard(backend, *args, **kwargs)
        started = time.monotonic()
        with patch.object(ProcessASRBackend, '_discard_worker', track):
            with self.assertRaisesRegex(RuntimeError, 'asr_preparation_cancelled'):
                ProcessASRBackend({'startup_delay': 60.}, ROOT, worker_target=fake_native_worker,
                    preparation_cancelled=lambda: time.monotonic()-started > .2)
        self.assertLess(time.monotonic()-started, 2.)
        self.assertTrue(created)
        self.assertTrue(all(not child.is_alive() for child in created))

    def test_worker_restart_preparation_cancel_is_recoverable(self):
        backend = self.make_backend()
        backend.create_recognizer().Cancel()
        backend.config['startup_delay'] = 60.
        started = time.monotonic()
        backend.preparation_cancelled = lambda: time.monotonic()-started > .2
        with self.assertRaisesRegex(RuntimeError, 'asr_preparation_cancelled'):
            backend.create_recognizer()
        self.assertLess(time.monotonic()-started, 2.)
        self.assertFalse(backend.ready)
        backend.config.pop('startup_delay')
        backend.preparation_cancelled = None
        self.decode(backend, 'after cancelled reload')

    def test_expired_session_in_same_worker_cannot_cancel_or_decode_new_capture(self):
        backend = self.make_backend()
        old = backend.create_recognizer()
        current = backend.create_recognizer()
        pid = backend._process.pid
        self.assertEqual(old.generation, current.generation)
        old.Cancel()
        with self.assertRaisesRegex(RuntimeError, 'asr_session_expired'):
            old.AcceptWaveform(b'old')
        self.assertTrue(backend.ready)
        self.assertEqual(backend._process.pid, pid)
        current.AcceptWaveform(b'current')
        self.assertEqual(json.loads(current.FinalResult())['text'], 'current')


if __name__ == '__main__':
    unittest.main()
