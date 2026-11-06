"""Microphone evidence mode cannot become a robot command; no devices are opened."""
import json
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision.audio_test import AudioTestRequestGate, PCMLevelMeter, diagnostic_transcript
import test_voice_capture_lifecycle as lifecycle

FakeThread = lifecycle.FakeThread


class AudioTestDataTests(unittest.TestCase):
    def test_levels_are_actual_pcm_and_silence_is_distinct_from_no_samples(self):
        meter = PCMLevelMeter()
        self.assertIsNone(meter.snapshot()['level_dbfs'])
        meter.add(b'\0\0' * 100)
        self.assertEqual(meter.snapshot(reset=True)['level_dbfs'], -96.)
        self.assertIsNone(meter.snapshot()['level_dbfs'])
        meter.add(struct.pack('<hhhh', 16384, -16384, 16384, -16384))
        self.assertAlmostEqual(meter.snapshot()['level_dbfs'], -6.02, places=2)
        self.assertEqual(meter.snapshot()['peak_dbfs'], -6.02)
        meter = PCMLevelMeter(); meter.add(struct.pack('<hhhh', 32767, -32768, 0, 0))
        self.assertEqual(meter.snapshot()['clipped_fraction'], .5)
        self.assertEqual(meter.snapshot()['peak_dbfs'], 0)
        with self.assertRaises(ValueError): meter.add(b'\0')

    def test_request_freshness_type_and_dedup(self):
        gate = AudioTestRequestGate()
        request = dict(enabled=True, command_id='test:1', issued_at=10., ttl_s=5.)
        for change in ({'enabled': 1}, {'issued_at': 4}, {'issued_at': 11},
                       {'ttl_s': float('nan')}, {'ttl_s': 11}, {'command_id': 'bad id'}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                gate.accept(dict(request, **change), 10.)
        self.assertEqual(gate.accept(request, 10), (True, 'test:1'))
        with self.assertRaisesRegex(ValueError, 'duplicate'):
            gate.accept(request, 10)

    def test_uncertain_transcript_is_visible_but_never_marked_accepted(self):
        result = diagnostic_transcript([dict(text='开始喷水', result=[{'conf': .2}]),
            dict(text='停止喷水', result=[{'conf': .8}])], .65)
        self.assertEqual(result['text'], '开始喷水 停止喷水')
        self.assertEqual(result['confidence'], .5)
        self.assertFalse(result['confidence_gate_passed'])
        self.assertEqual(result['reason'], 'low_asr_confidence')
        missing = diagnostic_transcript([dict(text='麦克风测试', result=[])], .65)
        self.assertEqual(missing['text'], '麦克风测试')
        self.assertIsNone(missing['confidence'])
        self.assertEqual(diagnostic_transcript([{}], .65)['reason'], 'no_speech')


class AudioTestNodeTests(unittest.TestCase):
    setUp = lifecycle.VoiceCaptureLifecycleTests.setUp
    make_voice = lifecycle.VoiceCaptureLifecycleTests.make_voice

    @classmethod
    def setUpClass(cls):
        lifecycle.VoiceCaptureLifecycleTests.setUpClass.__func__(cls)

    def request(self, voice, enabled=True, identifier='test:start', **overrides):
        data = dict(enabled=enabled, command_id=identifier, issued_at=10., ttl_s=5.)
        data.update(overrides)
        voice.audio_test_request(types.SimpleNamespace(data=json.dumps(data)))

    def run_capture(self, voice, blocks=None, status=None, after_open=None, recognizer=None):
        original = sys.modules['sounddevice'].RawInputStream
        request = self.request
        blocks = blocks if blocks is not None else [b'\0\0' * 4800]
        class Stream(original):
            def __enter__(self):
                for raw in blocks:
                    self.kwargs['callback'](raw, len(raw) // 2, None, status)
                if after_open:
                    after_open()
                else:
                    request(voice, False, 'test:stop')
                return self
        with patch.object(sys.modules['sounddevice'], 'RawInputStream', Stream):
            if recognizer:
                with patch.object(sys.modules['vosk'], 'KaldiRecognizer', recognizer):
                    voice.mic_thread.target()
            else:
                voice.mic_thread.target()

    def assert_no_robot_publications(self, voice):
        self.assertEqual(voice.command_pub.values, [])
        self.assertEqual(voice.transcript_pub.values, [])
        self.assertTrue(all(item['command_published'] is False for item in voice.audio_test_pub.values))

    def test_dangerous_low_confidence_multi_endpoint_audio_is_evidence_only(self):
        voice = self.make_voice()
        self.request(voice)
        self.assertEqual(voice.capture_mode, 'audio_test')
        self.assertTrue(voice.continuous_paused)
        original = sys.modules['vosk'].KaldiRecognizer
        class Recognizer(original):
            def Result(self):
                return json.dumps(dict(text='开始喷水', result=[{'conf': .2}]))
            def FinalResult(self):
                return json.dumps(dict(text='停止喷水', result=[{'conf': .3}]))
        blocks = [struct.pack('<h', 8000) * 4800 for _ in range(3)]
        with patch.object(voice, 'accept_text', side_effect=AssertionError('test must not parse commands')):
            self.run_capture(voice, blocks=blocks, recognizer=Recognizer)
        final = voice.audio_test_pub.values[-1]
        self.assertEqual(final['state'], 'completed')
        self.assertEqual(final['text'], '开始喷水 开始喷水 开始喷水 停止喷水')
        self.assertFalse(final['confidence_gate_passed'])
        self.assertEqual(final['reason'], 'low_asr_confidence')
        self.assertAlmostEqual(final['sampled_s'], .3)
        self.assertGreater(final['level_dbfs'], -14.)
        self.assertLess(final['level_dbfs'], -11.)
        self.assertFalse(final['capture_active'])
        self.assertFalse(voice.listening)
        self.assertTrue(voice.continuous_paused)
        self.assert_no_robot_publications(voice)

    def test_normal_and_test_capture_are_mutually_exclusive_and_stop_is_scoped(self):
        voice = self.make_voice()
        voice.listen(types.SimpleNamespace(data=True))
        self.request(voice)
        self.assertEqual(voice.audio_test_pub.values[-1]['reason'], 'microphone_busy')
        self.assertEqual(voice.capture_mode, 'command')
        self.request(voice, False, 'test:stop')
        self.assertEqual(voice.audio_test_pub.values[-1]['reason'], 'no_audio_test_active')
        self.assertFalse(voice.end_listen.is_set())
        self.assertEqual(len(FakeThread.created), 2)
        voice.mic_thread.target()
        self.assertEqual(len(voice.command_pub.values), 1)
        self.request(voice, identifier='test:later')
        thread = voice.mic_thread
        voice.listen(types.SimpleNamespace(data=True))
        self.assertIs(voice.mic_thread, thread)
        self.assertEqual(voice.capture_mode, 'audio_test')

    def test_cancel_preparation_never_opens_device_and_duplicate_never_restarts(self):
        voice = self.make_voice()
        self.request(voice)
        self.request(voice, False, 'test:stop')
        voice.mic_thread.target()
        self.assertEqual(self.streams, [])
        self.assertEqual(voice.audio_test_pub.values[-1]['state'], 'cancelled')
        self.assertFalse(voice.audio_test_pub.values[-1]['capture_active'])
        previous_threads = len(FakeThread.created)
        self.request(voice)
        self.assertEqual(voice.audio_test_pub.values[-1]['request_reason'], 'duplicate_audio_test_request')
        self.assertEqual(voice.audio_test_pub.values[-1]['reason'], 'capture_cancelled_during_preparation')
        self.assertEqual(voice.audio_test_pub.values[-1]['state'], 'cancelled')
        self.assertEqual(len(FakeThread.created), previous_threads)
        self.assert_no_robot_publications(voice)

    def test_expired_request_does_not_open_device_or_reserve_worker(self):
        voice = self.make_voice()
        self.request(voice, issued_at=0.)
        self.assertEqual(voice.audio_test_pub.values[-1]['reason'], 'expired_or_future_audio_test_request')
        self.assertEqual(len(FakeThread.created), 1)
        self.assertFalse(voice.listening)
        self.assertEqual(self.streams, [])

    def test_stop_acknowledgement_keeps_start_session_and_reaches_terminal(self):
        voice = self.make_voice(); self.request(voice)
        def stop():
            self.request(voice, False, 'test:stop')
            receipt = voice.audio_test_pub.values[-1]
            self.assertEqual(receipt['request_id'], 'test:stop')
            self.assertEqual(receipt['request_action'], 'stop')
            self.assertTrue(receipt['request_accepted'])
            self.assertEqual(receipt['session_id'], 'test:start')
            self.assertEqual(receipt['command_id'], 'test:start')
        self.run_capture(voice, after_open=stop)
        final = voice.audio_test_pub.values[-1]
        self.assertEqual(final['state'], 'completed')
        self.assertEqual(final['request_id'], 'test:stop')
        self.assertEqual(final['request_action'], 'stop')
        self.assertEqual(final['session_id'], 'test:start')
        self.assertEqual(final['command_id'], 'test:start')

    def test_heartbeat_does_not_forge_sample_or_result_freshness(self):
        voice = self.make_voice(); self.request(voice)
        for state in ('preparing', 'transcribing', 'completed'):
            voice.audio_test_status(state, level_dbfs=-14., level_stamp=7.)
            original = dict(voice.audio_test_last)
            voice.audio_test_published_at = 0.
            with patch.object(self.module.time, 'monotonic', return_value=6.), patch.object(voice, 'now', return_value=20.):
                voice.audio_test_heartbeat()
            heartbeat = voice.audio_test_pub.values[-1]
            self.assertEqual(heartbeat['stamp'], 20.)
            self.assertEqual(heartbeat['level_stamp'], 7.)
            self.assertEqual(heartbeat['result_stamp'], original['result_stamp'])
            self.assertEqual(voice.audio_test_last, original)
        self.request(voice, identifier='test:stale', issued_at=0.)
        self.assertEqual(voice.audio_test_last, original)
        self.assertTrue(voice.audio_test_pub.values[-1]['request_rejected'])

    def test_automatic_deadline_and_pcm_retention_are_limited_to_ten_seconds(self):
        voice = self.make_voice(); self.request(voice)
        clock = [0.]
        def advance():
            clock[0] += 3.
            return clock[0]
        with patch.object(self.module.time, 'monotonic', side_effect=advance):
            self.run_capture(voice, after_open=lambda: None)
        self.assertFalse(voice.end_listen.is_set())
        self.assertEqual(voice.audio_test_pub.values[-1]['elapsed_s'], 10.)
        self.assertEqual(voice.audio_test_pub.values[-1]['state'], 'completed')
        voice = self.make_voice(); self.request(voice)
        self.run_capture(voice, blocks=[b'\0\0' * 48000 * 12])
        self.assertEqual(voice.audio_test_pub.values[-1]['sampled_s'], 10.)
        self.assert_no_robot_publications(voice)

    def test_device_failure_and_late_transcription_cancellation_release_microphone(self):
        voice = self.make_voice(); self.request(voice)
        with patch.object(sys.modules['sounddevice'], 'RawInputStream', side_effect=RuntimeError('device_disconnected')):
            voice.mic_thread.target()
        self.assertEqual(voice.audio_test_pub.values[-1]['state'], 'error')
        self.assertEqual(voice.audio_test_pub.values[-1]['reason'], 'device_disconnected')
        self.assertFalse(voice.listening)
        voice = self.make_voice(); self.request(voice)
        original = sys.modules['vosk'].KaldiRecognizer
        class Recognizer(original):
            def FinalResult(self):
                voice.abort_listen.set()
                return json.dumps(dict(text='开始喷水', result=[{'conf': 1.}]))
        self.run_capture(voice, recognizer=Recognizer)
        self.assertEqual(voice.audio_test_pub.values[-1]['state'], 'cancelled')
        self.assertEqual(voice.audio_test_pub.values[-1]['text'], '')
        self.assertFalse(voice.listening)
        self.assert_no_robot_publications(voice)

    def test_test_never_resumes_continuous_command_recording(self):
        self.config['listening_mode'] = 'continuous'
        voice = self.make_voice()
        self.request(voice)
        self.run_capture(voice)
        before = len(FakeThread.created)
        for _ in range(8): voice.continuous_tick()
        self.assertEqual(len(FakeThread.created), before)
        voice.listen(types.SimpleNamespace(data=True))
        self.assertEqual(voice.capture_mode, 'command')
        self.assertFalse(voice.continuous_paused)

    def test_normal_stop_is_rejected_and_own_stop_finishes_test_without_commands(self):
        voice = self.make_voice()
        self.request(voice)
        voice.text_input(types.SimpleNamespace(data='开始喷水'))
        self.assertEqual(voice.command_pub.values, [])
        def scoped_stop():
            voice.listen(types.SimpleNamespace(data=False))
            self.assertFalse(voice.end_listen.is_set())
            self.assertFalse(voice.status_pub.values[-1]['stop_request_accepted'])
            self.request(voice, False, 'test:own-stop')
        self.run_capture(voice, after_open=scoped_stop)
        self.assertEqual(voice.audio_test_pub.values[-1]['state'], 'completed')
        self.assert_no_robot_publications(voice)

    def test_overrun_or_playback_cancel_without_transcription(self):
        for case in ('overrun', 'playback'):
            with self.subTest(case=case):
                voice = self.make_voice()
                self.request(voice)
                self.run_capture(voice, status=True if case == 'overrun' else None,
                    after_open=voice.abort_listen.set if case == 'playback' else None)
                final = voice.audio_test_pub.values[-1]
                self.assertEqual(final['state'], 'cancelled')
                self.assertEqual(final['reason'], 'audio_overrun' if case == 'overrun' else 'playback_or_shutdown')
                self.assertEqual(final['text'], '')
                self.assert_no_robot_publications(voice)

    def test_shutdown_suppresses_all_late_publications(self):
        voice = self.make_voice(); self.request(voice)
        captured = []
        def shutdown():
            voice.stop_event.set()
            captured.append(len(voice.audio_test_pub.values))
        self.run_capture(voice, after_open=shutdown)
        self.assertEqual(len(voice.audio_test_pub.values), captured[0])
        self.assertFalse(voice.capture_active)
        self.assertFalse(voice.listening)
        self.assert_no_robot_publications(voice)


if __name__ == '__main__': unittest.main()
