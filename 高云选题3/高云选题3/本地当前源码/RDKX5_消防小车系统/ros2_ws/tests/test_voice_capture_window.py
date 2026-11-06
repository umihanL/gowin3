"""Full PTT windows, streaming ASR and freshness; no physical audio or ROS."""
import json
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision.voice_capture import StreamingASR, capture_duration
import test_voice_capture_lifecycle as lifecycle


class CaptureWindowTests(unittest.TestCase):
    def test_ptt_has_default_fifteen_second_window_and_ten_second_minimum(self):
        self.assertEqual(capture_duration({}, 'push_to_talk'), 15.)
        self.assertEqual(capture_duration({'max_listen_seconds': 2}, 'push_to_talk'), 10.)
        self.assertEqual(capture_duration({'max_listen_seconds': 100}, 'push_to_talk'), 30.)
        self.assertEqual(capture_duration({'max_listen_seconds': 2}, 'continuous'), 2.)
        for value in (True, None, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                capture_duration({'max_listen_seconds': value}, 'push_to_talk')

    def test_only_confident_entire_standalone_stop_shortens_ptt(self):
        for text, confidence, expected in [('停止喷水', .9, True), ('小车 请关闭水泵', .9, True),
                ('停止喷水', .4, False), ('不要停止喷水', .9, False),
                ('去三号楼', .99, False), ('前进然后停止', .99, False)]:
            decoder = StreamingASR(None, 'push_to_talk', .65)
            decoder._endpoint_result(dict(text=text, result=[{'conf': confidence}]))
            self.assertEqual(decoder.endpoint.is_set(), expected, text)
        decoder = StreamingASR(None, 'push_to_talk', .65)
        decoder._endpoint_result(dict(text='去三号楼', result=[{'conf': 1.}]))
        decoder._endpoint_result(dict(text='停止喷水', result=[{'conf': 1.}]))
        self.assertFalse(decoder.endpoint.is_set())

    def test_continuous_mode_keeps_endpoint_and_ptt_keeps_all_trailing_speech(self):
        class Recognizer:
            def AcceptWaveform(self, pcm): self.text = pcm.decode(); return True
            def Result(self): return json.dumps(dict(text=self.text, result=[{'conf': .9}]))
            def FinalResult(self): return json.dumps(dict(text='继续观察', result=[{'conf': .8}]))
        for mode in ('push_to_talk', 'continuous'):
            decoder = StreamingASR(Recognizer(), mode, .65)
            decoder.submit('去三号楼'.encode()); decoder.submit('不要去'.encode())
            decoder.finish(); decoder.start()
            self.assertTrue(decoder.done.wait(2.))
            self.assertIsNone(decoder.error)
            self.assertEqual(decoder.combined()['text'], '去三号楼 不要去 继续观察')
            self.assertEqual(decoder.endpoint.is_set(), mode == 'continuous')

    def test_emergency_does_not_wait_for_trailing_decode_but_prior_text_prevents_fast_path(self):
        class Recognizer:
            def AcceptWaveform(self, pcm): self.text = pcm.decode(); return True
            def Result(self): return json.dumps(dict(text=self.text, result=[{'conf': .99}]))
            def FinalResult(self):
                raise AssertionError('standalone emergency must not wait for final decode')
        decoder = StreamingASR(Recognizer(), 'push_to_talk', .65)
        decoder.submit('停止喷水'.encode())
        decoder.submit('此处队列剩余语音不应拖延紧急停止'.encode())
        decoder.finish(); decoder.start()
        self.assertTrue(decoder.done.wait(2.))
        self.assertIsNone(decoder.error)
        self.assertEqual(decoder.emergency_result['text'], '停止喷水')
        self.assertEqual(len(decoder.results), 1)
        decoder = StreamingASR(None, 'push_to_talk', .65)
        decoder._endpoint_result(dict(text='不要', result=[{'conf': .99}]))
        decoder._endpoint_result(dict(text='停止喷水', result=[{'conf': .99}]))
        self.assertIsNone(decoder.emergency_result)
        self.assertFalse(decoder.endpoint.is_set())


class CaptureWindowNodeTests(unittest.TestCase):
    setUp = lifecycle.VoiceCaptureLifecycleTests.setUp
    make_voice = lifecycle.VoiceCaptureLifecycleTests.make_voice

    @classmethod
    def setUpClass(cls):
        lifecycle.VoiceCaptureLifecycleTests.setUpClass.__func__(cls)

    def test_multiple_endpoints_and_trailing_speech_preserved_on_manual_stop(self):
        voice = self.make_voice()
        original_stream = sys.modules['sounddevice'].RawInputStream
        original_recognizer = sys.modules['vosk'].KaldiRecognizer
        class Stream(original_stream):
            def __enter__(self):
                for _ in range(2): self.kwargs['callback'](b'\0\0' * 4800, 4800, None, None)
                voice.listen(types.SimpleNamespace(data=False))
                return self
        class Recognizer(original_recognizer):
            index = 0
            def Result(self):
                text = ['去三号楼', '不要去'][self.index]; self.index += 1
                return json.dumps(dict(text=text, result=[{'conf': .95}]))
            def FinalResult(self):
                return json.dumps(dict(text='帮我分析当前情况', result=[{'conf': .95}]))
        voice.listen(types.SimpleNamespace(data=True))
        with patch.object(sys.modules['sounddevice'], 'RawInputStream', Stream), \
                patch.object(sys.modules['vosk'], 'KaldiRecognizer', Recognizer), \
                patch.object(voice, 'accept_text', return_value={'state': 'command_rejected'}) as accept:
            voice.mic_thread.target()
        self.assertEqual(accept.call_count, 1)
        self.assertEqual(accept.call_args.args[0], '去三号楼 不要去 帮我分析当前情况')
        self.assertEqual(voice.transcript_pub.values[-1]['text'], accept.call_args.args[0])
        self.assertEqual(voice.status_pub.values[-1]['ended_by'], 'manual_stop')
        self.assertEqual(voice.status_pub.values[-1]['duration_s'], 15.)
        self.assertAlmostEqual(voice.status_pub.values[-1]['sampled_s'], .2)

    def test_slow_decoder_cannot_extend_microphone_deadline(self):
        self.auto_stop_after_stream = False
        voice = self.make_voice()
        closed = threading.Event()
        original_stream = sys.modules['sounddevice'].RawInputStream
        original_recognizer = sys.modules['vosk'].KaldiRecognizer
        class Stream(original_stream):
            def __exit__(self, *args):
                result = super().__exit__(*args); closed.set(); return result
        class Recognizer(original_recognizer):
            def AcceptWaveform(self, pcm):
                if not closed.wait(2.): raise AssertionError('decoder held microphone open')
                return super().AcceptWaveform(pcm)
        clock = [0.]
        def monotonic(): clock[0] += 1.; return clock[0]
        voice.listen(types.SimpleNamespace(data=True))
        with patch.object(sys.modules['sounddevice'], 'RawInputStream', Stream), \
                patch.object(sys.modules['vosk'], 'KaldiRecognizer', Recognizer), \
                patch.object(self.module.time, 'monotonic', side_effect=monotonic):
            voice.mic_thread.target()
        self.assertTrue(closed.is_set())
        self.assertEqual(voice.status_pub.values[-1]['elapsed_s'], 15.)
        self.assertEqual(voice.status_pub.values[-1]['remaining_s'], 0.)
        self.assertEqual(voice.status_pub.values[-1]['ended_by'], 'duration')
        self.assertEqual(voice.status_pub.values[-1]['state'], 'command_submitted')

    def test_ordinary_endpoint_does_not_close_ptt_before_deadline(self):
        self.auto_stop_after_stream = False
        voice = self.make_voice()
        clock = [0.]
        def monotonic(): clock[0] += .6; return clock[0]
        voice.listen(types.SimpleNamespace(data=True))
        with patch.object(self.module.time, 'monotonic', side_effect=monotonic):
            voice.mic_thread.target()
        self.assertEqual(voice.status_pub.values[-1]['elapsed_s'], 15.)
        self.assertEqual(voice.status_pub.values[-1]['ended_by'], 'duration')
        self.assertTrue(voice.transcript_pub.values)

    def test_status_heartbeat_advances_elapsed_but_not_actual_sample_time(self):
        voice = self.make_voice(); voice.listen(types.SimpleNamespace(data=True))
        voice.capture_active = True; voice.capture_started_monotonic = 20.
        voice.capture_metrics.update(capture_started_at=10., level_dbfs=-14., peak_dbfs=-8., level_stamp=11.)
        with patch.object(self.module.time, 'monotonic', return_value=26.):
            voice.status('listening')
        status = voice.status_pub.values[-1]
        self.assertEqual(status['elapsed_s'], 6.)
        self.assertEqual(status['remaining_s'], 9.)
        self.assertEqual(status['level_stamp'], 11.)
        self.assertEqual(status['capture_started_at'], 10.)

    def test_command_issue_time_is_fresh_but_capture_age_is_bounded(self):
        voice = self.make_voice()
        with patch.object(voice, 'now', return_value=19.):
            voice.accept_text('分析当前情况', 'voice', 10.)
        self.assertEqual(voice.command_pub.values[-1]['issued_at'], 19.)
        self.assertEqual(voice.command_pub.values[-1]['capture_stamp'], 10.)
        count = len(voice.command_pub.values)
        with patch.object(voice, 'now', return_value=41.):
            voice.accept_text('去三号楼', 'voice', 10.)
        self.assertEqual(len(voice.command_pub.values), count)
        self.assertEqual(voice.status_pub.values[-1]['reason'], 'transcription_expired')

    def test_transcript_correlates_submitted_command_and_retains_route_failure(self):
        voice = self.make_voice(); voice.listen(types.SimpleNamespace(data=True)); voice.mic_thread.target()
        transcript = voice.transcript_pub.values[-1]
        self.assertTrue(transcript['submitted'])
        self.assertEqual(transcript['command_id'], voice.command_pub.values[-1]['command_id'])
        self.assertEqual(transcript['action'], voice.command_pub.values[-1]['action'])
        self.assertIsNone(transcript['route_error'])
        voice = self.make_voice(); voice.listen(types.SimpleNamespace(data=True))
        with patch.object(voice, 'accept_text', return_value=dict(state='command_rejected', reason='test_route_failure')):
            voice.mic_thread.target()
        transcript = voice.transcript_pub.values[-1]
        self.assertTrue(transcript['text'])
        self.assertFalse(transcript['submitted'])
        self.assertIsNone(transcript['command_id'])
        self.assertEqual(transcript['route_error'], 'test_route_failure')

    def test_unfinished_native_decoder_blocks_new_normal_and_diagnostic_workers(self):
        voice = self.make_voice()
        done = threading.Event()
        decoder = types.SimpleNamespace(done=done, cancel=lambda: None)
        voice.asr_decoder = decoder
        before = len(lifecycle.FakeThread.created)
        voice.listen(types.SimpleNamespace(data=True))
        self.assertEqual(voice.status_pub.values[-1]['reason'], 'asr_decoder_still_stopping')
        request = dict(command_id='test:busy', enabled=True, issued_at=10., ttl_s=5.)
        voice.audio_test_request(types.SimpleNamespace(data=json.dumps(request)))
        self.assertEqual(voice.audio_test_pub.values[-1]['reason'], 'asr_decoder_still_stopping')
        self.assertEqual(len(lifecycle.FakeThread.created), before)
        self.assertFalse(voice.listening)
        self.assertIs(voice.asr_decoder, decoder)
        done.set()
        voice.listen(types.SimpleNamespace(data=True))
        self.assertTrue(voice.listening)
        self.assertEqual(len(lifecycle.FakeThread.created), before + 1)
        self.assertIsNone(voice.asr_decoder)

    def test_audio_overrun_discards_all_instead_of_truncated_command(self):
        voice = self.make_voice(); voice.listen(types.SimpleNamespace(data=True))
        original = sys.modules['sounddevice'].RawInputStream
        class Stream(original):
            def __enter__(self):
                self.kwargs['callback'](b'\0\0' * 4800, 4800, None, True)
                return self
        with patch.object(sys.modules['sounddevice'], 'RawInputStream', Stream):
            voice.mic_thread.target()
        self.assertEqual(voice.command_pub.values, [])
        self.assertEqual(voice.transcript_pub.values, [])
        self.assertEqual(voice.status_pub.values[-1]['state'], 'audio_discarded')

    def test_transcription_timeout_never_submits_an_old_recording(self):
        voice = self.make_voice(); voice.listen(types.SimpleNamespace(data=True))
        class StuckDecoder:
            def __init__(self, *args):
                self.endpoint = threading.Event()
                self.done = types.SimpleNamespace(is_set=lambda: False, wait=lambda timeout: False)
                self.emergency_result = None
            def start(self): pass
            def submit(self, pcm): pass
            def finish(self): pass
            def cancel(self): pass
        clock = [0.]
        def monotonic(): clock[0] += 10.; return clock[0]
        with patch.object(self.module, 'StreamingASR', StuckDecoder), \
                patch.object(self.module.time, 'monotonic', side_effect=monotonic):
            voice.mic_thread.target()
        self.assertEqual(voice.command_pub.values, [])
        self.assertEqual(voice.transcript_pub.values, [])
        self.assertEqual(voice.status_pub.values[-1]['reason'], 'transcription_timeout')
        self.assertFalse(voice.capture_active)
        self.assertFalse(voice.listening)
        self.assertIsNotNone(voice.asr_decoder)
        before = len(lifecycle.FakeThread.created)
        voice.listen(types.SimpleNamespace(data=True))
        self.assertEqual(len(lifecycle.FakeThread.created), before)
        self.assertEqual(voice.status_pub.values[-1]['reason'], 'asr_decoder_still_stopping')

    def test_standalone_stop_and_continuous_endpoint_end_recording_promptly(self):
        self.auto_stop_after_stream = False
        original = sys.modules['vosk'].KaldiRecognizer
        class StopRecognizer(original):
            def Result(self): return json.dumps(dict(text='停止喷水', result=[{'conf': .99}]))
        for mode, recognizer, reason in [('push_to_talk', StopRecognizer, 'emergency_stop'),
                                         ('continuous', original, 'endpoint')]:
            self.config['listening_mode'] = mode
            voice = self.make_voice(); voice.listen(types.SimpleNamespace(data=True))
            with patch.object(sys.modules['vosk'], 'KaldiRecognizer', recognizer):
                voice.mic_thread.target()
            self.assertEqual(voice.status_pub.values[-1]['ended_by'], reason)
            self.assertLess(voice.status_pub.values[-1]['elapsed_s'], 2.)
            self.assertTrue(voice.transcript_pub.values)


if __name__ == '__main__': unittest.main()
