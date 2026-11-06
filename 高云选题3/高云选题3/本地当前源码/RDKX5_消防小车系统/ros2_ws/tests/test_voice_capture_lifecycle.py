"""Production node lifecycle, with mocked ROS/audio: idle must never record."""
import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))


class Publisher:
    def __init__(self): self.values = []


class FakeNode:
    def __init__(self, _name): self.publishers = {}; self.timers = []
    def create_publisher(self, _type, topic, _depth):
        self.publishers[topic] = Publisher()
        return self.publishers[topic]
    def create_subscription(self, *args): pass
    def create_timer(self, period, callback): self.timers.append((period, callback))
    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=10_000_000_000))
    def destroy_node(self): pass


class FakeThread:
    created = []
    def __init__(self, *, target, daemon):
        self.target = target; self.started = False; self.created.append(self)
    def start(self): self.started = True
    def join(self, timeout=None): pass


class VoiceCaptureLifecycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Load numerical extension modules before patch.dict snapshots the
        # module table; restoring that table must not reload NumPy per case.
        from rescue_vision.audio_pcm import PCM16To16k
        from rescue_vision.voice_capture import StreamingASR
        PCM16To16k(48000)

    def setUp(self):
        self.config = json.loads((ROOT / 'config/voice.local.json').read_text(encoding='utf-8'))
        # These compatibility lifecycle cases deliberately exercise Vosk. New
        # SenseVoice cases inject their backend independently of deployed defaults.
        self.config['asr_backend'] = 'vosk'
        params = dict(workspace_dir=str(ROOT), task_commands=True)
        common = types.ModuleType('rescue_vision.node_common')
        common.param = lambda node, name, default: params.get(name, default)
        common.read_json = lambda path: (json.loads((ROOT / 'config/buildings.json').read_text(encoding='utf-8'))
            if str(path).endswith('buildings.json') else self.config.copy())
        common.publish_json = lambda pub, value: pub.values.append(value)
        common.spin = lambda factory: None
        rclpy = types.ModuleType('rclpy'); node = types.ModuleType('rclpy.node'); node.Node = FakeNode
        messages = types.ModuleType('std_msgs.msg')
        messages.Bool = messages.String = types.SimpleNamespace
        self.modules = patch.dict(sys.modules, {'rclpy': rclpy, 'rclpy.node': node,
            'std_msgs': types.ModuleType('std_msgs'), 'std_msgs.msg': messages,
            'rescue_vision.node_common': common})
        self.modules.start(); self.addCleanup(self.modules.stop)
        spec = importlib.util.spec_from_file_location('rescue_vision._voice_capture_test',
            ROOT / 'src/rescue_vision/rescue_vision/voice_node.py')
        self.module = importlib.util.module_from_spec(spec); spec.loader.exec_module(self.module)
        # These lifecycle fixtures provide in-memory native stubs. Process
        # cancellation/recovery is exercised separately with actual children.
        from rescue_vision.asr_backend import load_asr_backend
        backend_patch = patch.object(self.module, 'load_asr_backend',
            lambda config, workspace, **kwargs: load_asr_backend(config, workspace))
        backend_patch.start(); self.addCleanup(backend_patch.stop)
        FakeThread.created = []
        self.threads = patch.object(self.module.threading, 'Thread', FakeThread)
        self.threads.start(); self.addCleanup(self.threads.stop)
        self.streams = []
        self.model_calls = 0
        self.pcm_lengths = []
        self.auto_stop_after_stream = True
        outer = self
        class Stream:
            def __init__(self, **kwargs):
                self.kwargs = kwargs; self.exited = False; outer.streams.append(self)
            def __enter__(self):
                self.kwargs['callback'](b'\0\0' * self.kwargs['blocksize'], self.kwargs['blocksize'], None, None)
                if outer.auto_stop_after_stream:
                    voice = outer.active_voice
                    if voice.capture_mode == 'audio_test':
                        voice.audio_test_request(types.SimpleNamespace(data=json.dumps(dict(enabled=False,
                            command_id='fixture-stop:' + voice.audio_test_session, issued_at=voice.now(), ttl_s=5.))))
                    else:
                        voice.listen(types.SimpleNamespace(data=False))
                return self
            def __exit__(self, *args): self.exited = True
        class Recognizer:
            def __init__(self, model, rate): outer.assertEqual(rate, 16000)
            def SetWords(self, enabled): pass
            def AcceptWaveform(self, data): outer.pcm_lengths.append(len(data)); return True
            def Result(self): return json.dumps({'text': '现在有几个人', 'result': [{'conf': .9}]})
            def FinalResult(self): return json.dumps({'text': '', 'result': []})
        def model_loader(path): self.model_calls += 1; return object()
        self.audio_modules = patch.dict(sys.modules, {'sounddevice': types.SimpleNamespace(RawInputStream=Stream),
            'vosk': types.SimpleNamespace(Model=model_loader, KaldiRecognizer=Recognizer)})
        self.audio_modules.start(); self.addCleanup(self.audio_modules.stop)

    def make_voice(self):
        self.active_voice = self.module.Voice()
        return self.active_voice
    def states(self, node): return [value['state'] for value in node.status_pub.values]

    def test_enabled_push_to_talk_initialization_and_idle_never_open_audio(self):
        voice = self.make_voice()
        self.assertTrue(voice.config['microphone_enabled'])
        self.assertEqual(voice.listening_mode, 'push_to_talk')
        for _ in range(10): voice.continuous_tick()
        voice.heartbeat()
        self.assertFalse(voice.listening)
        self.assertFalse(voice.capture_active)
        self.assertEqual(self.streams, [])
        self.assertEqual(self.model_calls, 0)
        self.assertEqual(len(FakeThread.created), 1)  # Only idle TTS queue worker.
        self.assertEqual(self.states(voice)[-1], 'idle')

    def test_explicit_push_to_talk_can_switch_task_but_cannot_submit_traffic_dialogue(self):
        voice=self.make_voice();voice.extension_enabled=True
        voice.extension_state(types.SimpleNamespace(data=json.dumps(dict(stamp=10.,task='traffic',dialogue_enabled=False))))
        voice.listen(types.SimpleNamespace(data=True))
        self.assertTrue(voice.listening)
        self.assertFalse(voice.capture_active)
        self.assertEqual(self.streams,[])
        self.assertEqual(voice.accept_text('有几团火焰','text',10.)['state'],'command_rejected')
        self.assertEqual(voice.accept_text('停车','voice',10.)['state'],'command_submitted')
        self.assertEqual(voice.command_pub.values[-1]['action'],'stop')
        self.assertEqual(voice.accept_text('开启任务1','voice',10.)['state'],'command_submitted')
        self.assertEqual(voice.command_pub.values[-1]['action'],'extension_vision')

    def test_extension2_can_record_offline_then_switch_to_traffic_aborts(self):
        voice=self.make_voice();voice.extension_enabled=True
        voice.extension_state(types.SimpleNamespace(data=json.dumps(dict(stamp=10.,task='dialogue',dialogue_enabled=True,real_uart=False))))
        voice.listen(types.SimpleNamespace(data=True))
        self.assertTrue(voice.listening)
        self.assertFalse(voice.abort_listen.is_set())
        voice.extension_state(types.SimpleNamespace(data=json.dumps(dict(stamp=10.,task='traffic',dialogue_enabled=False))))
        self.assertTrue(voice.abort_listen.is_set())
        self.assertTrue(voice.end_listen.is_set())

    def test_extension1_can_record_and_accept_scene_question_without_uart(self):
        voice=self.make_voice();voice.extension_enabled=True
        voice.extension_state(types.SimpleNamespace(data=json.dumps(dict(stamp=10.,task='vision',dialogue_enabled=True,real_uart=False))))
        voice.listen(types.SimpleNamespace(data=True))
        self.assertTrue(voice.listening)
        self.assertFalse(voice.abort_listen.is_set())
        self.assertEqual(voice.accept_text('分析当前火场','text',10.)['state'],'command_submitted')
        self.assertEqual(voice.command_pub.values[-1]['action'],'scene')

    def test_click_starts_one_stream_then_closes_and_does_not_restart(self):
        voice = self.make_voice()
        voice.listen(types.SimpleNamespace(data=True))
        voice.listen(types.SimpleNamespace(data=True))
        self.assertEqual(len(FakeThread.created), 2)
        self.assertEqual(self.streams, [])
        voice.heartbeat()
        self.assertEqual(self.states(voice)[-1], 'preparing')
        voice.mic_thread.target()
        self.assertEqual(len(self.streams), 1)
        self.assertEqual(self.streams[0].kwargs['samplerate'], 48000)
        self.assertEqual(self.pcm_lengths, [3200])
        self.assertTrue(self.streams[0].exited)
        self.assertFalse(voice.listening)
        self.assertFalse(voice.capture_active)
        # This fake device receives stop inside __enter__, before its owner
        # can publish the first capture status. It must never revert the stop.
        self.assertIn('stopping', self.states(voice))
        final = voice.status_pub.values[-1]
        self.assertEqual(final['state'], 'command_submitted')
        self.assertFalse(final['listening'])
        self.assertFalse(final['capture_active'])
        self.assertEqual(final['recognition_phase'], 'idle')
        self.assertEqual(final['last_recognition']['command_id'], final['command_id'])
        for _ in range(10): voice.continuous_tick()
        self.assertEqual(len(self.streams), 1)
        voice.heartbeat()
        self.assertEqual(voice.status_pub.values[-1]['last_recognition'], final['last_recognition'])
        voice.listen(types.SimpleNamespace(data=True))
        self.assertTrue(voice.listening)
        self.assertEqual(len(FakeThread.created), 3)

    def test_six_complete_captures_reset_stop_state_and_publish_unique_commands(self):
        voice = self.make_voice()
        for index in range(6):
            voice.listen(types.SimpleNamespace(data=True))
            self.assertTrue(voice.listening)
            self.assertFalse(voice.end_listen.is_set())
            self.assertFalse(voice.abort_listen.is_set())
            self.assertFalse(voice.status_pub.values[-1]['stop_requested'])
            voice.mic_thread.target()
            self.assertFalse(voice.listening)
            self.assertFalse(voice.capture_active)
            self.assertTrue(voice.capture_stop_confirmed)
            self.assertIsNone(voice.asr_decoder)
            self.assertTrue(self.streams[-1].exited)
            self.assertEqual(len(voice.command_pub.values), index+1)
            self.assertEqual(len(voice.transcript_pub.values), index+1)
        self.assertEqual(len({c['command_id'] for c in voice.command_pub.values}), 6)
        self.assertEqual(self.model_calls, 1)

    def test_cancelled_preparation_can_be_followed_by_two_complete_recordings(self):
        voice = self.make_voice()
        voice.listen(types.SimpleNamespace(data=True))
        voice.listen(types.SimpleNamespace(data=False))
        voice.mic_thread.target()
        self.assertEqual(self.streams, [])
        for _ in range(2):
            voice.listen(types.SimpleNamespace(data=True))
            voice.mic_thread.target()
        self.assertEqual(len(voice.transcript_pub.values), 2)
        self.assertEqual(len(voice.command_pub.values), 2)
        self.assertFalse(voice.listening)

    def test_manual_diagnostic_stop_still_transcribes_twice_with_real_isolated_worker(self):
        from rescue_vision.voice_asr_process import ProcessASRBackend
        from test_voice_asr_process import fake_native_worker
        voice = self.make_voice()
        def load(config, workspace, **kwargs):
            backend = ProcessASRBackend(dict(config, response_text='今天进行语音测试'), workspace,
                worker_target=fake_native_worker, **kwargs)
            self.addCleanup(backend.close)
            return backend
        with patch.object(self.module, 'load_asr_backend', side_effect=load):
            for index in range(2):
                request = dict(command_id='isolated-diagnostic-' + str(index), enabled=True,
                               issued_at=voice.now(), ttl_s=5.)
                voice.audio_test_request(types.SimpleNamespace(data=json.dumps(request)))
                voice.mic_thread.target()
                self.assertTrue(voice.end_listen.is_set(), voice.audio_test_last)
                self.assertEqual(voice.audio_test_last['state'], 'completed', voice.audio_test_last)
                self.assertEqual(voice.audio_test_last['text'], '今天进行语音测试')
                self.assertFalse(voice.listening)
                self.assertIsNone(voice.asr_decoder)
                self.assertTrue(voice.asr_backend.ready)
        self.assertEqual(voice.command_pub.values, [])
        self.assertEqual(voice.asr_backend._session, 2)
        voice.asr_backend.preparation_cancelled = lambda: (_ for _ in ()).throw(RuntimeError('stale deadline'))
        voice.listen(types.SimpleNamespace(data=True))
        voice.mic_thread.target()
        self.assertEqual(len(voice.command_pub.values), 1)
        self.assertFalse(voice.listening)
        self.assertTrue(voice.asr_backend.ready)

    def test_stop_during_preparation_never_opens_microphone(self):
        voice = self.make_voice()
        voice.listen(types.SimpleNamespace(data=True))
        voice.listen(types.SimpleNamespace(data=False))
        voice.mic_thread.target()
        self.assertEqual(self.streams, [])
        self.assertFalse(voice.listening)
        self.assertFalse(voice.capture_active)
        self.assertEqual(voice.command_pub.values, [])
        final = voice.status_pub.values[-1]
        self.assertEqual(final['state'], 'listening_cancelled')
        self.assertEqual(final['reason'], 'capture_cancelled_during_preparation')
        self.assertFalse(final['listening'])
        self.assertFalse(final['capture_active'])
        voice.listen(types.SimpleNamespace(data=True))
        self.assertTrue(voice.listening)
        self.assertFalse(voice.end_listen.is_set())

    def test_stop_is_acknowledged_before_worker_runs_and_cancels_model_load(self):
        voice = self.make_voice()
        voice.listen(types.SimpleNamespace(data=True))
        voice.listen(types.SimpleNamespace(data=False))
        receipt = voice.status_pub.values[-1]
        self.assertTrue(receipt['stop_request_accepted'])
        self.assertTrue(receipt['stop_requested'])
        self.assertEqual(receipt['stop_request_seq'], 1)
        self.assertEqual(receipt['stop_requested_at'], 10.)
        self.assertEqual(receipt['state'], 'preparing')
        self.assertFalse(receipt['capture_active'])
        self.assertFalse(receipt['capture_stop_confirmed'])
        self.assertTrue(receipt['listening'])  # Worker still owns the session.
        voice.mic_thread.target()
        self.assertEqual(self.model_calls, 0)
        self.assertEqual(self.streams, [])
        self.assertTrue(voice.status_pub.values[-1]['capture_stop_confirmed'])
        self.assertFalse(voice.status_pub.values[-1]['listening'])

    def test_stop_during_model_load_acknowledges_without_claiming_closed(self):
        voice = self.make_voice()
        original_loader = self.module.load_asr_backend
        def loading(*args, **kwargs):
            voice.listen(types.SimpleNamespace(data=False))
            receipt = voice.status_pub.values[-1]
            self.assertEqual(receipt['state'], 'loading_asr')
            self.assertTrue(receipt['stop_request_accepted'])
            self.assertFalse(receipt['capture_stop_confirmed'])
            self.assertTrue(receipt['listening'])
            return original_loader(*args, **kwargs)
        voice.listen(types.SimpleNamespace(data=True))
        with patch.object(self.module, 'load_asr_backend', side_effect=loading):
            voice.mic_thread.target()
        self.assertEqual(self.streams, [])
        self.assertTrue(voice.status_pub.values[-1]['capture_stop_confirmed'])

    def test_recording_stop_stays_stopping_until_device_context_exits(self):
        voice = self.make_voice()
        original_stream = sys.modules['sounddevice'].RawInputStream
        original_status = voice.status
        def stop_on_recording(state, **details):
            original_status(state, **details)
            if state == 'listening' and not voice.end_listen.is_set():
                voice.listen(types.SimpleNamespace(data=False))
                receipt = voice.status_pub.values[-1]
                self.assertEqual(receipt['state'], 'stopping')
                self.assertTrue(receipt['capture_active'])
                self.assertFalse(receipt['capture_stop_confirmed'])
                # A stale recording update and heartbeat cannot undo stop.
                original_status('listening', capture_active=True)
                voice.heartbeat()
                self.assertEqual(voice.status_pub.values[-1]['state'], 'stopping')
        class CheckedStream(original_stream):
            def __exit__(inner, *args):
                self.assertTrue(voice.capture_active)
                self.assertFalse(voice.capture_stop_confirmed)
                return super().__exit__(*args)
        self.auto_stop_after_stream = False
        voice.listen(types.SimpleNamespace(data=True))
        with patch.object(voice, 'status', side_effect=stop_on_recording), \
                patch.object(sys.modules['sounddevice'], 'RawInputStream', CheckedStream):
            voice.mic_thread.target()
        processing = next(v for v in voice.status_pub.values if v['state'] == 'transcribing')
        self.assertFalse(processing['capture_active'])
        self.assertTrue(processing['capture_stop_confirmed'])
        self.assertTrue(self.streams[0].exited)
        self.assertEqual(voice.status_pub.values[-1]['state'], 'command_submitted')
        self.assertAlmostEqual(voice.status_pub.values[-1]['sampled_s'], .1)

    def test_repeated_stop_during_transcription_is_acknowledged_without_cancelling(self):
        voice = self.make_voice()
        original_status = voice.status
        def stop_while_processing(state, **details):
            original_status(state, **details)
            if state == 'transcribing' and not details.get('testing_repeat'):
                # Avoid recursively running this test hook for immediate receipts.
                with patch.object(voice, 'status', original_status):
                    before = voice.stop_request_seq
                    voice.listen(types.SimpleNamespace(data=False))
                    first = voice.status_pub.values[-1]
                    voice.listen(types.SimpleNamespace(data=False))
                    second = voice.status_pub.values[-1]
                    self.assertEqual(first['stop_request_seq'], before + 1)
                    self.assertEqual(second['stop_request_seq'], before + 2)
                    self.assertEqual(second['state'], 'transcribing')
                    self.assertTrue(second['capture_stop_confirmed'])
                    self.assertFalse(second['capture_active'])
                    self.assertFalse(voice.abort_listen.is_set())
                    count = len(FakeThread.created)
                    voice.listen(types.SimpleNamespace(data=True))
                    self.assertEqual(len(FakeThread.created), count)
        voice.listen(types.SimpleNamespace(data=True))
        with patch.object(voice, 'status', side_effect=stop_while_processing):
            voice.mic_thread.target()
        self.assertEqual(len(voice.command_pub.values), 1)
        self.assertEqual(len(voice.transcript_pub.values), 1)
        self.assertTrue(voice.capture_stop_confirmed)
        previous_seq = voice.stop_request_seq
        voice.listen(types.SimpleNamespace(data=True))
        prepared = voice.status_pub.values[-1]
        self.assertEqual(prepared['stop_request_seq'], previous_seq)
        self.assertIsNone(prepared['stop_request_accepted'])
        self.assertIsNone(prepared['stop_requested_at'])
        self.assertFalse(prepared['stop_requested'])
        self.assertFalse(prepared['capture_stop_confirmed'])

    def test_normal_stop_cannot_stop_the_independent_audio_test(self):
        voice = self.make_voice()
        request = dict(command_id='test:normal-stop', enabled=True, issued_at=10., ttl_s=5.)
        voice.audio_test_request(types.SimpleNamespace(data=json.dumps(request)))
        self.assertEqual(voice.capture_mode, 'audio_test')
        voice.listen(types.SimpleNamespace(data=False))
        receipt = voice.status_pub.values[-1]
        self.assertFalse(receipt['stop_request_accepted'])
        self.assertEqual(receipt['stop_request_reason'], 'audio_test_requires_own_stop')
        self.assertEqual(receipt['stop_request_seq'], 1)
        self.assertFalse(voice.end_listen.is_set())
        self.assertTrue(voice.listening)

    def test_capture_failure_immediately_releases_ui_and_survives_heartbeat(self):
        voice = self.make_voice()
        voice.listen(types.SimpleNamespace(data=True))
        with patch.object(sys.modules['sounddevice'], 'RawInputStream', side_effect=RuntimeError('device_disconnected')):
            voice.mic_thread.target()
        final = voice.status_pub.values[-1]
        self.assertEqual(final['state'], 'asr_unavailable_or_rejected')
        self.assertEqual(final['reason'], 'device_disconnected')
        self.assertFalse(final['listening'])
        self.assertFalse(final['capture_active'])
        voice.heartbeat()
        self.assertEqual(voice.status_pub.values[-1]['last_recognition']['reason'], 'device_disconnected')
        voice.listen(types.SimpleNamespace(data=True))
        self.assertTrue(voice.listening)

    def test_device_error_during_stop_is_terminal_without_a_false_closed_receipt(self):
        voice = self.make_voice()
        original_stream = sys.modules['sounddevice'].RawInputStream
        class FailingClose(original_stream):
            def __exit__(self, *args):
                raise RuntimeError('device_close_failed')
        voice.listen(types.SimpleNamespace(data=True))
        with patch.object(sys.modules['sounddevice'], 'RawInputStream', FailingClose):
            voice.mic_thread.target()
        final = voice.status_pub.values[-1]
        self.assertEqual(final['state'], 'asr_unavailable_or_rejected')
        self.assertEqual(final['reason'], 'device_close_failed')
        self.assertFalse(final['listening'])
        self.assertFalse(final['capture_stop_confirmed'])
        self.assertTrue(final['stop_requested'])
        self.assertEqual(voice.command_pub.values, [])
        voice.listen(types.SimpleNamespace(data=False))
        self.assertFalse(voice.status_pub.values[-1]['capture_stop_confirmed'])
        self.assertEqual(voice.status_pub.values[-1]['last_recognition']['reason'], 'device_close_failed')

    def test_command_rejection_immediately_releases_ui_and_preserves_reason(self):
        self.config['listening_mode'] = 'continuous'
        voice = self.make_voice()
        voice.listen(types.SimpleNamespace(data=True))
        voice.mic_thread.target()
        final = voice.status_pub.values[-1]
        self.assertEqual(final['state'], 'command_rejected')
        self.assertEqual(final['reason'], 'wake_word_required')
        self.assertFalse(final['listening'])
        self.assertFalse(final['capture_active'])
        self.assertEqual(voice.command_pub.values, [])
        voice.heartbeat()
        self.assertEqual(voice.status_pub.values[-1]['last_recognition']['reason'], 'wake_word_required')

    def test_stop_recording_finalizes_instead_of_cancelling_a_captured_utterance(self):
        voice = self.make_voice()
        original_stream = sys.modules['sounddevice'].RawInputStream
        class StopAfterFirstBlock(original_stream):
            def __enter__(self):
                result = super().__enter__()
                voice.listen(types.SimpleNamespace(data=False))
                return result
        voice.listen(types.SimpleNamespace(data=True))
        with patch.object(sys.modules['sounddevice'], 'RawInputStream', StopAfterFirstBlock):
            voice.mic_thread.target()
        self.assertEqual(len(voice.command_pub.values), 1)
        self.assertEqual(voice.status_pub.values[-1]['state'], 'command_submitted')
        self.assertFalse(voice.status_pub.values[-1]['listening'])

    def test_shutdown_during_final_transcription_does_not_publish(self):
        self.check_transcription_cancellation(shutdown=True)

    def test_playback_during_final_transcription_discards_command(self):
        self.check_transcription_cancellation(shutdown=False)

    def check_transcription_cancellation(self, shutdown):
        voice = self.make_voice()
        original_stream = sys.modules['sounddevice'].RawInputStream
        original_recognizer = sys.modules['vosk'].KaldiRecognizer
        class StopAfterFirstBlock(original_stream):
            def __enter__(self):
                result = super().__enter__()
                voice.listen(types.SimpleNamespace(data=False))
                return result
        counts_at_cancellation = []
        class FinalRecognizer(original_recognizer):
            def AcceptWaveform(self, data): return False
            def FinalResult(self):
                (voice.stop_event if shutdown else voice.abort_listen).set()
                counts_at_cancellation.append(len(voice.status_pub.values))
                return json.dumps({'text': '寻找火焰', 'result': [{'conf': .9}]})
        voice.listen(types.SimpleNamespace(data=True))
        with patch.object(sys.modules['sounddevice'], 'RawInputStream', StopAfterFirstBlock), \
             patch.object(sys.modules['vosk'], 'KaldiRecognizer', FinalRecognizer):
            voice.mic_thread.target()
        self.assertEqual(voice.command_pub.values, [])
        self.assertEqual(voice.transcript_pub.values, [])
        self.assertFalse(voice.listening)
        self.assertFalse(voice.capture_active)
        if shutdown:
            self.assertEqual(len(voice.status_pub.values), counts_at_cancellation[0])
        else:
            final = voice.status_pub.values[-1]
            self.assertEqual(final['state'], 'audio_discarded')
            self.assertEqual(final['reason'], 'playback_or_shutdown_during_transcription')
            self.assertFalse(final['listening'])

    def test_disabled_microphone_refuses_click(self):
        self.config['microphone_enabled'] = False
        voice = self.make_voice()
        voice.listen(types.SimpleNamespace(data=True))
        self.assertEqual(len(FakeThread.created), 1)
        self.assertEqual(self.streams, [])
        self.assertEqual(self.states(voice)[-1], 'microphone_disabled')

    def test_shutdown_does_not_accept_new_capture(self):
        voice = self.make_voice(); voice.stop_event.set()
        voice.listen(types.SimpleNamespace(data=True))
        self.assertEqual(len(FakeThread.created), 1)
        self.assertEqual(self.streams, [])

    def test_transient_status_keeps_capabilities_and_busy_fields(self):
        voice = self.make_voice()
        voice.listening = True
        for state in ('preparing', 'loading_asr', 'listening', 'transcribing'):
            voice.capture_active = state == 'listening'
            voice.status(state)
            payload = voice.status_pub.values[-1]
            self.assertTrue(payload['microphone_enabled'])
            self.assertFalse(payload['tts_enabled'])
            self.assertTrue(payload['listening'])
            self.assertEqual(payload['capture_active'], state == 'listening')
            self.assertEqual(payload['listening_mode'], 'push_to_talk')
            self.assertEqual(payload['recognition_phase'], state)
        voice.status('custom', microphone_enabled=False)
        self.assertFalse(voice.status_pub.values[-1]['microphone_enabled'])

    def test_heartbeat_retains_transcribing_until_worker_finishes(self):
        voice = self.make_voice()
        voice.listening = True
        voice.status('transcribing')
        voice.heartbeat()
        self.assertEqual(self.states(voice)[-1], 'transcribing')
        self.assertTrue(voice.status_pub.values[-1]['listening'])
        self.assertFalse(voice.status_pub.values[-1]['capture_active'])


if __name__ == '__main__': unittest.main()
