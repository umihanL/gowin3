"""Missing USB output and persistent TTS evidence; no actual sound/model/ROS."""
from pathlib import Path
import queue
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import patch, Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision.voice_tts_process import alsa_playback_status, speech_chunks
import test_voice_capture_lifecycle as lifecycle


class AlsaOutputTests(unittest.TestCase):
    def test_full_analytical_reply_is_split_without_losing_qualifiers(self):
        text = '以下基于提问时快照：' + ('建议先复核火源位置和人员情况，再核对消防员与普通人物的区别。' * 20) + '以上为观察建议，未执行控制。'
        self.assertGreater(len(text),350)
        parts = speech_chunks(text)
        self.assertEqual(''.join(parts),text)
        self.assertTrue(all(0 < len(part) <= 180 for part in parts))
        self.assertTrue(parts[-1].endswith('未执行控制。'))
        self.assertEqual(''.join(speech_chunks('无' * 900)), '无' * 900)
        self.assertTrue(all(len(part)<=180 for part in speech_chunks('句。'+' '*898)))
        for invalid in ('', ' '*900, '字'*901, None):
            with self.assertRaises(ValueError):
                speech_chunks(invalid)

    def test_named_usb_card_must_have_playback_pcm(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            mic = root / 'card0'; mic.mkdir(); (mic / 'id').write_text('Device\n')
            (mic / 'pcm0c').mkdir(); (mic / 'pcm0c/info').write_text('capture')
            device = 'plughw:CARD=RescueSpeaker,DEV=0'
            result = alsa_playback_status(device, root)
            self.assertFalse(result['tts_device_ready'])
            self.assertEqual(result['reason'], 'audio_output_device_missing')
            speaker = root / 'card2'; speaker.mkdir(); (speaker / 'id').write_text('RescueSpeaker\n')
            self.assertEqual(alsa_playback_status(device, root)['reason'], 'audio_output_pcm_missing')
            (speaker / 'pcm0p').mkdir(); info = speaker / 'pcm0p/info'; info.write_text('playback')
            self.assertTrue(alsa_playback_status(device, root)['tts_device_ready'])
            self.assertTrue(alsa_playback_status('hw:2,0', root)['tts_device_ready'])
            self.assertFalse(alsa_playback_status('hw:CARD=Device,DEV=0', root)['tts_device_ready'])
            info.unlink()
            self.assertFalse(alsa_playback_status(device, root)['tts_device_ready'])

    def test_plugin_or_unavailable_proc_is_unknown_not_false(self):
        with tempfile.TemporaryDirectory() as folder:
            for device in (None, 'default', 'pulse', 'custom_output'):
                self.assertIsNone(alsa_playback_status(device, folder)['tts_device_ready'])
            self.assertIsNone(alsa_playback_status('plughw:2,0', Path(folder)/'absent')['tts_device_ready'])


class SpeechOutputNodeTests(unittest.TestCase):
    setUp = lifecycle.VoiceCaptureLifecycleTests.setUp
    make_voice = lifecycle.VoiceCaptureLifecycleTests.make_voice

    @classmethod
    def setUpClass(cls):
        lifecycle.VoiceCaptureLifecycleTests.setUpClass.__func__(cls)

    def worker_once(self, voice):
        original = voice.speech_queue.get
        def once(timeout):
            if voice.speech_queue.empty():
                voice.stop_event.set()
                raise queue.Empty
            return original(timeout=timeout)
        voice.speech_queue.get = once
        voice.stop_event.wait = lambda _: False
        voice.speech_worker()

    def test_missing_device_does_not_synthesize_play_or_abort_mic(self):
        voice = self.make_voice()
        voice.config.update(tts_enabled=True, tts_backend='sherpa_matcha',
            tts_prewarm=True, tts_audio_device='plughw:CARD=RescueSpeaker,DEV=0')
        voice.speech_queue.put(('本地语音回答', time.monotonic()+10))
        status = dict(tts_device_ready=False, tts_audio_device=voice.config['tts_audio_device'],
                      reason='audio_output_device_missing')
        with patch('rescue_vision.voice_tts_process.alsa_playback_status', return_value=status), \
             patch('rescue_vision.voice_tts_process.ProcessNeuralTTS') as model, \
             patch.object(self.module.subprocess, 'Popen') as play:
            self.worker_once(voice)
        model.assert_not_called(); play.assert_not_called()
        self.assertFalse(voice.speaking.is_set()); self.assertFalse(voice.abort_listen.is_set())
        voice.heartbeat()
        evidence = voice.status_pub.values[-1]
        self.assertEqual(evidence['state'], 'tts_unavailable')
        self.assertFalse(evidence['tts_device_ready'])
        self.assertIn('audio_output_device_missing', evidence['tts_error'])
        self.assertEqual(evidence['tts_last_result']['text'], '本地语音回答')

    def test_heartbeat_does_not_call_synthesis_actual_playback(self):
        voice = self.make_voice(); voice.speaking.set()
        voice.speech_status('synthesizing', text='分析当前场景')
        voice.heartbeat()
        self.assertEqual(voice.status_pub.values[-1]['state'], 'synthesizing')
        voice.speech_status('playing', text='分析当前场景')
        voice.heartbeat()
        self.assertEqual(voice.status_pub.values[-1]['state'], 'speaking')

    def test_expired_queue_item_is_visible_and_never_played(self):
        voice = self.make_voice(); voice.config['tts_prewarm'] = False
        voice.speech_queue.put(('过期回答', time.monotonic()-1))
        with patch.object(self.module.subprocess, 'Popen') as play:
            self.worker_once(voice)
        play.assert_not_called(); voice.heartbeat()
        self.assertEqual(voice.status_pub.values[-1]['tts_last_result']['reason'], 'speech_expired_before_start')

    def test_replug_is_rechecked_and_same_config_recovers_without_restart(self):
        voice = self.make_voice(); voice.config['tts_audio_device'] = 'plughw:CARD=RescueSpeaker,DEV=0'
        missing = dict(tts_device_ready=False, tts_audio_device=voice.config['tts_audio_device'],
                       reason='audio_output_device_missing')
        present = dict(missing, tts_device_ready=True, reason='')
        with patch('rescue_vision.voice_tts_process.alsa_playback_status', side_effect=[missing, present]):
            with self.assertRaisesRegex(RuntimeError, 'audio_output_device_missing'):
                voice.check_speech_output()
            voice.check_speech_output()
        self.assertTrue(voice.speech_diagnostics['tts_device_ready'])

    def test_unplug_during_synthesis_prevents_playback_and_reports_missing_device(self):
        voice = self.make_voice()
        voice.config.update(tts_enabled=True, tts_backend='sherpa_matcha', tts_prewarm=False)
        voice.speech_queue.put(('测试回答', time.monotonic()+10))
        voice.neural_tts = Mock(); voice.neural_tts.synthesize.return_value = (b'fake wav', dict(duration_s=.1))
        with patch.object(voice, 'check_speech_output', side_effect=[None, RuntimeError('audio_output_device_missing')]), \
             patch.object(self.module.shutil, 'which', return_value='/usr/bin/aplay'), \
             patch.object(self.module.subprocess, 'Popen') as play:
            self.worker_once(voice)
        play.assert_not_called(); self.assertFalse(voice.speaking.is_set())
        voice.heartbeat()
        self.assertEqual(voice.status_pub.values[-1]['tts_error'], 'audio_output_device_missing')

    def queued_long_reply(self, voice):
        voice.config.update(tts_enabled=True,tts_backend='sherpa_matcha',tts_prewarm=False)
        text='以下基于提问时快照：'+('建议停稳后复核当前火源与人物位置，不把图案当作受困证据。'*16)+'以上为观察建议，未执行控制。'
        voice.say(types.SimpleNamespace(data=text))
        self.assertEqual(voice.speech_queue.qsize(),1)
        return text

    def test_long_answer_is_one_job_and_every_part_is_played_in_order(self):
        voice=self.make_voice();text=self.queued_long_reply(voice)
        voice.neural_tts=Mock();voice.neural_tts.synthesize.return_value=(b'fake wav',dict(duration_s=.1,synthesis_s=.2))
        player=Mock(returncode=0);player.communicate.return_value=(b'',b'');player.poll.return_value=0
        with patch.object(voice,'check_speech_output'), \
             patch.object(self.module.shutil,'which',return_value='/usr/bin/aplay'), \
             patch.object(self.module.subprocess,'Popen',return_value=player) as play:
            self.worker_once(voice)
        chunks=[call.args[0] for call in voice.neural_tts.synthesize.call_args_list]
        self.assertEqual(''.join(chunks),text)
        self.assertGreater(len(chunks),1)
        self.assertEqual(play.call_count,len(chunks))
        final=voice.speech_diagnostics['tts_last_result']
        self.assertEqual(final['state'],'spoken');self.assertEqual(final['text'],text)
        self.assertEqual(final['chunks_completed'],len(chunks))
        self.assertAlmostEqual(final['duration_s'],len(chunks)*.1)
        self.assertFalse(voice.speaking.is_set())
        self.assertFalse(any(v.get('tts_state')=='spoken' for v in voice.status_pub.values[:-1]))

    def test_unplug_after_first_part_stops_answer_without_replaying_after_replug(self):
        voice=self.make_voice();text=self.queued_long_reply(voice)
        voice.neural_tts=Mock();voice.neural_tts.synthesize.return_value=(b'fake wav',dict(duration_s=.1))
        player=Mock(returncode=0);player.communicate.return_value=(b'',b'');player.poll.return_value=0
        with patch.object(voice,'check_speech_output',side_effect=[None,None,RuntimeError('audio_output_device_missing')]), \
             patch.object(self.module.shutil,'which',return_value='/usr/bin/aplay'), \
             patch.object(self.module.subprocess,'Popen',return_value=player) as play:
            self.worker_once(voice)
        self.assertEqual(play.call_count,1)
        self.assertEqual(voice.neural_tts.synthesize.call_count,1)
        final=voice.speech_diagnostics['tts_last_result']
        self.assertEqual(final['state'],'unavailable');self.assertEqual(final['text'],text)
        self.assertEqual(final['chunks_completed'],1);self.assertTrue(voice.speech_queue.empty())
        self.assertFalse(voice.speaking.is_set())

    def test_stop_during_multi_part_playback_never_claims_full_answer_spoken(self):
        voice=self.make_voice();text=self.queued_long_reply(voice)
        def play_chunk(*args):
            voice.stop_event.set()
            return None
        with patch.object(voice,'check_speech_output'),patch.object(voice,'play_speech_chunk',side_effect=play_chunk) as play:
            self.worker_once(voice)
        self.assertEqual(play.call_count,1)
        self.assertEqual(voice.speech_diagnostics['tts_last_result']['state'],'interrupted')
        self.assertEqual(voice.speech_diagnostics['tts_last_result']['text'],text)
        self.assertFalse(voice.speaking.is_set());self.assertTrue(voice.speech_queue.empty())

    def test_oversize_answer_has_visible_error_instead_of_silent_drop(self):
        voice=self.make_voice();voice.say(types.SimpleNamespace(data='字'*901))
        self.assertTrue(voice.speech_queue.empty())
        self.assertEqual(voice.speech_diagnostics['tts_last_result']['reason'],'speech_text_requires_1_to_900_characters')

    def test_stopping_native_synthesis_is_interruption_not_a_device_failure(self):
        voice=self.make_voice();self.queued_long_reply(voice)
        def cancelled(*args):
            voice.stop_event.set()
            raise RuntimeError('TTS worker cancelled')
        with patch.object(voice,'check_speech_output'),patch.object(voice,'play_speech_chunk',side_effect=cancelled):
            self.worker_once(voice)
        self.assertEqual(voice.speech_diagnostics['tts_last_result']['state'],'interrupted')
        self.assertEqual(voice.speech_diagnostics['tts_last_result']['reason'],'speech_cancelled')
        self.assertFalse(voice.speaking.is_set());self.assertTrue(voice.speech_queue.empty())

    def test_three_complete_capture_and_long_answer_cycles_allow_the_next_question(self):
        """Exercise the shared recording/TTS flags rather than each worker alone."""
        voice = self.make_voice()
        voice.config.update(tts_enabled=True, tts_backend='sherpa_matcha', tts_prewarm=False)
        voice.neural_tts = Mock()
        voice.neural_tts.synthesize.return_value = (b'fake wav', dict(duration_s=.1))
        player = Mock(returncode=0)
        player.communicate.return_value = (b'', b'')
        player.poll.return_value = 0
        # Stop only this synchronous test scheduler when idle. Do not request
        # application shutdown, so the following recording uses the same node.
        class WorkerDrained(BaseException):
            pass
        original_get = voice.speech_queue.get
        def get_next(timeout):
            if voice.speech_queue.empty():
                raise WorkerDrained()
            return original_get(timeout=timeout)
        with patch.object(voice, 'check_speech_output'), \
             patch.object(self.module.shutil, 'which', return_value='/usr/bin/aplay'), \
             patch.object(self.module.subprocess, 'Popen', return_value=player), \
             patch.object(voice.speech_queue, 'get', side_effect=get_next), \
             patch.object(voice.stop_event, 'wait', return_value=False):
            for index in range(3):
                voice.listen(types.SimpleNamespace(data=True))
                self.assertTrue(voice.listening)
                self.assertFalse(voice.abort_listen.is_set())
                voice.mic_thread.target()
                self.assertEqual(len(voice.command_pub.values), index + 1)
                self.assertTrue(voice.capture_stop_confirmed)
                text = self.queued_long_reply(voice)
                start = voice.neural_tts.synthesize.call_count
                with self.assertRaises(WorkerDrained):
                    voice.speech_worker()
                parts = [call.args[0] for call in voice.neural_tts.synthesize.call_args_list[start:]]
                self.assertEqual(''.join(parts), text)
                self.assertEqual(voice.speech_diagnostics['tts_last_result']['state'], 'spoken')
                self.assertFalse(voice.speaking.is_set())
                self.assertFalse(voice.stop_event.is_set())
                self.assertTrue(voice.speech_queue.empty())
        self.assertEqual(self.model_calls, 1)
        self.assertEqual(len(voice.transcript_pub.values), 3)
        self.assertEqual(len({c['command_id'] for c in voice.command_pub.values}), 3)

    def test_synthesis_failure_is_reported_and_next_queued_answer_still_plays(self):
        voice = self.make_voice()
        voice.config.update(tts_enabled=True, tts_backend='sherpa_matcha', tts_prewarm=False)
        voice.neural_tts = Mock()
        voice.neural_tts.synthesize.side_effect = [TimeoutError('fixture synthesis timeout'),
                                                  (b'fake wav', dict(duration_s=.1))]
        voice.say(types.SimpleNamespace(data='第一条回答'))
        voice.say(types.SimpleNamespace(data='第二条回答'))
        player = Mock(returncode=0)
        player.communicate.return_value = (b'', b'')
        player.poll.return_value = 0
        with patch.object(voice, 'check_speech_output'), \
             patch.object(self.module.shutil, 'which', return_value='/usr/bin/aplay'), \
             patch.object(self.module.subprocess, 'Popen', return_value=player) as play:
            self.worker_once(voice)
        failures = [value for value in voice.status_pub.values if value.get('state') == 'tts_unavailable']
        self.assertEqual(failures[-1]['text'], '第一条回答')
        self.assertEqual(failures[-1]['reason'], 'fixture synthesis timeout')
        self.assertEqual(voice.speech_diagnostics['tts_last_result']['state'], 'spoken')
        self.assertEqual(voice.speech_diagnostics['tts_last_result']['text'], '第二条回答')
        play.assert_called_once()
        self.assertFalse(voice.speaking.is_set())

    def test_replug_next_answer_plays_without_replaying_the_missing_device_answer(self):
        voice = self.make_voice()
        voice.config.update(tts_enabled=True, tts_backend='sherpa_matcha', tts_prewarm=False)
        voice.neural_tts = Mock()
        voice.neural_tts.synthesize.return_value = (b'fake wav', dict(duration_s=.1))
        voice.say(types.SimpleNamespace(data='没有音响时的回答'))
        voice.say(types.SimpleNamespace(data='音响恢复后的新回答'))
        player = Mock(returncode=0)
        player.communicate.return_value = (b'', b'')
        player.poll.return_value = 0
        with patch.object(voice, 'check_speech_output', side_effect=[
                RuntimeError('audio_output_device_missing'), None, None]), \
             patch.object(self.module.shutil, 'which', return_value='/usr/bin/aplay'), \
             patch.object(self.module.subprocess, 'Popen', return_value=player) as play:
            self.worker_once(voice)
        voice.neural_tts.synthesize.assert_called_once_with('音响恢复后的新回答')
        play.assert_called_once()
        self.assertEqual(voice.speech_diagnostics['tts_last_result']['state'], 'spoken')
        self.assertEqual(voice.speech_diagnostics['tts_last_result']['text'], '音响恢复后的新回答')
        self.assertTrue(voice.speech_queue.empty())


if __name__ == '__main__':
    unittest.main()
