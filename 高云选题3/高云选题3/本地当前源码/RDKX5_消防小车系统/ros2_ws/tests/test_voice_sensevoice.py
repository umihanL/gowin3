"""SenseVoice routing and same-PCM agreement, with no microphones or native ASR."""
import json
from pathlib import Path
import struct
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rescue_vision'))
from rescue_vision.voice_capture import StreamingASR, combine_asr_results
from rescue_vision.voice_verification import recognized_text, needs_independent_verification, verify_command
from rescue_vision.audio_test import diagnostic_transcript
import test_voice_capture_lifecycle as lifecycle


def result(text='现在有几个人', **changes):
    data=dict(text=text,result=[],confidence=None,confidence_available=False,
        engine='sensevoice-small-int8-sherpa-onnx',asr_backend='sensevoice',
        speech_guard='silero_vad',speech_verified=True,vad_speech_detected=True,
        decode_s=.4, speech_s=.1, audio_s=.1, vad_s=.01, decode_audio_s=.1,
        segments=[dict(start_s=0.,end_s=.1)])
    data.update(changes);return data


class SenseVoiceEvidenceTests(unittest.TestCase):
    def test_final_metadata_and_no_confidence_are_preserved(self):
        combined=combine_asr_results([dict(text='',result=[]),result()])
        self.assertIsNone(combined['confidence'])
        self.assertFalse(combined['confidence_available'])
        self.assertEqual(combined['speech_s'],.1)
        self.assertEqual(combined['segments'],[dict(start_s=0.,end_s=.1)])
        self.assertEqual(recognized_text(combined),('现在有几个人',None))
        d=diagnostic_transcript([result()],.65)
        self.assertTrue(d['transcription_accepted'])
        self.assertFalse(d['confidence_gate_passed'])
        self.assertIsNone(d['confidence'])
        self.assertEqual(d['engine'],'sensevoice-small-int8-sherpa-onnx')
        combined=combine_asr_results([result('保留前文'),result('',speech_s=0.,decode_s=.05)])
        self.assertEqual(combined['text'],'保留前文')
        self.assertAlmostEqual(combined['decode_s'],.45)
    def test_missing_or_wrong_vad_never_turns_unscored_text_into_command(self):
        for changes in ({'speech_verified':False},{'vad_speech_detected':False},
                {'speech_guard':'nonzero_pcm_only'},{'asr_backend':'unknown'},{'text':''}):
            with self.assertRaises(ValueError):recognized_text(result(**changes))
    def test_only_precise_readonly_and_stop_modes_bypass_independent_motion_check(self):
        for text in ('前进','回退20厘米','开始巡逻','自动找火灭火','开始喷水','锁定第2个火焰','切换自动模式'):
            self.assertTrue(needs_independent_verification(text,result(text),None),text)
        for text in ('现在有几个人','观察','分析当前火场','报告场景','停车','停止喷水','任务结束','切换手动模式'):
            self.assertFalse(needs_independent_verification(text,result(text),None),text)
    def test_canonical_alias_agreement_retains_distance_and_direction_and_confidence(self):
        safe=verify_command('回退二十厘米',dict(text='后退20公分',result=[{'conf':.9}]),None)
        self.assertTrue(safe['verified'])
        for text,conf in (('前进20厘米',.99),('后退30厘米',.99),('不要后退',.99),('后退20厘米',.64),('后退20厘米',None)):
            wrong=verify_command('回退20厘米',dict(text=text,result=[{'conf':conf}]),None)
            self.assertFalse(wrong['verified'],(text,conf))
            self.assertEqual(wrong['reason'],'voice_command_disagreement')
    def test_verifier_never_shortcuts_endpoint_stop_or_discards_final_correction(self):
        class Recognizer:
            def AcceptWaveform(self,pcm):return True
            def Result(self):return json.dumps(dict(text='停止喷水',result=[{'conf':.99}]))
            def FinalResult(self):return json.dumps(dict(text='开始喷水',result=[{'conf':.99}]))
        decoder=StreamingASR(Recognizer(),'verification',.65)
        decoder.submit(b'\0\0');decoder.finish();decoder.start()
        self.assertTrue(decoder.done.wait(2.))
        self.assertIsNone(decoder.emergency_result)
        self.assertEqual(decoder.combined()['text'],'停止喷水 开始喷水')
    def test_cancel_before_start_completes_without_calling_native_recognizer(self):
        recognizer=types.SimpleNamespace(AcceptWaveform=lambda pcm: (_ for _ in ()).throw(AssertionError('native called')))
        decoder=StreamingASR(recognizer,'diagnostic',.65)
        decoder.submit(b'\0\0');decoder.cancel()
        self.assertTrue(decoder.done.is_set())
        decoder.start()
        self.assertFalse(decoder.started.is_set())


class SenseVoiceNodeTests(unittest.TestCase):
    make_voice=lifecycle.VoiceCaptureLifecycleTests.make_voice
    @classmethod
    def setUpClass(cls):lifecycle.VoiceCaptureLifecycleTests.setUpClass.__func__(cls)
    def setUp(self):
        lifecycle.VoiceCaptureLifecycleTests.setUp(self)
        self.config['asr_backend']='sensevoice'
        self.primary_result=result()
        self.secondary_text='现在有几个人'
        self.secondary_conf=.9
        self.loaded=[];self.primary_pcm=[];self.secondary_pcm=[]
        outer=self
        class PrimaryRecognizer:
            def SetWords(self,enabled):pass
            def AcceptWaveform(self,pcm):outer.primary_pcm.append(pcm);return False
            def FinalResult(self):return json.dumps(outer.primary_result)
        class SecondaryRecognizer:
            def SetWords(self,enabled):pass
            def AcceptWaveform(self,pcm):outer.secondary_pcm.append(pcm);return False
            def FinalResult(self):return json.dumps(dict(text=outer.secondary_text,result=[{'conf':outer.secondary_conf}]))
        def load(config,workspace,**kwargs):
            name=config.get('asr_backend');self.loaded.append(name)
            return types.SimpleNamespace(asr_backend=name,
                engine='sensevoice-small-int8-sherpa-onnx' if name=='sensevoice' else 'vosk-small-cn-0.22',
                confidence_available=name=='vosk',create_recognizer=lambda rate:
                    PrimaryRecognizer() if name=='sensevoice' else SecondaryRecognizer())
        loader=patch.object(self.module,'load_asr_backend',side_effect=load)
        loader.start();self.addCleanup(loader.stop)
    def run_voice(self,voice=None):
        voice=voice or self.make_voice()
        voice.listen(types.SimpleNamespace(data=True));voice.mic_thread.target();return voice

    def test_multiple_verified_motion_utterances_reuse_models_but_not_decoders(self):
        voice=self.make_voice()
        self.primary_result=result('前进20厘米')
        self.secondary_text='前进二十厘米'
        for index in range(4):
            self.run_voice(voice)
            self.assertEqual(len(voice.command_pub.values),index+1)
            self.assertTrue(voice.transcript_pub.values[-1]['verification']['verified'])
            self.assertIsNone(voice.asr_decoder)
            self.assertFalse(voice.listening)
        self.assertEqual(self.loaded,['sensevoice','vosk'])
        self.assertEqual(self.primary_pcm,self.secondary_pcm)

    def test_verifier_process_preparation_obeys_capture_deadline(self):
        from rescue_vision.voice_asr_process import ProcessASRBackend
        from test_voice_asr_process import fake_native_worker
        voice = self.make_voice()
        started = time.monotonic()
        def load(config, workspace, **kwargs):
            return ProcessASRBackend(dict(config, startup_delay=60.), workspace,
                worker_target=fake_native_worker, **kwargs)
        with patch.object(self.module, 'load_asr_backend', side_effect=load):
            with self.assertRaisesRegex(RuntimeError, 'transcription_timeout'):
                voice.verify_voice_action('前进20厘米', result('前进20厘米'), [b'pcm'], started+.2)
        self.assertLess(time.monotonic()-started, 2.)
        self.assertIsNone(voice.verification_backend)

    def test_cached_verifier_refreshes_expired_budget_and_ignores_manual_end(self):
        from rescue_vision.voice_asr_process import ProcessASRBackend
        from test_voice_asr_process import fake_native_worker
        voice = self.make_voice()
        backend = ProcessASRBackend({'response_text': '前进20厘米'}, ROOT,
            worker_target=fake_native_worker)
        self.addCleanup(backend.close)
        voice.verification_backend = backend
        backend.preparation_cancelled = lambda: (_ for _ in ()).throw(RuntimeError('stale deadline'))
        voice.end_listen.set()
        for _ in range(2):
            verified = voice.verify_voice_action('前进20厘米', result('前进20厘米'), [b'pcm'], time.monotonic()+3.)
            self.assertTrue(verified['verified'])
            backend.preparation_cancelled = lambda: (_ for _ in ()).throw(RuntimeError('stale deadline'))
        self.assertEqual(backend._session, 2)

    def test_verifier_create_is_bounded_and_next_recording_can_retry(self):
        from rescue_vision.voice_asr_process import ProcessASRBackend
        from test_voice_asr_process import fake_native_worker
        voice = self.make_voice()
        backend = ProcessASRBackend({'response_text': '前进20厘米', 'create_delay': 60.}, ROOT,
            worker_target=fake_native_worker)
        self.addCleanup(backend.close)
        voice.verification_backend = backend
        started = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, 'transcription_timeout'):
            voice.verify_voice_action('前进20厘米', result('前进20厘米'), [b'pcm'], started+.2)
        self.assertLess(time.monotonic()-started, 2.)
        self.assertFalse(backend.ready)
        backend.config.pop('create_delay')
        self.assertTrue(voice.verify_voice_action('前进20厘米', result('前进20厘米'), [b'pcm'],
            time.monotonic()+3.)['verified'])

    def test_timed_out_verifier_cancels_native_owner_then_next_utterance_recovers(self):
        voice=self.make_voice()
        self.primary_result=result('前进20厘米')
        self.secondary_text='前进二十厘米'
        from rescue_vision.voice_capture import StreamingASR
        instances=[]
        class ReclaimableVerifier:
            def __init__(inner):
                inner.done=threading.Event();inner.cancelled=False
                inner.error=None;instances.append(inner)
            def submit(inner,pcm):pass
            def finish(inner):pass
            def start(inner):pass
            def cancel(inner):inner.cancelled=True;inner.done.set()
        def factory(recognizer,mode,threshold):
            if mode=='verification':return ReclaimableVerifier()
            return StreamingASR(recognizer,mode,threshold)
        with patch.object(self.module,'StreamingASR',side_effect=factory), \
                patch.object(voice,'wait_decoder',side_effect=RuntimeError('transcription_timeout')):
            self.run_voice(voice)
        self.assertEqual(voice.command_pub.values,[])
        self.assertEqual(voice.transcript_pub.values[-1]['route_error'],'transcription_timeout')
        self.assertTrue(instances[0].cancelled)
        self.assertIsNone(voice.asr_decoder)
        self.assertFalse(voice.listening)
        self.run_voice(voice)
        self.assertEqual(len(voice.command_pub.values),1)
        self.assertTrue(voice.transcript_pub.values[-1]['submitted'])
    def test_readonly_unscored_vad_text_runs_without_fake_confidence_or_verifier(self):
        voice=self.run_voice()
        self.assertEqual(self.loaded,['sensevoice'])
        self.assertEqual(len(voice.command_pub.values),1)
        transcript=voice.transcript_pub.values[-1]
        self.assertIsNone(transcript['confidence'])
        self.assertFalse(transcript['confidence_available'])
        self.assertFalse(transcript['verification']['required'])
        self.assertEqual(transcript['decode_s'],.4)
        self.assertEqual(voice.status_pub.values[-1]['recording_levels']['level_samples'],1600)
        self.assertTrue(voice.status_pub.values[-1]['asr_ready'])
        self.run_voice(voice)
        self.assertEqual(self.loaded,['sensevoice'],'main model must remain cached')
    def test_motion_same_pcm_needs_independent_full_command_agreement(self):
        self.primary_result=result('回退二十厘米');self.secondary_text='后退20公分'
        voice=self.run_voice()
        self.assertEqual(self.loaded,['sensevoice','vosk'])
        self.assertEqual(self.primary_pcm,self.secondary_pcm)
        self.assertEqual(voice.command_pub.values[-1]['action'],'manual_move')
        self.assertTrue(voice.transcript_pub.values[-1]['verification']['verified'])
        self.run_voice(voice)
        self.assertEqual(self.loaded,['sensevoice','vosk'],'both models must remain cached')
    def test_disagreement_keeps_main_transcript_visible_and_publishes_no_action(self):
        self.primary_result=result('前进20厘米');self.secondary_text='后退20厘米'
        voice=self.run_voice()
        self.assertEqual(voice.command_pub.values,[])
        transcript=voice.transcript_pub.values[-1]
        self.assertEqual(transcript['text'],'前进20厘米')
        self.assertFalse(transcript['submitted'])
        self.assertEqual(transcript['route_error'],'voice_command_disagreement')
        self.assertEqual(transcript['verification']['text'],'后退20厘米')
    def test_low_secondary_score_never_publishes_motion(self):
        self.primary_result=result('开始喷水');self.secondary_text='开始喷水';self.secondary_conf=.2
        voice=self.run_voice()
        self.assertEqual(voice.command_pub.values,[])
        self.assertEqual(voice.transcript_pub.values[-1]['route_error'],'voice_command_disagreement')
    def test_stop_does_not_wait_for_independent_model(self):
        self.primary_result=result('停止喷水')
        voice=self.run_voice()
        self.assertEqual(self.loaded,['sensevoice'])
        self.assertEqual(voice.command_pub.values[-1]['action'],'stop_spray')
    def test_diagnostic_reuses_backend_and_never_routes_even_a_spray_phrase(self):
        voice=self.run_voice();voice.command_pub.values.clear();voice.transcript_pub.values.clear()
        self.primary_result=result('开始喷水')
        voice.audio_test_request(types.SimpleNamespace(data=json.dumps(dict(enabled=True,
            command_id='diagnostic',issued_at=10.,ttl_s=3.))))
        voice.mic_thread.target()
        final=voice.audio_test_pub.values[-1]
        self.assertEqual(final['state'],'completed')
        self.assertEqual(final['text'],'开始喷水')
        self.assertIsNone(final['confidence']);self.assertFalse(final['confidence_gate_passed'])
        self.assertTrue(final['transcription_accepted'])
        self.assertEqual(voice.command_pub.values,[]);self.assertEqual(voice.transcript_pub.values,[])
        self.assertEqual(self.loaded,['sensevoice'])
        self.assertEqual(final['recording_levels']['level_samples'],1600)
    def test_cancel_or_timeout_during_verification_never_submits_or_accumulates(self):
        self.primary_result=result('前进')
        voice=self.make_voice();voice.listen(types.SimpleNamespace(data=True))
        def verify(*args):voice.abort_listen.set();raise RuntimeError('playback_or_shutdown_during_transcription')
        with patch.object(voice,'verify_voice_action',side_effect=verify):voice.mic_thread.target()
        self.assertEqual(voice.command_pub.values,[]);self.assertEqual(voice.transcript_pub.values,[])
        self.assertEqual(voice.status_pub.values[-1]['state'],'audio_discarded')
    def test_timed_out_secondary_retains_worker_ownership_until_native_return(self):
        self.primary_result=result('前进')
        voice=self.make_voice();voice.listen(types.SimpleNamespace(data=True))
        primary_factory=self.module.StreamingASR
        secondary=types.SimpleNamespace(done=threading.Event(),submit=lambda pcm:None,
            finish=lambda:None,start=lambda:None,cancel=lambda:None)
        def factory(recognizer,mode,confidence):
            return secondary if mode=='verification' else primary_factory(recognizer,mode,confidence)
        with patch.object(self.module,'StreamingASR',side_effect=factory), \
                patch.object(voice,'wait_decoder',side_effect=RuntimeError('transcription_timeout')):
            voice.mic_thread.target()
        self.assertIs(voice.asr_decoder,secondary)
        self.assertEqual(voice.command_pub.values,[])
        self.assertEqual(voice.transcript_pub.values[-1]['text'],'前进')
        self.assertEqual(voice.transcript_pub.values[-1]['route_error'],'transcription_timeout')
        count=len(lifecycle.FakeThread.created)
        voice.listen(types.SimpleNamespace(data=True))
        self.assertEqual(len(lifecycle.FakeThread.created),count)
        self.assertEqual(voice.status_pub.values[-1]['reason'],'asr_decoder_still_stopping')
    def test_legacy_resume_aliases_never_bypass_unscored_primary_gate(self):
        for text in ('继续','恢复行驶','继续导航','去3号楼'):
            self.primary_result=result(text)
            voice=self.make_voice();voice.task_commands=False
            self.run_voice(voice)
            self.assertEqual(voice.command_pub.values,[],text)
            self.assertEqual(voice.transcript_pub.values[-1]['route_error'],
                             'unscored_asr_requires_task_commands',text)
        self.assertNotIn('vosk',self.loaded)
    def test_diagnostic_cancel_after_recognizer_creation_releases_unstarted_owner(self):
        voice=self.make_voice()
        backend=voice.load_backend();original=backend.create_recognizer
        def create(rate):
            recognizer=original(rate);voice.abort_listen.set();return recognizer
        backend.create_recognizer=create
        voice.audio_test_request(types.SimpleNamespace(data=json.dumps(dict(enabled=True,
            command_id='cancel-before-start',issued_at=10.,ttl_s=3.))))
        voice.mic_thread.target()
        self.assertEqual(voice.audio_test_pub.values[-1]['state'],'cancelled')
        self.assertIsNone(voice.asr_decoder)
        voice.listen(types.SimpleNamespace(data=True))
        self.assertTrue(voice.listening)


if __name__=='__main__':unittest.main(verbosity=2)
