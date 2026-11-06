"""Opt-in REAL ASR through production Voice, using synthetic WAV memory callbacks.

Never opens a microphone, speaker, UART or ROS graph. FakeNode captures every
publisher in memory. This validates software plumbing, NOT real human speech.

Example (X5, after sourcing workspace):
  RESCUE_RUN_REAL_ASR_TEST=1 ROS_DOMAIN_ID=85 ROS_LOCALHOST_ONLY=1 \
    python3 tests/test_sensevoice_board.py --wav reports/asr_upgrade_20261014/synthetic/case_00.wav
"""
import argparse
from contextlib import ExitStack
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch
import wave

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rescue_vision'))


class Publisher:
    def __init__(self):self.values=[]


class MemoryNode:
    def __init__(self,name):self.publishers={}
    def create_publisher(self,_type,topic,_depth):
        publisher=Publisher();self.publishers[topic]=publisher;return publisher
    def create_subscription(self,*args,**kwargs):return None
    def create_timer(self,*args,**kwargs):return None
    def get_clock(self):
        return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=time.time_ns()))
    def destroy_node(self):return None


class DeferredThread:
    def __init__(self,*,target,daemon):self.target=target
    def start(self):pass
    def join(self,timeout=None):pass


@unittest.skipUnless(os.environ.get('RESCUE_RUN_REAL_ASR_TEST')=='1','Explicit synthetic real-ASR test only')
class SenseVoiceBoardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID')!='85' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            raise AssertionError('Require ROS_DOMAIN_ID=85 ROS_LOCALHOST_ONLY=1 even though ROS is stubbed')
        wav_path=Path(os.environ['RESCUE_SYNTHETIC_WAV']).resolve()
        if wav_path.name!='case_00.wav' or wav_path.parent.name!='synthetic':
            raise AssertionError('This validation accepts the documented synthetic/case_00.wav only')
        with wave.open(str(wav_path),'rb') as wav:
            if wav.getsampwidth()!=2 or wav.getnchannels()!=1:
                raise AssertionError('Synthetic test WAV must be PCM16 mono')
            original_rate=wav.getframerate();raw=wav.readframes(wav.getnframes())
        import numpy as np
        from scipy.signal import resample_poly
        # Proper resampling preserves pitch and duration: no header-only rate change.
        source=np.frombuffer(raw,dtype='<i2').astype(np.float64)
        divisor=math.gcd(original_rate,48000)
        native=resample_poly(source,48000//divisor,original_rate//divisor)
        native=np.clip(np.rint(native),-32768,32767).astype('<i2')
        if native.size>48000*6:raise AssertionError('Synthetic fixture exceeds six-second capture')
        cls.fixture_seconds=len(source)/original_rate
        cls.pcm=np.pad(native,(0,48000*6-native.size)).astype('<i2').tobytes()
        # Import native dependencies and the production decoder before replacing
        # ROS modules/Thread constructors. The actual ASR work still uses threads.
        from rescue_vision.voice_capture import StreamingASR
        from rescue_vision.asr_backend import load_asr_backend
        cls.config=json.loads((ROOT/'config/voice.local.json').read_text(encoding='utf-8'))
        if cls.config.get('asr_backend')!='sensevoice':raise AssertionError('Production must select SenseVoice')
        if cls.config.get('capture_sample_rate_hz')!=48000:raise AssertionError('Production capture rate must be 48k')
        started=time.monotonic()
        cls.backend=load_asr_backend(cls.config,ROOT)
        cls.load_s=time.monotonic()-started
        cls.measurements=[]
        cls.wav_path=wav_path
    def setUp(self):
        self.stack=ExitStack();self.addCleanup(self.stack.close)
        config=dict(self.config,tts_enabled=False,microphone_enabled=True)
        common=types.ModuleType('rescue_vision.node_common')
        params=dict(workspace_dir=str(ROOT),task_commands=True)
        common.param=lambda node,name,default:params.get(name,default)
        common.read_json=lambda path:(json.loads((ROOT/'config/buildings.json').read_text(encoding='utf-8'))
            if str(path).endswith('buildings.json') else config.copy())
        common.publish_json=lambda publisher,data:publisher.values.append(json.loads(json.dumps(data,allow_nan=False)))
        common.spin=lambda factory:None
        rclpy=types.ModuleType('rclpy');rnode=types.ModuleType('rclpy.node');rnode.Node=MemoryNode
        messages=types.ModuleType('std_msgs.msg');messages.Bool=messages.String=types.SimpleNamespace
        self.stack.enter_context(patch.dict(sys.modules,{'rclpy':rclpy,'rclpy.node':rnode,
            'std_msgs':types.ModuleType('std_msgs'),'std_msgs.msg':messages,'rescue_vision.node_common':common}))
        spec=importlib.util.spec_from_file_location('rescue_vision._sensevoice_board_test',
            ROOT/'src/rescue_vision/rescue_vision/voice_node.py')
        self.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.module)
        self.stack.enter_context(patch.object(self.module,'load_asr_backend',return_value=self.backend))
        self.stack.enter_context(patch.object(self.module.subprocess,'Popen',side_effect=AssertionError('Device/process launch forbidden')))
        with patch.object(self.module.threading,'Thread',DeferredThread):self.voice=self.module.Voice()
        self.addCleanup(self.voice.destroy_node)
        outer=self
        class MemoryStream:
            def __init__(self,**kwargs):
                if kwargs['samplerate']!=48000 or kwargs['channels']!=1 or kwargs['dtype']!='int16':
                    raise AssertionError('Unexpected native audio format')
                self.callback=kwargs['callback'];self.closed=threading.Event();self.worker=None
            def __enter__(self):
                def feed():
                    for index in range(60):
                        if self.closed.wait(.1):return
                        block=outer.pcm[index*9600:(index+1)*9600]
                        self.callback(block,4800,None,None)
                    outer.voice.listen(types.SimpleNamespace(data=False))
                self.worker=threading.Thread(target=feed,daemon=True);self.worker.start();return self
            def __exit__(self,*args):
                self.closed.set();self.worker.join(timeout=2)
        self.stack.enter_context(patch.dict(sys.modules,{'sounddevice':types.SimpleNamespace(RawInputStream=MemoryStream)}))
    def verify_transcript(self,transcript):
        from rescue_vision.voice import normalize_voice_text
        normalized=normalize_voice_text(transcript['text'])
        self.assertIn('场景',normalized)
        self.assertIn('几团火焰',normalized)
        self.assertIsNone(transcript['confidence'])
        self.assertFalse(transcript['confidence_available'])
        self.assertTrue(transcript['speech_verified'])
        self.assertTrue(transcript['vad_speech_detected'])
        self.assertEqual(transcript['speech_guard'],'silero_vad')
        self.assertGreater(transcript['speech_s'],0.)
        self.assertGreater(transcript['decode_s'],0.)
    def test_real_primary_production_worker_synthetic_dialogue_only(self):
        with patch.object(self.module.threading,'Thread',DeferredThread):
            self.voice.listen(types.SimpleNamespace(data=True))
        started=time.monotonic();self.voice.mic_thread.target();wall_s=time.monotonic()-started
        self.assertTrue(self.voice.transcript_pub.values,self.voice.status_pub.values[-1])
        transcript=self.voice.transcript_pub.values[-1];self.verify_transcript(transcript)
        self.assertTrue(transcript['submitted']);self.assertEqual(transcript['action'],'dialogue')
        self.assertFalse(transcript['verification']['required'])
        self.assertEqual(len(self.voice.command_pub.values),1)
        self.assertEqual(self.voice.command_pub.values[0]['action'],'dialogue')
        state=self.voice.status_pub.values[-1]
        self.assertEqual(state['ended_by'],'manual_stop')
        self.assertAlmostEqual(state['sampled_s'],6.,places=3)
        self.assertEqual(state['recording_levels']['level_samples'],96000)
        self.assertFalse(self.voice.capture_active);self.assertFalse(self.voice.listening)
        self.assertLess(wall_s,36.)
        self.measurements.append(dict(mode='command',synthetic=True,human_validation=False,
            wall_s=wall_s,transcript=transcript,recording_levels=state['recording_levels']))
    def test_real_diagnostic_same_backend_never_dispatches_commands(self):
        request=dict(command_id='synthetic-asr-diagnostic',enabled=True,issued_at=time.time(),ttl_s=3.)
        with patch.object(self.module.threading,'Thread',DeferredThread):
            self.voice.audio_test_request(types.SimpleNamespace(data=json.dumps(request)))
        started=time.monotonic();self.voice.mic_thread.target();wall_s=time.monotonic()-started
        diagnostic=self.voice.audio_test_pub.values[-1]
        self.assertEqual(diagnostic['state'],'completed',diagnostic)
        self.verify_transcript(diagnostic)
        self.assertTrue(diagnostic['transcription_accepted'])
        self.assertFalse(diagnostic['confidence_gate_passed'])
        self.assertEqual(self.voice.command_pub.values,[]);self.assertEqual(self.voice.transcript_pub.values,[])
        self.assertAlmostEqual(diagnostic['sampled_s'],6.,places=3)
        self.assertEqual(diagnostic['recording_levels']['level_samples'],96000)
        self.assertLess(wall_s,36.)
        self.measurements.append(dict(mode='diagnostic',synthetic=True,human_validation=False,
            wall_s=wall_s,result=diagnostic))
    @classmethod
    def tearDownClass(cls):
        report=dict(synthetic=True,human_validation=False,no_microphone=True,no_speaker=True,
            no_uart=True,no_ros_graph=True,source_wav=str(cls.wav_path),source_duration_s=cls.fixture_seconds,
            backend_load_s=cls.load_s,measurements=cls.measurements)
        path=os.environ.get('RESCUE_SYNTHETIC_ASR_REPORT')
        if path:
            output=Path(path);output.parent.mkdir(parents=True,exist_ok=True)
            output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,allow_nan=False))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wav',required=True)
    parser.add_argument('--report')
    args=parser.parse_args()
    os.environ['RESCUE_SYNTHETIC_WAV']=args.wav
    if args.report:os.environ['RESCUE_SYNTHETIC_ASR_REPORT']=args.report
    unittest.main(argv=[sys.argv[0]],verbosity=2)
