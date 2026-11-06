"""Exercise real microphone/text/HTTP ingress without physical motion or a model server."""
import http.client
import json
import threading
import time
import unittest
from unittest.mock import Mock, patch

import test_voice_capture_lifecycle as capture_fixture
from test_presentation_server import module as presentation, WS


class RuleVoiceIngressTests(unittest.TestCase):
    setUpClass = classmethod(capture_fixture.VoiceCaptureLifecycleTests.setUpClass.__func__)
    setUp = capture_fixture.VoiceCaptureLifecycleTests.setUp
    make_voice = capture_fixture.VoiceCaptureLifecycleTests.make_voice

    def verify(self, primary, secondary, confidence=.95, primary_scored=True):
        voice=self.make_voice()
        voice.verification_backend=Mock()
        decoder=Mock()
        decoder.done.is_set.return_value=True
        decoder.combined.return_value=dict(text=secondary,result=[dict(conf=confidence)],confidence_available=True)
        with patch.object(self.module,'StreamingASR',return_value=decoder), patch.object(voice,'wait_decoder'):
            result=voice.verify_voice_action(primary,dict(confidence_available=primary_scored),
                                            [b'\0\0'*1600],time.monotonic()+20.)
        voice.verification_backend.create_recognizer.assert_called_once_with(16000)
        decoder.submit.assert_called_once();decoder.finish.assert_called_once();decoder.start.assert_called_once()
        return voice,result

    def test_scored_primary_still_requires_independent_vosk_for_numbered_navigation(self):
        sentence='你现在在8号点位，车头朝左，请导航前往至4号点位。'
        _,result=self.verify(sentence,'你 现在 在 八 号 点 位 车头 朝 左 请 前往 四 号 点 位')
        self.assertTrue(result['required']);self.assertTrue(result['verified'],result)
        self.assertEqual(result['engine'],'vosk-small-cn-0.22')

    def test_noncanonical_navigation_sentence_is_not_exempt_from_verification(self):
        sentence='小车目前位于8号点位，车头朝向左方，请帮我开往4号点位'
        voice,result=self.verify(sentence,sentence,primary_scored=False)
        self.assertTrue(result['verified'],result)
        outcome=voice.accept_text(sentence,'voice',10.,verification=result)
        self.assertEqual(outcome['state'],'command_submitted')
        command=voice.command_pub.values[-1]
        self.assertEqual(command['action'],'navigate_point')
        self.assertEqual(command['text'],sentence)
        self.assertEqual(command['voice_verification'],result)
        self.assertEqual(command['target_point'],4)
        self.assertEqual(command['start_heading_deg'],180.)

    def test_independent_point_or_heading_disagreement_never_publishes_navigation(self):
        primary='你现在在8号点位，车头朝左，请前往4号点位'
        for secondary in ('你现在在8号点位，车头朝左，请前往3号点位',
                          '你现在在8号点位，车头朝右，请前往4号点位'):
            with self.subTest(secondary=secondary):
                voice,result=self.verify(primary,secondary)
                self.assertFalse(result['verified'])
                outcome=voice.accept_text(primary,'voice',10.,verification=result)
                self.assertEqual(outcome['reason'],'numbered_navigation_independent_asr_required')
                self.assertEqual(voice.command_pub.values,[])

    def test_low_independent_confidence_rejects_even_when_primary_is_scored(self):
        _,result=self.verify('从8号点前往4号点','从8号点前往4号点',confidence=.3)
        self.assertFalse(result['verified'])
        self.assertEqual(result['detail'],'low_asr_confidence')

    def test_voice_without_receipt_rejected_text_uses_rules_stop_is_immediate(self):
        voice=self.make_voice();sentence='你现在在8号点位请前往4号点位'
        self.assertEqual(voice.accept_text(sentence,'voice',10.)['state'],'command_rejected')
        self.assertEqual(voice.command_pub.values,[])
        self.assertEqual(voice.accept_text(sentence,'text',10.)['state'],'command_submitted')
        self.assertEqual(voice.command_pub.values[-1]['action'],'navigate_point')
        self.assertEqual(voice.accept_text('停车','voice',10.)['state'],'command_submitted')
        self.assertEqual(voice.command_pub.values[-1]['action'],'stop')

    def test_region_scored_and_unscored_speech_preserves_verified_authority_to_router(self):
        from rescue_vision.task_interaction import TaskRouter
        sentence='在A区域搜寻火源并执行灭火'
        for scored in (True,False):
            with self.subTest(scored=scored):
                voice,verification=self.verify(sentence,'请在诶区搜索火焰并自动灭火',primary_scored=scored)
                self.assertTrue(verification['verified'],verification)
                outcome=voice.accept_text(sentence,'voice',10.,verification=verification)
                self.assertEqual(outcome['state'],'command_submitted',outcome)
                command=voice.command_pub.values[-1]
                self.assertEqual(command['action'],'search_region')
                self.assertEqual(command['region_id'],'A')
                self.assertIs(command['execute_fire'],True)
                self.assertEqual(command['voice_verification'],verification)
                routed=TaskRouter(extension_enabled=True).route(command,10.,extension=dict(
                    stamp=10.,task='dialogue',dialogue_enabled=True,real_uart=True,status_fresh=True,region_ready=True))
                self.assertEqual(routed['extension']['action'],'search_region')
                self.assertEqual(routed['extension']['voice_verification'],verification)
                self.assertEqual(routed['actuator']['action'],'disarm')

    def test_search_only_voice_preserves_mode_without_gimbal_readiness(self):
        from rescue_vision.task_interaction import TaskRouter
        voice,verification=self.verify('去C区域搜索火焰','前往西区寻找火源')
        self.assertTrue(verification['verified'],verification)
        outcome=voice.accept_text('去C区域搜索火焰','voice',10.,verification=verification)
        self.assertEqual(outcome['state'],'command_submitted')
        command=voice.command_pub.values[-1]
        self.assertIs(command['execute_fire'],False)
        routed=TaskRouter(extension_enabled=True).route(command,10.,extension=dict(
            stamp=10.,task='dialogue',dialogue_enabled=True,real_uart=True,status_fresh=True,
            region_observe_ready=True,region_ready=False,region_reason='region_real_gimbal_reference_required'))
        self.assertIs(routed['extension']['execute_fire'],False)

    def test_region_or_water_disagreement_and_low_confidence_never_publish(self):
        primary='在A区域搜寻火源并执行灭火'
        for secondary,confidence in (('在B区域搜寻火源并执行灭火',.99),
                                     ('在A区域搜索火源',.99),
                                     ('不要在A区域搜寻火源并执行灭火',.99),
                                     (primary,.3)):
            with self.subTest(secondary=secondary,confidence=confidence):
                voice,verification=self.verify(primary,secondary,confidence=confidence)
                self.assertFalse(verification['verified'])
                outcome=voice.accept_text(primary,'voice',10.,verification=verification)
                self.assertEqual(outcome['reason'],'region_search_independent_asr_required')
                self.assertEqual(voice.command_pub.values,[])

    def test_region_missing_or_malformed_voice_proof_cannot_publish_but_text_and_stop_can(self):
        voice=self.make_voice();sentence='在D区域搜寻火源并执行灭火'
        for verification in (None,{},'verified',dict(verified=False)):
            outcome=voice.accept_text(sentence,'voice',10.,verification=verification)
            self.assertEqual(outcome['reason'],'region_search_independent_asr_required')
        self.assertEqual(voice.command_pub.values,[])
        self.assertEqual(voice.accept_text(sentence,'text',10.)['state'],'command_submitted')
        self.assertNotIn('voice_verification',voice.command_pub.values[-1])
        self.assertEqual(voice.accept_text('停止区域搜索','voice',10.)['state'],'command_submitted')
        self.assertEqual(voice.command_pub.values[-1]['action'],'stop')


class RuleHttpIngressTests(unittest.TestCase):
    def setUp(self):
        self.now=1000.
        self.state=presentation.PresentationState(monotonic=lambda:self.now,wall=lambda:self.now)
        self.state.ros.update(available=True,router_ready=True)
        self.state.update('mission',dict(stamp=self.now,mode='inspect'))
        self.state.update('extension',dict(stamp=self.now,real_uart=True,status_fresh=True,
            vehicle_stopped=True,can_confirm_point_start=True,can_navigate_point=True,task='vision',dialogue_enabled=True))
        self.server=presentation.BoundedServer(('127.0.0.1',0),presentation.BaseHTTPRequestHandler)
        self.host='127.0.0.1:'+str(self.server.server_address[1])
        self.server.RequestHandlerClass=presentation.handler_class(self.state,WS/'web',Mock(),{self.host})
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()

    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join()

    def post(self,path,payload):
        connection=http.client.HTTPConnection('127.0.0.1',self.server.server_address[1],timeout=3)
        connection.request('POST',path,json.dumps(payload),
            {'Content-Type':'application/json','Origin':'http://'+self.host,'Host':self.host})
        response=connection.getresponse();status=response.status;body=json.loads(response.read());connection.close()
        return status,body

    def test_natural_navigation_http_submits_same_rule_command_as_voice(self):
        sentence='你现在在8号点位，车头朝左，请导航前往至4号点位。'
        status,body=self.post('/api/command',dict(text=sentence,request_id='natural-navigation'))
        self.assertEqual(status,202,body)
        self.assertEqual(body['action'],'navigate_point')
        self.assertEqual(body['status'],'queued')
        sent=[];self.state.drain(lambda kind,value:sent.append(value),lambda:self.now)
        self.assertEqual(len(sent),1)
        self.assertEqual(sent[0]['action'],'navigate_point');self.assertEqual(sent[0]['text'],sentence)
        self.assertNotIn('manual_point_command',sent[0])
        self.assertEqual((sent[0]['start_point'],sent[0]['target_point'],sent[0]['start_heading_deg']),(8,4,180.))

    def test_manual_map_control_is_explicit_structured_command(self):
        status,body=self.post('/api/point-navigation',dict(action='navigate_point',start_point=8,target_point=4,
                                                      start_heading_deg=180.,request_id='manual-map'))
        self.assertEqual(status,202,body)
        sent=[];self.state.drain(lambda kind,value:sent.append(value),lambda:self.now)
        self.assertEqual(len(sent),1);self.assertIs(sent[0]['manual_point_command'],True)
        self.assertEqual(sent[0]['action'],'navigate_point')

    def test_region_http_keeps_explicit_region_and_water_intent(self):
        self.state.update('extension',dict(stamp=self.now,real_uart=True,status_fresh=True,
            task='dialogue',dialogue_enabled=True,region_ready=True))
        self.state.update('actuators',dict(stamp=self.now,dry_run=False,enabled=True,pump_enabled=True))
        sentence='在C区域搜寻火源并执行灭火'
        status,body=self.post('/api/command',dict(text=sentence,request_id='region-http'))
        self.assertEqual(status,202,body);self.assertEqual(body['action'],'search_region')
        sent=[];self.state.drain(lambda kind,value:sent.append(value),lambda:self.now)
        self.assertEqual(len(sent),1)
        self.assertEqual((sent[0]['region_id'],sent[0]['execute_fire']),('C',True))
        self.assertEqual(sent[0]['text'],sentence)

    def test_region_search_only_http_queues_without_formal_gimbal(self):
        self.state.update('extension',dict(stamp=self.now,real_uart=True,status_fresh=True,
            task='dialogue',dialogue_enabled=True,region_observe_ready=True,
            region_ready=False,region_reason='region_real_gimbal_reference_required'))
        self.state.update('actuators',dict(stamp=self.now,dry_run=True,enabled=False,pump_enabled=False))
        status,body=self.post('/api/command',dict(text='去C区域搜索火焰',request_id='region-search-only'))
        self.assertEqual(status,202,body)
        sent=[];self.state.drain(lambda kind,value:sent.append(value),lambda:self.now)
        self.assertEqual(len(sent),1)
        self.assertEqual(sent[0]['region_id'],'C')
        self.assertIs(sent[0]['execute_fire'],False)


if __name__=='__main__':unittest.main()
