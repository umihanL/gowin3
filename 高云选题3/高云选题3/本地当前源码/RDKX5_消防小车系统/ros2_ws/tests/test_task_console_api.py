"""Actual HTTP map/self-test ingress; no robot and no fabricated receipts."""
import http.client
import json
import unittest
import test_numbered_interaction as console_fixture


class TaskConsoleApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        console_fixture.NumberedHttpTests.setUpClass()
        cls.module=console_fixture.NumberedHttpTests.module
    setUp=console_fixture.NumberedHttpTests.setUp
    tearDown=console_fixture.NumberedHttpTests.tearDown
    post=console_fixture.NumberedHttpTests.post

    def gimbal_ready(self):
        self.state.ros['gimbal_self_test_ready']=True
        self.state.update('gimbal_self_test',dict(stamp=100.,can_start=True,phase='idle',ready_reason='ready'))

    def request(self,**changes):
        request=dict(request_id='self-test1',action='start',axes='both',amplitude_deg=1.,speed_deg_s=6.)
        request.update(changes)
        return request

    def test_start_reaches_separate_real_ros_queue(self):
        self.gimbal_ready()
        status,record=self.post(self.request(),'/api/gimbal-self-test')
        self.assertEqual(status,202);self.assertEqual(record['status'],'queued')
        sent=[]
        self.state.drain(lambda kind,data:sent.append((kind,data)),lambda:100.)
        self.assertEqual(sent[0][0],'gimbal_self_test')
        self.assertEqual(sent[0][1]['axes'],'both')
        self.assertNotIn('completed',self.state.commands['self-test1'])
        self.assertEqual(self.state.commands['self-test1']['status'],'submitted')

    def test_stop_preempts_pending_test_without_requiring_ready_angles(self):
        self.gimbal_ready()
        self.post(self.request(),'/api/gimbal-self-test')
        self.state.update('gimbal_self_test',dict(stamp=100.,can_start=False,ready_reason='encoder_feedback_stale'))
        status,_=self.post(dict(request_id='stop1',action='stop'),'/api/gimbal-self-test')
        self.assertEqual(status,202)
        sent=[];self.state.drain(lambda kind,data:sent.append((kind,data)),lambda:100.)
        self.assertEqual([data['action'] for _,data in sent],['stop'])
        self.assertEqual(self.state.commands['self-test1']['status'],'rejected')

    def test_stale_or_lost_readiness_cannot_start(self):
        self.gimbal_ready()
        self.post(self.request(),'/api/gimbal-self-test')
        self.state.update('gimbal_self_test',dict(stamp=95.,can_start=True))
        sent=[];self.state.drain(lambda *args:sent.append(args),lambda:100.)
        self.assertFalse(sent)
        self.assertEqual(self.state.commands['self-test1']['status'],'rejected')
        status,_=self.post(self.request(request_id='new1'),'/api/gimbal-self-test')
        self.assertEqual(status,409)

    def test_unknown_fields_extreme_angles_or_nonce_reuse_rejected(self):
        self.gimbal_ready()
        for changes in ({'amplitude_deg':2},{'amplitude_deg':True},{'speed_deg_s':40},
                        {'source':'web'},{'confirmed':True},{'axes':'all'}):
            status,_=self.post(self.request(**changes),'/api/gimbal-self-test')
            self.assertEqual(status,400,changes)
        self.post(self.request(),'/api/gimbal-self-test')
        status,_=self.post(self.request(axes='pan'),'/api/gimbal-self-test')
        self.assertEqual(status,409)

    def test_map_is_static_until_signed_live_telemetry_exists(self):
        connection=http.client.HTTPConnection('127.0.0.1',self.server.server_address[1],timeout=3.)
        connection.request('GET','/api/fpga-map',headers={'Host':self.host})
        response=connection.getresponse();view=json.loads(response.read());connection.close()
        self.assertEqual(response.status,200)
        self.assertEqual(len(view['points']),9)
        self.assertFalse(view['live']['fresh'])
        self.assertIsNone(view['live']['pose'])
        self.assertIsNone(view['live']['route'])

    def test_feedback_correlates_request_and_does_not_claim_immediate_completion(self):
        self.gimbal_ready();_,record=self.post(self.request(),'/api/gimbal-self-test')
        self.state.drain(lambda *args:None,lambda:100.)
        self.state.update('gimbal_self_test',dict(stamp=100.,last_request_id='unrelated',
            last_result='completed',last_request_stamp=100.,hardware_verified=True))
        self.assertEqual(self.state.commands['self-test1']['status'],'submitted')
        self.state.update('gimbal_self_test',dict(stamp=100.,request_id=record['command_id'],
            last_request_id=record['command_id'],last_result='accepted',last_request_stamp=100.,
            phase='running',hardware_verified=False))
        latest=self.state.commands['self-test1']
        self.assertEqual(latest['status'],'observed')
        self.assertFalse(latest['gimbal_self_test_status']['hardware_verified'])

    def test_rejected_second_start_does_not_reject_original_session(self):
        self.gimbal_ready()
        _,first=self.post(self.request(),'/api/gimbal-self-test')
        # Both requests can leave HTTP before the running-state feedback arrives.
        _,second=self.post(self.request(request_id='self-test2'),'/api/gimbal-self-test')
        self.state.drain(lambda *args:None,lambda:100.)
        self.state.update('gimbal_self_test',dict(stamp=100.,
            request_id=first['command_id'],last_request_id=second['command_id'],
            last_result='rejected',last_request_reason='self_test_busy',
            last_request_stamp=100.,phase='running',reason='awaiting_encoder',
            hardware_verified=False))
        active=self.state.commands['self-test1'];rejected=self.state.commands['self-test2']
        self.assertEqual(active['status'],'observed')
        self.assertEqual(active['gimbal_self_test_status']['phase'],'running')
        self.assertEqual(active['reason'],'awaiting_encoder')
        self.assertEqual(rejected['status'],'rejected')
        self.assertEqual(rejected['reason'],'self_test_busy')

    def test_late_session_timeout_uses_progress_stamp_not_old_receipt(self):
        self.gimbal_ready();_,record=self.post(self.request(),'/api/gimbal-self-test')
        self.state.drain(lambda *args:None,lambda:100.)
        self.state.update('gimbal_self_test',dict(stamp=100.,
            request_id=record['command_id'],last_request_id=record['command_id'],
            last_result='accepted',last_request_stamp=100.,phase='running',
            hardware_verified=False))
        self.state.wall=lambda:106.
        self.state.monotonic=lambda:106.
        self.state.update('gimbal_self_test',dict(stamp=106.,
            request_id=record['command_id'],last_request_id=record['command_id'],
            last_result='accepted',last_request_stamp=100.,phase='failed',
            reason='encoder_target_timeout',hardware_verified=False))
        final=self.state.commands['self-test1']
        self.assertEqual(final['status'],'observed')
        self.assertEqual(final['gimbal_self_test_status']['phase'],'failed')
        self.assertEqual(final['reason'],'encoder_target_timeout')

    def test_stop_receipt_is_correlated_separately_from_cancelled_run(self):
        self.gimbal_ready();_,run=self.post(self.request(),'/api/gimbal-self-test')
        self.state.drain(lambda *args:None,lambda:100.)
        self.state.update('gimbal_self_test',dict(stamp=100.,request_id=run['command_id'],
            phase='running',can_start=False,last_request_id=run['command_id'],
            last_result='accepted',last_request_stamp=100.))
        _,stop=self.post(dict(request_id='stop1',action='stop'),'/api/gimbal-self-test')
        self.state.drain(lambda *args:None,lambda:100.)
        self.state.update('gimbal_self_test',dict(stamp=100.,request_id=run['command_id'],
            phase='cancelled',reason='operator_stop',last_request_id=stop['command_id'],
            last_result='completed',last_request_stamp=100.,hardware_verified=False))
        self.assertEqual(self.state.commands['stop1']['status'],'observed')
        self.assertEqual(self.state.commands['self-test1']['gimbal_self_test_status']['phase'],'cancelled')
        self.assertFalse(self.state.commands['self-test1']['gimbal_self_test_status']['hardware_verified'])


if __name__=='__main__':unittest.main()
