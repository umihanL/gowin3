"""Production angle-control HTTP ingress; no real actuators."""
import unittest
import test_numbered_interaction as fixture


class ManualGimbalApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.NumberedHttpTests.setUpClass();cls.module=fixture.NumberedHttpTests.module
    setUp=fixture.NumberedHttpTests.setUp
    tearDown=fixture.NumberedHttpTests.tearDown
    post=fixture.NumberedHttpTests.post

    def ready(self):
        self.state.ros['gimbal_manual_ready']=True
        self.state.update('gimbal_manual',dict(stamp=100.,can_start=True,phase='idle',motion_axes=['pan']))
        self.state.update('actuators',dict(stamp=100.,dry_run=False,status_fresh=True,feedback_fresh=True,
            axis_limits_deg=[[-90.,90.],[-90.,90.]],actual_angles_deg=[0.,12.]))

    def request(self,**changes):
        value=dict(request_id='manual1',action='start',pan_deg=45.)
        value.update(changes);return value

    def reference_ready(self):
        self.state.ros['gimbal_manual_ready']=True
        self.state.update('gimbal_manual',dict(stamp=100.,can_start=False,phase='idle',
            ready_reason='reference_required',reference_capture_ready=True))
        self.state.update('actuators',dict(stamp=100.,dry_run=False,enabled=True,
            status_fresh=False,feedback_fresh=False,raw_feedback_fresh=True,
            reference_confirmed_this_start=False,actual_angles_deg=None,
            motor_counts=[-7,27200],motor_rpm=[0,0],error='',estop=False))

    def reference_request(self,**changes):
        value=dict(request_id='reference1',action='confirm_reference',operator_zero_confirmed=True,
                   supported_load=True,stationary_confirmed=True)
        value.update(changes);return value

    def test_start_publishes_exact_finite_goal_to_separate_owner(self):
        self.ready();status,result=self.post(self.request(),'/api/gimbal-manual')
        self.assertEqual(status,202,result)
        sent=[];self.state.drain(lambda kind,value:sent.append((kind,value)),lambda:100.)
        self.assertEqual(sent, [('gimbal_manual',dict(action='start',pan_deg=45.,request_id=result['command_id'],issued_at=100.,ttl_s=3.))])
        self.assertEqual(self.state.commands['manual1']['status'],'submitted')
        self.assertNotIn('completed',self.state.commands['manual1'])

    def test_unverified_tilt_dynamic_limits_and_injected_fields_are_rejected(self):
        self.ready()
        for changes,expected in [({'tilt_deg':20},409),({'pan_deg':91},409),({'pan_deg':True},400),
                ({'pan_deg':float('nan')},400),({'issued_at':100.},400),({'source':'web'},400),({'speed_deg_s':30},400)]:
            status,_=self.post(self.request(**changes),'/api/gimbal-manual');self.assertEqual(status,expected,changes)
        self.assertFalse(self.state.pending)

    def test_queue_rechecks_changed_limits_and_freshness_before_movement(self):
        self.ready();self.post(self.request(),'/api/gimbal-manual')
        self.state.update('actuators',dict(stamp=100.,dry_run=False,status_fresh=True,feedback_fresh=True,axis_limits_deg=[[-10.,10.],[-90.,90.]]))
        self.state.drain(lambda *args:self.fail('changed limit bypass'),lambda:100.)
        self.assertEqual(self.state.commands['manual1']['reason'],'gimbal_manual_pan_outside_limits')
        self.state.update('gimbal_manual',dict(stamp=95.,can_start=True,motion_axes=['pan']))
        self.assertEqual(self.post(self.request(request_id='manual2',pan_deg=1.),'/api/gimbal-manual')[0],409)

    def test_stop_has_priority_without_feedback_and_does_not_cancel_motor_bench(self):
        self.ready();self.post(self.request(),'/api/gimbal-manual')
        self.state.ros['motor_test_ready']=True
        self.state.enqueue(request_id='bench-stop',motor_test='stop',motor_id='motor1')
        self.state.latest.pop('actuators');self.state.latest.pop('gimbal_manual')
        status,result=self.post(dict(request_id='manual-stop',action='stop'),'/api/gimbal-manual')
        self.assertEqual(status,202,result)
        sent=[];self.state.drain(lambda kind,value:sent.append((kind,value)),lambda:100.)
        self.assertEqual(sent[0][0],'gimbal_manual');self.assertEqual(sent[0][1]['action'],'stop')
        self.assertEqual(sent[1][0],'motor_test')
        self.assertEqual(self.state.commands['manual1']['reason'],'superseded_by_gimbal_manual_stop')

    def test_actual_request_id_receipt_only_and_repeated_start_does_not_duplicate(self):
        self.ready();_,record=self.post(self.request(),'/api/gimbal-manual')
        self.post(self.request(),'/api/gimbal-manual')
        self.assertEqual(len(self.state.pending),1)
        self.assertEqual(self.post(self.request(pan_deg=20),'/api/gimbal-manual')[0],409)
        self.state.drain(lambda *_:None,lambda:100.)
        self.state.update('gimbal_manual',dict(stamp=100.,last_request_id='other',last_result='completed',phase='completed'))
        self.assertEqual(self.state.commands['manual1']['status'],'submitted')
        self.state.update('gimbal_manual',dict(stamp=100.,last_request_id=record['command_id'],last_result='completed',phase='completed',reason='measured_arrival'))
        self.assertEqual(self.state.commands['manual1']['gimbal_manual_status']['phase'],'completed')

    def test_reference_confirmation_does_not_require_angles_and_publishes_exact_once(self):
        self.reference_ready()
        status,result=self.post(self.reference_request(),'/api/gimbal-manual')
        self.assertEqual(status,202,result)
        again,duplicate=self.post(self.reference_request(),'/api/gimbal-manual')
        self.assertEqual(again,202);self.assertEqual(duplicate['command_id'],result['command_id'])
        sent=[];self.state.drain(lambda kind,value:sent.append((kind,value)),lambda:100.)
        self.assertEqual(sent,[('gimbal_manual',dict(action='confirm_reference',operator_zero_confirmed=True,
            supported_load=True,stationary_confirmed=True,request_id=result['command_id'],issued_at=100.,ttl_s=3.))])
        self.assertEqual(self.state.commands['reference1']['status'],'submitted')
        self.assertEqual(self.post(self.request(),'/api/gimbal-manual')[0],409)

    def test_reference_requires_explicit_confirmations_and_rejects_motion_or_reset_fields(self):
        self.reference_ready()
        for changes in ({'operator_zero_confirmed':False},{'supported_load':1},{'stationary_confirmed':'true'},
                        {'pan_deg':0},{'tilt_deg':0},{'reset_encoder':True},{'source':'web'}):
            code,_=self.post(self.reference_request(**changes),'/api/gimbal-manual')
            self.assertEqual(code,400,changes)
        missing=self.reference_request();missing.pop('stationary_confirmed')
        self.assertEqual(self.post(missing,'/api/gimbal-manual')[0],400)
        self.assertFalse(self.state.pending)

    def test_reference_rechecks_live_raw_feedback_stationary_and_owner_readiness(self):
        self.reference_ready()
        for change in ({'raw_feedback_fresh':False},{'dry_run':True},{'estop':True},{'error':'uart fault'},
                       {'motor_counts':[True,2]},{'motor_rpm':[0,1]},{'motor_rpm':[None,0]}):
            self.reference_ready();value=self.state.envelope('actuators')['data'];value.update(change)
            self.state.update('actuators',value)
            self.assertEqual(self.post(self.reference_request(),'/api/gimbal-manual')[0],409,change)
        self.reference_ready();self.state.ros['gimbal_manual_ready']=False
        self.assertEqual(self.post(self.reference_request(),'/api/gimbal-manual')[0],409)
        self.reference_ready();self.assertEqual(self.post(self.reference_request(),'/api/gimbal-manual')[0],202)
        self.state.update('gimbal_manual',dict(stamp=100.,reference_capture_ready=False,reference_capture_reason='gimbal_busy'))
        self.state.drain(lambda *args:self.fail('must recheck before dispatch'),lambda:100.)
        self.assertEqual(self.state.commands['reference1']['reason'],'gimbal_busy')

    def test_reference_can_be_superseded_by_stop_and_stale_state_never_submits(self):
        self.reference_ready();self.post(self.reference_request(),'/api/gimbal-manual')
        self.assertEqual(self.post(dict(request_id='stop1',action='stop'),'/api/gimbal-manual')[0],202)
        sent=[];self.state.drain(lambda kind,value:sent.append((kind,value)),lambda:100.)
        self.assertEqual(len(sent),1);self.assertEqual(sent[0][1]['action'],'stop')
        self.assertEqual(self.state.commands['reference1']['reason'],'superseded_by_gimbal_manual_stop')
        self.state.update('gimbal_manual',dict(stamp=95.,reference_capture_ready=True))
        self.assertEqual(self.post(self.reference_request(request_id='reference2'),'/api/gimbal-manual')[0],409)

    def test_recorded_reference_can_be_explicitly_replaced_repeatedly(self):
        self.reference_ready()
        value=self.state.envelope('actuators')['data']
        value.update(reference_confirmed_this_start=True,reference_capture_supported=True)
        self.state.update('actuators',value)
        for request_id in ('repeat1','repeat2'):
            self.assertEqual(self.post(self.reference_request(request_id=request_id),'/api/gimbal-manual')[0],202)
            sent=[];self.state.drain(lambda kind,value:sent.append((kind,value)),lambda:100.)
            self.assertEqual(len(sent),1)
            self.assertEqual(sent[0][1]['action'],'confirm_reference')

    def test_reference_recovery_requires_matching_explicit_backend_permission(self):
        fault='tilt measured position outside calibrated limits/reference: actual_deg=-8.8'
        self.reference_ready();value=self.state.envelope('actuators')['data']
        value.update(error=fault,reference_capture_supported=True,reference_recovery_permitted=True,
                     reference_recovery_error=fault,reference_confirmed_this_start=True)
        self.state.update('actuators',value)
        self.assertEqual(self.post(self.reference_request(),'/api/gimbal-manual')[0],202)
        value['error']='UART fault';self.state.update('actuators',value)
        self.state.drain(lambda *args:self.fail('changed fault bypass'),lambda:100.)
        self.assertEqual(self.state.commands['reference1']['reason'],'actuator_fault_or_estop')
        for change in ({'reference_capture_supported':False},{'reference_recovery_permitted':False},
                       {'reference_recovery_error':'old fault'},{'estop':True}):
            check=dict(value,error=fault);check.update(change);self.state.update('actuators',check)
            self.assertEqual(self.post(self.reference_request(request_id='blocked'),'/api/gimbal-manual')[0],409)

    def test_explicit_recovery_reconnects_without_requiring_dead_encoder_feedback(self):
        self.ready()
        self.state.update('actuators',dict(stamp=100.,serial_connected=False,
            feedback_fresh=False,error='ZDT encoder left the commanded angle interval'))
        self.state.update('gimbal_manual',dict(stamp=100.,can_start=False,phase='failed'))
        self.assertEqual(self.post(self.request(),'/api/gimbal-manual')[0],409)
        request=dict(request_id='recover1',action='recover')
        code,record=self.post(request,'/api/gimbal-manual');self.assertEqual(code,202,record)
        self.assertEqual(self.post(request,'/api/gimbal-manual')[0],202)
        self.state.latest.pop('actuators');self.state.latest.pop('gimbal_manual')
        sent=[];self.state.drain(lambda kind,value:sent.append((kind,value)),lambda:100.)
        self.assertEqual(sent,[('gimbal_manual',dict(action='recover',request_id=record['command_id'],issued_at=100.,ttl_s=3.))])
        self.assertEqual(self.state.commands['recover1']['status'],'submitted')
        self.assertNotIn('completed',self.state.commands['recover1'])

    def test_recovery_cancels_unsent_goal_and_stop_can_cancel_recovery(self):
        self.ready();self.post(self.request(),'/api/gimbal-manual')
        self.assertEqual(self.post(dict(request_id='recover1',action='recover'),'/api/gimbal-manual')[0],202)
        self.assertEqual(self.state.commands['manual1']['reason'],'superseded_by_gimbal_recovery')
        self.assertEqual(self.post(dict(request_id='stop1',action='stop'),'/api/gimbal-manual')[0],202)
        sent=[];self.state.drain(lambda kind,value:sent.append((kind,value)),lambda:100.)
        self.assertEqual(len(sent),1);self.assertEqual(sent[0][1]['action'],'stop')
        self.assertEqual(self.state.commands['recover1']['reason'],'superseded_by_gimbal_manual_stop')

    def test_recovery_cannot_inject_angles_or_reset_encoder_and_rechecks_subscriber(self):
        self.ready()
        for extra in ({'pan_deg':0},{'tilt_deg':0},{'reset_encoder':True},{'operator_zero_confirmed':True}):
            code,_=self.post(dict(request_id='recover1',action='recover',**extra),'/api/gimbal-manual')
            self.assertEqual(code,400,extra)
        self.assertEqual(self.post(dict(request_id='recover2',action='recover'),'/api/gimbal-manual')[0],202)
        self.state.ros['gimbal_manual_ready']=False
        self.state.drain(lambda *args:self.fail('no subscriber'),lambda:100.)
        self.assertEqual(self.state.commands['recover2']['reason'],'gimbal_manual_subscriber_unavailable')

if __name__=='__main__':unittest.main()
