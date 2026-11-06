from copy import deepcopy
import pathlib
import sys
import unittest

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.gimbal_manual import GimbalManual
from rescue_vision.gimbal_self_test import measured_target_matches
from test_zdt_fd8_control import config as fd8_config


class ManualTests(unittest.TestCase):
    def setUp(self):
        axis=dict(reference_counts=0, reference_angle_deg=0, min_angle_deg=-90,max_angle_deg=90)
        self.servo=dict(pan=deepcopy(axis),tilt=deepcopy(axis), reference_confirmed_this_start=True,
            calibration_confirmed=True,load_supported_verified=True,enable_on_command=True, position_tolerance_deg=.5)
        self.servo['tilt'].update(min_angle_deg=0,max_angle_deg=60,motion_enabled=False)
        self.manual=GimbalManual(self.servo)
        self.fb=dict(feedback_fresh=True,actual_angles_deg=[0.,0.],motor_rpm=[0,0],error='',settled_measured=False)
        self.ctx=dict(enabled=True,dry_run=False,mission_fresh=True,vehicle_stopped=True,idle_mode=True,pump_on=False,pump_armed=False)

    def start(self,pan=30,**kwargs):
        self.manual.request(dict(action='start',request_id='move',issued_at=100.,ttl_s=2.,pan_deg=pan,**kwargs),
            wall_now=100.,now=10.,feedback=self.fb,context=self.ctx)

    def test_pan_move_holds_unverified_tilt(self):
        self.start()
        self.assertEqual(self.manual.tick(10.1,self.fb,self.ctx),(30,0))
        self.assertEqual(self.manual.status(self.fb,self.ctx)['motion_axes'],['pan'])

    def test_total180_does_not_allow_single_side180(self):
        with self.assertRaisesRegex(ValueError,'outside_measured_limits'):self.start(180)
        self.assertFalse(self.manual.active)

    def test_unverified_tilt_cannot_move(self):
        with self.assertRaisesRegex(ValueError,'direction_not_verified'):self.start(10,tilt_deg=1)

    def test_ack_without_position_cannot_complete(self):
        self.start();self.fb['command_accepted']=True
        self.assertEqual(self.manual.tick(10.1,self.fb,self.ctx),(30,0))
        self.fb.update(actual_angles_deg=[29.8,0],settled_measured=True,target_deg=[30,0])
        self.assertIsNone(self.manual.tick(10.2,self.fb,self.ctx))
        self.assertEqual(self.manual.phase,'completed')

    def test_feedback_loss_cancels(self):
        self.start();self.fb['feedback_fresh']=False
        self.assertIsNone(self.manual.tick(10.1,self.fb,self.ctx))
        self.assertEqual(self.manual.phase,'failed')

    def test_stop_is_unconditional_and_never_resumes(self):
        self.start()
        self.manual.request({'action':'stop'},wall_now=100,now=10,feedback={},context={})
        self.assertIsNone(self.manual.tick(10.1,self.fb,self.ctx))
        self.assertEqual(self.manual.phase,'cancelled')

    def test_timeout_not_extended_by_tick(self):
        self.start()
        deadline=self.manual.deadline
        self.manual.tick(deadline,self.fb,self.ctx)
        self.assertEqual(self.manual.reason,'manual_gimbal_arrival_timeout')

    def test_moving_vehicle_or_pump_rejects(self):
        for name,value in [('vehicle_stopped',False),('pump_on',True)]:
            with self.subTest(name=name):
                self.setUp();self.ctx[name]=value
                with self.assertRaises(ValueError):self.start()

    def test_locked_tilt_defaults_to_reference_despite_encoder_jitter(self):
        self.fb['actual_angles_deg'][1]=.02
        self.start(10)
        self.assertEqual(self.manual.target,[10,0])
        self.setUp();self.fb['actual_angles_deg'][1]=.02
        with self.assertRaisesRegex(ValueError,'direction_not_verified'):
            self.start(10,tilt_deg=.02)

    def test_outside_endpoint_rejected_before_running_then_other_request_works(self):
        self.servo=fd8_config()['servo'];self.manual=GimbalManual(self.servo)
        with self.assertRaisesRegex(ValueError,'outside_measured_limits'):self.start(91)
        self.assertFalse(self.manual.active)
        self.manual.request(dict(action='start',request_id='valid-after-error',issued_at=100.,ttl_s=2.,pan_deg=1.),
            wall_now=100.,now=10.,feedback=self.fb,context=self.ctx)
        self.assertTrue(self.manual.active)
        self.assertEqual(self.manual.target,[1.,0.])
        self.assertAlmostEqual(self.manual.realizable_target[0],1.045459,places=5)

    def test_full_endpoint_is_now_reachable_at_single_pulse_resolution(self):
        self.servo=fd8_config()['servo'];self.manual=GimbalManual(self.servo)
        self.start(90)
        self.assertTrue(self.manual.active)
        self.assertEqual(self.manual.target,[90.,0.])
        self.assertAlmostEqual(self.manual.realizable_target[0],89.920458984375)

    def test_completion_binds_original_goal_and_checks_actual_and_realizable_error(self):
        self.start(1)
        self.fb.update(settled_measured=True,target_deg=[.933,0],requested_target_deg=[1,0],actual_angles_deg=[.92,0])
        self.assertTrue(measured_target_matches(self.fb,[1,0],.5))
        for field,value in [('requested_target_deg',[.9,0]),('target_deg',[2,0]),
                            ('actual_angles_deg',[1.51,0]),('feedback_fresh',False),('error','fault')]:
            other=dict(self.fb,**{field:value})
            self.assertFalse(measured_target_matches(other,[1,0],.5),field)
        self.assertIsNone(self.manual.tick(10.1,self.fb,self.ctx))
        self.assertEqual(self.manual.phase,'completed')

    def test_status_distinguishes_mechanical_and_dynamic_reachable_ranges(self):
        self.servo=fd8_config()['servo'];self.manual=GimbalManual(self.servo)
        status=self.manual.status(self.fb,self.ctx)
        self.assertEqual(status['axis_limits_deg'],[[-90,90],[0,60]])
        self.assertEqual(status['axis_step_deg'],[.1125,None])
        self.assertAlmostEqual(status['axis_reachable_limits_deg'][0][0],-89.967041015625)
        self.assertAlmostEqual(status['axis_reachable_limits_deg'][0][1],89.920458984375)
        self.assertEqual(status['axis_reachable_limits_deg'][1],[0,60])
        self.servo['pan']['direction']=-1
        reversed_status=self.manual.status(self.fb,self.ctx)
        self.assertAlmostEqual(reversed_status['axis_reachable_limits_deg'][0][0],-89.920458984375)
        self.assertIsNone(self.manual.target)

    def test_normal_protocol_does_not_invent_discrete_step_size(self):
        status=self.manual.status(self.fb,self.ctx)
        self.assertEqual(status['axis_reachable_limits_deg'],status['axis_limits_deg'])
        self.assertEqual(status['axis_step_deg'],[None,None])


if __name__=='__main__':unittest.main()
