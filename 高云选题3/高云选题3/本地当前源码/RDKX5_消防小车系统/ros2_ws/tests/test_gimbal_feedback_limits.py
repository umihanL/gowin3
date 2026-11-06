"""Encoder jitter at travel endpoints does not expand commanded travel."""
import copy
import unittest

import test_pd42_actuator_io as pd_fixture
from test_zdt_fd8_control import fixture, config
from rescue_vision.pd42_actuator_io import validate as validate_pd42
from rescue_vision.gimbal_manual import GimbalManual
from rescue_vision.gimbal_self_test import GimbalSelfTest, measured_target_matches
from rescue_vision import zdt_fd8_control as fd8, zdt_vendor_bench as vendor


class FeedbackLimitTests(unittest.TestCase):
    def adapter(self, kind):
        io, pan, tilt, events = fixture() if kind == 'mixed' else pd_fixture.AdapterTests().fixture()
        io.servo['tilt'].update(min_angle_deg=0.,max_angle_deg=60.,reference_angle_deg=0.,
                                reference_counts=26408,motion_enabled=True)
        tilt.counts = tilt.target_counts = 26408
        return io, pan, tilt, events

    def test_pd42_and_mixed_keep_negative_and_upper_jitter_as_real_measurements(self):
        for kind in ('pd42','mixed'):
            for delta in (-19, round(60*51200/360)+19):
                io, _, tilt, _ = self.adapter(kind)
                tilt.counts = 26408+delta
                with self.subTest(kind=kind,delta=delta):
                    read = io._read_axis('tilt',io.servo['tilt'])
                    self.assertEqual(read['counts'],tilt.counts)
                    self.assertAlmostEqual(read['angle'],delta*360/51200)
                    self.assertTrue(read['angle'] < 0 or read['angle'] > 60)

    def test_pd42_and_mixed_point_two_one_beyond_bounds_faults_with_diagnostics(self):
        for kind in ('pd42','mixed'):
            for delta in (-30, round(60*51200/360)+30):
                io, _, tilt, _ = self.adapter(kind); tilt.counts = 26408+delta
                with self.subTest(kind=kind,delta=delta), self.assertRaises(RuntimeError) as caught:
                    io._read_axis('tilt',io.servo['tilt'])
                for text in ('actual_deg=',f'counts={tilt.counts}', 'limits_deg=[0.0,60.0]',
                             'feedback_limit_tolerance_deg=0.2'):
                    self.assertIn(text,str(caught.exception))

    def test_commanded_angles_remain_strict_before_any_motor_write(self):
        for kind in ('pd42','mixed'):
            io, _, tilt, events = self.adapter(kind)
            pan_target = io.servo['pan']['reference_angle_deg']
            for target in (-.01,60.01):
                with self.subTest(kind=kind,target=target), self.assertRaises(ValueError):
                    io.set_gimbal(pan_target,target,speed_deg_s=6,lease_s=.5)
            self.assertFalse(events)

    def test_feedback_allowance_defaults_and_validates_independently_of_target_tolerance(self):
        cfg = self.adapter('pd42')[0].config
        cfg['servo'].pop('feedback_limit_tolerance_deg')
        self.assertEqual(validate_pd42(cfg)['servo']['feedback_limit_tolerance_deg'],.2)
        for value in (-.01,.3001,float('inf'),float('nan'),True,'0.2'):
            changed=copy.deepcopy(cfg); changed['servo']['feedback_limit_tolerance_deg']=value
            with self.subTest(value=value), self.assertRaises(ValueError): validate_pd42(changed)
        for value in (0.,.1,.3):
            changed=copy.deepcopy(cfg); changed['servo']['feedback_limit_tolerance_deg']=value
            self.assertEqual(validate_pd42(changed)['servo']['feedback_limit_tolerance_deg'],value)

    def context(self):
        return dict(enabled=True,dry_run=False,mission_fresh=True,vehicle_stopped=True,idle_mode=True,
                    pump_on=False,pump_armed=False)

    def test_readiness_uses_same_default_allowance_without_clamping_or_promoting_settlement(self):
        servo=config()['servo']; servo.pop('feedback_limit_tolerance_deg',None)
        test=GimbalSelfTest(servo)
        for angle,expected in ((-.13,True),(-.21,False),(60.13,True),(60.21,False)):
            feedback=dict(actual_angles_deg=[0.,angle],motor_rpm=[0,0],feedback_fresh=True,error='')
            original=copy.deepcopy(feedback)
            with self.subTest(angle=angle):
                self.assertEqual(test.readiness(feedback,self.context())[0],expected)
                self.assertEqual(feedback,original)
                self.assertFalse(measured_target_matches(feedback,[0.,0.],.5))
        servo['feedback_limit_tolerance_deg']=.1
        self.assertFalse(test.readiness(dict(actual_angles_deg=[0.,-.13],motor_rpm=[0,0],
            feedback_fresh=True,error=''),self.context())[0])

    def test_manual_return_zero_and_omitted_tilt_target_work_from_measured_negative_jitter(self):
        for explicit in (False,True):
            servo=config()['servo']; manual=GimbalManual(servo)
            feedback=dict(actual_angles_deg=[0.,-.13],motor_rpm=[0,0],feedback_fresh=True,error='')
            document=dict(action='start',request_id='zero',issued_at=100.,ttl_s=2.,pan_deg=0.)
            if explicit: document['tilt_deg']=0.
            manual.request(document,wall_now=100.,now=10.,feedback=feedback,context=self.context())
            self.assertEqual(manual.target,[0.,0.])
            self.assertEqual(feedback['actual_angles_deg'][1],-.13)
            self.assertIsNotNone(manual.tick(10.1,feedback,self.context()))
            actual_target=manual.realizable_target
            feedback.update(requested_target_deg=[0.,0.],target_deg=actual_target,settled_measured=True)
            self.assertIsNone(manual.tick(10.2,feedback,self.context()))
            self.assertEqual(manual.phase,'completed')
        for target in (-.01,60.01):
            manual=GimbalManual(config()['servo']); document['tilt_deg']=target
            with self.assertRaisesRegex(ValueError,'outside_measured_limits'):
                manual.request(document,wall_now=100.,now=10.,feedback=feedback,context=self.context())

    def test_fd8_only_actual_feedback_has_allowance_targets_stay_strict(self):
        axis=config()['servo']['pan']; axis['reference_counts']=0
        for sign in (-1,1):
            target=sign*16384
            stepper=fd8.FD8Stepper(axis,dict(retained_target_counts=target))
            sample=dict(position_before_counts=target+sign*24,position_counts=target+sign*24,
                retained_target_counts=target,realtime_target_counts=target,
                position_error_deg=-sign*24*vendor.COUNT_DEG,speed_rpm=0,enabled=True,arrived=True)
            for i in range(3): ready=stepper.observe(sample,i*.04)
            self.assertTrue(ready)
            with self.assertRaisesRegex(RuntimeError,'mechanical limits'):
                stepper.plan_step(sign*91,.5)
            exceeded=dict(sample,position_counts=target+sign*39,position_before_counts=target+sign*39)
            with self.assertRaisesRegex(RuntimeError,'actual_deg=.*counts=.*limits_deg='):
                stepper.observe(exceeded,1.)
            changed=dict(sample,retained_target_counts=target+sign,realtime_target_counts=target+sign)
            with self.assertRaisesRegex(RuntimeError,'mechanical limits'):
                stepper.observe(changed,1.)


if __name__=='__main__':unittest.main(verbosity=2)
