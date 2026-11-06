import pathlib
import sys
import unittest
from copy import deepcopy

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.gimbal_self_test import GimbalSelfTest
from test_mixed_actuator_io import mixed_fixture
from test_zdt_fd8_control import config as fd8_config


class SelfTestTests(unittest.TestCase):
    def setUp(self):
        io, *_ = mixed_fixture()
        self.test = GimbalSelfTest(io.servo)
        self.context = dict(enabled=True, dry_run=False, estop=False, fault='', mission_fresh=True,
                            vehicle_stopped=True, idle_mode=True, pump_on=False, pump_armed=False,
                            active_owner=None)
        self.feedback = dict(feedback_fresh=True, error='', actual_angles_deg=[90., 90.],
                             motor_rpm=[0., 0.], settled_measured=False, target_deg=None)
        self.doc = dict(action='start', request_id='test-1', issued_at=100., ttl_s=2., axes='both',
                        amplitude_deg=1., speed_deg_s=6.)

    def start(self, **updates):
        self.test.request(dict(self.doc, **updates), wall_now=100., now=1.,
                          feedback=self.feedback, context=self.context)

    def test_requires_real_fresh_stationary_commissioned_owner(self):
        for field, value in [('dry_run', True), ('enabled', False), ('vehicle_stopped', False),
                             ('mission_fresh', False), ('active_owner', 'fire'), ('pump_on', True),
                             ('pump_armed', True), ('idle_mode', False)]:
            with self.subTest(field=field):
                ctx = dict(self.context, **{field:value})
                test = GimbalSelfTest(self.test.servo)
                with self.assertRaises(ValueError):
                    test.request(self.doc, wall_now=100., now=1., feedback=self.feedback, context=ctx)
                self.assertFalse(test.active)
        self.test.servo['reference_confirmed_this_start'] = False
        with self.assertRaises(ValueError):
            self.start()

    def test_all_legs_need_actual_angles_and_settled_feedback(self):
        self.start()
        self.assertEqual(self.test.tick(1.1, self.feedback, self.context), (91., 91.))
        for time, target in [(1.2, [91., 91.]), (1.5, [89., 89.]), (1.8, [90., 90.])]:
            # An ACK for the requested target without physical arrival is insufficient.
            f = dict(self.feedback, target_deg=target, settled_measured=True)
            if target != [90., 90.]:
                self.test.tick(time-.01, f, self.context)
                self.assertEqual(self.test.target, target)
            f['actual_angles_deg'] = target
            self.test.tick(time, f, self.context)
        status = self.test.status(f, self.context)
        self.assertEqual(status['phase'], 'completed')
        self.assertTrue(status['hardware_verified'])
        self.assertEqual(len(status['samples']), 3)
        self.assertEqual(status['last_result'], 'completed')

    def test_single_axis_keeps_other_at_measured_start(self):
        self.feedback['actual_angles_deg'] = [92., 87.]
        self.start(axes='tilt')
        self.assertEqual(self.test.points, [[92., 88.], [92., 86.], [92., 87.]])

    def test_start_rejects_unowned_motion_or_missing_speed(self):
        for speeds in (None, [1, 0], [0, -1]):
            self.setUp()
            self.feedback['motor_rpm'] = speeds
            with self.assertRaises(ValueError):
                self.start()

    def test_duplicate_active_request_preserves_owner_and_reports_rejection(self):
        self.start()
        with self.assertRaises(ValueError):
            self.start(request_id='second')
        self.assertEqual(self.test.request_id, 'test-1')
        self.assertEqual(self.test.last_request_id, 'second')
        self.assertEqual(self.test.last_result, 'rejected')
        self.assertTrue(self.test.active)

    def test_stale_replay_excess_motion_and_limit_requests_rejected(self):
        for changes in [dict(issued_at=97.), dict(amplitude_deg=2), dict(speed_deg_s=7),
                        dict(amplitude_deg=.1), dict(axes='unknown')]:
            self.setUp()
            with self.assertRaises(ValueError):
                self.start(**changes)
        self.setUp()
        self.feedback['actual_angles_deg'] = [149.9, 90.]
        with self.assertRaises(ValueError):
            self.start()

    def test_feedback_loss_timeout_and_stop_do_not_fake_completion(self):
        self.start()
        self.test.tick(1.2, dict(self.feedback, feedback_fresh=False), self.context)
        self.assertEqual(self.test.phase, 'failed')
        self.assertFalse(self.test.hardware_verified)
        self.setUp(); self.start()
        self.test.tick(6.1, self.feedback, self.context)
        self.assertEqual(self.test.reason, 'measured_arrival_timeout')
        self.setUp(); self.start()
        self.test.request(dict(action='stop', request_id='stop-1'), wall_now=101., now=2.,
                          feedback=self.feedback, context=self.context)
        self.assertEqual(self.test.request_id, 'test-1')
        self.assertEqual(self.test.last_request_id, 'stop-1')
        self.assertEqual(self.test.phase, 'cancelled')
        self.assertIsNone(self.test.tick(2.1, self.feedback, self.context))

    def test_fd8_quantized_feedback_advances_three_poses_without_exact_lattice_equality(self):
        self.test=GimbalSelfTest(fd8_config()['servo'])
        self.feedback['actual_angles_deg']=[0.,0.]
        self.start(axes='pan')
        for t,target,realized in [(1.2,[1.,0.],[.932959,0.]),
                                  (1.5,[-1.,0.],[-.867041,0.]),
                                  (1.8,[0.,0.],[.032959,0.])]:
            fb=dict(self.feedback, requested_target_deg=target,target_deg=realized,
                    settled_measured=True,actual_angles_deg=realized)
            self.test.tick(t,fb,self.context)
        self.assertEqual(self.test.phase,'completed')
        self.assertEqual(len(self.test.samples),3)

    def test_self_test_rejects_locked_tilt_before_claiming_owner(self):
        self.test=GimbalSelfTest(fd8_config()['servo'])
        self.test.servo['tilt']['motion_enabled']=False
        self.feedback['actual_angles_deg']=[0.,10.]
        for axis in ('tilt','both'):
            with self.assertRaisesRegex(ValueError,'direction_not_verified'):
                self.start(axes=axis,request_id=axis)
        self.assertFalse(self.test.active)


if __name__ == '__main__':
    unittest.main(verbosity=2)
