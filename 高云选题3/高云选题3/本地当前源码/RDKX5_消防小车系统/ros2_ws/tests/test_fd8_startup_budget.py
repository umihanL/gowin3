"""Current-boot startup snapshots and finite feedback-time budgets."""
import copy
import math
import unittest

from test_zdt_fd8_control import config
from rescue_vision import zdt_fd8_control as fd8, zdt_vendor_bench as vendor
from rescue_vision.mixed_actuator_io import validate
from rescue_vision.gimbal_manual import GimbalManual
from rescue_vision.gimbal_self_test import GimbalSelfTest


class StartupSnapshotTests(unittest.TestCase):
    def snapshot(self):
        cfg = config()
        cfg['servo']['reference_id'] = 'original-zero'
        cfg['commissioning'] = {'boot_id':'original-boot'}
        axis = cfg['servo']['pan']
        axis['reference_counts'] = -7
        axis['zdt_fd8_snapshot'].update(reference_position_counts=-7,
            startup_position_counts=190, retained_target_counts=180, realtime_target_counts=180,
            startup_reference_id='original-zero', startup_boot_id='original-boot')
        return cfg

    def test_nonzero_start_preserves_original_mechanical_zero_and_expected_target(self):
        cfg = self.snapshot(); before = copy.deepcopy(cfg)
        checked = validate(cfg)
        axis = checked['servo']['pan']; snap = axis['zdt_fd8_snapshot']
        self.assertEqual(axis['reference_counts'], -7)
        self.assertEqual(checked['servo']['reference_id'], 'original-zero')
        self.assertEqual(fd8.angle_for_counts(-7, axis), 0)
        self.assertAlmostEqual(fd8.angle_for_counts(190, axis), 197*vendor.COUNT_DEG)
        self.assertEqual(fd8.FD8Stepper(axis, snap).expected, 180)
        self.assertEqual(cfg, before)

    def test_all_startup_fields_required_and_context_cannot_be_unknown(self):
        for field in ('startup_position_counts','startup_reference_id','startup_boot_id'):
            cfg = self.snapshot(); del cfg['servo']['pan']['zdt_fd8_snapshot'][field]
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'together'):
                validate(cfg)
        for field in ('reference_id','boot_id'):
            cfg = self.snapshot()
            if field == 'reference_id': del cfg['servo'][field]
            else: del cfg['commissioning'][field]
            with self.subTest(context=field), self.assertRaisesRegex(ValueError, 'mismatch/unverified'):
                validate(cfg)

    def test_wrong_reference_or_boot_and_redefined_zero_are_rejected(self):
        for field, value in (('startup_reference_id','other-zero'),('startup_boot_id','other-boot'),
                             ('reference_position_counts',190)):
            cfg = self.snapshot(); cfg['servo']['pan']['zdt_fd8_snapshot'][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError): validate(cfg)

    def test_startup_range_type_target_agreement_and_point_fifteen_boundary(self):
        for changes in ({'startup_position_counts':208}, # 28 counts > .15 degree
                        {'realtime_target_counts':182},
                        {'startup_position_counts':False},
                        {'startup_position_counts':2**31},
                        {'startup_position_counts':20000,'retained_target_counts':20000,'realtime_target_counts':20000}):
            cfg = self.snapshot(); cfg['servo']['pan']['zdt_fd8_snapshot'].update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError): validate(cfg)
        cfg = self.snapshot(); cfg['servo']['pan']['zdt_fd8_snapshot']['startup_position_counts'] = 207
        validate(cfg)  # 27 counts is inside .15 degree.

    def test_legacy_snapshot_still_requires_actual_reference_near_retained(self):
        cfg = config(); validate(cfg)
        cfg['servo']['pan']['zdt_fd8_snapshot'].update(retained_target_counts=180, realtime_target_counts=180)
        with self.assertRaisesRegex(ValueError, 'unaligned retained target'): validate(cfg)


class FeedbackBudgetTests(unittest.TestCase):
    def test_three_stable_samples_can_finish_after_two_seconds_without_a_dwell(self):
        cfg = config(); axis = cfg['servo']['pan']; snap = axis['zdt_fd8_snapshot']
        stepper = fd8.FD8Stepper(axis, snap)
        expected = snap['retained_target_counts']+vendor.STEP_COUNTS
        stepper.mark_written(1, expected, 100.)
        self.assertAlmostEqual(stepper.pending['deadline'], 104.3)
        counts = round(expected)
        sample = dict(position_before_counts=counts+9, position_counts=counts+9,
            retained_target_counts=counts, realtime_target_counts=counts,
            position_error_deg=-9*vendor.COUNT_DEG, speed_rpm=0, enabled=True, arrived=True)
        for now in (101.9, 102.5):
            self.assertFalse(stepper.observe(sample, now))
            self.assertIsNone(stepper.plan_step(3.6, .5))
            self.assertEqual(stepper.steps, 1)
        self.assertTrue(stepper.observe(sample, 103.1))
        self.assertIsNotNone(stepper.plan_step(3.6, .5))
        self.assertEqual(stepper.steps, 1)  # Planning alone cannot resend/write.

    def test_distance_aware_arrival_deadline_remains_a_hard_timeout(self):
        cfg = config(); axis = cfg['servo']['pan']; snap = axis['zdt_fd8_snapshot']
        stepper = fd8.FD8Stepper(axis, snap); expected = snap['retained_target_counts']+vendor.STEP_COUNTS
        stepper.mark_written(1, expected, 100.)
        count = round(expected)
        sample = dict(position_before_counts=count, position_counts=count,
            retained_target_counts=count, realtime_target_counts=count,
            position_error_deg=0, speed_rpm=0, enabled=True, arrived=True)
        stepper.observe(sample, 103.8); stepper.observe(sample, 103.9)
        with self.assertRaisesRegex(RuntimeError,'arrival timeout'): stepper.observe(sample, 104.31)
        self.assertEqual(stepper.steps, 1)

    def context(self):
        return dict(enabled=True,dry_run=False,mission_fresh=True,vehicle_stopped=True,idle_mode=True,
                    pump_on=False,pump_armed=False)

    def test_manual_whole_move_budget_covers_single_packet_travel_but_finishes_on_arrival(self):
        servo = config()['servo']; manual = GimbalManual(servo)
        feedback = dict(feedback_fresh=True, actual_angles_deg=[-89.,0.],motor_rpm=[0,0],error='')
        manual.request(dict(action='start',request_id='whole-span',issued_at=100,ttl_s=2.,pan_deg=89.,tilt_deg=0.),
                       wall_now=100,now=10,feedback=feedback,context=self.context())
        original_deadline = manual.deadline
        self.assertGreaterEqual(original_deadline-10, fd8.motion_timeout_s(178))
        self.assertLessEqual(original_deadline-10, 180)
        self.assertIsNotNone(manual.tick(120,feedback,self.context()))
        self.assertEqual(manual.deadline, original_deadline)
        realized = manual.realizable_target
        feedback.update(actual_angles_deg=realized,settled_measured=True,target_deg=realized,
                        requested_target_deg=[89.,0.])
        self.assertIsNone(manual.tick(120.1,feedback,self.context()))
        self.assertEqual(manual.phase,'completed')

    def test_self_test_reverse_leg_budget_covers_full_target_travel(self):
        servo = config()['servo']; test = GimbalSelfTest(servo)
        feedback = dict(feedback_fresh=True,actual_angles_deg=[0.,0.],motor_rpm=[0,0],error='')
        test.request(dict(action='start',request_id='test-budget',issued_at=100,ttl_s=2.,axes='pan',
                          amplitude_deg=1.,speed_deg_s=6.),wall_now=100,now=1,
                     feedback=feedback,context=self.context())
        self.assertGreaterEqual(test.leg_timeouts[1], 2*4)
        self.assertEqual(test.total_timeout, sum(test.leg_timeouts))
        first_deadline = test.step_started+test.leg_timeouts[0]
        self.assertIsNotNone(test.tick(6.1, feedback, self.context()))
        self.assertIsNone(test.tick(first_deadline+.01,feedback,self.context()))
        self.assertEqual(test.reason,'measured_arrival_timeout')


if __name__ == '__main__': unittest.main(verbosity=2)
