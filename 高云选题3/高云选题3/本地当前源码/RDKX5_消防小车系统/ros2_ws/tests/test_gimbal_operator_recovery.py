"""Operator recovery never repeats interrupted movement or rewrites zero."""
from copy import deepcopy
import json
import threading
import types
import unittest
from unittest import mock

import test_gimbal_live_node as node_tests
import test_live_mixed_actuator_io as live_tests
from test_live_mixed_actuator_io import fixture, live
import test_zdt_fd8_control as fd8_tests
from test_zdt_fd8_control import fd8


class RecoveryNodeTests(unittest.TestCase):
    setUpClass = node_tests.LiveNodeTests.__dict__['setUpClass']
    setUp = node_tests.LiveNodeTests.setUp
    cleanup_worker = node_tests.LiveNodeTests.cleanup_worker
    request = node_tests.LiveNodeTests.request
    tick = node_tests.LiveNodeTests.tick

    def ready_recovery(self):
        self.node.fault = 'ZDT encoder left the commanded angle interval'
        self.io.restart_readback = mock.Mock(return_value=dict(reconnected=True, motion_commands_sent=False))
        # The production FPGA transport is separately tested. Give this node
        # test fresh OFF evidence without touching any UART or real pump.
        self.node.fpga_pump = None
        self.node.pump_on = False

    def finish_recovery(self):
        self.assertTrue(self.node.recovery_job['done'].wait(1.))
        self.tick()

    def test_recovery_cancels_previous_target_and_keeps_saved_zero(self):
        self.ready_recovery()
        self.node.requested = (20, 30)
        self.node.command_owner = 'manual'
        self.node.command_deadline = 20.
        original = deepcopy(self.io.config)
        self.request(action='recover')
        self.assertIsNone(self.node.requested)
        self.assertIsNone(self.node.command_owner)
        self.assertFalse(self.node.guard.armed)
        self.io.readback.update(error='', feedback_fresh=True, reference_confirmed_this_start=True,
                               actual_angles_deg=[4, 12], motor_rpm=[0, 0])
        self.finish_recovery()
        self.assertEqual(self.node.recovery_phase, 'completed')
        self.assertEqual(self.node.fault, '')
        self.assertEqual(self.io.config, original)
        self.assertNotIn('move', self.io.commands)
        self.assertEqual(self.io.calls, 0)
        self.persist.assert_not_called()
        self.assertEqual(self.node.manual.last_request_id, 'reference-1')

    def test_no_serial_feedback_needed_to_request_recovery_but_completion_needs_it(self):
        self.ready_recovery()
        self.io.readback.update(error='', feedback_fresh=False, raw_feedback_fresh=False,
                               serial_connected=False, motor_rpm=None)
        self.request(action='recover')
        self.finish_recovery()
        self.assertIsNotNone(self.node.recovery_job)
        self.assertTrue(self.node.fault)
        self.assertEqual(self.node.manual_output[-1]['recovery_phase'], 'checking')
        self.assertFalse(self.node.manual_output[-1]['can_start'])
        self.assertNotIn('move', self.io.commands)

    def test_current_driver_error_and_pump_on_do_not_clear_previous_interruption(self):
        self.ready_recovery()
        self.io.readback.update(error='FD8 driver fault/limit/reset state active', feedback_fresh=True,
                               reference_confirmed_this_start=True)
        self.request(action='recover')
        self.finish_recovery()
        self.assertTrue(self.node.fault)
        self.assertIsNotNone(self.node.recovery_job)
        self.io.readback['error'] = ''
        self.node.pump_on = True
        self.tick()
        self.assertEqual(self.node.recovery_reason, 'gimbal_recovery_waiting_pump_off')
        self.assertTrue(self.node.fault)

    def test_expired_and_duplicate_recovery_requests_never_replay(self):
        self.ready_recovery()
        self.request(action='recover', issued_at=2.)
        self.io.restart_readback.assert_not_called()
        self.request(action='recover')
        self.assertTrue(self.node.recovery_job['done'].wait(1.))
        self.request(action='recover')
        self.io.restart_readback.assert_called_once()

    def test_failed_connection_can_be_retried_without_restarting_ros(self):
        self.ready_recovery()
        self.io.restart_readback.side_effect = OSError('serial unplugged')
        self.request(action='recover')
        self.finish_recovery()
        self.assertIsNone(self.node.recovery_job)
        self.assertEqual(self.node.recovery_phase, 'waiting')
        self.assertTrue(self.node.recovery_readiness(10.)[0])
        self.io.restart_readback.side_effect = None
        self.io.readback.update(error='', feedback_fresh=True, reference_confirmed_this_start=True)
        self.request(action='recover', request_id='retry-2')
        self.finish_recovery()
        self.assertEqual(self.node.recovery_phase, 'completed')

    def test_estop_and_live_bench_reject_recovery(self):
        self.ready_recovery()
        self.node.estop = True
        self.request(action='recover')
        self.io.restart_readback.assert_not_called()
        self.node.estop = False
        self.node.motor_bench = {'motors': {'2': {'continuous_running': True}}}
        self.request(action='recover', request_id='bench')
        self.io.restart_readback.assert_not_called()

    def test_operator_stop_during_recovery_never_clears_interruption_or_restarts_goal(self):
        self.ready_recovery()
        self.request(action='recover')
        self.assertTrue(self.node.recovery_job['done'].wait(1.))
        self.request(action='stop', request_id='stop-recovery')
        self.io.readback.update(error='', feedback_fresh=True, reference_confirmed_this_start=True)
        self.tick()
        self.assertIsNone(self.node.recovery_job)
        self.assertTrue(self.node.fault)
        self.assertEqual(self.node.recovery_phase, 'waiting')
        self.assertNotIn('move', self.io.commands)

    def test_reset_estop_does_not_clear_interruption_and_allows_separate_recovery(self):
        self.ready_recovery()
        original = self.node.fault
        self.node.estop = True
        self.node.mission['operator_hold'] = True
        with mock.patch.object(self.mod.time, 'monotonic', return_value=10.):
            self.node.request(types.SimpleNamespace(data=json.dumps({'action': 'reset_estop'})))
        self.assertFalse(self.node.estop)
        self.assertEqual(self.node.fault, original)
        self.assertTrue(self.node.recovery_readiness(10.)[0])

    def test_timeout_does_not_release_still_running_owner_for_a_second_reconnect(self):
        self.ready_recovery()
        finished = threading.Event()
        self.io.restart_readback.side_effect = lambda: finished.wait(1.)
        self.request(action='recover')
        job = self.node.recovery_job
        try:
            self.node.tick_recovery(30.)
            self.assertIs(self.node.recovery_job, job)
            self.assertFalse(self.node.recovery_readiness(30.)[0])
        finally:
            finished.set()
            self.assertTrue(job['done'].wait(1.))
        self.node.tick_recovery(30.)
        self.assertIsNone(self.node.recovery_job)
        self.assertTrue(self.node.fault)


class BackendRecoveryTests(unittest.TestCase):
    capture = live_tests.LiveReferenceTests.capture
    restore_fixture = live_tests.LiveReferenceTests.restore_fixture
    attempt_restore = live_tests.LiveReferenceTests.attempt_restore

    def test_reconnect_releases_old_worker_and_restores_nonzero_saved_origin(self):
        io, pan, tilt, events = self.restore_fixture()
        original = deepcopy(io.servo['absolute_reference'])
        self.attempt_restore(io)
        io.finish_reference_restore(original['reference_id'], True)
        old = io._delegate
        io.restart_readback()
        self.assertTrue(old.closed)
        self.assertIsNone(io._delegate)
        self.assertFalse(io._restore_attempted)
        io._ports = {'/dev/ttyS1': tilt, '/dev/ttyS3': pan}
        io._owner = mock.Mock()
        for _ in range(3):
            io._sample()
        import time
        now = time.monotonic()
        for index, row in enumerate(io._rows):
            row['started'] = now-.1+index*.04
        state = io.feedback()
        self.assertTrue(state['diagnostic_feedback_fresh'])
        self.assertAlmostEqual(state['diagnostic_angles_deg'][0], 30., places=2)
        self.assertFalse(state['feedback_fresh'])
        self.attempt_restore(io)
        restored = io.take_restored_reference()
        self.assertEqual(restored['reference_id'], original['reference_id'])
        self.assertEqual(restored['config']['servo']['absolute_reference'], original)
        self.assertTrue(all(cmd is None or int(cmd)<0x80 for _, cmd, _ in events))

    def test_busy_old_worker_retains_ownership_and_does_not_start_second_owner(self):
        io, _, _, _ = fixture()
        io._thread = mock.Mock()
        io._thread.is_alive.return_value = True
        owner = io._owner
        with self.assertRaisesRegex(RuntimeError, 'ownership retained'):
            io.restart_readback()
        owner.close.assert_not_called()

    def test_real_alarm_keeps_reads_available_without_restoring_motion(self):
        io, pan, tilt, _ = self.restore_fixture()
        for row in io._rows:
            row['pan']['state']['raw'] = 4
        self.attempt_restore(io)
        self.assertIsNone(io._delegate)
        self.assertTrue(io.feedback()['raw_feedback_fresh'])
        self.assertIn('fault_limit', io.feedback()['absolute_reference_restore_reason'])


class SettlingEnvelopeTests(unittest.TestCase):
    stepper = fd8_tests.CompleteTargetTrajectoryTests.stepper
    sample = fd8_tests.CompleteTargetTrajectoryTests.sample

    def test_actual_reported_small_destination_overshoot_waits_for_real_arrival(self):
        stepper = self.stepper(0)
        stepper.mark_written(*stepper.plan_step(-6, .5), 100.)
        target = stepper.pending['expected']
        sample = self.sample(stepper, target*fd8.COUNT_DEG-.31)
        sample.update(position_before_counts=round(target-.08/fd8.COUNT_DEG),
                      position_error_deg=.1483, speed_rpm=-1, arrived=True)
        self.assertFalse(stepper.observe(sample, 100.5))
        self.assertIsNotNone(stepper.pending)
        self.assertEqual(stepper.steps, 1)
        exact = self.sample(stepper, target*fd8.COUNT_DEG)
        exact.update(speed_rpm=0, arrived=True)
        for t in (100.6, 100.7, 100.8):
            complete = stepper.observe(exact, t)
        self.assertTrue(complete)

    def test_large_overshoot_and_true_mechanical_limit_remain_rejected(self):
        stepper = self.stepper(0)
        stepper.mark_written(*stepper.plan_step(-6, .5), 100.)
        with self.assertRaisesRegex(RuntimeError, 'commanded angle interval'):
            stepper.observe(self.sample(stepper, -6.7), 100.5)
        stepper = self.stepper(0)
        stepper.mark_written(*stepper.plan_step(90, .5), 100.)
        with self.assertRaisesRegex(RuntimeError, 'outside calibrated limits'):
            stepper.observe(self.sample(stepper, 90.3), 100.5)


if __name__ == '__main__':
    unittest.main()
