"""Exercise live reference callbacks with actual node logic and no UART/ROS."""
from copy import deepcopy
import json
import threading
import types
import unittest
from unittest import mock

from test_gimbal_self_test_node import module
from test_zdt_fd8_control import config


class LiveIO:
    requires_measured_feedback = True
    backend = 'mixed_serial'
    def __init__(self):
        self.config = config()
        self.config['dry_run'] = False
        self.config['servo'].update(reference_confirmed_this_start=False,
            reference_id=None, live_readback_until_reference=True)
        self.counts = [-7, 27200]
        self.commands = []
        self.started = threading.Event()
        self.release = threading.Event()
        self.calls = 0
        self.readback = dict(error='', raw_feedback_fresh=True, raw_feedback_age_s=.01,
            serial_connected=True, feedback_fresh=False, actual_angles_deg=None,
            motor_counts=self.counts, motor_rpm=[0, 0], motor_enabled=[True, True],
            reference_confirmed_this_start=False, reference_capture_ready=True,
            reference_capture_reason='ready', reference_capture_supported=True)

    def feedback(self): return deepcopy(self.readback)
    def set_gimbal(self, *a, **k): self.commands.append('move')
    def set_pump(self, enabled): self.commands.append(('pump', enabled))
    def disable_gimbal(self): self.commands.append('disable')
    def close(self): self.commands.append('close')

    def commit_reference(self, **flags):
        self.calls += 1
        self.started.set()
        if not self.release.wait(2): raise RuntimeError('test release timeout')
        if flags['cancel_event'].is_set(): raise RuntimeError('reference_capture_cancelled')
        self.config['servo'].update(reference_confirmed_this_start=True, reference_id='manual-zero-test',
            live_readback_until_reference=False)
        self.readback.update(reference_confirmed_this_start=True, actual_angles_deg=[0, 0], feedback_fresh=True)
        return dict(config=deepcopy(self.config), reference_id='manual-zero-test', boot_id='boot-test',
            recovered_error=self.readback.get('reference_recovery_error', ''))


class LiveNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.mod = module()

    def setUp(self):
        self.io = LiveIO()
        self.mod.LocalActuator.parameters = dict(control_config=dict(enabled=True,
            pump_enabled=False, max_speed_deg_s=6.), io_config=self.io.config,
            dry_run=False, pd42_reference_confirmed=False)
        with mock.patch.object(self.mod, 'create_actuator_io', return_value=self.io):
            self.node = self.mod.LocalActuator()
        self.node.now = 10.
        self.node.last_tick = 9.9
        self.node.mission = dict(mode='inspect', vehicle_stopped=True, emergency_stop=False,
            traffic_active=False, auto_fire_active=False)
        self.node.mission_at = 10.
        self.save = mock.patch.object(self.mod, 'persist_reference')
        self.persist = self.save.start()
        self.addCleanup(self.save.stop)
        self.addCleanup(self.cleanup_worker)

    def cleanup_worker(self):
        job = self.node.reference_job
        if job:
            job['cancel'].set()
            self.io.release.set()
            job['thread'].join(1.)

    def request(self, **overrides):
        value = dict(action='confirm_reference', request_id='reference-1', issued_at=10., ttl_s=2.,
            operator_zero_confirmed=True, supported_load=True, stationary_confirmed=True)
        value.update(overrides)
        with mock.patch.object(self.mod.time, 'monotonic', return_value=10.):
            self.node.manual_request(types.SimpleNamespace(data=json.dumps(value)))

    def tick(self):
        with mock.patch.object(self.mod.time, 'monotonic', return_value=10.1):
            self.node.tick()

    def finish(self):
        job = self.node.reference_job
        self.io.release.set()
        self.assertTrue(job['done'].wait(1.))
        self.tick()

    def test_live_raw_feedback_publishes_without_inventing_joint_angles(self):
        self.node.publish_status(10.1, self.node.guard.snapshot(), force=True)
        state = self.node.output[-1]
        self.assertFalse(state['dry_run'])
        self.assertTrue(state['raw_feedback_fresh'] and state['serial_connected'])
        self.assertEqual(state['motor_counts'], [-7, 27200])
        self.assertIsNone(state['actual_angles_deg'])
        self.assertFalse(state['feedback_fresh'] or state['reference_confirmed_this_start'])
        self.assertTrue(self.node.manual_output[-1]['reference_capture_ready'])
        self.node.gimbal(types.SimpleNamespace())
        self.assertEqual(self.io.commands, [])

    def test_explicit_capture_is_async_persists_and_rebuilds_controllers_without_motion(self):
        self.request()
        self.assertTrue(self.io.started.wait(.5))
        self.assertEqual(self.node.manual.phase, 'referencing')
        self.node.publish_status(10.1, self.node.guard.snapshot(), force=True)
        self.assertFalse(self.node.manual_output[-1]['can_start'])
        self.assertFalse(self.node.manual_output[-1]['reference_capture_ready'])
        self.node.gimbal(types.SimpleNamespace())
        self.finish()
        self.persist.assert_called_once()
        self.assertTrue(self.node.manual.servo['reference_confirmed_this_start'])
        self.assertTrue(self.node.self_test.servo['reference_confirmed_this_start'])
        self.assertEqual(self.node.manual.last_request_id, 'reference-1')
        self.assertEqual(self.node.manual.last_result, 'completed')
        self.assertEqual(self.node.manual.last_request_reason, 'gimbal_reference_recorded_no_motion')
        self.assertEqual(self.io.commands, [])

    def test_three_explicit_confirmations_and_current_request_are_required(self):
        for change in ({'operator_zero_confirmed': False}, {'supported_load': 1},
                       {'stationary_confirmed': False}, {'issued_at': 5.}, {'ttl_s': 20.}):
            with self.subTest(change=change):
                self.request(request_id=str(change), **change)
                self.assertEqual(self.node.manual.last_result, 'rejected')
                self.assertIsNone(self.node.reference_job)
        self.assertEqual(self.io.calls, 0)

    def test_state_gates_reject_stale_moving_fault_and_missing_raw_feedback(self):
        changes = [lambda: setattr(self.node, 'mission_at', 0.),
                   lambda: self.node.mission.update(vehicle_moving=True),
                   lambda: self.node.mission.update(auto_fire_active=True),
                   lambda: self.io.readback.update(raw_feedback_fresh=False),
                   lambda: self.io.readback.update(motor_rpm=[0, 1]),
                   lambda: setattr(self.node, 'estop', True)]
        for index, change in enumerate(changes):
            self.node.mission_at=10.; self.node.mission.update(vehicle_stopped=True, vehicle_moving=False, auto_fire_active=False)
            self.io.readback.update(raw_feedback_fresh=True, motor_rpm=[0, 0]); self.node.estop=False
            change(); self.request(request_id='case-'+str(index))
            self.assertIsNone(self.node.reference_job)
        self.assertEqual(self.io.calls, 0)

    def test_stop_cancels_pending_capture_and_does_not_save_or_restore_readiness(self):
        self.request()
        self.assertTrue(self.io.started.wait(.5))
        self.request(action='stop', request_id='stop-1')
        self.assertTrue(self.node.reference_job['cancel'].is_set())
        self.finish()
        self.persist.assert_not_called()
        self.assertEqual(self.node.manual.last_result, 'cancelled')
        self.assertFalse(self.io.config['servo']['reference_confirmed_this_start'])
        self.assertNotIn('move', self.io.commands)

    def test_context_change_cancels_pending_capture(self):
        self.request()
        self.assertTrue(self.io.started.wait(.5))
        self.node.mission['vehicle_moving'] = True
        self.tick()
        self.assertTrue(self.node.reference_job['cancel'].is_set())
        self.finish()
        self.persist.assert_not_called()
        self.assertEqual(self.node.manual.last_request_reason, 'manual_vehicle_known_moving')

    def test_failed_persistence_blocks_motion_even_after_measured_capture(self):
        self.persist.side_effect = OSError('disk unavailable')
        self.request(); self.assertTrue(self.io.started.wait(.5)); self.finish()
        self.assertIn('reference_profile_save_failed', self.node.fault)
        self.assertEqual(self.node.manual.last_result, 'error')
        self.assertFalse(self.node.manual_output[-1]['can_start'])
        self.assertEqual(self.io.commands, [])

    def test_confirmed_reference_can_be_recaptured_explicitly_multiple_times(self):
        self.io.readback['reference_confirmed_this_start'] = True
        self.request()
        self.assertTrue(self.io.started.wait(.5)); self.finish()
        self.request(request_id='reference-2'); self.finish()
        self.assertEqual(self.persist.call_count, 2)
        self.assertEqual(self.io.calls, 2)
        self.assertEqual(self.node.manual.last_request_id, 'reference-2')
        self.assertEqual(self.io.commands, [])

    def test_raw_verified_reference_fault_can_be_recovered_but_no_other_fault_or_estop(self):
        error = ('tilt measured position outside calibrated limits/reference: actual_deg=-8.79609375, '
                 'counts=26238, limits_deg=[0.0,60.0], feedback_limit_tolerance_deg=0.2')
        self.node.fault = error
        self.io.readback.update(reference_recovery_permitted=True, reference_recovery_error=error)
        self.request(); self.assertTrue(self.io.started.wait(.5)); self.finish()
        self.assertEqual(self.node.fault, '')
        self.assertEqual(self.io.commands, [])
        self.node.fault = 'tilt driver fault: 3'
        self.io.readback['reference_recovery_error'] = self.node.fault
        self.request(request_id='other-fault')
        self.assertIsNone(self.node.reference_job)
        self.assertEqual(self.node.manual.last_result, 'rejected')
        self.node.fault = error; self.io.readback['reference_recovery_error'] = error
        self.node.estop = True
        self.request(request_id='estop')
        self.assertIsNone(self.node.reference_job)
        self.assertEqual(self.io.calls, 1)

    def test_new_fault_during_recapture_cancels_without_clearing_it(self):
        self.request(); self.assertTrue(self.io.started.wait(.5))
        self.node.fault = 'USB serial disconnected'
        self.tick(); self.finish()
        self.assertEqual(self.node.fault, 'USB serial disconnected')
        self.assertEqual(self.node.manual.last_result, 'cancelled')
        self.persist.assert_not_called()

    def test_fault_publication_retains_real_guard_armed_state(self):
        self.io.readback['error'] = 'tilt driver fault: 3'
        self.tick()
        self.assertIn('armed', self.node.output[-1]['guard'])
        self.assertEqual(self.node.output[-1]['guard']['armed'], self.node.guard.snapshot()['armed'])

    def restored_event(self):
        self.io.config['servo'].update(reference_confirmed_this_start=True,
            reference_id='fixed-origin', live_readback_until_reference=False)
        result = dict(config=deepcopy(self.io.config), reference_id='fixed-origin', boot_id='new-boot',
            restore_kind='absolute_encoder', restored_angles_deg=[30., 36.], motion_commands_sent=False)
        self.io.take_restored_reference = mock.Mock(side_effect=[result, None])
        self.io.finish_reference_restore = mock.Mock()
        return result

    def test_automatic_absolute_restore_is_persisted_once_before_opening_control_without_motion(self):
        result = self.restored_event()
        self.tick()
        self.persist.assert_called_once_with(result, self.node.cfg)
        self.io.finish_reference_restore.assert_called_once_with('fixed-origin', True)
        self.assertTrue(self.node.manual.servo['reference_confirmed_this_start'])
        self.assertEqual(self.node.manual.reason, 'absolute_reference_restored_no_motion')
        self.assertEqual(self.io.commands, [])
        self.assertFalse(self.node.consume_restored_reference())
        self.assertEqual(self.persist.call_count, 1)

    def test_automatic_restore_persistence_failure_keeps_outputs_blocked_without_retry(self):
        self.restored_event()
        self.persist.side_effect = OSError('disk full')
        self.tick()
        self.assertIn('absolute_reference_restore_save_failed', self.node.fault)
        self.assertEqual(self.io.finish_reference_restore.call_args.args[:2], ('fixed-origin', False))
        self.assertEqual(self.io.commands, [])
        self.assertFalse(self.node.consume_restored_reference())
        self.assertEqual(self.persist.call_count, 1)

    def test_absolute_restore_never_clears_a_latched_estop_or_unrelated_fault(self):
        self.restored_event()
        self.node.estop = True
        self.node.fault = 'unrelated driver failure'
        self.tick()
        self.assertTrue(self.node.estop)
        self.assertEqual(self.node.fault, 'unrelated driver failure')
        self.assertFalse(self.node.manual_output[-1]['can_start'])
        self.assertEqual(self.io.commands, [])

    def test_unknown_disabled_pump_allows_reference_and_gimbal_context_but_on_or_pending_blocks(self):
        self.node.pump_on = None
        self.assertFalse(self.node.self_test_context(10.)['pump_on'])
        self.assertTrue(self.node.reference_readiness(10., self.io.feedback())[0])
        self.node.pump_enabled=True
        self.assertTrue(self.node.self_test_context(10.)['pump_on'])
        self.node.pump_enabled=False
        self.node.pump_on=True
        self.assertTrue(self.node.pump_blocks_gimbal())
        self.node.pump_on=None
        self.node.fpga_pump.requested_on=True
        self.assertTrue(self.node.pump_blocks_gimbal())
        self.node.fpga_pump.requested_on=False
        self.node.manual_pump_transport.requested_on=True
        self.assertTrue(self.node.pump_blocks_gimbal())
        self.node.manual_pump_transport.requested_on=False
        self.node.guard.armed=True
        self.assertTrue(self.node.pump_blocks_gimbal())

    def test_missing_fpga_stop_feedback_allows_independent_reference_without_faking_stop(self):
        self.node.mission['vehicle_stopped'] = False
        self.node.extension = dict(real_uart=False, status_fresh=False, active=None, task='vision')
        self.node.extension_at = 10.
        self.assertTrue(self.node.reference_readiness(10., self.io.feedback())[0])
        self.assertFalse(self.node.mission['vehicle_stopped'])
        context = self.node.manual_context(10.)
        self.assertFalse(context['vehicle_stopped'])
        self.assertFalse(context['manual_vehicle_feedback_known'])
        self.node.publish_status(10.1, self.node.guard.snapshot(), force=True)
        state = self.node.manual_output[-1]
        self.assertTrue(state['independent_manual_adjustment'])
        self.assertFalse(state['manual_vehicle_feedback_required'])
        self.assertFalse(state['manual_vehicle_feedback_known'])
        self.request(); self.assertTrue(self.io.started.wait(.5)); self.finish()
        self.persist.assert_called_once()
        self.assertFalse(self.node.mission['vehicle_stopped'])
        self.assertEqual(self.io.commands, [])

    def test_fresh_moving_fpga_and_active_navigation_block_manual_and_reference(self):
        self.node.mission['vehicle_stopped'] = False
        self.node.extension = dict(real_uart=True, status_fresh=True, vehicle_stopped=False, active=None)
        self.node.extension_at = 10.
        self.assertEqual(self.node.reference_readiness(10., self.io.feedback()), (False, 'manual_vehicle_known_moving'))
        self.assertFalse(self.node.manual_context(10.)['manual_vehicle_policy_ready'])
        self.node.extension.update(status_fresh=False, active=dict(phase='navigating'))
        self.assertFalse(self.node.reference_readiness(10., self.io.feedback())[0])
        self.node.extension['active'] = None
        self.node.mission['mode'] = 'navigate'
        self.assertFalse(self.node.reference_readiness(10., self.io.feedback())[0])

    def test_missing_stop_telemetry_does_not_relax_selftest_or_automatic_context(self):
        self.node.mission['vehicle_stopped'] = False
        self.io.config['servo'].update(reference_confirmed_this_start=True, live_readback_until_reference=False)
        self.io.readback.update(reference_confirmed_this_start=True, feedback_fresh=True, actual_angles_deg=[0., 0.])
        self.node.self_test = self.mod.GimbalSelfTest(self.io.config['servo'])
        self.node.manual = self.mod.GimbalManual(self.io.config['servo'])
        self.assertTrue(self.node.manual.readiness(self.io.feedback(), self.node.manual_context(10.))[0])
        ready, reason = self.node.self_test.readiness(self.io.feedback(), self.node.self_test_context(10.))
        self.assertFalse(ready)
        self.assertEqual(reason, 'measured_vehicle_stop_required')
        self.node.mission['mode'] = 'fire'
        self.assertFalse(self.node.owner_allowed(10., 'fire'))
        self.assertFalse(self.node.manual.readiness(self.io.feedback(), self.node.manual_context(10.))[0])


if __name__ == '__main__': unittest.main()
