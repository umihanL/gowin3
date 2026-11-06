"""Installed gimbal completion keeps torque without replaying commands."""
import copy
import json
import types
import unittest
from unittest import mock

from test_zdt_fd8_control import fixture, config, Clock
import test_gimbal_self_test_node as node_fixture
from rescue_vision import zdt_fd8_control as fd8, zdt_vendor_bench as vendor, zdt_xs_protocol as zdt
from rescue_vision.pd42_protocol import Command as PD
from rescue_vision.fpga_pump_io import FPGAPumpIO


WRITES = (zdt.Command.ENABLE, zdt.Command.POSITION_EMM, PD.ENABLE, PD.ABSOLUTE_POSITION)


class HoldTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.timer = mock.patch('rescue_vision.mixed_actuator_io.time.monotonic', self.clock)
        self.timer.start()
        self.addCleanup(self.timer.stop)
        self.io, self.pan, self.tilt, self.events = fixture()

    def arrive(self, target=(1.8, 1.8)):
        for _ in range(40):
            self.io.set_gimbal(*target, speed_deg_s=6, lease_s=.5)
            if self.io._fd8.pending:
                self.pan.arrive()
            self.io._cycle()
            if self.io.feedback()['settled_measured']:
                return
            self.clock.tick()
        self.fail('fixture failed to arrive')

    def hold(self):
        self.arrive()
        self.io.hold_gimbal()
        self.events.clear()

    def test_completion_keeps_existing_torque_after_original_lease_without_writes(self):
        self.hold()
        expected = self.io._fd8.expected
        for _ in range(6):
            self.clock.tick(.2)
            self.io._cycle()
        feedback = self.io.feedback()
        self.assertTrue(feedback['holding_position'] and feedback['holding_measured'])
        self.assertTrue(feedback['hardware_written'])
        self.assertTrue(self.pan.enabled and self.tilt.enabled)
        self.assertFalse(feedback['command_accepted'] or feedback['settled_measured'])
        self.assertIsNone(feedback['target_deg'])
        self.assertAlmostEqual(feedback['hold_target_deg'][1], 1.8)
        self.assertEqual(self.io._fd8.expected, expected)
        self.assertFalse(any(e[1] in WRITES for e in self.events))

    def test_consecutive_move_takes_over_same_reference_without_enable(self):
        self.hold()
        reference = copy.deepcopy(self.io.servo['pan']['zdt_fd8_snapshot'])
        self.arrive((0, 0))
        self.assertFalse(self.io.feedback()['holding_position'])
        self.assertTrue(self.io.feedback()['settled_measured'])
        self.assertFalse(any(e[1] in (zdt.Command.ENABLE, PD.ENABLE) for e in self.events))
        self.assertEqual(self.io.servo['pan']['zdt_fd8_snapshot'], reference)
        self.assertAlmostEqual(self.tilt.counts, self.io.servo['tilt']['reference_counts'], delta=1)

    def test_hold_cannot_convert_unarrived_stale_or_expired_goal_to_success(self):
        for failure in ('unarrived', 'stale', 'expired'):
            self.io, self.pan, self.tilt, self.events = fixture()
            if failure == 'unarrived':
                self.io.set_gimbal(1.8, 1.8, speed_deg_s=6, lease_s=.5)
                self.io._cycle()
            else:
                self.arrive()
                if failure == 'stale': self.io._received -= 1.
                else: self.io._goal['deadline'] = self.clock()-1
            with self.subTest(failure=failure), self.assertRaisesRegex(RuntimeError, 'measured-arrived'):
                self.io.hold_gimbal()
            self.assertFalse(self.io.feedback()['holding_position'])

    def test_explicit_stop_clears_hold_and_releases_both_axes(self):
        self.hold()
        self.io.disable_gimbal()
        self.io._cycle()
        self.assertFalse(self.io.feedback()['holding_position'] or self.io._active)
        self.assertFalse(self.pan.enabled or self.tilt.enabled)
        self.assertFalse(any(e[1] in (zdt.Command.POSITION_EMM, PD.ABSOLUTE_POSITION) for e in self.events))

    def test_hold_transition_during_worker_gate_does_not_release_completed_goal(self):
        self.arrive()
        old_goal = copy.deepcopy(self.io._goal)
        self.io.hold_gimbal()
        self.events.clear()
        self.assertFalse(self.io._motion_gate(old_goal, self.clock()))
        self.assertTrue(self.io._active and self.pan.enabled and self.tilt.enabled)
        self.assertFalse(self.events)

    def test_hold_fault_keeps_stop_path_and_never_reenables_or_retries(self):
        for disturbance in ('target', 'tilt_drift', 'enable_lost'):
            self.io, self.pan, self.tilt, self.events = fixture()
            self.hold()
            if disturbance == 'target': self.pan.target_counts += 3
            elif disturbance == 'tilt_drift': self.tilt.counts += 100
            else: self.tilt.enabled = False
            self.io._run()
            with self.subTest(disturbance=disturbance):
                self.assertTrue(self.io.feedback()['error'])
                self.assertFalse(self.io.feedback()['holding_position'])
                self.assertFalse(self.pan.enabled or self.tilt.enabled)
                self.assertFalse(any(e[1] in (zdt.Command.POSITION_EMM, PD.ABSOLUTE_POSITION) for e in self.events))

    def test_close_retains_worker_shutdown_ownership_for_holding_axes(self):
        self.hold()
        # Model the existing owner's join after close has set its quit event.
        self.io._thread = types.SimpleNamespace(join=lambda timeout: self.io._run(), is_alive=lambda: False)
        for motor in (self.pan, self.tilt): motor.close = lambda: None
        self.io.close()
        self.assertTrue(self.io.closed)
        self.assertFalse(self.pan.enabled or self.tilt.enabled or self.io._active)
        self.assertFalse(self.io.feedback()['holding_position'])

    def test_stationary_tolerance_has_exact_error_boundary_and_count_boundary(self):
        sample = dict(position_counts=27, position_before_counts=27, retained_target_counts=0,
                      realtime_target_counts=0, position_error_deg=.15, speed_rpm=0,
                      enabled=True, arrived=True)
        self.assertTrue(vendor.settled(sample, 0))
        self.assertFalse(vendor.settled(dict(sample, position_error_deg=.150001), 0))
        self.assertFalse(vendor.settled(dict(sample, position_counts=28, position_before_counts=28), 0))
        self.assertFalse(vendor.settled(dict(sample, retained_target_counts=3), 0))
        self.assertFalse(vendor.settled(dict(sample, position_before_counts=20), 0))

    def test_snapshot_tolerance_and_live_stability_use_same_point_fifteen_limit(self):
        for delta, valid in ((27, True), (28, False)):
            cfg = config(); axis = cfg['servo']['pan']
            snapshot = axis['zdt_fd8_snapshot']
            snapshot['retained_target_counts'] = snapshot['realtime_target_counts'] = axis['reference_counts']+delta
            with self.subTest(delta=delta):
                if valid:
                    fd8.validate_snapshot(axis, 115200, .5)
                else:
                    with self.assertRaisesRegex(ValueError, 'unaligned retained target'):
                        fd8.validate_snapshot(axis, 115200, .5)
                stepper = fd8.FD8Stepper(axis, snapshot)
                sample = dict(position_before_counts=axis['reference_counts'], position_counts=axis['reference_counts'],
                              retained_target_counts=snapshot['retained_target_counts'],
                              realtime_target_counts=snapshot['realtime_target_counts'],
                              position_error_deg=delta*vendor.COUNT_DEG, speed_rpm=0, enabled=True, arrived=True)
                for i in range(3): ready = stepper.observe(sample, i*.04)
                self.assertEqual(ready, valid)


class HoldNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.mod = node_fixture.module()

    setUp = node_fixture.FD8NodeTests.setUp
    run_tick = node_fixture.FD8NodeTests.run_tick
    manual_request = node_fixture.FD8NodeTests.manual_request

    def run_to_completion(self, controller):
        for _ in range(160):
            if self.io._fd8.pending: self.pan.arrive()
            self.run_tick(round(self.node.now+.05, 6))
            if controller.phase == 'completed': return
            if controller.phase == 'failed': self.fail(controller.reason)
        self.fail('node did not finish')

    def test_manual_completion_wrapper_holds_then_second_request_can_return(self):
        self.node.io = FPGAPumpIO(self.io, self.node.fpga_pump, self.io.config['pump'])
        self.run_tick(10.); self.run_tick(10.05)
        self.manual_request(1.8, 'outward')
        self.run_to_completion(self.node.manual)
        self.events.clear()
        for _ in range(15): self.run_tick(round(self.node.now+.05, 6))
        self.assertTrue(self.io.feedback()['holding_position'])
        self.assertTrue(self.node.output[-1]['gimbal_holding_measured'])
        self.assertTrue(self.node.manual_output[-1]['can_start'])
        self.assertFalse(any(e[1] in WRITES for e in self.events))
        self.manual_request(0, 'return')
        self.run_to_completion(self.node.manual)
        self.assertFalse(self.node.fault)
        self.assertFalse(any(e[1] in (zdt.Command.ENABLE, PD.ENABLE) for e in self.events))
        self.assertFalse(self.node.pump_on or self.node.guard.armed)

    def test_self_test_completion_holds_but_explicit_stop_releases(self):
        self.run_tick(10.); self.run_tick(10.05)
        value = dict(action='start', request_id='selftest-hold', issued_at=self.node.now,
                     ttl_s=2., axes='pan', amplitude_deg=1., speed_deg_s=6.)
        with mock.patch.object(self.mod.time, 'monotonic', return_value=self.node.now):
            self.node.self_test_request(types.SimpleNamespace(data=json.dumps(value)))
        self.run_to_completion(self.node.self_test)
        self.assertTrue(self.io.feedback()['holding_position'])
        self.assertTrue(self.pan.enabled and self.tilt.enabled)
        with mock.patch.object(self.mod.time, 'monotonic', return_value=self.node.now):
            self.node.manual_request(types.SimpleNamespace(data='{"action":"stop"}'))
        self.run_tick(round(self.node.now+.05, 6))
        self.assertFalse(self.pan.enabled or self.tilt.enabled)
        self.assertFalse(self.io.feedback()['holding_position'])


if __name__ == '__main__': unittest.main(verbosity=2)
