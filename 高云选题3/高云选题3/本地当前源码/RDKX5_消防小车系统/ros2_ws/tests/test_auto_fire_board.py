"""Isolated ROS actuator race tests. Hardware constructors are blocked by harness.

Run RESCUE_RUN_ROS_TEST=1 ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 only.
Direct callback ordering deliberately reproduces cross-publisher worst cases.
"""
import json
import time
import unittest

from test_selected_pump_board import SelectedPumpBoardTests
from rescue_vision.local_actuator_guard import PumpGuard


class AutoFireBoardTests(SelectedPumpBoardTests):
    def auto_request(self, action, run_id='auto-run'):
        data = dict(action=action, command_id=run_id,
                    issued_at=self.probe.get_clock().now().nanoseconds/1e9, ttl_s=2.)
        self.actuator.auto_request(self.String(data=json.dumps(data)))
        return data

    def direct_actuator(self, data):
        self.actuator.request(self.String(data=json.dumps(data)))

    def auto_arm(self, run_id='auto-run'):
        data = dict(action='arm_pump', auto_run_id=run_id, max_bursts=1, lease_s=3.,
                    command_id='auto-arm-'+str(time.monotonic_ns()), ttl_s=.5,
                    issued_at=self.probe.get_clock().now().nanoseconds/1e9, **self.identity)
        self.direct_actuator(data)
        return data

    def test_auto_arm_after_manual_disarm_is_rejected(self):
        self.start_selected()
        self.auto_request('start')
        self.assertEqual(self.actuator.auto_fence.active_run, 'auto-run')
        self.direct_actuator(dict(action='disarm'))
        self.auto_arm()
        self.assertFalse(self.actuator.guard.armed)
        self.assertIn('no active explicit run authority', self.actuator.last_request)
        self.run_for(.4)
        self.assertFalse(self.actuator.pump_on)

    def test_auto_cancel_stops_burst_but_late_completion_does_not_stop_manual(self):
        self.start_selected()
        self.auto_request('start')
        self.auto_arm()
        self.run_until(lambda: self.actuator.pump_on)
        self.assertEqual(self.actuator.armed_auto_run, 'auto-run')
        self.auto_request('cancel')
        self.assertFalse(self.actuator.pump_on)
        self.assertFalse(self.actuator.guard.armed)
        self.auto_arm()  # fresh ID/time cannot revive the cancelled run
        self.assertFalse(self.actuator.guard.armed)
        self.request()  # a later separate operator request can still arm
        self.run_until(lambda: self.actuator.pump_on)
        self.assertIsNone(self.actuator.armed_auto_run)
        self.direct_actuator(dict(action='disarm', auto_run_id='auto-run', retire_auto_run=True))
        self.assertTrue(self.actuator.pump_on)
        self.assertTrue(self.actuator.guard.armed)

    def test_auto_start_stop_message_reordering_and_completion_releases_authority(self):
        self.start_selected()
        for before in (True, False):
            run_id = 'auto-before' if before else 'auto-after'
            if before: self.direct_actuator(dict(action='disarm', preserve_auto_run_id=run_id))
            self.auto_request('start', run_id)
            if not before: self.direct_actuator(dict(action='disarm', preserve_auto_run_id=run_id))
            self.run_for(.15)
            self.assertEqual(self.states[-1][1]['auto_authorized_run_id'], run_id)
            self.direct_actuator(dict(action='disarm', auto_run_id=run_id, retire_auto_run=True))
            self.assertIsNone(self.actuator.auto_fence.active_run)
        self.assertFalse(self.actuator.pump_on)

    def test_auto_per_request_burst_budget_is_enforced_in_actuator(self):
        self.start_selected()
        self.actuator.guard = PumpGuard(dict(max_bursts=3, burst_s=.04, stable_lock_s=.02, cooldown_s=.5))
        self.auto_request('start')
        self.auto_arm()
        self.run_for(1.5)
        starts = [c for c in self.actuator.io.commands if c.get('operation') == 'pump' and c.get('enabled')]
        self.assertEqual(len(starts), 1)
        self.assertEqual(self.actuator.guard.state, 'EXHAUSTED')
        self.assertEqual(self.actuator.guard.session_max_bursts, 1)
        self.assertFalse(self.actuator.pump_on)

    def test_auto_fpga_estop_revokes_authority_and_all_motion_owners(self):
        self.start_selected()
        self.auto_request('start')
        self.auto_arm()
        self.run_until(lambda: self.actuator.pump_on)
        mission = dict(self.actuator.mission, emergency_stop=True,
                       stamp=self.probe.get_clock().now().nanoseconds/1e9)
        self.actuator.mission_state(self.String(data=json.dumps(mission)))
        self.assertFalse(self.actuator.pump_on)
        self.assertFalse(self.actuator.pwm_active)
        self.assertIsNone(self.actuator.auto_fence.active_run)
        for owner in ('fire', 'traffic', 'road', 'mission_home'):
            self.assertFalse(self.actuator.owner_allowed(time.monotonic(), owner))
        self.assertFalse(self.actuator.gates(time.monotonic())[0])
        self.run_for(.4)
        self.assertFalse(self.actuator.guard.armed)
        self.assertFalse(self.actuator.pump_on)


def load_tests(loader, tests, pattern):
    # Reuse the hardware-blocked harness without rerunning inherited tests here.
    return unittest.TestSuite(AutoFireBoardTests(name) for name in AutoFireBoardTests.__dict__
                              if name.startswith('test_auto_'))


if __name__ == '__main__':
    unittest.main()
