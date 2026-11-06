"""Isolated ROS identity-bound pump tests, dry-run and hardware blocked.

Run RESCUE_RUN_ROS_TEST=1 ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 only.
The inherited harness forbids PWM, GPIO and exclusive-hardware constructors.
"""
import json
import time
from unittest.mock import patch
import unittest

import test_local_actuator_board as actuator_tests


class SelectedPumpBoardTests(actuator_tests.LocalActuatorBoardTests):
    identity = dict(target_id=7, selection_generation=2, registry_session='isolated-test-registry')

    def start_selected(self):
        import rescue_vision.local_actuator_node as module
        original = module.param
        def parameters(node, name, default):
            return original(node, name, True if name == 'require_target_selection' else default)
        with patch.object(module, 'param', side_effect=parameters):
            self.start(burst_s=1.)
        self.assertTrue(self.actuator.require_selection)
        self.set_aim_identity()
        self.run_until(lambda: all(self.actuator.gates(time.monotonic())),
                       message='identity-bound fresh dry-run gates did not become valid')

    def set_aim_identity(self, **changes):
        identity = dict(self.identity, **changes)
        self.aim_override = dict(identity, aim=dict(identity,
            coordinate_frame='undistorted_left_optical', image_size_wh=[1280,1088],
            camera_calibration_id='isolated-fixture-camera', image_locked=True))

    def request(self, changes=None, *, omit_identity=False):
        request = dict(action='arm_pump', lease_s=10., ttl_s=1.,
            issued_at=self.probe.get_clock().now().nanoseconds/1e9,
            command_id='isolated-' + str(time.monotonic_ns()))
        if not omit_identity:
            request.update(self.identity)
        request.update(changes or {})
        self.request_pub.publish(self.String(data=json.dumps(request)))
        return request

    def test_explicit_arm_requires_fresh_matching_complete_identity(self):
        self.start_selected()
        self.request(omit_identity=True)
        self.run_for(.08)
        self.assertFalse(self.actuator.guard.armed)
        for changes in (dict(target_id=8), dict(selection_generation=3),
                        dict(registry_session='other-registry'), dict(issued_at=0.),
                        dict(ttl_s=0.), dict(ttl_s=20.), dict(target_id=True)):
            with self.subTest(rejected=changes):
                self.request(changes)
                self.run_for(.08)
                self.assertFalse(self.actuator.guard.armed)
                self.assertFalse(self.actuator.pump_on)
        self.request()
        self.run_until(lambda: self.actuator.pump_on,
                       message='fresh explicit selected target did not arm dry-run pump')
        self.assertEqual(self.actuator.armed_target,
            (self.identity['target_id'], self.identity['selection_generation'], self.identity['registry_session']))
        self.assertTrue(self.actuator.io.dry_run)

    def test_target_generation_or_registry_change_cuts_pump_without_auto_rearm(self):
        self.start_selected()
        for field, changed in (('target_id', 8), ('selection_generation', 3),
                               ('registry_session', 'new-test-registry')):
            with self.subTest(changed=field):
                self.set_aim_identity()
                self.run_until(lambda: all(self.actuator.gates(time.monotonic())))
                self.request()
                self.run_until(lambda: self.actuator.pump_on)
                changed_at = time.monotonic()
                self.set_aim_identity(**{field: changed})
                self.run_until(lambda: not self.actuator.pump_on and not self.actuator.guard.armed,
                    timeout=.6, message='changed target identity did not cancel pump')
                self.assertLess(time.monotonic()-changed_at, .3)
                self.set_aim_identity()
                self.run_for(.55)
                self.assertFalse(self.actuator.guard.armed,
                                 'return of old target must not restore previous pump arm')
                self.assertFalse(self.actuator.pump_on)

    def test_manual_stop_turns_pump_off_immediately_and_keeps_it_off(self):
        self.start_selected()
        self.request()
        self.run_until(lambda: self.actuator.pump_on)
        stopped_at = time.monotonic()
        self.request_pub.publish(self.String(data=json.dumps(dict(action='disarm'))))
        self.run_until(lambda: not self.actuator.pump_on and not self.actuator.guard.armed, timeout=.5)
        self.assertLess(time.monotonic()-stopped_at, .2)
        self.assertIsNone(self.actuator.armed_target)
        self.run_for(.6)
        self.assertFalse(self.actuator.pump_on)
        self.assertFalse(self.actuator.guard.armed)

    def test_replayed_arm_command_and_inconsistent_nested_target_are_rejected(self):
        self.start_selected()
        original = self.request(dict(ttl_s=5.))
        self.run_until(lambda: self.actuator.pump_on)
        self.request_pub.publish(self.String(data=json.dumps(dict(action='disarm'))))
        self.run_until(lambda: not self.actuator.guard.armed)
        self.request_pub.publish(self.String(data=json.dumps(original)))
        self.run_for(.45)
        self.assertFalse(self.actuator.guard.armed, 'duplicate accepted arm must not create a new session')
        self.aim_override['aim']['target_id'] = 999
        self.run_for(.12)
        self.request()
        self.run_for(.15)
        self.assertFalse(self.actuator.guard.armed, 'top-level and nested target identities must agree')


if __name__ == '__main__':
    unittest.main()
