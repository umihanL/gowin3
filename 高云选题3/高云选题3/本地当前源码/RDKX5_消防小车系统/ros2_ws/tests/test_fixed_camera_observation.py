"""Fixed camera observation never sweeps the nozzle; all transport is fake."""
import json
import unittest

from rescue_vision.auto_fire import AutoFireRound
import test_auto_fire as auto_fixture
import test_traffic_task_control as mission_fixture


class FixedObservationRoundTests(unittest.TestCase):
    def start(self, action='start'):
        self.round = AutoFireRound(fixed_camera_observation=True)
        self.round.request(dict(action=action, command_id='run1', issued_at=100., ttl_s=2.),100.)
        return self.round

    def evidence(self, now, mode='inspect', fire=False):
        value = auto_fixture.AutoFireRoundTests.evidence(self, now, mode)
        value['mission'].update(fixed_camera_observation=True, gimbal_ready=False,
                               scan_hardware_ready=True)
        if not fire:
            value['targets']['targets'] = []
        return value

    def test_no_fire_finishes_after_one_current_view_window_without_pan_offsets(self):
        for action in ('start', 'scan'):
            with self.subTest(action=action):
                c = self.start(action)
                self.assertEqual(c.tick(100.01, **self.evidence(100.01)), [])
                self.assertEqual(c.scan_ready_since, 100.01)
                self.assertEqual(c.tick(100.8, **self.evidence(100.8)), [])
                result = c.tick(101.02, **self.evidence(101.02))
                self.assertFalse(c.active)
                self.assertEqual(c.reason, 'no_selectable_fire_in_current_view')
                self.assertEqual((c.scan_index, c.pan_offset), (0, 0.))
                self.assertFalse(any(k == 'select' or k == 'actuator' and v['action'] == 'arm_pump'
                                     for k,v in result))

    def test_missing_or_stale_camera_data_cannot_be_called_a_clear_view(self):
        for fault in ('missing', 'stale', 'invalid', 'missing_session'):
            with self.subTest(fault=fault):
                c = self.start(); c.tick(100.01, **self.evidence(100.01))
                state = self.evidence(101.1)
                if fault == 'missing': state['targets'] = {}
                elif fault == 'stale': state['targets']['capture_stamp'] = 100.
                elif fault == 'invalid': state['targets']['valid'] = False
                else: state['targets']['registry_session'] = ''
                self.assertEqual(c.tick(101.1, **state), [])
                self.assertTrue(c.active)
                self.assertEqual(c.reason, 'waiting_for_fixed_camera_observation')

    def test_stop_and_same_camera_mode_are_required_before_counting_observation(self):
        for bad in ({'vehicle_stopped':False}, {'fixed_camera_observation':False}):
            with self.subTest(bad=bad):
                c = self.start(); state = self.evidence(100.1, fire=True)
                state['mission'].update(bad)
                self.assertEqual(c.tick(100.1, **state), [])
                self.assertIsNone(c.scan_ready_since)
                self.assertEqual(c.phase, 'SEARCH')

    def test_detection_without_settled_nozzle_can_select_but_cannot_arm(self):
        c = self.start()
        c.tick(100.01, **self.evidence(100.01, fire=True))
        old = self.evidence(100.02, fire=True)
        old['targets']['capture_stamp'] = 100.
        self.assertEqual(c.tick(100.02, **old), [])
        result = c.tick(100.03, **self.evidence(100.03, fire=True))
        self.assertEqual(dict(result)['select']['target_id'], 7)
        self.assertEqual(c.phase, 'AIM')
        self.assertEqual(c.tick(100.1, **self.evidence(100.1, mode='fire', fire=True)), [])
        self.assertEqual(c.phase, 'AIM')
        bad_lock = self.evidence(100.2, mode='fire', fire=True)
        bad_lock['mission']['gimbal_ready'] = True
        bad_lock['proposal']['pump_candidate'] = False
        self.assertEqual(c.tick(100.2, **bad_lock), [])
        self.assertEqual(c.phase, 'AIM')
        ready = self.evidence(100.3, mode='fire', fire=True)
        ready['mission']['gimbal_ready'] = True
        arm = dict(c.tick(100.3, **ready))['actuator']
        self.assertEqual(arm['action'], 'arm_pump')

    def test_fixed_scan_lists_targets_without_selecting_or_arming(self):
        c = self.start('scan')
        c.tick(100.01, **self.evidence(100.01, fire=True))
        result = c.tick(100.02, **self.evidence(100.02, fire=True))
        self.assertEqual(c.phase, 'WAIT_SELECTION')
        self.assertEqual(c.observed_targets,[dict(target_id=7,registry_session='camera-session')])
        self.assertNotIn('select', dict(result))
        self.assertEqual(dict(result)['actuator']['action'], 'disarm')


class FixedObservationMissionTests(unittest.TestCase):
    setUp = mission_fixture.TrafficTaskTests.setUp

    def prepare(self, fixed, action):
        node = self.node
        node.fixed_camera_observation = fixed
        node.extension_enabled = True
        node.mount['mounting_verified'] = True
        node.event(mission_fixture.Message(event_type=mission_fixture.Message.STATUS,vehicle_stopped=True))
        node.link_state(mission_fixture.Message(data=json.dumps(dict(stamp=100.,enabled=True,dry_run=False,
            status_fresh=True,feedback_fresh=True,actual_angles_deg=[90.,90.]))))
        node.auto_request(mission_fixture.Message(data=json.dumps(dict(action=action,
            command_id='run1',issued_at=100.,ttl_s=2.))))
        node.auto_state(mission_fixture.Message(data=json.dumps(dict(stamp=100.,active=True,phase='SEARCH',
            scan_only=action=='scan',command_id='run1',pan_offset_deg=20.))))
        return node

    def test_fixed_camera_auto_search_never_sweeps_nozzle_even_with_live_motors(self):
        node = self.prepare(True, 'start')
        node.tick()
        self.assertEqual(node.gimbal_pub.values, [])
        self.assertIsNone(node.target)
        state = node.state_pub.values[-1]
        self.assertEqual(state['camera_pose'], 'fixed_observe')
        self.assertTrue(state['fixed_camera_observation'])
        self.assertFalse(state['gimbal_ready'])
        self.assertFalse(state['scan_motion_allowed'])

    def test_fixed_scan_only_observes_and_does_not_need_nozzle_reference(self):
        node = self.prepare(True, 'scan')
        node.mount['mounting_verified'] = False
        node.link = {}; node.link_time = None
        node.tick()
        self.assertEqual(node.gimbal_pub.values, [])
        self.assertTrue(node.state_pub.values[-1]['scan_hardware_ready'])
        self.assertFalse(node.state_pub.values[-1]['scan_motion_allowed'])

    def test_old_moving_camera_mode_retains_scan_motion(self):
        node = self.prepare(False, 'start')
        node.tick()
        self.assertEqual(len(node.gimbal_pub.values), 1)
        msg = node.gimbal_pub.values[0]
        self.assertEqual((msg.owner,msg.pan_deg,msg.tilt_deg), ('mission_home',110.,90.))


if __name__ == '__main__':
    unittest.main()
