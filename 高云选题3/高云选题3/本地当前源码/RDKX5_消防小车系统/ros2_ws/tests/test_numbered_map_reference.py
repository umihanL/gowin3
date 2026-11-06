"""Numbered destinations accept an actual confirmed map origin and heading."""
import unittest

from test_area_navigation import Harness
from rescue_vision import ov5640_protocol as wire
from rescue_vision.task_interaction import TaskRouter, parse_task_text, point_request_guard


def confirm_map(h, x=327, y=159, heading=-90):
    h.c.route_catalog = dict(width_cm=352, height_cm=320)
    h.c.request(dict(action='set_reference', command_id=f'confirm-map-{x}-{y}-{heading}', source='web',
        issued_at=h.now, ttl_s=3., confirmed=True, x_cm=x, y_cm=y, heading_deg=heading), h.now+.002)


def go(h, point=4, cid='goto-point'):
    request = dict(parse_task_text(f'去{point}号点'), command_id=cid, source='web',
                   issued_at=h.now, ttl_s=3.)
    h.c.request(request, h.now+.003)
    return request


class NumberedMapReferenceTests(unittest.TestCase):
    def test_board_repro_uses_confirmed_map_origin_and_heading_without_gimbal(self):
        h = Harness()
        confirm_map(h)
        h.c.actuator.update(enabled=False, dry_run=True, reference_confirmed_this_start=False,
                            status_fresh=False, feedback_fresh=False)
        self.assertNotEqual(h.c.area.readiness(h.now+.003), '')
        state = h.c.state(h.now+.003)
        self.assertTrue(state['can_navigate_point'], state['point_navigation_reason'])
        self.assertFalse(state['point_reference']['confirmed'])
        self.assertEqual(state['point_start_reference']['cell'], [163, 80])
        self.assertEqual(point_request_guard(state, h.now+.004), '')
        request = go(h)
        leg = h.c.area.navigation.active
        self.assertEqual((leg['start_cell'], leg['start_heading_deg']), ([163, 80], -90))
        self.assertEqual(leg['goal_cell'], [88, 147])
        self.assertEqual(leg['motion_kind'], 'numbered_point')
        self.assertEqual(leg['target_point'], 4)
        self.assertIsNone(h.c.numbered.active)
        self.assertFalse(h.c.area.search.active)
        self.assertFalse(h.c.reference['confirmed'])
        self.assertEqual(h.c.area.navigation.state(h.now+.004)['start_point'], None)
        h.drive_leg()
        self.assertEqual(h.c.numbered.state(h.now+.003)['command_id'], request['command_id'])

    def test_non_numbered_road_origin_is_not_snapped_to_a_numbered_point(self):
        h = Harness(); confirm_map(h, x=327, y=179, heading=-90); go(h)
        self.assertEqual(h.c.area.navigation.active['start_cell'], [163, 70])
        self.assertNotIn([163, 70], [p['planner_cell'] for p in h.c.numbered.points.values()])

    def test_router_accepts_map_heading_for_text_and_verified_voice(self):
        h = Harness(); confirm_map(h)
        router = TaskRouter(extension_enabled=True)
        for source in ('text', 'voice'):
            request = dict(parse_task_text('去4号点'), command_id='route-'+source,
                source=source, issued_at=h.now, ttl_s=3., voice_verification=dict(verified=True))
            out = router.route(request, h.now+.003, extension=h.c.state(h.now+.003))
            self.assertIn('从已确认当前位置前往4号点', out['speech'])
            self.assertNotIn('None', out['speech'])
            self.assertEqual(out['rule_navigation']['start_heading_deg'], -90)
            self.assertEqual(out['extension']['arrival_policy'], 'manual')
            self.assertNotIn('start_point', out['extension'])

    def test_done_and_fresh_measured_stop_pose_are_all_required(self):
        h = Harness(); confirm_map(h); go(h); h.drive_leg(); n = h.c.area.navigation
        h.c.receive(wire.Packet(0, wire.DONE, bytes((99, 3, 0, 0))), h.now+.005)
        self.assertEqual(n.active['phase'], 'running')
        h.feedback(motion=3, pose=(n.active['goal_cell'], 900))
        h.c.area.tick(h.now+.003)
        self.assertFalse(n.state(h.now+.003)['arrived'])
        self.assertFalse(h.c.last_result['completed'])
        h.arrive(); h.c.area.tick(h.now+.003)
        self.assertTrue(h.c.last_result['completed'])
        self.assertEqual(h.c.last_result['command_id'], 'goto-point')
        self.assertEqual(h.c.effects[-1][1]['mode'], 'inspect')
        self.assertFalse(any(k == 'auto' and v.get('action') == 'start' for k, v in h.c.effects))
        self.assertFalse(h.c.numbered.reference['confirmed'])
        self.assertEqual(h.c.state(h.now+.003)['point_start_reference']['source'], 'fpga_encoder_imu_pose')

    def test_next_numbered_goal_starts_at_current_measured_pose_not_prior_goal(self):
        h = Harness(); confirm_map(h); go(h); h.drive_leg(); n = h.c.area.navigation
        h.c.receive(wire.Packet(0, wire.DONE, bytes((n.active['task_id'], 3, 0, 0))), h.now+.004)
        actual = [90, 147]
        h.feedback(motion=3, pose=(actual, 370)); h.c.area.tick(h.now+.003)
        self.assertTrue(n.state(h.now+.003)['arrived'])
        go(h, point=1, cid='second')
        self.assertEqual((n.active['start_cell'], n.active['start_heading_deg']), (actual, 37.))
        self.assertEqual(n.active['goal_cell'], [17, 17])

    def test_restart_missing_heading_and_stale_pose_do_not_invent_a_start(self):
        h = Harness()
        h.c.area.navigation.map['start_reference'] = {}
        h.c.reference = dict(confirmed=True, x_cm=327, y_cm=159)
        self.assertFalse(h.c.state(h.now+.002)['can_navigate_point'])
        self.assertFalse(h.c.state(h.now+.002)['point_start_reference']['valid'])
        before = len(h.c.outbox)
        with self.assertRaisesRegex(ValueError, 'confirm_physical'):
            go(h)
        self.assertEqual(len(h.c.outbox), before)
        h = Harness(); confirm_map(h); go(h); h.drive_leg(); h.arrive()
        h.c.area.navigation.map['pose']['stamp'] = h.now - 2
        self.assertFalse(h.c.state(h.now+.002)['can_navigate_point'])

    def test_real_map_identity_and_pump_off_remain_required(self):
        for change, reason in (
                (lambda h: h.c.area.navigation.map.update(map_signature=0), 'map_identity'),
                (lambda h: h.c.actuator.update(pump_on=True), 'pump_off')):
            h = Harness(); confirm_map(h); change(h); before = len(h.c.outbox)
            with self.assertRaisesRegex(ValueError, reason):
                go(h)
            self.assertEqual(len(h.c.outbox), before)

    def test_new_host_session_cannot_reuse_previous_sessions_fresh_pose(self):
        for session, next_task in ((False, None), (True, 99)):
            h = Harness(); confirm_map(h); go(h); h.drive_leg(); h.arrive()
            h.c.area.tick(h.now+.003)
            # A newer HEARTBEAT can arrive before the next pose frame. Both
            # the cached pose and measured stop are fresh, but no longer form
            # one correlated reference for this navigation leg.
            h.feedback(session=session, task=next_task, motion=3)
            self.assertIsNone(h.c.area.navigation.current_reference(h.now+.003))
            self.assertFalse(h.c.state(h.now+.003)['point_start_reference']['valid'])
            before = len(h.c.outbox)
            with self.assertRaisesRegex(ValueError, 'confirm_physical'):
                go(h, point=1, cid='other-session')
            self.assertEqual(len(h.c.outbox), before)

    def test_cancel_is_not_arrival_and_waits_for_stop_feedback(self):
        h = Harness(); confirm_map(h); go(h); h.drive_leg(); n = h.c.area.navigation
        h.c.stop(h.now+.003, 'operator_stop'); h.ack()
        h.feedback(motion=2, stopped=False); h.c.area.tick(h.now+.003)
        self.assertEqual(n.active['phase'], 'stopping')
        h.feedback(motion=2, pose=([160, 100], -900)); h.c.area.tick(h.now+.003)
        self.assertEqual(n.active['phase'], 'cancelled')
        self.assertFalse(h.c.last_result['completed'])
        self.assertFalse(n.state(h.now+.003)['arrived'])

    def test_explicit_numbered_confirmation_replaces_old_map_reference(self):
        h = Harness(); confirm_map(h)
        h.c.request(dict(action='confirm_point_start', command_id='numbered-confirm',
            source='web', confirmed=True, point_id=8, start_heading_deg=180,
            issued_at=h.now, ttl_s=3.), h.now+.003)
        self.assertFalse(h.c.reference['confirmed'])
        self.assertEqual(h.c.state(h.now+.004)['point_start_reference']['kind'], 'numbered')
        confirm_map(h, x=25, y=139, heading=90)
        self.assertFalse(h.c.numbered.reference['confirmed'])


if __name__ == '__main__':
    unittest.main()
