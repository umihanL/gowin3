"""Local task handoffs with real protocol parsing and synthetic feedback."""
import unittest
from test_area_navigation import Harness
from test_numbered_map_reference import confirm_map
import test_numbered_navigation as numbered_fixture
from rescue_vision import ov5640_protocol as wire


class RegionTaskHandoffTests(unittest.TestCase):
    def test_long_leg_done_waits_for_new_stop_pose_without_cancelling_region(self):
        h = Harness(); h.start(); h.drive_leg()
        n = h.c.area.navigation
        for _ in range(50):
            h.feedback(run=True, stopped=False, motion=2, pose=(n.active['start_cell'], 1800))
            h.c.area.tick(h.now+.003)
        h.c.receive(wire.Packet(0, wire.DONE, bytes((n.active['task_id'], 3, 0, 0))), h.now+.004)
        h.c.area.tick(h.now+.005)
        self.assertEqual(n.active['phase'], 'confirming_arrival')
        self.assertTrue(h.c.area.search.active)
        self.assertEqual(h.c.area.search.phase, 'NAVIGATING')
        for _ in range(3):
            h.feedback(run=True, stopped=False, motion=2)
            h.c.area.tick(h.now+.003)
            self.assertTrue(h.c.area.search.active)
        h.feedback(motion=3, pose=(n.active['goal_cell'], 900))
        h.c.area.tick(h.now+.003)
        self.assertEqual(h.c.area.search.phase, 'OBSERVING')

    def test_external_motion_consumes_hmi_and_old_panel_origin(self):
        for trigger in ('host_run', 'measured_wheel_move'):
            h = Harness(); confirm_map(h)
            self.assertTrue(h.c.reference['confirmed'])
            if trigger == 'host_run':
                h.query(wire.HEARTBEAT, bytes((0, 2, 31, 88)))
            else:
                h.query(wire.EXT_STATUS, bytes((0, 8, 0, 0)))
            self.assertFalse(h.c.reference['confirmed'])
            h.feedback(session=False)
            self.assertFalse(h.c.area.reference(h.now+.003)['valid'])
            self.assertFalse(h.c.state(h.now+.003)['can_navigate_point'])

    def test_numbered_arrival_provides_actual_region_origin_not_target_coordinate(self):
        h = Harness(); c = h.c
        c.task = 'dialogue'
        c.request(numbered_fixture.request('confirm_point_start', h.now,
            point_id=2, confirmed=True, start_heading_deg=-90), h.now+.002)
        c.request(numbered_fixture.request('navigate_point', h.now,
            target_point=3, start_point=2), h.now+.003)
        a = c.numbered.active
        numbered_fixture.ack(c, h.now+.01)
        heading = bytes((0,)) + wire.heading_payload(-90)[:2] + bytes((0,))
        numbered_fixture.ack(c, h.now+.02, heading)
        numbered_fixture.ack(c, h.now+.03, bytes((0, 3, 1, a['task_id'])))
        numbered_fixture.query(c, wire.POINT_STATUS, bytes((0, 3, 2, 0)), h.now+.04)
        numbered_fixture.query(c, wire.HEARTBEAT, bytes((0, 0, 19, a['task_id'])), h.now+.05)
        numbered_fixture.ack(c, h.now+.06)
        numbered_fixture.telemetry(c, h.now+.07)
        numbered_fixture.done(c, h.now+.10)
        numbered_fixture.telemetry(c, h.now+.13, motion=3, run=False, stopped=True)
        self.assertEqual(a['phase'], 'arrived')
        h.now += .15
        # Actual capture differs from destination [17,142]; no snapping here.
        pose = dict(valid=True, stamp=h.now, mode=9, status=3, task_id=a['task_id'],
            x_mm=390, y_mm=310, heading_tenths=370)
        c.area.navigation.map['pose'] = pose
        reference = c.area.reference(h.now+.003)
        self.assertTrue(reference['valid'])
        self.assertEqual((reference['cell'], reference['heading_deg']), ([19, 144], 37.))
        self.assertEqual(reference['source'], 'fpga_encoder_imu_pose')
        pose['task_id'] = 99
        self.assertFalse(c.area.reference(h.now+.004)['valid'])

    def test_region_cancel_to_traffic_rejects_late_done_and_cannot_restart_in_parallel(self):
        h = Harness(); h.start(); h.drive_leg(); c = h.c
        a = c.area.navigation.active
        original_task = a['task_id']
        h.now += .01
        request = dict(action='traffic_task', command_id='switch-traffic', source='web',
                       issued_at=h.now, ttl_s=3.)
        c.request(request, h.now)
        self.assertEqual(c.task, 'traffic')
        self.assertFalse(c.area.search.active)
        self.assertEqual(a['phase'], 'stopping')
        c.receive(wire.Packet(0, wire.DONE, bytes((original_task, 3, 0, 0))), h.now+.001)
        self.assertEqual(a['phase'], 'stopping')
        with self.assertRaisesRegex(ValueError, 'duplicate'):
            c.request(request, h.now+.002)
        with self.assertRaisesRegex(ValueError, 'in_progress'):
            c.request(dict(action='navigate_point', target_point=4, command_id='parallel-go',
                source='web', issued_at=h.now, ttl_s=3.), h.now+.003)
        h.ack(); h.feedback(motion=2, pose=(a['start_cell'], 1800))
        c.area.tick(h.now+.003)
        self.assertEqual(a['phase'], 'cancelled')
        self.assertFalse(c.area.navigation.state(h.now+.004)['arrived'])
        self.assertFalse(c.last_result.get('completed'))
        self.assertFalse(any(k == 'auto' and v.get('action') == 'start' for k, v in c.effects))


if __name__ == '__main__':
    unittest.main()
