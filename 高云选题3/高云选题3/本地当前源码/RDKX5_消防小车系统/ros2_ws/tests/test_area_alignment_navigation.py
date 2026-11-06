"""Bounded alignment uses real A* protocol phases with synthetic feedback only."""
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision import ov5640_protocol as wire
from rescue_vision.area_navigation import AreaNavigation, RESET_NAV, SET_SPEED


class Harness:
    def __init__(self):
        self.now, self.sequence, self.sent = 100., 0, []
        n = SimpleNamespace(host={}, point={}, invalidate_reference=lambda reason: None)
        self.owner = SimpleNamespace(numbered=n, real_uart=True, protocol_ready=True,
            reference={}, effects=[], _motion_sequence=lambda: 7, send=self.send)
        self.nav = AreaNavigation(self.owner)
        self.nav.observe_map(dict(region_supported=True, route=dict(generation=1)))
        self.refresh()

    def send(self, command, data=bytes(4), now=None):
        self.sequence += 1
        self.sent.append((self.sequence, command, data))
        return self.sequence

    def refresh(self, *, session=False, run=False, motion=0, cell=None, heading=0, advance=.05):
        self.now += advance
        c, n = self.owner, self.owner.numbered
        n.host_at = n.point_at = n.host_request_at = c.status_at = c.status_request_at = self.now
        n.point_request_at = self.now
        n.host = dict(selected=True, scope=True, link_ok=True, nav_session=session,
                      run=run, motion_status=motion, task_id=7)
        n.point = dict(selected_point=0, state=2)
        c.status = dict(vehicle_stopped=not run)
        c.actuator = dict(stamp=self.now, pump_on=False, pump_feedback_fresh=True,
                         guard=dict(armed=False))
        if cell is not None:
            self.nav.map['pose'] = dict(valid=True, stamp=self.now, mode=9, status=motion,
                task_id=7, x_mm=cell[0]*20+10, y_mm=3200-cell[1]*20-10,
                heading_tenths=round(heading*10))

    def request(self, **changes):
        request = dict(command_id='alignment-test', goal_cell=[60, 50], execute_fire=False,
            start_reference=dict(valid=True, cell=[50, 50], heading_deg=0., source='fpga_encoder_imu_pose'),
            leg_kind='alignment', motion_kind='semantic_alignment',
            alignment_guard=dict(start_cell=[50, 50], heading_deg=0., max_advance_cm=20.,
                lateral_tolerance_cm=6., heading_tolerance_deg=15., speed_mm_s=60))
        request.update(changes)
        return request

    def start(self, **changes):
        self.nav.start(self.request(**changes), self.now)

    def ack(self, result=0, speed=None):
        self.now += .01
        seq, command, data = self.sent[-1]
        reply = bytes((result, 0, 0, 0))
        if command == RESET_NAV:
            reply = bytes((result, 0))+struct.pack('<H', 200 if speed is None else speed)
        elif command == SET_SPEED:
            reply = bytes((result, 0))+(data[:2] if speed is None else struct.pack('<H', speed))
        elif command == wire.SET_START_HEADING:
            reply = bytes((result,))+data[:2]+bytes((0,))
        self.nav.on_reply(wire.Packet(seq, command|128, reply), self.now-.001, self.now)

    def prepared(self):
        self.start(); self.ack(); self.refresh(); self.nav.tick(self.now)
        assert self.sent[-1][1] == SET_SPEED
        self.ack()
        for _ in range(4):
            self.ack()
        assert self.nav.active['phase'] == 'planning'
        self.refresh(); self.nav.tick(self.now)
        assert self.nav.active['phase'] == 'alignment_route_check'

    def route(self, points=None, **changes):
        points = points or [[1010, 1010], [1110, 1010], [1210, 1010]]
        self.nav.map['route'] = dict(valid=True, sampled=False, generation=2, total_count=len(points),
            points_mm=points, sample_indices=list(range(len(points))), stamp=self.now, **changes)

    def running(self):
        self.prepared(); self.route(); self.nav.tick(self.now)
        assert self.nav.active['phase'] == 'starting'
        self.ack(); self.refresh(session=True, run=True, motion=2, cell=[50, 50]); self.nav.tick(self.now)
        assert self.nav.active['phase'] == 'running'


class AlignmentNavigationTests(unittest.TestCase):
    def test_request_validation_does_not_send_uart(self):
        for changes in (dict(start_cell=[51, 50]), dict(max_advance_cm=31),
                        dict(lateral_tolerance_cm=7), dict(heading_tolerance_deg=16),
                        dict(heading_deg=90), dict(speed_mm_s=100), dict(max_advance_cm=float('nan'))):
            h = Harness(); request = h.request(); request['alignment_guard'].update(changes)
            with self.assertRaises(ValueError):
                h.nav.start(request, h.now)
            self.assertEqual(h.sent, [])

    def test_alignment_goal_must_be_forward_in_corridor(self):
        for goal in ([49,50], [60,54], [62,50]):
            h = Harness()
            with self.assertRaisesRegex(ValueError, 'alignment_goal'):
                h.start(goal_cell=goal)
            self.assertEqual(h.sent, [])

    def test_original_speed_saved_and_actual_complete_route_required(self):
        h = Harness(); h.prepared()
        self.assertEqual(h.nav.restore_speed_mm_s, 200)
        self.assertEqual([x[1] for x in h.sent], [RESET_NAV, SET_SPEED, wire.SET_START,
            wire.SET_START_HEADING, wire.SET_GOAL, wire.PLAN])
        h.nav.tick(h.now)
        self.assertNotIn(wire.START, [x[1] for x in h.sent])
        h.route(); h.nav.tick(h.now)
        self.assertEqual(h.sent[-1][1], wire.START)
        self.assertEqual(h.nav.active['alignment_route_length_mm'], 200.)

    def test_old_sampled_incomplete_stale_route_never_starts(self):
        for changes in (dict(generation=1), dict(sampled=True), dict(total_count=4),
                        dict(sample_indices=[0,2,3]), dict(stamp=99.)):
            h = Harness(); h.prepared(); h.route(); h.nav.map['route'].update(changes)
            h.nav.tick(h.now)
            self.assertEqual(h.nav.active['phase'], 'alignment_route_check')
            h.refresh(advance=5.01); h.nav.tick(h.now)
            self.assertEqual(h.sent[-1][1], wire.STOP)
            self.assertEqual(h.nav.active['reason'], 'alignment_complete_new_route_timeout')

    def test_bad_full_routes_explicitly_stop(self):
        variants = (
            ([[1010,1010],[1110,1090],[1210,1010]], 'lateral'),
            ([[1010,1010],[1150,1010],[1110,1010],[1210,1010]], 'reverses'),
            ([[1010,1010],[1110,1050],[1210,1010]], 'heading'),
            ([[1010,1010],[1250,1010]], 'forward'),
        )
        for points, reason in variants:
            h = Harness(); h.prepared(); h.route(points); h.nav.tick(h.now)
            self.assertEqual(h.sent[-1][1], wire.STOP)
            self.assertIn(reason, h.nav.active['reason'])

    def test_running_pose_and_heading_exit_guard_stop(self):
        for cell, heading, reason in (([50,54],0,'lateral'), ([63,50],0,'forward'),
                                     ([55,50],20,'heading')):
            h = Harness(); h.running()
            h.refresh(session=True, run=True, motion=2, cell=cell, heading=heading)
            h.nav.tick(h.now)
            self.assertEqual(h.sent[-1][1], wire.STOP)
            self.assertIn(reason, h.nav.active['reason'])

    def test_running_reverse_and_stale_pose_stop(self):
        h = Harness(); h.running()
        h.refresh(session=True, run=True, motion=2, cell=[55,50]); h.nav.tick(h.now)
        h.refresh(session=True, run=True, motion=2, cell=[53,50]); h.nav.tick(h.now)
        self.assertEqual(h.nav.active['reason'], 'alignment_actual_pose_reverses')
        h = Harness(); h.running()
        h.refresh(session=True, run=True, motion=2, advance=1.3); h.nav.tick(h.now)
        self.assertEqual(h.nav.active['reason'], 'alignment_fresh_actual_pose_required')

    def test_cancel_restores_original_speed_only_after_measured_stop(self):
        h = Harness(); h.running(); h.nav.stop(h.now, 'operator_stop'); h.ack()
        h.refresh(session=True, run=True, motion=2); h.nav.tick(h.now)
        self.assertEqual(h.nav.active['phase'], 'stopping')
        h.refresh(session=True, motion=2); h.nav.tick(h.now)
        self.assertEqual(h.nav.active['phase'], 'restoring_speed')
        self.assertEqual(h.sent[-1][2], struct.pack('<HH', 200, 0))
        h.ack()
        self.assertEqual(h.nav.active['phase'], 'cancelled')
        self.assertIsNone(h.nav.restore_speed_mm_s)

    def test_pause_restores_before_next_target_can_replan(self):
        h = Harness(); h.running(); h.nav.pause('alignment-test',h.now); h.ack()
        h.refresh(session=True, motion=2, cell=[54,50]); h.nav.tick(h.now)
        self.assertEqual(h.nav.active['phase'], 'restoring_speed')
        self.assertTrue(h.nav.busy())
        h.ack(); self.assertEqual(h.nav.active['phase'], 'paused')
        self.assertEqual(h.nav.current_reference(h.now)['cell'], [54,50])
        self.assertIsNone(h.nav.restore_speed_mm_s)

    def test_restore_nack_mismatch_timeout_never_claim_success(self):
        for failure in ('nack', 'mismatch', 'timeout'):
            h = Harness(); h.running(); h.nav.stop(h.now,'operator_stop'); h.ack()
            h.refresh(session=True, motion=2); h.nav.tick(h.now)
            if failure == 'timeout':
                h.refresh(session=True, motion=2, advance=2.01); h.nav.tick(h.now)
            else:
                h.ack(result=2 if failure == 'nack' else 0, speed=100)
            self.assertEqual(h.nav.active['phase'], 'failed')
            self.assertIn('speed_restore', h.nav.active['reason'])
            self.assertEqual(h.nav.restore_speed_mm_s, 200)

    def test_done_and_measured_arrival_restores_before_arrived(self):
        h = Harness(); h.running()
        h.nav.on_event(wire.Packet(0, wire.DONE, bytes((7,3,0,0))),h.now+.001)
        h.refresh(session=True, motion=3, cell=[60,50]); h.nav.tick(h.now)
        self.assertEqual(h.nav.active['phase'], 'restoring_speed')
        self.assertFalse(h.nav.state(h.now)['arrived'])
        h.ack(); self.assertEqual(h.nav.active['phase'], 'arrived')
        self.assertTrue(h.nav.state(h.now)['arrived'])

    def test_arrival_pose_outside_guard_does_not_become_arrived(self):
        h = Harness(); h.running()
        h.nav.on_event(wire.Packet(0, wire.DONE, bytes((7,3,0,0))),h.now+.001)
        h.refresh(session=True, motion=3, cell=[63,50]); h.nav.tick(h.now)
        self.assertEqual(h.nav.active['reason'], 'alignment_forward_distance_exceeded')
        self.assertEqual(h.sent[-1][1], wire.STOP)

    def test_missing_generation_and_false_alignment_kind_do_not_send(self):
        h = Harness(); h.nav.map['route'] = {}
        with self.assertRaisesRegex(ValueError, 'generation_baseline'):
            h.start()
        self.assertEqual(h.sent, [])
        h = Harness()
        with self.assertRaisesRegex(ValueError, 'must_match'):
            h.start(motion_kind='semantic_region')
        self.assertEqual(h.sent, [])

    def test_failed_restore_is_retried_before_subsequent_normal_navigation(self):
        h = Harness(); h.running(); h.nav.stop(h.now,'operator_stop'); h.ack()
        h.refresh(session=True, motion=2); h.nav.tick(h.now); h.ack(result=2)
        h.refresh(session=True, motion=2)
        h.start(leg_kind='search', motion_kind='semantic_region', alignment_guard=None)
        h.ack(); h.refresh(); h.nav.tick(h.now)
        self.assertEqual(h.sent[-1][1], SET_SPEED)
        self.assertEqual(h.sent[-1][2], struct.pack('<HH', 200, 0))
        h.ack()
        self.assertEqual(h.sent[-1][1], wire.SET_START)

    def test_plain_leg_has_unchanged_protocol_steps(self):
        h = Harness(); h.start(leg_kind='search', motion_kind='semantic_region', alignment_guard=None)
        h.ack(); h.refresh(); h.nav.tick(h.now)
        self.assertEqual(h.sent[-1][1], wire.SET_START)
        for _ in range(4): h.ack()
        h.refresh(); h.nav.tick(h.now)
        self.assertEqual(h.sent[-1][1], wire.START)
        self.assertNotIn(SET_SPEED, [x[1] for x in h.sent])


if __name__ == '__main__':
    unittest.main()
