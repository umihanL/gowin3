import pathlib
import struct
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.fpga_navigation import FpgaNavigator
from rescue_vision import protocol as p


class FpgaNavigationTests(unittest.TestCase):
    def setUp(self):
        self.core = FpgaNavigator({'buildings': [{'building_id':'C', 'number':'03', 'name':'春晖楼'}]}, session_id=42)

    def request(self, now=10., **changes):
        r = dict(command_id='go3', building_id='C', mode='execute', confirmed=True,
                 arrival_policy='auto', issued_at=now, ttl_s=5.)
        r.update(changes)
        return self.core.request(r, now)

    def status(self, now=10.1, seq=1, **changes):
        s = dict(session_id=42, goal_id=self.core.active['goal_id'], building_number=3,
                 status_sequence=seq, state='accepted', vehicle_stopped=True, estop=False, stamp=now)
        s.update(changes)
        return self.core.observe(s, now)

    def test_destination_handoff_requires_no_map_or_line_or_local_pose(self):
        state = self.request()
        self.assertIsNone(state['route'])
        self.assertFalse(state['route_computed_on_x5'])
        self.assertEqual(state['building_id'], 'C')
        self.assertEqual(self.core.next_command(10.)['building_number'], '03')
        self.assertFalse(self.core.proposal(10.)['ready_to_move'])
        self.assertTrue(self.status())
        self.assertTrue(self.core.proposal(10.1)['ready_to_move'])
        self.assertFalse(self.core.proposal(10.1)['line_tracking_required'])

    def test_preview_never_sends_and_unknown_destination_rejected(self):
        self.request(mode='preview')
        self.assertIsNone(self.core.next_command(10.))
        with self.assertRaises(ValueError):
            self.request(command_id='other', building_id='999')

    def test_arrival_identity_and_stopped_are_separate(self):
        self.request()
        self.assertFalse(self.status(state='arrived', session_id=41))
        self.assertFalse(self.status(state='arrived', goal_id=100))
        self.assertFalse(self.status(state='arrived', building_number=2))
        self.assertTrue(self.status(state='arrived', vehicle_stopped=False))
        proposal = self.core.proposal(10.1)
        self.assertTrue(proposal['arrived'])
        self.assertFalse(proposal['vehicle_stopped'])
        self.assertEqual(proposal['arrival_policy'], 'auto')
        self.assertFalse(self.core.proposal(11.)['arrived'])

    def test_duplicate_status_does_not_extend_freshness(self):
        self.request()
        self.assertTrue(self.status())
        self.assertFalse(self.status(now=10.6, seq=1))
        self.assertFalse(self.core.proposal(10.8)['ready_to_move'])

    def test_cancel_and_new_goal_ignore_old_arrival(self):
        self.request()
        first = self.core.active['goal_id']
        self.request(now=10.2, command_id='cancel', action='cancel')
        self.assertFalse(self.status(now=10.3, state='arrived'))
        self.assertFalse(self.core.proposal(10.3)['arrived'])
        self.request(now=10.4, command_id='again')
        self.assertNotEqual(self.core.active['goal_id'], first)
        self.assertFalse(self.status(now=10.5, state='arrived', goal_id=first))

    def test_pause_blocks_immediately_and_resume_requires_ack(self):
        self.request()
        self.status()
        self.request(now=10.2, command_id='pause', action='pause')
        self.assertFalse(self.core.proposal(10.2)['ready_to_move'])
        with self.assertRaises(ValueError):
            self.request(now=10.3, command_id='resume1', action='resume')
        self.assertTrue(self.status(now=10.4, seq=2, state='paused'))
        self.request(now=10.5, command_id='resume2', action='resume')
        self.assertFalse(self.core.proposal(10.5)['ready_to_move'])
        self.assertTrue(self.status(now=10.6, seq=3, state='navigating', vehicle_stopped=False))
        self.assertTrue(self.core.proposal(10.6)['ready_to_move'])

    def test_retries_reuse_goal_and_timeout_never_late_resumes(self):
        self.request()
        a = self.core.next_command(10.)
        self.assertIsNone(self.core.next_command(10.1))
        b = self.core.next_command(10.5)
        self.assertEqual((a['session_id'], a['goal_id']), (b['session_id'], b['goal_id']))
        self.assertIsNone(self.core.next_command(13.1))
        self.assertFalse(self.status(now=13.2))
        self.assertFalse(self.core.proposal(13.2)['ready_to_move'])

    def test_estop_and_reordered_status_revoke_or_cannot_restore(self):
        self.request()
        self.status(seq=100)
        self.assertTrue(self.status(now=10.2, seq=101, estop=True))
        self.assertFalse(self.core.proposal(10.2)['ready_to_move'])
        self.assertFalse(self.status(now=10.3, seq=100, estop=False))
        self.assertFalse(self.core.proposal(10.3)['ready_to_move'])

    def test_completed_goal_can_be_cancelled_at_task_finish(self):
        self.request()
        self.assertTrue(self.status(state='arrived'))
        self.request(now=10.2, command_id='finish', action='cancel')
        self.assertTrue(self.status(now=10.3, seq=2, state='cancelled'))
        self.assertIsNone(self.core.pending)
        self.assertFalse(self.core.proposal(10.3)['arrived'])

    def test_wire_roundtrip_and_all_fields(self):
        payload = p.navigation_goal_payload(42, 7, 3, 'auto')
        self.assertEqual(struct.unpack('<IIHBH',payload), (42,7,3,1,1000))
        self.assertEqual(p.Parser().feed(p.encode(p.NAV_GOAL,1,payload))[0].payload, payload)
        control = p.navigation_control_payload(42,7,'cancel')
        self.assertEqual(struct.unpack('<IIBH',control), (42,7,1,1000))
        status = struct.pack(p.NAV_STATUS_FORMAT,42,7,901,4,1,3,0)
        packet = p.Parser().feed(p.encode(p.NAV_STATUS,9,status))[0]
        out = p.navigation_status(packet,12.)
        self.assertEqual(out['state'],'arrived')
        self.assertEqual(out['status_sequence'],901)
        self.assertTrue(out['vehicle_stopped'])
        with self.assertRaises(ValueError):p.navigation_goal_payload(42,7,3,'invalid')
        with self.assertRaises(ValueError):p.navigation_control_payload(42,0,'resume')


if __name__ == '__main__':
    unittest.main(verbosity=2)
