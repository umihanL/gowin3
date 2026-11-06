"""Real fixed-10 protocol progression, correlation, cancellation and stale data."""
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision import ov5640_protocol as wire
from rescue_vision.extension_control import ExtensionControl, dialogue_allowed


def request(action, now=100., **kwargs):
    return dict(action=action, command_id=kwargs.pop('command_id', action),
                source='web', issued_at=now, ttl_s=3., **kwargs)


def ready():
    core = ExtensionControl(real_uart=True)
    core.protocol_ready = True
    core.numbered.heading_supported = True
    core.capabilities = dict(distance=True, velocity=True, route_turn=True)
    core.status = dict(vehicle_stopped=True, phase=0, mode_selected=True, zebra_wait=False)
    core.status_at = core.status_request_at = 100.
    core.numbered.load_catalog(json.loads((ROOT/'config/numbered_points.json').read_text(encoding='utf-8-sig')))
    query(core, wire.HEARTBEAT, bytes((0, 0, 19, 0)), 100.)
    query(core, wire.POINT_STATUS, bytes(4), 100.)
    return core


def query(core, command, data, now):
    seq = core.send(command, now=now)
    core.receive(wire.Packet(seq, command | 0x80, data), now+.001)


def ack(core, now, data=bytes(4)):
    seq = core.numbered.active['request_sequence']
    command = core.pending[seq][0]
    core.receive(wire.Packet(seq, command | 0x80, data), now)


def telemetry(core, now, *, task=None, point=None, motion=2, run=True, stopped=False, point_state=4):
    active = core.numbered.active or {}
    task = active.get('task_id', 33) if task is None else task
    point = active.get('target_point', 3) if point is None else point
    flags = 19 | (4 if run else 0) | (8 if point_state == 4 else 0)
    query(core, wire.POINT_STATUS, bytes((0, point, point_state, 0)), now)
    query(core, wire.HEARTBEAT, bytes((0, motion, flags, task)), now+.003)
    query(core, wire.EXT_STATUS, bytes((11 if motion == 3 else 10, 8 | (2 if stopped else 0), 0, 0)), now+.006)
    core.tick(now+.01)


def begin(core=None, profile='vision'):
    core = core or ready()
    core.task = profile
    core.request(request('confirm_point_start', point_id=2, confirmed=True, start_heading_deg=90), 100.01)
    core.request(request('navigate_point', 100.02, target_point=3, start_point=2), 100.02)
    return core


def running(core=None, profile='vision'):
    core = begin(core, profile)
    ack(core, 100.03)
    ack(core, 100.035, bytes((0, 0x84, 0x03, 0)))
    a = core.numbered.active
    ack(core, 100.04, bytes((0, 3, 1, a['task_id'])))
    query(core, wire.POINT_STATUS, bytes((0, 3, 2, 0)), 100.05)
    query(core, wire.HEARTBEAT, bytes((0, 0, 19, a['task_id'])), 100.06)
    ack(core, 100.07)
    telemetry(core, 100.08)
    return core


def done(core, now=100.2, task=None):
    a = core.numbered.active
    core.receive(wire.Packet(0, wire.DONE, bytes((a['task_id'] if task is None else task, 3, 0, 0))), now)


class NumberedWireTests(unittest.TestCase):
    def test_point_vector_matches_fpga_table_and_wire(self):
        self.assertEqual(wire.encode(7, wire.SELECT_POINT, wire.point_payload(9, 42)).hex(),
                         'a55a010708092a00002d')
        core = ready()
        self.assertEqual(core.numbered.points[9]['planner_cell'], [88, 73])
        self.assertEqual(core.numbered.points[1]['display_cm'], [35, 285])

    def test_wire_rejects_bool_bad_points_and_reserved_bits(self):
        for point in (True, 0, 10, 3.0):
            with self.assertRaises(ValueError):
                wire.point_payload(point, 1)
        for data in (bytes((0, 10, 2, 0)), bytes((0, 1, 5, 0))):
            with self.assertRaises(ValueError):
                wire.point_status(data)
        with self.assertRaises(ValueError):
            wire.host_status(bytes((0, 0, 128, 0)))

    def test_legacy_unnumbered_navigation_query_remains_parseable(self):
        self.assertEqual(wire.point_status(bytes((0, 0, 2, 0)))['state'], 2)
        self.assertEqual(wire.point_status(bytes((0, 0, 4, 0)))['selected_point'], 0)
        self.assertEqual(wire.point_status(bytes((0, 0, 1, 0)))['state'], 1)


class NumberedNavigationTests(unittest.TestCase):
    def test_legacy_firmware_cannot_silently_choose_reverse_heading(self):
        core = ready()
        core.numbered.heading_supported = False
        with self.assertRaisesRegex(ValueError, 'heading_firmware_required'):
            core.request(request('confirm_point_start', point_id=8, confirmed=True, start_heading_deg=180), 100.01)

    def test_operator_heading_is_not_inferred_from_destination(self):
        core = ready()
        with self.assertRaisesRegex(ValueError, 'start_heading_required'):
            core.request(request('navigate_point', text='我现在在8号点，请前往4号点',
                start_point=8, target_point=4, start_declared=True), 100.01)
        self.assertIsNone(core.numbered.active)
        core = ready()
        core.request(request('navigate_point', text='我现在在8号点，车头朝左，请前往4号点',
            start_point=8, target_point=4, start_declared=True, start_heading_deg=180), 100.01)
        ack(core,100.03)
        self.assertEqual(core.outbox[-1][4:9], bytes((wire.SET_START_HEADING,0x08,0x07,0,0)))
        ack(core,100.04,bytes((0,0x08,0x07,0)))
        self.assertEqual(core.outbox[-1][4],wire.SELECT_POINT)
        self.assertEqual(core.numbered.active['start_heading_deg'],180)

    def test_heading_mismatch_or_missing_ack_never_starts(self):
        core=begin()
        ack(core,100.03)
        ack(core,100.04,bytes(4))
        self.assertEqual(core.numbered.active['phase'],'stopping')
        self.assertEqual(core.numbered.active['reason'],'fpga_heading_ack_mismatch')
        self.assertFalse(any(f[4]==wire.START for f in core.outbox))

    def test_arrival_needs_fresh_same_session_heading_for_next_leg(self):
        core=running(); done(core); telemetry(core,100.25,motion=3,run=False,stopped=True)
        for pose in ({},dict(valid=True,stamp=90.,mode=9,status=3,task_id=core.numbered.active['task_id'],heading_tenths=1800),
                     dict(valid=True,stamp=100.28,mode=9,status=3,task_id=255,heading_tenths=1800)):
            core.numbered.map_pose=pose
            self.assertIsNone(core.numbered.reference_heading(3,100.29))

    def test_unconfirmed_and_voice_start_cannot_establish_location(self):
        core = ready()
        with self.assertRaisesRegex(ValueError, 'confirm_physical'):
            core.request(request('navigate_point', target_point=3, start_point=2), 100.01)
        r = request('confirm_point_start', point_id=2, confirmed=True, start_heading_deg=90)
        r['source'] = 'voice'
        with self.assertRaisesRegex(ValueError, 'operator_confirmation'):
            core.request(r, 100.01)
        self.assertFalse(core.numbered.reference['confirmed'])

    def test_start_point_must_match_confirmation(self):
        core = ready()
        core.request(request('confirm_point_start', point_id=2, confirmed=True, start_heading_deg=90), 100.01)
        with self.assertRaisesRegex(ValueError, 'does_not_match'):
            core.request(request('navigate_point', target_point=3, start_point=8), 100.02)
        self.assertIsNone(core.numbered.active)

    def test_planning_and_start_are_separate_explicit_packets(self):
        core = begin()
        self.assertEqual(core.outbox[-1][4:9], bytes((wire.SET_START, 12, 80, 0, 0)))
        self.assertFalse(any(f[4] == wire.START for f in core.outbox))
        ack(core, 100.03)
        self.assertEqual(core.numbered.active['phase'], 'setting_heading')
        self.assertEqual(core.outbox[-1][4:9], bytes((wire.SET_START_HEADING,0x84,3,0,0)))
        ack(core, 100.035, bytes((0,0x84,3,0)))
        a = core.numbered.active
        self.assertEqual(core.outbox[-1][4:9], bytes((wire.SELECT_POINT, 3, a['task_id'], 0, 0)))
        ack(core, 100.04, bytes((0, 3, 1, a['task_id'])))
        self.assertEqual(a['phase'], 'planning')
        query(core, wire.POINT_STATUS, bytes((0, 3, 1, 0)), 100.05)
        query(core, wire.HEARTBEAT, bytes((0, 0, 19, a['task_id'])), 100.06)
        self.assertFalse(any(f[4] == wire.START for f in core.outbox))
        query(core, wire.POINT_STATUS, bytes((0, 3, 2, 0)), 100.07)
        self.assertEqual(core.outbox[-1][4:9], bytes((wire.START, 1, a['task_id'], 0, 0)))
        self.assertFalse(core.numbered.state(100.08)['arrived'])

    def test_only_matching_done_plus_subsequent_terminal_stop_is_arrival(self):
        core = running()
        self.assertEqual(core.numbered.active['phase'], 'running')
        self.assertTrue(core.numbered.active['seen_running'])
        done(core)
        self.assertEqual(core.numbered.active['phase'], 'confirming_arrival')
        self.assertFalse(core.numbered.state(100.2)['arrived'])
        telemetry(core, 100.22, motion=3, run=False, stopped=False)
        self.assertFalse(core.numbered.state(100.23)['arrived'])
        telemetry(core, 100.25, motion=3, run=False, stopped=True)
        state = core.numbered.state(100.27)
        self.assertTrue(state['arrived'])
        self.assertTrue(state['valid'])
        self.assertEqual(state['owner'], 'fpga')
        self.assertEqual(core.numbered.reference['point_id'], 3)
        self.assertEqual(core.numbered.reference['source'], 'confirmed_arrival')
        self.assertFalse(core.numbered.state(101.5)['arrived'])

    def test_ack_or_terminal_status_without_done_is_not_arrival(self):
        core = running()
        telemetry(core, 100.2, motion=3, run=False, stopped=True)
        self.assertFalse(core.numbered.state(100.21)['arrived'])
        self.assertEqual(core.numbered.active['reason'], 'terminal_status_waiting_matching_done_event')

    def test_wrong_task_done_and_unsolicited_ack_are_ignored(self):
        core = running()
        task = core.numbered.active['task_id']
        done(core, task=(task+1) % 256)
        core.receive(wire.Packet(250, wire.SELECT_POINT | 128, bytes((0, 3, 1, task))), 100.2)
        self.assertEqual(core.numbered.active['phase'], 'running')
        self.assertIsNone(core.numbered.active['done_at'])

    def test_delayed_pre_done_queries_cannot_supply_post_done_stop_evidence(self):
        core = running()
        task = core.numbered.active['task_id']
        replies = [(wire.POINT_STATUS, bytes((0, 3, 4, 0))),
                   (wire.HEARTBEAT, bytes((0, 3, 27, task))),
                   (wire.EXT_STATUS, bytes((11, 10, 0, 0)))]
        queued = [(core.send(command, now=100.15), command, data) for command, data in replies]
        done(core, 100.2)
        for seq, command, data in queued:
            core.receive(wire.Packet(seq, command | 128, data), 100.22)
        core.tick(100.23)
        self.assertFalse(core.numbered.state(100.23)['arrived'])
        telemetry(core, 100.25, motion=3, run=False, stopped=True)
        self.assertTrue(core.numbered.state(100.27)['arrived'])

    def test_duplicate_done_does_not_reissue_arrival_and_confirmed_arrival_can_be_next_start(self):
        core = running()
        done(core)
        telemetry(core, 100.25, motion=3, run=False, stopped=True)
        arrival = core.numbered.state(100.27)['arrival_stamp']
        done(core, 100.28)
        self.assertEqual(core.numbered.state(100.28)['arrival_stamp'], arrival)
        core.numbered.map_pose = dict(valid=True,stamp=100.28,mode=9,status=3,task_id=core.numbered.active['task_id'],heading_tenths=1750)
        core.request(request('navigate_point', now=100.29, command_id='next-trip', target_point=8), 100.29)
        self.assertEqual(core.numbered.active['start_point'], 3)
        self.assertEqual(core.outbox[-1][4:9], bytes((wire.SET_START, 17, 142, 0, 0)))

    def test_same_task_watchdog_failure_stops_and_never_arrives(self):
        core = running()
        core.receive(wire.Packet(0, wire.LINK_TIMEOUT,
            bytes((core.numbered.active['task_id'], 1, 0, 0))), 100.2)
        self.assertEqual(core.numbered.active['phase'], 'stopping')
        self.assertFalse(core.numbered.state(100.2)['arrived'])
        self.assertEqual(core.outbox[-1][4], wire.STOP)

    def test_done_before_execution_witness_is_ignored(self):
        core = begin()
        done(core, 100.03)
        self.assertEqual(core.numbered.active['phase'], 'setting_start')

    def test_duplicate_request_and_busy_second_navigation_do_not_replace_route(self):
        core = begin()
        with self.assertRaisesRegex(ValueError, 'duplicate'):
            core.request(request('navigate_point', target_point=4), 100.03)
        with self.assertRaisesRegex(ValueError, 'busy'):
            core.request(request('navigate_point', command_id='new', target_point=4), 100.03)
        self.assertEqual(core.numbered.active['target_point'], 3)

    def test_expired_command_cannot_start(self):
        core = ready()
        with self.assertRaisesRegex(ValueError, 'expired'):
            core.request(request('navigate_point', now=90., target_point=3), 100.)

    def test_stop_then_late_done_never_arrives(self):
        core = running()
        core.request(request('stop', now=100.18), 100.18)
        done(core)
        telemetry(core, 100.22, motion=3, run=False, stopped=True)
        self.assertEqual(core.numbered.active['phase'], 'cancelled')
        self.assertFalse(core.numbered.state(100.23)['arrived'])
        self.assertFalse(core.numbered.reference['confirmed'])

    def test_lost_feedback_sends_stop_and_consumes_reference(self):
        core = running()
        core.tick(102.)
        self.assertEqual(core.numbered.active['phase'], 'stopping')
        self.assertTrue(any(f[4] == wire.STOP for f in core.outbox))
        self.assertFalse(core.numbered.reference['confirmed'])

    def test_acknowledged_stop_not_repeated_while_wheels_settle(self):
        core = running()
        core.stop(100.15, 'operator_stop')
        seq = core.numbered.active['stop_sequence']
        core.receive(wire.Packet(seq, wire.STOP | 128, bytes(4)), 100.17)
        core.outbox.clear()
        core.tick(100.7)
        self.assertFalse(any(f[4] == wire.STOP for f in core.outbox))
        self.assertEqual(core.numbered.active['phase'], 'stopping')

    def test_failed_plan_or_changed_selected_point_cannot_start(self):
        for selected, state in ((3, 3), (4, 2)):
            core = begin()
            ack(core, 100.03)
            ack(core, 100.035, bytes((0,0x84,3,0)))
            ack(core, 100.04, bytes((0, 3, 1, core.numbered.active['task_id'])))
            query(core, wire.POINT_STATUS, bytes((0, selected, state, 3 if state == 3 else 0)), 100.05)
            self.assertEqual(core.numbered.active['phase'], 'stopping')
            self.assertFalse(any(f[4] == wire.START for f in core.outbox))

    def test_external_numbered_session_requires_running_query_and_same_done(self):
        core = ready()
        telemetry(core, 100.05, task=77, point=5)
        a = core.numbered.active
        self.assertIsNotNone(a)
        self.assertEqual(a['origin'], 'observed_fpga_host_session')
        self.assertEqual(a['target_point'], 5)
        done(core, 100.2)
        telemetry(core, 100.25, task=77, point=5, motion=3, run=False, stopped=True)
        self.assertTrue(core.numbered.state(100.27)['arrived'])
        old_id = a['command_id']
        telemetry(core, 100.35, task=77, point=5)
        self.assertEqual(core.numbered.active['command_id'], old_id)
        self.assertFalse(core.numbered.state(100.37)['arrived'])

    def test_physical_panel_without_host_session_does_not_manufacture_arrival(self):
        core = ready()
        query(core, wire.HEARTBEAT, bytes((0, 2, 19, 22)), 100.05)
        query(core, wire.POINT_STATUS, bytes((0, 3, 2, 0)), 100.06)
        core.receive(wire.Packet(0, wire.DONE, bytes((22, 3, 0, 0))), 100.1)
        self.assertIsNone(core.numbered.active)
        self.assertFalse(core.numbered.state(100.1)['arrived'])

    def test_both_extensions_arrive_in_observation_without_automatic_scan(self):
        c = ExtensionControl()
        self.assertTrue(dialogue_allowed(c.state(100.), 100.))
        c.task = 'traffic'
        self.assertFalse(dialogue_allowed(c.state(100.), 100.))
        core = running(profile='dialogue')
        self.assertEqual(core.numbered.active['arrival_policy'], 'manual')
        self.assertEqual(core.numbered.active['task_profile'], 'extension2')
        self.assertTrue(any(effect[0] == 'mission' and effect[1].get('arrival_policy') == 'manual' for effect in core.effects))
        self.assertFalse(any(effect[0] == 'interaction' for effect in core.effects))


if __name__ == '__main__':
    unittest.main()
