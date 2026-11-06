"""Whole semantic-search controller with synthetic FPGA and YOLO feedback.

Legacy per-leg coverage uses the shipped catalog/directed ROM graph and real
packet dispatch. The production continuous task is covered in
test_region_continuous_flow.py. Nothing
in this file opens UART, publishes ROS commands, or controls physical hardware.
"""
import heapq
import json
import math
from pathlib import Path
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision import ov5640_protocol as wire
from rescue_vision.area_navigation import RESET_NAV, SET_REGION_PREFERENCE, SET_SPEED
from rescue_vision.extension_control import ExtensionControl
from rescue_vision.task_interaction import parse_task_text


def document(name):
    return json.loads((ROOT / 'config' / name).read_text(encoding='utf-8'))


class MapGeometry:
    """Real map input, with no fabricated stereo/nozzle calibration."""
    def __init__(self):
        self.traversability = document('region_traversability.json')

    def observe_map(self, value, now):
        pass

    def observe_targets(self, value, now):
        pass

    def readiness(self):
        return 'fixture_has_no_physical_calibration'


class ControllerHarness:
    def __init__(self):
        self.now = 100.
        self.c = ExtensionControl(True)
        self.c.task = 'dialogue'
        self.c.protocol_ready = True
        self.c.capabilities = wire.capabilities(bytes((0, 2, 127, 0)))
        self.c.numbered.load_catalog(document('numbered_points.json'))
        self.c.numbered.heading_supported = True
        self.catalog = document('semantic_regions.json')
        self.signature = document('fpga_map_catalog.json')['map_signature']
        self.c.area.load(self.catalog, self.signature, MapGeometry(),
                         alignment_config=document('checkpoint_alignment.json'))
        self.nav = self.c.area.navigation
        self.search = self.c.area.search
        self.pose = None
        self.targets = []
        self.session = False
        self.run = False
        self.motion = 0
        self.task = 0
        self.point_state = 0
        self.route = dict(generation=1)
        self.speed = 200
        self.sent_commands = []
        self.legs = []
        self.feedback()

    def query(self, command, payload):
        sequence = self.c.send(command, now=self.now)
        self.c.receive(wire.Packet(sequence, command | 128, payload), self.now + .001)

    def feedback(self, advance=.05, *, tick=False, **changes):
        self.now += advance
        for name, value in changes.items():
            setattr(self, name, value)
        c = self.c
        c.actuator = dict(stamp=self.now, enabled=True, dry_run=False,
            status_fresh=True, feedback_fresh=True, reference_confirmed_this_start=True,
            pump_enabled=True, pump_ready=True, pump_on=False, pump_feedback_fresh=True,
            guard=dict(armed=False))
        c.mission_state = dict(stamp=self.now, mounting_verified=False, gimbal_transport='x5')
        c.auto_state = dict(stamp=self.now, active=False, phase='IDLE')
        c.observe_targets(dict(capture_stamp=self.now, capture_stamp_ns=round(self.now * 1e9),
            sequence=round(self.now * 1000), frame_valid=True,
            registry_session='integration-camera', image_size_wh=[640, 480],
            targets=[dict(target_id=key, selectable=True, confidence=.95,
                          bbox_xyxy=[x * 640 - 12, 100, x * 640 + 12, 200])
                     for key, x in self.targets]), self.now)
        self.query(wire.HEARTBEAT, bytes((0, self.motion,
            19 | (8 if self.session else 0) | (4 if self.run else 0), self.task)))
        self.query(wire.POINT_STATUS, bytes((0, 0, self.point_state, 0)))
        self.query(wire.EXT_STATUS, bytes((10 if self.session else 0,
            8 | (0 if self.run else 2), 0, 0)))
        pose = {}
        if self.pose:
            cell, heading = self.pose
            pose = dict(stamp=self.now, valid=True, x_mm=cell[0] * 20 + 10,
                y_mm=3200 - cell[1] * 20 - 10, heading_tenths=round(heading * 10),
                mode=9, status=self.motion, task_id=self.task,
                route_generation=self.route.get('generation'),
                execution_index=getattr(self, 'execution_index', 0))
            if getattr(self, 'image_position', None) is not None:
                pose.update(x_mm=round(self.image_position[0]), y_mm=3200-round(self.image_position[1]))
        c.area.observe_map(dict(stamp=self.now, supported=True, region_supported=True,
            region_preference_supported=True, map_signature=self.signature,
            continuous_route_supported=True,
            pose=pose, route=self.route,
            start_reference=dict(stamp=self.now, valid=True, cell=[88, 13],
                                 heading_deg=180., generation=1)), self.now)
        if tick:
            c.area.tick(self.now + .002)
            if self.search.phase == 'FAILED':
                raise AssertionError(self.search.reason + ' / ' + repr(self.nav.active))

    def start(self, region='A'):
        self.now += .01
        # Deliberately exercise the supported low-level legacy leg FSM; normal
        # RegionControl.start now always requests a single continuous mission.
        request = dict(parse_task_text('去' + region + '区域搜索火焰'),
            command_id='integration-' + region, source='web', issued_at=self.now, ttl_s=3.)
        self.c.task = 'semantic'
        self.c.area._effects(self.search.start(request, self.now, self.c.area.reference(self.now)), self.now)
        assert self.nav.active is not None, self.c.last_result
        assert self.search.execute_fire is False

    def ack(self):
        self.now += .01
        a = self.nav.active
        sequence = a['sequence']
        command, _ = self.c.pending[sequence][:2]
        payload = bytes(4)
        if command == RESET_NAV:
            payload = bytes(2) + struct.pack('<H', self.speed)
        elif command == SET_SPEED:
            desired = (self.nav.restore_speed_mm_s if a['phase'] == 'restoring_speed' else
                       a['checkpoint_speed_mm_s'] if a['phase'] == 'setting_checkpoint_speed' else 60)
            self.speed = desired
            payload = bytes(2) + struct.pack('<H', desired)
        elif command == wire.SET_START_HEADING:
            payload = bytes((0,)) + wire.heading_payload(a['start_heading_deg'])[:2] + bytes((0,))
        self.sent_commands.append((command, a['command_id'], a['task_id'], self.speed))
        self.c.receive(wire.Packet(sequence, command | 128, payload), self.now)
        return command

    def graph_path(self):
        """Synthesize a realizable returned route from shipped directed edges."""
        a, planner = self.nav.active, self.c.area.route_planner
        nodes = planner.nodes
        blocked = [False] * len(nodes)
        gx, gy = (v * 512 + 256 for v in a['goal_cell'])
        nearest = min(range(len(nodes)), key=lambda i: abs(nodes[i][0] - gx) + abs(nodes[i][1] - gy))
        target = nodes[nearest][:2]
        goals = {i for i, row in enumerate(nodes) if row[:2] == target and row[2] != 255}
        preference = a.get('region_preference_cells')
        for seeds in planner._seeds(a['start_cell'], a['start_heading_deg'], blocked):
            distance, parent, queue = {}, {}, []
            for node, access in seeds:
                distance[node], parent[node] = access, None
                heapq.heappush(queue, (access, node))
            end = None
            while queue:
                cost, node = heapq.heappop(queue)
                if distance[node] != cost:
                    continue
                if node in goals:
                    end = node
                    break
                row = nodes[node]
                outside = preference is not None and not (
                    preference[0] <= row[0] // 512 <= preference[2] and
                    preference[1] <= row[1] // 512 <= preference[3])
                for neighbor, length in row[3]:
                    candidate = cost + length * (33 if outside else 1)
                    if candidate < distance.get(neighbor, math.inf):
                        distance[neighbor], parent[neighbor] = candidate, node
                        heapq.heappush(queue, (candidate, neighbor))
            if end is not None:
                path, cursor = [], end
                while cursor is not None:
                    path.append(cursor)
                    cursor = parent[cursor]
                path.reverse()
                return path
        raise AssertionError('Synthetic FPGA cannot reach current goal through real ROM edges')

    def drive_leg(self):
        a = self.nav.active
        assert a['phase'] == 'resetting', a
        self.legs.append(dict(a))
        self.ack()
        self.feedback(session=False, run=False, motion=0, point_state=0, tick=True)
        for _ in range(8):
            if a['phase'] == 'planning':
                break
            assert a['phase'] in ('setting_alignment_speed', 'setting_region_preference',
                'setting_start', 'setting_heading', 'setting_goal', 'plan_ack'), a
            self.ack()
            self.feedback(tick=True)
        assert a['phase'] == 'planning', a
        self.feedback(point_state=2, tick=True)
        path = self.graph_path()
        nodes = self.c.area.route_planner.nodes
        row = nodes[path[-1]]
        self.arrival_pose = ([row[0] // 512, row[1] // 512], row[2])
        if a.get('alignment_guard'):
            assert a['phase'] == 'alignment_route_check', a
            self.route = dict(valid=True, sampled=False,
                generation=self.route['generation'] + 1, stamp=self.now + .01,
                points_mm=[[nodes[i][0] / 25.6, nodes[i][1] / 25.6] for i in path],
                sample_indices=list(range(len(path))), total_count=len(path))
            self.feedback(tick=True)
        assert a['phase'] == 'starting', a
        assert self.ack() == wire.START
        self.feedback(session=True, run=True, motion=2, task=a['task_id'],
            pose=(list(a['start_cell']), a['start_heading_deg']), tick=True)
        assert a['phase'] == 'running', a

    def arrive(self):
        a = self.nav.active
        self.now += .01
        self.c.receive(wire.Packet(0, wire.DONE, bytes((a['task_id'], 3, 0, 0))), self.now)
        self.feedback(run=False, motion=3, pose=self.arrival_pose, tick=True)
        if a['phase'] == 'restoring_speed':
            self.ack()
            self.feedback(tick=True)
        assert (a['phase'] == 'arrived' or
                a['leg_kind'] == 'exit' and self.search.phase == 'COMPLETE' and a['phase'] == 'stopping'), a

    def until(self, phase, limit=100, **changes):
        for _ in range(limit):
            if self.search.phase == phase:
                return
            self.feedback(tick=True, **changes)
        raise AssertionError((phase, self.search.phase, self.search.reason, self.nav.active))


class CheckpointControlIntegrationTests(unittest.TestCase):
    def test_two_front_targets_stop_dwell_resume_then_all_waypoints_exit(self):
        h = ControllerHarness()
        h.start()
        h.drive_leg()
        h.targets = [(7, .80), (3, .64), (2, .20)]
        for _ in range(10):
            h.feedback(tick=True)
        self.assertEqual(h.nav.active['phase'], 'running')
        self.assertEqual(h.search.phase, 'ENTERING')
        self.assertFalse(any(command == wire.STOP for command, *_ in h.sent_commands))
        h.arrive()
        self.assertEqual(h.search.phase, 'CHECKPOINT_COLLECT')
        self.assertEqual(h.search.checkpoint.waypoint_id, 'A2')
        h.targets = [(7, .80), (3, .64), (2, .20)]
        h.until('ALIGN_SELECT')
        self.assertEqual([q['target_id'] for q in h.search.checkpoint.queue], [3, 7])
        processed = []
        previous_task = h.nav.active['task_id']
        last_stopped_pose = None
        for target in (3, 7):
            h.until('ALIGNING')
            self.assertEqual(h.search.checkpoint.identity, ('integration-camera', target))
            self.assertNotEqual(h.nav.active['task_id'], previous_task)
            previous_task = h.nav.active['task_id']
            if last_stopped_pose:
                self.assertEqual(h.nav.active['start_cell'], last_stopped_pose[0])
                self.assertEqual(h.nav.active['start_heading_deg'], last_stopped_pose[1])
            h.drive_leg()
            self.assertEqual(h.speed, 60)
            start = h.nav.active['start_cell']
            # A2 is a westbound straight road: actual early stop after 6 cm.
            stopped_pose = ([start[0] - 3, start[1]], 180)
            h.targets = [(target, .50), (7, .70)] if target == 3 else [(7, .50)]
            h.until('ALIGN_STOP', pose=stopped_pose)
            self.assertEqual(h.nav.active['phase'], 'pausing')
            self.assertEqual(h.ack(), wire.STOP)
            for _ in range(5):
                h.feedback(run=True, motion=2, pose=stopped_pose, tick=True)
            self.assertEqual(h.search.phase, 'ALIGN_STOP')
            self.assertIsNone(h.search.checkpoint.dwell_at)
            h.feedback(run=False, motion=2, pose=stopped_pose, tick=True)
            self.assertEqual(h.nav.active['phase'], 'restoring_speed')
            h.ack()
            self.assertEqual(h.speed, 200)
            h.until('ALIGN_DWELL')
            dwell_at = h.search.checkpoint.dwell_at
            for _ in range(37):
                h.feedback(tick=True)
            self.assertLess(h.now - dwell_at, 2.)
            self.assertEqual(h.search.phase, 'ALIGN_DWELL')
            h.until('ALIGN_SELECT')
            self.assertGreaterEqual(h.now - dwell_at, 2.)
            processed.append(target)
            self.assertEqual(h.search.checkpoint.queue[len(processed) - 1]['status'], 'completed')
            last_stopped_pose = stopped_pose
        self.assertEqual(processed, [3, 7])
        checkpoint = h.search.checkpoint.snapshot(h.now)
        self.assertEqual(checkpoint['processed_count'], 2)
        self.assertLess(checkpoint['total_advance_cm'], 60.)
        h.targets = []
        # Completing this point plans from actual post-alignment pose/heading.
        for _ in range(5):
            if h.nav.active['phase'] == 'resetting':
                break
            h.feedback(tick=True)
        self.assertEqual(h.nav.active['phase'], 'resetting')
        self.assertEqual(h.nav.active['start_cell'], last_stopped_pose[0])
        self.assertEqual(h.nav.active['start_heading_deg'], last_stopped_pose[1])
        self.assertEqual(h.speed, 200)
        for _ in range(5):
            if not h.search.active:
                break
            h.drive_leg()
            h.arrive()
            if h.search.phase == 'CHECKPOINT_COLLECT':
                h.until('ALIGN_SELECT')
                h.feedback(tick=True)
        self.assertEqual(h.search.phase, 'COMPLETE', h.search.reason)
        self.assertEqual(set(h.search.visited), {'A1', 'A2', 'A3', 'A4'})
        self.assertEqual(h.legs[-1]['leg_kind'], 'exit')
        self.assertEqual(h.nav.active['phase'], 'stopping')
        h.ack()
        h.feedback(tick=True)
        self.assertEqual(h.nav.active['phase'], 'arrived')
        self.assertFalse(h.run)
        self.assertEqual(h.speed, 200)
        self.assertGreaterEqual(sum(cmd == SET_REGION_PREFERENCE for cmd, *_ in h.sent_commands), 7)
        self.assertEqual(len({leg['task_id'] for leg in h.legs}), len(h.legs))
        self.assertFalse(any(key == 'auto' and value.get('action') == 'start' for key, value in h.c.effects))
        self.assertFalse(any(key == 'actuator' and value.get('action') in ('arm', 'pump_on')
                             for key, value in h.c.effects))
        self.assertFalse(h.c.area.pump_allowed(h.now))

    def test_all_four_regions_without_targets_finish_after_real_exit_stop(self):
        for region in 'ABCD':
            with self.subTest(region=region):
                h = ControllerHarness()
                h.start(region)
                for _ in range(6):
                    if not h.search.active:
                        break
                    h.drive_leg()
                    h.arrive()
                    if h.search.phase == 'CHECKPOINT_COLLECT':
                        h.until('ALIGN_SELECT')
                        h.feedback(tick=True)
                self.assertEqual(h.search.phase, 'COMPLETE', h.search.reason)
                self.assertEqual(set(h.search.visited), {region + str(i) for i in range(1, 5)})
                self.assertEqual(h.legs[-1]['leg_kind'], 'exit')
                self.assertEqual(h.nav.active['phase'], 'stopping')
                self.assertFalse(h.run)
                h.ack()
                h.feedback(tick=True)
                self.assertEqual(h.nav.active['phase'], 'arrived')
                self.assertFalse(any(leg['leg_kind'] == 'alignment' for leg in h.legs))
                self.assertFalse(any(command == SET_SPEED for command, *_ in h.sent_commands))
                self.assertFalse(h.c.area.pump_allowed(h.now))

    def test_wrong_done_identity_and_ack_alone_cannot_begin_checkpoint(self):
        h = ControllerHarness()
        h.start()
        h.drive_leg()
        a = h.nav.active
        h.now += .01
        h.c.receive(wire.Packet(0, wire.DONE, bytes(((a['task_id'] + 1) % 255, 3, 0, 0))), h.now)
        h.feedback(tick=True)
        self.assertEqual(a['phase'], 'running')
        self.assertEqual(h.search.phase, 'ENTERING')
        h.now += .01
        h.c.receive(wire.Packet(0, wire.DONE, bytes((a['task_id'], 3, 0, 0))), h.now)
        h.feedback(run=True, motion=3, pose=h.arrival_pose, tick=True)
        self.assertEqual(a['phase'], 'confirming_arrival')
        self.assertEqual(h.search.phase, 'ENTERING')
        h.feedback(run=False, motion=3, pose=h.arrival_pose, tick=True)
        self.assertEqual(a['phase'], 'arrived')
        self.assertEqual(h.search.phase, 'CHECKPOINT_COLLECT')


if __name__ == '__main__':
    unittest.main()
