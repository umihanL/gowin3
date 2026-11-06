"""Real ROM graph → one task PLAN → uninterrupted waypoints and fire pauses.

All telemetry and detector input are synthetic; no hardware or UART is opened.
Five-Hz pose samples exercise the production controller and packet dispatch.
"""
import heapq
import math
import unittest

from test_checkpoint_control_flow import ControllerHarness
from rescue_vision import ov5640_protocol as wire
from rescue_vision.area_navigation import RESET_NAV, SET_SPEED
from rescue_vision.task_interaction import parse_task_text
from rescue_vision.retained_route import local_coordinates


class ContinuousHarness(ControllerHarness):
    def start(self, region='A', execute_fire=False):
        self.fire_mode = execute_fire
        if execute_fire:
            self.c.area.approach = FireGeometry(self)
        self.feedback()
        self.now += .01
        text = '去' + region + '区域搜索火焰' + ('并执行灭火' if execute_fire else '')
        self.c.request(dict(parse_task_text(text), command_id='continuous-' + region,
            source='web', issued_at=self.now, ttl_s=3.), self.now)
        assert self.nav.active is not None, self.c.last_result
        assert self.nav.active['continuous_route'] is True

    def feedback(self, advance=.2, **changes):
        tick = changes.pop('tick', False)
        if getattr(self, 'fire_offsets', None) and self.nav.centering.active:
            anchor = self.nav.centering.session['anchor']
            offset, _ = local_coordinates(anchor, self.image_position)
            self.targets = [(key, .5+(goal-offset)/200.) for key, goal in self.fire_offsets.items()]
        result = super().feedback(advance, tick=False, **changes)
        if getattr(self, 'local_moving', False):
            # A retained-route jog keeps nav_session but HOST.run remains false;
            # independent EXT_STATUS must still report the actual moving wheels.
            self.query(wire.EXT_STATUS, bytes((10, 8, 0, 0)))
        if getattr(self, 'fire_mode', False):
            self.c.mission_state.update(mounting_verified=True)
        if getattr(self, 'auto_factory', None):
            self.c.auto_state = self.auto_factory(self.now)
        if tick:
            self.c.area.tick(self.now + .002)
            assert self.search.phase != 'FAILED', (self.search.reason, self.nav.active)
        return result

    def full_graph_path(self):
        """Return a legal directed ROM route through every uploaded waypoint."""
        a, planner = self.nav.active, self.c.area.route_planner
        nodes = planner.nodes
        preference = a['region_preference_cells']
        seeds = planner._seeds(a['start_cell'], a['start_heading_deg'], [False] * len(nodes))[0]
        result = []
        for goal in [p['cell'] for p in a['patrol_waypoints']] + [a['goal_cell']]:
            gx, gy = (v * 512 + 256 for v in goal)
            nearest = min(range(len(nodes)), key=lambda i: abs(nodes[i][0] - gx) + abs(nodes[i][1] - gy))
            xy = nodes[nearest][:2]
            goals = {i for i, row in enumerate(nodes) if row[:2] == xy and row[2] != 255}
            distances, parents, queue = {}, {}, []
            for node, access in seeds:
                distances[node], parents[node] = access, None
                heapq.heappush(queue, (access, node))
            end = None
            while queue:
                cost, node = heapq.heappop(queue)
                if distances[node] != cost:
                    continue
                if node in goals:
                    end = node
                    break
                row = nodes[node]
                outside = not (preference[0] <= row[0] // 512 <= preference[2] and
                               preference[1] <= row[1] // 512 <= preference[3])
                for neighbor, length in row[3]:
                    candidate = cost + length * (33 if outside else 1)
                    if candidate < distances.get(neighbor, math.inf):
                        distances[neighbor], parents[neighbor] = candidate, node
                        heapq.heappush(queue, (candidate, neighbor))
            assert end is not None, (goal, seeds)
            path, cursor = [], end
            while cursor is not None:
                path.append(cursor)
                cursor = parents[cursor]
            path.reverse()
            result.extend(path[1:] if result and result[-1] == path[0] else path)
            seeds = [(end, 0.)]
        return result

    def drive_mission(self):
        a = self.nav.active
        assert a['phase'] == 'resetting', a
        self.legs.append(dict(a))
        self.ack()
        self.feedback(session=False, run=False, motion=0, point_state=0, tick=True)
        for _ in range(20):
            if a['phase'] == 'planning':
                break
            self.ack()
            self.feedback(tick=True)
        assert a['phase'] == 'planning', a
        self.feedback(point_state=2, tick=True)
        assert a['phase'] == 'continuous_route_check', a
        self.path = self.full_graph_path()
        nodes = self.c.area.route_planner.nodes
        points = [[nodes[i][0] / 25.6, nodes[i][1] / 25.6] for i in self.path]
        self.route = dict(valid=True, sampled=False, generation=self.route['generation'] + 1,
            stamp=self.now + .01, points_mm=points, sample_indices=list(range(len(points))), total_count=len(points))
        self.feedback(tick=True)
        assert a['phase'] == 'starting', a
        assert self.ack() == wire.START
        self.path_cursor = 0
        self.execution_index = 0
        self.feedback(session=True, run=True, motion=2, task=a['task_id'],
            pose=(list(a['start_cell']), a['start_heading_deg']), tick=True)
        assert a['phase'] == 'running', a

    def path_pose(self, index):
        points = self.route['points_mm']
        point = points[index]
        self.image_position = list(point)
        after = min(index + 1, len(points) - 1)
        before = index if after != index else index - 1
        direction = math.degrees(math.atan2(points[before][1] - points[after][1],
                                           points[after][0] - points[before][0]))
        return ([int(v // 20) for v in point], direction)

    def advance(self, count=1):
        for _ in range(count):
            self.path_cursor += 1
            assert self.path_cursor < len(self.path), 'end of retained route'
            self.execution_index = min(self.path_cursor + 3, len(self.path) - 1)
            self.feedback(run=True, motion=2, pose=self.path_pose(self.path_cursor), tick=True)
            if self.nav.active['phase'] == 'pausing':
                return

    def detect(self, identifier=1):
        self.targets = [(identifier, .50)]
        for _ in range(6):
            self.advance()
            if self.search.phase == 'WAIT_STOP':
                return
        raise AssertionError(('stable flame not paused', self.search.snapshot(self.now)))

    def verify_pause(self):
        a = self.nav.active
        assert a['phase'] == 'pausing', a
        assert self.ack() == wire.STOP
        self.feedback(run=False, motion=2, tick=True)
        if a['phase'] == 'restoring_speed':
            assert self.ack() == SET_SPEED
            self.feedback(tick=True)
        assert a['phase'] == 'paused', a

    def resume(self):
        a = self.nav.active
        assert a['phase'] == 'resuming', a
        assert self.ack() == wire.RESUME
        self.feedback(run=True, motion=2, tick=True)
        assert a['phase'] == 'running', a

    def finish_observation(self):
        self.until('CENTER_DWELL')
        entered = self.search.centering.dwell_at
        while self.now + .002 - entered < .8:
            self.feedback(tick=True)
        assert self.nav.active['phase'] == 'paused'
        self.until('EXITING' if len(self.search.visited) == 4 else 'NAVIGATING')
        assert self.now + .002 - entered >= 1.
        self.resume()

    def complete_jog(self):
        a, s = self.nav.active, self.nav.centering.session
        assert a['phase'] == 'centering_starting', a
        assert self.ack() == wire.RETAINED_JOG
        jog = s['jog']; offset = jog['target_offset_mm']
        angle = math.radians(s['anchor']['heading_deg'])
        self.image_position = [s['anchor']['image_mm'][0] + offset*math.cos(angle),
                               s['anchor']['image_mm'][1] - offset*math.sin(angle)]
        self.pose = ([int(v//20) for v in self.image_position], s['anchor']['heading_deg'])
        self.local_moving = True
        self.feedback(advance=abs(jog['proposal']['distance_mm'])/jog['proposal']['speed_mm_s'], run=False, motion=2, tick=True)
        sequence = s['query_sequence']
        assert sequence is not None, a
        self.now += .01
        self.c.receive(wire.Packet(sequence, wire.RETAINED_JOG_STATUS|128,
            bytes((3, 0, jog['sequence'], 3))), self.now)
        self.local_moving = False
        self.feedback(tick=True)
        assert a['phase'] in ('paused', 'resuming'), (a['phase'], self.search.phase)

    def finish_exit(self):
        a = self.nav.active
        for index in range(self.path_cursor, len(self.path)):
            self.path_cursor = index
            self.execution_index = min(index + 3, len(self.path) - 1)
            self.feedback(run=True, motion=2, pose=self.path_pose(index), tick=True)
        self.now += .01
        self.c.receive(wire.Packet(0, wire.DONE, bytes((a['task_id'], 3, 0, 0))), self.now)
        self.feedback(run=False, motion=3, tick=True)
        assert self.search.phase == 'COMPLETE', (self.search.reason, a)
        assert a['phase'] == 'stopping', a
        self.ack()
        self.feedback(tick=True)
        assert a['phase'] == 'arrived', a


class FireGeometry:
    """Explicit synthetic stereo fixture; never a claim of physical calibration."""
    def __init__(self, harness): self.h = harness
    def readiness(self): return ''
    def set_region(self, region): self.region = region
    def observe_map(self, value, now): pass
    def observe_targets(self, value, now): pass
    def proposal(self, now, preferred):
        identity = preferred or (('integration-camera', self.h.targets[0][0]) if self.h.targets else None)
        if identity is None: return {}
        return dict(stamp=now, capture_stamp=now, registry_session=identity[0], target_id=identity[1],
            valid=True, calibration_verified=True, map_verified=True, clearance_verified=True,
            map_sha256=self.h.catalog['map_sha256'], fire_world_m=[1., 2.5, .2],
            goal_cell=list(self.h.pose[0]), in_spray_range=True)


class ContinuousRegionIntegrationTests(unittest.TestCase):
    def assert_single_plan(self, h):
        commands = [row[0] for row in h.sent_commands]
        self.assertEqual(commands.count(wire.PLAN), 1)
        self.assertEqual(commands.count(RESET_NAV), 1)
        self.assertEqual(commands.count(wire.START), 1)
        self.assertEqual(commands.count(wire.WAYPOINT), 4)
        self.assertEqual(commands.count(SET_SPEED), 0)
        self.assertEqual(len(h.legs), 1)

    def test_all_four_regions_pass_every_waypoint_without_stop_or_resume_then_exit(self):
        for region in 'ABCD':
            with self.subTest(region=region):
                h = ContinuousHarness(); h.start(region); h.drive_mission()
                identity = h.nav.active['task_id'], h.nav.active['route_generation']
                for _ in range(len(h.path) - 1):
                    h.advance()
                    self.assertEqual(h.nav.active['phase'], 'running')
                    self.assertIn(h.search.phase, ('NAVIGATING', 'EXITING'))
                    self.assertFalse(h.nav.state(h.now + .002)['checkpoint_ready'])
                    self.assertEqual((h.nav.active['task_id'], h.nav.active['route_generation']), identity)
                self.assertEqual(len(h.search.visited), 4)
                self.assertEqual([row for row in h.sent_commands if row[0] in (wire.STOP, wire.RESUME)], [])
                self.assertEqual(h.search.phase, 'EXITING')
                h.finish_exit()
                self.assertEqual(set(h.search.visited), {region + str(i) for i in range(1, 5)})
                self.assert_single_plan(h)
                self.assertFalse(h.c.area.pump_allowed(h.now))

    def test_flame_centers_one_second_without_consuming_waypoint_and_keeps_route(self):
        h = ContinuousHarness(); h.start(); h.drive_mission()
        identity = h.nav.active['task_id'], h.nav.active['route_generation']
        h.detect()
        self.assertEqual(h.search.phase, 'WAIT_STOP')
        passed = len(h.search.visited)
        self.assertIsNone(h.search.observation_dwell_at)
        h.verify_pause()
        self.assertEqual(h.search.phase, 'CENTER_COLLECT')
        h.finish_observation()
        self.assertEqual(len(h.search.visited), passed)
        self.assertEqual((h.nav.active['task_id'], h.nav.active['route_generation']), identity)
        self.assertEqual(h.search.handled, {('integration-camera', 1)})
        # Keep the processed flame visible throughout the remaining route.
        h.finish_exit()
        self.assertEqual(len(h.search.visited), 4)
        self.assertEqual(sum(row[0] == wire.RESUME for row in h.sent_commands), 1)
        self.assert_single_plan(h)

    def test_both_image_sides_jog_forward_reverse_then_restore_same_rom_route(self):
        h = ContinuousHarness(); h.start(); h.drive_mission()
        # Locate a straight part of the real ROM route with room on both sides.
        found = False
        points = h.route['points_mm']
        for index in range(3, len(points)-8):
            angle = h.path_pose(index)[1]
            anchor = dict(image_mm=points[index], heading_deg=angle)
            behind = local_coordinates(anchor, points[index-3])
            ahead = local_coordinates(anchor, points[index+6])
            if behind[0] < -100 and ahead[0] > 100 and abs(behind[1]) < 10 and abs(ahead[1]) < 10:
                while h.path_cursor < index-2:
                    h.advance()
                found = True
                break
        self.assertTrue(found, 'shipped route must have a straight bidirectional test section')
        h.targets = [(1, .7), (2, .3)]
        while h.search.phase != 'WAIT_STOP': h.advance()
        h.verify_pause()
        identity = (h.nav.active['task_id'], h.nav.active['route_generation'], h.execution_index,
                    h.nav.active['progress_mm'], h.nav.active['checkpoint_index'])
        h.fire_offsets = {1:40, 2:-40}
        observed_dwell = set()
        for _ in range(100):
            if h.nav.active['phase'] == 'resuming': break
            if h.nav.active['phase'] == 'centering_starting': h.complete_jog()
            else: h.feedback(tick=True)
            if h.search.phase == 'CENTER_DWELL': observed_dwell.add(h.search.centering.identity)
        self.assertEqual(h.nav.active['phase'], 'resuming', h.search.snapshot(h.now))
        self.assertEqual(h.search.handled, {('integration-camera',1), ('integration-camera',2)})
        self.assertEqual(observed_dwell, h.search.handled)
        self.assertEqual(identity, (h.nav.active['task_id'], h.nav.active['route_generation'], h.execution_index,
                                   h.nav.active['progress_mm'], h.nav.active['checkpoint_index']))
        self.assertGreaterEqual(h.nav.centering.session['jog_count'], 4)
        self.assertAlmostEqual(local_coordinates(h.nav.centering.session['anchor'], h.image_position)[0], 0, delta=1)
        h.resume(); h.fire_offsets = {}; h.finish_exit()
        self.assert_single_plan(h)
        self.assertFalse(any(row[0] in (wire.MOVE, wire.VELOCITY) for row in h.sent_commands))

    def test_cooldown_keeps_driving_old_flame_deduped_new_flame_can_pause_afterward(self):
        h = ContinuousHarness(); h.start(); h.drive_mission(); h.detect(); h.verify_pause(); h.finish_observation()
        h.targets = [(1, .50), (2, .50)]
        for _ in range(10):
            h.advance()
            self.assertEqual(h.nav.active['phase'], 'running')
        for _ in range(12):
            h.advance()
            if h.search.phase == 'WAIT_STOP': break
        self.assertEqual(h.search.target, ('integration-camera', 2))
        h.verify_pause(); h.finish_observation()
        h.finish_exit()
        self.assertEqual(h.search.handled, {('integration-camera', 1), ('integration-camera', 2)})
        self.assertEqual(sum(row[0] == wire.RESUME for row in h.sent_commands), 2)
        self.assert_single_plan(h)

    def test_firefighting_switch_observes_then_waits_for_confirmed_off_before_resuming(self):
        h = ContinuousHarness(); h.start(execute_fire=True); h.drive_mission(); h.detect(); h.verify_pause()
        self.assertEqual(h.search.phase, 'CENTER_COLLECT')
        self.assertFalse(any(k == 'auto' and v.get('action') == 'start' for k, v in h.c.effects))
        h.until('WAIT_SPRAY_RANGE')
        h.feedback(tick=True)
        self.assertEqual(h.search.phase, 'EXTINGUISHING')
        run_id = h.search.run_id
        self.assertTrue(any(k == 'auto' and v.get('action') == 'start' for k, v in h.c.effects))
        h.auto_factory = lambda now: dict(stamp=now, command_id=run_id, active=False,
            phase='COMPLETE', reason='one_target_round_finished', saw_pump_on=True,
            burst_completed=True, confirmed_pump_off_at=now-.01)
        h.feedback(tick=True)
        self.assertEqual(h.search.phase, 'WAIT_PUMP_OFF')
        self.assertEqual(h.nav.active['phase'], 'paused')
        h.feedback(tick=True)
        self.assertEqual(h.search.phase, 'CENTER_SELECT')
        h.until('NAVIGATING')
        self.assertEqual(h.nav.active['phase'], 'resuming')
        h.resume(); h.finish_exit()
        self.assertEqual(h.search.rounds, 1)
        self.assertEqual(len(h.search.visited), 4)
        self.assert_single_plan(h)

    def test_exit_leg_still_detects_fire_then_returns_to_original_exit(self):
        h = ContinuousHarness(); h.start(); h.drive_mission()
        while h.search.phase != 'EXITING':
            h.advance()
        self.assertEqual(len(h.search.visited), 4)
        h.detect(identifier=7); h.verify_pause(); h.finish_observation()
        self.assertEqual(h.search.phase, 'EXITING')
        h.finish_exit()
        self.assertEqual(h.search.handled, {('integration-camera', 7)})
        self.assertEqual(sum(row[0] == wire.RESUME for row in h.sent_commands), 1)
        self.assert_single_plan(h)

    def test_missing_continuous_firmware_capability_blocks_before_motion(self):
        h = ContinuousHarness()
        # start refreshes the normal fixture; inject missing capability at the
        # real entry instead, without fabricating a motion state.
        h.nav.map['continuous_route_supported'] = False
        with self.assertRaisesRegex(ValueError, 'continuous_route_firmware_required'):
            h.c.request(dict(parse_task_text('去A区域搜索火焰'), command_id='unsupported',
                source='web', issued_at=h.now + .01, ttl_s=3.), h.now + .01)
        self.assertIsNone(h.nav.active)
        self.assertFalse(h.search.active)

    def test_flame_pause_ack_without_measured_stop_does_not_start_centered_timer(self):
        h = ContinuousHarness(); h.start(); h.drive_mission(); h.detect()
        self.assertEqual(h.ack(), wire.STOP)
        for _ in range(5):
            h.feedback(run=True, motion=2, tick=True)
            self.assertEqual(h.search.phase, 'WAIT_STOP')
            self.assertIsNone(h.search.observation_dwell_at)
        h.feedback(run=False, motion=2, tick=True)
        self.assertEqual(h.search.phase, 'CENTER_COLLECT')
        self.assertIsNone(h.search.centering.dwell_at)
        h.until('CENTER_DWELL')
        self.assertGreater(h.search.centering.dwell_at, h.nav.active['pause_ack_at'])


if __name__ == '__main__':
    unittest.main()
