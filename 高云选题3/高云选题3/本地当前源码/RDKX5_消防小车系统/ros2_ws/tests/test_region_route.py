"""Directed-ROM scheduling and correlated entry/search/exit regressions."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision.region_route import RegionRoutePlanner, inside
from rescue_vision.region_search import RegionSearch
from rescue_vision.area_navigation import SET_REGION_PREFERENCE
from test_region_search import frame, actuator, geometry
from test_area_navigation import Harness as WireHarness

CATALOG = json.loads((ROOT / 'config/semantic_regions.json').read_text(encoding='utf8'))
GRAPH = json.loads((ROOT / 'config/region_traversability.json').read_text(encoding='utf8'))


class DirectedRegionRouteTests(unittest.TestCase):
    def setUp(self):
        self.planner = RegionRoutePlanner(GRAPH, CATALOG, GRAPH['map_signature'])

    def plan(self, region='A', heading=180., cell=(88, 13), remaining=None):
        return self.planner.plan(region, CATALOG['regions'][region]['waypoints'] if remaining is None else remaining,
                                 dict(valid=True, cell=list(cell), heading_deg=heading))

    def test_same_position_opposite_heading_changes_order(self):
        left, right = self.plan(heading=180.), self.plan(heading=0.)
        self.assertNotEqual([p['id'] for p in left['waypoints']], [p['id'] for p in right['waypoints']])
        self.assertEqual(left['waypoints'][0]['id'], 'A2')
        self.assertEqual(right['waypoints'][0]['id'], 'A3')

    def test_all_regions_visit_each_point_once_then_exit_with_terminal_clearance(self):
        for region in 'ABCD':
            with self.subTest(region=region):
                result = self.plan(region, 90., (163, 80))
                self.assertEqual(set(p['id'] for p in result['waypoints']),
                                 set(p['id'] for p in CATALOG['regions'][region]['waypoints']))
                self.assertEqual(len(result['waypoints']), 4)
                bounds = CATALOG['regions'][region]['bounds_image_cm']
                cell = result['exit']['cell']
                self.assertFalse(inside(cell, bounds))
                # Every pose within the 4-cell acceptance box is still outside.
                for dx in (-4, 4):
                    for dy in (-4, 4):
                        self.assertFalse(inside([cell[0] + dx, cell[1] + dy], bounds))
                self.assertFalse(result['exit']['confirmed_stopped'])

    def test_known_in_region_tour_avoids_external_detour(self):
        result = self.plan('A')
        self.assertEqual(result['route_policy']['estimated_outside_cm'], 0)
        self.assertIsNone(result['route_policy']['fallback_reason'])
        self.assertEqual(result['route_policy']['outside_extra_multiplier'], 32)

    def test_missing_actual_heading_and_wrong_map_are_rejected(self):
        for ref in (dict(valid=True, cell=[88, 13]), dict(valid=False, cell=[88, 13], heading_deg=0)):
            with self.assertRaisesRegex(ValueError, 'actual_pose_and_heading'):
                self.planner.plan('A', CATALOG['regions']['A']['waypoints'], ref)
        wrong = copy.deepcopy(GRAPH)
        wrong['directed_nodes'][0][0] += 1
        with self.assertRaisesRegex(ValueError, 'identity_invalid'):
            RegionRoutePlanner(wrong, CATALOG, GRAPH['map_signature'])

    def test_blocked_road_not_replaced_with_straight_line(self):
        self.planner.observe_map(dict(barriers=dict(valid=True, segments_mm=[[0, 0, 3520, 3200]])))
        with self.assertRaises(ValueError):
            self.plan()

    def test_remaining_subset_and_exit_only_have_no_revisited_stop(self):
        remaining = CATALOG['regions']['A']['waypoints'][2:]
        result = self.plan(remaining=remaining)
        self.assertEqual(set(p['id'] for p in result['waypoints']), {'A3', 'A4'})
        result = self.plan(remaining=[])
        self.assertEqual(result['waypoints'], [])
        self.assertFalse(inside(result['exit']['cell'], CATALOG['regions']['A']['bounds_image_cm']))

    def test_start_on_each_waypoint_never_sends_same_point_as_first_goal(self):
        for region in 'ABCD':
            for point in CATALOG['regions'][region]['waypoints']:
                candidates = [(abs(row[0] - (point['cell'][0] * 512 + 256)) +
                               abs(row[1] - (point['cell'][1] * 512 + 256)), row[2])
                              for row in self.planner.nodes if row[2] != 255]
                heading = min(candidates)[1]
                result = self.plan(region, heading, point['cell'])
                self.assertNotEqual(result['waypoints'][0]['id'], point['id'])
                self.assertIn(point['id'], [p['id'] for p in result['waypoints']])

    def test_rejects_invalid_edge_even_with_recomputed_hash(self):
        wrong = copy.deepcopy(GRAPH)
        wrong['directed_nodes'][0][3] = [[999999, 1]]
        wrong['directed_nodes_sha256'] = hashlib.sha256(json.dumps(wrong['directed_nodes'], separators=(',', ':')).encode()).hexdigest()
        with self.assertRaisesRegex(ValueError, 'edge_invalid'):
            RegionRoutePlanner(wrong, CATALOG, GRAPH['map_signature'])


class SchedulerFixture:
    def __init__(self): self.calls = []

    def plan(self, region, remaining, reference):
        if reference.get('heading_deg') is None:
            raise ValueError('region_replan_requires_actual_pose_and_heading')
        self.calls.append(copy.deepcopy(reference))
        # Deliberately reverse after each stop to prove live measured pose is
        # supplied and already visited points cannot be silently revisited.
        return dict(waypoints=list(reversed(remaining)),
                    exit=dict(id='A_EXIT', cell=[96, 13], heading_deg=0, confirmed_stopped=False),
                    route_policy=dict(planned_from=copy.deepcopy(reference)))


class RegionExitStateTests(unittest.TestCase):
    def setUp(self):
        self.planner = SchedulerFixture()
        self.search = RegionSearch(CATALOG, self.planner)
        self.now = 100.
        self.search.start(dict(action='search_region', region_id='A', execute_fire=False, command_id='task'),
                          self.now, dict(valid=True, cell=[163, 80], heading_deg=90.))

    def tick(self, *, phase='arrived', stopped=True, changes=None):
        self.now += .2
        nav = dict(stamp=self.now, command_id=self.search.leg_id, phase=phase,
                   stopped=stopped, pose_valid=True, pose_cell=self.search.goal_cell,
                   pose_heading_deg=37.5, arrived=phase == 'arrived')
        nav.update(changes or {})
        return self.search.tick(self.now, nav, frame(self.now), {}, actuator(self.now))

    def visit(self):
        self.tick()
        for _ in range(8): self.tick()

    def reach_exit(self):
        for _ in range(4): self.visit()
        self.assertEqual(self.search.phase, 'EXITING')

    def test_entry_then_dynamic_order_uses_measured_heading(self):
        self.assertEqual(self.search.phase, 'ENTERING')
        self.visit()
        self.assertEqual(self.search.phase, 'NAVIGATING')
        self.assertEqual(self.planner.calls[-1]['heading_deg'], 37.5)
        state = self.search.snapshot(self.now)
        self.assertEqual(len(state['remaining_waypoints']), 3)
        self.assertEqual(state['waypoints'][0]['id'], state['visited_waypoint_ids'][0])

    def test_all_search_stops_do_not_complete_before_exit_arrival(self):
        self.reach_exit()
        self.assertTrue(self.search.active)
        self.tick(phase='running', stopped=False)
        self.assertEqual(self.search.phase, 'EXITING')
        self.tick(changes=dict(command_id='old-leg'))
        self.assertEqual(self.search.phase, 'EXITING')
        self.tick(stopped=False)
        self.assertEqual(self.search.phase, 'EXITING')
        self.tick()
        self.assertEqual(self.search.phase, 'COMPLETE')
        self.assertTrue(self.search.exit['confirmed_stopped'])
        self.assertEqual(self.search.reason, 'region_search_exited_and_stopped')

    def test_exit_needs_outside_pose_not_only_done(self):
        self.reach_exit()
        self.search.goal_cell = [89, 13]  # Corrupt/older externally supplied exit.
        self.tick(changes=dict(pose_cell=[87, 13]))
        self.assertEqual(self.search.phase, 'FAILED')
        self.assertEqual(self.search.reason, 'region_exit_pose_still_inside')

    def test_missing_heading_fails_reschedule_without_reusing_start_heading(self):
        self.tick()
        for _ in range(8): self.tick(changes=dict(pose_heading_deg=None))
        self.assertEqual(self.search.phase, 'FAILED')
        self.assertIn('actual_pose_and_heading', self.search.reason)

    def test_cancelled_exit_cannot_complete_from_late_done(self):
        self.reach_exit()
        self.search.cancel(self.now, 'operator_stop')
        self.tick()
        self.assertEqual(self.search.phase, 'CANCELLED')
        self.assertFalse(self.search.exit['confirmed_stopped'])

    def test_last_remaining_point_at_measured_paused_pose_observes_without_new_route(self):
        search = self.search
        search.visited = [point['id'] for point in search.waypoints[:-1]]
        goal = search.waypoints[-1]['cell']
        search.last_navigation = dict(stamp=self.now, command_id=search.leg_id, phase='paused',
                                     stopped=True, pose_valid=True, pose_cell=goal, pose_heading_deg=37.5)
        effects_before = len(search.effects)
        search._schedule(self.now, dict(valid=True, cell=goal, heading_deg=37.5))
        self.assertEqual(search.phase, 'OBSERVING')
        self.assertEqual(len(search.effects), effects_before)
        self.assertEqual(search.goal_cell, goal)

    def test_fire_seen_during_paused_observation_works_in_both_modes(self):
        for execute_fire in (False, True):
            self.setUp()
            search = self.search
            search.execute_fire = execute_fire
            search.visited = [point['id'] for point in search.waypoints[:-1]]
            goal = search.waypoints[-1]['cell']
            search.last_navigation = dict(stamp=self.now, command_id=search.leg_id, phase='paused',
                                         stopped=True, pose_valid=True, pose_cell=goal, pose_heading_deg=37.5)
            search._schedule(self.now, dict(valid=True, cell=goal, heading_deg=37.5))
            for _ in range(3):
                self.now += .2
                nav = dict(search.last_navigation, stamp=self.now)
                search.tick(self.now, nav, frame(self.now, 11), {}, actuator(self.now),
                            geometry(self.now, target=11, in_spray_range=True))
            self.assertEqual(search.phase, 'EXTINGUISHING' if execute_fire else 'OBSERVATION_DWELL')
            if not execute_fire:
                self.assertTrue(search.active)
                # A confirmed sighting is an observation, not task completion.
                # Keep measured stop fresh through two seconds, then complete
                # the remaining checkpoint observation and route to the exit.
                for _ in range(22):
                    self.now += .2
                    nav = dict(search.last_navigation, stamp=self.now)
                    search.tick(self.now, nav, frame(self.now, 11), {}, actuator(self.now))
                    if search.phase == 'EXITING':
                        break
                self.assertEqual(search.phase, 'EXITING')
                self.assertEqual(len(search.visited), 4)
                self.assertTrue(search.active)


class RegionPreferenceWireTests(unittest.TestCase):
    def test_preference_is_acknowledged_after_reset_and_before_start(self):
        for bounds in ([0, 0, 87, 79], None):
            h = WireHarness()
            n = h.c.area.navigation
            n.map['region_preference_supported'] = True
            n.start(dict(command_id='route-policy', goal_cell=[17, 17], execute_fire=False,
                         start_reference=dict(valid=True, cell=[88, 13], heading_deg=180.),
                         region_preference_cells=bounds, region_route_policy_required=True), h.now + .002)
            h.ack()
            h.feedback(session=False)
            n.tick(h.now + .002)
            self.assertEqual(n.active['phase'], 'setting_region_preference')
            self.assertEqual(h.c.pending[n.active['sequence']][0], SET_REGION_PREFERENCE)
            payload = bytes(bounds) if bounds is not None else b'\xff' * 4
            self.assertEqual(h.c.outbox[-1][5:9], payload)
            self.assertEqual(h.ack(), SET_REGION_PREFERENCE)
            self.assertEqual(n.active['phase'], 'setting_start')

    def test_old_firmware_rejected_before_any_new_motion_command(self):
        h = WireHarness()
        before = len(h.c.outbox)
        with self.assertRaisesRegex(ValueError, 'preference_firmware_required'):
            h.c.area.navigation.start(dict(command_id='route-policy', goal_cell=[17, 17], execute_fire=False,
                start_reference=dict(valid=True, cell=[88, 13], heading_deg=180.),
                region_route_policy_required=True), h.now + .002)
        self.assertEqual(len(h.c.outbox), before)


if __name__ == '__main__':
    unittest.main()
