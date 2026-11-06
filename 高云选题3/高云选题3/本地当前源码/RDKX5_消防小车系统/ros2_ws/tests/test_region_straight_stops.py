"""Production-ROM stop placement and post-arrival heading regressions.

These use simulated odometry and independent leg solving, never hardware I/O.
"""
import heapq
import math
import unittest

from test_region_route import CATALOG, GRAPH
from test_region_route_rollout import fpga_leg
from rescue_vision.region_route import RegionRoutePlanner, inside


NODES = GRAPH['directed_nodes']


def terminal_nodes(cell):
    x, y = [v * 512 + 256 for v in cell]
    nearest = min(range(len(NODES)), key=lambda i: abs(NODES[i][0] - x) + abs(NODES[i][1] - y))
    gx, gy = NODES[nearest][:2]
    return [i for i, row in enumerate(NODES) if abs(row[0] - gx) + abs(row[1] - gy) <= 6 * 256]


def forward_runway(index):
    heading = NODES[index][2]
    distance, queue = {index: 0}, [(0, index)]
    while queue:
        cost, node = heapq.heappop(queue)
        if cost != distance[node]:
            continue
        for neighbor, length in NODES[node][3]:
            if NODES[neighbor][2] == heading and cost + length < distance.get(neighbor, math.inf):
                distance[neighbor] = cost + length
                heapq.heappush(queue, (cost + length, neighbor))
    return max(distance.values())


class StraightCheckpointTests(unittest.TestCase):
    def test_every_terminal_alias_is_straight_with_forward_clearance(self):
        for region in CATALOG['regions'].values():
            for point in region['waypoints']:
                with self.subTest(point=point['id']):
                    aliases = terminal_nodes(point['cell'])
                    headings = {NODES[index][2] for index in aliases}
                    self.assertIn(headings, ({0, 180}, {-90, 90}))
                    # Includes BOTH arrival directions and every node in the
                    # FPGA's 6 cm terminal goal tolerance, not just the pin.
                    self.assertGreaterEqual(min(forward_runway(i) for i in aliases), 16)

    def test_stop_tolerance_and_measured_heading_drift_allow_next_leg(self):
        planner = RegionRoutePlanner(GRAPH, CATALOG, GRAPH['map_signature'])
        for region_id, region in CATALOG['regions'].items():
            for point in region['waypoints']:
                remaining = [p for p in region['waypoints'] if p['id'] != point['id']]
                for index in terminal_nodes(point['cell']):
                    row = NODES[index]
                    cell = [row[0] >> 9, row[1] >> 9]
                    for drift in (-25., 0., 25.):
                        heading = (row[2] + drift + 180) % 360 - 180
                        with self.subTest(point=point['id'], cell=cell, heading=heading):
                            plan = planner.plan(region_id, remaining,
                                dict(valid=True, cell=cell, heading_deg=heading))
                            self.assertEqual({p['id'] for p in plan['waypoints']}, {p['id'] for p in remaining})
                            self.assertEqual(plan['route_policy']['planned_from']['heading_deg'], heading)
                            # Independent FPGA-ROM single-leg solver must
                            # agree that the actual measured heading can leave.
                            bounds = [int(v / 2) - (1 if i > 1 else 0)
                                      for i, v in enumerate(region['bounds_image_cm'])]
                            fpga_leg(NODES, cell, heading, plan['waypoints'][0]['cell'], bounds)

    def test_all_regions_finish_with_nonideal_arrival_headings(self):
        planner = RegionRoutePlanner(GRAPH, CATALOG, GRAPH['map_signature'])
        for region_id, region in CATALOG['regions'].items():
            for first in region['waypoints']:
                aliases = terminal_nodes(first['cell'])
                for initial_heading in sorted({NODES[i][2] for i in aliases}):
                    for error in (-20., 20.):
                        cell, heading, remaining = list(first['cell']), initial_heading, list(region['waypoints'])
                        visits = []
                        while remaining:
                            plan = planner.plan(region_id, remaining,
                                dict(valid=True, cell=cell, heading_deg=heading))
                            target = plan['waypoints'][0]
                            bounds = [int(v / 2) - (1 if i > 1 else 0)
                                      for i, v in enumerate(region['bounds_image_cm'])]
                            cell, actual_heading, _ = fpga_leg(NODES, cell, heading, target['cell'], bounds)
                            heading = (actual_heading + error + 180) % 360 - 180
                            visits.append(target['id'])
                            remaining = [p for p in remaining if p['id'] != target['id']]
                        plan = planner.plan(region_id, [], dict(valid=True, cell=cell, heading_deg=heading))
                        cell, _, _ = fpga_leg(NODES, cell, heading, plan['exit']['cell'], None)
                        self.assertEqual(len(set(visits)), 4)
                        self.assertFalse(inside(cell, region['bounds_image_cm']))

    def test_reported_d4_pose_can_continue_without_replacing_actual_heading(self):
        planner = RegionRoutePlanner(GRAPH, CATALOG, GRAPH['map_signature'])
        region = CATALOG['regions']['D']
        remaining = [p for p in region['waypoints'] if p['id'] in ('D3', 'D4')]
        result = planner.plan('D', remaining, dict(valid=True, cell=[163, 119], heading_deg=73.8))
        self.assertEqual(result['route_policy']['planned_from'], dict(cell=[163, 119], heading_deg=73.8))
        self.assertEqual({p['id'] for p in result['waypoints']}, {'D3', 'D4'})

    def test_continue_from_straight_alignment_stops_up_to_sixty_cm(self):
        planner = RegionRoutePlanner(GRAPH, CATALOG, GRAPH['map_signature'])
        checked = set()
        for region_id, region in CATALOG['regions'].items():
            reachable = planner.exit_reachable_nodes(region_id)
            for point in region['waypoints']:
                remaining = [p for p in region['waypoints'] if p['id'] != point['id']]
                for origin in terminal_nodes(point['cell']):
                    direction = NODES[origin][2]
                    reached, pending = {origin: 0}, [(0, origin)]
                    while pending:
                        travelled, index = heapq.heappop(pending)
                        if travelled != reached[index]:
                            continue
                        row = NODES[index]
                        cell = [row[0] >> 9, row[1] >> 9]
                        if index not in reachable or not inside(cell, region['bounds_image_cm']):
                            continue
                        key = (point['id'], tuple(cell), direction)
                        if key not in checked:
                            checked.add(key)
                            # Fifteen degrees matches the permitted alignment
                            # corridor, including a stopped target midway.
                            heading = (direction + 15 + 180) % 360 - 180
                            try:
                                result = planner.plan(region_id, remaining,
                                    dict(valid=True, cell=cell, heading_deg=heading))
                            except ValueError as exc:
                                raise AssertionError((key, travelled, str(exc))) from exc
                            self.assertEqual({p['id'] for p in result['waypoints']}, {p['id'] for p in remaining})
                        for neighbor, length in row[3]:
                            cost = travelled + length
                            if (neighbor in reachable and NODES[neighbor][2] == direction and cost <= 60 and
                                    cost < reached.get(neighbor, math.inf)):
                                reached[neighbor] = cost
                                heapq.heappush(pending, (cost, neighbor))
        self.assertGreater(len(checked), 200)

    def test_alignment_must_not_pass_turnoff_into_straight_dead_end(self):
        planner = RegionRoutePlanner(GRAPH, CATALOG, GRAPH['map_signature'])
        # At the end of this vertical lane the only successor has no outgoing
        # edges. A direction-compatible straight line alone is not resumable.
        reachable = planner.exit_reachable_nodes('A')
        self.assertNotIn(950, reachable)
        for heading in (75, 90, 105):
            self.assertFalse(planner.can_continue_from_pose('A', [52, 15], heading))
        self.assertTrue(planner.can_continue_from_pose('A', [52, 34], 90))
        planner.observe_map(dict(barriers=dict(valid=True, segments_mm=[[0, 0, 3520, 3200]])))
        self.assertEqual(planner.exit_reachable_nodes('A'), frozenset())
        self.assertFalse(planner.can_continue_from_pose('A', [52, 34], 90))


if __name__ == '__main__':
    unittest.main()
