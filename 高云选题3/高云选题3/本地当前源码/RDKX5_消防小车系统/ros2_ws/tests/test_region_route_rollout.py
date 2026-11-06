"""Complete offline region FSM runs against an independent FPGA-ROM leg solver."""
import heapq
import json
import math
from pathlib import Path
import time
import unittest

from test_region_route import CATALOG, GRAPH, ROOT
from test_region_search import frame, actuator
from rescue_vision.region_route import RegionRoutePlanner
from rescue_vision.region_search import RegionSearch


class TimedPlanner(RegionRoutePlanner):
    def __init__(self):
        super().__init__(GRAPH, CATALOG, GRAPH['map_signature'])
        self.elapsed = []

    def plan(self, *args):
        started = time.perf_counter()
        try:
            return super().plan(*args)
        finally:
            self.elapsed.append(1000 * (time.perf_counter() - started))


def fpga_leg(nodes, start, heading, goal, bounds):
    """Shortest single leg, not the X5 all-stop bitmask scheduler.

    This is synthetic road feedback for local regression, not hardware proof.
    Incoming final segment defines the simulated measured heading.
    """
    x, y = [v * 512 + 256 for v in start]
    gx, gy = [v * 512 + 256 for v in goal]
    nearest_start = min(range(len(nodes)), key=lambda i: abs(nodes[i][0] - x) + abs(nodes[i][1] - y))
    nearest_goal = min(range(len(nodes)), key=lambda i: abs(nodes[i][0] - gx) + abs(nodes[i][1] - gy))
    goal_xy = nodes[nearest_goal][:2]
    for fallback in (False, True):
        distance, parent, queue = {}, {}, []
        for index, row in enumerate(nodes):
            if row[2] == 255 or abs((row[2] - heading + 180) % 360 - 180) > 35:
                continue
            separation = abs(row[0] - x) + abs(row[1] - y)
            forward = (row[0] >= x - 512 if -45 <= heading <= 45 else
                       row[1] <= y + 512 if 45 < heading < 135 else
                       row[0] <= x + 512 if heading >= 135 or heading <= -135 else row[1] >= y - 512)
            if (not fallback and row[:2] != nodes[nearest_start][:2]) or (fallback and (separation > 1536 or not forward)):
                continue
            cost = math.ceil(separation / 256.) if fallback else 0
            distance[index], parent[index] = cost, None
            heapq.heappush(queue, (cost, index))
        end = None
        while queue:
            cost, index = heapq.heappop(queue)
            if distance.get(index) != cost:
                continue
            row = nodes[index]
            if abs(row[0] - goal_xy[0]) + abs(row[1] - goal_xy[1]) <= 1536:
                end = index
                break
            outside = bounds is not None and not (bounds[0] * 512 <= row[0] < (bounds[2] + 1) * 512 and
                                                   bounds[1] * 512 <= row[1] < (bounds[3] + 1) * 512)
            for neighbor, length in row[3]:
                score = cost + length * (33 if outside else 1)
                if score < distance.get(neighbor, math.inf):
                    distance[neighbor], parent[neighbor] = score, index
                    heapq.heappush(queue, (score, neighbor))
        if end is not None:
            trace, current = [], end
            while current is not None:
                trace.append(current)
                current = parent[current]
            final = nodes[end]
            previous = next((nodes[i] for i in trace[1:] if nodes[i][:2] != final[:2]), None)
            if previous is None:
                raise AssertionError('zero_length_fpga_route')
            angle = math.degrees(math.atan2(previous[1] - final[1], final[0] - previous[0]))
            return [final[0] >> 9, final[1] >> 9], angle, len(trace)
    raise AssertionError('fpga_directed_leg_unreachable')


def run_all():
    records, all_timings = [], []
    for region, data in CATALOG['regions'].items():
        for point in data['waypoints']:
            x, y = [v * 512 + 256 for v in point['cell']]
            nearest = min(range(len(GRAPH['directed_nodes'])),
                          key=lambda i: abs(GRAPH['directed_nodes'][i][0] - x) + abs(GRAPH['directed_nodes'][i][1] - y))
            xy = GRAPH['directed_nodes'][nearest][:2]
            headings = sorted({row[2] for row in GRAPH['directed_nodes'] if row[:2] == xy and row[2] != 255})
            for heading in headings:
                planner, now = TimedPlanner(), 100.
                search = RegionSearch(CATALOG, planner)
                position, angle = point['cell'], heading
                effects = search.start(dict(action='search_region', region_id=region, execute_fire=False,
                    command_id='rollout-' + point['id'] + '-' + str(heading)), now,
                    dict(valid=True, cell=position, heading_deg=angle))
                legs = []
                while search.active and len(legs) < 8:
                    request = next(value for key, value in effects if key == 'navigate')
                    assert request['goal_cell'] != position, (point['id'], heading, request)
                    position, angle, samples = fpga_leg(planner.nodes, position, angle,
                                                       request['goal_cell'], request.get('region_preference_cells'))
                    legs.append(dict(kind=request['leg_kind'], waypoint=request['waypoint_id'],
                                     goal=request['goal_cell'], pose=position, heading=round(angle, 3), samples=samples))
                    effects = []
                    while search.active and not any(key == 'navigate' for key, value in effects):
                        now += .2
                        navigation = dict(stamp=now, command_id=search.leg_id, phase='arrived', stopped=True,
                            pose_valid=True, pose_cell=position, pose_heading_deg=angle, arrived=True)
                        effects = search.tick(now, navigation, frame(now), {}, actuator(now))
                        if now > 130:
                            raise AssertionError(('FSM_stall', search.snapshot(now)))
                assert search.phase == 'COMPLETE', search.snapshot(now)
                assert search.reason == 'region_search_exited_and_stopped', search.snapshot(now)
                assert len(search.visited) == len(set(search.visited)) == 4, search.snapshot(now)
                assert len(legs) == 5 and legs[-1]['kind'] == 'exit', legs
                all_timings.extend(planner.elapsed)
                records.append(dict(region=region, start=point['id'], heading=heading, legs=legs,
                    visited=search.visited, exit=search.exit, max_plan_ms=round(max(planner.elapsed), 3)))
    return dict(simulated_feedback_only=True, graph_nodes=len(GRAPH['directed_nodes']), scenarios=len(records),
                planning_calls=len(all_timings), max_plan_ms=round(max(all_timings), 3),
                mean_plan_ms=round(sum(all_timings) / len(all_timings), 3), runs=records)


class FullRegionRolloutTests(unittest.TestCase):
    def test_all_waypoints_and_each_legal_heading_complete_without_oscillation(self):
        report = run_all()
        self.assertGreaterEqual(report['scenarios'], 32)
        self.assertTrue(all(len(run['legs']) == 5 for run in report['runs']))


if __name__ == '__main__':
    report = run_all()
    destination = ROOT / 'reports/region_route_full_angle_20261103/region_rollout.json'
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    print(json.dumps({key: value for key, value in report.items() if key != 'runs'}))
