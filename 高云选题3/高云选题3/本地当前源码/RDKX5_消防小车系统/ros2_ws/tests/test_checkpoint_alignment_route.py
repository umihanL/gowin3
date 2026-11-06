"""Verify road proposals, not physical travel or image-based depth claims."""
import hashlib
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision.checkpoint_alignment_route import forward_road_proposal, _quantized_stop_has_continuation
from rescue_vision.region_route import RegionRoutePlanner


def straight():
    catalog = {'map_sha256': 'a' * 64, 'regions': {'A': {'bounds_image_cm': [0, 0, 176, 160]}}}
    # Extend the road past the region's formal exit: a short isolated line
    # cannot prove the vehicle can resume the region task after an early stop.
    nodes = [[(51 + i * 2) * 256, 51 * 256, 0, [[i + 1, 2]] if i < 80 else []]
             for i in range(81)]
    document = dict(map_sha256='a' * 64, map_signature=7, directed_nodes=nodes,
                    directed_nodes_sha256=hashlib.sha256(json.dumps(nodes, separators=(',', ':')).encode()).hexdigest())
    return RegionRoutePlanner(document, catalog, 7)


class AlignmentRoadTests(unittest.TestCase):
    def test_actual_a3_forward_search_stops_before_directed_dead_end(self):
        catalog = json.loads((ROOT / 'config/semantic_regions.json').read_text(encoding='utf-8'))
        data = json.loads((ROOT / 'config/region_traversability.json').read_text(encoding='utf-8'))
        planner = RegionRoutePlanner(data, catalog, data['map_signature'])
        self.assertNotIn(950, planner.exit_reachable_nodes('A'))
        self.assertFalse(planner.can_continue_from_pose('A', [52,15], 90))
        proposal = forward_road_proposal(planner, 'A',
            dict(valid=True, cell=[52,34], heading_deg=90.), 100.)
        self.assertTrue(proposal['valid'])
        self.assertGreaterEqual(proposal['goal_cell'][1], 21)
        for y in range(proposal['goal_cell'][1], 35):
            for heading in (75., 90., 105.):
                self.assertTrue(planner.can_continue_from_pose('A', [52,y], heading), (y, heading))
        next_leg = forward_road_proposal(planner, 'A',
            dict(valid=True, cell=proposal['goal_cell'], heading_deg=90.), 101.)
        self.assertFalse(next_leg['valid'])
        self.assertEqual(next_leg['reason'], 'checkpoint_alignment_no_straight_road_in_region')

    def test_fast_quantized_seed_witness_matches_fpga_fallback_rules(self):
        catalog = json.loads((ROOT / 'config/semantic_regions.json').read_text(encoding='utf-8'))
        data = json.loads((ROOT / 'config/region_traversability.json').read_text(encoding='utf-8'))
        planner = RegionRoutePlanner(data, catalog, data['map_signature'])
        blocked = [False] * len(planner.nodes)
        for index in range(0, len(planner.nodes), 37):
            row = planner.nodes[index]
            if row[2] == 255:
                continue
            self.assertTrue(_quantized_stop_has_continuation(row, row[2], 15.))
            for delta in (-15., 0., 15.):
                heading = (row[2] + delta + 180.) % 360. - 180.
                _, fallback = planner._seeds([row[0] >> 9, row[1] >> 9], heading, blocked)
                self.assertIn(index, [node for node, _ in fallback])

    def test_actual_rom_quantized_goal_does_not_snap_to_opposite_junction_branch(self):
        catalog = json.loads((ROOT / 'config/semantic_regions.json').read_text(encoding='utf-8'))
        data = json.loads((ROOT / 'config/region_traversability.json').read_text(encoding='utf-8'))
        planner = RegionRoutePlanner(data, catalog, data['map_signature'])
        proposal = forward_road_proposal(planner, 'A',
            dict(valid=True, cell=[76, 13], heading_deg=180.), 100.)
        self.assertTrue(proposal['valid'])
        self.assertNotEqual(proposal['goal_cell'], [61, 13])
        gx, gy = (value * 512 + 256 for value in proposal['goal_cell'])
        nearest = min(planner.nodes, key=lambda row: abs(row[0] - gx) + abs(row[1] - gy))
        aliases = [row for row in planner.nodes if row[:2] == nearest[:2]]
        self.assertTrue(any(abs((row[2] - 180 + 180) % 360 - 180) <= 15 for row in aliases))
        self.assertLessEqual(proposal['path_length_cm'], 30.)

    def test_forward_goal_is_bounded_and_carries_start_guard(self):
        planner = straight()
        value = forward_road_proposal(planner, 'A', {'valid': True, 'cell': [25, 25], 'heading_deg': 0}, 10.)
        self.assertTrue(value['valid'])
        self.assertEqual(value['goal_cell'], [40, 25])
        self.assertEqual(value['path_length_cm'], 30.)
        self.assertEqual(value['alignment_guard']['speed_mm_s'], 60)
        self.assertEqual(value['alignment_guard']['start_cell'], [25, 25])

    def test_remaining_budget_limits_next_leg(self):
        value = forward_road_proposal(straight(), 'A', {'valid': True, 'cell': [25, 25], 'heading_deg': 0}, 10., 18.)
        self.assertTrue(value['valid'])
        self.assertEqual(value['path_length_cm'], 18.)
        value = forward_road_proposal(straight(), 'A', {'valid': True, 'cell': [25, 25], 'heading_deg': 0}, 10., 8.)
        self.assertFalse(value['valid'])

    def test_wrong_heading_is_not_rewritten(self):
        value = forward_road_proposal(straight(), 'A', {'valid': True, 'cell': [25, 25], 'heading_deg': 180}, 10.)
        self.assertFalse(value['valid'])

    def test_obstacle_or_region_boundary_prevents_blind_advance(self):
        planner = straight()
        planner.observe_map({'barriers': {'valid': True, 'segments_mm': [[650, 500, 650, 520]]}})
        value = forward_road_proposal(planner, 'A', {'valid': True, 'cell': [25, 25], 'heading_deg': 0}, 10.)
        self.assertFalse(value['valid'])
        planner = straight()
        planner.catalog['regions']['A']['bounds_image_cm'] = [0, 0, 60, 160]
        value = forward_road_proposal(planner, 'A', {'valid': True, 'cell': [25, 25], 'heading_deg': 0}, 10.)
        self.assertFalse(value['valid'])

    def test_detour_and_reversing_edge_cannot_be_used_for_alignment(self):
        planner = straight()
        planner.nodes[4][2] = 90
        value = forward_road_proposal(planner, 'A', {'valid': True, 'cell': [25, 25], 'heading_deg': 0}, 10.)
        self.assertFalse(value['valid'])

    def test_invalid_or_missing_pose_does_not_propose_motion(self):
        for ref in (None, {}, {'valid': True, 'cell': [25, 25], 'heading_deg': float('nan')},
                    {'valid': True, 'cell': [-4, 25], 'heading_deg': 0}):
            self.assertFalse(forward_road_proposal(straight(), 'A', ref, 10.)['valid'])


if __name__ == '__main__':
    unittest.main()
