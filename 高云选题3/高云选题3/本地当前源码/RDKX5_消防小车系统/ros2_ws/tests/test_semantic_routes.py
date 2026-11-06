import hashlib
import json
from pathlib import Path
import sys
import unittest

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS/'src/rescue_vision'))
from rescue_vision.semantic_routes import build_junction_route


class SemanticRoutesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.map = json.loads((WS/'config/extension_map.json').read_text(encoding='utf8'))
        cls.pose = dict(x_cm=104.22, y_cm=40., heading_deg=90.)

    def test_actual_t_junction_sides_and_order(self):
        # The actual map alternates the branch side at j, i and h.
        cases = [(1, 'left', ['j'], 'f'), (2, 'right', ['j', 'i'], 's0'),
                 (3, 'left', ['j', 'i', 'h'], 'g')]
        for n, turn, expected, exit_vertex in cases:
            with self.subTest(n=n):
                route = build_junction_route(self.map, self.pose, n, turn)
                self.assertEqual(route['junctions'], expected)
                self.assertEqual(route['exit_toward'], exit_vertex)
                self.assertEqual(route['start'], [52, 140])
                self.assertLess(route['waypoints'][0][1], route['start'][1]-6)
                self.assertLessEqual(len(route['waypoints']), 8)
                for x, y in [route['start'], route['goal']]+route['waypoints']:
                    self.assertTrue(0 <= x < 176 and 0 <= y < 160)

    def test_missing_side_is_rejected_without_other_route(self):
        for n, turn in [(1, 'right'), (2, 'left'), (3, 'right')]:
            with self.subTest(n=n), self.assertRaisesRegex(ValueError, 'turn_not_present'):
                build_junction_route(self.map, self.pose, n, turn)

    def test_straight_instruction_does_not_follow_corner_and_count_again(self):
        with self.assertRaisesRegex(ValueError, 'straight_road_ends'):
            build_junction_route(self.map, self.pose, 5, 'left')

    def test_left_right_for_south_facing_vehicle(self):
        pose = dict(x_cm=104.22, y_cm=260, heading_deg=-90)
        route = build_junction_route(self.map, pose, 1, 'right')
        self.assertEqual(route['target_junction'], 'h')
        self.assertEqual(route['exit_toward'], 'g')

    def test_left_right_for_east_and_west(self):
        east = dict(x_cm=145, y_cm=293.37, heading_deg=0)
        west = dict(x_cm=210, y_cm=293.37, heading_deg=180)
        self.assertEqual(build_junction_route(self.map, east, 1, 'right')['exit_toward'], 'k')
        self.assertEqual(build_junction_route(self.map, west, 1, 'left')['exit_toward'], 'h')

    def test_start_must_be_on_road_and_heading_initialized(self):
        for pose in [dict(x_cm=175, y_cm=160, heading_deg=90),
                     dict(x_cm=104.22, y_cm=40, heading_deg=0),
                     dict(x_cm=104.22, y_cm=40, heading_deg=float('nan')),
                     dict(x_cm=104.22, y_cm=40),
                     dict(x_cm=True, y_cm=40, heading_deg=90)]:
            with self.subTest(pose=pose), self.assertRaises(ValueError):
                build_junction_route(self.map, pose, 1, 'left')

    def test_start_inside_turn_entry_is_not_accepted(self):
        with self.assertRaisesRegex(ValueError, 'start_too_close'):
            build_junction_route(self.map, dict(x_cm=104.22, y_cm=90, heading_deg=90), 1, 'left')

    def test_map_matches_fpga_source_if_present(self):
        source = WS.parent.parent/'ov5640/src/navigation/road_centerlines.json'
        if not source.exists():
            self.skipTest('FPGA source is not in this deployment')
        self.assertEqual(self.map['map_sha256'], hashlib.sha256(source.read_bytes()).hexdigest())

    def test_input_limits(self):
        for n, direction in [(0, 'left'), (9, 'right'), (True, 'left'), (1.0, 'left'), (1, 'back')]:
            with self.subTest(n=n), self.assertRaises(ValueError):
                build_junction_route(self.map, self.pose, n, direction)


if __name__ == '__main__':
    unittest.main()
