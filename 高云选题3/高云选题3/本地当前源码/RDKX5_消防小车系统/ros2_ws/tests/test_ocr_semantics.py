from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rescue_vision'))
from rescue_vision.building_ocr import BuildingMatcher, BuildingTracker
from rescue_vision.semantics import SceneMemory

CONFIG = {'buildings': [
    {'building_id': 'A', 'number': '01', 'name': '安居楼', 'observation_point': {'id': 'observe_A'}},
    {'building_id': 'K', 'number': '11', 'name': '绿荫楼', 'observation_point': {'id': 'observe_K'}}]}


def reading(text='01安居楼', confidence=.96, x=20):
    return {'text': text, 'confidence': confidence,
            'quad': [[x, 200], [x+130, 200], [x+130, 225], [x, 225]]}


def confirmed():
    tracker = BuildingTracker(CONFIG)
    for i in range(3):
        out = tracker.update([reading()], i*.2, str(i))
    return out


class OCRSemanticsTests(unittest.TestCase):
    def test_exact_numeric_no_substring(self):
        matcher = BuildingMatcher(CONFIG)
        self.assertEqual(matcher.match([reading('11')])['candidate']['building_id'], 'K')
        self.assertEqual(matcher.match([reading('１号楼')])['candidate']['building_id'], 'A')
        self.assertEqual(matcher.match([reading('111')])['status'], 'unmatched')
        self.assertEqual(matcher.match([reading('11安居楼')])['status'], 'unmatched')

    def test_conflict_and_unreliable_ocr(self):
        matcher = BuildingMatcher(CONFIG)
        self.assertEqual(matcher.match([reading('11'), reading('安居楼')])['status'], 'ambiguous')
        self.assertEqual(matcher.match([reading(confidence=.4)])['status'], 'unmatched')
        self.assertEqual(matcher.match([reading(confidence=float('nan'))])['status'], 'unmatched')

    def test_confirm_needs_distinct_recent_frames(self):
        tracker = BuildingTracker(CONFIG)
        self.assertFalse(tracker.update([reading()], 1, 'a')['confirmed'])
        self.assertFalse(tracker.update([reading()], 1.1, 'a')['confirmed'])
        self.assertFalse(tracker.update([reading()], 1.2, 'b')['confirmed'])
        self.assertTrue(tracker.update([reading()], 1.3, 'c')['confirmed'])
        self.assertFalse(tracker.update([reading()], 5, 'd')['confirmed'])

    def test_moving_anchor_does_not_mix_views(self):
        tracker = BuildingTracker(CONFIG)
        tracker.update([reading()], 1, 'a')
        tracker.update([reading()], 1.1, 'b')
        self.assertFalse(tracker.update([reading(x=500)], 1.2, 'c')['confirmed'])

    def test_report_current_view_and_no_casualty_inference(self):
        memory = SceneMemory(CONFIG)
        detections = [{'class_name': n, 'confidence': .9, 'bbox': box}
                      for n, box in [('fire', [40, 40, 60, 70]),
                                     ('person', [70, 40, 100, 90]),
                                     ('person', [110, 40, 140, 90]),
                                     ('firefighter', [170, 40, 200, 90])]]
        report = memory.observe(confirmed(), detections, .4, '2', [0, 0, 250, 250], True)
        self.assertTrue(report['valid'])
        self.assertEqual(report['fire_status'], 'observed')
        self.assertEqual(report['visible_person_count'], 2)
        self.assertEqual(report['visible_firefighter_count'], 1)
        self.assertIsNone(report['casualty_count'])
        self.assertEqual(report['observation_scope'], 'current_view')

    def test_no_global_image_to_building_assumption(self):
        memory = SceneMemory(CONFIG)
        report = memory.observe(confirmed(), [], .4, '2')
        self.assertEqual(report['building_id'], 'A')
        self.assertEqual(report['fire_status'], 'unknown')
        self.assertIsNone(report['visible_person_count'])

    def test_wrong_frame_or_roi_not_associated(self):
        for frame, roi in [('other', [0, 0, 250, 250]), ('2', [500, 0, 750, 250])]:
            report = SceneMemory(CONFIG).observe(confirmed(), [], .4, frame, roi, True)
            self.assertFalse(report['valid'])
            self.assertEqual(report['fire_status'], 'unknown')

    def test_outside_objects_excluded_and_no_detection_not_no_fire(self):
        detections = [{'class_name': 'fire', 'confidence': .9, 'bbox': [300, 20, 350, 60]}]
        report = SceneMemory(CONFIG).observe(confirmed(), detections, .4, '2', [0, 0, 250, 250], True)
        self.assertEqual(report['fire_status'], 'not_observed')
        self.assertEqual(report['visible_person_count'], 0)
        self.assertFalse(report['physical_fire_verified'])

    def test_expired_report_not_current(self):
        memory = SceneMemory(CONFIG)
        memory.observe(confirmed(), [], .4, '2', [0, 0, 250, 250], True)
        stale = memory.current(4)
        self.assertEqual(stale['fire_status'], 'unknown')
        self.assertFalse(stale['valid'])
        self.assertIsNone(stale['visible_person_count'])

    def test_no_cross_frame_count_accumulation(self):
        memory = SceneMemory(CONFIG)
        b = confirmed()
        detections = [{'class_name': 'person', 'confidence': .9, 'bbox': [30, 30, 60, 80]}]
        memory.observe(b, detections, .4, '2', [0, 0, 250, 250], True)
        b = dict(b, stamp=.6, frame_id='3')
        report = memory.observe(b, detections, .6, '3', [0, 0, 250, 250], True)
        self.assertEqual(report['visible_person_count'], 1)

    def test_symbolic_routes_not_motor_commands(self):
        memory = SceneMemory(CONFIG)
        result = memory.resolve_request('请去11号楼观察', 1)
        self.assertEqual(result['building_id'], 'K')
        self.assertFalse(result['can_execute'])
        self.assertEqual(result['status'], 'symbolic_goal_only')
        self.assertEqual(memory.resolve_request('去1号楼绿荫楼', 2)['status'], 'ambiguous_or_unknown_goal')
        self.assertEqual(memory.resolve_request('去111号楼', 2)['status'], 'ambiguous_or_unknown_goal')


if __name__ == '__main__':
    unittest.main(verbosity=2)
