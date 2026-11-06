"""Pure tests for timed traffic stop and human-facing flame ordinals."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.traffic import TrafficController
from rescue_vision.fire_targets import FireTargetRegistry


def observation(t, color='green'):
    return dict(capture_time=t, frame_id=str(t), status=color, confidence=.9,
                fixture_bbox=[140,60,180,160])


class DelayTrafficTests(unittest.TestCase):
    def create(self):
        c = TrafficController({'stop_policy':'delay', 'stop_delay_s':2.})
        c.on_crosswalk('cross1',10.)
        return c

    def test_exact_delay_no_feedback_needed_or_shortcut_from_feedback(self):
        for stopped in (False, True):
            c = self.create()
            self.assertEqual(c.tick(11.999, vehicle_stopped=stopped, gimbal_ready=True)['state'], 'WAIT_STOP')
            out = c.tick(12., vehicle_stopped=stopped, gimbal_ready=True)
            self.assertEqual(out['state'], 'SCANNING')
            self.assertTrue(out['stop_wait_complete'])
            self.assertFalse(out['move_allowed'])

    def test_green_requires_new_settled_frames_then_red_revokes_and_waits_again(self):
        c = self.create()
        c.tick(12., vehicle_stopped=False, gimbal_ready=True)
        for i in range(7):
            t = 12.31+i*.1
            out = c.tick(t, observation(t), vehicle_stopped=False, gimbal_ready=True)
        self.assertTrue(out['move_allowed'])
        out = c.tick(13.02, observation(13.02,'red'), vehicle_stopped=False, gimbal_ready=True)
        self.assertFalse(out['move_allowed'])
        out = c.tick(13.03, vehicle_stopped=True, gimbal_ready=True)
        self.assertEqual(out['state'], 'WAIT_STOP')
        self.assertFalse(out['stop_wait_complete'])
        self.assertFalse(c.tick(15.02, gimbal_ready=True)['stop_wait_complete'])
        self.assertTrue(c.tick(15.03, gimbal_ready=True)['stop_wait_complete'])


class FlameOrdinalTests(unittest.TestCase):
    def setUp(self):
        self.registry = FireTargetRegistry()
        self.registry.set_enabled(True)

    def update(self, boxes, sequence):
        self.registry.update([dict(class_id=0, confidence=.9, bbox_xyxy=box) for box in boxes],
            stamp_ns=sequence*100_000_000, sequence=sequence, frame_id='physical_left',
            size=(640,480), now=sequence*.1)

    def select(self, **fields):
        return self.registry.select(dict(command_id='select', issued_at=50., ttl_s=1., **fields),
                                    now=.2, ros_now=50.1)

    def test_second_means_left_to_right_not_persistent_id(self):
        boxes = [[410,100,450,150], [100,100,140,150], [260,100,300,150]]
        self.update(boxes, 1)
        self.update(list(reversed(boxes)), 2)
        self.assertTrue(self.select(ordinal=2)[0])
        self.assertEqual(self.registry.selected, 3)
        listed = self.registry.snapshot(.2)[0]['targets']
        self.assertEqual([t['ordinal'] for t in listed], [1,2,3])
        self.assertEqual([t['target_id'] for t in listed], [2,3,1])

    def test_ordinal_outside_view_or_mixed_with_id_does_not_guess(self):
        self.update([[100,100,140,150]], 1)
        self.update([[100,100,140,150]], 2)
        self.assertFalse(self.select(ordinal=2)[0])
        self.assertIsNone(self.registry.selected)
        self.assertFalse(self.registry.select(dict(command_id='mixed', issued_at=50., ttl_s=1.,
            ordinal=1, target_id=1), now=.2, ros_now=50.1)[0])

    def test_tied_horizontal_position_is_ambiguous(self):
        boxes = [[100,80,140,120], [100,260,140,300]]
        self.update(boxes, 1)
        self.update(boxes, 2)
        self.assertFalse(self.select(ordinal=1)[0])


if __name__ == '__main__':
    unittest.main(verbosity=2)
