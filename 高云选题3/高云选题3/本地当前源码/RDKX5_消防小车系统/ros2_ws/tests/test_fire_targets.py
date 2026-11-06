"""Explicit target identity contracts; no camera or actuator access."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.fire_targets import FireTargetRegistry, SelectionHistory, selected_detection


def fire(x, y=100, width=40):
    return dict(class_id=0, confidence=.9, bbox_xyxy=[x, y, x+width, y+50])


class RegistryTests(unittest.TestCase):
    def setUp(self):
        self.registry = FireTargetRegistry()
        self.registry.set_enabled(True)

    def update(self, boxes, number=1, **kwargs):
        self.registry.update(boxes, stamp_ns=100_000_000_000+number*50_000_000,
            sequence=number, frame_id=kwargs.get('frame', 'undistorted_left_optical'),
            size=kwargs.get('size', (1280, 1088)), now=number*.05)

    def select(self, **kwargs):
        return self.registry.select(dict(command_id='command1', issued_at=100., ttl_s=1., **kwargs),
                                    now=.1, ros_now=100.1)

    def selected(self):
        return self.registry.snapshot(.1)[1]

    def test_stable_ids_survive_reordering_and_small_motion(self):
        self.update([fire(100), fire(400)])
        self.update([fire(402), fire(102)], 2)
        self.assertEqual(set(self.registry.tracks), {1, 2})
        self.assertEqual(self.registry.tracks[1]['bbox_xyxy'][0], 102)
        self.assertTrue(self.select(target_id=2)[0])
        self.assertEqual(self.selected()['target_id'], 2)

    def test_fresh_id_requires_two_observations_and_only_avoids_guessing(self):
        self.update([fire(100)])
        self.assertFalse(self.select(selector='only')[0])
        self.update([fire(100), fire(400)], 2)
        self.update([fire(100), fire(400)], 3)
        result = self.registry.select(dict(command_id='c2', issued_at=100., ttl_s=1., selector='only'),
                                      now=.15, ros_now=100.2)
        self.assertFalse(result[0])

    def test_lost_selected_target_never_transfers_to_new_target(self):
        self.update([fire(100), fire(400)])
        self.update([fire(100), fire(400)], 2)
        self.assertTrue(self.select(target_id=1)[0])
        generation = self.registry.generation
        self.update([fire(400)], 3)
        self.assertIsNone(self.registry.selected)
        self.assertGreater(self.registry.generation, generation)
        self.update([fire(100), fire(400)], 4)
        self.assertNotIn(1, self.registry.tracks)
        self.assertIn(3, self.registry.tracks)
        self.assertIsNone(self.registry.selected)

    def test_crossing_ambiguous_tracks_retired_not_nearest_reassigned(self):
        self.update([fire(100), fire(175)])
        self.update([fire(100), fire(175)], 2)
        self.assertTrue(self.select(target_id=1)[0])
        self.update([fire(128), fire(147)], 3)
        self.assertIsNone(self.registry.selected)
        self.assertNotIn(1, self.registry.tracks)
        self.assertNotIn(2, self.registry.tracks)

    def test_duplicate_frame_does_not_extend_lease(self):
        self.update([fire(100)]); self.update([fire(100)], 2)
        self.assertTrue(self.select(target_id=1)[0])
        self.update([fire(100)], 2)
        self.assertFalse(self.registry.snapshot(.61)[1]['valid'])

    def test_restart_geometry_and_mode_clear_selection(self):
        for action in ('restart', 'geometry', 'mode'):
            self.setUp()
            self.update([fire(100)]); self.update([fire(100)], 2)
            self.assertTrue(self.select(target_id=1)[0])
            if action == 'restart': self.update([fire(100)], 1)
            elif action == 'geometry': self.update([fire(100)], 3, frame='different_eye')
            else: self.registry.set_enabled(False)
            self.assertIsNone(self.registry.selected)

    def test_commands_expire_and_cannot_be_replayed(self):
        self.update([fire(100)]); self.update([fire(100)], 2)
        self.assertTrue(self.select(target_id=1)[0])
        generation = self.registry.generation
        self.assertFalse(self.select(target_id=1)[0])
        self.assertEqual(self.registry.generation, generation)
        self.assertFalse(self.registry.select(dict(command_id='expired', target_id=1,
            issued_at=90., ttl_s=1.), now=.1, ros_now=100.)[0])

    def test_selector_tie_fails_and_largest_uses_area(self):
        self.update([fire(100, width=60), fire(400)])
        self.update([fire(100, width=60), fire(400)], 2)
        self.assertTrue(self.select(selector='largest')[0])
        self.assertEqual(self.registry.selected, 1)

    def test_fresh_empty_frame_is_healthy_without_claiming_any_fire(self):
        self.update([])
        targets, selection = self.registry.snapshot(.05)
        self.assertTrue(targets['frame_valid'])
        self.assertFalse(targets['valid'])
        self.assertEqual(targets['targets'], [])
        self.assertFalse(selection['valid'])
        self.assertEqual(targets['reason'], 'explicit_selection_required')

    def test_frame_health_clears_on_disabled_unavailable_stale_and_malformed_input(self):
        for failure in ('disabled', 'unavailable', 'expired', 'dimensions', 'detections', 'capture_key'):
            with self.subTest(failure=failure):
                self.setUp(); self.update([])
                if failure == 'disabled':
                    self.registry.set_enabled(False)
                elif failure == 'expired':
                    self.registry.expire(.7)
                else:
                    args=dict(detections=[], stamp_ns=100_100_000_000, sequence=2,
                              frame_id='undistorted_left_optical',size=(1280,1088),now=.1)
                    args.update({'unavailable':dict(available=False), 'dimensions':dict(size=None),
                        'detections':dict(detections=[None]), 'capture_key':dict(sequence=True)}[failure])
                    self.registry.update(**args)
                self.assertFalse(self.registry.snapshot(.1)[0]['frame_valid'])

    def test_duplicate_empty_frame_cannot_extend_frame_health_and_new_frame_recovers(self):
        self.update([], 1)
        self.registry.update([],stamp_ns=100_050_000_000,sequence=1,
            frame_id='undistorted_left_optical',size=(1280,1088),now=.54)
        self.assertFalse(self.registry.snapshot(.56)[0]['frame_valid'])
        self.update([], 12)
        self.assertTrue(self.registry.snapshot(.601)[0]['frame_valid'])

    def test_target_loss_does_not_turn_healthy_empty_frame_into_camera_failure(self):
        self.update([fire(100)], 1); self.update([fire(100)], 2)
        self.assertTrue(self.select(target_id=1)[0])
        self.update([], 3)
        targets, selection=self.registry.snapshot(.151)
        self.assertEqual(targets['reason'],'selected_target_lost')
        self.assertTrue(targets['frame_valid'])
        self.assertFalse(targets['valid'])
        self.assertFalse(selection['valid'])


class SelectionHistoryTests(unittest.TestCase):
    def selection(self, index=1, generation=1, target=3):
        return dict(valid=True, target_id=target, selection_generation=generation,
            registry_session='registry1', capture_stamp_ns=100_000_000_000+index,
            sequence=index, coordinate_frame='undistorted_left_optical',
            image_size_wh=[1280, 1088], physical_eye='left', bbox_xyxy=[100, 100, 140, 150])

    def lookup(self, history, document, **kwargs):
        values = dict(stamp_ns=document['capture_stamp_ns'], sequence=document['sequence'],
            frame_id=document['coordinate_frame'], size=document['image_size_wh'], now=.3)
        values.update(kwargs)
        return history.lookup(**values)

    def test_keeps_capture_history_for_slow_depth_and_enforces_frame(self):
        history = SelectionHistory()
        first = self.selection()
        history.observe(first, .1)
        for index in range(2, 8): history.observe(self.selection(index), .2)
        self.assertEqual(self.lookup(history, first)['target_id'], 3)
        self.assertIsNone(self.lookup(history, first, sequence=99))
        self.assertIsNone(self.lookup(history, first, frame_id='right'))
        self.assertIsNone(self.lookup(history, first, size=(640, 480)))
        self.assertIsNone(self.lookup(history, first, now=.61))

    def test_selection_change_and_loss_invalidate_old_capture_history(self):
        history = SelectionHistory()
        first = self.selection(); history.observe(first, .1)
        history.observe(self.selection(2, generation=2, target=4), .2)
        self.assertIsNone(self.lookup(history, first))
        history.observe(dict(valid=False), .3)
        self.assertIsNone(history.identity)
        self.assertFalse(history.frames)

    def test_timer_duplicates_cannot_refresh_stale_selection(self):
        history = SelectionHistory(); value = self.selection()
        history.observe(value, .1); history.observe(value, .49)
        self.assertIsNone(self.lookup(history, value, now=.61))

    def test_old_generation_or_retired_registry_cannot_restore_selection(self):
        history = SelectionHistory(); old = self.selection()
        history.observe(old, .1)
        new = self.selection(2, generation=2, target=4)
        history.observe(new, .2)
        history.observe(old, .3)
        self.assertEqual(history.identity, ('registry1', 2, 4))
        restarted = dict(new, registry_session='registry2')
        history.observe(restarted, .3)
        history.observe(new, .4)
        self.assertEqual(history.identity, ('registry2', 2, 4))

    def test_history_is_bounded_to_32_frames(self):
        history = SelectionHistory()
        for number in range(40): history.observe(self.selection(number), .1)
        self.assertEqual(len(history.frames), 32)
        self.assertIsNone(self.lookup(history, self.selection(1)))

    def test_detection_must_be_same_bbox_not_nearby_flame(self):
        first, second = NS(**fire(100)), NS(**fire(400))
        selected = self.selection()
        self.assertIs(selected_detection([first, second], selected), first)
        self.assertIsNone(selected_detection([second], selected))
        self.assertIsNone(selected_detection([first, first], selected))


from test_fire_aim_geometry import FireAimCallbackTests


class SelectedAimCallbackTests(FireAimCallbackTests):
    def setUp(self):
        super().setUp()
        from collections import OrderedDict
        self.node.require_selection = True
        self.node.selection_history = SelectionHistory()
        self.node.pending_detections = OrderedDict()
        self.node.target_identity = None

    def choose(self, number, target_id=7, generation=1, box=None):
        frame = self.frame(number)
        document = dict(valid=True, target_id=target_id, selection_generation=generation,
            registry_session='test_registry', capture_stamp_ns=100_000_000_000+number*100_000_000,
            sequence=number, coordinate_frame=frame.header.frame_id, image_size_wh=[1280,1088],
            physical_eye='left', bbox_xyxy=box or frame.detections[0].bbox_xyxy)
        self.node.selection(NS(data=json.dumps(document)))

    def test_target_switch_clears_previous_lock(self):
        for number in range(7):
            self.node.test_now = 100.+number*.1; self.node.mission_received = number*.1
            with patch.object(self.module.time, 'monotonic', return_value=number*.1):
                self.choose(number); self.node.receive(self.frame(number))
        self.assertTrue(self.node.publisher[-1]['pump_candidate'])
        self.assertEqual(self.node.publisher[-1]['target_id'], 7)
        self.node.test_now = 100.7; self.node.mission_received = .7
        with patch.object(self.module.time, 'monotonic', return_value=.7):
            self.choose(7, target_id=8, generation=2)
            self.node.receive(self.frame(7))
        result = self.node.publisher[-1]
        self.assertEqual(result['target_id'], 8)
        self.assertFalse(result['image_locked'])
        self.assertFalse(result['pump_candidate'])

    def test_missing_selection_never_falls_back_to_single_fire(self):
        with patch.object(self.module.time, 'monotonic', return_value=.1):
            self.node.receive(self.frame(0))
        self.assertFalse(self.node.publisher[-1]['valid'])
        self.assertIsNone(self.node.tracker)

    def test_selection_arriving_after_detection_still_builds_continuous_lock(self):
        for number in range(7):
            self.node.test_now = 100.+number*.1; self.node.mission_received = number*.1
            with patch.object(self.module.time, 'monotonic', return_value=number*.1):
                self.node.receive(self.frame(number)); self.choose(number)
        self.assertTrue(self.node.publisher[-1]['image_locked'])

    # Inherited compatibility cases explicitly use legacy optional-selection mode.
    def test_proposal_lock_contract_and_mission_expiration(self):
        self.node.require_selection = False
        super().test_proposal_lock_contract_and_mission_expiration()

    def test_changed_camera_info_immediately_invalidates_previous_lock(self):
        self.node.require_selection = False
        super().test_changed_camera_info_immediately_invalidates_previous_lock()


if __name__ == '__main__':
    unittest.main()
