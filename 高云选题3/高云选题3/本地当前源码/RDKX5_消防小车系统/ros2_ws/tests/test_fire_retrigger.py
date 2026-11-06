"""Processed fires must not restart a stop when leaving a checkpoint."""
import unittest

from test_checkpoint_alignment import AlignmentHarness, view
from rescue_vision.fire_retrigger import FireRetriggerGuard


class FireRetriggerGuardTests(unittest.TestCase):
    @staticmethod
    def context(stamp, **changes):
        return dict(dict(stamp=stamp, valid=True, status_fresh=True, pose_valid=True,
            retained_route=True, command_id='patrol-1', task_id=7, route_generation=2,
            pose_cell=[10, 20], pose_heading_deg=0., route_progress_mm=100.), **changes)

    @staticmethod
    def context_view(stamp, targets):
        return dict(view(stamp, targets), coordinate_frame='left',
                    capture_stamp_ns=round(stamp*1e9), sequence=round(stamp*1000))

    def test_same_view_gap_requires_three_exposures_and_does_not_block_distinct_fire(self):
        g = FireRetriggerGuard()
        g.observe(self.context_view(100., ((1, .5),)), 100., context=self.context(100.))
        g.mark_processed(('camera-session', 1), 100.)
        for stamp in (101., 101.1):
            g.observe(self.context_view(stamp, ((9, .5), (10, .8))), stamp, context=self.context(stamp))
            self.assertNotIn(('camera-session', 9), g.processed)
        g.observe(self.context_view(101.2, ((9, .5), (10, .8))), 101.2, context=self.context(101.2))
        self.assertFalse(g.allows(('camera-session', 9), 103.1))
        self.assertTrue(g.allows(('camera-session', 10), 103.1))
        self.assertEqual(g.snapshot(103.1)['same_view_alias_count'], 1)
        self.assertEqual(g.snapshot(103.1)['cooldown_s'], 3.)

    def test_skipped_lost_target_uses_its_last_observed_context_without_claiming_completed(self):
        g = FireRetriggerGuard()
        g.observe(self.context_view(100., ((1, .5),)), 100., context=self.context(100.))
        g.observe(self.context_view(100.8, ()), 100.8, context=self.context(100.8))
        g.mark_processed(('camera-session', 1), 100.8, status='skipped')
        for stamp in (101., 101.1, 101.2):
            g.observe(self.context_view(stamp, ((9, .5),)), stamp, context=self.context(stamp))
        self.assertFalse(g.allows(('camera-session', 9), 104.))
        self.assertEqual(g.snapshot(104.)['skipped_identity_count'], 2)
        self.assertEqual(g.snapshot(104.)['completed_identity_count'], 0)

    def test_long_alias_rejects_changed_route_pose_progress_camera_and_ambiguous_or_queued_fire(self):
        for changes, targets, exempt, coordinate in (
            ({'route_generation': 3}, ((9, .5),), (), 'left'),
            ({'command_id': 'patrol-2'}, ((9, .5),), (), 'left'),
            ({'pose_cell': [13, 20]}, ((9, .5),), (), 'left'),
            ({'pose_heading_deg': 10.}, ((9, .5),), (), 'left'),
            ({'route_progress_mm': 200.}, ((9, .5),), (), 'left'),
            ({'pose_valid': False}, ((9, .5),), (), 'left'),
            ({}, ((9, .495), (10, .505)), (), 'left'),
            ({}, ((9, .5),), (('camera-session', 9),), 'left'),
            ({}, ((9, .5),), (), 'right')):
            with self.subTest(changes=changes, targets=targets, exempt=exempt, coordinate=coordinate):
                g = FireRetriggerGuard()
                g.observe(self.context_view(100., ((1, .5),)), 100., context=self.context(100.))
                g.mark_processed(('camera-session', 1), 100.)
                for stamp in (101., 101.1, 101.2):
                    doc = dict(self.context_view(stamp, targets), coordinate_frame=coordinate)
                    g.observe(doc, stamp, exempt=exempt, context=self.context(stamp, **changes))
                self.assertTrue(g.allows(('camera-session', 9), 104.))
                self.assertEqual(g.same_view_alias_count, 0)

    def test_same_view_memory_expires_and_duplicate_exposure_cannot_gain_votes(self):
        for stamps in ((104.1, 104.2, 104.3), (101., 101.1, 101.2)):
            g = FireRetriggerGuard()
            g.observe(self.context_view(100., ((1, .5),)), 100., context=self.context(100.))
            g.mark_processed(('camera-session', 1), 100.)
            for stamp in stamps:
                doc = self.context_view(stamp, ((9, .5),))
                if stamp < 104.: doc.update(capture_stamp_ns=101000000000, sequence=101000)
                g.observe(doc, stamp, context=self.context(stamp))
            self.assertTrue(g.allows(('camera-session', 9), 105.))
            self.assertEqual(g.same_view_alias_count, 0)

    def test_three_second_cooldown_then_new_target_allowed_processed_id_still_blocked(self):
        guard = FireRetriggerGuard()
        guard.observe(view(100., ((1, .5), (2, .8))), 100.)
        guard.mark_processed(('camera-session', 1), 100.)
        self.assertFalse(guard.allows(('camera-session', 2), 102.99))
        self.assertTrue(guard.allows(('camera-session', 2), 103.))
        self.assertFalse(guard.allows(('camera-session', 1), 500.))

    def test_other_flame_already_in_queue_remains_allowed(self):
        guard = FireRetriggerGuard()
        guard.observe(view(100., ((1, .5), (2, .53))), 100.)
        guard.mark_processed(('camera-session', 1), 100.)
        guard.observe(view(100.2, ((2, .53),)), 100.2, exempt={('camera-session', 2)})
        self.assertTrue(guard.allows(('camera-session', 2), 100.2))
        self.assertEqual(guard.alias_count, 0)
        self.assertFalse(guard.allows(('camera-session', 1), 100.2))

    def test_short_overlapping_track_id_change_is_suppressed_after_cooldown(self):
        guard = FireRetriggerGuard()
        guard.observe(view(100., ((1, .5),)), 100.)
        guard.mark_processed(('camera-session', 1), 100.)
        guard.observe(view(100.2, ((9, .495),)), 100.2)
        self.assertFalse(guard.allows(('camera-session', 9), 104.))
        self.assertEqual(guard.alias_count, 1)

    def test_long_gap_new_session_different_box_or_ambiguous_overlap_not_aliased(self):
        for stamp, targets, session in ((101., ((9, .5),), 'camera-session'),
                (100.2, ((9, .5),), 'new-session'),
                (100.2, ((9, .7),), 'camera-session'),
                (100.2, ((9, .495), (10, .505)), 'camera-session')):
            with self.subTest(stamp=stamp, targets=targets, session=session):
                guard = FireRetriggerGuard()
                guard.observe(view(100., ((1, .5),)), 100.)
                guard.mark_processed(('camera-session', 1), 100.)
                guard.observe(view(stamp, targets, session), stamp)
                self.assertTrue(guard.allows((session, 9), 104.))
                self.assertEqual(guard.alias_count, 0)

    def test_duplicate_stale_and_low_confidence_frames_cannot_reassociate(self):
        guard = FireRetriggerGuard()
        guard.observe(view(100., ((1, .5),)), 100.)
        guard.mark_processed(('camera-session', 1), 100.)
        guard.observe(view(100., ((9, .5),)), 100.2)
        guard.observe(view(100.1, ((9, .5),)), 101.)
        doc = view(100.2, ((9, .5),)); doc['targets'][0]['confidence'] = .89
        guard.observe(doc, 100.2)
        self.assertTrue(guard.allows(('camera-session', 9), 104.))
        self.assertEqual(guard.alias_count, 0)

    def test_invalid_boxes_or_ids_do_not_break_filter(self):
        guard = FireRetriggerGuard()
        doc = view(100., ((1, .5),))
        doc['targets'] += [None, dict(target_id=[], selectable=True), dict(target_id=2, selectable=True,
            confidence=.95, bbox_xyxy=[0, 0, 0, 0])]
        guard.observe(doc, 100.)
        self.assertEqual(guard.boxes.keys(), {('camera-session', 1)})
        self.assertIsInstance(guard.filtered(doc, 100.)['targets'], list)

    def test_new_task_clears_processed_identity_and_cooldown(self):
        guard = FireRetriggerGuard()
        guard.mark_processed(('camera-session', 1), 100.)
        guard.reset()
        self.assertTrue(guard.allows(('camera-session', 1), 100.))
        self.assertEqual(guard.snapshot(100.)['cooldown_remaining_s'], 0.)


class CheckpointRetriggerIntegrationTests(unittest.TestCase):
    @staticmethod
    def finish_first(h):
        h.centered_dwell()
        for _ in range(12):
            h.tick(((1, .5),))
            if h.c.phase == 'NAVIGATING':
                break
        assert h.c.phase == 'NAVIGATING', h.c.snapshot(h.now)
        assert h.c.fire_retrigger.remaining(h.now) > 0

    def test_next_checkpoint_waits_cooldown_and_never_requeues_processed_flame(self):
        h = AlignmentHarness()
        self.finish_first(h)
        h.position = list(h.c.goal_cell)
        for _ in range(10):
            h.tick(((1, .5), (2, .7)))
            self.assertEqual(h.c.phase, 'CHECKPOINT_COLLECT')
            self.assertEqual(h.c.checkpoint.queue, [])
        for _ in range(20):
            h.tick(((1, .5), (2, .7)))
            if h.c.phase == 'ALIGN_SELECT':
                break
        self.assertEqual(h.c.phase, 'ALIGN_SELECT')
        self.assertEqual([r['target_id'] for r in h.c.checkpoint.queue], [2])
        self.assertEqual(h.c.fire_statistics(h.now)['current_visible'], 2)

    def test_second_queued_flame_completes_even_while_first_has_cooldown(self):
        h = AlignmentHarness()
        h.collect(((1, .48), (2, .53)))
        observed_during_cooldown = False
        for _ in range(50):
            h.tick(((1, .48), (2, .53)))
            if h.c.checkpoint.identity == ('camera-session', 2) and h.c.fire_retrigger.remaining(h.now):
                observed_during_cooldown = True
            if h.c.phase == 'NAVIGATING':
                break
        self.assertTrue(observed_during_cooldown)
        self.assertEqual([r['status'] for r in h.c.checkpoint.queue], ['completed', 'completed'])
        self.assertEqual(h.c.fire_retrigger.alias_count, 0)
        self.assertEqual(h.c.fire_retrigger.snapshot(h.now)['processed_identity_count'], 2)


if __name__ == '__main__':
    unittest.main()
