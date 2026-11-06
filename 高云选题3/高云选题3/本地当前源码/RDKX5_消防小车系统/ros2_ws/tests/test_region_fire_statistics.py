"""Cumulative observations and mapped estimates, separate from pump completion."""
import unittest
from unittest.mock import patch

from test_region_search import Harness, geometry


def targets(*ids):
    return [dict(target_id=identifier, selectable=True, confidence=.95, bbox_xyxy=[100, 100, 180, 220])
            for identifier in ids]


class RegionFireStatisticsTests(unittest.TestCase):
    def test_all_stable_targets_count_once_including_handled_and_excluded(self):
        h = Harness(execute_fire=False)
        h.c.handled.add(('camera-session', 2)); h.c.excluded.add(('camera-session', 3))
        h.tick(frame_changes=dict(targets=targets(1, 2, 3)))
        for _ in range(2):
            h.tick(phase='planning', frame_changes=dict(targets=targets(1, 2, 3)))
        stats = h.c.snapshot(h.now)['fire_statistics']
        self.assertEqual(stats['current_visible'], 3)
        self.assertEqual(stats['tracked_observations_count'], 3)
        self.assertEqual(stats['max_visible'], 3)
        self.assertEqual(stats['handled_observations_count'], 1)
        self.assertEqual(stats['rounds_completed'], 0)
        self.assertEqual(stats['mapped_unique_count'], 0)
        self.assertIs(stats['physical_total_verified'], False)

    def test_bad_duplicate_and_stale_frames_do_not_invent_new_observations(self):
        h = Harness(execute_fire=False)
        h.tick(target=1)
        stamp = h.now
        h.tick(target=2, frame_changes=dict(capture_stamp=stamp))
        self.assertEqual(h.c.fire_statistics(h.now)['tracked_observations_count'], 1)
        h.tick(frame_changes=dict(targets=targets(True, -1, 0) + [
            dict(target_id=2, selectable=False, confidence=.95, bbox_xyxy=[100, 100, 180, 220]),
            dict(target_id=3, selectable=True, confidence=.95, bbox_xyxy=[-1, 100, 180, 220])]))
        self.assertEqual(h.c.fire_statistics(h.now)['tracked_observations_count'], 1)
        self.assertEqual(h.c.fire_statistics(h.now)['current_visible'], 0)
        h.tick(target=4, frame_changes=dict(capture_stamp=99.))
        self.assertFalse(h.c.fire_statistics(h.now)['current_fresh'])

    def test_new_registry_is_new_track_but_verified_world_can_merge(self):
        h = Harness()
        now = h.now + .1
        records = [geometry(now, 1), geometry(now, 2, fire_world_m=[.8, 2.7, .35])]
        h.tick(phase='planning', frame_changes=dict(targets=targets(1, 2)),
               proposal=dict(mapped_targets=records))
        self.assertEqual(h.c.fire_statistics(h.now)['mapped_unique_count'], 2)
        # New identities after occlusion remain distinct tracks; verified
        # positions within 12 cm are merged only for the mapped estimate.
        h.tick(target=9, phase='planning', frame_changes=dict(registry_session='new-session'),
               proposal=dict(registry_session='new-session', fire_world_m=[.41, 2.71, .35]))
        stats = h.c.fire_statistics(h.now)
        self.assertEqual(stats['tracked_observations_count'], 3)
        self.assertEqual(stats['mapped_unique_count'], 2)
        # A later noisy estimate for the same identity is not another fire.
        h.tick(target=9, phase='planning', frame_changes=dict(registry_session='new-session'),
               proposal=dict(registry_session='new-session', fire_world_m=[1., 2.71, .35]))
        self.assertEqual(h.c.fire_statistics(h.now)['mapped_unique_count'], 2)

    def test_mapping_requires_fresh_visible_identity_calibration_and_current_map(self):
        for change in (dict(calibration_verified=False), dict(map_verified=False),
                       dict(map_sha256='0' * 64), dict(capture_stamp=99.), dict(target_id=7),
                       dict(fire_world_m=[.4, 2.7, float('nan')]),
                       dict(fire_world_m=[2.5, .3, .35])):
            with self.subTest(change=change):
                h = Harness()
                h.tick(target=1, phase='planning', proposal=change)
                self.assertEqual(h.c.fire_statistics(h.now)['mapped_unique_count'], 0)

    def test_statistics_retained_after_completion_stale_visible_zero_and_next_task_resets(self):
        h = Harness(execute_fire=False)
        for _ in range(3):
            h.tick(target=1)
        h.tick(target=1, phase='paused', stopped=True)
        for _ in range(11):
            h.tick(.2, target=1, phase='paused', stopped=True)
        for _ in range(4):
            h.visit()
        self.assertFalse(h.c.active)
        stats = h.c.snapshot(h.now + 1.)['fire_statistics']
        self.assertEqual(stats['tracked_observations_count'], 1)
        self.assertEqual(stats['current_visible'], 0)
        self.assertFalse(stats['current_fresh'])
        h.c.start(dict(action='search_region', region_id='B', execute_fire=False, command_id='second'),
                  h.now + 1., dict(valid=True, cell=[88, 13], heading_deg=0.))
        self.assertEqual(h.c.fire_statistics(h.now + 1.)['tracked_observations_count'], 0)

    def test_counts_are_bounded_and_do_not_increment_for_every_frame_after_cap(self):
        h = Harness(execute_fire=False)
        with patch('rescue_vision.region_search.MAX_FIRE_OBSERVATIONS', 2):
            for _ in range(5):
                h.tick(phase='planning', frame_changes=dict(targets=targets(1, 2, 3)))
        stats = h.c.fire_statistics(h.now)
        self.assertEqual(stats['tracked_observations_count'], 2)
        self.assertEqual(stats['current_visible'], 3)
        self.assertTrue(stats['truncated'])

    def test_completed_round_count_requires_actual_pulse_and_off_evidence(self):
        h = Harness(); h.enter_round()
        self.assertEqual(h.c.fire_statistics(h.now)['rounds_completed'], 0)
        h.finish_round()
        stats = h.c.fire_statistics(h.now)
        self.assertEqual(stats['rounds_completed'], 1)
        self.assertEqual(stats['handled_observations_count'], 1)
        self.assertFalse(stats['physical_total_verified'])


if __name__ == '__main__':
    unittest.main()
