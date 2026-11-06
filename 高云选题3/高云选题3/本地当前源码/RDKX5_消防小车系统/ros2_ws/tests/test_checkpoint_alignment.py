"""Checkpoint target queues use real stop semantics and bounded route effects."""
import copy
import unittest

from test_region_search import CATALOG, actuator, geometry
from rescue_vision.checkpoint_alignment import CheckpointAlignment
from rescue_vision.region_search import RegionSearch


def view(now, targets=(), session='camera-session'):
    return dict(capture_stamp=now, frame_valid=True, registry_session=session, image_size_wh=[640, 480],
        targets=[dict(target_id=identifier, selectable=True, confidence=.95, bbox_xyxy=[center * 640 - 12, 100, center * 640 + 12, 200])
                 for identifier, center in targets])


class AlignmentHarness:
    def __init__(self, execute_fire=False, **config):
        self.now = 100.
        self.c = RegionSearch(CATALOG, alignment_config=dict(enabled=True, forward_image_side='right', **config))
        self.c.start(dict(action='search_region', region_id='A', execute_fire=execute_fire, command_id='cp-task'),
                     self.now, dict(valid=True, cell=[88, 13], heading_deg=180.))
        self.position, self.heading = list(self.c.goal_cell), 0.
        self.history = []

    def tick(self, targets=(), *, phase='arrived', stopped=True, dt=.2, provide=False,
             proposal_change=None, nav_change=None, frame_change=None, approach=False, auto=None, output=None):
        self.now = round(self.now + dt, 8)
        navigation = dict(stamp=self.now, command_id=self.c.leg_id, phase=phase, stopped=stopped,
            pose_valid=True, pose_cell=list(self.position), pose_heading_deg=self.heading, arrived=phase == 'arrived')
        navigation.update(nav_change or {})
        document = view(self.now, targets)
        document.update(frame_change or {})
        proposed = None
        if provide:
            proposed = dict(stamp=self.now, valid=True, road_path_verified=True,
                goal_cell=[self.position[0] + 12, self.position[1]], from_pose_cell=list(self.position),
                from_heading_deg=self.heading, path_length_cm=24.,
                alignment_guard=dict(start_cell=list(self.position), heading_deg=self.heading,
                    max_advance_cm=30., lateral_tolerance_cm=6., heading_tolerance_deg=15., speed_mm_s=60))
            proposed.update(proposal_change or {})
        proposal = geometry(self.now, target=targets[0][0], in_spray_range=True) if approach and targets else None
        effects = self.c.tick(self.now, navigation, document, auto or {}, output or actuator(self.now), proposal, proposed)
        self.history.extend(effects)
        return effects

    def collect(self, targets):
        self.tick(targets)
        for _ in range(8):
            self.tick(targets)
        assert self.c.phase == 'ALIGN_SELECT', self.c.snapshot(self.now)

    def start_move(self, targets=((1, .75),)):
        self.collect(targets)
        self.tick(targets, provide=True)
        assert self.c.phase == 'ALIGNING', self.c.snapshot(self.now)

    def centered_dwell(self, target=1):
        self.collect(((target, .5),))
        for _ in range(4):
            self.tick(((target, .5),))
        assert self.c.phase == 'ALIGN_DWELL', self.c.snapshot(self.now)


class CheckpointQueueTests(unittest.TestCase):
    def test_semantic_confidence_threshold_rejects_missing_invalid_and_sub90_scores(self):
        for score in (None, True, float('nan'), float('inf'), .40, .899999, 1.01, .90, .99):
            with self.subTest(score=score):
                h = AlignmentHarness()
                for _ in range(10):
                    frame = view(h.now + .2, ((1, .7),))
                    if score is None:
                        frame['targets'][0].pop('confidence')
                    else:
                        frame['targets'][0]['confidence'] = score
                    h.tick(frame_change=frame)
                accepted = type(score) is float and .90 <= score <= 1.
                self.assertEqual(len(h.c.checkpoint.queue), int(accepted))
                self.assertEqual(h.c.fire_statistics(h.now)['tracked_observations_count'], int(accepted))
                self.assertEqual(h.c.checkpoint.config['min_confidence'], .90)
        with self.assertRaises(ValueError):
            CheckpointAlignment(dict(min_confidence=.89))

    def test_low_confidence_breaks_consecutive_confirmation_even_for_same_target(self):
        c = CheckpointAlignment(dict(forward_image_side='right'))
        c.begin(100., 'A1')
        for index, score in enumerate((.95, .95, .89, .95, .95)):
            now = 101. + index * .15
            frame = view(now, ((1, .7),))
            frame['targets'][0]['confidence'] = score
            c.observe(frame, now)
        self.assertTrue(c.freeze_queue(now))
        self.assertEqual(c.queue, [])
        c.observe(view(now + .15, ((1, .7),)), now + .15)
        self.assertTrue(c.freeze_queue(now + .15))
        self.assertEqual([row['target_id'] for row in c.queue], [1])

    def test_duplicate_frames_and_single_frame_noise_do_not_make_queue(self):
        c = CheckpointAlignment(dict(forward_image_side='right'))
        c.begin(100, 'A1')
        for now in (100.1, 100.2, 100.3):
            c.observe(view(100.1, ((1, .7),)), now)
        self.assertEqual(c.frame_count, 1)
        c.observe(view(101.5, ((2, .75),)), 101.5)
        c.observe(view(101.6), 101.6)
        c.observe(view(101.7), 101.7)
        self.assertTrue(c.freeze_queue(101.7))
        self.assertEqual(c.queue, [])

    def test_queue_uses_stable_identity_and_nearest_center_first(self):
        h = AlignmentHarness()
        h.collect(((7, .8), (3, .64), (2, .2), (6, .5)))
        self.assertEqual([row['target_id'] for row in h.c.checkpoint.queue], [6, 3, 7])
        self.assertEqual(h.c.checkpoint.snapshot(h.now)['forward_image_side'], 'right')

    def test_unknown_camera_direction_does_not_authorize_movement(self):
        h = AlignmentHarness()
        h.c.configure_checkpoint_alignment(dict(enabled=True, forward_image_side='unset'))
        h.tick(((1, .7),))
        for _ in range(8): h.tick(((1, .7),))
        self.assertEqual(h.c.phase, 'FAILED')
        self.assertEqual(h.c.reason, 'checkpoint_forward_image_side_unset')
        self.assertFalse(any(key == 'navigate' for key, value in h.history))

    def test_left_installation_selects_left_half(self):
        c = CheckpointAlignment(dict(forward_image_side='left'))
        c.begin(100, 'A1')
        for i in range(10):
            now = 100.2 + i * .2
            c.observe(view(now, ((1, .2), (2, .8))), now)
        self.assertTrue(c.freeze_queue(now))
        self.assertEqual([row['target_id'] for row in c.queue], [1])

    def test_config_aliases_preserve_validated_actual_values(self):
        c = CheckpointAlignment(dict(forward_image_side='right', stable_frames=4, settle_s=3,
                                    max_move_s=12, capture_ttl_s=.6))
        state = c.snapshot(100)
        self.assertEqual((state['confirm_frames'], state['center_confirm_frames'], state['dwell_s']), (4, 3, 3))
        with self.assertRaises(ValueError):
            CheckpointAlignment(dict(settle_s=.1))


class CheckpointMovementTests(unittest.TestCase):
    def test_known_road_dead_end_skips_unaligned_target_and_runs_next_checkpoint(self):
        for fire in (False, True):
            h = AlignmentHarness(fire); h.collect(((1, .7),))
            h.tick(((1, .7),), provide=True, proposal_change=dict(valid=False,
                reason='checkpoint_alignment_no_straight_road_in_region'))
            self.assertEqual(h.c.phase, 'ALIGN_SELECT')
            self.assertEqual(h.c.checkpoint.queue[0]['status'], 'skipped')
            self.assertEqual(h.c.snapshot(h.now)['skipped_observations'][0]['target_id'], 1)
            self.assertFalse(h.c.snapshot(h.now)['skipped_observations'][0]['extinguished_verified'])
            h.tick(())
            self.assertEqual(h.c.phase, 'NAVIGATING')
            self.assertEqual(len(h.c.visited), 1)
            self.assertFalse(any(key == 'auto' for key, _ in h.history))

    def test_selection_pose_gap_waits_then_resumes_without_ending_remaining_points(self):
        for broken in (dict(pose_valid=False, pose_cell=None), dict(stopped=False),
                       dict(stamp=0.), dict(command_id='older-leg')):
            with self.subTest(broken=broken):
                h = AlignmentHarness(); h.collect(((1, .5),))
                history = len(h.history)
                for _ in range(5):
                    h.tick(((1, .5),), nav_change=broken, provide=True)
                    self.assertTrue(h.c.active)
                    self.assertEqual(h.c.phase, 'ALIGN_SELECT')
                self.assertEqual(h.history[history:], [])
                self.assertIsNotNone(h.c.checkpoint.snapshot(h.now)['feedback_wait_reason'])
                for _ in range(25):
                    h.tick(((1, .5),))
                    if h.c.phase == 'NAVIGATING':
                        break
                self.assertEqual(h.c.phase, 'NAVIGATING')
                self.assertEqual(len(h.c.visited), 1)
                self.assertTrue(h.c.active)
                self.assertEqual(len(h.c.snapshot(h.now)['remaining_waypoints']), 3)

    def test_unrecovered_stop_feedback_still_times_out_without_new_navigation(self):
        h = AlignmentHarness(); h.collect(((1, .7),))
        for _ in range(18):
            h.tick(((1, .7),), nav_change=dict(pose_valid=False, pose_cell=None), provide=True)
        self.assertEqual(h.c.phase, 'FAILED')
        self.assertEqual(h.c.reason, 'checkpoint_waiting_measured_stop_timeout')
        self.assertFalse(any(key == 'navigate' for key, _ in h.history))
        self.assertTrue(any(key == 'stop' for key, _ in h.history))

    def test_feedback_gap_during_dwell_requires_new_continuous_two_seconds(self):
        h = AlignmentHarness(); h.centered_dwell()
        for _ in range(6):
            h.tick(((1, .5),))
        h.tick(((1, .5),), nav_change=dict(pose_valid=False, pose_cell=None))
        self.assertEqual(h.c.phase, 'ALIGN_DWELL')
        self.assertIsNone(h.c.checkpoint.dwell_at)
        for _ in range(8):
            h.tick(((1, .5),))
        self.assertEqual(h.c.phase, 'ALIGN_DWELL')
        self.assertEqual(h.c.checkpoint.queue[0]['status'], 'aligned')
        for _ in range(15):
            h.tick(((1, .5),))
            if h.c.phase == 'NAVIGATING':
                break
        self.assertEqual(h.c.phase, 'NAVIGATING')

    def test_enroute_detection_does_not_interrupt_checkpoint_navigation(self):
        for fire in (False, True):
            h = AlignmentHarness(fire)
            for _ in range(8): h.tick(((1, .7),), phase='running', stopped=False, approach=fire)
            self.assertEqual(h.c.phase, 'NAVIGATING')
            self.assertFalse(any(key in ('pause', 'auto') for key, value in h.history))
            self.assertEqual(h.c.fire_statistics(h.now)['max_visible'], 1)

    def test_move_effect_contains_verified_guard_and_reserves_budget(self):
        h = AlignmentHarness()
        h.start_move()
        effect = next(value for key, value in h.history if key == 'navigate')
        self.assertEqual((effect['leg_kind'], effect['motion_kind']), ('alignment', 'semantic_alignment'))
        self.assertEqual(effect['alignment_guard']['speed_mm_s'], 60)
        self.assertEqual(h.c.checkpoint.total_advance_cm, 30)

    def test_unverified_origin_or_oversized_move_never_navigates(self):
        for change in (dict(path_length_cm=61), dict(from_pose_cell=[10, 10]), dict(road_path_verified=False),
                       dict(from_heading_deg=90), dict(alignment_guard={}), dict(goal_cell=[999, 2])):
            h = AlignmentHarness(); h.collect(((1, .7),))
            h.tick(((1, .7),), provide=True, proposal_change=change)
            self.assertEqual(h.c.phase, 'FAILED', change)
            self.assertFalse(any(key == 'navigate' for key, value in h.history))

    def test_lost_moving_target_waits_for_actual_stop_without_switching_identity(self):
        h = AlignmentHarness(); h.start_move(((1, .65), (2, .8)))
        for _ in range(4): h.tick(((2, .8),), phase='running', stopped=False)
        self.assertEqual(h.c.phase, 'ALIGN_STOP')
        self.assertEqual(h.c.checkpoint.identity, ('camera-session', 1))
        self.assertEqual(sum(key == 'pause' for key, value in h.history), 1)
        h.tick(((2, .8),), phase='pausing', stopped=False)
        self.assertEqual(h.c.phase, 'ALIGN_STOP')
        h.tick(((2, .8),), phase='paused', stopped=True)
        self.assertEqual(h.c.phase, 'ALIGN_SELECT')
        self.assertEqual(h.c.checkpoint.queue[0]['status'], 'lost')

    def test_target_lost_before_start_cancels_prepared_route(self):
        h = AlignmentHarness(); h.start_move()
        for _ in range(4): h.tick((), phase='planning', stopped=True)
        self.assertEqual(h.c.phase, 'FAILED')
        self.assertIn('cancelled_before_running', h.c.reason)
        self.assertTrue(any(key == 'stop' for key, value in h.history))

    def test_preparation_and_speed_restore_are_fresh_feedback_but_not_arrival(self):
        h = AlignmentHarness(); h.start_move()
        for phase in ('setting_alignment_speed', 'alignment_route_check', 'restoring_speed'):
            h.tick(((1, .7),), phase=phase, stopped=True)
            self.assertEqual(h.c.phase, 'ALIGNING', phase)
            self.assertIsNone(h.c.checkpoint.dwell_at)
        h.tick((), phase='restoring_speed', stopped=True, dt=.7)
        self.assertEqual(h.c.phase, 'ALIGNING')
        self.assertFalse(any(key == 'pause' for key, value in h.history))

    def test_center_confirmation_racing_done_waits_for_verified_arrival(self):
        h = AlignmentHarness(); h.start_move()
        h.position = list(h.c.goal_cell)
        for _ in range(3):
            h.tick(((1, .5),), phase='confirming_arrival', stopped=False)
            self.assertEqual(h.c.phase, 'ALIGNING')
        self.assertFalse(any(key in ('pause', 'stop') for key, value in h.history))
        h.tick(((1, .5),), phase='restoring_speed', stopped=True)
        self.assertEqual(h.c.phase, 'ALIGNING')
        h.tick(((1, .5),), phase='arrived', stopped=True)
        self.assertEqual(h.c.phase, 'ALIGN_STOP')
        self.assertFalse(any(key in ('pause', 'stop') for key, value in h.history))
        for _ in range(4): h.tick(((1, .5),), phase='arrived', stopped=True)
        self.assertEqual(h.c.phase, 'ALIGN_DWELL')

    def test_stop_and_two_second_dwell_require_new_center_frames_after_stop(self):
        h = AlignmentHarness(); h.start_move()
        for _ in range(3): h.tick(((1, .5),), phase='running', stopped=False)
        self.assertEqual(h.c.phase, 'ALIGN_STOP')
        h.tick(((1, .5),), phase='paused', stopped=True)
        self.assertIsNone(h.c.checkpoint.dwell_at)
        for _ in range(2): h.tick(((1, .5),), phase='paused', stopped=True)
        self.assertEqual(h.c.phase, 'ALIGN_STOP')
        h.tick(((1, .5),), phase='paused', stopped=True)
        self.assertEqual(h.c.phase, 'ALIGN_DWELL')
        entered = h.now
        for _ in range(9): h.tick(((1, .5),), phase='paused', stopped=True)
        self.assertEqual(h.c.phase, 'ALIGN_DWELL')
        self.assertLess(h.now - entered, 2)
        h.tick(((1, .5),), phase='paused', stopped=True)
        self.assertEqual(h.c.phase, 'ALIGN_SELECT')
        self.assertEqual(h.c.checkpoint.queue[0]['status'], 'completed')

    def test_bound_reached_continues_only_with_fresh_stop_and_budget(self):
        h = AlignmentHarness(); h.start_move()
        h.position = list(h.c.goal_cell)
        h.tick(((1, .7),), phase='arrived', stopped=True)
        h.tick(((1, .7),), phase='arrived', stopped=True)
        h.tick(((1, .7),), phase='arrived', stopped=True)
        self.assertEqual(h.c.phase, 'ALIGN_SELECT')
        h.tick(((1, .7),), provide=True)
        self.assertEqual(h.c.phase, 'ALIGNING')
        self.assertGreater(h.c.checkpoint.total_advance_cm, 56)
        self.assertLessEqual(h.c.checkpoint.total_advance_cm, 60)
        h.position = list(h.c.goal_cell)
        for _ in range(3): h.tick(((1, .7),))
        h.tick(((1, .7),), provide=True)
        self.assertEqual(h.c.phase, 'ALIGN_SELECT')
        self.assertEqual(h.c.reason, 'checkpoint_forward_distance_budget_exhausted')
        self.assertEqual(h.c.checkpoint.queue[0]['status'], 'skipped')
        h.tick(())
        self.assertEqual(h.c.phase, 'NAVIGATING')
        self.assertEqual(len(h.c.visited), 1)

    def test_movement_timeout_stops_then_reports_failure(self):
        h = AlignmentHarness(); h.start_move()
        for _ in range(61):
            h.tick(((1, .7),), phase='running', stopped=False)
            if h.c.phase == 'ALIGN_STOP': break
        self.assertEqual(h.c.phase, 'ALIGN_STOP')
        h.tick(((1, .7),), phase='paused', stopped=True)
        self.assertEqual(h.c.phase, 'FAILED')
        self.assertEqual(h.c.reason, 'checkpoint_alignment_move_timeout')

    def test_short_stop_settles_budget_once_even_with_repeated_pause_done(self):
        h = AlignmentHarness(); h.start_move()
        h.position[0] += 1  # Synthetic measured 2 cm forward stop.
        for _ in range(3): h.tick(((1, .5),), phase='running', stopped=False)
        h.tick(((1, .5),), phase='paused', stopped=True)
        spent = h.c.checkpoint.total_advance_cm
        self.assertGreaterEqual(spent, 6)
        self.assertLess(spent, 7)
        for _ in range(5): h.tick(((1, .5),), phase='paused', stopped=True)
        self.assertEqual(h.c.checkpoint.total_advance_cm, spent)
        self.assertTrue(h.c.checkpoint.pending_leg['settled'])

    def test_three_close_targets_finish_with_single_checkpoint_distance_budget(self):
        h = AlignmentHarness()
        all_rows = [(1, .62), (2, .72), (3, .82)]
        h.start_move(all_rows)
        for index, identifier in enumerate((1, 2, 3)):
            if index:
                h.tick(all_rows[index:], phase='paused', stopped=True, provide=True)
                self.assertEqual(h.c.phase, 'ALIGNING')
            h.position[0] += 1
            rows = [(identifier, .5)] + all_rows[index + 1:]
            for _ in range(3): h.tick(rows, phase='running', stopped=False)
            self.assertEqual(h.c.phase, 'ALIGN_STOP')
            for _ in range(4): h.tick(rows, phase='paused', stopped=True)
            self.assertEqual(h.c.phase, 'ALIGN_DWELL')
            for _ in range(10): h.tick(rows, phase='paused', stopped=True)
            self.assertEqual(h.c.phase, 'ALIGN_SELECT')
        h.tick((), phase='paused', stopped=True)
        self.assertEqual(h.c.phase, 'NAVIGATING')
        self.assertEqual(h.c.checkpoint.snapshot(h.now)['processed_count'], 3)
        self.assertLess(h.c.checkpoint.total_advance_cm, 21)

    def test_missing_or_unconfirmed_stop_never_refunds_reserved_distance(self):
        c = CheckpointAlignment(dict(forward_image_side='right'))
        c.begin(100, 'A1')
        c.reserve_leg(dict(start_cell=[10, 10], heading_deg=0, max_advance_cm=30), 101)
        for row in (dict(stamp=102, pose_valid=False, stopped=True, phase='paused', pose_cell=[10, 10]),
                    dict(stamp=102, pose_valid=True, stopped=False, phase='paused', pose_cell=[10, 10]),
                    dict(stamp=101, pose_valid=True, stopped=True, phase='paused', pose_cell=[10, 10])):
            c.settle_leg(row, 102)
        self.assertEqual(c.total_advance_cm, 30)
        self.assertFalse(c.pending_leg['settled'])

    def test_session_change_does_not_reassociate_by_same_number(self):
        h = AlignmentHarness(); h.start_move()
        for _ in range(4):
            h.tick(((1, .5),), phase='running', stopped=False, frame_change=dict(registry_session='new-camera'))
        self.assertEqual(h.c.phase, 'ALIGN_STOP')
        self.assertEqual(h.c.checkpoint.stop_reason, 'checkpoint_alignment_target_lost')

    def test_dwell_losing_center_does_not_keep_original_timer(self):
        h = AlignmentHarness(); h.centered_dwell()
        h.tick(((1, .7),))
        self.assertEqual(h.c.phase, 'ALIGN_SELECT')
        self.assertIsNone(h.c.checkpoint.dwell_at)
        self.assertEqual(h.c.checkpoint.queue[0]['status'], 'aligned')

    def test_two_centered_targets_processed_without_pump_in_search_mode(self):
        h = AlignmentHarness(); h.collect(((1, .48), (2, .53)))
        for _ in range(40):
            h.tick(((1, .48), (2, .53)))
            if h.c.phase == 'NAVIGATING': break
        self.assertEqual(h.c.phase, 'NAVIGATING')
        self.assertEqual(h.c.checkpoint.snapshot(h.now)['processed_count'], 2)
        self.assertEqual(len(h.c.visited), 1)
        self.assertFalse(any(key == 'auto' for key, value in h.history))

    def test_fire_starts_only_after_dwell_then_returns_to_queue_after_confirmed_off(self):
        h = AlignmentHarness(True); h.centered_dwell()
        for _ in range(10): h.tick(((1, .5),), approach=True)
        self.assertEqual(h.c.phase, 'ALIGN_FIRE_WAIT')
        h.tick(((1, .5),), approach=True)
        self.assertEqual(h.c.phase, 'EXTINGUISHING')
        fire_id = h.c.run_id
        h.tick(((1, .5),), auto=dict(stamp=h.now + .2, command_id=fire_id, active=False, phase='COMPLETE',
            reason='one_target_round_finished', saw_pump_on=True, burst_completed=True, confirmed_pump_off_at=h.now + .1))
        self.assertEqual(h.c.phase, 'WAIT_PUMP_OFF')
        h.tick(((1, .5),), auto=dict(stamp=h.now + .2, command_id=fire_id, active=False))
        self.assertEqual(h.c.phase, 'ALIGN_SELECT')
        self.assertEqual(h.c.checkpoint.queue[0]['status'], 'completed')
        self.assertEqual(h.c.rounds, 1)
        self.assertEqual(len(h.c.visited), 0)
        h.tick(((1, .5),))
        self.assertEqual(h.c.phase, 'NAVIGATING')
        self.assertEqual(len(h.c.visited), 1)
        # The same task must still visit all three remaining points after the
        # extinguishing round, even when subsequent views contain no flames.
        for _ in range(3):
            h.position = list(h.c.goal_cell)
            for _ in range(12):
                h.tick(())
                if h.c.phase in ('NAVIGATING', 'COMPLETE'):
                    break
        self.assertEqual(h.c.phase, 'COMPLETE')
        self.assertEqual(len(h.c.visited), 4)
        self.assertEqual(h.c.rounds, 1)


if __name__ == '__main__':
    unittest.main()
