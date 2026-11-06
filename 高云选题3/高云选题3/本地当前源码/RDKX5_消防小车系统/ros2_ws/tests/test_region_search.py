"""Semantic-region coordination with decoded fake feedback; no hardware I/O."""
import copy
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision.region_search import RegionSearch, load_catalog


CATALOG = json.loads((ROOT / 'config/semantic_regions.json').read_text(encoding='utf-8'))


def frame(now, target=None, session='camera-session'):
    return dict(capture_stamp=now, frame_valid=True, valid=target is not None,
                registry_session=session, image_size_wh=[640, 480], targets=[] if target is None else
                [dict(target_id=target, selectable=True, confidence=.95, bbox_xyxy=[100, 100, 180, 220])])


def actuator(now, **changes):
    return dict(dict(stamp=now, pump_on=False, pump_feedback_fresh=True,
                     guard=dict(armed=False)), **changes)


def geometry(now, target=1, **changes):
    return dict(dict(stamp=now, capture_stamp=now, registry_session='camera-session',
                     target_id=target, valid=True, calibration_verified=True,
                     map_verified=True, clearance_verified=True,
                     map_sha256=CATALOG['map_sha256'], fire_world_m=[.4, 2.7, .35],
                     goal_cell=[25, 13], in_spray_range=False), **changes)


class Harness:
    def __init__(self, region='A', execute_fire=True):
        self.c = RegionSearch(CATALOG)
        self.now = 100.
        self.history = self.c.start(dict(action='search_region', region_id=region,
                                        execute_fire=execute_fire, command_id='search:1'), self.now,
                                    dict(valid=True, cell=[88, 13], heading_deg=-33.5))

    def tick(self, dt=.1, *, target=None, phase='running', stopped=False,
             nav_changes=None, proposal=None, auto=None, output=None, frame_changes=None):
        self.now = round(self.now + dt, 8)
        nav = dict(stamp=self.now, command_id=self.c.leg_id, phase=phase,
                   stopped=stopped, pose_valid=True, pose_cell=self.c.goal_cell,
                   arrived=phase == 'arrived')
        nav.update(nav_changes or {})
        view = frame(self.now, target)
        view.update(frame_changes or {})
        if isinstance(proposal, dict):
            proposal = geometry(self.now, target or 1, **proposal)
        result = self.c.tick(self.now, nav, view,
                             auto or dict(stamp=self.now, active=False, phase='IDLE'),
                             output or actuator(self.now), proposal)
        self.history += result
        return result

    def detect(self, **proposal_changes):
        for _ in range(3):
            self.tick(target=1, proposal=proposal_changes)
        return self.c.leg_id

    def visit(self):
        self.tick(phase='arrived', stopped=True)
        for _ in range(8):
            self.tick(.2, phase='arrived', stopped=True)

    def enter_round(self):
        self.detect()
        self.tick(target=1, phase='paused', stopped=True, proposal={})
        self.tick(target=1, phase='arrived', stopped=True, proposal={})
        self.tick(target=1, phase='arrived', stopped=True, proposal=dict(in_spray_range=True))
        assert self.c.phase == 'EXTINGUISHING', self.c.snapshot(self.now)

    def finish_round(self, **changes):
        now = self.now + .1
        state = dict(stamp=now, command_id=self.c.run_id, active=False, phase='COMPLETE',
                     reason='one_target_round_finished', saw_pump_on=True,
                     burst_completed=True, confirmed_pump_off_at=now - .01)
        state.update(changes)
        return self.tick(target=1, phase='arrived', stopped=True, auto=state)


class RegionCatalogTests(unittest.TestCase):
    def test_all_sixteen_waypoints_are_free_cells_in_actual_fpga_map(self):
        catalog = load_catalog(CATALOG)
        bitmap = ROOT.parents[1] / 'ov5640/src/navigation/map_plan_2cm.mem'
        occupancy = [int(word, 16) for word in bitmap.read_text().split()]
        self.assertEqual(len(occupancy), 176 * 160)
        cells = []
        for region in catalog['regions'].values():
            self.assertLessEqual(len(region['waypoints']), 4)
            for point in region['waypoints']:
                x, y = point['cell']
                self.assertEqual(occupancy[y * 176 + x], 0, point)
                self.assertEqual(point['image_cm'], [2 * x + 1, 2 * y + 1])
                self.assertAlmostEqual(point['world_m'][1], (320 - (2 * y + 1)) / 100)
                cells.append((x, y))
        self.assertEqual(len(set(cells)), 16)

    def test_outside_or_excess_waypoints_rejected(self):
        for mutation in ('outside', 'extra', 'duplicate'):
            value = copy.deepcopy(CATALOG)
            points = value['regions']['A']['waypoints']
            if mutation == 'outside':
                points[0]['cell'] = [158, 17]
            elif mutation == 'extra':
                points.append(copy.deepcopy(points[0]))
            else:
                points[1]['id'] = points[0]['id']
            with self.assertRaises(ValueError):
                load_catalog(value)


class RegionSearchTests(unittest.TestCase):
    def test_search_only_stops_on_three_frames_dwells_and_resumes_remaining_route(self):
        h = Harness('C', execute_fire=False)
        for _ in range(2):
            h.tick(target=7)
            self.assertEqual(h.c.phase, 'NAVIGATING')
        effects = h.tick(target=7)
        self.assertEqual(h.c.phase, 'WAIT_STOP')
        self.assertEqual([key for key, value in effects], ['pause'])
        self.assertFalse(h.c.pump_allowed(h.now))
        h.tick(phase='paused', stopped=False)
        self.assertEqual(h.c.phase, 'WAIT_STOP')
        # A target may leave the view while stopping. Preserve its observation
        # timestamp; neither fabricate current visibility nor start spraying.
        h.tick(phase='paused', stopped=True)
        state = h.c.snapshot(h.now)
        self.assertEqual(state['phase'], 'OBSERVATION_DWELL')
        self.assertEqual(state['reason'], 'fire_observed_waiting_two_second_stop')
        self.assertEqual(state['target_id'], 7)
        self.assertTrue(state['fire_observed'])
        self.assertLess(state['fire_observed_at'], h.now)
        self.assertFalse(state['execute_fire'])
        self.assertFalse(state['extinguished_verified'])
        self.assertIsNone(h.c.target_world)
        self.assertEqual(sum(key == 'navigate' for key, value in h.history), 1)
        self.assertFalse(any(key == 'auto' for key, value in h.history))
        self.assertTrue(all(value['action'] == 'disarm' for key, value in h.history if key == 'actuator'))
        for _ in range(9):
            h.tick(.2, phase='paused', stopped=True)
            self.assertEqual(h.c.phase, 'OBSERVATION_DWELL')
        h.tick(.21, phase='paused', stopped=True)
        self.assertEqual(h.c.phase, 'NAVIGATING')
        self.assertIn(('camera-session', 7), h.c.handled)
        for _ in range(4):
            h.visit()
        self.assertEqual(h.c.phase, 'COMPLETE')
        self.assertEqual(len(h.c.visited), 4)
        self.assertFalse(any(key == 'auto' for key, value in h.history))

    def test_search_only_observation_at_waypoint_continues_without_geometry_wait(self):
        h = Harness(execute_fire=False)
        h.tick(phase='arrived', stopped=True)
        for _ in range(3):
            h.tick(target=1, phase='arrived', stopped=True)
        self.assertEqual(h.c.phase, 'OBSERVATION_DWELL')
        self.assertFalse(any(key in ('pause', 'auto') for key, value in h.history))
        for _ in range(11):
            h.tick(.2, target=1, phase='arrived', stopped=True)
        self.assertEqual(h.c.phase, 'OBSERVING')
        for _ in range(8):
            h.tick(.2, target=1, phase='arrived', stopped=True)
            if h.c.phase == 'NAVIGATING':
                break
        self.assertEqual(h.c.phase, 'NAVIGATING')
        self.assertEqual(len(h.c.visited), 1)

    def test_search_only_same_frame_or_unready_navigation_never_stops_for_fire(self):
        for phase, capture in (('running', 100.1), ('planning', None)):
            h = Harness(execute_fire=False)
            for _ in range(4):
                h.tick(target=1, phase=phase,
                       frame_changes=dict(capture_stamp=capture) if capture else None)
            self.assertEqual(h.c.phase, 'NAVIGATING')
            self.assertFalse(any(key == 'pause' for key, value in h.history))

    def test_search_only_requires_real_stop_and_pump_off_before_reporting_found(self):
        for changes in (dict(pump_feedback_fresh=False), dict(guard=dict(armed=True))):
            h = Harness(execute_fire=False)
            for _ in range(3):
                h.tick(target=1)
            for _ in range(51):
                h.tick(phase='paused', stopped=True, output=actuator(h.now + .1, **changes))
            self.assertEqual(h.c.phase, 'FAILED')
            self.assertEqual(h.c.reason, 'region_stop_or_pump_off_not_confirmed')
        h = Harness(execute_fire=False)
        for _ in range(3):
            h.tick(target=1)
        for _ in range(51):
            h.tick(phase='paused', stopped=False)
        self.assertEqual(h.c.phase, 'FAILED')

    def test_search_only_traverses_all_waypoints_without_fire_and_rejects_implicit_mode(self):
        h = Harness('C', execute_fire=False)
        for _ in range(4):
            h.visit()
        self.assertEqual((h.c.phase, h.c.reason), ('COMPLETE', 'no_fire_found'))
        self.assertEqual(len(h.c.visited), 4)
        self.assertFalse(h.c.snapshot(h.now)['fire_observed'])
        self.assertFalse(any(key == 'auto' for key, value in h.history))
        for mode in (None, 0, 1, 'false'):
            with self.assertRaisesRegex(ValueError, 'invalid_region_search_request'):
                h.c.start(dict(action='search_region', region_id='C', execute_fire=mode,
                               command_id='invalid-mode'), h.now,
                          dict(valid=True, cell=[88, 13], heading_deg=0.))

    def test_reassociation_is_only_accepted_stopped_after_approach_with_fresh_evidence(self):
        h = Harness()
        h.detect()
        h.tick(target=1, phase='paused', stopped=True, proposal={})
        h.tick(target=1, phase='arrived', stopped=True, proposal={})
        self.assertEqual(h.c.phase, 'WAIT_SPRAY_RANGE')
        wait_at = h.now
        keys = []
        for index in range(3):
            stamp = round(h.now + .1, 8)
            keys.append([round(stamp * 1e9), 50 + index])
            proof = dict(in_spray_range=True, capture_stamp_ns=keys[-1][0], sequence=keys[-1][1],
                         reassociation_verified=index == 2, original_identity=['camera-session', 1],
                         reassociation_stationary_verified=True, reassociation_exposure_keys=list(keys),
                         reassociation_stopped_since=wait_at, reassociation_second_distance_m=None)
            h.tick(target=2, phase='arrived', stopped=True, proposal=proof)
        self.assertEqual(h.c.phase, 'EXTINGUISHING')
        self.assertEqual(h.c.target, ('camera-session', 2))
        start = [v for k, v in h.history if k == 'auto' and v['action'] == 'start'][-1]
        self.assertEqual(start['preferred_target_id'], 2)

    def test_reassociation_cannot_change_active_spray_target(self):
        h = Harness()
        h.enter_round()
        h.tick(target=2, phase='arrived', stopped=True,
               proposal=dict(in_spray_range=True, reassociation_verified=True,
                             original_identity=['camera-session', 1]),
               auto=dict(stamp=h.now + .1, command_id=h.c.run_id, active=True, phase='SPRAYING'))
        self.assertEqual(h.c.target, ('camera-session', 1))

    def test_initial_point_already_at_first_search_waypoint_rotates_it_to_last(self):
        c = RegionSearch(CATALOG)
        start_cell = CATALOG['regions']['A']['waypoints'][0]['cell']
        effects = c.start(dict(action='search_region', region_id='A', execute_fire=True, command_id='start-here'),
                          100., dict(valid=True, cell=start_cell, heading_deg=0.))
        self.assertEqual(c.waypoints[-1]['id'], 'A1')
        self.assertEqual(len(c.waypoints), 4)
        self.assertNotEqual(effects[0][1]['goal_cell'], start_cell)

    def test_no_fire_visits_each_waypoint_once_and_finishes_stopped(self):
        for region in 'ABCD':
            h = Harness(region)
            for _ in range(4):
                h.visit()
            state = h.c.snapshot(h.now)
            self.assertEqual(state['phase'], 'COMPLETE', state)
            self.assertEqual(state['reason'], 'no_fire_found')
            self.assertEqual(state['progress'], 1.)
            self.assertEqual(state['visited_waypoint_ids'], [region + str(i) for i in range(1, 5)])
            legs = [value for kind, value in h.history if kind == 'navigate']
            self.assertEqual(len(legs), 4)
            self.assertTrue(all(value['use_current_pose'] for value in legs))
            self.assertIn('start_reference', legs[0])
            self.assertTrue(all('start_reference' not in value for value in legs[1:]))
            self.assertTrue(any(kind == 'stop' for kind, value in h.history))
            self.assertFalse(any(kind == 'auto' for kind, value in h.history))
            self.assertFalse(state['extinguished_verified'])

    def test_fire_approaches_only_after_verified_stop_then_resumes_unvisited_waypoint(self):
        h = Harness()
        original = h.detect()
        self.assertEqual(h.c.phase, 'WAIT_STOP')
        self.assertEqual(h.c.visited, [])
        h.tick(target=1, phase='paused', proposal={})
        self.assertEqual(h.c.phase, 'WAIT_STOP')  # ACK without physical stop.
        h.tick(target=1, phase='paused', stopped=True, proposal={})
        self.assertEqual(h.c.phase, 'APPROACHING')
        self.assertNotEqual(h.c.leg_id, original)
        h.tick(target=1, phase='arrived', stopped=True, proposal={})
        self.assertEqual(h.c.phase, 'WAIT_SPRAY_RANGE')
        h.tick(target=1, phase='arrived', stopped=True, proposal=dict(in_spray_range=True))
        self.assertEqual(h.c.phase, 'EXTINGUISHING')
        auto = [v for k, v in h.history if k == 'auto' and v['action'] == 'start'][-1]
        self.assertEqual((auto['preferred_registry_session'], auto['preferred_target_id']), ('camera-session', 1))
        self.assertTrue(h.c.pump_allowed(h.now))
        run = h.c.run_id
        h.finish_round()
        self.assertEqual(h.c.phase, 'WAIT_PUMP_OFF')
        h.tick(target=1, phase='arrived', stopped=True,
               auto=dict(stamp=h.now + .1, active=False, phase='CANCELLED', command_id=run))
        self.assertEqual(h.c.phase, 'NAVIGATING')
        self.assertEqual(h.c.visited, [])
        self.assertEqual(h.c.snapshot(h.now)['current_waypoint_id'], 'A1')
        self.assertNotEqual(h.c.leg_id, original)
        self.assertEqual(h.c.rounds, 1)
        # Persistent wallpaper identity is not sprayed repeatedly.
        for _ in range(4):
            h.tick(target=1, proposal=dict(in_spray_range=True))
        self.assertEqual(h.c.phase, 'NAVIGATING')

    def test_missing_stale_wrong_map_and_wrong_target_geometry_cannot_interrupt(self):
        variants = [dict(calibration_verified=False), dict(map_verified=False),
                    dict(clearance_verified=False), dict(map_sha256='0' * 64),
                    dict(stamp=99.), dict(capture_stamp=99.), dict(target_id=2)]
        for proposal in variants:
            h = Harness()
            for _ in range(4):
                h.tick(target=1, proposal=proposal)
            self.assertEqual(h.c.phase, 'NAVIGATING', proposal)
            self.assertFalse(any(k in ('pause', 'auto') for k, v in h.history))
        h = Harness()
        for _ in range(4):
            h.tick(target=1)
        self.assertEqual(h.c.phase, 'NAVIGATING')

    def test_stationary_visible_fire_without_geometry_reports_unfinished(self):
        h = Harness()
        h.tick(phase='arrived', stopped=True)
        for _ in range(30):
            h.tick(.2, target=1, phase='arrived', stopped=True)
        self.assertEqual(h.c.phase, 'FAILED')
        self.assertEqual(h.c.reason, 'fire_geometry_or_calibration_unavailable')
        self.assertEqual(h.c.visited, [])
        self.assertFalse(any(k == 'auto' and v['action'] == 'start' for k, v in h.history))

    def test_verified_fire_outside_selected_region_is_ignored(self):
        h = Harness()
        for _ in range(4):
            h.tick(target=1, proposal=dict(fire_world_m=[2.9, 2.7, .35]))
        self.assertEqual(h.c.phase, 'NAVIGATING')
        h.tick(target=1, phase='arrived', stopped=True)
        for _ in range(8):
            h.tick(.2, target=1, phase='arrived', stopped=True)
        self.assertEqual(h.c.visited, ['A1'])

    def test_delayed_mapping_can_release_geometry_wait_when_fire_is_outside(self):
        h = Harness()
        h.tick(phase='arrived', stopped=True)
        for _ in range(3):
            h.tick(target=1, phase='arrived', stopped=True)
        self.assertEqual(h.c.phase, 'WAIT_GEOMETRY')
        h.tick(target=1, phase='arrived', stopped=True,
               proposal=dict(fire_world_m=[2.9, 2.7, .35]))
        self.assertEqual(h.c.phase, 'OBSERVING')
        for _ in range(8):
            h.tick(.2, target=1, phase='arrived', stopped=True)
        self.assertEqual(h.c.visited, ['A1'])

    def test_repeated_exposure_does_not_count_as_stable_fire(self):
        h = Harness()
        for _ in range(4):
            h.tick(target=1, proposal={}, frame_changes=dict(capture_stamp=100.1))
        self.assertEqual(h.c.phase, 'NAVIGATING')
        self.assertEqual(h.c.hits, 1)

    def test_old_arrival_wrong_pose_or_not_stopped_cannot_advance(self):
        for changes in (dict(command_id='old-leg'), dict(pose_valid=False),
                        dict(pose_cell=[88, 13]), dict(arrived=False), dict(stopped=False)):
            h = Harness()
            h.tick(phase='arrived', stopped=True, nav_changes=changes)
            self.assertEqual(h.c.phase, 'NAVIGATING', changes)
            self.assertEqual(h.c.visited, [])

    def test_planning_without_pose_stays_alive_and_cannot_pause_for_fire(self):
        h = Harness()
        for _ in range(40):
            h.tick(.2, target=1, phase='planning', proposal={},
                   nav_changes=dict(pose_valid=False, pose_cell=None))
        self.assertEqual(h.c.phase, 'NAVIGATING')
        self.assertFalse(any(k == 'pause' for k, value in h.history))
        h.tick(phase='failed', nav_changes=dict(pose_valid=False, pose_cell=None))
        self.assertEqual(h.c.phase, 'FAILED')
        self.assertEqual(h.c.reason, 'region_navigation_failed')

    def test_failed_round_or_missing_high_low_transition_never_resumes(self):
        for changes in (dict(reason='pump_not_ready'), dict(burst_completed=False),
                        dict(saw_pump_on=False), dict(confirmed_pump_off_at=None),
                        dict(phase='CANCELLED')):
            h = Harness()
            h.enter_round()
            previous_legs = sum(k == 'navigate' for k, v in h.history)
            h.finish_round(**changes)
            self.assertEqual(h.c.phase, 'FAILED', changes)
            self.assertEqual(sum(k == 'navigate' for k, v in h.history), previous_legs)
            self.assertEqual(h.c.rounds, 0)

    def test_pump_off_ack_and_fresh_disarm_required_before_replanning(self):
        h = Harness()
        h.enter_round()
        h.finish_round()
        run = h.c.run_id
        result = h.tick(target=1, phase='arrived', stopped=True,
                        auto=dict(stamp=h.now + .1, command_id=run, active=False),
                        output=actuator(h.now + .1, guard=dict(armed=True)))
        self.assertEqual(h.c.phase, 'WAIT_PUMP_OFF')
        self.assertFalse(any(k == 'navigate' for k, v in result))
        h.tick(target=1, phase='arrived', stopped=True,
               auto=dict(stamp=h.now + .1, command_id='unrelated-run', active=False))
        self.assertEqual(h.c.phase, 'WAIT_PUMP_OFF')

    def test_fire_at_search_waypoint_in_range_resumes_observation_without_zero_length_leg(self):
        h = Harness()
        h.tick(phase='arrived', stopped=True)
        for _ in range(3):
            h.tick(target=1, phase='arrived', stopped=True, proposal=dict(in_spray_range=True))
        self.assertEqual(h.c.phase, 'EXTINGUISHING')
        run = h.c.run_id
        count = sum(k == 'navigate' for k, v in h.history)
        h.finish_round()
        h.tick(target=1, phase='arrived', stopped=True,
               auto=dict(stamp=h.now + .1, active=False, phase='CANCELLED', command_id=run))
        self.assertEqual(h.c.phase, 'OBSERVING')
        self.assertEqual(sum(k == 'navigate' for k, v in h.history), count)

    def test_cancel_cannot_be_revived_by_delayed_feedback(self):
        h = Harness()
        h.enter_round()
        h.history += h.c.cancel(h.now, 'operator_stop')
        count = len(h.history)
        h.finish_round()
        self.assertEqual(h.c.phase, 'CANCELLED')
        self.assertEqual(len(h.history), count)
        self.assertFalse(h.c.pump_allowed(h.now))

    def test_no_bare_pixel_detections_are_used_as_world_positions(self):
        h = Harness()
        h.detect(calibration_verified=False)
        self.assertEqual(h.c.phase, 'NAVIGATING')
        self.assertFalse(any(kind == 'pause' for kind, value in h.history))

    def test_unexpected_pump_on_aborts_task(self):
        h = Harness()
        h.tick(output=actuator(100.1, pump_on=True))
        self.assertEqual(h.c.phase, 'FAILED')
        self.assertEqual(h.c.reason, 'unexpected_pump_output')


if __name__ == '__main__':
    unittest.main()
