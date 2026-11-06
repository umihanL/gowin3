"""Geometry and map registration tests with explicitly synthetic calibration."""
import copy
import hashlib
import json
import re
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision.region_approach import RegionApproach


def read(name):
    return json.loads((ROOT / 'config' / name).read_text(encoding='utf-8'))


def configured(**changes):
    config = read('region_approach.json')
    config.update(calibration_verified=True, map_registration_verified=True,
                  spray_range_verified=True, body_envelope_verified=True, time_sync_verified=True,
                  camera_coordinate_frame='corrected_left_image', stereo_calibration_id='synthetic-test-only',
                  R_camera_to_vehicle=[[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
                  t_camera_to_vehicle_m=[0, 0, .2], nozzle_origin_vehicle_m=[0, 0, .2],
                  spray_range_m=[.4, .8], fire_height_range_m=[.1, 1.], body_clearance_m=.095,
                  traversability=read('region_traversability.json'))
    config.update(changes)
    a = RegionApproach(config, read('fpga_map_catalog.json'), read('semantic_regions.json'), read('extension_map.json'))
    a.set_region('A')
    return a


def telemetry(now, *, x=350, y=2850, heading=0, task=1, stopped=False, **changes):
    result = dict(stamp=now, supported=True, map_signature=read('fpga_map_catalog.json')['map_signature'],
                  pose=dict(stamp=now, valid=True, x_mm=x, y_mm=y, heading_tenths=heading,
                            mode=9, status=2, flags=3, task_id=task),
                  barriers=dict(stamp=now, valid=True, generation=1, segments_mm=[]),
                  stopped=stopped, stopped_stamp=now)
    result.update(changes)
    return result


def observations(a, stamp=100.2, sequence=12, *, position=None, depth_changes=None,
                 detection_changes=None, registry_changes=None, target_id=1):
    registry = dict(capture_stamp=stamp, capture_stamp_ns=round(stamp * 1e9), sequence=sequence,
                    frame_valid=True, valid=True, coordinate_frame='corrected_left_image',
                    registry_session='live-registry', image_size_wh=[640, 480],
                    targets=[dict(target_id=target_id, selectable=True, bbox_xyxy=[100, 100, 180, 220])])
    registry.update(registry_changes or {})
    detection = dict(class_id=0, depth_valid=True, depth_reason='verified_stereo_wallpaper_surface',
                     bbox_xyxy=[100, 100, 180, 220], position_m=position or [0, 0, 1.])
    detection.update(detection_changes or {})
    depth = dict(capture_stamp_ns=round(stamp * 1e9), sequence=sequence,
                 coordinate_frame='corrected_left_image', position_frame='rectified_left_optical',
                 stereo_calibration_id='synthetic-test-only', detections=[detection])
    depth.update(depth_changes or {})
    a.observe_targets(registry, stamp)
    a.observe_depth(depth, stamp)


def matched(**config):
    a = configured(**config)
    a.observe_map(telemetry(100., x=570, y=2930), 100.)
    observations(a)
    a.observe_map(telemetry(100.4, x=650, y=2930), 100.4)
    return a


class ApproachTests(unittest.TestCase):
    def test_reassociation_requires_three_distinct_stopped_depth_exposures(self):
        a = configured()
        a.observe_map(telemetry(100., stopped=True), 100.)
        saved = [.95, 2.85, .2]
        for index in range(3):
            capture = 100.1 + index * .2
            now = 100.2 + index * .2
            observations(a, stamp=capture, sequence=20 + index, target_id=2, position=[0, 0, .6])
            a.observe_map(telemetry(now, stopped=True), now)
            p = a.reassociate(now, ('live-registry', 1), saved)
            self.assertEqual(p.get('reassociation_verified'), index == 2, p)
            if index == 0:
                repeat = a.reassociate(now + .01, ('live-registry', 1), saved)
                self.assertFalse(repeat['reassociation_verified'])
                self.assertEqual(repeat['observations'], 1)
        self.assertTrue(p['valid'])
        self.assertEqual(p['target_id'], 2)
        self.assertEqual(p['original_identity'], ['live-registry', 1])
        self.assertEqual(len(p['reassociation_exposure_keys']), 3)

    def test_reassociation_rejects_moving_registry_restart_and_old_id_still_alive(self):
        for state in ('moving', 'restart', 'old-alive'):
            a = configured()
            a.observe_map(telemetry(100., stopped=state != 'moving'), 100.)
            observations(a, stamp=100.1, position=[0, 0, .6], target_id=1 if state == 'old-alive' else 2,
                         registry_changes=dict(registry_session='restart') if state == 'restart' else None)
            a.observe_map(telemetry(100.2, stopped=state != 'moving'), 100.2)
            p = a.reassociate(100.2, ('live-registry', 1), [.95, 2.85, .2])
            self.assertFalse(p['reassociation_verified'], state)

    def test_reassociation_rejects_two_close_3d_targets_or_unmeasured_competitor(self):
        for depth_available in (False, True):
            a = configured()
            a.observe_map(telemetry(100., stopped=True), 100.)
            observations(a, stamp=100.1, position=[0, 0, .6], target_id=2)
            extra = dict(target_id=3, selectable=True, bbox_xyxy=[220, 100, 280, 220])
            key = next(reversed(a.targets))
            a.targets[key]['targets'].append(extra)
            a.latest_targets['targets'].append(copy.deepcopy(extra))
            if depth_available:
                a.depth[key]['detections'].append(dict(class_id=0, depth_valid=True,
                    depth_reason='verified_stereo_wallpaper_surface', bbox_xyxy=extra['bbox_xyxy'],
                    position_m=[0, 0, .64]))
            a.observe_map(telemetry(100.2, stopped=True), 100.2)
            p = a.reassociate(100.2, ('live-registry', 1), [.95, 2.85, .2])
            self.assertFalse(p['reassociation_verified'])
            self.assertEqual(p['reason'], 'region_reassociation_not_unique_in_world' if depth_available else
                             'region_reassociation_other_target_depth_unknown')

    def test_template_is_uncommissioned_and_cannot_generate_motion_goal(self):
        a = RegionApproach(ROOT / 'config/region_approach.json', read('fpga_map_catalog.json'),
                           read('semantic_regions.json'), read('extension_map.json'))
        self.assertIn('calibration_verified_required', a.readiness())
        self.assertFalse(a.proposal(100.)['valid'])

    def test_exact_frame_transform_and_same_task_pose_interpolation(self):
        a = matched()
        self.assertEqual(a.readiness(), '')
        proposal = a.proposal(100.4)
        self.assertTrue(proposal['valid'], proposal)
        self.assertAlmostEqual(proposal['fire_world_m'][0], 1.61)
        self.assertAlmostEqual(proposal['fire_world_m'][1], 2.93)
        self.assertAlmostEqual(proposal['fire_world_m'][2], .2)
        self.assertEqual(proposal['pose_registration'], 'same_task_bracket_interpolation')
        self.assertEqual(proposal['registry_session'], 'live-registry')
        self.assertEqual(proposal['target_id'], 1)
        self.assertFalse(proposal['in_spray_range'])
        cell = tuple(proposal['goal_cell'])
        self.assertIn(cell, a.road_cells)
        self.assertIn(cell, a._distances((32, 13), a._grid(100.4)))
        self.assertGreater(proposal['approach_path_length_m'], 0.)

    def test_current_pose_in_verified_range_needs_no_approach(self):
        a = configured()
        a.observe_map(telemetry(100.), 100.)
        observations(a, position=[0, 0, .6])
        a.observe_map(telemetry(100.4), 100.4)
        proposal = a.proposal(100.4)
        self.assertTrue(proposal['valid'], proposal)
        self.assertTrue(proposal['in_spray_range'])
        self.assertEqual(proposal['goal_cell'], [17, 17])
        self.assertEqual(proposal['approach_path_length_m'], 0.)

    def test_heading_rotation_uses_world_y_up(self):
        a = configured(spray_range_m=[.2, .8])
        a.observe_map(telemetry(100., x=1050, y=1990, heading=900), 100.)
        observations(a, position=[0, 0, .3])
        a.observe_map(telemetry(100.4, x=1050, y=1990, heading=900), 100.4)
        proposal = a.proposal(100.4)
        self.assertTrue(proposal['valid'], proposal)
        self.assertAlmostEqual(proposal['fire_world_m'][0], 1.05)
        self.assertAlmostEqual(proposal['fire_world_m'][1], 2.29)

    def test_pose_extrapolation_not_used_for_moving_vehicle(self):
        a = configured()
        a.observe_map(telemetry(100.), 100.)
        observations(a)
        proposal = a.proposal(100.2)
        self.assertFalse(proposal['valid'])
        self.assertEqual(proposal['reason'], 'region_capture_pose_not_time_registered')

    def test_stopped_nearest_pose_requires_two_stationary_samples_and_real_stop(self):
        for stopped in (False, True):
            a = configured()
            a.observe_map(telemetry(100., stopped=stopped), 100.)
            a.observe_map(telemetry(100.2, stopped=stopped), 100.2)
            observations(a, stamp=100.25, position=[0, 0, .6])
            p = a.proposal(100.25)
            self.assertEqual(p['valid'], stopped, p)
            if stopped:
                self.assertEqual(p['pose_registration'], 'verified_stationary_near_capture')

    def test_pose_task_change_cannot_bridge_capture(self):
        a = configured()
        a.observe_map(telemetry(100., task=1), 100.)
        observations(a)
        a.observe_map(telemetry(100.4, task=2), 100.4)
        p = a.proposal(100.4)
        self.assertFalse(p['valid'])
        self.assertEqual(p['reason'], 'region_capture_pose_not_time_registered')

    def test_wrong_frame_sequence_calibration_box_and_depth_are_rejected(self):
        variations = [dict(depth_changes=dict(sequence=13)),
                      dict(depth_changes=dict(capture_stamp_ns=100200000001)),
                      dict(depth_changes=dict(stereo_calibration_id='old-calibration')),
                      dict(depth_changes=dict(position_frame='uncorrected_optical')),
                      dict(depth_changes=dict(coordinate_frame='other-image')),
                      dict(detection_changes=dict(bbox_xyxy=[101, 100, 181, 220])),
                      dict(detection_changes=dict(depth_valid=False)),
                      dict(detection_changes=dict(depth_reason='fixed_distance_fallback')),
                      dict(detection_changes=dict(position_m=[0, 0, -1]))]
        for changes in variations:
            a = configured()
            a.observe_map(telemetry(100.), 100.)
            observations(a, **changes)
            a.observe_map(telemetry(100.4), 100.4)
            self.assertFalse(a.proposal(100.4)['valid'], changes)

    def test_old_or_retired_target_identity_cannot_be_reused(self):
        a = matched()
        observations(a, stamp=100.4, sequence=13, target_id=2)
        self.assertFalse(a.proposal(100.4, ('live-registry', 1))['valid'])
        self.assertFalse(a.proposal(101.1)['valid'])

    def test_signature_dynamic_barriers_and_actual_pose_are_required(self):
        for changes in (dict(map_signature=0), dict(barriers={}), dict(supported=False),
                        dict(pose=dict(stamp=100.4, valid=False))):
            a = matched()
            a.observe_map(telemetry(100.4, **changes), 100.4)
            self.assertFalse(a.proposal(100.4)['valid'], changes)

    def test_occupied_or_disconnected_standoff_is_never_selected(self):
        a = matched()
        # Artificial topology limitation isolates candidates behind a barrier.
        a.road_cells = {(52, 60)}
        p = a.proposal(100.4)
        self.assertFalse(p['valid'])
        self.assertEqual(p['reason'], 'region_no_reachable_standoff_in_measured_spray_range')
        a = matched()
        a.observe_map(telemetry(100.4, barriers=dict(stamp=100.4, valid=True,
                                                  generation=2, segments_mm=[[350, 200, 350, 500]])), 100.4)
        p = a.proposal(100.4)
        self.assertFalse(p['valid'])
        self.assertEqual(p['reason'], 'region_current_pose_not_in_clear_map_corridor')

    def test_outside_region_has_verified_exclusion_without_motion_goal(self):
        a = configured()
        a.observe_map(telemetry(100.), 100.)
        observations(a, position=[0, 0, 2.])
        a.observe_map(telemetry(100.4), 100.4)
        p = a.proposal(100.4)
        self.assertFalse(p['valid'])
        self.assertNotIn('goal_cell', p)
        self.assertEqual(p['excluded_targets'][0]['target_id'], 1)
        self.assertTrue(p['excluded_targets'][0]['map_verified'])

    def test_changed_grid_data_or_reflection_cannot_pass_local_readiness(self):
        occupancy = read('region_traversability.json')
        occupancy['free_runs'][0][2] -= 1
        a = configured(traversability=occupancy)
        self.assertEqual(a.readiness(), 'region_approach_traversability_hash_mismatch')
        a = configured(R_camera_to_vehicle=[[-1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.assertEqual(a.readiness(), 'region_approach_rotation_reflection_invalid')

    def test_exported_grid_hash_matches_actual_fpga_planner_rom(self):
        source = ROOT.parents[1] / 'ov5640/src/navigation/map_plan_2cm.mem'
        data = source.read_bytes()
        t = read('region_traversability.json')
        self.assertEqual(hashlib.sha256(data).hexdigest(), t['planner_mem_sha256'])
        self.assertEqual(hashlib.sha256(bytes(int(v, 16) for v in data.split())).hexdigest(), t['cells_sha256'])

    def test_exported_heading_samples_keep_rom_order_and_legal_corner_tangents(self):
        fpga = ROOT.parents[1] / 'ov5640/src/navigation'
        t = read('region_traversability.json')
        samples = t['start_heading_samples']
        self.assertEqual([s['node'] for s in samples], sorted(s['node'] for s in samples))
        self.assertEqual(len(samples), len({s['node'] for s in samples}))
        heading_bytes = (fpga / 'nav_centerline_heading.vh').read_bytes()
        self.assertEqual(hashlib.sha256(heading_bytes).hexdigest(), t['heading_rom_sha256'])
        self.assertEqual(hashlib.sha256(json.dumps(samples, separators=(',', ':'), ensure_ascii=True).encode('ascii')).hexdigest(),
                         t['start_heading_samples_sha256'])
        headings = {int(n): int(word, 16) for n, word in re.findall(
            r"start_tangent\[(\d+)\]\s*=\s*9'h([0-9a-fA-F]+)", heading_bytes.decode('utf-8'))}
        for sample in samples:
            raw = headings[sample['node']]
            expected = raw - 512 if raw & 256 else raw
            self.assertNotEqual(expected, 255)
            self.assertEqual(sample['heading_deg'], expected)
            self.assertEqual(sample['cell'], [v >> 9 for v in sample['xy_q8']])
        for point, expected_heading in (([17, 17], 37), ([17, 142], 127),
                                        ([158, 142], -135), ([158, 17], -45)):
            query = [v * 512 + 256 for v in point]
            chosen = min(samples, key=lambda s: sum(abs(a - b) for a, b in zip(s['xy_q8'], query)))
            self.assertEqual(chosen['heading_deg'], expected_heading)


if __name__ == '__main__':
    unittest.main()
