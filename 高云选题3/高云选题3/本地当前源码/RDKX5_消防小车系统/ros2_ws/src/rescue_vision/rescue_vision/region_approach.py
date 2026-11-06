"""Measured stereo/map projection and reachable road standoff selection.

No guessed depth, camera mounting or nozzle reach is used.  Configuration is
deliberately uncommissioned.  The FPGA still performs every actual A* plan;
this module merely selects a connected free goal on its existing road map.
"""
from collections import OrderedDict, deque
import copy
import hashlib
import json
import math
from pathlib import Path

from .region_search import _number, _cell, load_catalog
from .route_fire_patrol import fresh


def _document(value):
    return (json.loads(Path(value).read_text(encoding='utf-8-sig'))
            if isinstance(value, (str, Path)) else copy.deepcopy(value))


def _vector(value, length=3):
    return isinstance(value, (list, tuple)) and len(value) == length and all(_number(x) for x in value)


def _frame_key(document):
    key = (document.get('capture_stamp_ns'), document.get('sequence'))
    if any(type(value) is not int or value < 0 for value in key):
        raise ValueError('exact_capture_frame_key_required')
    return key


def _angle(a, b):
    return (b - a + 180.) % 360. - 180.


def _rotate(vector, heading):
    cosine, sine = math.cos(math.radians(heading)), math.sin(math.radians(heading))
    return [cosine * vector[0] - sine * vector[1],
            sine * vector[0] + cosine * vector[1], vector[2]]


def _segment_distance(x, y, line):
    x1, y1, x2, y2 = line
    dx, dy = x2 - x1, y2 - y1
    t = max(0., min(1., ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy))) if dx or dy else 0.
    return math.hypot(x - (x1 + t * dx), y - (y1 + t * dy))


class RegionApproach:
    def __init__(self, config, map_catalog, region_catalog, road_config):
        source_path = Path(config) if isinstance(config, (str, Path)) else None
        self.config = _document(config)
        self.map_catalog = _document(map_catalog)
        self.regions = load_catalog(_document(region_catalog))
        self.roads = _document(road_config)
        occupancy = self.config.get('traversability')
        if isinstance(occupancy, str) and source_path and not Path(occupancy).is_absolute():
            occupancy = source_path.parent / occupancy
        try:
            self.traversability = _document(occupancy) if occupancy is not None else {}
        except (OSError, ValueError):
            self.traversability = {}
        self.region_id = None
        self.targets, self.depth = OrderedDict(), OrderedDict()
        self.latest_targets, self.map = {}, {}
        self.poses = deque(maxlen=64)
        self.free, self.road_cells = set(), set()
        self._grid_cache = self._bfs_cache = None
        self._proposal_cache = None
        self._reassociation = None
        self.stopped_since = None
        self._configuration_reason = self._validate()

    def _validate(self):
        c, t, m, r = self.config, self.traversability, self.map_catalog, self.regions
        if c.get('schema_version') != 1:
            return 'region_approach_configuration_invalid'
        for field in ('calibration_verified', 'map_registration_verified', 'spray_range_verified',
                      'body_envelope_verified', 'time_sync_verified'):
            if c.get(field) is not True:
                return 'region_approach_' + field + '_required'
        if (c.get('reference') != 'nose_midpoint' or c.get('position_frame') not in
                ('rectified_left_optical', 'undistorted_left_optical')):
            return 'region_approach_reference_frame_invalid'
        if (not isinstance(c.get('camera_coordinate_frame'), str) or not c['camera_coordinate_frame'] or
                not isinstance(c.get('stereo_calibration_id'), str) or not c['stereo_calibration_id']):
            return 'region_approach_camera_calibration_identity_required'
        matrix = c.get('R_camera_to_vehicle')
        if (not isinstance(matrix, list) or len(matrix) != 3 or any(not _vector(row) for row in matrix) or
                any(abs(sum(matrix[i][k] * matrix[j][k] for k in range(3)) - (1. if i == j else 0.)) > 1e-5
                    for i in range(3) for j in range(3))):
            return 'region_approach_rotation_not_orthonormal'
        determinant = (matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
                       matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
                       matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]))
        if abs(determinant - 1.) > 1e-5:
            return 'region_approach_rotation_reflection_invalid'
        if any(not _vector(c.get(field)) or math.dist(c[field], [0., 0., 0.]) > 1.
               for field in ('t_camera_to_vehicle_m', 'nozzle_origin_vehicle_m')):
            return 'region_approach_measured_mount_translation_required'
        reach, height = c.get('spray_range_m'), c.get('fire_height_range_m')
        if not _vector(reach, 2) or not .1 <= reach[0] < reach[1] <= 5.:
            return 'region_approach_measured_spray_range_required'
        if not _vector(height, 2) or not 0 <= height[0] < height[1] <= 4.:
            return 'region_approach_measured_target_height_required'
        if not _number(c.get('body_clearance_m')) or not .01 <= c['body_clearance_m'] <= .5:
            return 'region_approach_measured_body_clearance_required'
        for field, low, high in (('capture_ttl_s', .1, .6), ('max_pose_gap_s', .05, .8),
                                 ('stopped_pose_slop_s', .01, .15), ('pose_speed_limit_mps', .01, 2.),
                                 ('pose_turn_limit_deg_s', 5., 360.)):
            if not _number(c.get(field)) or not low <= c[field] <= high:
                return 'region_approach_invalid_' + field
        if (c.get('map_sha256') != r['map_sha256'] or self.roads.get('map_sha256') != r['map_sha256'] or
                t.get('map_sha256') != r['map_sha256'] or t.get('map_signature') != m.get('map_signature') or
                c.get('map_signature') != m.get('map_signature') or
                t.get('raw_map_sha256') != m.get('source_hashes', {}).get('src/navigation/map_raw_1cm.mem') or
                c.get('planner_cells_sha256') != t.get('cells_sha256') or
                c.get('planner_graph_sha256') != t.get('graph_rom_sha256') or
                c.get('planner_heading_sha256') != t.get('heading_rom_sha256')):
            return 'region_approach_local_map_identity_mismatch'
        if (t.get('schema_version') != 1 or t.get('width_cells') != 176 or t.get('height_cells') != 160 or
                t.get('cell_cm') != 2 or t.get('free_value') != 0 or t.get('reference') != 'nose_midpoint' or
                not _number(t.get('planner_inflation_m')) or not 0 < t['planner_inflation_m'] <= .5):
            return 'region_approach_traversability_invalid'
        cells = bytearray([1] * (176 * 160))
        for run in t.get('free_runs', []):
            if (not isinstance(run, list) or len(run) != 3 or any(type(x) is not int for x in run) or
                    not 0 <= run[0] < 160 or not 0 <= run[1] < 176 or not 1 <= run[2] <= 176 - run[1]):
                return 'region_approach_traversability_run_invalid'
            start, end = run[0] * 176 + run[1], run[0] * 176 + run[1] + run[2]
            if any(x == 0 for x in cells[start:end]):
                return 'region_approach_traversability_overlapping_runs'
            cells[start:end] = bytes(run[2])
        if hashlib.sha256(cells).hexdigest() != t.get('cells_sha256'):
            return 'region_approach_traversability_hash_mismatch'
        free = {(i % 176, i // 176) for i, value in enumerate(cells) if value == 0}
        extra = max(0., c['body_clearance_m'] - t['planner_inflation_m'])
        if extra:
            radius = math.ceil(extra / .02 + 1.)
            offsets = [(x, y) for x in range(-radius, radius + 1) for y in range(-radius, radius + 1)
                       if math.hypot(x, y) * .02 <= extra + math.sqrt(2.) * .01]
            free = {cell for cell in free if all((cell[0] + x, cell[1] + y) in free for x, y in offsets)}
        self.free = free
        graph_cells = t.get('graph_cells')
        if (not isinstance(graph_cells, list) or not graph_cells or
                any(not _cell(cell) for cell in graph_cells) or
                hashlib.sha256(bytes(value for cell in graph_cells for value in cell)).hexdigest() !=
                t.get('graph_cells_sha256')):
            return 'region_approach_execution_graph_cells_invalid'
        self.road_cells = {tuple(cell) for cell in graph_cells if tuple(cell) in free}
        return '' if self.road_cells else 'region_approach_no_clear_road_candidates'

    def readiness(self):
        return self._configuration_reason

    def set_region(self, region_id):
        if region_id not in self.regions['regions']:
            raise ValueError('unknown_semantic_region')
        self.region_id = region_id
        self._reassociation = None

    def _observe_frame(self, store, document, now):
        if not isinstance(document, dict):
            return
        try:
            key = _frame_key(document)
        except ValueError:
            return
        stamp = key[0] / 1e9
        if not 0 <= now - stamp <= .6:
            return
        store[key] = copy.deepcopy(document)
        while len(store) > 32:
            store.popitem(last=False)
        for old in list(store):
            if now - old[0] / 1e9 > .8:
                store.pop(old)

    def observe_targets(self, document, now):
        self.latest_targets = copy.deepcopy(document) if isinstance(document, dict) else {}
        self._observe_frame(self.targets, document, now)

    def observe_depth(self, document, now):
        self._observe_frame(self.depth, document, now)

    def observe_map(self, telemetry, now):
        # Geometry needs pose/obstacles, not the potentially 2048-point patrol
        # path. Avoid copying that immutable path on every serial-owner tick.
        self.map = copy.deepcopy({key: telemetry[key] for key in
            ('stamp', 'supported', 'map_signature', 'stopped', 'stopped_stamp', 'pose', 'barriers')
            if key in telemetry}) if isinstance(telemetry, dict) else {}
        if self.map.get('stopped') is True and fresh(self.map, now, .6, 'stopped_stamp'):
            if self.stopped_since is None:
                self.stopped_since = self.map['stopped_stamp']
        else:
            self.stopped_since = None
            self._reassociation = None
        pose = self.map.get('pose', {})
        if (self.map.get('supported') is not True or self.map.get('map_signature') != self.map_catalog.get('map_signature') or
                not fresh(self.map, now, 3.) or not self._valid_pose(pose, now)):
            return
        if self.poses and pose['stamp'] <= self.poses[-1]['stamp']:
            return
        if self.poses and not self._same_pose_epoch(self.poses[-1], pose):
            # A route replan may reuse the 8-bit task ID. Never interpolate a
            # camera exposure across its old coordinate/reference generation.
            self.poses.clear()
            self._reassociation = self._proposal_cache = None
            self.stopped_since = (self.map['stopped_stamp'] if self.map.get('stopped') is True
                and fresh(self.map, now, .6, 'stopped_stamp') else None)
        self.poses.append(copy.deepcopy(pose))

    @staticmethod
    def _valid_pose(pose, now):
        return (isinstance(pose, dict) and pose.get('valid') is True and fresh(pose, now, .8) and
                _number(pose.get('x_mm')) and 0 <= pose['x_mm'] < 3520 and
                _number(pose.get('y_mm')) and 0 < pose['y_mm'] <= 3200 and
                _number(pose.get('heading_tenths')) and -1800 <= pose['heading_tenths'] <= 1800 and
                pose.get('mode') == 9 and pose.get('status') in (2, 3) and
                # Bit 0 is the FPGA's coherent pose-health decision. Bit 1 is
                # raw IMU availability only: a bounded healthy inertial bridge
                # can legitimately clear it. Older firmware already gates bit
                # 0 on raw availability, so this preserves its stricter policy.
                type(pose.get('flags')) is int and pose['flags'] & 1 == 1 and
                ('route_generation' not in pose or type(pose['route_generation']) is int
                 and 1 <= pose['route_generation'] <= 65534) and
                type(pose.get('task_id')) is int and 0 <= pose['task_id'] < 256)

    @staticmethod
    def _same_pose_epoch(a, b):
        # Legacy telemetry omits generation on both sides. A legacy/current
        # schema transition is also a boundary, never a wildcard match.
        return (a.get('task_id') == b.get('task_id') and
                a.get('route_generation') == b.get('route_generation'))

    def _pose_for_capture(self, stamp, now):
        latest = self.map.get('pose', {})
        if not self._valid_pose(latest, now):
            return None
        poses = [p for p in self.poses if self._same_pose_epoch(p, latest) and now - p['stamp'] <= 1.2]
        before = next((p for p in reversed(poses) if p['stamp'] <= stamp), None)
        after = next((p for p in poses if p['stamp'] >= stamp), None)
        if before and after:
            gap = after['stamp'] - before['stamp']
            distance = math.hypot(after['x_mm'] - before['x_mm'], after['y_mm'] - before['y_mm']) / 1000.
            turn = _angle(before['heading_tenths'] / 10., after['heading_tenths'] / 10.)
            if gap == 0.:
                return dict(x=before['x_mm'] / 1000., y=before['y_mm'] / 1000.,
                            heading=before['heading_tenths'] / 10., method='capture_latched_pose')
            if (gap <= self.config['max_pose_gap_s'] and distance / gap <= self.config['pose_speed_limit_mps'] and
                    abs(turn / gap) <= self.config['pose_turn_limit_deg_s']):
                ratio = (stamp - before['stamp']) / gap
                return dict(x=(before['x_mm'] + ratio * (after['x_mm'] - before['x_mm'])) / 1000.,
                            y=(before['y_mm'] + ratio * (after['y_mm'] - before['y_mm'])) / 1000.,
                            heading=(before['heading_tenths'] / 10. + ratio * turn + 180.) % 360. - 180.,
                            method='same_task_bracket_interpolation')
        # A one-sided pose is permitted only with actual fresh stopped telemetry
        # plus two stationary encoder samples; a command ACK is insufficient.
        if (self.map.get('stopped') is True and fresh(self.map, now, .6, 'stopped_stamp') and len(poses) >= 2 and
                abs(stamp - poses[-1]['stamp']) <= self.config['stopped_pose_slop_s'] and
                .1 <= poses[-1]['stamp'] - poses[-2]['stamp'] <= self.config['max_pose_gap_s'] and
                math.hypot(poses[-1]['x_mm'] - poses[-2]['x_mm'], poses[-1]['y_mm'] - poses[-2]['y_mm']) <= 2. and
                abs(_angle(poses[-2]['heading_tenths'] / 10., poses[-1]['heading_tenths'] / 10.)) <= .2):
            p = poses[-1]
            return dict(x=p['x_mm'] / 1000., y=p['y_mm'] / 1000., heading=p['heading_tenths'] / 10.,
                        method='verified_stationary_near_capture')
        return None

    def _grid(self, now):
        barriers = self.map.get('barriers', {})
        if barriers.get('valid') is not True or not fresh(barriers, now, 2.):
            return None
        segments = barriers.get('segments_mm')
        if (not isinstance(segments, list) or len(segments) > 4 or
                any(not _vector(line, 4) or any(not 0 <= v <= (3520 if i % 2 == 0 else 3200)
                    for i, v in enumerate(line)) for line in segments)):
            return None
        key = tuple(tuple(segment) for segment in segments)
        if self._grid_cache and self._grid_cache[0] == key:
            return self._grid_cache[1]
        radius_mm = max(110., self.config['body_clearance_m'] * 1000.)
        free = {cell for cell in self.free if all(_segment_distance(20 * cell[0] + 10, 20 * cell[1] + 10, line) > radius_mm
                                                for line in segments)}
        self._grid_cache, self._bfs_cache = (key, free), None
        return free

    def _distances(self, start, free):
        if self._bfs_cache and self._bfs_cache[0] == start:
            return self._bfs_cache[1]
        queue, distances = deque([start]), {start: 0}
        while queue:
            x, y = queue.popleft()
            for cell in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if cell in free and cell not in distances:
                    distances[cell] = distances[(x, y)] + 1
                    queue.append(cell)
        self._bfs_cache = (start, distances)
        return distances

    def _inside(self, world):
        bounds = self.regions['regions'][self.region_id]['bounds_image_cm']
        return bounds[0] <= world[0] * 100. < bounds[2] and bounds[1] <= 320. - world[1] * 100. < bounds[3]

    def _select_goal(self, world, current, now):
        free = self._grid(now)
        if free is None:
            return None, 'region_dynamic_barriers_unverified'
        start = (int(current['x_mm'] / 20.), int((3200. - current['y_mm']) / 20.))
        if start not in free:
            return None, 'region_current_pose_not_in_clear_map_corridor'
        distances = self._distances(start, free)
        low, high = self.config['spray_range_m']
        nozzle = self.config['nozzle_origin_vehicle_m']
        rotated = _rotate(nozzle, current['heading_tenths'] / 10.)
        current_nozzle = [current['x_mm'] / 1000. + rotated[0], current['y_mm'] / 1000. + rotated[1], rotated[2]]
        current_distance = math.dist(current_nozzle, world)
        if low <= current_distance <= high:
            return dict(goal_cell=list(start), in_spray_range=True, distance_m=current_distance,
                        approach_path_length_m=0.), ''
        # The final arrival heading is chosen by FPGA A*. Candidate distance is
        # bounded over all headings of the measured nozzle offset. Actual range
        # and target identity must be rechecked after the vehicle has stopped.
        offset = math.hypot(nozzle[0], nozzle[1])
        candidates = []
        for cell in self.road_cells:
            if cell not in distances:
                continue
            position = [(cell[0] * 20. + 10.) / 1000., (3200. - (cell[1] * 20. + 10.)) / 1000., 0.]
            if not self._inside(position):
                continue
            planar = math.hypot(position[0] - world[0], position[1] - world[1])
            dz = world[2] - nozzle[2]
            lower = math.hypot(max(0., planar - offset), dz)
            upper = math.hypot(planar + offset, dz)
            # Reserve 8 cm for the FPGA's 6 cm terminal window and 2 cm raster.
            # Range is still measured again after the actual stop.
            if low + .08 <= lower and upper <= high - .08:
                path = distances[cell] * .02
                candidates.append((path + .2 * abs((lower + upper) / 2. - (low + high) / 2.), cell, path))
        if not candidates:
            return None, 'region_no_reachable_standoff_in_measured_spray_range'
        _, cell, path = min(candidates)
        return dict(goal_cell=list(cell), in_spray_range=False, distance_m=current_distance,
                    approach_path_length_m=path), ''

    def proposal(self, now, preferred_identity=None):
        result = dict(stamp=now, valid=False, reason=self.readiness() or 'region_target_depth_unavailable',
                      calibration_verified=False, map_verified=False, clearance_verified=False,
                      map_sha256=self.regions['map_sha256'], region_id=self.region_id,
                      excluded_targets=[], mapped_targets=[])
        if self.readiness():
            return result
        if self.region_id not in self.regions['regions']:
            result['reason'] = 'region_not_selected'
            return result
        if (self.map.get('supported') is not True or not fresh(self.map, now, 3.) or
                self.map.get('map_signature') != self.map_catalog.get('map_signature')):
            result['reason'] = 'region_live_map_signature_unverified'
            return result
        current = self.map.get('pose', {})
        if not self._valid_pose(current, now):
            result['reason'] = 'region_current_pose_unverified'
            return result
        latest = self.latest_targets
        if (latest.get('frame_valid') is not True or not fresh(latest, now, self.config['capture_ttl_s'], 'capture_stamp') or
                latest.get('coordinate_frame') != self.config['camera_coordinate_frame']):
            result['reason'] = 'region_current_detector_frame_unavailable'
            return result
        alive = {(latest.get('registry_session'), target.get('target_id')) for target in latest.get('targets', [])
                 if isinstance(target, dict) and target.get('selectable') is True}
        if self._grid(now) is None:
            result['reason'] = 'region_dynamic_barriers_unverified'
            return result
        cache_key = (self.region_id, tuple(preferred_identity) if preferred_identity else None,
                     latest.get('capture_stamp_ns'), latest.get('sequence'), tuple(sorted(alive)),
                     next(reversed(self.depth), None), current['stamp'], current['x_mm'], current['y_mm'],
                     current['heading_tenths'], current['task_id'], current.get('route_generation'), self._grid_cache[0],
                     self.map.get('stopped'), self.map.get('stopped_stamp'))
        if self._proposal_cache and self._proposal_cache[0] == cache_key:
            cached = self._proposal_cache[1]
            # Refresh only presentation time, never the captured evidence time.
            # Geometry can expire sooner than the newest visible detector frame.
            if (cached.get('valid') is not True or
                    fresh(cached, now, self.config['capture_ttl_s'], 'capture_stamp')):
                result = copy.deepcopy(cached)
                result['stamp'] = now
                return result
        for key in sorted(set(self.targets) & set(self.depth), reverse=True):
            capture = key[0] / 1e9
            if not 0 <= now - capture <= self.config['capture_ttl_s']:
                continue
            target_doc, depth_doc = self.targets[key], self.depth[key]
            if (target_doc.get('frame_valid') is not True or
                    depth_doc.get('coordinate_frame') != self.config['camera_coordinate_frame'] or
                    target_doc.get('coordinate_frame') != depth_doc.get('coordinate_frame') or
                    depth_doc.get('position_frame') != self.config['position_frame'] or
                    depth_doc.get('stereo_calibration_id') != self.config['stereo_calibration_id']):
                result['reason'] = 'region_stereo_frame_or_calibration_mismatch'
                continue
            pose = self._pose_for_capture(capture, now)
            if pose is None:
                result['reason'] = 'region_capture_pose_not_time_registered'
                continue
            for target in target_doc.get('targets', []):
                if not isinstance(target, dict):
                    continue
                identity = (target_doc.get('registry_session'), target.get('target_id'))
                if (identity not in alive or target.get('selectable') is not True or
                        (preferred_identity is not None and tuple(preferred_identity) != identity) or
                        not _vector(target.get('bbox_xyxy'), 4)):
                    continue
                matches = [d for d in depth_doc.get('detections', []) if isinstance(d, dict) and
                           d.get('class_id') == 0 and d.get('depth_valid') is True and
                           d.get('depth_reason') == 'verified_stereo_wallpaper_surface' and
                           _vector(d.get('bbox_xyxy'), 4) and
                           all(abs(a - b) <= .01 for a, b in zip(d['bbox_xyxy'], target['bbox_xyxy'])) and
                           _vector(d.get('position_m')) and .1 <= d['position_m'][2] <= 5.]
                if len(matches) != 1:
                    result['reason'] = 'region_exact_target_depth_match_missing_or_ambiguous'
                    continue
                camera = matches[0]['position_m']
                c = self.config
                vehicle = [sum(c['R_camera_to_vehicle'][i][j] * camera[j] for j in range(3)) +
                           c['t_camera_to_vehicle_m'][i] for i in range(3)]
                rotated = _rotate(vehicle, pose['heading'])
                world = [pose['x'] + rotated[0], pose['y'] + rotated[1], rotated[2]]
                if not c['fire_height_range_m'][0] <= world[2] <= c['fire_height_range_m'][1]:
                    result['reason'] = 'region_fire_height_outside_measured_geometry'
                    continue
                result['mapped_targets'].append(dict(capture_stamp=capture,
                    registry_session=identity[0], target_id=identity[1], fire_world_m=world,
                    calibration_verified=True, map_verified=True, map_sha256=self.regions['map_sha256']))
                if not self._inside(world):
                    result['reason'] = 'region_fire_outside_selected_region'
                    result['excluded_targets'].append(dict(capture_stamp=capture,
                        registry_session=identity[0], target_id=identity[1], fire_world_m=world,
                        calibration_verified=True, map_verified=True, map_sha256=self.regions['map_sha256']))
                    continue
                goal, reason = self._select_goal(world, current, now)
                if goal is None:
                    result['reason'] = reason
                    continue
                result = dict(result, **goal, valid=True, reason='verified_mapped_standoff',
                            capture_stamp=capture, capture_stamp_ns=key[0], sequence=key[1],
                            registry_session=identity[0], target_id=identity[1],
                            fire_world_m=world, calibration_verified=True, map_verified=True,
                            clearance_verified=True, pose_registration=pose['method'],
                            coordinate_frame='fpga_world_m_x_right_y_up_z_up',
                            reachability_method='inflated_grid_connectivity_then_fpga_astar')
                self._proposal_cache = (cache_key, copy.deepcopy(result))
                return result
        self._proposal_cache = (cache_key, copy.deepcopy(result))
        return result

    def reassociate(self, now, old_identity, target_world):
        """Reacquire a lost visual ID at a stopped standoff, by unique 3D evidence.

        Three distinct depth exposures must each identify one target within
        8 cm of the saved fire. Any other selectable fire lacking mapped depth
        is ambiguous, as is a second mapped fire within 16 cm. This method does
        not authorize an active spray to switch targets.
        """
        result = dict(stamp=now, valid=False, reassociation_verified=False,
                      reason='region_reassociation_not_ready')

        def reject(reason):
            self._reassociation = None
            return dict(result, reason=reason)

        if (not isinstance(old_identity, (list, tuple)) or len(old_identity) != 2 or
                not _vector(target_world)):
            return reject('region_reassociation_reference_invalid')
        old_identity = tuple(old_identity)
        view, pose = self.latest_targets, self.map.get('pose', {})
        if (view.get('frame_valid') is not True or not fresh(view, now, .6, 'capture_stamp') or
                view.get('registry_session') != old_identity[0]):
            return reject('region_reassociation_registry_changed_or_stale')
        ids = [(view.get('registry_session'), target.get('target_id')) for target in view.get('targets', [])
               if isinstance(target, dict) and target.get('selectable') is True and
               type(target.get('target_id')) is int and target['target_id'] > 0]
        if old_identity in ids:
            return reject('region_original_fire_identity_still_visible')
        same_task = [p for p in self.poses if self._same_pose_epoch(p, pose)]
        if (self.map.get('stopped') is not True or not fresh(self.map, now, .6, 'stopped_stamp') or
                not self._valid_pose(pose, now) or len(same_task) < 2 or
                self.stopped_since is None or same_task[-2]['stamp'] < self.stopped_since or
                not .1 <= same_task[-1]['stamp'] - same_task[-2]['stamp'] <= self.config.get('max_pose_gap_s', 0.) or
                math.hypot(same_task[-1]['x_mm'] - same_task[-2]['x_mm'],
                           same_task[-1]['y_mm'] - same_task[-2]['y_mm']) > 2. or
                abs(_angle(same_task[-2]['heading_tenths'] / 10., same_task[-1]['heading_tenths'] / 10.)) > .2):
            return reject('region_reassociation_requires_verified_stationary_pose')
        contenders = []
        for identity in ids:
            proposal = self.proposal(now, identity)
            mapped = (proposal.get('fire_world_m') if proposal.get('valid') is True else
                      next((p.get('fire_world_m') for p in proposal.get('mapped_targets', [])
                            if (p.get('registry_session'), p.get('target_id')) == identity), None))
            if not _vector(mapped):
                return reject('region_reassociation_other_target_depth_unknown')
            contenders.append((math.dist(mapped, target_world), identity, proposal))
        contenders.sort(key=lambda item: item[0])
        if (not contenders or contenders[0][0] > .08 or
                (len(contenders) > 1 and contenders[1][0] < .16)):
            return reject('region_reassociation_not_unique_in_world')
        distance, identity, proposal = contenders[0]
        if proposal.get('valid') is not True or proposal.get('in_spray_range') is not True:
            return reject('region_reassociation_spray_range_unverified')
        if proposal['capture_stamp'] < self.stopped_since:
            return reject('region_reassociation_capture_precedes_verified_stop')
        key = (proposal['capture_stamp_ns'], proposal['sequence'])
        owner = (old_identity, tuple(target_world), identity)
        memory = self._reassociation
        if memory is None or memory['owner'] != owner or key[0] - memory['keys'][-1][0] > 450000000:
            memory = dict(owner=owner, keys=[key])
        elif key != memory['keys'][-1]:
            if key[0] <= memory['keys'][-1][0] or key[1] <= memory['keys'][-1][1]:
                return reject('region_reassociation_exposure_regressed')
            memory['keys'].append(key)
            memory['keys'] = memory['keys'][-3:]
        self._reassociation = memory
        if len(memory['keys']) < 3 or any(now - exposure[0] / 1e9 > .6 for exposure in memory['keys']):
            return dict(result, reason='region_reassociation_waiting_three_depth_exposures',
                        observations=len(memory['keys']), proposed_target_id=identity[1])
        return dict(proposal, reassociation_verified=True, original_identity=list(old_identity),
                    reassociation_stationary_verified=True, reassociation_exposure_keys=list(memory['keys']),
                    reassociation_stopped_since=self.stopped_since,
                    reassociation_distance_m=distance,
                    reassociation_second_distance_m=contenders[1][0] if len(contenders) > 1 else None,
                    reason='verified_stationary_three_frame_world_reassociation')
