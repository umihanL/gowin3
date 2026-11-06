"""Pure coordinator for mapped semantic-region search.

The FPGA remains responsible for A* and inertial driving.  Every navigation
effect starts from its measured pose; only the first leg carries the operator's
start reference.  This module neither writes a serial port nor energizes GPIO.

``tick`` receives already decoded navigation feedback, fresh detector targets,
automatic-fire feedback, actual actuator feedback, and an optional approach
proposal from the calibrated geometry adapter.  A proposal is not a pixel-box
heuristic: ``calibration_verified``, ``map_verified`` and
``clearance_verified`` certify camera/map registration, the matching map, and a
free standoff goal. ``in_spray_range`` must be established at the current pose.
The adapter must never populate those flags from configuration alone.
"""
import hashlib
import json
import math
from copy import deepcopy
from pathlib import Path

from .route_fire_patrol import fresh, off_disarmed, navigation_off_disarmed
from .checkpoint_alignment import MIN_FIRE_CONFIDENCE, confident_fire
from .fire_retrigger import FireRetriggerGuard
from .flame_centering import FlameCentering
from .flame_stop_trigger import FlameStopTrigger

# 6 cm FPGA terminal goal radius plus raster quantization. Distance alone
# never declares arrival: exact leg DONE, fresh pose and stop are mandatory.
ARRIVAL_TOLERANCE_CELLS = 4
MAX_FIRE_OBSERVATIONS = 4096


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _cell(value, width=176, height=160):
    return (isinstance(value, (list, tuple)) and len(value) == 2 and
            all(type(x) is int for x in value) and
            0 <= value[0] < width and 0 <= value[1] < height)


def load_catalog(source):
    """Validate and detach a catalog; malformed regions never become routes."""
    if isinstance(source, (str, Path)):
        source = json.loads(Path(source).read_text(encoding='utf-8'))
    value = json.loads(json.dumps(source))
    if (value.get('schema_version') != 1 or value.get('cell_cm') != 2 or
            value.get('width_cm') != 352 or value.get('height_cm') != 320 or
            not isinstance(value.get('map_sha256'), str) or
            len(value['map_sha256']) != 64 or
            set(value.get('regions', {})) != {'A', 'B', 'C', 'D'}):
        raise ValueError('invalid_semantic_region_catalog')
    for name, region in value['regions'].items():
        bounds, points = region.get('bounds_image_cm'), region.get('waypoints')
        if (not isinstance(bounds, list) or len(bounds) != 4 or
                any(not _number(x) for x in bounds) or
                not 0 <= bounds[0] < bounds[2] <= 352 or
                not 0 <= bounds[1] < bounds[3] <= 320 or
                not isinstance(points, list) or not 1 <= len(points) <= 4):
            raise ValueError('invalid_semantic_region:' + name)
        seen = set()
        for point in points:
            if not isinstance(point, dict) or not _cell(point.get('cell')):
                raise ValueError('invalid_semantic_waypoint:' + name)
            identifier = point.get('id')
            x, y = (2 * n + 1 for n in point['cell'])
            if (not isinstance(identifier, str) or not identifier or identifier in seen or
                    not bounds[0] <= x < bounds[2] or not bounds[1] <= y < bounds[3]):
                raise ValueError('semantic_waypoint_outside_region:' + name)
            seen.add(identifier)
    return value


class RegionSearch:
    """Bounded search FSM; methods return ``[(effect_name, payload), ...]``.

    Navigation input: ``stamp, command_id, phase, stopped, pose_valid,
    pose_cell, arrived``. Arrival must represent a correlated FPGA completion
    event plus fresh stopped/pose feedback, not merely an accepted command.

    Proposal input: ``stamp, capture_stamp, registry_session, target_id,
    valid, calibration_verified, map_verified, clearance_verified,
    map_sha256, fire_world_m, goal_cell, in_spray_range``. Coordinates are
    metres, x right and y up from the bottom left of the registered map.
    """

    def __init__(self, catalog, route_planner=None, alignment_config=None):
        self.catalog = load_catalog(catalog)
        self.checkpoint = None
        self.configure_checkpoint_alignment(alignment_config)
        self.route_planner = route_planner
        self.route_policy = {}
        self.continuous_route = False
        self.exit = None
        self.active = False
        self.phase, self.reason = 'IDLE', 'not_requested'
        self.command_id = self.region_id = self.leg_id = self.leg_kind = None
        self.target = self.run_id = None
        self.execute_fire = False
        self.fire_observed_at = None
        self.waypoints = []
        self.visited = []
        self.rounds = self.leg_counter = 0
        self.started_at = self.phase_at = 0.
        self.last_capture = None
        self.visual_degraded = False
        self.visual_degraded_since = None
        self.visual_recovery_reason = None
        self.visual_recovered_at = None
        self.visual_coverage_incomplete = False
        self.candidate = None
        self.hits = self.observation_frames = 0
        self.handled = set()
        self.excluded = set()
        self.handled_world = []
        self.target_world = None
        self.goal_cell = None
        self.start_reference = None
        self.last_navigation = {}
        self.navigation_missing_since = None
        self.observation_dwell_at = None
        self.geometry_reason = None
        self.geometry_wait_target = None
        self.failure_detail = None
        self.last_fire_trigger = None
        self.consumed_motion_exhaustions = set()
        self.effects = []
        self._reset_fire_statistics()

    def configure_checkpoint_alignment(self, config):
        from .checkpoint_alignment import CheckpointAlignment
        self.checkpoint = CheckpointAlignment(config) if isinstance(config, dict) else None
        self.fire_retrigger = FireRetriggerGuard(config)
        self.centering = FlameCentering(config)
        self.stop_trigger = FlameStopTrigger(config)

    def _checkpoint_enabled(self):
        return bool(self.checkpoint and self.checkpoint.enabled)

    @property
    def min_fire_confidence(self):
        return self.checkpoint.config['min_confidence'] if self.checkpoint else MIN_FIRE_CONFIDENCE

    def _reset_fire_statistics(self):
        self.observed_identities = set()
        self.visible_identities = set()
        self.visible_capture = None
        self.max_visible = 0
        self.mapped_identities = set()
        self.mapped_world = []
        self.statistics_truncated = False
        self.skipped_observations = []

    def _map_fire_statistics(self, proposal, now):
        """Count only fresh independently mapped observations in this region.

        Registry IDs can change after occlusion. Spatial deduplication is an
        estimate, never proof that every physical flame has been discovered.
        """
        if not isinstance(proposal, dict) or not fresh(proposal, now):
            return
        records = proposal.get('mapped_targets', [])
        records = list(records) if isinstance(records, list) else []
        records.append(proposal)
        bounds = self.catalog['regions'][self.region_id]['bounds_image_cm']
        for item in records:
            if not isinstance(item, dict):
                continue
            identity = (item.get('registry_session'), item.get('target_id'))
            world = item.get('fire_world_m')
            if (not isinstance(identity[0], str) or type(identity[1]) is not int or
                    identity not in self.visible_identities or
                    identity in self.mapped_identities or
                    not fresh(item, now, .6, 'capture_stamp') or
                    item.get('calibration_verified') is not True or item.get('map_verified') is not True or
                    item.get('map_sha256') != self.catalog['map_sha256'] or
                    not isinstance(world, (list, tuple)) or len(world) != 3 or
                    any(not _number(x) for x in world) or not 0 <= world[2] <= 4.):
                continue
            x, y = world[0] * 100., self.catalog['height_cm'] - world[1] * 100.
            if not (bounds[0] <= x < bounds[2] and bounds[1] <= y < bounds[3]):
                continue
            if len(self.mapped_identities) >= MAX_FIRE_OBSERVATIONS:
                self.statistics_truncated = True
                continue
            self.mapped_identities.add(identity)
            if not any(math.dist(world, point) < .12 for point in self.mapped_world):
                self.mapped_world.append(list(world))

    def fire_statistics(self, now):
        current_fresh = (self.visible_capture is not None and 0 <= now - self.visible_capture <= .6)
        return dict(current_visible=len(self.visible_identities) if current_fresh else 0,
                    current_fresh=current_fresh, max_visible=self.max_visible,
                    tracked_observations_count=len(self.observed_identities),
                    mapped_unique_count=len(self.mapped_world),
                    rounds_completed=self.rounds, handled_observations_count=len(self.handled),
                    min_confidence=self.min_fire_confidence,
                    skipped_alignment_count=len(self.skipped_observations),
                    physical_total_verified=False, truncated=self.statistics_truncated)

    def _result(self):
        effects, self.effects = self.effects, []
        return effects

    def _phase(self, phase, reason, now):
        self.phase, self.reason, self.phase_at = phase, reason, now
        self._centering_feedback_wait_since = None
        if self.centering.active:
            self.centering.phase, self.centering.reason, self.centering.phase_at = phase, reason, now
        if phase in ('ENTERING', 'NAVIGATING', 'EXITING'):
            self.navigation_motion_started_at = None

    def _navigation_elapsed(self, navigation, now):
        if not self.continuous_route:
            return now-self.phase_at
        if (self.navigation_motion_started_at is None and navigation.get('phase') in ('running', 'setting_checkpoint_speed') and
                self._navigation(navigation, now)):
            self.navigation_motion_started_at = now
        # PLAN and complete-route readback have their own bounded deadlines
        # in AreaNavigation. Do not consume the drive budget while stationary.
        return 0. if self.navigation_motion_started_at is None else now-self.navigation_motion_started_at

    def _identifier(self, kind, number):
        seed = '%s:%s:%s' % (self.command_id, kind, number)
        return 'region-' + kind + ':' + hashlib.sha256(seed.encode()).hexdigest()[:24]

    def _navigate(self, now, goal, kind, first=False, alignment_guard=None, retained_proposal=None):
        if self.continuous_route and kind == 'alignment':
            # Retain the patrol's route RAM, task identity and cursor. Visual
            # alignment may only advance along its already verified corridor.
            self.effects.append(('align_retained', dict(retained_proposal or {}, command_id=self.leg_id,
                goal_cell=list(goal), alignment_guard=dict(alignment_guard or {}))))
            self.leg_kind = 'alignment'
            self._phase('ALIGNING', 'bounded_alignment_on_retained_patrol_route', now)
            return
        self.leg_counter += 1
        self.leg_id = self._identifier('leg', self.leg_counter)
        self.leg_kind, self.goal_cell = kind, list(goal)
        waypoint = self.exit if kind == 'exit' else self.waypoints[len(self.visited)]
        payload = dict(command_id=self.leg_id, region_task_id=self.command_id,
                       region_id=self.region_id, goal_cell=self.goal_cell,
                       execute_fire=self.execute_fire,
                       waypoint_id=waypoint['id'], leg_kind=kind,
                       use_current_pose=True, issued_at=now, ttl_s=2.)
        if kind == 'alignment':
            payload['motion_kind'] = 'semantic_alignment'
            payload['alignment_guard'] = dict(alignment_guard or {})
        if self.route_planner:
            bounds = self.catalog['regions'][self.region_id]['bounds_image_cm']
            payload['region_preference_cells'] = ([int(bounds[0] // 2), int(bounds[1] // 2),
                int(bounds[2] // 2) - 1, int(bounds[3] // 2) - 1] if kind == 'search' else None)
            payload['region_route_policy_required'] = True
        if first:
            payload['start_reference'] = self.start_reference.copy()
        self.effects.append(('navigate', payload))
        self.candidate, self.hits, self.observation_frames = None, 0, 0
        self._phase({'approach': 'APPROACHING', 'entry': 'ENTERING', 'exit': 'EXITING', 'alignment': 'ALIGNING'}.get(kind, 'NAVIGATING'),
                    {'approach': 'approaching_mapped_fire', 'entry': 'entering_search_region',
                     'exit': 'leaving_region_to_verified_stop'}.get(kind, 'following_search_waypoint'), now)

    def _schedule(self, now, reference, first=False):
        """Reorder remaining stops using the latest actual stopped heading."""
        if not self.route_planner:
            self._navigate(now, self.waypoints[len(self.visited)]['cell'], 'search', first=first)
            return
        from .region_route import inside
        remaining = [point for point in self.waypoints if point['id'] not in self.visited]
        if (not first and len(remaining) == 1 and
                all(abs(reference['cell'][i] - remaining[0]['cell'][i]) <= ARRIVAL_TOLERANCE_CELLS for i in range(2)) and
                self._navigation(self.last_navigation, now, stopped=True) and
                self.last_navigation.get('phase') in ('paused', 'arrived')):
            # Existing correlated stop can observe this point directly, e.g.
            # after fire approach. Do not invent a DONE for a new zero route.
            self.goal_cell, self.leg_kind = list(remaining[0]['cell']), 'search'
            self.candidate, self.hits, self.observation_frames = None, 0, 0
            self._begin_checkpoint(now) if self._checkpoint_enabled() else self._phase('OBSERVING', 'observing_current_waypoint_after_verified_stop', now)
            return
        plan = self.route_planner.plan(self.region_id, remaining, reference)
        visited_points = [next(point for point in self.waypoints if point['id'] == key) for key in self.visited]
        self.waypoints = visited_points + plan['waypoints']
        self.route_policy, self.exit = plan['route_policy'], plan['exit']
        if self.continuous_route:
            if not first:
                raise ValueError('retained_route_must_not_be_replanned')
            self.leg_counter += 1
            self.leg_id = self._identifier('patrol', self.leg_counter)
            self.goal_cell, self.leg_kind = list(self.waypoints[0]['cell']), 'search'
            bounds = self.catalog['regions'][self.region_id]['bounds_image_cm']
            self.route_policy.update(continuous_route=True, planning_count=1,
                waypoint_policy='pass_through', checkpoint_stop='fire_only',
                fire_stop_dwell_s=1., fire_centering='all_visible_bidirectional_then_return_anchor',
                resume_policy='same_task_same_route')
            self.effects.append(('navigate', dict(command_id=self.leg_id,
                region_task_id=self.command_id, region_id=self.region_id,
                continuous_route=True, patrol_waypoints=[dict(p) for p in self.waypoints],
                goal_cell=list(self.exit['cell']), start_reference=dict(reference),
                execute_fire=self.execute_fire, leg_kind='patrol',
                region_preference_cells=[int(bounds[0]//2), int(bounds[1]//2),
                    int(bounds[2]//2)-1, int(bounds[3]//2)-1],
                region_route_policy_required=True, issued_at=now, ttl_s=2.)))
            self._phase('NAVIGATING', 'planning_complete_patrol_route_once', now)
            return
        if not remaining:
            self._navigate(now, self.exit['cell'], 'exit', first=first)
        else:
            kind = 'search' if inside(reference['cell'], self.catalog['regions'][self.region_id]['bounds_image_cm']) else 'entry'
            self._navigate(now, self.waypoints[len(self.visited)]['cell'], kind, first=first)

    def _reschedule(self, now, navigation):
        if self.continuous_route:
            self._continue_retained(now, advance=False)
            return
        try:
            self._schedule(now, dict(valid=navigation.get('pose_valid') is True,
                cell=navigation.get('pose_cell'), heading_deg=navigation.get('pose_heading_deg')))
        except (ValueError, KeyError, TypeError) as exc:
            self._finish(now, 'FAILED', 'region_replan_failed:' + str(exc))

    def _continue_retained(self, now, advance):
        self.effects.append(('advance_checkpoint' if advance else 'resume_retained',
            dict(command_id=self.leg_id, issued_at=now)))
        self.candidate, self.hits, self.observation_frames = None, 0, 0
        exiting = len(self.visited) == len(self.waypoints)
        self.leg_kind = 'exit' if exiting else 'search'
        self.goal_cell = list(self.exit['cell'] if exiting else self.waypoints[len(self.visited)]['cell'])
        self._phase('EXITING' if exiting else 'NAVIGATING', 'resuming_same_patrol_route', now)

    def _checkpoint_arrived(self, navigation, now):
        if self.continuous_route:
            return bool(self._navigation(navigation, now, stopped=True) and
                navigation.get('retained_route') is True and navigation.get('phase') == 'paused' and
                navigation.get('checkpoint_ready') is True and
                navigation.get('checkpoint_index') == len(self.visited))
        return self._navigation(navigation, now, stopped=True, arrived=True)

    def start(self, request, now, start_reference):
        if self.active:
            raise ValueError('region_search_already_active')
        if (request.get('action') != 'search_region' or
                request.get('region_id') not in self.catalog['regions'] or
                request.get('target_class') != 'firefighter' or
                request.get('execute_fire') is not False or
                not isinstance(request.get('command_id'), str) or not request['command_id']):
            raise ValueError('invalid_region_search_request')
        if (not isinstance(start_reference, dict) or start_reference.get('valid') is not True or
                not _cell(start_reference.get('cell')) or
                not _number(start_reference.get('heading_deg')) or
                not -180. <= start_reference['heading_deg'] <= 180.):
            raise ValueError('region_start_pose_and_heading_required')
        self.active = True
        self.command_id, self.region_id = request['command_id'], request['region_id']
        self.execute_fire = request['execute_fire']
        self.continuous_route = bool(request.get('continuous_route') is True and self.route_planner)
        self.fire_observed_at = None
        self.waypoints = list(self.catalog['regions'][self.region_id]['waypoints'])
        first = self.waypoints[0]['cell']
        if (len(self.waypoints) > 1 and
                all(abs(first[i] - start_reference['cell'][i]) <= ARRIVAL_TOLERANCE_CELLS for i in range(2))):
            self.waypoints = self.waypoints[1:] + self.waypoints[:1]
        self.visited, self.handled, self.handled_world, self.excluded = [], set(), [], set()
        self.fire_retrigger.reset()
        self.centering.reset()
        self.stop_trigger.reset()
        self.rounds = self.leg_counter = 0
        self._reset_fire_statistics()
        self.started_at = now
        self.navigation_missing_since = self.observation_dwell_at = None
        self.last_capture = self.target = self.run_id = self.target_world = None
        self.visual_degraded = False
        self.visual_degraded_since = self.visual_recovery_reason = self.visual_recovered_at = None
        self.visual_coverage_incomplete = False
        self.geometry_reason = self.geometry_wait_target = None
        self.failure_detail = None
        self.last_fire_trigger = None
        self.consumed_motion_exhaustions = set()
        self.start_reference = dict(valid=True, cell=list(start_reference['cell']),
                                    heading_deg=start_reference['heading_deg'])
        self.effects = []
        if self.checkpoint:
            self.checkpoint.active = False
        try:
            self._schedule(now, self.start_reference, first=True)
        except (ValueError, KeyError, TypeError):
            self.active = False
            self._phase('FAILED', 'region_initial_schedule_failed', now)
            raise
        return self._result()

    def _retire_auto(self, now):
        if self.run_id:
            self.effects.append(('auto', dict(action='cancel', command_id='cancel:' + self.run_id,
                                             issued_at=now, ttl_s=.5)))
        self.effects.append(('actuator', dict(action='disarm', issued_at=now, ttl_s=.5)))

    def cancel(self, now, reason='operator_stop'):
        if self.active:
            self._finish(now, 'CANCELLED', reason)
        return self._result()

    def _finish(self, now, phase, reason):
        if phase in ('FAILED', 'CANCELLED'):
            navigation = self.last_navigation or {}
            self.failure_detail = dict(stamp=now, task_phase=self.phase, reason=reason,
                navigation_phase=navigation.get('phase'), navigation_reason=navigation.get('reason'),
                navigation_identity_failure=navigation.get('identity_failure'),
                navigation_motion_failure=deepcopy((navigation.get('centering') or {}).get('motion_failure')),
                target_id=self.target[1] if self.target else None,
                command_id=self.command_id, leg_id=self.leg_id)
        self._retire_auto(now)
        self.effects.append(('stop', dict(command_id=self.leg_id, region_task_id=self.command_id,
                                         reason=reason, issued_at=now, ttl_s=.5)))
        self.active = False
        if self.checkpoint and self.checkpoint.active:
            self.checkpoint.active = False
            self.checkpoint.phase = 'FAILED' if phase != 'COMPLETE' else 'DONE'
            self.checkpoint.reason = reason
        if self.centering.active:
            self.centering.active = False
            self.centering.phase, self.centering.reason = phase, reason
            self.centering.move_direction = 'stopped'
            self.centering.reset_dwell()
        self.stop_trigger.reset()
        self._phase(phase, reason, now)

    def _navigation(self, navigation, now, stopped=False, arrived=False):
        local = navigation.get('centering') or {}
        cell = navigation.get('pose_cell')
        local_pose = bool(not arrived and self.centering.active and local.get('active') is True
            and local.get('map_limits_enabled') is False and isinstance(cell, (list, tuple))
            and len(cell) == 2 and all(type(value) is int for value in cell))
        phases = ('resetting', 'reset_readback', 'setting_region_preference', 'setting_start', 'setting_heading',
                  'setting_goal', 'setting_waypoint', 'plan_ack', 'planning', 'starting', 'running',
                  'continuous_route_check', 'resuming', 'setting_retained_alignment_speed', 'setting_checkpoint_speed',
                  'centering_starting', 'centering_moving', 'centering_stopping', 'centering_verifying',
                  'setting_alignment_speed', 'alignment_route_check', 'restoring_speed',
                  'pausing', 'paused', 'arriving', 'confirming_arrival', 'arrived', 'failed', 'cancelled', 'stopping')
        return bool(fresh(navigation, now) and navigation.get('command_id') == self.leg_id and
                    navigation.get('phase') in phases and
                    (not (stopped or arrived) or (navigation.get('pose_valid') is True and
                                                  (_cell(cell) or local_pose))) and
                    (not stopped or navigation.get('stopped') is True) and
                    (not arrived or (navigation.get('arrived') is True and
                                     navigation.get('phase') == 'arrived' and
                                     abs(navigation['pose_cell'][0] - self.goal_cell[0]) <= ARRIVAL_TOLERANCE_CELLS and
                                     abs(navigation['pose_cell'][1] - self.goal_cell[1]) <= ARRIVAL_TOLERANCE_CELLS)))

    def _begin_checkpoint(self, now):
        self.checkpoint.begin(now, self.waypoints[len(self.visited)]['id'])
        self._phase('CHECKPOINT_COLLECT', 'collecting_stable_checkpoint_targets', now)

    def _checkpoint_phase(self, phase, reason, now):
        self.checkpoint.phase, self.checkpoint.reason = phase, reason
        self._phase(phase, reason, now)

    def _checkpoint_complete(self, now, navigation):
        self.checkpoint.active = False
        self.checkpoint.phase, self.checkpoint.reason = 'DONE', 'checkpoint_queue_finished'
        self.visited.append(self.waypoints[len(self.visited)]['id'])
        if self.continuous_route:
            self._continue_retained(now, advance=True)
        elif self.route_planner:
            self._reschedule(now, navigation)
        elif len(self.visited) == len(self.waypoints):
            self._finish(now, 'COMPLETE', 'no_fire_found' if self.fire_observed_at is None and not self.rounds else 'region_search_finished')
        else:
            self._navigate(now, self.waypoints[len(self.visited)]['cell'], 'search')

    def _checkpoint_skip(self, now, reason):
        """An unreachable view target does not abandon later search stops."""
        c = self.checkpoint
        if c.current and len(self.skipped_observations) < MAX_FIRE_OBSERVATIONS:
            self.skipped_observations.append(dict(registry_session=c.identity[0], target_id=c.identity[1],
                waypoint_id=c.waypoint_id, reason=reason, stamp=now, extinguished_verified=False))
        c.finish_target('skipped', reason)
        self._checkpoint_phase('ALIGN_SELECT', reason, now)

    def _checkpoint_stop(self, now, navigation, reason):
        c = self.checkpoint
        c.stop_reason, c.stopped_at = reason, None
        c.center_hits = 0
        if (navigation.get('phase') in ('running', 'setting_checkpoint_speed') or
                self.continuous_route and navigation.get('phase') in
                ('setting_retained_alignment_speed', 'resuming')):
            self.effects.append(('pause', dict(command_id=self.leg_id, region_task_id=self.command_id,
                                              issued_at=now, ttl_s=.5)))
        elif navigation.get('phase') not in ('pausing', 'restoring_speed', 'paused', 'arrived'):
            # If the target vanishes during reset/PLAN/START, cancel this route
            # instead of letting a queued START run on an obsolete observation.
            self._finish(now, 'FAILED', 'checkpoint_alignment_cancelled_before_running:' + reason)
            return
        self._checkpoint_phase('ALIGN_STOP', reason, now)

    def _checkpoint_tick(self, now, navigation, targets, actuator, approach, proposal):
        """Process one checkpoint while retaining the single FPGA motion owner."""
        c = self.checkpoint
        c.observe(self.fire_retrigger.filtered(targets, now, self.handled | self.excluded), now)
        if now - c.started_at > c.config['checkpoint_timeout_s']:
            self._finish(now, 'FAILED', 'checkpoint_alignment_timeout')
            return
        pump_check = off_disarmed if self.execute_fire else navigation_off_disarmed
        stopped = self._navigation(navigation, now, stopped=True) and navigation.get('phase') in ('paused', 'arrived')
        stationary_phase = self.phase in ('CHECKPOINT_COLLECT', 'ALIGN_SELECT', 'ALIGN_DWELL', 'ALIGN_FIRE_WAIT')
        if stationary_phase and (not stopped or not pump_check(actuator, now)):
            # HEARTBEAT, EXT_STATUS and POSE arrive independently. A single
            # expired/misaligned reply must not abandon every remaining point.
            # Wait without issuing movement or claiming a continuous dwell.
            c.feedback_wait_reason = ('checkpoint_waiting_measured_stop' if not stopped else
                                      'checkpoint_waiting_pump_off_feedback')
            if c.feedback_wait_at is None:
                c.feedback_wait_at = now
            c.dwell_at = None
            c.center_hits = 0
            if self.phase == 'CHECKPOINT_COLLECT':
                c.frame_count, c.seen, c.current_frame = 0, {}, {}
            if now - c.feedback_wait_at > c.config['feedback_wait_s']:
                self._finish(now, 'FAILED', c.feedback_wait_reason + '_timeout')
            return
        c.feedback_wait_at = c.feedback_wait_reason = None
        if not pump_check(actuator, now):
            self._finish(now, 'FAILED', 'checkpoint_pump_off_not_confirmed')
            return
        if self.phase == 'CHECKPOINT_COLLECT':
            if not stopped:
                self._finish(now, 'FAILED', 'checkpoint_observation_stop_lost')
            elif self.fire_retrigger.has_deferred(now, self.handled | self.excluded):
                # A nearby next checkpoint may arrive before departure cooldown
                # ends. Start a complete fresh observation window afterwards.
                c.frame_count, c.started_at = 0, now
                self._checkpoint_phase('CHECKPOINT_COLLECT', 'processed_fire_departure_cooldown', now)
            else:
                try:
                    ready = c.freeze_queue(now)
                except ValueError as exc:
                    self._finish(now, 'FAILED', str(exc))
                    return
                if ready:
                    if c.queue:
                        self.fire_observed_at = c.capture
                    self._checkpoint_phase('ALIGN_SELECT', 'checkpoint_queue_ready', now)
        elif self.phase == 'ALIGN_SELECT':
            if not stopped:
                self._finish(now, 'FAILED', 'checkpoint_selection_requires_measured_stop')
                return
            if c.select() is None:
                self._checkpoint_complete(now, navigation)
                return
            relation = c.relation(now)
            if relation == 'missing':
                if now - c.current['last_seen'] > c.config['target_lost_s']:
                    c.finish_target('lost', 'checkpoint_queued_target_no_longer_visible')
                    self._checkpoint_phase('ALIGN_SELECT', 'checkpoint_queued_target_no_longer_visible', now)
                return
            if relation == 'passed':
                c.finish_target('passed', 'checkpoint_target_already_behind_centerline')
                self._checkpoint_phase('ALIGN_SELECT', 'checkpoint_target_already_behind_centerline', now)
                return
            if relation == 'unconfigured':
                self._finish(now, 'FAILED', 'checkpoint_forward_image_side_unset')
                return
            if relation == 'center':
                if c.center_hits >= c.config['center_confirm_frames']:
                    c.dwell_at = now
                    c.current['status'] = 'aligned'
                    self._checkpoint_phase('ALIGN_DWELL', 'centerline_confirmed_waiting_two_second_stop', now)
                return
            observation = c.observation(now)
            if observation['hits'] < c.config['confirm_frames']:
                return
            budget = c.config['max_advance_cm'] - c.total_advance_cm
            if budget <= 8.:
                self._checkpoint_skip(now, 'checkpoint_forward_distance_budget_exhausted')
                return
            if not isinstance(proposal, dict) or not fresh(proposal, now, .6) or proposal.get('valid') is not True:
                if (isinstance(proposal, dict) and fresh(proposal, now, .6) and
                        proposal.get('reason') in ('checkpoint_alignment_no_straight_road_in_region',
                            'checkpoint_alignment_no_forward_road_entry', 'checkpoint_alignment_forward_road_blocked',
                            'checkpoint_alignment_forward_budget_exhausted', 'checkpoint_alignment_start_outside_region',
                            'checkpoint_alignment_no_straight_retained_route_budget',
                            'retained_alignment_no_straight_corridor', 'retained_alignment_next_checkpoint_boundary')):
                    self._checkpoint_skip(now, proposal['reason'])
                    return
                if now - self.phase_at > 3.:
                    self._finish(now, 'FAILED', 'checkpoint_forward_path_unavailable:' + str((proposal or {}).get('reason', 'no_proposal')))
                return
            length, origin, heading = proposal.get('path_length_cm'), proposal.get('from_pose_cell'), proposal.get('from_heading_deg')
            guard = proposal.get('alignment_guard')
            if (not _number(length) or not 8. < length <= min(c.config['leg_max_cm'], budget) or
                    proposal.get('road_path_verified') is not True or not _cell(proposal.get('goal_cell')) or
                    origin != navigation.get('pose_cell') or not _number(heading) or
                    not _number(navigation.get('pose_heading_deg')) or
                    abs((heading - navigation['pose_heading_deg'] + 180.) % 360. - 180.) > 1. or
                    not isinstance(guard, dict) or guard.get('start_cell') != origin or
                    not _number(guard.get('heading_deg')) or abs((guard['heading_deg'] - heading + 180.) % 360. - 180.) > 1. or
                    not _number(guard.get('max_advance_cm')) or not length <= guard['max_advance_cm'] <= min(c.config['leg_max_cm'], budget) or
                    not _number(guard.get('heading_tolerance_deg')) or not 0 < guard['heading_tolerance_deg'] <= 15 or
                    not _number(guard.get('lateral_tolerance_cm')) or not 0 < guard['lateral_tolerance_cm'] <= 6 or
                    not _number(guard.get('speed_mm_s')) or not 0 < guard['speed_mm_s'] <= 60):
                self._finish(now, 'FAILED', 'checkpoint_forward_path_or_origin_not_verified')
                return
            if proposal['goal_cell'] == origin:
                self._finish(now, 'FAILED', 'checkpoint_zero_length_forward_route')
                return
            c.reserve_leg(guard, now)
            c.move_at, c.stopped_at, c.center_hits = now, None, 0
            c.phase, c.reason = 'ALIGNING', 'bounded_forward_centerline_alignment'
            self._navigate(now, proposal['goal_cell'], 'alignment', alignment_guard=guard,
                           retained_proposal=proposal)
        elif self.phase == 'ALIGNING':
            if not self._navigation(navigation, now):
                self._finish(now, 'FAILED', 'checkpoint_alignment_navigation_feedback_lost')
                return
            if navigation.get('phase') in ('confirming_arrival', 'restoring_speed'):
                # The leg owner only publishes paused/arrived after restoring
                # and acknowledging the original drive speed. Do not interrupt
                # DONE verification or that handoff with a second STOP, and do
                # not treat physical stop alone as completion.
                return
            relation = c.relation(now)
            if now - c.move_at > c.config['leg_timeout_s']:
                self._checkpoint_stop(now, navigation, 'checkpoint_alignment_move_timeout')
            elif relation == 'missing' and now - c.current['last_seen'] > c.config['target_lost_s']:
                self._checkpoint_stop(now, navigation, 'checkpoint_alignment_target_lost')
            elif relation == 'passed':
                self._checkpoint_stop(now, navigation, 'checkpoint_alignment_crossed_centerline')
            elif relation == 'center' and c.center_hits >= c.config['center_confirm_frames']:
                self._checkpoint_stop(now, navigation, 'checkpoint_alignment_center_candidate')
            elif (self._navigation(navigation, now, stopped=True, arrived=True) or
                  self.continuous_route and stopped and navigation.get('retained_alignment_finished') is True):
                self._checkpoint_stop(now, navigation, 'checkpoint_alignment_bounded_leg_finished')
        elif self.phase == 'ALIGN_STOP':
            if not stopped:
                if now - self.phase_at > 3.:
                    self._finish(now, 'FAILED', 'checkpoint_alignment_stop_not_confirmed')
                return
            c.settle_leg(navigation, now)
            if c.stop_reason == 'checkpoint_alignment_move_timeout':
                self._finish(now, 'FAILED', c.stop_reason)
                return
            if c.stop_reason == 'checkpoint_alignment_target_lost':
                c.finish_target('lost', c.stop_reason)
                self._checkpoint_phase('ALIGN_SELECT', 'checkpoint_lost_target_stopped', now)
                return
            if c.stopped_at is None:
                c.stopped_at, c.center_hits = now, 0
                return
            relation = c.relation(now)
            if relation == 'center' and c.center_hits >= c.config['center_confirm_frames']:
                c.dwell_at = now
                c.current['status'] = 'aligned'
                self._checkpoint_phase('ALIGN_DWELL', 'centerline_confirmed_waiting_two_second_stop', now)
            elif relation == 'passed':
                c.finish_target('passed', 'checkpoint_stopped_after_centerline')
                self._checkpoint_phase('ALIGN_SELECT', 'checkpoint_target_passed_without_alignment_claim', now)
            elif relation == 'forward' and c.observation(now)['hits'] >= c.config['confirm_frames']:
                self._checkpoint_phase('ALIGN_SELECT', 'checkpoint_continue_bounded_forward_alignment', now)
            elif now - c.stopped_at > c.config['target_lost_s'] and relation == 'missing':
                c.finish_target('lost', 'checkpoint_target_lost_after_verified_stop')
                self._checkpoint_phase('ALIGN_SELECT', 'checkpoint_target_lost_after_verified_stop', now)
        elif self.phase == 'ALIGN_DWELL':
            if not stopped:
                self._finish(now, 'FAILED', 'checkpoint_alignment_dwell_stop_lost')
                return
            relation = c.relation(now)
            if relation != 'center':
                c.dwell_at = None
                if relation == 'missing':
                    if now - c.current['last_seen'] <= c.config['target_lost_s']:
                        return
                    c.finish_target('lost', 'checkpoint_target_lost_during_dwell')
                elif relation == 'passed':
                    c.finish_target('passed', 'checkpoint_target_shifted_behind_centerline')
                self._checkpoint_phase('ALIGN_SELECT', 'checkpoint_centerline_requires_reconfirmation', now)
                return
            if c.dwell_at is None:
                c.center_hits = 0
                self._checkpoint_phase('ALIGN_SELECT', 'checkpoint_centerline_requires_reconfirmation', now)
            elif now - c.dwell_at >= c.config['dwell_s']:
                if self.execute_fire:
                    self.target = c.identity
                    c.phase, c.reason = 'ALIGN_FIRE', 'checkpoint_aligned_waiting_fire_geometry'
                    self._phase('ALIGN_FIRE_WAIT', 'checkpoint_aligned_waiting_fire_geometry', now)
                else:
                    self.handled.add(c.identity)
                    self.fire_retrigger.mark_processed(c.identity, now)
                    c.finish_target('completed', 'checkpoint_centerline_observation_complete')
                    self._checkpoint_phase('ALIGN_SELECT', 'checkpoint_next_queued_target', now)
        elif self.phase == 'ALIGN_FIRE_WAIT':
            if not stopped:
                self._finish(now, 'FAILED', 'checkpoint_fire_requires_measured_stop')
                return
            if c.relation(now) != 'center':
                self._finish(now, 'FAILED', 'checkpoint_fire_target_no_longer_aligned')
                return
            target_proposal = self._proposal(approach, now, c.identity, [(c.identity, 1)])
            if target_proposal:
                self.candidate = c.identity
                self._begin_fire(target_proposal, navigation, now)
            elif now - self.phase_at > 5.:
                self._finish(now, 'FAILED', 'fire_geometry_or_calibration_unavailable')

    def _frame(self, document, now, preferred_identity=None):
        queued = {(row['registry_session'], row['target_id']) for row in self.checkpoint.queue
                  if row['status'] in ('queued', 'aligning', 'aligned')} if self.checkpoint and self.checkpoint.active else set()
        if self.centering.active:
            queued |= self.centering.pending_identities
        self.fire_retrigger.observe(document, now, queued, context=self.last_navigation)
        size, session = document.get('image_size_wh'), document.get('registry_session')
        if (document.get('frame_valid') is not True or not fresh(document, now, .6, 'capture_stamp') or
                not isinstance(session, str) or not session or
                not isinstance(size, (list, tuple)) or len(size) != 2 or
                any(type(x) is not int or x <= 0 for x in size) or
                not isinstance(document.get('targets'), list)):
            self.candidate, self.hits = None, 0
            self.visible_identities = set()
            self.visible_capture = None
            return []
        candidates, visible = [], set()
        for target in document['targets']:
            if not isinstance(target, dict):
                continue
            box = target.get('bbox_xyxy')
            identity = (session, target.get('target_id'))
            if (target.get('selectable') is not True or type(identity[1]) is not int or identity[1] <= 0 or
                    not confident_fire(target, self.min_fire_confidence) or
                    not isinstance(box, (list, tuple)) or len(box) != 4 or
                    any(not _number(x) for x in box)):
                continue
            x1, y1, x2, y2 = box
            if 0 <= x1 < x2 <= size[0] and 0 <= y1 < y2 <= size[1]:
                visible.add(identity)
                if identity not in self.handled and identity not in self.excluded and self.fire_retrigger.allows(identity, now):
                    candidates.append((identity, (x2 - x1) * (y2 - y1)))
        stamp = document['capture_stamp']
        if self.last_capture is not None and stamp <= self.last_capture:
            return candidates
        self.visible_identities, self.visible_capture = visible, stamp
        self.max_visible = max(self.max_visible, len(visible))
        for identity in visible:
            if identity in self.observed_identities:
                continue
            if len(self.observed_identities) >= MAX_FIRE_OBSERVATIONS:
                self.statistics_truncated = True
            else:
                self.observed_identities.add(identity)
        gap = self.last_capture is None or stamp - self.last_capture > .4
        self.last_capture = stamp
        if self.phase == 'OBSERVING':
            self.observation_frames += 1
        choice = next((item for item in candidates if item[0] == preferred_identity), None)
        choice = choice or next((item for item in candidates if item[0] == self.candidate), None)
        choice = choice or max(candidates, key=lambda item: item[1], default=None)
        if not choice:
            self.candidate, self.hits = None, 0
        else:
            self.hits = self.hits + 1 if self.candidate == choice[0] and not gap else 1
            self.candidate = choice[0]
        return candidates

    def _proposal(self, proposal, now, identity, candidates):
        if not identity or identity not in [item[0] for item in candidates] or not isinstance(proposal, dict):
            return None
        if (not fresh(proposal, now) or not fresh(proposal, now, .6, 'capture_stamp') or
                (proposal.get('registry_session'), proposal.get('target_id')) != identity or
                any(proposal.get(key) is not True for key in
                    ('valid', 'calibration_verified', 'map_verified', 'clearance_verified')) or
                proposal.get('map_sha256') != self.catalog['map_sha256'] or
                not _cell(proposal.get('goal_cell')) or type(proposal.get('in_spray_range')) is not bool):
            return None
        world = proposal.get('fire_world_m')
        if (not isinstance(world, (list, tuple)) or len(world) != 3 or
                any(not _number(x) for x in world) or not 0 <= world[2] <= 4.):
            return None
        x, y = world[0] * 100., self.catalog['height_cm'] - world[1] * 100.
        bounds = self.catalog['regions'][self.region_id]['bounds_image_cm']
        if not (bounds[0] <= x < bounds[2] and bounds[1] <= y < bounds[3]):
            self.excluded.add(identity)
            return None
        # Distinct confirmed members of this stationary view must each get
        # their own output round. Nearby fires can legitimately be <12 cm
        # apart; queue aliases already prevent reprocessing the same track.
        stationary_batch_member=(self.continuous_route and self.execute_fire and
            self.centering.gimbal_only and self.centering.current and
            list(identity) in self.centering.current.get('observed_id_aliases',[]))
        if not stationary_batch_member and any(math.dist(world, point) < .12 for point in self.handled_world):
            self.handled.add(identity)
            self.fire_retrigger.mark_processed(identity, now, start_cooldown=False)
            return None
        return proposal

    def _consume_exclusions(self, proposal, now, candidates):
        """Ignore only independently mapped fires proven outside this region."""
        if not isinstance(proposal, dict) or not fresh(proposal, now):
            return
        self.geometry_reason = proposal.get('reason')
        bounds = self.catalog['regions'][self.region_id]['bounds_image_cm']
        for item in proposal.get('excluded_targets', []):
            if not isinstance(item, dict):
                continue
            identity = (item.get('registry_session'), item.get('target_id'))
            world = item.get('fire_world_m')
            if (identity not in [candidate[0] for candidate in candidates] or
                    not fresh(item, now, .6, 'capture_stamp') or
                    item.get('calibration_verified') is not True or item.get('map_verified') is not True or
                    item.get('map_sha256') != self.catalog['map_sha256'] or
                    not isinstance(world, (list, tuple)) or len(world) != 3 or any(not _number(x) for x in world)):
                continue
            x, y = world[0] * 100., self.catalog['height_cm'] - world[1] * 100.
            if not (bounds[0] <= x < bounds[2] and bounds[1] <= y < bounds[3]):
                self.excluded.add(identity)

    def _begin_fire(self, proposal, navigation, now):
        self.target, self.target_world = self.candidate, list(proposal['fire_world_m'])
        if self._navigation(navigation, now, stopped=True):
            self._approach_or_fire(proposal, now)
        else:
            self.effects.append(('pause', dict(command_id=self.leg_id, region_task_id=self.command_id,
                                              issued_at=now, ttl_s=.5)))
            self._phase('WAIT_STOP', 'waiting_verified_fpga_stop', now)

    def _start_fire_round(self, now):
        """Start target selection/aiming; this dispatch is not pump authority.

        AutoFireRound still waits for the current fixed-camera geometry, actual
        settled encoders and fresh LOW readiness before requesting one pulse.
        """
        self.run_id = self._identifier('fire', self.rounds + 1)
        if self.continuous_route and self.execute_fire and self.centering.gimbal_only:
            row=self.centering.current
            if not row or not self.target or list(self.target) not in row.get('observed_id_aliases',[]):
                self._finish(now, 'FAILED', 'region_fire_batch_target_identity_mismatch')
                return
            row['spray_attempt']=dict(run_id=self.run_id,registry_session=self.target[0],
                target_id=self.target[1],started_at=now)
        # Navigation STOP latches the FPGA pump LOW. Prepare this exact new
        # explicit round with OFF; never manufacture an in-range/world point
        # or infer an output pulse from dispatching the automatic controller.
        self.effects.append(('actuator', dict(action='disarm',
            command_id='region-prepare:' + self.run_id, issued_at=now,
            ttl_s=.5, preserve_auto_run_id=self.run_id)))
        self.effects.append(('auto', dict(action='start', command_id=self.run_id,
            issued_at=now, ttl_s=2., preferred_registry_session=self.target[0],
            preferred_target_id=self.target[1], return_to_standby=False)))
        self._phase('EXTINGUISHING', 'automatic_bounded_fire_round', now)

    def _approach_or_fire(self, proposal, now):
        if proposal['in_spray_range']:
            self._start_fire_round(now)
        elif self.continuous_route:
            # A side-facing target outside the calibrated spray envelope does
            # not authorize replacing the patrol with a new approach route.
            if self._checkpoint_enabled() and self.checkpoint.active:
                self._checkpoint_skip(now, 'checkpoint_fire_out_of_range_on_retained_route')
                self.target = self.target_world = None
                self.candidate, self.hits = None, 0
            else:
                self._finish(now, 'FAILED', 'continuous_fire_requires_checkpoint_observation')
        else:
            self._navigate(now, proposal['goal_cell'], 'approach')

    def _accept_reassociation(self, proposal, navigation, now, candidates):
        if (self.phase != 'WAIT_SPRAY_RANGE' or not self._navigation(navigation, now, stopped=True) or
                self.target in [item[0] for item in candidates] or not isinstance(proposal, dict) or
                proposal.get('reassociation_verified') is not True or
                proposal.get('reassociation_stationary_verified') is not True or
                proposal.get('original_identity') not in (self.target, list(self.target))):
            return None
        identity = (proposal.get('registry_session'), proposal.get('target_id'))
        verified = self._proposal(proposal, now, identity, candidates)
        keys = proposal.get('reassociation_exposure_keys')
        second = proposal.get('reassociation_second_distance_m')
        stopped_since = proposal.get('reassociation_stopped_since')
        if (not verified or identity[0] != self.target[0] or
                math.dist(verified['fire_world_m'], self.target_world) > .08 or
                (second is not None and (not _number(second) or second < .16)) or
                not isinstance(keys, list) or len(keys) != 3 or
                any(not isinstance(key, (list, tuple)) or len(key) != 2 or
                    any(type(x) is not int or x < 0 for x in key) for key in keys) or
                any(not 0 <= now - key[0] / 1e9 <= .6 for key in keys) or
                any(keys[i][0] >= keys[i + 1][0] or keys[i][1] >= keys[i + 1][1] for i in range(2)) or
                tuple(keys[-1]) != (proposal.get('capture_stamp_ns'), proposal.get('sequence'))):
            return None
        if (not _number(stopped_since) or stopped_since > now or
                keys[0][0] / 1e9 < max(stopped_since, self.phase_at)):
            return None
        self.target = identity
        return verified

    def _sync_passed_waypoints(self, navigation, now):
        """Consume pose-verified route progress, never a stop or an arrival."""
        if not (self._navigation(navigation, now) and navigation.get('retained_route') is True
                and navigation.get('pose_valid') is True):
            return
        count = navigation.get('passed_waypoint_count')
        expected = [point['id'] for point in self.waypoints]
        if (type(count) is not int or not len(self.visited) <= count <= len(expected)
                or navigation.get('passed_waypoint_ids') != expected[:count]):
            return
        if count > len(self.visited):
            self.visited = expected[:count]
            self.navigation_motion_started_at = now
        if self.phase in ('ENTERING', 'NAVIGATING', 'EXITING'):
            exiting = count == len(expected)
            self.leg_kind = 'exit' if exiting else 'search'
            self.goal_cell = list(self.exit['cell'] if exiting else self.waypoints[count]['cell'])
            if exiting and self.phase != 'EXITING':
                self._phase('EXITING', 'all_waypoints_passed_following_original_exit_route', now)

    def _complete_retained_exit(self, now, navigation):
        from .region_route import inside
        if (len(self.visited) != len(self.waypoints)
                or not self._navigation(navigation, now, stopped=True)
                or navigation.get('phase') != 'arrived' or navigation.get('arrived') is not True
                or any(abs(navigation['pose_cell'][i] - self.exit['cell'][i]) > ARRIVAL_TOLERANCE_CELLS for i in range(2))
                or inside(navigation['pose_cell'], self.catalog['regions'][self.region_id]['bounds_image_cm'])):
            self._finish(now, 'FAILED', 'region_exit_before_verified_waypoint_passage')
            return
        self.exit['confirmed_stopped'] = True
        self.exit['actual_pose_cell'] = list(navigation['pose_cell'])
        self.goal_cell, self.leg_kind = list(self.exit['cell']), 'exit'
        self._finish(now, 'COMPLETE', 'region_search_exited_and_stopped')

    def _finish_retained_fire(self, now, navigation, *, skipped_reason=None, pump_off_stamp=None):
        """Retire one batch member; the next flame can be on either image side."""
        row=self.centering.current or {}
        if self.execute_fire:
            # A confirmed fire which could not be sprayed keeps the car held.
            # It is never converted into a skip that releases the patrol.
            if skipped_reason:
                self._finish(now, 'FAILED', 'region_confirmed_fire_unprocessed:'+skipped_reason)
                return
            evidence=row.get('spray_evidence') or {}
            attempt=row.get('spray_attempt') or {}
            if (not self.target or evidence.get('run_id')!=self.run_id or
                    attempt.get('run_id')!=self.run_id or
                    (evidence.get('registry_session'),evidence.get('target_id'))!=self.target or
                    evidence.get('saw_pump_on') is not True or
                    not _number(evidence.get('confirmed_pump_off_at')) or
                    not _number(pump_off_stamp) or pump_off_stamp<=self.phase_at or
                    not attempt.get('started_at',math.inf)<=evidence['confirmed_pump_off_at']<=pump_off_stamp<=now):
                self._finish(now, 'FAILED', 'region_fire_batch_spray_evidence_missing')
                return
            evidence['confirmed_disarmed_stamp']=pump_off_stamp
            row['spray_confirmed']=True
            row['spray_confirmed_at']=now
        if skipped_reason:
            self.excluded.add(self.target)
            if len(self.skipped_observations) < MAX_FIRE_OBSERVATIONS:
                self.skipped_observations.append(dict(registry_session=self.target[0],
                    target_id=self.target[1], reason=skipped_reason, stamp=now,
                    extinguished_verified=False))
        else:
            self.handled.add(self.target)
        if skipped_reason:
            row['skip_recorded']=True
        aliases={tuple(identity) for identity in row.get('observed_id_aliases',[])} | {self.target}
        aliases.add(tuple(row.get('original_identity',self.target)))
        for identity in aliases:
            (self.excluded if skipped_reason else self.handled).add(identity)
            self.fire_retrigger.mark_processed(identity, now, start_cooldown=False,
                                               status='skipped' if skipped_reason else 'completed')
        self.centering.finish_target('skipped' if skipped_reason else 'completed',
            skipped_reason or ('flame_short_spray_and_output_low_confirmed' if self.execute_fire else
                               'flame_centered_and_stopped_for_one_second'), now)
        self.run_id = self.target = self.target_world = None
        self.observation_dwell_at = None
        self._phase('CENTER_SELECT', 'selecting_next_visible_flame', now)

    def _centering_skipped(self, now, navigation, reason):
        self.target = self.centering.identity
        self._finish_retained_fire(now, navigation, skipped_reason=reason)

    def _retire_unconfirmed_batch_rows(self, now):
        """Expired candidates are explicit skips, never completed fire visits."""
        for row in self.centering.queue:
            if row['status']!='skipped' or row.get('skip_recorded'):
                continue
            row['skip_recorded']=True
            aliases={tuple(identity) for identity in row.get('observed_id_aliases',[])}
            aliases.add((row['registry_session'],row['target_id']))
            for identity in aliases:
                self.excluded.add(identity)
                self.fire_retrigger.mark_processed(identity,now,start_cooldown=False,status='skipped')
            if len(self.skipped_observations)<MAX_FIRE_OBSERVATIONS:
                self.skipped_observations.append(dict(registry_session=row['registry_session'],
                    target_id=row['target_id'],batch_id=row['batch_id'],reason=row.get('reason'),
                    stamp=now,extinguished_verified=False))

    def _centering_motion(self, proposal, now, returning=False):
        payload = dict(proposal, command_id=self.leg_id)
        payload.pop('motion_target_key', None)
        payload.pop('target_identity', None)
        if not returning and self.centering.current and self.centering.identity:
            # Batch identity survives visual tracker reassociation; a delayed
            # short-stop result must never retire whichever target comes next.
            payload['motion_target_key'] = self.centering.current['batch_id']
            payload['target_identity'] = list(self.centering.identity)
        self.effects.append(('center_jog', payload))
        self.centering.move_direction = 'forward' if proposal['distance_mm'] > 0 else 'reverse'
        if not returning:
            self._phase('CENTER_MOVING', 'moving_selected_flame_towards_centerline', now)

    def _update_visual_health(self, now):
        age=now-(self.last_capture if self.last_capture is not None else self.started_at)
        reason='region_yolo_feedback_lost' if self.last_capture is not None else 'region_yolo_feedback_missing'
        if self.centering.active and not self.centering.abandoned_reason:
            c=self.centering
            center_age=now-(c.capture if c.capture is not None else c.started_at)
            if center_age>age:
                age=center_age
                reason='fire_centering_fresh_exposure_unavailable'
        degraded=age>2.
        if degraded:
            self.visual_coverage_incomplete=True
            if not self.visual_degraded:self.visual_degraded_since=now
            self.visual_recovery_reason=reason
        elif self.visual_degraded:
            self.visual_recovered_at=now
        self.visual_degraded=degraded

    def _recover_search_only_centering(self, now, navigation, stopped, pump_off):
        """Abandon visual work, never the retained navigation task or feedback.

        A missing camera is not an empty scene and grants no completion credit.
        Existing motion must stop and return to the saved anchor before RESUME.
        """
        c=self.centering
        if self.execute_fire or not self.continuous_route or not (self.visual_degraded or c.abandoned_reason):
            return False
        c.abandon_unfinished(now,'search_only_visual_unavailable_skip_observation')
        self._retire_unconfirmed_batch_rows(now)
        self.target=self.target_world=None
        self.candidate,self.hits=None,0
        if self.phase=='CENTER_RETURN':
            return False
        if not stopped:
            if (navigation.get('phase') in ('centering_starting','centering_moving')
                    and self.phase!='CENTER_STOP'):
                self.effects.append(('center_stop',dict(command_id=self.leg_id)))
                self._phase('CENTER_STOP','visual_unavailable_waiting_verified_local_stop',now)
            return True
        if not pump_off:
            return True
        if self.phase!='CENTER_RETURN':
            self._phase('CENTER_RETURN','visual_unavailable_returning_original_route_anchor',now)
            return True
        return False

    def _centering_tick(self, now, navigation, actuator, proposal):
        c = self.centering
        stopped = (self._navigation(navigation, now, stopped=True)
                   and navigation.get('phase') in ('paused', 'arrived'))
        pump_check = off_disarmed if self.execute_fire else navigation_off_disarmed
        pump_off = pump_check(actuator, now)
        if now-c.started_at > 180.:
            self._finish(now, 'FAILED', 'fire_centering_batch_timeout')
            return
        if self.phase in ('CENTER_COLLECT', 'CENTER_SELECT', 'CENTER_DWELL'):
            if not stopped or not pump_off:
                c.reset_dwell()
                if c.current:
                    c.current.pop('reacquire_since', None)
                    c.current.pop('_reassociation_probe', None)
                if getattr(self, '_centering_feedback_wait_since', None) is None:
                    self._centering_feedback_wait_since = now
                if now-self._centering_feedback_wait_since > 3.:
                    self._finish(now, 'FAILED', 'fire_centering_stop_or_pump_off_missing')
                return
            self._centering_feedback_wait_since = None
        if self._recover_search_only_centering(now,navigation,stopped,pump_off):
            return
        if self.phase == 'CENTER_COLLECT':
            if c.freeze(now):
                self._retire_unconfirmed_batch_rows(now)
                self._phase('CENTER_SELECT', 'all_visible_flames_queued_both_sides', now)
            else:
                self.reason=c.collect_reason
        elif self.phase == 'CENTER_SELECT':
            if c.select() is None:
                pending=c.pending_candidates(now)
                self._retire_unconfirmed_batch_rows(now)
                if pending:
                    self.reason='waiting_new_batch_candidate_confirmation'
                    return
                if self.execute_fire and any('confirmed_at' in row and
                        (row['status']!='completed' or row.get('spray_confirmed') is not True)
                        for row in c.queue):
                    self._finish(now, 'FAILED', 'region_fire_batch_has_unsprayed_targets')
                    return
                self.target = None
                self._phase('CENTER_RETURN', 'returning_to_saved_route_anchor', now)
                return
            self.target, self.target_world = c.identity, None
            relation = c.relation(now)
            local = navigation.get('centering') or {}
            if self.execute_fire:
                observation=c.observation(now)
                if observation and observation['hits']>=c.confirm_frames:
                    c.begin_dwell(now)
                    self._phase('CENTER_DWELL', 'stationary_flame_confirmation_before_gimbal_aim', now)
                elif now-self.phase_at>5.:
                    self._finish(now, 'FAILED', 'region_confirmed_fire_lost_before_gimbal_aim')
                return
            if relation=='missing' and local.get('return_ready') is True:
                c.current['reacquire_attempted']=True
                if 'reacquire_since' not in c.current:
                    c.current['reacquire_since'] = now
                    c.current['reacquire_wait_until'] = now + 1.6
                if c.reassociate_at_anchor(now, c.current['reacquire_since']):
                    self.target = c.identity
                    relation = c.relation(now)
            if relation == 'missing':
                unseen_for=now-max(c.current['last_seen'],c.current.get('last_candidate_stamp',-math.inf))
                if (not c.batch_at_terminal and not c.current.get('reacquire_attempted')
                        and _number(local.get('offset_mm')) and abs(local['offset_mm']) > 20.
                        and unseen_for>c.ttl_s):
                    c.current['reacquire_attempted'] = True
                    self._phase('CENTER_REACQUIRE', 'returning_to_anchor_to_reacquire_queued_flame', now)
                elif (unseen_for > c.ttl_s
                        and now >= c.current.get('reacquire_wait_until', -math.inf)):
                    self._centering_skipped(now, navigation, 'fire_centering_target_lost')
            elif relation == 'unconfigured':
                self._finish(now, 'FAILED', 'fire_centering_camera_direction_unset')
            elif relation == 'center':
                if c.center_hits >= c.confirm_frames:
                    c.begin_dwell(now)
                    self._phase('CENTER_DWELL', 'centered_stationary_one_second_required', now)
            elif c.observation(now)['hits'] >= c.confirm_frames:
                if c.batch_at_terminal:
                    self._centering_skipped(now, navigation, 'fire_centering_route_already_finished')
                elif isinstance(proposal, dict) and fresh(proposal, now) and proposal.get('valid') is True:
                    distance = proposal.get('distance_mm')
                    if _number(distance) and 0 < abs(distance) <= 100 and (distance > 0) == (relation == 'forward'):
                        self._centering_motion(proposal, now)
                elif isinstance(proposal, dict) and fresh(proposal, now) and proposal.get('valid') is False:
                    self._centering_skipped(now, navigation, str(proposal.get('reason', 'fire_centering_no_local_corridor')))
                elif now-self.phase_at > 3.:
                    self._finish(now, 'FAILED', 'fire_centering_motion_proposal_missing')
        elif self.phase in ('CENTER_MOVING', 'CENTER_STOP'):
            if stopped:
                local = navigation.get('centering') or {}
                token = local.get('motion_exhausted_token')
                if (navigation.get('phase') == 'paused' and c.current
                        and local.get('target_motion_exhausted') is True
                        and local.get('jog_returning') is False
                        and local.get('motion_target_key') == c.current['batch_id']
                        and type(local.get('command_sequence')) is int
                        and isinstance(token, str) and token
                        and token not in self.consumed_motion_exhaustions):
                    if not pump_off:
                        c.reset_dwell()
                        if self._centering_feedback_wait_since is None:
                            self._centering_feedback_wait_since = now
                        if now-self._centering_feedback_wait_since > 3.:
                            self._finish(now, 'FAILED', 'fire_centering_stop_or_pump_off_missing')
                        return
                    self._centering_feedback_wait_since = None
                    self.consumed_motion_exhaustions.add(token)
                    c.current['motion_exhaustion'] = dict(token=token,
                        command_sequence=local['command_sequence'],
                        short_stop_count=local.get('short_stop_count'), stamp=now)
                    self._centering_skipped(now, navigation, 'centering_target_motion_exhausted')
                    return
                c.move_direction = 'stopped'
                c.reset_dwell()
                self._phase('CENTER_SELECT', 'local_move_stopped_rechecking_visual_center', now)
                return
            if self.phase == 'CENTER_MOVING':
                relation = c.relation(now)
                if ((relation == 'center' and c.center_hits >= c.confirm_frames)
                        or relation in ('forward', 'reverse') and relation != c.move_direction
                        or relation == 'missing' and now-c.current['last_seen'] > c.ttl_s):
                    self.effects.append(('center_stop', dict(command_id=self.leg_id)))
                    c.reset_dwell()
                    self._phase('CENTER_STOP', 'waiting_measured_stop_after_visual_center', now)
        elif self.phase == 'CENTER_DWELL':
            if self.execute_fire:
                if c.observation(now) is None:
                    c.reset_dwell()
                    self._phase('CENTER_SELECT', 'stationary_fire_requires_fresh_confirmation', now)
                elif c.dwell_at is None:
                    self._phase('CENTER_SELECT', 'stationary_fire_requires_fresh_confirmation', now)
                elif c.dwell_complete(now):
                    self._phase('WAIT_SPRAY_RANGE', 'stationary_fire_waiting_stereo_gimbal_geometry', now)
                return
            if c.relation(now) != 'center':
                c.reset_dwell()
                self._phase('CENTER_SELECT', 'centerline_changed_restart_one_second_confirmation', now)
            elif c.dwell_at is None:
                self._phase('CENTER_SELECT', 'centerline_requires_fresh_stationary_confirmation', now)
            elif c.dwell_complete(now):
                if self.execute_fire:
                    self._phase('WAIT_SPRAY_RANGE', 'centered_one_second_waiting_fire_geometry', now)
                else:
                    self._finish_retained_fire(now, navigation)
        elif self.phase in ('CENTER_RETURN', 'CENTER_REACQUIRE'):
            if not stopped or not pump_off:
                return
            c.move_direction = 'stopped'
            if c.batch_at_terminal:
                c.active = False
                c.phase, c.reason = 'COMPLETE', 'terminal_flame_batch_finished'
                self._complete_retained_exit(now, navigation)
            elif isinstance(proposal, dict) and fresh(proposal, now) and proposal.get('valid') is True:
                if proposal.get('at_anchor') is True:
                    if self.phase == 'CENTER_REACQUIRE':
                        c.current['reacquire_since'] = now
                        c.current['reacquire_wait_until'] = now + 1.6
                        c.reset_dwell()
                        self._phase('CENTER_SELECT', 'rechecking_queued_flame_at_original_view', now)
                        return
                    self.effects.append(('end_centering', dict(command_id=self.leg_id)))
                    c.active = False
                    c.phase, c.reason = 'COMPLETE', 'all_flames_processed_and_route_anchor_restored'
                    self.fire_retrigger.begin_cooldown(now)
                    self._continue_retained(now, advance=False)
                else:
                    self._centering_motion(proposal, now, returning=True)
            elif isinstance(proposal, dict) and fresh(proposal, now) and proposal.get('valid') is False:
                self._finish(now, 'FAILED', 'fire_centering_anchor_return_unavailable:' + str(proposal.get('reason')))

    def _tick_retained_fire(self, now, navigation, candidates, auto_state, actuator, approach, centering_proposal=None):
        """One patrol route; bounded bidirectional centering only after fire STOP."""
        if self.execute_fire and self.centering.active and self.centering.queue_truncated:
            self._finish(now, 'FAILED', 'region_fire_batch_capacity_exceeded')
            return
        self._sync_passed_waypoints(navigation, now)
        nav_valid = self._navigation(navigation, now)
        stopped = (self._navigation(navigation, now, stopped=True)
                   and navigation.get('phase') in ('paused', 'arrived'))
        pump_check = off_disarmed if self.execute_fire else navigation_off_disarmed
        pump_off = pump_check(actuator, now)
        if self.phase.startswith('CENTER_'):
            self._centering_tick(now, navigation, actuator, centering_proposal)
            return
        if self.phase in ('ENTERING', 'NAVIGATING', 'EXITING'):
            if nav_valid and navigation.get('phase') == 'arrived':
                self._complete_retained_exit(now, navigation)
                return
            if (nav_valid and navigation.get('phase') == 'running' and pump_off
                    and self.stop_trigger.ready and self.candidate in [item[0] for item in candidates]
                    and self.candidate not in self.handled and self.candidate not in self.excluded):
                self.target = self.candidate
                self.target_world = None
                self._retained_trigger_frame=getattr(self,'_latest_target_document',None)
                frame=self._retained_trigger_frame or {}
                trigger_target=next((target for target in frame.get('targets',[])
                    if isinstance(target,dict) and
                    (frame.get('registry_session'),target.get('target_id'))==self.target),{})
                # Resetting the live qualifier after STOP must not erase why
                # the car stopped. Keep the exact detector evidence, including
                # whether this was a fire stop or a navigation feedback fault.
                self.last_fire_trigger=dict(self.stop_trigger.snapshot(),stamp=now,
                    capture_stamp=frame.get('capture_stamp'),sequence=frame.get('sequence'),
                    confidence=trigger_target.get('confidence'),
                    bbox_xyxy=list(trigger_target.get('bbox_xyxy') or []),
                    image_size_wh=list(frame.get('image_size_wh') or []),
                    route_generation=navigation.get('route_generation'),
                    route_progress_mm=navigation.get('route_progress_mm'),
                    navigation_pose_valid=navigation.get('pose_valid'))
                self.fire_observed_at = self.last_capture
                self.observation_dwell_at = None
                proposal = self._proposal(approach, now, self.target, candidates) if self.execute_fire else None
                if self.target in self.handled or self.target in self.excluded:
                    self.target = None
                    return
                if proposal:
                    self.target_world = list(proposal['fire_world_m'])
                self.effects.append(('pause', dict(command_id=self.leg_id,
                    region_task_id=self.command_id, issued_at=now, ttl_s=.5)))
                self._phase('WAIT_STOP', 'confirmed_fire_waiting_verified_fpga_stop', now)
            elif self._navigation_elapsed(navigation, now) > 180.:
                self._finish(now, 'FAILED', 'region_waypoint_navigation_timeout')
        elif self.phase == 'WAIT_STOP':
            if stopped and pump_off:
                terminal = navigation.get('phase') == 'arrived'
                self.centering.begin(now, terminal=terminal, seed=getattr(self,'_retained_trigger_frame',None),
                    gimbal_only=self.execute_fire)
                if not terminal:
                    self.effects.append(('begin_centering', dict(command_id=self.leg_id)))
                self.target = self.target_world = None
                self._phase('CENTER_COLLECT', 'collecting_all_visible_stable_flames', now)
            elif now - self.phase_at > 5.:
                self._finish(now, 'FAILED', 'region_stop_or_pump_off_not_confirmed')
        elif self.phase == 'WAIT_SPRAY_RANGE':
            if not stopped or not pump_off:
                if now - self.phase_at > 3.:
                    self._finish(now, 'FAILED', 'region_spray_stop_or_pump_off_lost')
                return
            if not self.execute_fire and self.centering.relation(now) != 'center':
                self.centering.reset_dwell()
                self._phase('CENTER_SELECT', 'centerline_changed_before_fire_round', now)
                return
            if self.execute_fire and self.centering.identity!=self.target:
                # Three fresh stationary exposures have already rebound this
                # same queue member. This may change selection before arm only.
                row=self.centering.current
                if not row or not self.target or list(self.target) not in row.get('observed_id_aliases',[]):
                    self._finish(now, 'FAILED', 'region_fire_batch_target_identity_mismatch')
                    return
                self.target,self.target_world=self.centering.identity,None
                self._phase('CENTER_SELECT', 'stationary_fire_requires_fresh_confirmation', now)
                return
            if self.execute_fire and self.centering.gimbal_only:
                observation=self.centering.observation(now)
                if (not observation or observation['hits']<self.centering.confirm_frames or
                        not self.centering.dwell_complete(now)):
                    self.centering.reset_dwell()
                    self._phase('CENTER_SELECT', 'stationary_fire_requires_fresh_confirmation', now)
                    return
                # The fixed-camera solver uses exact stereo pixels and the
                # camera/nozzle transform. No vehicle-to-world map or wheel
                # approach/range estimate is needed while the car stays still.
                self._start_fire_round(now)
                return
            proposal = self._proposal(approach, now, self.target, candidates)
            if not proposal and self.target_world is not None:
                proposal = self._accept_reassociation(approach, navigation, now, candidates)
            if proposal:
                self.target_world = list(proposal['fire_world_m'])
                if proposal['in_spray_range']:
                    self._approach_or_fire(proposal, now)
                else:
                    self._finish(now, 'FAILED', 'region_confirmed_fire_unprocessed:checkpoint_fire_out_of_range_on_retained_route')
            elif now - self.phase_at > 5.:
                self._finish(now, 'FAILED', 'region_confirmed_fire_unprocessed:'+str(self.geometry_reason or 'fire_geometry_or_calibration_unavailable'))
        elif self.phase == 'EXTINGUISHING':
            if not stopped:
                self._finish(now, 'FAILED', 'region_spray_stop_lost')
            elif fresh(auto_state, now) and auto_state.get('command_id') == self.run_id:
                if auto_state.get('active') is False:
                    off_at = auto_state.get('confirmed_pump_off_at')
                    if (auto_state.get('phase') == 'COMPLETE'
                            and auto_state.get('reason') == 'one_target_round_finished'
                            and auto_state.get('burst_completed') is True
                            and auto_state.get('saw_pump_on') is True and _number(off_at)
                            and (auto_state.get('preferred_registry_session'),auto_state.get('preferred_target_id'))==self.target
                            and auto_state.get('selected_target_id')==self.target[1]
                            and self.phase_at <= off_at <= now):
                        row=self.centering.current
                        attempt=(row or {}).get('spray_attempt') or {}
                        if (not row or attempt.get('run_id')!=self.run_id or
                                (attempt.get('registry_session'),attempt.get('target_id'))!=self.target):
                            self._finish(now, 'FAILED', 'region_fire_batch_target_identity_mismatch')
                            return
                        row['spray_evidence']=dict(run_id=self.run_id,registry_session=self.target[0],
                            target_id=self.target[1],saw_pump_on=True,confirmed_pump_off_at=off_at,
                            auto_completion_state_stamp=auto_state['stamp'])
                        self.rounds += 1
                        if self.target_world is not None:
                            self.handled_world.append(self.target_world)
                        self._retire_auto(now)
                        self._phase('WAIT_PUMP_OFF', 'waiting_confirmed_output_low_and_disarmed', now)
                    else:
                        self._finish(now, 'FAILED', 'region_auto_failed:' + str(auto_state.get('reason', 'unknown')))
                elif now - self.phase_at > 92.:
                    self._finish(now, 'FAILED', 'region_auto_round_timeout')
            elif now - self.phase_at > 2.:
                self._finish(now, 'FAILED', 'region_auto_feedback_missing_or_mismatched')
        elif self.phase == 'WAIT_PUMP_OFF':
            off_stamp=actuator.get('pump_feedback_stamp')
            if (stopped and off_disarmed(actuator, now) and actuator['stamp'] > self.phase_at
                    and _number(off_stamp) and self.phase_at < off_stamp <= now
                    and fresh(auto_state, now) and auto_state.get('active') is False
                    and auto_state.get('command_id') in (self.run_id, 'cancel:' + self.run_id)):
                self._finish_retained_fire(now, navigation, pump_off_stamp=off_stamp)
            elif now - self.phase_at > 3.:
                self._finish(now, 'FAILED', 'region_pump_off_not_confirmed')

    def tick(self, now, navigation, targets, auto_state, actuator, approach=None, alignment_proposal=None, centering_proposal=None):
        if not self.active:
            return self._result()
        self.last_navigation = navigation
        self._latest_target_document=targets
        if self._navigation(navigation, now):
            self.navigation_missing_since = None
        elif self.navigation_missing_since is None:
            self.navigation_missing_since = now
        preferred = ((approach.get('registry_session'), approach.get('target_id'))
                     if self.execute_fire and isinstance(approach, dict) and approach.get('valid') is True else None)
        if self._checkpoint_enabled() and self.checkpoint.active and self.checkpoint.identity:
            preferred = self.checkpoint.identity
        candidates = self._frame(targets, now, preferred)
        if self.continuous_route:
            trigger_running = (self.phase in ('ENTERING', 'NAVIGATING', 'EXITING')
                and self._navigation(navigation, now) and navigation.get('phase') == 'running')
            self.stop_trigger.observe(targets, now, {item[0] for item in candidates}, running=trigger_running)
            if trigger_running:
                self.candidate = self.stop_trigger.choice
                self.hits = self.stop_trigger.snapshot()['hits']
        if self.centering.active:
            stationary=(self._navigation(navigation,now,stopped=True) and navigation.get('phase') in ('paused','arrived'))
            local=navigation.get('centering') or {}
            self.centering.observe(self.fire_retrigger.filtered(targets, now, self.handled | self.excluded), now,
                stationary=stationary,at_anchor=stationary and
                (self.phase=='CENTER_COLLECT' or local.get('return_ready') is True or self.centering.batch_at_terminal))
        self._update_visual_health(now)
        self._map_fire_statistics(approach, now)
        if self.execute_fire:
            self._consume_exclusions(approach, now, candidates)
        if now - self.started_at > 1800.:
            self._finish(now, 'FAILED', 'region_task_timeout')
        elif (self.execute_fire or not self.continuous_route) and self.last_capture is None and now - self.started_at > 2.:
            self._finish(now, 'FAILED', 'region_yolo_feedback_missing')
        elif (self.execute_fire or not self.continuous_route) and self.last_capture is not None and now - self.last_capture > 2.:
            self._finish(now, 'FAILED', 'region_yolo_feedback_lost')
        elif (fresh(actuator, now) and actuator.get('pump_feedback_fresh') is True and
              actuator.get('pump_on') is True and self.phase != 'EXTINGUISHING' and
              not (self.continuous_route and self.phase == 'WAIT_PUMP_OFF')):
            # During WAIT_PUMP_OFF the auto result and actuator readback can
            # arrive in either order. Its own bounded wait keeps the car held
            # and requires a new measured LOW/disarmed reply before resuming.
            self._finish(now, 'FAILED', 'unexpected_pump_output')
        elif (self._navigation(navigation, now) and navigation.get('phase') in ('failed', 'cancelled')):
            # Keep the first concrete cause; a generic wrapper made an ended
            # route look like another unrelated task had silently interrupted it.
            self._finish(now, 'FAILED', navigation.get('reason') or 'region_navigation_' + navigation['phase'])
        elif (self.navigation_missing_since is not None and now - self.navigation_missing_since >
              (self.checkpoint.config['feedback_wait_s'] if self._checkpoint_enabled() else 2.)):
            self._finish(now, 'FAILED', 'region_navigation_feedback_missing_or_mismatched')
        if not self.active:
            return self._result()

        if self.continuous_route:
            self._tick_retained_fire(now, navigation, candidates, auto_state, actuator, approach, centering_proposal)
            return self._result()

        if self._checkpoint_enabled() and self.phase in ('CHECKPOINT_COLLECT', 'ALIGN_SELECT', 'ALIGNING',
                'ALIGN_STOP', 'ALIGN_DWELL', 'ALIGN_FIRE_WAIT'):
            self._checkpoint_tick(now, navigation, targets, actuator, approach, alignment_proposal)
            return self._result()

        # Search-only stops for two seconds, records the stable observation,
        # then continues all remaining points without issuing spray commands.
        if (not self._checkpoint_enabled() and not self.execute_fire and self.phase in ('ENTERING', 'NAVIGATING', 'OBSERVING') and
                self.hits >= 3 and self.candidate in [item[0] for item in candidates] and
                navigation_off_disarmed(actuator, now) and self._navigation(navigation, now) and
                (navigation.get('phase') in ('running', 'setting_checkpoint_speed', 'arrived') or
                 navigation.get('phase') == 'paused' and self._navigation(navigation, now, stopped=True))):
            self.target = self.candidate
            self.fire_observed_at = self.last_capture
            if navigation.get('phase') in ('paused', 'arrived') and self._navigation(navigation, now, stopped=True):
                self.observation_dwell_at = now
                self._phase('OBSERVATION_DWELL', 'fire_observed_waiting_two_second_stop', now)
            else:
                self.effects.append(('pause', dict(command_id=self.leg_id, region_task_id=self.command_id,
                                                  issued_at=now, ttl_s=.5)))
                self._phase('WAIT_STOP', 'waiting_verified_fpga_stop', now)
            return self._result()
        proposal = self._proposal(approach, now, self.candidate, candidates) if self.execute_fire else None
        if not self._checkpoint_enabled() and self.phase in ('ENTERING', 'NAVIGATING', 'OBSERVING', 'WAIT_GEOMETRY'):
            if (self.hits >= 3 and proposal and off_disarmed(actuator, now) and
                    self._navigation(navigation, now) and
                    (navigation.get('phase') in ('running', 'setting_checkpoint_speed', 'arrived') or
                     navigation.get('phase') == 'paused' and self._navigation(navigation, now, stopped=True))):
                self._begin_fire(proposal, navigation, now)
                return self._result()
        if self.phase == 'OBSERVATION_DWELL':
            if not (self._navigation(navigation, now, stopped=True) and
                    navigation.get('phase') in ('paused', 'arrived') and navigation_off_disarmed(actuator, now)):
                self.observation_dwell_at = None
                if now - self.phase_at > 5.:
                    self._finish(now, 'FAILED', 'region_observation_dwell_stop_not_confirmed')
            elif self.observation_dwell_at is None:
                self.observation_dwell_at = now
            elif now - self.observation_dwell_at >= 2.:
                self.handled.add(self.target)
                self.fire_retrigger.mark_processed(self.target, now)
                self.target = None
                if self._navigation(navigation, now, stopped=True, arrived=True):
                    self.observation_frames, self.hits, self.candidate = 0, 0, None
                    self._phase('OBSERVING', 'continuing_checkpoint_after_fire_observation', now)
                else:
                    self._reschedule(now, navigation)
        elif self.phase == 'EXITING':
            if self._navigation(navigation, now, stopped=True, arrived=True):
                from .region_route import inside
                if inside(navigation['pose_cell'], self.catalog['regions'][self.region_id]['bounds_image_cm']):
                    self._finish(now, 'FAILED', 'region_exit_pose_still_inside')
                else:
                    self.exit['confirmed_stopped'] = True
                    self.exit['actual_pose_cell'] = list(navigation['pose_cell'])
                    self._finish(now, 'COMPLETE', 'region_search_exited_and_stopped')
            elif self._navigation_elapsed(navigation, now) > 180.:
                self._finish(now, 'FAILED', 'region_exit_navigation_timeout')
        elif self.phase in ('ENTERING', 'NAVIGATING'):
            if self._checkpoint_arrived(navigation, now):
                self.observation_frames = 0
                self._begin_checkpoint(now) if self._checkpoint_enabled() else self._phase('OBSERVING', 'observing_waypoint_with_yolo', now)
            elif self._navigation_elapsed(navigation, now) > 180.:
                self._finish(now, 'FAILED', 'region_waypoint_navigation_timeout')
        elif self.phase == 'OBSERVING':
            if not self._navigation(navigation, now, stopped=True):
                self._finish(now, 'FAILED', 'region_observation_stop_lost')
            elif self.fire_retrigger.has_deferred(now, self.handled | self.excluded):
                self.observation_frames, self.phase_at = 0, now
            elif self.execute_fire and self.hits >= 3 and self.candidate not in self.handled and self.candidate not in self.excluded:
                self.geometry_wait_target = self.candidate
                self._phase('WAIT_GEOMETRY', 'fire_detected_waiting_verified_geometry', now)
            elif now - self.phase_at >= 1.5 and self.observation_frames >= 3:
                pump_check = off_disarmed if self.execute_fire else navigation_off_disarmed
                if not pump_check(actuator, now):
                    self._finish(now, 'FAILED', 'region_pump_off_not_confirmed')
                else:
                    self.visited.append(self.waypoints[len(self.visited)]['id'])
                    if self.continuous_route:
                        self._continue_retained(now, advance=True)
                    elif self.route_planner:
                        self._reschedule(now, navigation)
                    elif len(self.visited) == len(self.waypoints):
                        self._finish(now, 'COMPLETE', 'no_fire_found' if self.fire_observed_at is None and not self.rounds else 'region_search_finished')
                    else:
                        self._navigate(now, self.waypoints[len(self.visited)]['cell'], 'search')
        elif self.phase == 'WAIT_GEOMETRY':
            if not self._navigation(navigation, now, stopped=True):
                self._finish(now, 'FAILED', 'region_geometry_wait_stop_lost')
            elif self.geometry_wait_target in self.handled or self.geometry_wait_target in self.excluded:
                self.observation_frames = 0
                self._phase('OBSERVING', 'mapped_target_already_handled_or_outside_region', now)
            elif now - self.phase_at > 5.:
                self._finish(now, 'FAILED', 'fire_geometry_or_calibration_unavailable')
        elif self.phase == 'WAIT_STOP':
            if not self.execute_fire:
                if (self._navigation(navigation, now, stopped=True) and
                        navigation.get('phase') in ('paused', 'arrived') and navigation_off_disarmed(actuator, now)):
                    self.observation_dwell_at = now
                    self._phase('OBSERVATION_DWELL', 'fire_observed_waiting_two_second_stop', now)
                elif now - self.phase_at > 5.:
                    self._finish(now, 'FAILED', 'region_stop_or_pump_off_not_confirmed')
                return self._result()
            target_proposal = self._proposal(approach, now, self.target, candidates)
            if (self._navigation(navigation, now, stopped=True) and
                    navigation.get('phase') in ('paused', 'arrived') and
                    target_proposal and off_disarmed(actuator, now)):
                self._approach_or_fire(target_proposal, now)
            elif now - self.phase_at > 5.:
                self._finish(now, 'FAILED', 'region_stop_or_fire_geometry_not_confirmed')
        elif self.phase == 'APPROACHING':
            if self._navigation(navigation, now, stopped=True, arrived=True):
                self._phase('WAIT_SPRAY_RANGE', 'verifying_range_after_approach', now)
            elif now - self.phase_at > 60.:
                self._finish(now, 'FAILED', 'region_fire_approach_timeout')
        elif self.phase == 'WAIT_SPRAY_RANGE':
            target_proposal = self._proposal(approach, now, self.target, candidates)
            target_proposal = target_proposal or self._accept_reassociation(approach, navigation, now, candidates)
            if not self._navigation(navigation, now, stopped=True):
                self._finish(now, 'FAILED', 'region_spray_stop_lost')
            elif target_proposal and target_proposal['in_spray_range'] and off_disarmed(actuator, now):
                self._approach_or_fire(target_proposal, now)
            elif now - self.phase_at > 5.:
                self._finish(now, 'FAILED', 'region_verified_spray_range_unavailable')
        elif self.phase == 'EXTINGUISHING':
            if not self._navigation(navigation, now, stopped=True):
                self._finish(now, 'FAILED', 'region_spray_stop_lost')
            elif fresh(auto_state, now) and auto_state.get('command_id') == self.run_id:
                if auto_state.get('active') is False:
                    off_at = auto_state.get('confirmed_pump_off_at')
                    if (auto_state.get('phase') == 'COMPLETE' and
                            auto_state.get('reason') == 'one_target_round_finished' and
                            auto_state.get('burst_completed') is True and
                            auto_state.get('saw_pump_on') is True and _number(off_at) and
                            self.phase_at <= off_at <= now):
                        self.rounds += 1
                        self.handled.add(self.target)
                        self.fire_retrigger.mark_processed(self.target, now, start_cooldown=False)
                        self.handled_world.append(self.target_world)
                        self._retire_auto(now)
                        self._phase('WAIT_PUMP_OFF', 'waiting_confirmed_output_low_and_disarmed', now)
                    else:
                        self._finish(now, 'FAILED', 'region_auto_failed:' + str(auto_state.get('reason', 'unknown')))
                elif now - self.phase_at > 92.:
                    self._finish(now, 'FAILED', 'region_auto_round_timeout')
            elif now - self.phase_at > 2.:
                self._finish(now, 'FAILED', 'region_auto_feedback_missing_or_mismatched')
        elif self.phase == 'WAIT_PUMP_OFF':
            if (off_disarmed(actuator, now) and actuator['stamp'] > self.phase_at and
                    fresh(auto_state, now) and auto_state.get('active') is False and
                    auto_state.get('command_id') in (self.run_id, 'cancel:' + self.run_id) and
                    self._navigation(navigation, now, stopped=True)):
                self.fire_retrigger.mark_processed(self.target, now)
                self.run_id = self.target = self.target_world = None
                if self._checkpoint_enabled() and self.checkpoint.active:
                    self.checkpoint.finish_target('completed', 'checkpoint_fire_round_and_pump_off_confirmed')
                    self._checkpoint_phase('ALIGN_SELECT', 'checkpoint_next_queued_target_after_fire', now)
                    return self._result()
                search_goal = self.waypoints[len(self.visited)]['cell']
                if (self._navigation(navigation, now, stopped=True, arrived=True) and
                        all(abs(navigation['pose_cell'][i] - search_goal[i]) <= ARRIVAL_TOLERANCE_CELLS for i in range(2))):
                    self.goal_cell, self.leg_kind = list(search_goal), 'search'
                    self.observation_frames = self.hits = 0
                    self.candidate = None
                    self._phase('OBSERVING', 'observing_waypoint_after_fire_round', now)
                else:
                    self._reschedule(now, navigation)
            elif now - self.phase_at > 3.:
                self._finish(now, 'FAILED', 'region_pump_off_not_confirmed')
        return self._result()

    def pump_allowed(self, now):
        return self.execute_fire and self.active and self.phase == 'EXTINGUISHING' and self._navigation(self.last_navigation, now, stopped=True)

    def snapshot(self, now):
        region = self.catalog['regions'].get(self.region_id, {})
        return dict(stamp=now, active=self.active, phase=self.phase, reason=self.reason,
            target_class='firefighter', observation_only=True,
            firefighter_observed=self.fire_observed_at is not None,
            firefighter_observed_at=self.fire_observed_at,
            firefighter_stop_enabled=bool(self.active and self.last_capture is not None
                and 0<=now-self.last_capture<=.6 and not self.visual_degraded),
            failure_detail=dict(self.failure_detail) if self.failure_detail else None,
            execute_fire=self.execute_fire, fire_observed=self.fire_observed_at is not None,
            visual_degraded=self.visual_degraded,visual_degraded_since=self.visual_degraded_since,
            visual_recovery_reason=self.visual_recovery_reason,visual_recovered_at=self.visual_recovered_at,
            visual_coverage_incomplete=self.visual_coverage_incomplete,
            fire_stop_enabled=bool(self.active and self.last_capture is not None
                and 0<=now-self.last_capture<=.6 and not self.visual_degraded),
            fire_observed_at=self.fire_observed_at,
            command_id=self.command_id, region_id=self.region_id, region_label=region.get('label'),
            waypoints=self.waypoints, waypoints_total=len(self.waypoints), waypoints_visited=len(self.visited),
            visited_waypoint_ids=list(self.visited),
            remaining_waypoints=[point for point in self.waypoints if point['id'] not in self.visited],
            route_policy=dict(self.route_policy), exit=dict(self.exit) if self.exit else None,
            continuous_route=self.continuous_route,
            waypoint_policy='pass_through' if self.continuous_route else 'legacy_stop',
            passed_waypoint_count=len(self.visited), passed_waypoint_ids=list(self.visited),
            observation_dwell_remaining_s=(max(0., 2. - (now - self.observation_dwell_at))
                if self.phase == 'OBSERVATION_DWELL' and self.observation_dwell_at is not None else None),
            current_waypoint_id=self.waypoints[len(self.visited)]['id'] if len(self.visited) < len(self.waypoints) else None,
            progress=len(self.visited) / len(self.waypoints) if self.waypoints else 0.,
            rounds_completed=self.rounds, leg_id=self.leg_id, leg_kind=self.leg_kind, goal_cell=self.goal_cell,
            target_id=self.target[1] if self.target else None,
            registry_session=self.target[0] if self.target else None, auto_run_id=self.run_id,
            pending_unmapped_target_id=self.candidate[1] if self.candidate and self.phase == 'WAIT_GEOMETRY' else None,
            geometry_reason=self.geometry_reason, extinguished_verified=False,
            skipped_observations=list(self.skipped_observations),
            checkpoint_alignment=self.checkpoint.snapshot(now) if self.checkpoint else dict(enabled=False, active=False),
            centering=self.centering.snapshot(now),
            fire_stop_trigger=self.stop_trigger.snapshot(),
            last_fire_trigger=self.last_fire_trigger,
            fire_retrigger=self.fire_retrigger.snapshot(now),
            fire_statistics=self.fire_statistics(now))
