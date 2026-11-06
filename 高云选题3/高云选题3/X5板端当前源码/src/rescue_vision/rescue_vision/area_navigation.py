"""Correlated FPGA routes for generic goals and continuous region search.

The FPGA remains the planner and encoder/IMU motion controller. A regional tour
uploads all checkpoints once and retains one route/task through STOP/RESUME;
neither an ACK nor proximity to a marker is arrival evidence.
"""
import math
import struct
from . import ov5640_protocol as wire
from .numbered_navigation import recent
from .route_fire_patrol import off_disarmed, navigation_off_disarmed
from .retained_route import waypoint_references, project_progress, forward_proposal, angle_error
from .retained_centering import RetainedCentering, PHASES as CENTERING_PHASES

RESET_NAV = 0x25
SET_REGION_PREFERENCE = 0x26
SET_SPEED = 0x02
TERMINAL = ('idle', 'arrived', 'held', 'failed', 'cancelled')


def pose_reference_reason(pose, now, task_id=None):
    if pose.get('valid') is not True:
        return pose.get('reason') or 'pose_not_valid'
    if not recent(pose.get('stamp'), now, 1.2):
        return 'pose_stale'
    if pose.get('mode') != 9 or pose.get('status') not in (2, 3):
        return 'pose_navigation_mode_or_status_changed'
    if task_id is not None and pose.get('task_id') != task_id:
        return 'pose_task_changed'
    x, y, h = (pose.get(k) for k in ('x_mm', 'y_mm', 'heading_tenths'))
    if (any(type(v) is not int for v in (x, y, h)) or
            not 0 <= x < 3520 or not 0 < y <= 3200 or not -1800 <= h <= 1800):
        return 'pose_coordinates_invalid'
    return ''


def pose_reference(pose, now, task_id=None):
    if pose_reference_reason(pose, now, task_id):
        return None
    x, y, h = (pose[k] for k in ('x_mm', 'y_mm', 'heading_tenths'))
    return dict(valid=True, cell=[x//20, (3200-y)//20], heading_deg=h/10.,
                source='fpga_encoder_imu_pose', stamp=pose['stamp'],
                world_mm=[x, y], quantization_bound_mm=10,
                physical_position_verified=False)


class AreaNavigation:
    def __init__(self, owner):
        self.owner = owner
        self.active = None
        self.map = {}
        self.restore_speed_mm_s = None
        self.centering = RetainedCentering(self)

    @staticmethod
    def _alignment_guard(request, cell, heading):
        alignment = request.get('leg_kind') == 'alignment'
        if alignment != (request.get('motion_kind') == 'semantic_alignment'):
            raise ValueError('alignment_leg_and_motion_kind_must_match')
        guard = request.get('alignment_guard')
        if not alignment:
            if guard is not None:
                raise ValueError('alignment_guard_requires_alignment_leg')
            return None
        if not isinstance(guard, dict) or guard.get('start_cell') != list(cell):
            raise ValueError('alignment_start_reference_mismatch')
        for key, maximum in (('max_advance_cm', 30), ('lateral_tolerance_cm', 6),
                             ('heading_tolerance_deg', 15)):
            value = guard.get(key)
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= maximum:
                raise ValueError('invalid_alignment_' + key)
        angle = guard.get('heading_deg')
        wire.heading_payload(angle)
        if abs((angle-heading+180.) % 360.-180.) > .1 or guard.get('speed_mm_s') != 60:
            raise ValueError('alignment_heading_or_speed_mismatch')
        return dict(guard, start_cell=list(cell))

    @staticmethod
    def _projection(guard, point):
        angle = math.radians(guard['heading_deg'])
        dx, dy = point[0]-(guard['start_cell'][0]*20+10), point[1]-(guard['start_cell'][1]*20+10)
        return dx*math.cos(angle)-dy*math.sin(angle), dx*math.sin(angle)+dy*math.cos(angle)

    @classmethod
    def _corridor_reason(cls, guard, point):
        forward, lateral = cls._projection(guard, point)
        if forward < -20. or forward > guard['max_advance_cm']*10.+20.:
            return 'alignment_forward_distance_exceeded'
        if abs(lateral) > guard['lateral_tolerance_cm']*10.:
            return 'alignment_lateral_corridor_exceeded'
        return ''

    def _route_validation(self, now):
        a, route = self.active, self.map.get('route') or {}
        generation = route.get('generation')
        if (route.get('valid') is not True or route.get('sampled') is not False or
                type(generation) is not int or not 1 <= generation <= 65534 or
                generation == a.get('previous_route_generation') or
                not recent(route.get('stamp'), now, 5.) or route['stamp'] <= a['plan_ack_at']):
            return None
        points, count = route.get('points_mm'), route.get('total_count')
        if (not isinstance(points, list) or type(count) is not int or not 2 <= count <= 2048 or
                len(points) != count or ('sample_indices' in route and route['sample_indices'] != list(range(count)))):
            return None
        guard, previous, length = a['alignment_guard'], None, 0.
        for point in points:
            if (not isinstance(point, (list, tuple)) or len(point) != 2 or
                    any(type(v) not in (int, float) or not math.isfinite(v) for v in point)):
                return 'alignment_route_point_invalid'
            reason = self._corridor_reason(guard, point)
            if reason:
                return reason
            forward, _ = self._projection(guard, point)
            if previous is not None:
                old_forward, _ = self._projection(guard, previous)
                dx, dy = point[0]-previous[0], point[1]-previous[1]
                step = math.hypot(dx, dy)
                length += step
                if forward < old_forward-2.:
                    return 'alignment_route_reverses'
                if step > .1:
                    angle = math.degrees(math.atan2(-dy, dx))
                    if abs((angle-guard['heading_deg']+180.) % 360.-180.) > guard['heading_tolerance_deg']:
                        return 'alignment_route_heading_exceeded'
            previous = point
        if length <= .1 or self._projection(guard, points[-1])[0] <= self._projection(guard, points[0])[0]:
            return 'alignment_route_has_no_forward_progress'
        if length > guard['max_advance_cm']*10.+20.:
            return 'alignment_route_length_exceeded'
        if (math.dist(points[0], [v*20+10 for v in a['start_cell']]) > 60. or
                math.dist(points[-1], [v*20+10 for v in a['goal_cell']]) > 60.):
            return 'alignment_route_endpoint_mismatch'
        a.update(alignment_route_generation=generation, alignment_route_length_mm=length)
        return ''

    def _prepare_start(self, now):
        a = self.active
        if a.get('region_route_policy_required'):
            data = bytes(a['region_preference_cells']) if a['region_preference_cells'] is not None else bytes([255]*4)
            self._send(SET_REGION_PREFERENCE, data, 'setting_region_preference', now)
        else:
            self._send(wire.SET_START, bytes((*a['start_cell'], 0, 0)), 'setting_start', now)

    def _alignment_pose_reason(self, now):
        a, pose = self.active, self.map.get('pose') or {}
        ref = pose_reference(pose, now, a['task_id'])
        if not ref or pose.get('stamp', -math.inf) <= a['start_ack_at']:
            return 'alignment_fresh_actual_pose_required' if now-a['start_ack_at'] > 1.2 else None
        guard = a['alignment_guard']
        point = [pose['x_mm'], 3200-pose['y_mm']]
        reason = self._corridor_reason(guard, point)
        forward, _ = self._projection(guard, point)
        if abs((ref['heading_deg']-guard['heading_deg']+180.) % 360.-180.) > guard['heading_tolerance_deg']:
            reason = 'alignment_actual_heading_exceeded'
        if forward < a.get('furthest_forward_mm', forward)-5.:
            reason = 'alignment_actual_pose_reverses'
        a['furthest_forward_mm'] = max(forward, a.get('furthest_forward_mm', forward))
        return reason

    def _finish(self, phase, now):
        a = self.active
        if phase == 'paused':
            a.setdefault('pause_at', now)
            pose = self.map.get('pose') or {}
            if a.get('continuous_route') and pose_reference(pose, now, a['task_id']):
                a['paused_pose'] = [pose['x_mm'], pose['y_mm'], pose['heading_tenths']/10.]
            if a.get('retained_alignment_active'):
                a['retained_alignment_active'] = False
                a['retained_alignment_completed'] = True
        if self.restore_speed_mm_s is not None:
            a.update(after_speed_restore=phase, terminal_reason=a['reason'])
            self._send(SET_SPEED, struct.pack('<HH', self.restore_speed_mm_s, 0), 'restoring_speed', now)
            a['reason'] = 'alignment_waiting_speed_restore_ack'
        else:
            a.update(phase=phase, phase_at=now)

    def busy(self):
        return bool(self.active and self.active['phase'] not in TERMINAL)

    def observe_map(self, value):
        self.map = value

    def feedback(self, now):
        c, n = self.owner, self.owner.numbered
        return bool(c.real_uart and c.protocol_ready and recent(c.status_at, now)
                    and recent(n.host_at, now) and recent(n.point_at, now, 1.2))

    def stopped(self, now, after=None):
        c, n = self.owner, self.owner.numbered
        return bool(self.feedback(now) and c.status.get('vehicle_stopped') is True
                    and n.host.get('run') is False and
                    (after is None or (n.host_request_at is not None and n.host_request_at > after
                     and c.status_request_at is not None and c.status_request_at > after)))

    def current_reference(self, now):
        if not self.stopped(now):
            return None
        a = self.active or {}
        host = self.owner.numbered.host
        if a and (host.get('task_id') != a['task_id'] or host.get('nav_session') is not True):
            return None
        after = (a.get('pause_ack_at') if a.get('phase') == 'paused' else
                 a.get('stop_ack_at') if a.get('phase') in ('held', 'cancelled', 'failed') else a.get('done_at'))
        pose = self.map.get('pose') or {}
        if after is not None and (pose.get('stamp', -math.inf) <= after or not self.stopped(now, after)):
            return None
        return pose_reference(pose, now, a.get('task_id'))

    def start(self, request, now):
        c, n = self.owner, self.owner.numbered
        if self.busy() and self.active['phase'] != 'paused':
            raise ValueError('area_navigation_busy')
        pump_check = navigation_off_disarmed if request.get('execute_fire') is False else off_disarmed
        if not self.stopped(now) or not pump_check(c.actuator, now):
            raise ValueError('area_start_requires_measured_stop_and_pump_off')
        if (self.map.get('region_supported') is not True or not n.host.get('selected')
                or not n.host.get('scope') or not n.host.get('link_ok')):
            raise ValueError('area_firmware_or_host_page_required')
        ref = request.get('start_reference') or self.current_reference(now)
        if not isinstance(ref, dict) or ref.get('valid') is not True:
            raise ValueError('area_actual_start_pose_required')
        cells = [ref.get('cell'), request.get('goal_cell')]
        for cell in cells:
            if (not isinstance(cell, (list, tuple)) or len(cell) != 2 or
                    any(type(v) is not int for v in cell) or
                    not 0 <= cell[0] < 176 or not 0 <= cell[1] < 160):
                raise ValueError('area_goal_outside_map')
        heading = ref.get('heading_deg')
        wire.heading_payload(heading)
        guard = self._alignment_guard(request, cells[0], heading)
        continuous = request.get('continuous_route') is True
        checkpoints = request.get('patrol_waypoints', [])
        if continuous:
            if self.map.get('continuous_route_supported') is not True:
                raise ValueError('continuous_route_firmware_required')
            baseline = (self.map.get('route') or {}).get('generation')
            if type(baseline) is not int or not 0 <= baseline <= 65534:
                raise ValueError('continuous_route_generation_baseline_required')
            if (guard or not isinstance(checkpoints, list) or not 1 <= len(checkpoints) <= 8 or
                    any(not isinstance(p, dict) or not isinstance(p.get('id'), str) or not p['id'] or
                        not isinstance(p.get('cell'), (list, tuple)) or len(p['cell']) != 2 or
                        any(type(v) is not int for v in p['cell']) or
                        not 0 <= p['cell'][0] < 176 or not 0 <= p['cell'][1] < 160 for p in checkpoints) or
                    len({p['id'] for p in checkpoints}) != len(checkpoints)):
                raise ValueError('continuous_route_checkpoints_invalid')
        if guard:
            old_generation = (self.map.get('route') or {}).get('generation')
            if type(old_generation) is not int or not 0 <= old_generation <= 65534:
                raise ValueError('alignment_route_generation_baseline_required')
            goal_point = [v*20+10 for v in cells[1]]
            if self._corridor_reason(guard, goal_point) or self._projection(guard, goal_point)[0] <= 0.:
                raise ValueError('alignment_goal_not_forward_in_corridor')
        preference = request.get('region_preference_cells')
        route_policy_required = request.get('region_route_policy_required') is True
        if route_policy_required and self.map.get('region_preference_supported') is not True:
            raise ValueError('semantic_region_route_preference_firmware_required')
        if preference is not None and (not isinstance(preference, (list, tuple)) or len(preference) != 4 or
                any(type(v) is not int for v in preference) or
                not 0 <= preference[0] <= preference[2] < 176 or
                not 0 <= preference[1] <= preference[3] < 160):
            raise ValueError('invalid_region_preference_bounds')
        a = dict(command_id=request['command_id'], goal_cell=list(cells[1]),
                 start_cell=list(cells[0]), start_heading_deg=heading,
                 start_source=ref.get('source'), motion_kind=request.get('motion_kind', 'semantic_region'),
                 target_point=request.get('target_point'), task_profile=request.get('task_profile', 'extension2'),
                 region_id=request.get('region_id'), region_task_id=request.get('region_task_id'),
                 waypoint_id=request.get('waypoint_id'), leg_kind=request.get('leg_kind', 'search'),
                 execute_fire=request.get('execute_fire'),
                 continuous_route=continuous,
                 patrol_waypoints=[dict(p, cell=list(p['cell'])) for p in checkpoints] if continuous else [],
                 checkpoint_index=0, passed_waypoint_ids=[], waypoint_passages=[], waypoint_write_index=0,
                 route_generation=None, retained_alignment_active=False,
                 alignment_guard=guard,
                 previous_route_generation=(self.map.get('route') or {}).get('generation'),
                 region_preference_cells=list(preference) if preference is not None else None,
                 region_route_policy_required=route_policy_required,
                 task_id=c._motion_sequence(), phase='resetting', phase_at=now,
                 started_at=now, deadline=now+(1200. if continuous else 180.), seen_running=False,
                 reason='reset_previous_route_after_verified_stop')
        self.active = a
        self.centering.reset()
        a['sequence'] = c.send(RESET_NAV, now=now)
        c.numbered.invalidate_reference('generic_navigation_reference_consumed')
        c.reference['confirmed'] = False
        c.effects.append(('mission', dict(mode='navigate', command_id=a['command_id'],
                         arrival_policy='manual', task_profile=a['task_profile'],
                         motion_kind=a['motion_kind'], region_id=a['region_id'],
                         target_point=a['target_point'])))

    def _send(self, command, data, phase, now):
        self.active.update(phase=phase, phase_at=now)
        self.active['sequence'] = self.owner.send(command, data, now=now)

    def pause(self, command_id, now):
        a = self.active or {}
        pausable = a.get('phase') == 'running' or (
            a.get('continuous_route') and a.get('phase') in ('setting_retained_alignment_speed', 'resuming'))
        if a.get('command_id') != command_id or not pausable:
            raise ValueError('area_pause_requires_same_running_leg')
        self._send(wire.STOP, bytes(4), 'pausing', now)
        a.update(pause_at=now, pause_ack_at=None)
        a['reason'] = 'waiting_for_measured_pause'

    def _retained_identity_reason(self, now, *, stopped=False):
        a, h = self.active or {}, self.owner.numbered.host
        pose, route = self.map.get('pose') or {}, self.map.get('route') or {}
        if not a.get('continuous_route') or a.get('route_generation') is None:
            return 'retained_route_not_active'
        if not self.feedback(now):
            return 'retained_navigation_feedback_stale'
        if h.get('task_id') != a.get('task_id') or h.get('nav_session') is not True:
            return 'retained_navigation_session_changed'
        if not h.get('selected') or not h.get('scope') or not h.get('link_ok'):
            return 'retained_host_permission_lost'
        if h.get('motion_status') != 2:
            return 'retained_navigation_motion_status_changed'
        reason=pose_reference_reason(pose, now, a['task_id'])
        if reason:
            return reason
        if pose.get('route_generation') != a['route_generation']:
            return 'retained_pose_generation_changed'
        if route.get('valid') is not True or route.get('generation') != a['route_generation']:
            return 'retained_route_generation_changed'
        reference = a.get('paused_pose')
        if stopped and reference is not None and (
                math.dist(reference[:2], [pose['x_mm'], pose['y_mm']]) > 40. or
                angle_error(reference[2], pose['heading_tenths']/10.) > 8.):
            return 'retained_paused_pose_moved'
        if stopped and not self.stopped(now, a.get('pause_ack_at')):
            return 'retained_stop_not_confirmed'
        return ''

    def _retained_identity(self, now, *, stopped=False):
        return not self._retained_identity_reason(now, stopped=stopped)

    def _retained_identity_evidence(self, now):
        """Freeze the rejected sample before STOP/cancel changes its context."""
        a=self.active or {}
        pose,route=self.map.get('pose') or {},self.map.get('route') or {}
        stamp=pose.get('stamp')
        return dict(stamp=now,reason=self._retained_identity_reason(now,stopped=a.get('phase')=='paused'),
            phase=a.get('phase'),expected_task_id=a.get('task_id'),
            expected_route_generation=a.get('route_generation'),pose=dict(pose),
            pose_age_s=now-stamp if type(stamp) in (int,float) else None,
            paused_pose=list(a['paused_pose']) if a.get('paused_pose') is not None else None,
            route_valid=route.get('valid'),route_generation=route.get('generation'),
            map_reason=self.map.get('reason'),map_stamp=self.map.get('stamp'),
            host=dict(self.owner.numbered.host),status_at=self.owner.status_at,
            host_at=self.owner.numbered.host_at,point_at=self.owner.numbered.point_at)

    def resume_write_ready(self, sequence, now):
        a = self.active or {}
        check = navigation_off_disarmed if a.get('execute_fire') is False else off_disarmed
        return bool(not self.centering.active and a.get('phase') == 'resuming' and a.get('sequence') == sequence and
                    self._retained_identity(now, stopped=True) and check(self.owner.actuator, now) and
                    not self.owner.mode_transition and not self.owner.zebra_transition)

    def _resume(self, now):
        a = self.active
        if self.centering.active:
            raise ValueError('continuous_resume_requires_centering_anchor_restored')
        check = navigation_off_disarmed if a.get('execute_fire') is False else off_disarmed
        if not self._retained_identity(now, stopped=True) or not check(self.owner.actuator, now):
            raise ValueError('continuous_resume_requires_same_route_stop_and_pump_off')
        a.update(resume_ack_at=None, last_progress_pose_stamp=None)
        self._send(wire.RESUME, bytes(4), 'resuming', now)
        a['reason'] = 'continuing_retained_fpga_route'

    def resume(self, command_id, now):
        a = self.active or {}
        if a.get('command_id') != command_id or a.get('phase') != 'paused':
            raise ValueError('continuous_resume_requires_same_paused_route')
        a['pause_kind'] = None
        self._resume(now)

    def advance_checkpoint(self, now):
        raise ValueError('waypoints_advance_from_measured_route_progress')

    def begin_centering(self, now):
        return self.centering.begin(now)

    def centering_proposal(self, direction, now, step_mm=40, speed_mm_s=40):
        if isinstance(direction, str):
            direction = {'forward': 1, 'reverse': -1}.get(direction)
        return self.centering.proposal(direction, now, step_mm, speed_mm_s)

    def begin_centering_jog(self, proposal, now):
        return self.centering.begin_jog(proposal, now)

    def centering_write_ready(self, sequence, now):
        return self.centering.write_ready(sequence, now)

    def stop_centering_jog(self, now):
        return self.centering.stop_jog(now)

    def centering_return_proposal(self, now):
        return self.centering.return_proposal(now)

    def end_centering(self, now):
        return self.centering.end(now)

    def alignment_proposal(self, now, max_advance_cm=30.):
        a = self.active or {}
        ref = self.current_reference(now)
        if a.get('phase') != 'paused' or not self._retained_identity(now, stopped=True) or not ref:
            return dict(stamp=now, valid=False, reason='checkpoint_alignment_retained_stop_required')
        pose = self.map.get('pose') or {}
        ref['image_mm'] = [pose['x_mm'], 3200-pose['y_mm']]
        return forward_proposal(a['retained_route'], ref, pose.get('execution_index'),
            a.get('progress_index', 0), a.get('progress_mm', 0.), now, max_advance_cm)

    def begin_retained_alignment(self, proposal, now):
        a = self.active or {}
        if a.get('phase') != 'paused' or not self._retained_identity(now, stopped=True):
            raise ValueError('checkpoint_alignment_retained_stop_required')
        guard = proposal.get('alignment_guard') or {}
        checked = self.alignment_proposal(now, guard.get('max_advance_cm', 0.))
        if (proposal.get('valid') is not True or proposal.get('retained_route') is not True or
                checked.get('valid') is not True or proposal.get('route_generation') != a['route_generation'] or
                proposal.get('stop_route_index') != checked.get('stop_route_index') or
                proposal.get('stop_progress_mm') != checked.get('stop_progress_mm')):
            raise ValueError('checkpoint_alignment_retained_proposal_changed')
        check = navigation_off_disarmed if a.get('execute_fire') is False else off_disarmed
        if not check(self.owner.actuator, now):
            raise ValueError('checkpoint_alignment_pump_off_required')
        a.update(retained_alignment_active=True, retained_alignment=checked,
                 retained_alignment_completed=False,
                 pause_kind='alignment', retained_alignment_start_pose=self.current_reference(now),
                 retained_alignment_guard=checked['alignment_guard'])
        self.restore_speed_mm_s = a['cruise_speed_mm_s']
        self._send(SET_SPEED, struct.pack('<HH', 60, 0), 'setting_retained_alignment_speed', now)
        a['reason'] = 'checkpoint_alignment_setting_low_speed'

    def _continuous_route_validation(self, now):
        a, route = self.active, self.map.get('route') or {}
        if (route.get('valid') is not True or route.get('sampled') is not False or
                route.get('generation') == a.get('previous_route_generation') or
                not recent(route.get('stamp'), now, 5.) or route.get('stamp', 0.) <= a['plan_ack_at']):
            return None
        try:
            points, lengths, references = waypoint_references(route, a['patrol_waypoints'])
            if (math.dist(points[0], [v*20+10 for v in a['start_cell']]) > 80. or
                    math.dist(points[-1], [v*20+10 for v in a['goal_cell']]) > 80.):
                return 'continuous_route_endpoint_mismatch'
        except (KeyError, TypeError, ValueError) as exc:
            return str(exc)
        a.update(retained_route=dict(route, points_mm=[list(p) for p in points]),
            route_generation=route['generation'], route_points=points, route_lengths=lengths,
            waypoint_references=references, progress_index=0, progress_mm=0., last_progress_pose_stamp=None)
        return ''

    def _continuous_tick(self, now):
        a, pose = self.active, self.map.get('pose') or {}
        if not self._retained_identity(now):
            # A lost atomic cursor reply invalidates that sample. It cannot
            # advance a waypoint, and one missing sample should not cancel an
            # otherwise healthy UART session after minutes of travel.
            a.setdefault('running_identity_missing_at', now)
            if now - a['running_identity_missing_at'] > 1.2:
                self.stop(now, 'continuous_route_or_pose_identity_lost')
            return
        a.pop('running_identity_missing_at', None)
        if pose.get('stamp', 0) == a.get('last_progress_pose_stamp'):
            return
        a['last_progress_pose_stamp'] = pose['stamp']
        position = [pose['x_mm'], 3200-pose['y_mm']]
        try:
            index, progress, lateral = project_progress(a['route_points'], a['route_lengths'], position,
                pose.get('execution_index'), a.get('progress_index', 0), a.get('progress_mm', 0.))
        except ValueError as exc:
            self.stop(now, str(exc))
            return
        a.update(progress_index=index, progress_mm=progress, route_lateral_error_mm=lateral)
        references = a['waypoint_references']
        while a['checkpoint_index'] < len(references) and progress >= references[a['checkpoint_index']]['progress_mm']:
            reference = references[a['checkpoint_index']]
            a['passed_waypoint_ids'].append(reference['id'])
            a['waypoint_passages'].append(dict(id=reference['id'], stamp=pose['stamp'],
                route_progress_mm=progress, measured_image_mm=position,
                physical_position_verified=False))
            a['checkpoint_index'] += 1
        if a.get('retained_alignment_active'):
            proposal, guard = a['retained_alignment'], a['retained_alignment_guard']
            reason = self._corridor_reason(guard, position)
            if angle_error(pose['heading_tenths']/10., guard['heading_deg']) > 15.:
                reason = 'alignment_actual_heading_exceeded'
            if reason or progress >= proposal['stop_progress_mm']-10.:
                self.pause(a['command_id'], now)
                a.update(pause_kind='alignment', reason=reason or 'retained_alignment_distance_reached')
            return

    def stop(self, now, reason, sequence=None):
        a = self.active
        if not a or a['phase'] in ('failed', 'cancelled'):
            return
        if 'identity_failure' not in a and reason in ('continuous_route_or_pose_identity_lost', 'continuous_paused_route_or_pose_changed',
                      'continuous_stopped_route_changed', 'continuous_resume_route_changed',
                      'centering_same_route_pose_lost'):
            a['identity_failure']=self._retained_identity_evidence(now)
        self.centering.cancel(reason)
        a.update(phase='stopping', phase_at=now, stop_at=now, reason=reason,
                 stop_ack_at=None)
        a['sequence'] = sequence if sequence is not None else self.owner.send(wire.STOP, now=now)

    def on_reply(self, packet, sent_at, now):
        if self.centering.on_reply(packet, sent_at, now):
            return
        a = self.active
        if not a or packet.sequence != a.get('sequence'):
            return
        command = packet.command & 0x7f
        expected = {'resetting': RESET_NAV, 'setting_region_preference': SET_REGION_PREFERENCE,
                    'setting_start': wire.SET_START,
                    'setting_heading': wire.SET_START_HEADING, 'setting_waypoint': wire.WAYPOINT,
                    'setting_goal': wire.SET_GOAL,
                    'plan_ack': wire.PLAN, 'starting': wire.START,
                    'setting_alignment_speed': SET_SPEED, 'setting_retained_alignment_speed': SET_SPEED,
                    'restoring_speed': SET_SPEED, 'resuming': wire.RESUME,
                    'pausing': wire.STOP, 'stopping': wire.STOP}.get(a['phase'])
        if command != expected:
            return
        if packet.data[0] != 0:
            if a['phase'] == 'restoring_speed':
                a.update(phase='failed', phase_at=now, reason='alignment_speed_restore_rejected')
                return
            self.stop(now, 'area_fpga_rejected_%02x_%d' % (command, packet.data[0]))
            return
        if command == RESET_NAV:
            if a.get('continuous_route'):
                speed = int.from_bytes(packet.data[2:4], 'little')
                if not 1 <= speed <= 1000:
                    self.stop(now, 'continuous_original_speed_invalid')
                    return
                a['cruise_speed_mm_s'] = speed
            if a.get('alignment_guard') and self.restore_speed_mm_s is None:
                self.restore_speed_mm_s = int.from_bytes(packet.data[2:4], 'little')
                if not 0 <= self.restore_speed_mm_s <= 1000:
                    self.restore_speed_mm_s = None
                    self.stop(now, 'alignment_original_speed_invalid')
                    return
            a.update(phase='reset_readback', phase_at=now, reason='waiting_reset_readback')
        elif command == SET_SPEED:
            wanted = self.restore_speed_mm_s if a['phase'] == 'restoring_speed' else 60
            if int.from_bytes(packet.data[2:4], 'little') != wanted:
                if a['phase'] == 'restoring_speed':
                    a.update(phase='failed', phase_at=now, reason='alignment_speed_restore_ack_mismatch')
                else:
                    self.stop(now, 'alignment_speed_ack_mismatch')
                return
            if a['phase'] == 'restoring_speed':
                if not self.stopped(now):
                    self.stop(now, 'alignment_speed_restore_vehicle_not_stopped')
                    return
                self.restore_speed_mm_s = None
                phase = a.pop('after_speed_restore')
                if phase == 'prepare_start':
                    self._prepare_start(now)
                else:
                    a.update(phase=phase, phase_at=now, reason=a.pop('terminal_reason'))
            elif a['phase'] == 'setting_retained_alignment_speed':
                try:
                    self._resume(now)
                except ValueError as exc:
                    self.stop(now, str(exc))
            else:
                self._prepare_start(now)
        elif command == SET_REGION_PREFERENCE:
            self._send(wire.SET_START, bytes((*a['start_cell'], 0, 0)), 'setting_start', now)
        elif command == wire.SET_START:
            self._send(wire.SET_START_HEADING, wire.heading_payload(a['start_heading_deg']), 'setting_heading', now)
        elif command == wire.SET_START_HEADING:
            if packet.data != bytes((0,)) + wire.heading_payload(a['start_heading_deg'])[:2] + bytes((0,)):
                self.stop(now, 'area_heading_ack_mismatch')
                return
            if a.get('continuous_route'):
                point = a['patrol_waypoints'][0]
                self._send(wire.WAYPOINT, bytes((*point['cell'], 0, 0)), 'setting_waypoint', now)
            else:
                self._send(wire.SET_GOAL, bytes((*a['goal_cell'], 0, 0)), 'setting_goal', now)
        elif command == wire.WAYPOINT:
            a['waypoint_write_index'] += 1
            if a['waypoint_write_index'] < len(a['patrol_waypoints']):
                point = a['patrol_waypoints'][a['waypoint_write_index']]
                self._send(wire.WAYPOINT, bytes((*point['cell'], 0, 0)), 'setting_waypoint', now)
            else:
                self._send(wire.SET_GOAL, bytes((*a['goal_cell'], 0, 0)), 'setting_goal', now)
        elif command == wire.SET_GOAL:
            self._send(wire.PLAN, bytes(4), 'plan_ack', now)
        elif command == wire.PLAN:
            a.update(phase='planning', phase_at=now, plan_ack_at=now, reason='fpga_astar_planning')
        elif command == wire.START:
            a.update(phase='running', phase_at=now, start_ack_at=now, reason='awaiting_actual_navigation')
        elif command == wire.RESUME:
            a['resume_ack_at'] = now
        elif command == wire.STOP:
            a['pause_ack_at' if a['phase'] == 'pausing' else 'stop_ack_at'] = now

    def on_event(self, packet, now):
        a = self.active
        if (not a or packet.command not in (wire.DONE, wire.FAILED, wire.LINK_TIMEOUT)
                or packet.sequence != 0 or packet.data[0] != a['task_id']):
            return False
        if a['phase'] in TERMINAL or a['phase'] == 'stopping':
            return True
        if packet.command != wire.DONE:
            self.stop(now, 'area_fpga_motion_failed_%d' % packet.data[1])
        elif (packet.data[1:] == b'\x03\0\0' and a.get('start_ack_at') is not None
              and now >= a['start_ack_at'] and self.feedback(now)
              and a['phase'] in ('running', 'pausing', 'paused')):
            if a.get('continuous_route') and a['checkpoint_index'] < len(a['patrol_waypoints']):
                self.stop(now, 'continuous_final_arrival_before_all_checkpoints')
                return True
            a.update(phase='confirming_arrival', phase_at=now, done_at=now,
                     reason='matching_done_waiting_measured_stop')
        return True

    def tick(self, now):
        a, c, n = self.active, self.owner, self.owner.numbered
        if not a or a['phase'] in TERMINAL:
            return
        h, p, phase = n.host, n.point, a['phase']
        if phase in CENTERING_PHASES:
            self.centering.tick(now)
            return
        if phase == 'stopping':
            if a.get('stop_ack_at') is not None and self.stopped(now, a['stop_ack_at']):
                terminal = ('arrived' if a['reason'] in ('no_fire_found', 'region_search_finished', 'region_search_exited_and_stopped')
                              else 'held' if a['reason'] == 'fire_found_waiting_operator'
                              else 'cancelled' if a['reason'] in ('operator_stop', 'visual_task_selected', 'traffic_task_selected')
                              else 'failed')
                self._finish(terminal, now)
            elif now-a['phase_at'] > .6:
                a['sequence'] = c.send(wire.STOP, now=now)
                a['phase_at'] = now
            return
        if phase == 'restoring_speed':
            if not self.stopped(now):
                self.stop(now, 'alignment_speed_restore_vehicle_not_stopped')
            elif now-a['phase_at'] > 2.:
                a.update(phase='failed', phase_at=now, reason='alignment_speed_restore_ack_timeout')
            return
        if not self.feedback(now):
            self.stop(now, 'area_navigation_feedback_lost')
            return
        if not h.get('selected') or not h.get('scope') or not h.get('link_ok'):
            self.stop(now, 'area_navigation_host_permission_lost')
            return
        if now >= a['deadline'] and phase not in ('paused', 'pausing', 'resuming', 'setting_retained_alignment_speed'):
            self.stop(now, 'area_navigation_timeout')
            return
        if phase == 'planning':
            if n.point_request_at is None or n.point_request_at <= a['phase_at']:
                return
            if p.get('selected_point') != 0 or p.get('state') == 3:
                self.stop(now, 'area_astar_plan_failed')
            elif p.get('state') == 2 and not h.get('nav_session') and not h.get('run'):
                if a.get('continuous_route'):
                    a.update(phase='continuous_route_check', phase_at=now, reason='waiting_complete_mission_route')
                elif a.get('alignment_guard'):
                    a.update(phase='alignment_route_check', phase_at=now, reason='alignment_waiting_complete_new_route')
                else:
                    self._send(wire.START, bytes((1, a['task_id'], 0, 0)), 'starting', now)
            elif now-a['phase_at'] > 60.:
                self.stop(now, 'area_astar_plan_timeout')
            return
        if phase == 'continuous_route_check':
            if p.get('selected_point') != 0 or p.get('state') == 3:
                self.stop(now, 'area_astar_plan_failed')
                return
            if h.get('nav_session'):
                self.stop(now, 'continuous_prestart_session_changed')
                return
            reason = self._continuous_route_validation(now) if p.get('state') == 2 else None
            if reason is None:
                # 2048 points share idle UART slots with heartbeat, pose and
                # status. The maximum route can exceed two minutes to read;
                # it remains stationary until the entire generation verifies.
                if now-a['phase_at'] >= 240.:
                    self.stop(now, 'continuous_complete_new_route_timeout')
            elif reason:
                self.stop(now, reason)
            elif not self.stopped(now):
                self.stop(now, 'continuous_prestart_vehicle_not_stopped')
            else:
                self._send(wire.START, bytes((1, a['task_id'], 0, 0)), 'starting', now)
            return
        if phase == 'alignment_route_check':
            if p.get('selected_point') != 0 or p.get('state') == 3:
                self.stop(now, 'area_astar_plan_failed')
                return
            if h.get('nav_session'):
                self.stop(now, 'alignment_prestart_session_changed')
                return
            reason = self._route_validation(now)
            if p.get('state') != 2:
                reason = None
            if reason is None:
                if now-a['phase_at'] >= 5.:
                    self.stop(now, 'alignment_complete_new_route_timeout')
            elif reason:
                self.stop(now, reason)
            elif not self.stopped(now):
                self.stop(now, 'alignment_prestart_vehicle_not_stopped')
            else:
                self._send(wire.START, bytes((1, a['task_id'], 0, 0)), 'starting', now)
            return
        if phase == 'reset_readback':
            if self.stopped(now, a['phase_at']) and h.get('nav_session') is False:
                if a.get('alignment_guard'):
                    self._send(SET_SPEED, struct.pack('<HH', 60, 0), 'setting_alignment_speed', now)
                elif self.restore_speed_mm_s is not None:
                    self._finish('prepare_start', now)
                else:
                    self._prepare_start(now)
            elif now-a['phase_at'] > 3.:
                self.stop(now, 'area_reset_readback_timeout')
            return
        if phase in ('running', 'pausing', 'paused', 'resuming', 'confirming_arrival'):
            if n.host_request_at is None or n.host_request_at <= a.get('start_ack_at', a['started_at']):
                return
            if h.get('task_id') != a['task_id'] or h.get('nav_session') is not True:
                self.stop(now, 'area_navigation_identity_changed')
                return
            if h.get('motion_status', 0) >= 4:
                self.stop(now, 'area_navigation_motion_failed')
                return
            if a.get('alignment_guard') and phase in ('running', 'pausing', 'confirming_arrival'):
                reason = self._alignment_pose_reason(now)
                if reason:
                    self.stop(now, reason)
                    return
                if reason is None:
                    return
            if phase == 'running':
                if h.get('run') and h.get('motion_status') == 2:
                    a['seen_running'] = True
                    a['reason'] = 'fpga_navigation_running'
                    if a.get('continuous_route'):
                        self._continuous_tick(now)
                elif h.get('motion_status') == 3:
                    a.setdefault('terminal_seen_at', now)
                    if now-a['terminal_seen_at'] > 2.:
                        self.stop(now, 'area_matching_done_missing')
                elif h.get('run') is False:
                    self.stop(now, 'area_navigation_stopped_without_arrival')
            elif phase == 'pausing':
                ack = a.get('pause_ack_at')
                if ack is not None and self.stopped(now, ack):
                    if a.get('continuous_route'):
                        pose = self.map.get('pose') or {}
                        if (pose.get('stamp', -math.inf) <= ack or
                                not pose_reference(pose, now, a['task_id'])):
                            if now-ack > 1.2:
                                self.stop(now, 'continuous_stop_pose_unconfirmed')
                            return
                        if pose.get('route_generation') != a['route_generation']:
                            self.stop(now, 'continuous_stopped_route_changed')
                            return
                    a['reason'] = 'area_pause_verified'
                    self._finish('paused', now)
                elif now-a['phase_at'] > 3.:
                    self.stop(now, 'area_pause_timeout')
            elif phase == 'paused':
                if not self.stopped(now):
                    self.stop(now, 'area_paused_vehicle_moved')
                elif a.get('continuous_route') and not self._retained_identity(now, stopped=True):
                    a.setdefault('paused_identity_missing_at', now)
                    if now-a['paused_identity_missing_at'] > 1.2:
                        self.stop(now, 'continuous_paused_route_or_pose_changed')
                else:
                    a.pop('paused_identity_missing_at', None)
            elif phase == 'resuming':
                ack = a.get('resume_ack_at')
                if ack is not None and n.host_request_at > ack and h.get('run') is True:
                    if not self._retained_identity(now):
                        self.stop(now, 'continuous_resume_route_changed')
                    else:
                        a['deadline'] += now-a.pop('pause_at', now)
                        a.update(phase='running', phase_at=now, reason='fpga_retained_navigation_running')
                elif now-a['phase_at'] > 3.:
                    self.stop(now, 'continuous_resume_confirmation_timeout')
            elif (self.stopped(now, a['done_at']) and h.get('motion_status') == 3
                  and (navigation_off_disarmed(c.actuator, now) if a.get('execute_fire') is False
                       else off_disarmed(c.actuator, now))):
                pose = self.map.get('pose') or {}
                ref = pose_reference(pose, now, a['task_id'])
                if pose.get('stamp', -math.inf) > a['done_at'] and ref:
                    # Planner goal capture is 6 cm; include cell quantization
                    # without letting distance replace DONE + stopped evidence.
                    if any(abs(ref['cell'][i]-a['goal_cell'][i]) > 4 for i in (0, 1)):
                        self.stop(now, 'area_terminal_pose_not_at_goal')
                    else:
                        a.update(reason='matching_done_measured_stop_pose', arrival_stamp=now)
                        self._finish('arrived', now)
                elif now-a['phase_at'] > 3.:
                    self.stop(now, 'area_arrival_pose_not_confirmed')
            elif now-a['phase_at'] > 3.:
                self.stop(now, 'area_arrival_stop_not_confirmed')
        elif now-a['phase_at'] > 2.:
            self.stop(now, 'area_navigation_ack_timeout')

    def state(self, now):
        a = self.active or {}
        phase = a.get('phase', 'idle')
        h = self.owner.numbered.host
        same = bool(a and h.get('task_id') == a.get('task_id') and h.get('nav_session'))
        pose = self.map.get('pose') or {}
        ref = pose_reference(pose, now, a.get('task_id')) if same else None
        if phase == 'paused' and (a.get('pause_ack_at') is None or pose.get('stamp', -math.inf) <= a['pause_ack_at']):
            ref = None
        if (a.get('continuous_route') and phase in ('running', 'pausing', 'paused', 'resuming',
                'setting_retained_alignment_speed', 'restoring_speed') and
                not self._retained_identity(now, stopped=phase == 'paused')):
            # Search must wait for the same atomic pose/route snapshot too;
            # otherwise it might consume a checkpoint during telemetry grace.
            ref = None
        arrived = bool(phase == 'arrived' and same and self.stopped(now) and h.get('motion_status') == 3)
        stopped = self.stopped(now)
        return dict(stamp=now, command_id=a.get('command_id'), task_id=a.get('task_id'),
            phase=phase, status=phase, stopped=stopped, vehicle_stopped=stopped,
            pose_valid=bool(ref), pose_cell=ref['cell'] if ref else None,
            pose_heading_deg=ref['heading_deg'] if ref else None,
            arrived=arrived, valid=bool(self.feedback(now) and same),
            ready_to_move=bool(phase == 'running' and same and self.feedback(now) and h.get('run')),
            status_fresh=self.feedback(now), failed=phase == 'failed', cancelled=phase == 'cancelled',
            reason=a.get('reason', 'area_navigation_idle'), arrival_policy='manual',
            identity_failure=a.get('identity_failure'),
            owner='fpga', motion_kind=a.get('motion_kind', 'semantic_region'),
            task_profile=a.get('task_profile', 'extension2'), origin='x5',
            start_point=None, target_point=a.get('target_point'), start_cell=a.get('start_cell'),
            start_heading_deg=a.get('start_heading_deg'), heading_source=a.get('start_source'),
            region_id=a.get('region_id'), goal_cell=a.get('goal_cell'),
            leg_kind=a.get('leg_kind'), arrival_stamp=a.get('arrival_stamp'),
            region_preference_cells=a.get('region_preference_cells'),
            alignment_guard=a.get('alignment_guard'),
            alignment_route_generation=a.get('alignment_route_generation'),
            alignment_route_length_mm=a.get('alignment_route_length_mm'),
            speed_restore_pending=self.restore_speed_mm_s is not None,
            retained_route=a.get('continuous_route', False),
            route_generation=a.get('route_generation'), checkpoint_index=a.get('checkpoint_index'),
            checkpoint_count=len(a.get('patrol_waypoints', [])),
            checkpoint_ready=False,
            waypoint_references=a.get('waypoint_references', []),
            passed_waypoint_count=a.get('checkpoint_index', 0),
            passed_waypoint_ids=list(a.get('passed_waypoint_ids', [])),
            waypoint_passages=list(a.get('waypoint_passages', [])),
            route_progress_mm=a.get('progress_mm'),
            retained_alignment_active=a.get('retained_alignment_active', False),
            retained_alignment_finished=bool(a.get('retained_alignment_completed') and phase == 'paused' and
                self.restore_speed_mm_s is None and self._retained_identity(now, stopped=True)),
            retained_alignment=a.get('retained_alignment'),
            centering=self.centering.state(now),
            route_download=self.map.get('route_download', {}),
            physical_position_verified=False, pose_source='encoder_imu_dead_reckoning')
