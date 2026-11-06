"""Numbered FPGA destinations over the existing exclusive fixed-10 UART.

No I/O or pose estimation lives here. Arrival needs a witnessed session, its
DONE event, and subsequent queried terminal/stop evidence. An explicit starting
point declaration is operator input, not a measured physical starting pose.
"""
import math
import uuid
from . import ov5640_protocol as wire
from .route_fire_patrol import RouteFirePatrol, off_disarmed


TERMINAL = ('arrived', 'cancelled', 'failed')


def recent(stamp, now, ttl=.8):
    return type(stamp) in (int, float) and math.isfinite(stamp) and 0 <= now-stamp <= ttl


class NumberedNavigation:
    def __init__(self, owner):
        self.owner = owner
        self.points = {}
        self.active = None
        self.reference = dict(confirmed=False, reason='confirm_physical_numbered_start')
        self.host = {}
        self.host_at = self.host_request_at = None
        self.point = {}
        self.point_at = self.point_request_at = None
        self.supported = False
        self.heading_supported = False
        self.map_pose = {}
        self.last_poll = -math.inf
        self.observed_tasks = set()
        self.session = uuid.uuid4().hex
        self.observation_count = 0
        self.last_observation_reason = 'no_numbered_session_observed'
        self.patrol = RouteFirePatrol(self)

    def load_catalog(self, catalog):
        points = catalog.get('points', []) if isinstance(catalog, dict) else []
        indexed = {}
        for point in points:
            if not isinstance(point, dict):
                raise ValueError('invalid_numbered_catalog')
            number, cell = point.get('id'), point.get('planner_cell')
            if (type(number) is not int or number in indexed or not 1 <= number <= 9
                    or not isinstance(cell, list) or len(cell) != 2
                    or type(cell[0]) is not int or not 0 <= cell[0] <= 175
                    or type(cell[1]) is not int or not 0 <= cell[1] <= 159):
                raise ValueError('invalid_numbered_catalog')
            indexed[number] = dict(point)
        if set(indexed) != set(range(1, 10)):
            raise ValueError('numbered_catalog_requires_nine_points')
        self.points = indexed

    def busy(self):
        return bool(self.active and self.active['phase'] not in TERMINAL)

    def invalidate_reference(self, reason):
        self.reference = dict(confirmed=False, reason=reason)

    def observe_map(self, snapshot):
        self.heading_supported = snapshot.get('heading_supported') is True
        self.map_pose = dict(snapshot.get('pose') or {})

    def reference_heading(self, point, now):
        ref = self.reference
        if ref.get('confirmed') is not True or ref.get('point_id') != point:
            return None
        if ref.get('source') == 'confirmed_arrival':
            pose, active = self.map_pose, self.active or {}
            if (pose.get('valid') is True and recent(pose.get('stamp'), now, 1.2)
                    and pose.get('mode') == 9 and pose.get('status') == 3
                    and pose.get('task_id') == active.get('task_id')
                    and active.get('phase') == 'arrived'):
                value = pose.get('heading_tenths')
                if type(value) is int and -1800 <= value <= 1800:
                    return value / 10.
            return None
        value = ref.get('start_heading_deg')
        if type(value) in (int, float) and math.isfinite(value) and -180 <= value <= 180:
            return value
        return None

    def start_reference(self, now):
        """Describe a confirmed origin without inventing a numbered start."""
        area = getattr(self.owner, 'area', None)
        map_ref = area.reference(now) if area is not None else {}
        if map_ref.get('valid') is True:
            cell, heading = map_ref.get('cell'), map_ref.get('heading_deg')
            if (not isinstance(cell, (list, tuple)) or len(cell) != 2 or
                    any(type(v) is not int for v in cell) or
                    not 0 <= cell[0] < 176 or not 0 <= cell[1] < 160 or
                    type(heading) not in (int, float) or not math.isfinite(heading) or
                    not -180 <= heading <= 180):
                map_ref = dict(valid=False)
        if (map_ref.get('valid') is True and
                map_ref.get('source') in ('operator_hmi', 'fpga_touchscreen')):
            return dict(map_ref, kind='map')
        point = self.reference.get('point_id')
        heading = self.reference_heading(point, now)
        if heading is not None:
            return dict(valid=True, kind='numbered', point_id=point,
                        cell=list(self.points[point]['planner_cell']), heading_deg=heading,
                        source=self.reference.get('source'), physical_position_verified=False)
        if map_ref.get('valid') is True:
            return dict(map_ref, kind='map')
        return dict(valid=False, reason=('numbered_start_heading_required'
            if self.reference.get('confirmed') is True else 'confirm_physical_numbered_start'))

    def map_start_readiness(self, now):
        area = self.owner.area
        if area.navigation.map.get('region_supported') is not True:
            return 'semantic_region_fpga_firmware_required'
        if (area.map_signature is None or
                area.navigation.map.get('map_signature') != area.map_signature):
            return 'semantic_region_map_identity_mismatch'
        if not off_disarmed(self.owner.actuator, now):
            return 'area_start_requires_measured_stop_and_pump_off'
        return ''

    def readiness(self, now, require_reference=True):
        c = self.owner
        if getattr(c, 'area', None) is not None and c.area.busy():
            return 'semantic_region_search_in_progress'
        if not c.real_uart:
            return 'real_extension_uart_required'
        if not c.protocol_ready or not recent(c.status_at, now):
            return 'extension_firmware_or_feedback_unavailable'
        if not c.status.get('vehicle_stopped'):
            return 'measured_stop_required'
        if not recent(self.host_at, now) or not self.host.get('selected') or not self.host.get('scope'):
            return 'select_fpga_communication_navigation_page'
        if not self.host.get('link_ok'):
            return 'fpga_heartbeat_not_ready'
        if self.host.get('run') or self.busy() or c.mode_transition or c.zebra_transition:
            return 'numbered_navigation_busy'
        if c.active and c.active.get('phase') not in ('completed', 'fault'):
            return 'extension_movement_in_progress'
        if c.task == 'traffic':
            return 'leave_traffic_task_before_numbered_navigation'
        if not self.points:
            return 'numbered_catalog_unavailable'
        if not self.heading_supported:
            return 'numbered_heading_firmware_required'
        if require_reference:
            if not self.supported or not recent(self.point_at, now, 1.2):
                return 'numbered_firmware_status_unavailable'
            ref = self.start_reference(now)
            if ref.get('valid') is not True:
                return ref['reason']
            if ref['kind'] == 'map':
                return self.map_start_readiness(now)
        return ''

    def confirm_start(self, request, now):
        reason = self.readiness(now, require_reference=False)
        if reason:
            raise ValueError(reason)
        if request.get('source') != 'web' or request.get('confirmed') is not True:
            raise ValueError('physical_start_requires_explicit_operator_confirmation')
        point = request.get('point_id')
        if type(point) is not int or point not in self.points:
            raise ValueError('numbered_point_requires_1_to_9')
        heading = request.get('start_heading_deg')
        if heading is None:
            raise ValueError('numbered_start_heading_required')
        wire.heading_payload(heading)
        self.reference = dict(confirmed=True, point_id=point, source='operator', stamp=now,
                              start_heading_deg=heading, position_and_heading_confirmed=True,
                              physical_position_verified=False)
        self.owner.reference['confirmed'] = False
        self.owner.area.consumed_panel_reference = (
            self.owner.area.navigation.map.get('start_reference') or {}).get('generation')

    def start(self, request, now):
        self.patrol.validate_start(request, now)
        declared = request.get('start_declared') is True
        reason = self.readiness(now, require_reference=not declared)
        if reason:
            raise ValueError(reason)
        point = request.get('target_point')
        if type(point) is not int or point not in self.points:
            raise ValueError('numbered_point_requires_1_to_9')
        if not declared and self.start_reference(now).get('kind') == 'map':
            if request.get('start_point') is not None:
                raise ValueError('spoken_start_does_not_match_confirmed_location')
            ref = self.start_reference(now)
            if (request.get('start_heading_deg') is not None and
                    request['start_heading_deg'] != ref['heading_deg']):
                raise ValueError('spoken_heading_does_not_match_confirmed_location')
            goal = self.points[point]['planner_cell']
            if list(ref['cell']) == goal:
                raise ValueError('already_at_confirmed_point_no_navigation_started')
            area = self.owner.area
            area.navigation.start(dict(command_id=request['command_id'],
                goal_cell=goal, start_reference=ref, motion_kind='numbered_point',
                target_point=point, task_profile=('extension1' if self.owner.task == 'vision' else 'extension2'),
                leg_kind='numbered_destination'), now)
            area.consumed_panel_reference = (
                area.navigation.map.get('start_reference') or {}).get('generation')
            return
        if declared:
            from .task_interaction import parse_task_text
            parsed = parse_task_text(request.get('text', ''))
            if (request.get('source') not in ('voice', 'text', 'web') or
                    parsed.get('action') != 'navigate_point' or parsed.get('start_declared') is not True or
                    any(parsed.get(k) != request.get(k) for k in ('start_point', 'target_point', 'start_heading_deg'))):
                raise ValueError('explicit_start_statement_required')
            if not self.supported or not recent(self.point_at, now, 1.2):
                raise ValueError('numbered_firmware_status_unavailable')
            start = request.get('start_point')
            if type(start) is not int or start not in self.points or start == point:
                raise ValueError('invalid_declared_start_point')
            heading = request.get('start_heading_deg')
            if heading is None:
                heading = self.reference_heading(start, now)
            if heading is None:
                raise ValueError('numbered_start_heading_required')
            wire.heading_payload(heading)
            self.reference = dict(confirmed=True, point_id=start, source='operator_statement', stamp=now,
                start_heading_deg=heading, position_and_heading_confirmed=True, physical_position_verified=False)
        start = request.get('start_point', self.reference['point_id'])
        if type(start) is not int or start != self.reference['point_id']:
            raise ValueError('spoken_start_does_not_match_confirmed_location')
        if point == start:
            raise ValueError('already_at_confirmed_point_no_navigation_started')
        heading = request.get('start_heading_deg')
        heading_source = 'operator_statement' if heading is not None else self.reference.get('source')
        if heading is None:
            heading = self.reference_heading(start, now)
        if heading is None:
            raise ValueError('numbered_start_heading_required')
        wire.heading_payload(heading)
        # Never recycle an event's one-byte task identity in this process.
        task_id = self.owner._motion_sequence()
        while task_id in self.observed_tasks:
            task_id = self.owner._motion_sequence()
        self.observed_tasks.add(task_id)
        profile = 'extension1' if self.owner.task == 'vision' else 'extension2'
        policy = 'manual'
        self.active = dict(command_id=request['command_id'], action='navigate_point',
            origin='x5', start_point=start, target_point=point, task_id=task_id,
            task_profile=profile, arrival_policy=policy, phase='setting_start',
            start_source=self.reference.get('source'), physical_position_verified=False,
            start_heading_deg=heading, heading_source=heading_source,
            started_at=now, phase_at=now, deadline=now+180., seen_running=False,
            done_at=None, arrival_stamp=None, reason='uploading_confirmed_start',
            request_sequence=None, patrol_fire=request.get('patrol_fire') is True)
        self.patrol.start(self.active['patrol_fire'], now)
        self.invalidate_reference('navigation_requested_reference_consumed')
        self.owner.reference['confirmed'] = False
        self.owner.area.consumed_panel_reference = (
            self.owner.area.navigation.map.get('start_reference') or {}).get('generation')
        cell = self.points[start]['planner_cell']
        self.active['request_sequence'] = self.owner.send(wire.SET_START, bytes((*cell, 0, 0)), now=now)
        self._mission()

    def _mission(self):
        a = self.active
        self.owner.effects.append(('mission', dict(mode='navigate', command_id=a['command_id'],
            arrival_policy=a['arrival_policy'], task_profile=a['task_profile'],
            motion_kind='numbered_point', target_point=a['target_point'])))

    def cancel(self, now, reason, stop_sequence=None):
        if not self.busy():
            return
        self.patrol.cancel(now, reason)
        a = self.active
        a.update(phase='stopping', phase_at=now, stop_at=now, reason=reason,
                 done_at=None, arrival_stamp=None, stop_sequence=stop_sequence,
                 stop_ack_at=None)
        self.invalidate_reference('navigation_cancelled_location_unknown')

    def fail(self, now, reason):
        if self.busy() and self.active['phase'] != 'stopping':
            self.owner.stop(now, reason)

    def pause_for_fire(self, now):
        a = self.active
        if not a or not a.get('patrol_fire') or a['phase'] != 'running':
            raise ValueError('patrol_pause_requires_running_route')
        a.update(phase='pausing', phase_at=now, pause_at=now, pause_ack_at=None,
                 reason='patrol_waiting_verified_stop')
        # FPGA STOP retains route/pose/task id. owner.stop is cancellation.
        a['pause_sequence'] = self.owner.send(wire.STOP, now=now)

    def pause_verified(self, now):
        a, h, p, c = self.active or {}, self.host, self.point, self.owner
        ack = a.get('pause_ack_at')
        return bool(a.get('phase') == 'paused' and a.get('patrol_fire') and ack is not None and
            c.protocol_ready and self._feedback_fresh(now) and
            self.host_request_at > ack and self.point_request_at > ack and
            c.status_request_at is not None and c.status_request_at > ack and
            h.get('task_id') == a.get('task_id') and h.get('nav_session') is True and
            h.get('selected') is True and h.get('scope') is True and h.get('link_ok') is True and
            h.get('run') is False and h.get('motion_status') == 2 and
            p.get('selected_point') == a.get('target_point') and p.get('state') == 4 and
            c.status.get('vehicle_stopped') is True)

    def resume_after_fire(self, now):
        if not self.pause_verified(now) or not off_disarmed(self.owner.actuator, now):
            raise ValueError('patrol_resume_requires_paused_and_pump_off')
        a = self.active
        a.update(phase='resuming', phase_at=now, resume_ack_at=None,
                 reason='patrol_resume_waiting_fpga')
        a['resume_sequence'] = self.owner.send(wire.RESUME, now=now)
        self._mission()

    def resume_write_ready(self, sequence, now):
        a, h, p, c = self.active or {}, self.host, self.point, self.owner
        return bool(a.get('phase') == 'resuming' and sequence == a.get('resume_sequence') and
            c.protocol_ready and self._feedback_fresh(now) and off_disarmed(c.actuator, now) and
            h.get('task_id') == a.get('task_id') and h.get('nav_session') and not h.get('run') and
            h.get('selected') and h.get('scope') and h.get('link_ok') and
            h.get('motion_status') == 2 and p.get('state') == 4 and
            p.get('selected_point') == a.get('target_point') and
            c.status.get('vehicle_stopped') is True and not c.mode_transition and not c.zebra_transition)

    def _advance_pause(self, now):
        a, h, p, c = self.active, self.host, self.point, self.owner
        if not self._feedback_fresh(now):
            return
        if self.host_request_at <= a['phase_at'] or self.point_request_at <= a['phase_at']:
            return
        if (h.get('task_id') != a['task_id'] or not h.get('nav_session') or
                not h.get('selected') or not h.get('scope') or not h.get('link_ok') or
                p.get('selected_point') != a['target_point'] or p.get('state') != 4):
            self.fail(now, 'patrol_paused_route_changed')
            return
        if h.get('motion_status') == 3 and h.get('run') is False:
            a.setdefault('terminal_seen_at', now)
            a['reason'] = 'terminal_status_waiting_matching_done_event'
            return
        if h.get('motion_status') != 2:
            self.fail(now, 'patrol_paused_route_changed')
            return
        if a['phase'] == 'pausing':
            ack = a.get('pause_ack_at')
            if (ack is not None and self.host_request_at > ack and self.point_request_at > ack and
                    c.status_request_at is not None and c.status_request_at > ack and
                    h.get('run') is False and c.status.get('vehicle_stopped') is True):
                a.update(phase='paused', reason='patrol_paused_route_verified')
        elif a['phase'] == 'paused':
            if h.get('run') or c.status.get('vehicle_stopped') is not True:
                self.fail(now, 'patrol_paused_vehicle_moved')
        elif a['phase'] == 'resuming':
            ack = a.get('resume_ack_at')
            if (ack is not None and self.host_request_at > ack and h.get('run') is True
                    and off_disarmed(c.actuator, now)):
                a['deadline'] += now-a['pause_at']
                a.update(phase='running', phase_at=now, reason='fpga_navigation_running')

    def on_reply(self, packet, sent_at, now):
        command = packet.command & 0x7f
        if command in (wire.HEARTBEAT, wire.HOST_STATUS):
            value = wire.host_status(packet.data)
            self.host, self.host_at, self.host_request_at = value, now, sent_at
            if value['run'] and not self.busy():
                self.owner.invalidate_start_references('observed_vehicle_movement')
        elif command == wire.POINT_STATUS:
            if packet.data[0] != 0:
                self.supported = False
                return
            self.point = wire.point_status(packet.data)
            self.point_at, self.point_request_at, self.supported = now, sent_at, True
        a = self.active
        if not a or a['phase'] in TERMINAL:
            self._observe_external(now)
            return
        if command == wire.STOP and packet.sequence == a.get('stop_sequence'):
            if packet.data[0] == 0:
                a['stop_ack_at'] = now
            return
        if ((command == wire.STOP and packet.sequence == a.get('pause_sequence') and a['phase'] == 'pausing') or
                (command == wire.RESUME and packet.sequence == a.get('resume_sequence') and a['phase'] == 'resuming')):
            if packet.data[0] != 0:
                self.fail(now, 'patrol_pause_resume_rejected_' + str(packet.data[0]))
            elif command == wire.STOP:
                a['pause_ack_at'] = now
            else:
                a['resume_ack_at'] = now
            return
        if packet.sequence == a.get('request_sequence') and command in (wire.SET_START, wire.SET_START_HEADING, wire.SELECT_POINT, wire.START):
            if packet.data[0] != 0:
                self.fail(now, 'fpga_numbered_command_rejected_' + str(packet.data[0]))
                return
            if command == wire.SET_START and a['phase'] == 'setting_start':
                a.update(phase='setting_heading', phase_at=now, reason='uploading_actual_start_heading')
                a['request_sequence'] = self.owner.send(wire.SET_START_HEADING,
                    wire.heading_payload(a['start_heading_deg']), now=now)
            elif command == wire.SET_START_HEADING and a['phase'] == 'setting_heading':
                expected = bytes((0,)) + wire.heading_payload(a['start_heading_deg'])[:2] + bytes((0,))
                if packet.data != expected:
                    self.fail(now, 'fpga_heading_ack_mismatch')
                    return
                a.update(phase='selecting_point', phase_at=now, reason='selecting_numbered_destination')
                a['request_sequence'] = self.owner.send(wire.SELECT_POINT,
                    wire.point_payload(a['target_point'], a['task_id']), now=now)
            elif command == wire.SELECT_POINT and a['phase'] == 'selecting_point':
                if packet.data != bytes((0, a['target_point'], 1, a['task_id'])):
                    self.fail(now, 'selected_point_ack_identity_mismatch')
                    return
                a.update(phase='planning', phase_at=now, reason='fpga_planning')
            elif command == wire.START and a['phase'] == 'starting':
                a.update(phase='running', phase_at=now, start_ack_at=now,
                         reason='start_accepted_waiting_actual_motion')
        self._advance(now)

    def _observe_external(self, now):
        """Observe a numbered host session already started by another producer.

        Pure physical-panel navigation has no host nav_session identity in the
        current FPGA protocol; it is reported as uncorrelated, never arrived.
        """
        c, h, p = self.owner, self.host, self.point
        if getattr(c, 'area', None) is not None and c.area.busy():
            return
        if (not c.real_uart or not c.protocol_ready or not self.supported
                or not recent(self.host_at, now) or not recent(self.point_at, now, 1.2)
                or not recent(c.status_at, now) or not h.get('run') or not h.get('nav_session')
                or not h.get('selected') or not h.get('scope') or not h.get('link_ok')
                or h.get('motion_status') != 2 or p.get('state') != 4
                or not 1 <= p.get('selected_point', 0) <= 9):
            if h.get('motion_status') == 2 and not h.get('nav_session'):
                self.last_observation_reason = 'physical_panel_motion_has_no_correlated_host_session'
            return
        if c.task == 'traffic' or c.mode_transition or c.zebra_transition:
            self.last_observation_reason = 'external_session_conflicts_with_active_task'
            return
        if c.active and c.active.get('phase') not in ('completed', 'fault'):
            return
        task_id = h['task_id']
        if task_id in self.observed_tasks or task_id in c.used_motion:
            self.last_observation_reason = 'external_task_identity_reused_not_adopted'
            return
        self.observed_tasks.add(task_id)
        c.used_motion.add(task_id)
        self.observation_count += 1
        profile = 'extension1' if c.task == 'vision' else 'extension2'
        self.active = dict(command_id=f'fpga-point:{self.session}:{self.observation_count}',
            action='navigate_point', origin='observed_fpga_host_session', start_point=None,
            target_point=p['selected_point'], task_id=task_id, task_profile=profile,
            arrival_policy='manual', phase='running',
            started_at=now, phase_at=now, deadline=now+180., seen_running=True,
            start_ack_at=None, done_at=None, arrival_stamp=None,
            reason='correlated_external_numbered_session', request_sequence=None)
        self.invalidate_reference('external_navigation_running')
        self._mission()

    def on_event(self, packet, now):
        a = self.active
        if not a or packet.command not in (wire.DONE, wire.FAILED, wire.LINK_TIMEOUT):
            return False
        if packet.sequence != 0 or packet.data[0] != a['task_id']:
            return False
        if a['phase'] in TERMINAL or a['phase'] == 'stopping':
            return True
        if packet.command != wire.DONE:
            self.fail(now, 'fpga_numbered_failure_' + str(packet.command) + '_' + str(packet.data[1]))
            return True
        if (packet.data[1:] != b'\x03\x00\x00' or a['phase'] not in ('running', 'pausing', 'paused', 'resuming')
                or not a['seen_running'] or not self._feedback_fresh(now)):
            return True
        a.update(phase='confirming_arrival', phase_at=now, done_at=now,
                 reason='done_received_waiting_queried_stop')
        self.patrol.arrival_pending(now)
        return True

    def _feedback_fresh(self, now):
        return bool(self.owner.real_uart and recent(self.owner.status_at, now)
                    and recent(self.host_at, now) and recent(self.point_at, now, 1.2))

    def _advance(self, now):
        a, h, p, c = self.active, self.host, self.point, self.owner
        if not a or a['phase'] in TERMINAL or a['phase'] == 'stopping':
            return
        phase = a['phase']
        if phase in ('pausing', 'paused', 'resuming'):
            self._advance_pause(now)
            return
        if phase == 'planning' and recent(self.point_at, now, 1.2) and self.point_request_at >= a['phase_at']:
            if p['selected_point'] != a['target_point']:
                self.fail(now, 'fpga_selected_destination_changed')
            elif p['state'] == 3:
                self.fail(now, 'fpga_numbered_plan_failed_' + str(p['error']))
            elif p['state'] == 2:
                if not recent(self.host_at, now) or self.host_request_at < a['phase_at']:
                    return
                if h['task_id'] != a['task_id'] or h['run'] or h['nav_session']:
                    self.fail(now, 'fpga_planning_session_identity_changed')
                    return
                a.update(phase='starting', phase_at=now, reason='route_ready_explicit_start')
                a['request_sequence'] = c.send(wire.START, bytes((1, a['task_id'], 0, 0)), now=now)
            elif p['state'] not in (1, 2):
                self.fail(now, 'fpga_numbered_route_invalidated')
        elif phase in ('running', 'confirming_arrival') and self._feedback_fresh(now):
            after_start = self.host_request_at >= a.get('start_ack_at', a['started_at']) if a.get('start_ack_at') is not None else self.host_request_at >= a['started_at']
            if not after_start:
                return
            if h['task_id'] != a['task_id'] or not h['nav_session'] or not h['selected'] or not h['scope']:
                self.fail(now, 'fpga_navigation_session_changed')
                return
            session_at = a.get('start_ack_at') if a.get('start_ack_at') is not None else a['started_at']
            if self.point_request_at < session_at:
                return
            if p['selected_point'] != a['target_point'] or p['state'] != 4:
                self.fail(now, 'fpga_numbered_destination_session_changed')
                return
            if h['motion_status'] >= 4:
                self.fail(now, 'fpga_navigation_motion_failed_' + str(h['motion_status']))
                return
            if phase == 'running':
                if h['run'] and h['motion_status'] == 2:
                    a['seen_running'] = True
                    a['reason'] = 'fpga_navigation_running'
                elif not h['run'] and h['motion_status'] != 3:
                    self.fail(now, 'fpga_navigation_stopped_without_arrival')
                elif h['motion_status'] == 3:
                    a.setdefault('terminal_seen_at', now)
                    a['reason'] = 'terminal_status_waiting_matching_done_event'
            elif (self.host_request_at > a['done_at'] and self.point_request_at > a['done_at']
                    and c.status_request_at is not None and c.status_request_at > a['done_at']
                    and h['motion_status'] == 3 and not h['run'] and c.status.get('vehicle_stopped')
                    and (not a.get('patrol_fire') or off_disarmed(c.actuator, now))):
                a.update(phase='arrived', arrival_stamp=now, reason='matching_done_and_measured_stop')
                self.reference = dict(confirmed=True, point_id=a['target_point'], source='confirmed_arrival',
                    stamp=now, command_id=a['command_id'], position_and_heading_confirmed=False)
                c.last_result = dict(command_id=a['command_id'], accepted=True, completed=True,
                                     reason=a['reason'], stamp=now)

    def tick(self, now):
        c = self.owner
        if c.real_uart and now-self.last_poll >= .35:
            c.send(wire.POINT_STATUS, now=now)
            self.last_poll = now
        if self.reference.get('confirmed') and (not c.real_uart or not recent(c.status_at, now)
                or not recent(self.host_at, now) or self.host.get('run') or not c.status.get('vehicle_stopped')):
            self.invalidate_reference('reference_lost_motion_or_feedback')
        if not self.busy():
            self._observe_external(now)
            return
        a = self.active
        if a['phase'] == 'stopping':
            stopped = (recent(c.status_at, now) and c.status_request_at is not None
                and c.status_request_at > a['stop_at'] and c.status.get('vehicle_stopped')
                and recent(self.host_at, now) and self.host_request_at > a['stop_at'] and not self.host.get('run'))
            if stopped:
                a['phase'] = 'cancelled' if a['reason'] in ('operator_stop', 'visual_task_selected', 'traffic_task_selected') else 'failed'
                c.last_result = dict(command_id=a['command_id'], accepted=True, completed=False,
                                     reason=a['reason'], stamp=now)
            elif a.get('stop_ack_at') is None and now-a.get('last_stop_at', a['stop_at']) >= .5:
                a['stop_sequence'] = c.send(wire.STOP, now=now)
                a['last_stop_at'] = now
            return
        if not self._feedback_fresh(now):
            self.fail(now, 'numbered_navigation_feedback_lost')
        elif now >= a['deadline'] and a['phase'] not in ('pausing', 'paused', 'resuming'):
            self.fail(now, 'numbered_navigation_timeout')
        elif a['phase'] in ('setting_start', 'setting_heading', 'selecting_point', 'starting') and now-a['phase_at'] > 2.:
            self.fail(now, 'numbered_navigation_ack_timeout')
        elif a['phase'] == 'planning' and now-a['phase_at'] > 60.:
            self.fail(now, 'numbered_navigation_planning_timeout')
        elif a['phase'] == 'confirming_arrival' and now-a['done_at'] > 2.:
            self.fail(now, 'numbered_arrival_stop_not_confirmed')
        elif a['phase'] in ('pausing', 'resuming') and now-a['phase_at'] > 3.:
            self.fail(now, 'patrol_pause_resume_timeout')
        elif (a['phase'] == 'resuming' and not off_disarmed(c.actuator, now) and
                (c.actuator.get('pump_on') is True or now-a['phase_at'] > .6)):
            self.fail(now, 'patrol_resume_pump_feedback_lost')
        elif a.get('terminal_seen_at') is not None and now-a['terminal_seen_at'] > 2.:
            self.fail(now, 'numbered_done_event_missing')
        else:
            self._advance(now)
        self.patrol.tick(now)

    def state(self, now):
        generic = self.owner.area.navigation.active or {}
        if (generic.get('motion_kind') == 'numbered_point' and
                generic.get('started_at', -math.inf) >= (self.active or {}).get('started_at', -math.inf)):
            return self.owner.area.navigation.state(now)
        a = self.active or {}
        live = self._feedback_fresh(now)
        phase = a.get('phase', 'idle')
        same = bool(a and self.host.get('task_id') == a.get('task_id')
                    and self.point.get('selected_point') == a.get('target_point'))
        arrived = bool(phase == 'arrived' and live and same and self.host.get('nav_session')
                       and self.host.get('motion_status') == 3 and not self.host.get('run')
                       and self.owner.status.get('vehicle_stopped'))
        moving = bool(phase == 'running' and live and same and self.host.get('nav_session')
                      and self.host.get('run') and self.host.get('motion_status') in (1, 2))
        return dict(stamp=now, owner='fpga', motion_kind='numbered_point',
            patrol_fire=a.get('patrol_fire', False), patrol=self.patrol.snapshot(now),
            command_id=a.get('command_id'), start_point=a.get('start_point'), target_point=a.get('target_point'),
            start_heading_deg=a.get('start_heading_deg'), heading_source=a.get('heading_source'),
            task_id=a.get('task_id'), task_profile=a.get('task_profile'), origin=a.get('origin'),
            arrival_policy=a.get('arrival_policy', 'manual'), arrival_stamp=a.get('arrival_stamp'),
            phase=phase, status=phase, valid=bool(arrived or moving), status_fresh=bool(live),
            arrived=arrived, ready_to_move=moving, vehicle_stopped=bool(live and self.owner.status.get('vehicle_stopped')),
            estop=False, failed=phase == 'failed', cancelled=phase == 'cancelled',
            reason=a.get('reason', self.last_observation_reason), selected_point=dict(self.point),
            observed_host=dict(self.host), physical_position_verified=False,
            completion_basis='same_task_done_then_queried_terminal_and_measured_stop',
            local_panel_limit='physical_panel_navigation_without_host_session_cannot_be_correlated')
