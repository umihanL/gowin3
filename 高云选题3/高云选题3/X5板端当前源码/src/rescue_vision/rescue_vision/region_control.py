"""Integrate semantic search with the single FPGA UART owner, without I/O."""
import math
from .area_navigation import AreaNavigation, pose_reference
from .region_search import RegionSearch
from .route_fire_patrol import fresh, off_disarmed, navigation_off_disarmed


def pump_readiness_reason(actuator, capabilities):
    """Explain the actual B13 gate rather than calling every failure a wire fault."""
    if capabilities.get('pump_b13') is not True:
        return 'region_pump_b13_firmware_required'
    if actuator.get('pump_enabled') is not True:
        return 'region_pump_output_disabled'
    if actuator.get('pump_ready') is True:
        return ''
    if actuator.get('pump_feedback_fresh') is False:
        return 'region_pump_feedback_stale'
    reason = actuator.get('pump_reason')
    return {
        'fpga_pump_vehicle_moving': 'region_pump_vehicle_moving',
        'fpga_pump_scope_lost': 'region_pump_scope_required',
        'fpga_pump_link_lost': 'region_pump_link_required',
        'fpga_pump_feedback_unavailable': 'region_pump_feedback_stale',
        'fpga_pump_feedback_stale': 'region_pump_feedback_stale',
        'fpga_pump_awaiting_readback': 'region_pump_feedback_stale',
        'fpga_pump_stopped': 'region_pump_off_reset_required',
        'fpga_pump_pulse_complete': 'region_pump_off_reset_required',
        'fpga_pump_reset': 'region_pump_off_reset_required',
        'fpga_pump_mode_or_motion_changed': 'region_pump_off_reset_required',
    }.get(reason, 'region_pump_b13_not_ready')


class RegionControl:
    def __init__(self, owner):
        self.owner = owner
        self.navigation = AreaNavigation(owner)
        self.search = None
        self.approach = None
        self.targets = {}
        self.catalog = {}
        self.map_signature = None
        self.last_approach = {}
        self.consumed_panel_reference = None
        self.route_planner = None
        self.route_planning_error = None

    def load(self, catalog, map_signature, approach=None, alignment_config=None):
        self.search = RegionSearch(catalog)
        if alignment_config is not None:
            self.search.configure_checkpoint_alignment(alignment_config)
        self.catalog = self.search.catalog
        self.map_signature = map_signature
        self.approach = approach
        self.route_planner, self.route_planning_error = None, None
        if approach is not None and hasattr(approach, 'traversability'):
            from .region_route import RegionRoutePlanner
            try:
                self.route_planner = RegionRoutePlanner(approach.traversability, self.catalog, map_signature)
                self.search.route_planner = self.route_planner
            except (ValueError, TypeError, KeyError) as exc:
                self.route_planning_error = str(exc)

    def busy(self):
        return bool(self.search and self.search.active or self.navigation.busy())

    def reference(self, now):
        c, m = self.owner, self.navigation.map
        if not self.navigation.stopped(now):
            return dict(valid=False, reason='region_start_requires_measured_stop')
        ref = c.reference
        if ref.get('confirmed') is True:
            # Existing HMI reference uses centimetres, bottom-left world origin.
            x, y, h = (ref.get(k) for k in ('x_cm', 'y_cm', 'heading_deg'))
            if (all(type(v) in (int, float) and math.isfinite(v) for v in (x, y, h))
                    and 0 <= x < 352 and 0 < y <= 320):
                return dict(valid=True, cell=[int(x//2), int((320-y)//2)],
                            heading_deg=(h+180)%360-180, source='operator_hmi',
                            physical_position_verified=False)
        panel = m.get('start_reference') or {}
        generation = panel.get('generation')
        if (panel.get('valid') is True and fresh(panel, now, 1.2)
                and type(generation) is int and generation > 0
                and generation != self.consumed_panel_reference):
            return dict(panel, source='fpga_touchscreen', physical_position_verified=False)
        # A previous completed/paused leg can provide its current measured pose.
        if self.navigation.active and self.navigation.active['phase'] in ('arrived', 'paused', 'held', 'cancelled', 'failed'):
            current = self.navigation.current_reference(now)
            if current:
                return current
        numbered = c.numbered.active or {}
        done_at = numbered.get('done_at')
        host, pose = c.numbered.host, m.get('pose') or {}
        if (numbered.get('phase') == 'arrived' and type(done_at) in (float, int)
                and host.get('nav_session') is True and host.get('task_id') == numbered.get('task_id')
                and host.get('motion_status') == 3 and self.navigation.stopped(now, done_at)
                and pose.get('stamp', -math.inf) > done_at):
            current = pose_reference(pose, now, numbered['task_id'])
            if current:
                return current
        return dict(valid=False, reason='mark_start_and_heading_on_fpga_or_hmi')

    def readiness(self, now, execute_fire=True):
        c, n = self.owner, self.owner.numbered
        if not self.search:
            return 'semantic_region_catalog_unavailable'
        if self.route_planning_error:
            return self.route_planning_error
        if self.busy() or n.busy() or c.mode_transition or c.zebra_transition:
            return 'region_or_navigation_busy'
        if c.active and c.active['phase'] not in ('completed', 'fault'):
            return 'extension_movement_in_progress'
        if not c.real_uart or not c.protocol_ready or not self.navigation.feedback(now):
            return 'real_extension_uart_and_fresh_feedback_required'
        m = self.navigation.map
        if m.get('region_supported') is not True:
            return 'semantic_region_fpga_firmware_required'
        if self.route_planner and m.get('region_preference_supported') is not True:
            return 'semantic_region_route_preference_firmware_required'
        if self.route_planner and m.get('continuous_route_supported') is not True:
            return 'semantic_region_continuous_route_firmware_required'
        if self.route_planner and c.capabilities.get('retained_jog') is not True:
            return 'semantic_region_bidirectional_centering_firmware_required'
        if not execute_fire and self.route_planner and self.search.centering.side not in ('left', 'right'):
            return 'fire_centering_camera_direction_unset'
        if m.get('map_signature') != self.map_signature:
            return 'semantic_region_map_identity_mismatch'
        panel = m.get('start_reference') or {}
        if not fresh(panel, now, 1.2) or type(panel.get('generation')) is not int:
            return 'region_start_reference_query_required'
        if c.task not in ('dialogue', 'semantic'):
            return 'region_requires_extension2'
        if not n.host.get('selected') or not n.host.get('scope') or not n.host.get('link_ok'):
            return 'select_fpga_communication_navigation_page'
        if self.reference(now).get('valid') is not True:
            return self.reference(now)['reason']
        pump_check = off_disarmed if execute_fire else navigation_off_disarmed
        if not pump_check(c.actuator, now):
            return 'region_pump_off_and_disarmed_required'
        a, mission = c.actuator, c.mission_state
        if execute_fire and (a.get('enabled') is not True or a.get('dry_run') is not False
                or a.get('status_fresh') is not True or a.get('feedback_fresh') is not True
                or a.get('reference_confirmed_this_start') is not True):
            return 'region_real_gimbal_reference_required'
        if execute_fire:
            pump_reason = pump_readiness_reason(a, c.capabilities)
            if pump_reason:
                return pump_reason
        if (not fresh(mission, now) or mission.get('operator_hold')
                or mission.get('emergency_stop') or mission.get('traffic_active')):
            return 'region_priority_state_not_ready'
        if execute_fire and (mission.get('mounting_verified') is not True
                or mission.get('gimbal_transport') != 'x5'):
            return 'region_mount_or_priority_state_not_ready'
        # Search-only map motion does not depend on a camera frame or an idle
        # auto-fire heartbeat. Fresh physical LOW/disarmed evidence above still
        # applies, and an active auto-fire owner cannot be preempted here.
        if not execute_fire:
            if mission.get('auto_fire_active') is True or mission.get('pump_armed') is True:
                return 'region_auto_controller_not_idle'
            if c.auto_state.get('active') is True:
                previous = c.auto_state.get('stamp')
                newer_idle = (type(previous) in (int, float) and math.isfinite(previous)
                    and mission.get('auto_fire_active') is False and mission['stamp'] > previous
                    and a['stamp'] > previous)
                if fresh(c.auto_state, now) or not newer_idle:
                    return 'region_auto_controller_not_idle'
            return ''
        if not fresh(c.auto_state, now) or c.auto_state.get('active') is not False:
            return 'region_auto_controller_not_idle'
        if self.targets.get('frame_valid') is not True or not fresh(self.targets, now, .6, 'capture_stamp'):
            return 'region_live_yolo_required'
        if self.route_planner:
            # Retained-route batches stop the vehicle and aim only the nozzle.
            # The actual AutoFireRound/fixed-camera actuator gate validates
            # current stereo geometry and encoder alignment before any pulse.
            # World-map registration and wheel standoff reach are a different
            # operation and must not block this stationary nozzle workflow.
            return ''
        # Camera-to-nozzle geometry remains mandatory for extinguishing.
        if self.approach is None:
            return 'region_approach_calibration_required'
        reason = self.approach.readiness()
        if not reason and tuple(self.reference(now).get('cell') or []) not in getattr(self.approach, 'free', {tuple(self.reference(now).get('cell') or [])}):
            return 'region_start_not_in_clear_map_corridor'
        return reason or ''

    def observe_map(self, value, now):
        self.navigation.observe_map(value)
        if self.route_planner:
            self.route_planner.observe_map(value)
        if self.approach:
            self.approach.observe_map(value, now)

    def observe_targets(self, value, now):
        self.targets = value
        if self.approach:
            self.approach.observe_targets(value, now)

    def observe_depth(self, value, now):
        if self.approach:
            self.approach.observe_depth(value, now)

    def start(self, request, now):
        from .task_interaction import parse_task_text
        parsed = parse_task_text(request.get('text', ''))
        if (request.get('source') not in ('web', 'text', 'voice') or
                parsed.get('action') != 'search_region' or
                parsed.get('region_id') != request.get('region_id') or
                type(request.get('execute_fire')) is not bool or
                parsed.get('execute_fire') is not request.get('execute_fire')):
            raise ValueError('explicit_semantic_region_request_required')
        if request.get('source') == 'voice' and (request.get('voice_verification') or {}).get('verified') is not True:
            raise ValueError('region_search_independent_asr_required')
        reason = self.readiness(now, execute_fire=request['execute_fire'])
        if reason:
            raise ValueError(reason)
        ref = self.reference(now)
        if request['execute_fire']:
            self.approach.set_region(request['region_id'])
        self.last_approach = {}
        effects = self.search.start(dict(request, continuous_route=bool(self.route_planner)), now, ref)
        # Failed scheduling has not sent a navigation command and must not
        # consume the only operator start reference, so a corrected retry works.
        self.consumed_panel_reference = (self.navigation.map.get('start_reference') or {}).get('generation')
        self.owner.task = 'semantic'
        self._effects(effects, now)

    def _effects(self, effects, now):
        for key, value in effects:
            if key == 'navigate':
                try:
                    self.navigation.start(value, now)
                except (ValueError, KeyError, TypeError) as exc:
                    self._effects(self.search.cancel(now, 'region_leg_rejected:' + str(exc)), now)
                    self.search.phase = 'FAILED'
                    return
            elif key == 'pause':
                try:
                    self.navigation.pause(value['command_id'], now)
                except ValueError as exc:
                    self._effects(self.search.cancel(now, str(exc)), now)
                    self.search.phase = 'FAILED'
                    return
            elif key in ('advance_checkpoint', 'resume_retained', 'align_retained'):
                try:
                    if key == 'advance_checkpoint':
                        self.navigation.advance_checkpoint(now)
                    elif key == 'resume_retained':
                        self.navigation.resume(value['command_id'], now)
                    else:
                        self.navigation.begin_retained_alignment(value, now)
                except (ValueError, KeyError, TypeError) as exc:
                    self._effects(self.search.cancel(now, 'retained_route_rejected:' + str(exc)), now)
                    self.search.phase = 'FAILED'
                    return
            elif key in ('begin_centering', 'center_jog', 'center_stop', 'end_centering'):
                try:
                    if key == 'begin_centering':
                        self.navigation.begin_centering(now)
                    elif key == 'center_jog':
                        self.navigation.begin_centering_jog(value, now)
                    elif key == 'center_stop':
                        self.navigation.stop_centering_jog(now)
                    else:
                        self.navigation.end_centering(now)
                except (ValueError, KeyError, TypeError) as exc:
                    self._effects(self.search.cancel(now, 'fire_centering_rejected:' + str(exc)), now)
                    self.search.phase = 'FAILED'
                    return
            elif key == 'stop':
                self.navigation.stop(now, value.get('reason', 'region_task_stopped'))
                self.owner.effects.append(('mission', dict(mode='inspect')))
            else:
                self.owner.effects.append((key, value))

    def cancel(self, now, reason):
        if self.search and self.search.active:
            self._effects(self.search.cancel(now, reason), now)
            self.owner.last_result = dict(command_id=self.search.command_id, accepted=True,
                                         completed=False, reason=reason, stamp=now)
        elif self.navigation.busy():
            self.navigation.stop(now, reason)

    def tick(self, now):
        # Readback recovery leaves FPGA configuration ACKs intact. Only the
        # prestart transition is held; a running route's STOP still gets verified.
        if not getattr(self, 'pump_readback_hold_prestart', False):
            self.navigation.tick(now)
        leg = self.navigation.active or {}
        if (leg.get('motion_kind') == 'numbered_point' and
                leg.get('phase') in ('arrived', 'cancelled', 'failed') and not leg.get('result_reported')):
            leg['result_reported'] = True
            self.owner.last_result = dict(command_id=leg['command_id'], accepted=True,
                completed=leg['phase'] == 'arrived', reason=leg['reason'], stamp=now)
            self.owner.effects.append(('mission', dict(mode='inspect', command_id=leg['command_id'],
                arrival_policy='manual', task_profile=leg['task_profile'],
                motion_kind='numbered_point', target_point=leg['target_point'])))
        search = self.search
        if not search or not search.active:
            return
        c, m = self.owner, self.owner.mission_state
        if (fresh(m, now) and (m.get('operator_hold') or m.get('emergency_stop') or m.get('traffic_active'))):
            self.cancel(now, 'region_operator_or_priority_stop')
            return
        if getattr(self, 'pump_readback_hold_search', False):
            return
        preferred = search.target or search.candidate
        needs_world_approach=search.execute_fire and not search.continuous_route
        self.last_approach = self.approach.proposal(now, preferred) if self.approach and needs_world_approach else {}
        if (needs_world_approach and self.approach and search.phase == 'WAIT_SPRAY_RANGE' and search.target
                and not self.last_approach.get('valid')):
            alive = {(self.targets.get('registry_session'), t.get('target_id'))
                     for t in self.targets.get('targets', []) if isinstance(t, dict) and t.get('selectable') is True}
            if search.target not in alive and search.target_world is not None:
                self.last_approach = self.approach.reassociate(now, search.target, search.target_world)
        centering_proposal = None
        centering = search.centering
        if centering.active and self.navigation.active and self.navigation.active.get('phase') == 'paused':
            if search.phase in ('CENTER_RETURN', 'CENTER_REACQUIRE'):
                centering_proposal = self.navigation.centering_return_proposal(now)
            elif not search.execute_fire and search.phase == 'CENTER_SELECT' and centering.current:
                direction = centering.relation(now)
                row = centering.observation(now)
                if direction in ('forward', 'reverse') and row:
                    near = abs(row['center_x_norm']-.5) < .12
                    centering_proposal = self.navigation.centering_proposal(direction, now,
                        step_mm=20 if near else 40, speed_mm_s=20 if near else 40)
        alignment_proposal = None
        checkpoint = getattr(search, 'checkpoint', None)
        if (checkpoint and checkpoint.enabled and search.phase == 'ALIGN_SELECT'
                and checkpoint.current is not None):
            from .checkpoint_alignment_route import forward_road_proposal
            remaining = checkpoint.config['max_advance_cm'] - checkpoint.total_advance_cm
            if search.continuous_route:
                alignment_proposal = self.navigation.alignment_proposal(now,
                    min(checkpoint.config['leg_max_cm'], remaining))
            else:
                alignment_proposal = forward_road_proposal(self.route_planner, search.region_id,
                    self.navigation.current_reference(now), now,
                    min(checkpoint.config['leg_max_cm'], remaining))
        self._effects(search.tick(now, self.navigation.state(now), self.targets,
                                  c.auto_state, c.actuator, self.last_approach,
                                  alignment_proposal=alignment_proposal, centering_proposal=centering_proposal), now)
        if not search.active:
            c.last_result = dict(command_id=search.command_id, accepted=True,
                                 completed=search.phase == 'COMPLETE', reason=search.reason, stamp=now)

    def pump_allowed(self, now):
        c, n = self.owner, self.navigation
        return bool(self.search and self.search.pump_allowed(now) and n.stopped(now)
                    and n.active and n.active['phase'] in ('paused', 'arrived')
                    and not c.mode_transition and not c.zebra_transition
                    and c.task == 'semantic')

    def snapshot(self, now):
        result = self.search.snapshot(now) if self.search else dict(active=False, phase='UNAVAILABLE')
        result['pump_readback_recovery'] = dict(getattr(self, 'pump_readback_recovery', {}))
        result['approach'] = dict(self.last_approach)
        result['fire_dispatch_mode'] = ('stationary_gimbal_batch' if self.route_planner else 'mapped_wheel_approach')
        result['world_approach_required'] = self.route_planner is None
        result['navigation'] = self.navigation.state(now)
        return result
