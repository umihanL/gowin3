"""Bounded automatic observation/aim/one-burst round. No hardware I/O.

The operator chooses automatic operation explicitly (or as arrival policy).
Image evidence and the existing pump gate still decide whether a burst can run.
One round handles one target; persistent wallpaper flames never cause rearming.
"""
import math
import hashlib
from collections import OrderedDict

from .task_interaction import TaskRouter, fresh


class AutoFireRound:
    def __init__(self, timeout_s=90., scan_dwell_s=1., fixed_camera_observation=False):
        self.timeout = float(timeout_s)
        self.dwell = float(scan_dwell_s)
        if not 5 <= self.timeout <= 180 or not .5 <= self.dwell <= 5:
            raise ValueError('invalid_auto_fire_timing')
        if type(fixed_camera_observation) is not bool:
            raise ValueError('fixed_camera_observation_requires_boolean')
        self.fixed_camera_observation = fixed_camera_observation
        self.router = TaskRouter(pump_lease_s=3.)
        self.seen = OrderedDict()
        self.active = False
        self.scan_only = False
        self.scan_started = False
        self.observed_targets = []
        self.phase = 'IDLE'
        self.reason = 'not_requested'
        self.run_id = None
        self.pan_offset = 0.
        self.chosen = None
        self.preferred = None
        self.deadline = self.phase_at = self.scan_after = 0.
        self.scan_ready_since = None
        self.scan_index = 0
        self.scan_offsets = (0., -20., 20., -40., 40., 0.)
        self.saw_pump = False
        self.confirmed_pump_off_at = None
        self.return_to_standby = True
        self.initial_inspect_seen = False
        self.last_inspect_request_at = -math.inf
        self.arm_attempt = 0
        self.cancelled_through = -math.inf
        self.last_now = 0.

    def disarm(self, now, retire=False):
        return ('actuator', dict(action='disarm', command_id='auto-stop:'+str(self.run_id),
                                 issued_at=now, stamp=now, ttl_s=.5, auto_run_id=self.run_id,
                                 retire_auto_run=retire))

    def finish(self, now, reason, *, standby=True):
        self.active = False
        self.phase = 'WAIT_SELECTION' if self.scan_only else 'COMPLETE'
        self.reason = reason
        result = [self.disarm(now, retire=True)]
        if standby and self.return_to_standby and not self.scan_only:
            result.append(('mission', dict(mode='standby')))
        return result

    def request(self, command, now):
        if not isinstance(command, dict) or type(now) not in (int, float) or not math.isfinite(now):
            raise ValueError('invalid_auto_request')
        if command.get('action') == 'cancel':
            was_active = self.active
            self.active = False
            self.phase, self.reason = 'CANCELLED', 'operator_cancelled'
            self.cancelled_through = max(self.cancelled_through, now)
            # Idle cancels must not race with a separately submitted manual arm.
            return [self.disarm(now, retire=True)] if was_active else []
        cid = command.get('command_id')
        ttl = command.get('ttl_s')
        if (command.get('action') not in ('start', 'scan') or not isinstance(cid, str) or not 1 <= len(cid) <= 100
                or type(ttl) not in (int, float) or not 0 < ttl <= 5
                or not fresh(command, now, ttl, 'issued_at') or command['issued_at'] <= self.cancelled_through):
            raise ValueError('invalid_or_expired_auto_start')
        if cid in self.seen or self.active:
            raise ValueError('duplicate_or_busy_auto_round')
        return_to_standby = command.get('return_to_standby', True)
        if type(return_to_standby) is not bool:
            raise ValueError('return_to_standby_requires_boolean')
        preferred = None
        if 'preferred_target_id' in command or 'preferred_registry_session' in command:
            target_id, session = command.get('preferred_target_id'), command.get('preferred_registry_session')
            if (type(target_id) is not int or target_id <= 0 or
                    not isinstance(session, str) or not 1 <= len(session) <= 160):
                raise ValueError('invalid_required_fire_identity')
            preferred = (session, target_id)
        self.seen[cid] = now
        while len(self.seen) > 128:
            self.seen.popitem(last=False)
        self.run_id = cid
        self.scan_only = command.get('action') == 'scan'
        self.scan_started = False
        self.observed_targets = []
        self.last_now = now
        self.active, self.phase, self.reason = True, 'SEARCH', 'waiting_for_stopped_observation'
        self.deadline, self.phase_at, self.scan_after = now+self.timeout, now, now
        self.scan_ready_since = None
        self.scan_index, self.pan_offset = 0, 0.
        self.chosen, self.saw_pump = None, False
        self.confirmed_pump_off_at = None
        self.return_to_standby = return_to_standby
        self.initial_inspect_seen = False
        self.last_inspect_request_at = now
        self.preferred = preferred
        self.arm_attempt = 0
        return [self.disarm(now), ('mission', dict(mode='inspect', arrival_policy='scan' if self.scan_only else 'auto'))]

    def manual_selection(self, command, now):
        """A fresh explicit target choice releases scan ownership, never arms water."""
        if (not self.active or not self.scan_only or not isinstance(command, dict) or
                type(command.get('target_id')) is not int or command['target_id'] < 1 or
                not isinstance(command.get('command_id'), str) or
                not fresh(command, now, min(2., float(command.get('ttl_s', 0))), 'issued_at') or
                command.get('issued_at', 0) < self.phase_at):
            return []
        return self.finish(now, 'manual_target_selected', standby=False)

    def tick(self, now, *, mission=None, targets=None, selection=None, proposal=None, actuator=None):
        if not self.active:
            return []
        if type(now) not in (int,float) or not math.isfinite(now) or now < self.last_now:
            return self.finish(self.last_now, 'automatic_clock_invalid')
        self.last_now = now
        if now >= self.deadline:
            return self.finish(now, 'automatic_round_timeout')
        mission, targets, selection, proposal, actuator = (
            mission or {}, targets or {}, selection or {}, proposal or {}, actuator or {})
        if not fresh(mission, now, .5):
            if now-self.phase_at > 1.:
                return self.finish(now, 'mission_unavailable')
            return []
        if mission.get('operator_hold') or mission.get('traffic_active') or mission.get('emergency_stop'):
            return self.finish(now, 'higher_priority_task', standby=False)
        if mission.get('mode') == 'inspect':
            self.initial_inspect_seen = True
        if mission.get('mode') not in ('inspect', 'fire'):
            if (not self.return_to_standby and self.phase == 'SEARCH' and
                    not self.initial_inspect_seen and mission.get('mode') == 'navigate'):
                # The grant and this request travel on different ROS topics.
                # Reassert only the initial patrol observation transition;
                # Mission still independently checks the same run authority.
                if now-self.phase_at >= 1.:
                    return self.finish(now, 'initial_inspect_not_confirmed', standby=False)
                self.reason = 'waiting_for_initial_inspect_authority'
                if now-self.last_inspect_request_at >= .2:
                    self.last_inspect_request_at = now
                    return [('mission', dict(mode='inspect',
                        arrival_policy='scan' if self.scan_only else 'auto'))]
                return []
            if now-self.phase_at < .5 and self.phase=='SEARCH':
                return []
            return self.finish(now, 'mission_changed', standby=False)
        if self.scan_only:
            if mission.get('mode') == 'fire':
                return self.finish(now, 'manual_target_selected', standby=False)
            scan_gate = mission.get('scan_hardware_ready') is True and mission.get('vehicle_stopped') is True
            if not scan_gate:
                self.scan_ready_since = None
                self.reason = str(mission.get('scan_wait_reason') or 'scan_waiting_for_verified_hardware')
                if self.scan_started:
                    return self.finish(now, 'scan_interrupted:'+self.reason, standby=False)
                return []
            self.scan_started = True
        if self.phase == 'SEARCH':
            if (self.fixed_camera_observation and
                    mission.get('fixed_camera_observation') is not True):
                self.scan_ready_since = None
                self.reason = 'fixed_camera_observation_mode_not_confirmed'
                return []
            if (mission.get('vehicle_stopped') is not True or
                    (not self.fixed_camera_observation and mission.get('gimbal_ready') is not True)):
                self.scan_ready_since = None
                self.reason = ('waiting_for_real_stop' if self.fixed_camera_observation else
                               'waiting_for_real_stop_and_gimbal')
                return []
            # Mission learns the new requested scan offset on the next state
            # cycle. Old ready=True must not count as acknowledgement of it.
            if not self.fixed_camera_observation and now-self.scan_after < .2:
                self.scan_ready_since = None
                self.reason = 'waiting_for_scan_pose_ack'
                return []
            if self.scan_ready_since is None:
                self.scan_ready_since = now
                self.reason = 'waiting_for_post_settle_observation'
                return []
            candidates = []
            current_view = (fresh(targets, now, .5, 'capture_stamp') and targets.get('valid') is True
                    and targets.get('capture_stamp', 0) > self.scan_ready_since
                    and isinstance(targets.get('registry_session'), str) and bool(targets.get('registry_session'))
                    and isinstance(targets.get('targets'), list))
            if current_view:
                session = targets.get('registry_session')
                raw_targets = targets.get('targets', [])
                if isinstance(session, str) and session and isinstance(raw_targets, list):
                    for target in raw_targets:
                        if not isinstance(target, dict):
                            continue
                        box = target.get('bbox_xyxy')
                        if (target.get('selectable') is True and type(target.get('target_id')) is int
                                and target['target_id'] > 0 and isinstance(box, (list, tuple))
                                and len(box) == 4 and all(type(v) in (int, float) and math.isfinite(v) for v in box)
                                and box[0] < box[2] and box[1] < box[3]):
                            candidates.append(target)
            if self.preferred is not None:
                candidates = [target for target in candidates if
                              (targets.get('registry_session'), target['target_id']) == self.preferred]
                if not candidates:
                    # Give the stopped pose a short observation window. A
                    # different visible flame is never substituted for the one
                    # which caused the car to stop.
                    if now-self.scan_ready_since >= self.dwell:
                        return self.finish(now, 'required_fire_lost', standby=False)
                    self.reason = 'waiting_for_required_fire_after_stop'
                    return []
            if candidates:
                if self.scan_only:
                    self.observed_targets = [dict(registry_session=targets['registry_session'],
                        target_id=t['target_id']) for t in candidates]
                    return self.finish(now, 'fire_found_waiting_for_manual_selection', standby=False)+[
                        ('speech', '已发现可选火焰，扫描已停止。请说瞄准第几个火焰；喷水仍需另下明确指令。')]
                def area(t):
                    x1,y1,x2,y2 = t['bbox_xyxy']
                    return (x2-x1)*(y2-y1)
                target = max(candidates, key=area)
                self.chosen = (targets['registry_session'], target['target_id'])
                self.phase, self.phase_at = 'AIM', now
                self.reason = 'selected_required_fire' if self.preferred else 'selected_largest_visible_fire'
                return [('mission', dict(mode='fire')),
                    ('select', dict(command_id='auto-select:'+self.run_id, issued_at=now,
                        ttl_s=2., target_id=target['target_id']))]
            if now-self.scan_ready_since >= self.dwell:
                if self.fixed_camera_observation:
                    if not current_view:
                        self.reason = 'waiting_for_fixed_camera_observation'
                        return []
                    result = self.finish(now, 'no_selectable_fire_in_current_view')
                    return result+[('speech', '当前固定相机视场未发现可锁定火焰，已结束本轮观察。')]
                self.scan_index += 1
                if self.scan_index >= len(self.scan_offsets):
                    result = self.finish(now, 'no_selectable_fire_in_scan')
                    return result+[('speech', '本轮扫描未找到可锁定火焰，保持观察，等待指令。' if self.scan_only else
                                   '本轮扫描未找到可锁定火焰，已返回待机。' if self.return_to_standby else
                                   '本轮扫描未找到可锁定火焰，等待后续任务。')]
                self.pan_offset = self.scan_offsets[self.scan_index]
                self.scan_after = now
                self.scan_ready_since = None
                self.reason = 'scanning_next_pose'
            return []
        if self.phase in ('AIM','BURST'):
            selected = (selection.get('registry_session'), selection.get('target_id'))
            valid = (fresh(selection, now, .5, 'capture_stamp') and selection.get('valid') is True
                     and selected == self.chosen)
            if not valid:
                if self.phase == 'AIM' and now-self.phase_at < 1.:
                    return []
                return self.finish(now, 'selected_fire_lost')
        if self.phase == 'AIM':
            if not fresh(actuator, now, .5):
                self.reason = 'waiting_for_actuator_status'
                return []
            if actuator.get('enabled') is not True or actuator.get('pump_enabled') is not True:
                return self.finish(now, 'pump_hardware_not_enabled')
            if actuator.get('auto_authorized_run_id') != self.run_id:
                self.reason = 'waiting_for_actuator_auto_authority'
                return []
            # A region round's preceding STOP/OFF handshake is asynchronous.
            # Wait for its genuine fresh LOW-ready reply before consuming the
            # one arm attempt. Otherwise a healthy but pending OFF gets rejected
            # downstream while this state machine incorrectly enters BURST.
            if actuator.get('pump_backend') == 'fpga_uart' and (
                    actuator.get('pump_feedback_fresh') is not True or
                    actuator.get('pump_on') is not False or
                    actuator.get('pump_ready') is not True):
                self.reason = 'waiting_for_fpga_pump_ready'
                return []
            # Reuse the same target/calibration/readiness gate as manual speech.
            self.arm_attempt += 1
            arm_id = 'auto-arm:'+hashlib.sha256(self.run_id.encode()).hexdigest()[:24]+':'+str(self.arm_attempt)
            request = dict(command_id=arm_id, source='test', action='start_spray',
                           issued_at=now, ttl_s=.5)
            try:
                result = self.router.route(request, now, mission=mission, selection=selection,
                                           proposal=proposal, actuator=actuator)
            except ValueError as exc:
                self.reason = str(exc)
                return []
            if result.get('actuator'):
                self.phase, self.phase_at, self.reason = 'BURST', now, 'waiting_for_burst'
                return [('actuator', dict(result['actuator'], auto_run_id=self.run_id, max_bursts=1))]
        elif self.phase == 'BURST':
            if not fresh(actuator, now, .5):
                return self.finish(now, 'actuator_status_lost')
            # Unknown output after a UART generation change is not OFF. Keep
            # the bounded burst wait; only explicit fresh HIGH then LOW is a
            # completed output pulse, and still never extinguishment proof.
            pump_fresh = (actuator.get('pump_backend') != 'fpga_uart' or
                          actuator.get('pump_feedback_fresh') is True)
            if pump_fresh and actuator.get('pump_on') is True:
                self.saw_pump = True
            elif pump_fresh and actuator.get('pump_on') is False and self.saw_pump:
                self.confirmed_pump_off_at = now
                self.phase, self.phase_at, self.reason = 'REVIEW', now, 'burst_finished_not_extinguish_proof'
                return [self.disarm(now, retire=True), ('mission', dict(mode='inspect')),
                        ('speech', '本轮短喷指令结束，正在复查。不能仅凭喷水结束判定火已扑灭。')]
            if now-self.phase_at > 2.:
                return self.finish(now, 'burst_not_confirmed_or_too_long')
        elif self.phase == 'REVIEW' and now-self.phase_at >= 1.:
            # Get facts while inspection is still active, then return to standby.
            self.phase, self.phase_at = 'REPORT', now
            return [('question', dict(command_id='auto-report:'+self.run_id, stamp=now,
                                       ttl_s=3., text='报告当前场景'))]
        elif self.phase == 'REPORT' and now-self.phase_at >= .5:
            message = ('本轮任务结束，已返回待机。' if self.return_to_standby else
                       '本轮短喷与复查结束，等待后续任务。')
            return self.finish(now, 'one_target_round_finished')+[('speech', message)]
        return []

    def snapshot(self, now):
        return dict(stamp=now, active=self.active,
                    fixed_camera_observation=self.fixed_camera_observation,
                    phase='WAIT_READY' if self.scan_only and self.active and not self.scan_started else self.phase, reason=self.reason,
                    command_id=self.run_id, pan_offset_deg=self.pan_offset,
                    scan_only=self.scan_only, scan_started=self.scan_started,
                    observed_targets=list(self.observed_targets), pump_authorized=not self.scan_only and self.active,
                    scan_ready_since=self.scan_ready_since,
                    preferred_target_id=self.preferred[1] if self.preferred else None,
                    preferred_registry_session=self.preferred[0] if self.preferred else None,
                    selected_target_id=self.chosen[1] if self.chosen else None,
                    return_to_standby=self.return_to_standby,
                    saw_pump_on=self.saw_pump,
                    confirmed_pump_off_at=self.confirmed_pump_off_at,
                    burst_completed=bool(self.saw_pump and self.confirmed_pump_off_at is not None),
                    max_targets_per_round=1, extinguished_verified=False)
