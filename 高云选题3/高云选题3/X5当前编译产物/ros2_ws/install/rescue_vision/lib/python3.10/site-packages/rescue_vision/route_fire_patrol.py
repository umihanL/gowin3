"""Explicit along-route fire rounds; FPGA retains the original route/odometry.

The three-second cooldown suppresses fire-stop triggers, never image inference
or operator stops. A successful round proves a bounded output pulse, not that
a printed flame or physical fire has disappeared.
"""
import hashlib
import math


def fresh(value, now, ttl=.6, key='stamp'):
    stamp = value.get(key) if isinstance(value, dict) else None
    return type(stamp) in (int, float) and math.isfinite(stamp) and 0 <= now-stamp <= ttl


def off_disarmed(value, now):
    return bool(fresh(value, now) and value.get('pump_on') is False and
                value.get('pump_feedback_fresh') is True and
                (value.get('guard') or {}).get('armed') is False and
                not value.get('estop') and not value.get('error'))


def navigation_off_disarmed(value, now):
    """Vehicle-only search may use independent B13 feedback during an axis fault.

    A gimbal reference/encoder fault cannot erase the separately read FPGA
    output level. Automatic aiming and firefighting keep off_disarmed().
    Unknown/local GPIO outputs and an armed or requested pump still block.
    """
    if value.get('manual_pump_active') is True or value.get('pump_requested_on') is True:
        return False
    if off_disarmed(value, now):
        return True
    return bool(value.get('pump_backend') == 'fpga_uart' and value.get('pump_pin') == 'B13'
                and fresh(value, now) and value.get('pump_feedback_fresh') is True
                and value.get('pump_on') is False and not value.get('estop')
                and (value.get('guard') or {}).get('armed') is False)


class RouteFirePatrol:
    def __init__(self, navigation):
        self.nav = navigation
        self.enabled = False
        self.phase, self.reason = 'DISABLED', 'not_requested'
        self.last_capture = None
        self.candidate = None
        self.hits = 0
        self.target = None
        self.run_id = None
        self.rounds = 0
        self.phase_at = self.ignore_until = 0.

    @property
    def core(self):
        return self.nav.owner

    def readiness(self, now):
        c, a, m = self.core, self.core.actuator, self.core.mission_state
        if c.task != 'dialogue':
            return 'patrol_requires_extension2'
        if not fresh(a, now) or a.get('enabled') is not True or a.get('dry_run') is not False:
            return 'patrol_real_actuator_required'
        if (a.get('status_fresh') is not True or a.get('feedback_fresh') is not True or
                a.get('reference_confirmed_this_start') is not True):
            return 'patrol_encoder_reference_required'
        if (a.get('pump_enabled') is not True or a.get('pump_ready') is not True or
                not off_disarmed(a, now) or c.capabilities.get('pump_b13') is not True):
            return 'patrol_pump_not_ready'
        if (not fresh(m, now) or m.get('mounting_verified') is not True or
                m.get('gimbal_transport') != 'x5' or m.get('operator_hold') or m.get('emergency_stop')):
            return 'patrol_mount_verification_required'
        if self.last_capture is None or not 0 <= now-self.last_capture <= .6:
            return 'patrol_yolo_feedback_required'
        if not fresh(c.auto_state, now) or c.auto_state.get('active') is not False:
            return 'patrol_auto_round_busy'
        return ''

    def validate_start(self, request, now):
        if request.get('patrol_fire') is not True:
            if 'patrol_fire' in request or 'fire_ignore_s' in request:
                raise ValueError('invalid_patrol_authority')
            return
        from .task_interaction import parse_task_text
        parsed = parse_task_text(request.get('text', ''))
        if (request.get('source') not in ('web', 'voice', 'text') or
                parsed.get('action') != 'navigate_point' or parsed.get('patrol_fire') is not True or
                type(request.get('fire_ignore_s')) not in (int, float) or request['fire_ignore_s'] != 3. or
                any(parsed.get(k) != request.get(k) for k in
                    ('start_point', 'target_point', 'start_heading_deg', 'patrol_fire', 'fire_ignore_s'))):
            raise ValueError('explicit_patrol_fire_statement_required')
        reason = self.readiness(now)
        if reason:
            raise ValueError(reason)

    def start(self, enabled, now):
        self.enabled = enabled
        self.phase = 'WATCHING' if enabled else 'DISABLED'
        self.reason = 'patrol_watching_for_fire' if enabled else 'not_requested'
        self.candidate = self.target = self.run_id = None
        self.hits = self.rounds = 0
        self.phase_at, self.ignore_until = now, 0.

    def observe(self, document, now):
        stamp = document.get('capture_stamp')
        # targets.valid=False can be a healthy, empty scene. frame_valid is
        # emitted separately by the registry from actual detector input health.
        session, size = document.get('registry_session'), document.get('image_size_wh')
        if (document.get('frame_valid') is not True or not fresh(document, now, .5, 'capture_stamp') or
                not isinstance(session, str) or not session or not isinstance(size, (list, tuple)) or
                len(size) != 2 or any(type(x) is not int or x <= 0 for x in size) or
                not isinstance(document.get('targets'), list)):
            self.hits = 0
            return
        if self.last_capture is not None and stamp <= self.last_capture:
            return
        gap = self.last_capture is None or stamp-self.last_capture > .3
        self.last_capture = stamp
        if (not self.enabled or self.phase != 'WATCHING' or now < self.ignore_until or stamp < self.ignore_until or
                not self.nav.state(now).get('ready_to_move')):
            self.hits = 0
            return
        candidates = []
        for target in document.get('targets', []):
            if not isinstance(target, dict):
                continue
            box = target.get('bbox_xyxy')
            if (target.get('selectable') is not True or type(target.get('target_id')) is not int or
                    target['target_id'] <= 0 or not isinstance(box, (list, tuple)) or len(box) != 4 or
                    any(type(x) not in (int, float) or not math.isfinite(x) for x in box)):
                continue
            x1, y1, x2, y2 = box
            if 0 <= x1 < x2 <= size[0] and 0 <= y1 < y2 <= size[1]:
                candidates.append(((session, target['target_id']), (x2-x1)*(y2-y1)))
        candidate = next((v for v in candidates if v[0] == self.candidate), None)
        if candidate is None:
            candidate = max(candidates, key=lambda v: v[1], default=None)
        if candidate is None:
            self.candidate, self.hits = None, 0
            return
        self.hits = self.hits+1 if self.candidate == candidate[0] and not gap else 1
        self.candidate = candidate[0]
        if self.hits >= 3:
            self.target = self.candidate
            self.phase, self.reason, self.phase_at = 'WAIT_STOP', 'patrol_waiting_verified_stop', now
            self.nav.pause_for_fire(now)

    def pump_allowed(self, now):
        c = self.core
        return bool(self.enabled and self.phase == 'EXTINGUISHING' and self.nav.pause_verified(now)
            and not c.mode_transition and not c.zebra_transition and c.task == 'dialogue'
            and not (c.active and c.active.get('phase') not in ('completed', 'fault')))

    def _stop_auto(self, now):
        if self.run_id:
            self.core.effects.append(('auto', dict(action='cancel', command_id='cancel:'+self.run_id,
                                                   issued_at=now, ttl_s=.5)))
        self.core.effects.append(('actuator', dict(action='disarm', issued_at=now, ttl_s=.5)))

    def cancel(self, now, reason):
        if not self.enabled:
            return
        self._stop_auto(now)
        self.enabled = False
        self.phase = 'CANCELLED' if reason in ('operator_stop', 'visual_task_selected', 'traffic_task_selected') else 'FAILED'
        self.reason = reason

    def arrival_pending(self, now):
        if self.enabled:
            self._stop_auto(now)
            self.phase, self.reason = 'ARRIVING', 'destination_waiting_stop_confirmation'

    def tick(self, now):
        if not self.enabled:
            return
        n, c, a = self.nav, self.core, self.nav.active
        if not a or a['phase'] in ('stopping', 'failed', 'cancelled'):
            self.cancel(now, (a or {}).get('reason', 'navigation_missing'))
            return
        if a['phase'] == 'arrived':
            self.enabled = False
            self.phase, self.reason = 'COMPLETE', 'destination_reached'
            return
        actuator, auto, mission = c.actuator, c.auto_state, c.mission_state
        if fresh(mission, now) and (mission.get('operator_hold') or mission.get('emergency_stop') or mission.get('traffic_active')):
            n.fail(now, 'patrol_operator_or_priority_stop')
            return
        if self.phase in ('WATCHING', 'COOLDOWN'):
            if a['phase'] == 'running' and (self.last_capture is None or now-self.last_capture > 1.):
                n.fail(now, 'patrol_yolo_feedback_lost')
            elif self.phase == 'COOLDOWN' and now >= self.ignore_until:
                self.phase, self.reason = 'WATCHING', 'patrol_watching_for_fire'
                self.hits = 0
            return
        if self.phase == 'WAIT_STOP' and now-self.phase_at > 4.:
            n.fail(now, 'patrol_stop_not_confirmed')
            return
        if self.phase == 'WAIT_STOP' and n.pause_verified(now):
            if not off_disarmed(actuator, now):
                self.reason = 'patrol_waiting_verified_pump_off'
                return
            seed = a['command_id'] + ':' + str(self.rounds+1)
            self.run_id = 'route-fire:' + hashlib.sha256(seed.encode()).hexdigest()[:32]
            self.phase, self.reason, self.phase_at = 'EXTINGUISHING', 'patrol_automatic_fire_round', now
            c.effects.append(('auto', dict(action='start', command_id=self.run_id,
                issued_at=now, ttl_s=2., preferred_registry_session=self.target[0],
                preferred_target_id=self.target[1], return_to_standby=False)))
        elif self.phase == 'EXTINGUISHING':
            matching = fresh(auto, now) and auto.get('command_id') == self.run_id
            if not matching:
                if now-self.phase_at > 2.:
                    n.fail(now, 'patrol_auto_state_missing')
                return
            if auto.get('active') is False:
                off_at = auto.get('confirmed_pump_off_at')
                success = (auto.get('phase') == 'COMPLETE' and auto.get('reason') == 'one_target_round_finished'
                    and auto.get('burst_completed') is True and type(off_at) in (int, float)
                    and self.phase_at <= off_at <= now)
                if not success:
                    n.fail(now, 'patrol_auto_failed:' + str(auto.get('reason', 'unknown')))
                    return
                self.rounds += 1
                self._stop_auto(now)  # retire this run's Mission/actuator authority
                self.phase, self.reason, self.phase_at = 'WAIT_PUMP_OFF', 'patrol_waiting_verified_pump_off', now
            elif now-self.phase_at > 92.:
                n.fail(now, 'patrol_auto_round_timeout')
        elif self.phase == 'WAIT_PUMP_OFF':
            if (off_disarmed(actuator, now) and actuator['stamp'] > self.phase_at and
                    fresh(auto, now) and auto.get('active') is False and n.pause_verified(now)):
                self.phase, self.reason, self.phase_at = 'RESUMING', 'patrol_resume_waiting_fpga', now
                n.resume_after_fire(now)
            elif now-self.phase_at > 3.:
                n.fail(now, 'patrol_pump_off_not_confirmed')
        elif self.phase == 'RESUMING' and a['phase'] == 'running' and n.state(now)['ready_to_move']:
            self.ignore_until = now+3.
            self.phase, self.reason, self.phase_at = 'COOLDOWN', 'patrol_fire_cooldown', now
            self.candidate, self.hits = None, 0

    def snapshot(self, now):
        return dict(phase=self.phase, reason=self.reason, rounds_completed=self.rounds,
            fire_ignore_remaining_s=max(0., self.ignore_until-now) if self.phase == 'COOLDOWN' else 0.,
            target_id=self.target[1] if self.target else None,
            registry_session=self.target[0] if self.target else None, auto_run_id=self.run_id,
            extinguished_verified=False)
