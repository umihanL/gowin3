"""Bounded LOW-readback recovery for search-only retained navigation.

Integration: call step before the normal actuator guard. While hold_prestart is
true, leave ACK parsing enabled, skip navigation.tick and block physical START,
RESUME and RETAINED_JOG writes. While hold_search is true, skip only the region
search FSM; ordinary navigation.tick must confirm the helper's STOP. Send one
OFF for request_off and prioritize read-only status queries for prioritize_readback.
Never ignore abort_reason: pass it to the normal core.stop/cancel path.

This helper does not invent LOW feedback, renew its TTL, restart a task, or
resume a pause created by an operator/fire FSM. A centering jog first stops;
only its stop/status/pose verification may continue with unknown pump output.
The original visual FSM stays held until both standstill and fresh LOW exist.
"""
from dataclasses import dataclass
import math

from .route_fire_patrol import navigation_off_disarmed


TRANSIENT = frozenset((
    'pump_output_pending_or_not_confirmed_off',
    'pump_off_fresh_actuator_feedback_required',
    'region_pump_off_and_disarmed_required',
))
PREPARING = frozenset((
    'resetting', 'reset_readback', 'setting_region_preference', 'setting_start',
    'setting_heading', 'setting_waypoint', 'setting_goal', 'plan_ack',
    'planning', 'continuous_route_check',
))
TERMINAL = frozenset(('stopping', 'cancelled', 'failed', 'arrived', 'held', 'idle'))


@dataclass(frozen=True)
class RecoveryDecision:
    handled: bool = False
    hold_prestart: bool = False
    hold_search: bool = False
    request_off: bool = False
    prioritize_readback: bool = False
    abort_reason: str = ''
    reason: str = ''
    resumed: bool = False


class SearchPumpRecovery:
    def __init__(self, timeout_s=3.0):
        if not math.isfinite(timeout_s) or not 0 < timeout_s <= 3.0:
            raise ValueError('search_pump_recovery_timeout_must_be_at_most_3s')
        self.timeout_s = timeout_s
        self.session = None

    @property
    def active(self):
        return self.session is not None

    def reset(self):
        s = self.session
        if s and s.get('centering') is not None:
            s['centering'].clear_pump_stop_recovery(s['token'])
        self.session = None

    def _abort(self, reason):
        self.reset()
        return RecoveryDecision(handled=True, abort_reason=reason, reason=reason)

    @staticmethod
    def _prestart_feedback(navigation, now):
        """Separate a measured movement from a temporarily missing stop readback."""
        owner = getattr(navigation, 'owner', None)
        numbered = getattr(owner, 'numbered', None)
        missing = []
        def recent(stamp, ttl):
            return (type(stamp) in (int, float) and math.isfinite(stamp)
                    and 0 <= now-stamp <= ttl)
        status_fresh = recent(getattr(owner, 'status_at', None), .8)
        host_fresh = recent(getattr(numbered, 'host_at', None), .8)
        status = getattr(owner, 'status', {}) or {}
        host = getattr(numbered, 'host', {}) or {}
        # Each positive movement report stands on its own. A stale POINT_STATUS
        # must neither erase fresh wheel motion nor masquerade as wheel motion.
        moving = ((status_fresh and status.get('vehicle_stopped') is False)
                  or (host_fresh and host.get('run') is True))
        if not getattr(owner, 'real_uart', False) or not getattr(owner, 'protocol_ready', False):
            missing.append('uart_protocol')
        if not status_fresh or status.get('vehicle_stopped') is not True:
            missing.append('vehicle_stop')
        if not host_fresh or host.get('run') is not False:
            missing.append('host_stop')
        if not recent(getattr(numbered, 'point_at', None), 1.2):
            missing.append('point_status')
        return moving, missing

    @staticmethod
    def _unsafe(actuator, physical, pending_on, mission):
        if pending_on or actuator.get('pump_requested_on') is True:
            return 'search_pump_recovery_on_requested'
        if actuator.get('pump_on') is True or physical.get('pump_on') is True:
            return 'search_pump_recovery_output_on'
        if actuator.get('manual_pump_active') is True:
            return 'search_pump_recovery_manual_pump_active'
        if (actuator.get('guard') or {}).get('armed') is not False:
            return 'search_pump_recovery_not_disarmed'
        if actuator.get('estop') or mission.get('emergency_stop') or mission.get('operator_hold'):
            return 'search_pump_recovery_operator_stop'
        if mission.get('traffic_active'):
            return 'search_pump_recovery_priority_changed'
        if physical.get('error'):
            return 'search_pump_recovery_bridge_error:' + str(physical['error'])
        if (physical.get('backend') != 'fpga_uart' or physical.get('pin') != 'B13'
                or actuator.get('pump_backend') != 'fpga_uart' or actuator.get('pump_pin') != 'B13'):
            return 'search_pump_recovery_requires_independent_b13_feedback'
        return ''

    def step(self, navigation, actuator, physical, reason, now, *, pending_on=False, mission=None):
        """Inspect existing snapshots; pause/resume only this helper's own leg.

        physical must be the sole UART owner's pump bridge state(now), not an
        HMI cache. pending_on includes queued/in-flight, noncancelled ON writes.
        A missing/stale value is never treated as an actual LOW.
        """
        a = navigation.active or {}
        s = self.session
        if s and (a is not s['leg'] or a.get('command_id') != s['command_id']
                  or a.get('task_id') != s['task_id']):
            self.reset()
            return RecoveryDecision(reason='search_pump_recovery_superseded')
        phase = a.get('phase')
        if a.get('execute_fire') is not False or a.get('continuous_route') is not True:
            if s:
                return self._abort('search_pump_recovery_task_mode_changed')
            return RecoveryDecision()
        if phase in TERMINAL:
            self.reset()
            if reason in TRANSIENT:
                # Navigation may have entered STOP on the preceding 20 ms
                # tick, before the 100 ms state publication. Its STOP already
                # owns LOW and standstill verification. Starting another
                # recovery here used to replace the original navigation fault
                # with "motion_requires_explicit_stop" and cancel the search.
                # Keep the region FSM held until that existing STOP completes;
                # never restart/resume a terminal leg or mask a definite ON.
                unsafe = self._unsafe(actuator, physical, pending_on, mission or {})
                if unsafe:
                    return self._abort(unsafe)
                return RecoveryDecision(handled=True, hold_search=phase == 'stopping',
                    prioritize_readback=phase == 'stopping',
                    reason='search_pump_recovery_existing_stop_preserved')
            return RecoveryDecision(reason='search_pump_recovery_stopped_externally')
        if not s and reason not in TRANSIENT:
            return RecoveryDecision()
        owner = getattr(navigation, 'owner', None)
        if owner is not None and (getattr(owner, 'mode_transition', None)
                or getattr(owner, 'zebra_transition', None)
                or getattr(owner, 'task', None) not in ('dialogue', 'semantic')):
            return self._abort('search_pump_recovery_task_mode_changed')
        unsafe = self._unsafe(actuator, physical, pending_on, mission or {})
        if unsafe:
            return self._abort(unsafe)
        if reason and reason not in TRANSIENT:
            return self._abort(reason)
        ready = bool(not reason and navigation_off_disarmed(actuator, now)
                     and physical.get('feedback_fresh') is True
                     and physical.get('pump_on') is False)
        first = s is None
        if first:
            tracked_unwritten_start = (phase == 'starting' and a.get('start_write_tracking') is True
                and a.get('start_written_sequence') != a.get('sequence'))
            stopped_speed_restore = (phase == 'restoring_speed' and
                a.get('after_speed_restore') in ('prepare_start', 'paused', 'arrived', 'held', 'cancelled', 'failed'))
            if phase in PREPARING or tracked_unwritten_start or stopped_speed_restore:
                kind = 'prestart'
            elif phase in ('running', 'resuming', 'setting_retained_alignment_speed'):
                kind = 'owned_pause'
            elif phase in ('pausing', 'paused'):
                kind = 'existing_pause'
            elif phase in ('centering_starting', 'centering_moving', 'centering_stopping', 'centering_verifying'):
                centering = getattr(navigation, 'centering', None)
                if not centering or not hasattr(centering, 'hold_pump_stop_recovery'):
                    return self._abort('search_pump_recovery_centering_stop_interface_missing')
                kind = 'centering_stop'
            else:
                # START may already be in flight. Do not fabricate its ACK.
                return self._abort('search_pump_recovery_motion_requires_explicit_stop:' + str(phase))
            s = self.session = dict(leg=a, command_id=a.get('command_id'),
                task_id=a.get('task_id'), generation=a.get('route_generation'),
                started=now, initial_reason=reason, kind=kind)
            if kind == 'owned_pause':
                try:
                    navigation.pause(s['command_id'], now)
                except ValueError as exc:
                    return self._abort(str(exc))
                s.update(pause_at=a.get('pause_at'), sequence=a.get('sequence'))
            elif kind == 'centering_stop':
                s.update(centering=centering, token=object())
                try:
                    centering.hold_pump_stop_recovery(s['token'], now, now + self.timeout_s)
                except ValueError as exc:
                    return self._abort(str(exc))
        if now < s['started'] or now - s['started'] >= self.timeout_s:
            return self._abort('search_pump_recovery_timeout:' + s['initial_reason'])
        low_stamp = physical.get('feedback_stamp')
        ready = bool(ready and type(low_stamp) in (int, float) and math.isfinite(low_stamp)
                     and s['started'] <= low_stamp <= now)
        generation = a.get('route_generation')
        if s['generation'] is None and s['kind'] == 'prestart':
            s['generation'] = generation
        elif generation != s['generation']:
            return self._abort('search_pump_recovery_route_changed')
        if s['kind'] == 'prestart':
            moving, missing = self._prestart_feedback(navigation, now)
            if moving:
                return self._abort('search_pump_recovery_preparation_motion_detected')
            s['waiting_feedback'] = missing
            if not navigation.stopped(now):
                # No motion command can pass the prestart writer hold. Retain
                # that hold within the original deadline while fresh stop/point
                # replies are reacquired; missing data is not measured motion.
                s['reason'] = 'search_pump_recovery_waiting_fresh_stop_and_low'
                return RecoveryDecision(handled=True, hold_prestart=True, hold_search=True,
                    request_off=first, prioritize_readback=True, reason=s['reason'])
            s['reason'] = 'search_pump_recovery_waiting_fresh_low'
        if s['kind'] == 'centering_stop':
            center = s['centering']
            if not center.pump_stop_recovery_matches(s['token'], now, include_paused=True):
                return self._abort('search_pump_recovery_centering_stop_ownership_lost')
            if (ready and a.get('phase') == 'paused' and navigation.stopped(now, s['started'])
                    and center.state(now).get('jog_complete') is True):
                self.reset()
                return RecoveryDecision(handled=True, reason='search_pump_recovery_centering_stopped_low_confirmed')
        elif s['kind'] == 'owned_pause':
            if (a.get('phase') not in ('pausing', 'paused')
                    or a.get('pause_at') != s['pause_at'] or a.get('sequence') != s['sequence']):
                return self._abort('search_pump_recovery_pause_ownership_lost')
            if ready and a.get('phase') == 'paused' and navigation.stopped(now, s['pause_at']):
                try:
                    navigation.resume(s['command_id'], now)
                except ValueError as exc:
                    return self._abort(str(exc))
                self.reset()
                return RecoveryDecision(handled=True, resumed=True,
                                        reason='search_pump_recovery_same_route_resumed')
        elif ready:
            if s['kind'] == 'existing_pause' and not (
                    a.get('phase') == 'paused' and navigation.stopped(now, s['started'])):
                return RecoveryDecision(handled=True, hold_search=True, prioritize_readback=True,
                    reason='search_pump_recovery_waiting_existing_measured_stop')
            if s['kind'] == 'prestart':
                # ACK parsing may advance a preparation phase during the hold.
                # Exclude only this bounded wait from phase/task deadlines.
                if 'phase_at' in a:
                    a['phase_at'] += now - max(s['started'], a['phase_at'])
                if 'deadline' in a:
                    a['deadline'] += now - s['started']
            self.reset()
            return RecoveryDecision(handled=True, reason='search_pump_recovery_low_confirmed')
        return RecoveryDecision(handled=True, hold_prestart=s['kind'] == 'prestart',
            hold_search=True, request_off=first, prioritize_readback=True,
            reason='search_pump_recovery_waiting_fresh_low')

    def snapshot(self, now):
        s = self.session
        return dict(active=bool(s), reason=s.get('reason', 'search_pump_recovery_waiting_fresh_low') if s else '',
                    elapsed_s=max(0., now-s['started']) if s else 0., timeout_s=self.timeout_s,
                    waiting_feedback=list(s.get('waiting_feedback', [])) if s else [],
                    command_id=s['command_id'] if s else None,
                    kind=s['kind'] if s else None)
