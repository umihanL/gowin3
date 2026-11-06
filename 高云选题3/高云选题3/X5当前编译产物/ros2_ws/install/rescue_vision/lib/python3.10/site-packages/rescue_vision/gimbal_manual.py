"""Finite production gimbal moves with measured completion, no UART ownership."""
from collections import deque
from .gimbal_self_test import GimbalSelfTest, finite, preflight_target, measured_target_matches


class GimbalManual:
    def __init__(self, servo):
        self.servo = servo
        self.guard = GimbalSelfTest(servo)
        self.phase, self.reason = 'idle', 'not_requested'
        self.target = None
        self.realizable_target = None
        self.deadline = 0.
        self.speed = 6.
        self.last_request_id = None
        self.last_result = 'none'
        self.last_request_reason = 'not_requested'
        self.seen = deque(maxlen=128)

    @property
    def active(self):
        return self.phase == 'running'

    def readiness(self, feedback, context, own=False):
        if context.get('independent_manual_adjustment') is True:
            if context.get('manual_vehicle_known_moving') is True:
                return False, 'manual_vehicle_known_moving'
            if context.get('manual_vehicle_policy_ready') is not True:
                return False, context.get('manual_vehicle_policy_reason') or 'manual_vehicle_policy_unavailable'
            # This local guard argument represents manual admission only;
            # mission telemetry and published measured-stop evidence remain
            # untouched. All angle, pump, owner and encoder checks still run.
            context = dict(context, vehicle_stopped=True)
        return self.guard.readiness(feedback, context, own=own)

    def cancel(self, reason, failed=False):
        self.phase = 'failed' if failed else 'cancelled'
        self.reason = self.last_request_reason = reason
        self.last_result = 'error' if failed else 'cancelled'
        self.target = None
        self.realizable_target = None

    def request(self, value, *, wall_now, now, feedback, context):
        self.last_request_id = value.get('request_id')
        if value.get('action') == 'stop':
            self.cancel('operator_stop')
            return
        try:
            if value.get('action') != 'start':
                raise ValueError('Expected manual gimbal start or stop')
            rid, stamp, ttl = (value.get(k) for k in ('request_id', 'issued_at', 'ttl_s'))
            if not isinstance(rid, str) or not 1 <= len(rid) <= 100 or rid in self.seen:
                raise ValueError('Unique request_id required')
            if not finite(stamp) or not finite(ttl) or not 0 < ttl <= 3 or not 0 <= wall_now-stamp <= ttl:
                raise ValueError('Fresh issued_at/ttl_s required')
            self.seen.append(rid)
            if self.active:
                raise ValueError('manual_gimbal_already_running')
            ready, reason = self.readiness(feedback, context)
            if not ready:
                raise ValueError(reason)
            current = feedback['actual_angles_deg']
            tilt_default = (self.servo['tilt']['reference_angle_deg']
                            if self.servo['tilt'].get('motion_enabled', True) is not True else current[1])
            # An omitted-axis hold target stays inside commanded travel even
            # if its unmodified measured reading jitters just across a limit.
            tilt_default = min(self.servo['tilt']['max_angle_deg'],
                               max(self.servo['tilt']['min_angle_deg'], tilt_default))
            target = [value.get('pan_deg'), value.get('tilt_deg', tilt_default)]
            realized = preflight_target(target, self.servo)
            self.target = target
            self.realizable_target = realized
            span = max(abs(a-b) for a, b in zip(target, current))
            budget = min(180., max(10., span * 1.5 + 10.))
            if self.servo['pan'].get('zdt_fd8_snapshot') is not None:
                from .zdt_fd8_control import motion_timeout_s
                budget = max(budget, motion_timeout_s(realized[0]-current[0]))
            self.deadline = now + budget
            self.phase, self.reason, self.last_result = 'running', 'waiting_measured_arrival', 'accepted'
            self.last_request_reason = self.reason
        except (ValueError, TypeError) as exc:
            self.last_result, self.last_request_reason = 'rejected', str(exc)
            raise

    def tick(self, now, feedback, context):
        if not self.active:
            return None
        ready, reason = self.readiness(feedback, context, own=True)
        if not ready:
            self.cancel(reason, failed=True)
            return None
        if now >= self.deadline:
            self.cancel('manual_gimbal_arrival_timeout', failed=True)
            return None
        tolerance = self.servo.get('position_tolerance_deg', .3)
        # FD8 reports the realizable lattice target separately; requested target
        # remains the bounded user intent and is checked against actual encoders.
        if measured_target_matches(feedback, self.target, tolerance):
            self.phase, self.reason, self.last_result = 'completed', 'measured_arrival', 'completed'
            self.last_request_reason = self.reason
            self.target = None
            self.realizable_target = None
            return None
        return tuple(self.target)

    def status(self, feedback, context):
        ready, reason = self.readiness(feedback, context)
        mechanical = [[self.servo[n]['min_angle_deg'], self.servo[n]['max_angle_deg']] for n in ('pan','tilt')]
        reachable, steps = [list(pair) for pair in mechanical], [None,None]
        snapshot = self.servo['pan'].get('zdt_fd8_snapshot')
        if snapshot is not None:
            from .zdt_fd8_control import reachable_range
            geometry = reachable_range(self.servo['pan'], snapshot)
            reachable[0], steps[0] = geometry['limits_deg'], geometry['step_deg']
            if reachable[0] is None:
                ready, reason = False, 'fd8_no_reachable_lattice_point'
        return dict(active=self.active, phase=self.phase, reason=self.reason,
                    can_start=ready and not self.active, ready_reason='manual_gimbal_running' if self.active else reason,
                    target_deg=self.target, actual_angles_deg=feedback.get('actual_angles_deg'),
                    realizable_target_deg=self.realizable_target,
                    last_request_id=self.last_request_id, last_result=self.last_result,
                    last_request_reason=self.last_request_reason,
                    independent_manual_adjustment=context.get('independent_manual_adjustment', False),
                    manual_vehicle_feedback_required=context.get('manual_vehicle_feedback_required', True),
                    manual_vehicle_feedback_known=context.get('manual_vehicle_feedback_known', False),
                    manual_vehicle_known_moving=context.get('manual_vehicle_known_moving', False),
                    axis_limits_deg=mechanical, axis_reachable_limits_deg=reachable,
                    axis_step_deg=steps,
                    motion_axes=[n for n in ('pan','tilt') if self.servo[n].get('motion_enabled',True) is True])
