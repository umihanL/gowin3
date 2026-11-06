"""Finite, encoder-verified +/-/return test; no hardware or ROS imports.

This controller owns neither calibration nor homing. It requires production
commissioning and fresh stopped-vehicle evidence. Completion means all three
poses were measured, not that a UART accepted three commands.
"""
from collections import deque
import math
from .gimbal_limits import feedback_limit_tolerance, measured_in_limits


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def feedback_target_matches(feedback, target, tolerance):
    """Associate the original command with its realizable driver target.

    A nearby but unrelated old goal must never satisfy this check. Quantization
    changes the physical command, not the original request identity.
    """
    requested = feedback.get('requested_target_deg', feedback.get('target_deg'))
    realized = feedback.get('target_deg')
    if not finite(tolerance) or tolerance < 0:
        return False
    for pair in (target, requested, realized):
        if not isinstance(pair, (list, tuple)) or len(pair) != 2 or not all(finite(v) for v in pair):
            return False
    return list(requested) == list(target) and all(abs(a-b) <= tolerance for a,b in zip(realized,target))


def measured_target_matches(feedback, target, tolerance):
    actual = feedback.get('actual_angles_deg')
    return bool(feedback_target_matches(feedback, target, tolerance)
        and feedback.get('feedback_fresh') is True and not feedback.get('error')
        and feedback.get('settled_measured') is True
        and isinstance(actual, (list, tuple)) and len(actual) == 2 and all(finite(v) for v in actual)
        and all(abs(a-b) <= tolerance for a,b in zip(actual,target)))


def preflight_target(target, servo):
    """Pure validation/quantization preview; no I/O or target state mutation."""
    realized = list(target)
    for i, name in enumerate(('pan', 'tilt')):
        axis, angle = servo[name], target[i]
        if not finite(angle) or not axis['min_angle_deg'] <= angle <= axis['max_angle_deg']:
            raise ValueError(name+'_angle_outside_measured_limits')
        if axis.get('motion_enabled', True) is not True and abs(angle-axis['reference_angle_deg']) > .01:
            raise ValueError(name+'_direction_not_verified')
    if servo['pan'].get('zdt_fd8_snapshot') is not None:
        from .zdt_fd8_control import quantized_target
        realized[0] = quantized_target(target[0], servo['pan'], servo['pan']['zdt_fd8_snapshot'],
                                       servo['position_tolerance_deg'])
    return realized


class GimbalSelfTest:
    def __init__(self, servo):
        self.servo = servo
        self.phase = 'idle'
        self.reason = 'not_started'
        self.request_id = None
        self.axes = None
        self.points = []
        self.index = 0
        self.target = None
        self.started = self.step_started = 0.
        self.leg_timeouts = [5., 5., 5.]
        self.total_timeout = 15.
        self.speed = 6.
        self.seen = deque(maxlen=128)
        self.hardware_verified = False
        self.samples = []
        self.last_request_id = None
        self.last_result = 'none'
        self.last_request_reason = 'not_started'
        self.last_request_stamp = None
        self.wall_offset = 0.

    @property
    def active(self):
        return self.phase == 'running'

    def readiness(self, feedback, context, *, own=False):
        for ok, reason in (
            (context.get('enabled') is True and context.get('dry_run') is False, 'real_actuator_not_enabled'),
            (not context.get('estop') and not context.get('fault'), 'actuator_fault_or_estop'),
            (context.get('mission_fresh') is True, 'mission_status_stale'),
            (context.get('vehicle_stopped') is True, 'measured_vehicle_stop_required'),
            (context.get('idle_mode') is True, 'set_idle_before_self_test'),
            (not context.get('pump_on') and not context.get('pump_armed'), 'pump_must_be_disarmed'),
            (not context.get('active_owner') or own, 'another_gimbal_owner_active'),
            (feedback.get('feedback_fresh') is True, 'encoder_feedback_stale'),
            (not feedback.get('error'), 'encoder_or_driver_fault'),
            (self.servo.get('reference_confirmed_this_start') is True, 'reference_this_start_unconfirmed'),
            (self.servo.get('calibration_confirmed') is True, 'installation_calibration_unconfirmed'),
            (self.servo.get('load_supported_verified') is True, 'supported_load_unconfirmed'),
            (self.servo.get('enable_on_command') is True, 'enable_on_command_not_configured'),
        ):
            if not ok:
                return False, reason
        angles = feedback.get('actual_angles_deg')
        if not isinstance(angles, (list, tuple)) or len(angles) != 2 or not all(finite(v) for v in angles):
            return False, 'two_measured_angles_required'
        speeds = feedback.get('motor_rpm')
        if not own and (not isinstance(speeds, (list, tuple)) or len(speeds) != 2 or
                        not all(finite(v) and v == 0 for v in speeds)):
            return False, 'both_axes_must_be_measured_stationary'
        for value, name in zip(angles, ('pan', 'tilt')):
            axis = self.servo.get(name, {})
            lo, hi = axis.get('min_angle_deg'), axis.get('max_angle_deg')
            if (not finite(lo) or not finite(hi) or
                    not measured_in_limits(value, axis, feedback_limit_tolerance(self.servo)) or
                    axis.get('reference_counts') is None):
                return False, 'measured_limits_or_reference_missing'
        return True, 'ready'

    def request(self, document, *, wall_now, now, feedback, context):
        self.last_request_id = document.get('request_id')
        self.last_request_stamp = wall_now
        self.wall_offset = wall_now-now
        try:
            self._request(document, wall_now=wall_now, now=now, feedback=feedback, context=context)
        except (ValueError, TypeError) as exc:
            self.last_result, self.last_request_reason = 'rejected', str(exc)
            raise
        self.last_result, self.last_request_reason = 'accepted', self.reason

    def _request(self, document, *, wall_now, now, feedback, context):
        action = document.get('action')
        if action == 'stop':
            self.cancel('operator_stop')
            return
        if action != 'start':
            raise ValueError('Expected self-test start or stop')
        rid, stamp, ttl = document.get('request_id'), document.get('issued_at'), document.get('ttl_s')
        if not isinstance(rid, str) or not 1 <= len(rid) <= 100 or rid in self.seen:
            raise ValueError('Unique request_id required')
        if not finite(stamp) or not finite(ttl) or not 0 < ttl <= 5 or not 0 <= wall_now-stamp <= ttl:
            raise ValueError('Fresh issued_at/ttl_s required')
        self.seen.append(rid)
        if self.active:
            raise ValueError('Self-test already owns gimbal')
        axes = document.get('axes', 'both')
        amplitude, speed = document.get('amplitude_deg', 1.), document.get('speed_deg_s', 6.)
        if axes not in ('pan', 'tilt', 'both') or not finite(amplitude) or not 0 < amplitude <= 1:
            raise ValueError('Use pan/tilt/both and amplitude in (0,1] degree')
        if not finite(speed) or not 0 < speed <= 6:
            raise ValueError('Self-test speed must be in (0,6] degrees/s')
        if amplitude <= 2*min(.3, self.servo.get('position_tolerance_deg', .3)):
            raise ValueError('Test amplitude must exceed twice measured arrival tolerance')
        ok, reason = self.readiness(feedback, context)
        if not ok:
            raise ValueError(reason)
        centre = list(feedback['actual_angles_deg'])
        mask = (axes in ('pan', 'both'), axes in ('tilt', 'both'))
        for i, name in enumerate(('pan', 'tilt')):
            if not mask[i]:
                axis = self.servo[name]
                centre[i] = min(axis['max_angle_deg'], max(axis['min_angle_deg'], centre[i]))
            if self.servo[name].get('motion_enabled', True) is not True:
                if mask[i]:
                    raise ValueError(name+'_direction_not_verified')
                centre[i] = self.servo[name]['reference_angle_deg']
        points = [[v+sign*amplitude*mask[i] for i, v in enumerate(centre)] for sign in (1, -1, 0)]
        for point in points:
            for value, name in zip(point, ('pan', 'tilt')):
                axis = self.servo[name]
                if not axis['min_angle_deg'] <= value <= axis['max_angle_deg']:
                    raise ValueError('Self-test sweep would exceed measured limits')
            realized = preflight_target(point, self.servo)
            if max(abs(a-b) for a,b in zip(realized,point)) > min(.3, self.servo.get('position_tolerance_deg',.3)):
                raise ValueError('Self-test target quantization exceeds measured test tolerance')
        self.phase, self.reason, self.request_id, self.axes = 'running', 'outbound', rid, axes
        self.points, self.index, self.target, self.speed = points, 0, points[0], speed
        self.leg_timeouts = [5., 5., 5.]
        if self.servo['pan'].get('zdt_fd8_snapshot') is not None:
            from .zdt_fd8_control import motion_timeout_s
            previous = centre[0]
            for i, point in enumerate(points):
                realized = preflight_target(point, self.servo)
                self.leg_timeouts[i] = motion_timeout_s(realized[0]-previous, overhead_s=5.)
                previous = realized[0]
        self.total_timeout = sum(self.leg_timeouts)
        self.started = self.step_started = now
        self.hardware_verified, self.samples = False, []

    def cancel(self, reason, failed=False):
        self.phase = 'failed' if failed else 'cancelled'
        self.reason, self.target, self.hardware_verified = reason, None, False
        if failed:
            self.last_result, self.last_request_reason = 'error', reason

    def tick(self, now, feedback, context):
        if not self.active:
            return None
        ready, reason = self.readiness(feedback, context, own=True)
        if not ready:
            self.cancel(reason, failed=True)
            return None
        # Each leg has a bounded deadline; repeated requests cannot prolong it.
        if now-self.step_started > self.leg_timeouts[self.index] or now-self.started > self.total_timeout:
            self.cancel('measured_arrival_timeout', failed=True)
            return None
        angles = feedback['actual_angles_deg']
        tolerance = min(.3, self.servo.get('position_tolerance_deg', .3))
        if measured_target_matches(feedback, self.target, tolerance):
            self.samples.append(dict(target_deg=list(self.target), actual_angles_deg=list(angles),
                                     error_deg=[a-b for a, b in zip(angles, self.target)]))
            self.index += 1
            if self.index == len(self.points):
                self.phase, self.reason, self.hardware_verified = 'completed', 'three_poses_measured', True
                self.last_request_id, self.last_result = self.request_id, 'completed'
                self.last_request_reason, self.last_request_stamp = self.reason, now+self.wall_offset
                self.target = None
                return None
            self.target = self.points[self.index]
            self.reason = ('outbound', 'reverse', 'return')[self.index]
            self.step_started = now
        return tuple(self.target)

    def status(self, feedback, context):
        ready, reason = self.readiness(feedback, context)
        angles = feedback.get('actual_angles_deg')
        return dict(phase=self.phase, reason=self.reason, request_id=self.request_id, axes=self.axes,
                    active=self.active, can_start=ready and not self.active,
                    ready_reason='self_test_active' if self.active else reason,
                    target_deg=self.target, actual_angles_deg=angles,
                    error_deg=[a-b for a, b in zip(angles, self.target)] if angles and self.target else None,
                    progress=self.index/3, samples=self.samples,
                    last_request_id=self.last_request_id, last_result=self.last_result,
                    last_request_reason=self.last_request_reason, last_request_stamp=self.last_request_stamp,
                    hardware_verified=self.hardware_verified,
                    verification_scope='physical_encoder_feedback' if self.hardware_verified else 'not_hardware_verified')
