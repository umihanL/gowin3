"""Single-command Emm target positioning for the installed ZDT configuration.

Use the vendor FD mode 0 (relative to the retained command target), with the
same encoded speed 1 and acceleration 1 as the earlier eight-pulse trial.
The complete remaining angle is sent in one packet. The 0.1125-degree pulse
resolution is not a software travel cap. Unknown option bit 10 remains part
of the exact configuration fingerprint, not a general protocol approval.
"""
import math

from . import zdt_vendor_bench as vendor
from . import zdt_xs_protocol as zdt
from .gimbal_limits import feedback_limit_tolerance, require_measured_limits

PROFILE = 'zdt_emm_target_closed_loop_v2'
LEGACY_PROFILE = 'zdt_fd8_closed_loop_v1'
STEP_TIMEOUT_S = 4.0
COUNT_DEG = vendor.COUNT_DEG
PULSE_DEG = 1.8/16
STEP_COUNTS = PULSE_DEG/COUNT_DEG
ENCODED_SPEED = 1
ENCODED_ACCELERATION = 1
NOMINAL_SPEED_DEG_S = 6.0


def validate_snapshot(axis, baudrate, tolerance, *, reference_id=None, boot_id=None,
                      feedback_limit_tolerance_deg=.2):
    measurement_tolerance = feedback_limit_tolerance({'feedback_limit_tolerance_deg':feedback_limit_tolerance_deg})
    snap = axis.get('zdt_fd8_snapshot')
    if not isinstance(snap, dict) or snap.get('profile') not in (PROFILE, LEGACY_PROFILE):
        raise ValueError('Explicit installed ZDT commissioning snapshot required')
    # Migrate software strategy only; captured encoder/configuration values and
    # all driver settings are unchanged, and no command is sent by validation.
    snap['profile'] = PROFILE
    try:
        raw = bytes.fromhex(snap['config_data_hex'])
        options = {'raw_bytes_hex': snap['options_raw_hex']}
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('FD8 raw configuration/options snapshot invalid') from exc
    if not vendor.matches(snap.get('version', {}), raw, options):
        raise ValueError('FD8 exact firmware/config/options profile mismatch')
    if baudrate != 115200 or axis['address'] != 1 or axis['gear_ratio'] != 1:
        raise ValueError('FD8 requires address 1, 115200 and confirmed direct drive')
    if not .45 <= tolerance <= .5:
        raise ValueError('FD8 requires explicit 0.45..0.5 degree target tolerance')
    lo, hi, ref = (axis[k] for k in ('min_angle_deg', 'max_angle_deg', 'reference_angle_deg'))
    if hi-lo > 180 or lo < ref-90 or hi > ref+90:
        raise ValueError('FD8 travel is at most 180 degrees total, +/-90 from reference')
    for key in ('reference_position_counts', 'retained_target_counts', 'realtime_target_counts'):
        if type(snap.get(key)) is not int or not -(2**31) <= snap[key] < 2**31:
            raise ValueError('FD8 captured counts must be signed int32')
    if snap['reference_position_counts'] != axis['reference_counts']:
        raise ValueError('FD8 reference must use this start captured actual position')
    offset = snap.get('startup_encoder_offset_counts')
    if offset is not None and (type(offset) is not int or not 0 <= offset < 65536):
        raise ValueError('FD8 absolute encoder offset must be uint16')
    startup_keys = ('startup_position_counts', 'startup_reference_id', 'startup_boot_id')
    startup = snap['reference_position_counts']
    if any(key in snap for key in startup_keys):
        if not all(key in snap for key in startup_keys):
            raise ValueError('FD8 startup snapshot requires position, reference ID and boot ID together')
        if (not isinstance(reference_id, str) or not reference_id or
                snap['startup_reference_id'] != reference_id or
                not isinstance(boot_id, str) or not boot_id or snap['startup_boot_id'] != boot_id):
            raise ValueError('FD8 startup snapshot reference ID or boot ID mismatch/unverified')
        startup = snap['startup_position_counts']
        if type(startup) is not int or not -(2**31) <= startup < 2**31:
            raise ValueError('FD8 startup position must be signed int32')
        try:
            require_measured_limits('FD8 startup', startup, angle_for_counts(startup, axis), axis, measurement_tolerance)
            for value in (snap['retained_target_counts'], snap['realtime_target_counts']):
                require_limits(value, axis)
        except RuntimeError as exc:
            raise ValueError(str(exc)) from exc
    if (abs(snap['retained_target_counts']-startup)*COUNT_DEG > vendor.STATIONARY_TOLERANCE_DEG or
            abs(snap['realtime_target_counts']-snap['retained_target_counts']) > 1):
        raise ValueError('FD8 snapshot contains an unaligned retained target')
    return snap


def motion_timeout_s(pan_span_deg, *, overhead_s=10.):
    """Travel budget at unchanged 1 RPM, plus measured-settling allowance."""
    return overhead_s + STEP_TIMEOUT_S + 2*abs(pan_span_deg)/NOMINAL_SPEED_DEG_S


def motion_frame(direction, pulses):
    if type(direction) is not int or direction not in (-1, 1):
        raise ValueError('ZDT direction must be -1 or 1')
    if type(pulses) is not int or not 1 <= pulses <= 0xFFFFFFFF:
        raise ValueError('ZDT pulse count must be a nonzero uint32')
    return zdt.position_request(1, direction*pulses*PULSE_DEG, ENCODED_SPEED,
        mode='emm', motion_mode=0, microsteps=16, motor_step_deg=1.8,
        speed_div10=False, acceleration=ENCODED_ACCELERATION)


def angle_for_counts(counts, axis):
    return axis['reference_angle_deg'] + (counts-axis['reference_counts'])*COUNT_DEG/axis['direction']


def require_limits(counts, axis):
    angle = angle_for_counts(counts, axis)
    if not axis['min_angle_deg'] <= angle <= axis['max_angle_deg']:
        raise RuntimeError('FD8 measured/projected position exceeds mechanical limits')
    return angle


def reachable_range(axis, snapshot):
    """Actual in-limit lattice endpoints for the captured origin/direction.

    Mechanical limits are unchanged. This is a pure preview, never a clamp,
    motor command or evidence that the full travel was measured on hardware.
    """
    origin = snapshot['retained_target_counts']
    lo, hi = sorted(axis['reference_counts'] + (a-axis['reference_angle_deg'])*axis['direction']/COUNT_DEG
                    for a in (axis['min_angle_deg'], axis['max_angle_deg']))
    first = math.ceil((lo-origin)/STEP_COUNTS)
    last = math.floor((hi-origin)/STEP_COUNTS)
    limits = (sorted(angle_for_counts(origin+i*STEP_COUNTS, axis) for i in (first,last))
              if first <= last else None)
    return dict(limits_deg=limits, step_deg=PULSE_DEG, step_indices=[first,last])


def quantized_target(requested, axis, snapshot, tolerance):
    """Choose an in-limit lattice point; never silently accept an unreachable end."""
    origin = snapshot['retained_target_counts']
    desired = axis['reference_counts'] + (requested-axis['reference_angle_deg'])*axis['direction']/COUNT_DEG
    reachable = reachable_range(axis, snapshot)
    if reachable['limits_deg'] is None:
        raise ValueError('FD8 no realizable lattice point inside mechanical limits')
    first, last = reachable['step_indices']
    index = max(first, min(last, round((desired-origin)/STEP_COUNTS)))
    actual = require_limits(origin+index*STEP_COUNTS, axis)
    if abs(actual-requested) > tolerance:
        raise ValueError('ZDT requested boundary is not reachable on the pulse lattice within tolerance')
    return actual


class FD8Stepper:
    """Compatibility class name; one complete target command is outstanding."""
    def __init__(self, axis, snapshot, *, feedback_limit_tolerance_deg=.2,
                 position_tolerance_deg=.5):
        self.axis = axis
        self.feedback_limit_tolerance = feedback_limit_tolerance(
            {'feedback_limit_tolerance_deg':feedback_limit_tolerance_deg})
        if not .45 <= position_tolerance_deg <= .5:
            raise ValueError('FD8 position tolerance must be 0.45..0.5 degrees')
        self.position_tolerance = position_tolerance_deg
        self.expected = snapshot['retained_target_counts']
        self.pending = None
        self.stable = 0
        self.stable_counts = []
        self.steps = 0
        self.last_observation = None
        self.last_unstable_observation = None
        self.last_fault_detail = None
        self.unstable_samples = 0

    def _record(self, sample, expected, now, outcome):
        # Retain the actual rejected/unstable sample after a later good sample
        # so an intermittent fault can be diagnosed without guessing its cause.
        record = dict(at_monotonic=now, expected_counts=expected,
                      pending=self.pending is not None, outcome=outcome,
                      sample={key: (str(value) if isinstance(value, float) and not math.isfinite(value) else value)
                              for key, value in sample.items()})
        self.last_observation = record
        if outcome not in ('stable', 'collecting_stable_samples'):
            self.last_unstable_observation = record
            self.unstable_samples += 1

    def diagnostic(self):
        return dict(expected_counts=self.expected, stable_samples=self.stable,
                    stationary_tolerance_deg=vendor.STATIONARY_TOLERANCE_DEG,
                    feedback_limit_tolerance_deg=self.feedback_limit_tolerance,
                    trajectory_tolerance_deg=self.position_tolerance,
                    unstable_samples=self.unstable_samples,
                    last_observation=self.last_observation,
                    last_unstable_observation=self.last_unstable_observation,
                    last_fault_detail=self.last_fault_detail)

    def observe(self, sample, now):
        expected = self.pending['expected'] if self.pending else self.expected
        def reject(reason, message, *, fault_kind=None, value=None, limit=None):
            self.last_fault_detail = dict(kind=fault_kind or reason, value=value, limit=limit,
                speed_rpm=sample.get('speed_rpm'), following_error_deg=sample.get('position_error_deg'),
                command_pending=self.pending is not None, commanded_speed_rpm=ENCODED_SPEED,
                at_monotonic=now)
            self._record(sample, expected, now, reason)
            raise RuntimeError(message)
        for key in ('position_counts', 'position_before_counts', 'retained_target_counts',
                    'realtime_target_counts'):
            if type(sample.get(key)) is not int:
                reject('invalid_integer_feedback', 'FD8 complete integer encoder/target feedback required')
        for key in ('position_counts', 'position_before_counts', 'retained_target_counts',
                    'realtime_target_counts'):
            try:
                if key in ('position_counts', 'position_before_counts'):
                    require_measured_limits('FD8 '+key, sample[key], angle_for_counts(sample[key], self.axis),
                                            self.axis, self.feedback_limit_tolerance)
                else:
                    require_limits(sample[key], self.axis)
            except RuntimeError as exc:
                reject('mechanical_limit', str(exc))
        for key in ('speed_rpm', 'position_error_deg'):
            if type(sample.get(key)) not in (int, float) or not math.isfinite(sample[key]):
                reject('invalid_dynamic_feedback', 'FD8 finite speed and following error required')
        if abs(sample['speed_rpm']) > 2:
            reject('overspeed_or_following_error', 'FD8 overspeed or excessive following error',
                   fault_kind='overspeed', value=sample['speed_rpm'], limit=2.)
        allowed_error = 1.2
        if self.pending is not None:
            p = self.pending
            # The retained target is the complete destination. The realtime
            # setpoint may traverse the path, so a large remaining angle alone
            # is not a following-error fault during a long command.
            low, high = sorted((p['start'], p['expected']))
            # Encoder reads bracket several sequential UART transactions. A
            # low-speed controller may overshoot a destination by 0.3 degrees
            # before settling. Use the already configured arrival tolerance
            # here; mechanical endpoints retain their independent stricter
            # measured-limit check above. This never declares arrival early.
            allowance = max(self.feedback_limit_tolerance, self.position_tolerance)/COUNT_DEG
            if any(not low-allowance <= sample[key] <= high+allowance
                   for key in ('position_before_counts', 'position_counts')):
                reject('trajectory_envelope', 'ZDT encoder left the commanded angle interval')
            if abs(sample['retained_target_counts']-p['expected']) > 2:
                reject('command_target_mismatch', 'ZDT retained target differs from full requested target; no resend')
            if not low-2 <= sample['realtime_target_counts'] <= high+2:
                reject('trajectory_target_mismatch', 'ZDT realtime target left the commanded angle interval')
            remaining = abs(sample['position_counts']-p['expected'])*COUNT_DEG
            allowed_error += max(abs(sample['position_before_counts']-p['expected'])*COUNT_DEG, remaining)
            if remaining < p['best_remaining_deg']-.05:
                p['best_remaining_deg'] = remaining
                p['progress_at'] = now
            if remaining > vendor.STATIONARY_TOLERANCE_DEG and now-p['progress_at'] >= STEP_TIMEOUT_S:
                reject('motion_no_progress', 'ZDT measured motion made no progress for four seconds; no resend')
        if abs(sample['position_error_deg']) > allowed_error:
            reject('overspeed_or_following_error', 'FD8 overspeed or excessive following error',
                   fault_kind='following_error', value=sample['position_error_deg'], limit=allowed_error)
        if self.pending is not None and now >= self.pending['deadline']:
            reject('step_arrival_timeout', 'FD8 measured step arrival timeout; no resend')
        # With no command pending, a changed command register invalidates the
        # reference. Unchanged targets with encoder jitter or nonzero speed
        # only mean "not settled": no new relative step may be issued yet.
        if self.pending is None and (
                abs(sample['retained_target_counts']-expected) > 2 or
                abs(sample['realtime_target_counts']-expected) > 2 or
                abs(sample['retained_target_counts']-sample['realtime_target_counts']) > 1):
            reject('retained_target_changed', 'FD8 stationary reference/retained target changed; recommission required')
        good = vendor.settled(sample, expected)
        if self.pending is not None:
            good = good and sample.get('enabled') is True and sample.get('arrived') is True
        if good:
            self.stable_counts = (self.stable_counts + [sample['position_counts']])[-3:]
            self.stable = len(self.stable_counts)
            if (max(self.stable_counts)-min(self.stable_counts))*COUNT_DEG > .03:
                self.stable = 0
                self.stable_counts = []
                good = False
        else:
            self.stable = 0
            self.stable_counts = []
        if self.pending and self.stable >= 3:
            # Carry the measured retained target forward, avoiding cumulative
            # firmware integer rounding assumptions over a long trajectory.
            self.expected = sample['retained_target_counts']
            self.pending = None
        self._record(sample, expected, now,
                     'stable' if self.pending is None and self.stable >= 3 else
                     'collecting_stable_samples' if good else
                     'waiting_step_arrival' if self.pending is not None else 'waiting_stationary_feedback')
        return self.pending is None and self.stable >= 3

    def plan_step(self, target, tolerance):
        if not self.axis['min_angle_deg'] <= target <= self.axis['max_angle_deg']:
            raise RuntimeError('ZDT requested target exceeds mechanical limits')
        if self.pending is not None or self.stable < 3:
            return None
        current = angle_for_counts(self.expected, self.axis)
        if abs(current-target) <= tolerance:
            return None
        direction = (1 if target > current else -1)*self.axis['direction']
        desired = self.axis['reference_counts'] + (target-self.axis['reference_angle_deg'])*self.axis['direction']/COUNT_DEG
        pulses = max(1, round(abs(desired-self.expected)/STEP_COUNTS))
        expected = self.expected + direction*pulses*STEP_COUNTS
        # Rounding to a pulse must never cross the installed endpoints.
        while pulses and not self.axis['min_angle_deg'] <= angle_for_counts(expected, self.axis) <= self.axis['max_angle_deg']:
            pulses -= 1
            expected = self.expected + direction*pulses*STEP_COUNTS
        if not pulses:
            raise RuntimeError('ZDT remaining target has no in-limit pulse')
        require_limits(expected, self.axis)
        return direction, expected

    def mark_written(self, direction, expected, now):
        if self.pending is not None:
            raise RuntimeError('FD8 cannot stack relative motion')
        pulses = round(abs(expected-self.expected)/STEP_COUNTS)
        if (direction not in (-1, 1) or not pulses or
                abs(expected-(self.expected+direction*pulses*STEP_COUNTS)) > 1e-6):
            raise ValueError('ZDT full target must match its signed pulse count')
        require_limits(expected, self.axis)
        span = abs(expected-self.expected)*COUNT_DEG
        packet = motion_frame(direction, pulses)
        self.pending = {'expected': expected, 'start': self.expected, 'pulses': pulses,
                        'deadline': now+motion_timeout_s(span, overhead_s=0),
                        'progress_at': now, 'best_remaining_deg': span}
        self.stable = 0
        self.stable_counts = []
        self.steps += 1
        return packet
