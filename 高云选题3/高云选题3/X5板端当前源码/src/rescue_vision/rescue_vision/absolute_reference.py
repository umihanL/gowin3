"""Persist a mechanical reference in the driver's single-turn encoder frame.

No I/O or movement. PD42 uses READ_POSITION modulo 51200 (V1.2 p14).
ZDT uses calibrated absolute encoder 0x31, not its boot-relative 0x36.
The caller verifies device/config identity and three stationary driver states.
Recalibration, mechanical coupling changes or external coordinate clearing
require a new reference; multi-turn ambiguity is never silently resolved.
"""
import math
import time


PERIODS = {'pd42': 51200, 'zdt_xs': 65536}
AXES = ('pan', 'tilt')
FIELDS = ('protocol', 'reference_angle_deg', 'direction', 'gear_ratio',
          'min_angle_deg', 'max_angle_deg')


def _number(value):
    return type(value) in (float, int) and math.isfinite(value)


def _axis(axis):
    if not isinstance(axis, dict) or axis.get('protocol') not in PERIODS:
        raise ValueError('absolute_reference_protocol_unsupported')
    for key in ('reference_angle_deg', 'gear_ratio', 'min_angle_deg', 'max_angle_deg'):
        if not _number(axis.get(key)):
            raise ValueError('absolute_reference_axis_geometry_invalid')
    lo, hi, ref, gear = (axis[k] for k in (
        'min_angle_deg', 'max_angle_deg', 'reference_angle_deg', 'gear_ratio'))
    if (not lo < hi or not lo <= ref <= hi or gear <= 0 or
            type(axis.get('direction')) is not int or axis['direction'] not in (-1, 1)):
        raise ValueError('absolute_reference_axis_geometry_invalid')
    if (hi-lo)*gear >= 360:
        raise ValueError('absolute_reference_multiturn_range_ambiguous')
    return PERIODS[axis['protocol']]


def _samples(servo, rows, now):
    if not isinstance(rows, (list, tuple)) or len(rows) < 3:
        raise ValueError('absolute_reference_three_samples_required')
    now = time.monotonic() if now is None else now
    ttl = servo.get('feedback_timeout_s', .6)
    if not _number(now) or not _number(ttl) or not 0 < ttl <= 1:
        raise ValueError('absolute_reference_feedback_timing_invalid')
    rows = rows[-3:]
    stamps = [r.get('started') if isinstance(r, dict) else None for r in rows]
    if (not all(_number(s) for s in stamps) or not stamps[0] < stamps[1] < stamps[2]
            or not 0 <= now-stamps[-1] <= ttl or stamps[-1]-stamps[0] > max(1., 3*ttl)):
        raise ValueError('absolute_reference_feedback_stale_or_unordered')
    for name in AXES:
        period = _axis(servo[name])
        relative, absolute = [], []
        for row in rows:
            sample = row.get(name)
            if not isinstance(sample, dict):
                raise ValueError('absolute_reference_encoder_feedback_missing')
            pos, enc = sample.get('position_counts'), sample.get('absolute_counts')
            if (type(pos) is not int or not -(2**31) <= pos < 2**31 or
                    type(enc) is not int or not 0 <= enc < period or sample.get('error')):
                raise ValueError('absolute_reference_encoder_feedback_missing')
            if not _number(sample.get('speed_rpm')) or sample['speed_rpm'] != 0:
                raise ValueError('absolute_reference_axis_not_stationary')
            if servo[name]['protocol'] == 'pd42' and enc != pos % period:
                raise ValueError('absolute_reference_pd42_encoder_source_mismatch')
            relative.append(pos)
            absolute.append(enc)
        # Raw driver angle, independent of gearing: match the existing capture
        # stability bound, allowing a single-turn wrap through encoder zero.
        if ((max(relative)-min(relative))*360/period > .03 or
                any(abs((value-absolute[-1]+period//2) % period-period//2)*360/period > .03
                    for value in absolute)):
            raise ValueError('absolute_reference_axis_moved_during_capture')
    return rows


def build_absolute_reference(servo, rows, reference_id, *, now=None):
    """Record the operator-confirmed mechanical placement; never clear encoders."""
    if not isinstance(reference_id, str) or not reference_id.strip():
        raise ValueError('absolute_reference_identity_required')
    rows = _samples(servo, rows, now)
    axes = {}
    for name in AXES:
        axis = servo[name]
        axes[name] = dict({key: axis[key] for key in FIELDS},
            counts_per_turn=PERIODS[axis['protocol']],
            zero_absolute_counts=rows[-1][name]['absolute_counts'])
    return dict(schema_version=1, reference_id=reference_id, axes=axes)


def restore_absolute_reference(servo, rows, anchor, *, now=None):
    """Resolve one unique mechanical angle within limits, rebase live counts.

    Returns pan/tilt dictionaries containing reference_counts and actual_angle_deg.
    Current position is NOT marked zero: the saved single-turn anchor determines
    displacement even when the ZDT runtime coordinate resets to zero on reboot.
    """
    rows = _samples(servo, rows, now)
    if (not isinstance(anchor, dict) or anchor.get('schema_version') != 1 or
            not isinstance(anchor.get('reference_id'), str) or not anchor['reference_id'].strip()
            or not isinstance(anchor.get('axes'), dict)
            or set(anchor.get('axes', {})) != set(AXES)):
        raise ValueError('absolute_reference_anchor_invalid')
    tolerance = servo.get('feedback_limit_tolerance_deg', .2)
    if not _number(tolerance) or not 0 <= tolerance <= .5:
        raise ValueError('absolute_reference_limit_tolerance_invalid')
    result = {}
    for name in AXES:
        axis, saved = servo[name], anchor['axes'][name]
        period = _axis(axis)
        if (not isinstance(saved, dict) or saved.get('counts_per_turn') != period or
                any(saved.get(key) != axis.get(key) for key in FIELDS)):
            raise ValueError('absolute_reference_installation_changed')
        zero = saved.get('zero_absolute_counts')
        if type(zero) is not int or not 0 <= zero < period:
            raise ValueError('absolute_reference_anchor_encoder_invalid')
        factor = axis['direction']*axis['gear_ratio']*period/360.
        if not math.isfinite(factor) or factor == 0:
            raise ValueError('absolute_reference_axis_geometry_invalid')
        ref = axis['reference_angle_deg']
        low, high = sorted(((axis['min_angle_deg']-tolerance-ref)*factor,
                            (axis['max_angle_deg']+tolerance-ref)*factor))
        reference_values = []
        for row in rows:
            sample = row[name]
            residual = (sample['absolute_counts']-zero) % period
            first, last = math.ceil((low-residual)/period), math.floor((high-residual)/period)
            if first > last:
                raise ValueError('absolute_reference_measured_angle_outside_limits')
            if first != last:
                raise ValueError('absolute_reference_wrap_ambiguous')
            displacement = residual+first*period
            reference_counts = sample['position_counts']-displacement
            if not -(2**31) <= reference_counts < 2**31:
                raise ValueError('absolute_reference_live_counts_overflow')
            reference_values.append(reference_counts)
        if (max(reference_values)-min(reference_values))*360/period > .03:
            raise ValueError('absolute_reference_live_coordinate_changed_during_capture')
        result[name] = dict(reference_counts=reference_values[-1],
            actual_angle_deg=ref+displacement/factor,
            absolute_counts=rows[-1][name]['absolute_counts'],
            zero_absolute_counts=zero, source='persisted_single_turn_encoder')
    return result
