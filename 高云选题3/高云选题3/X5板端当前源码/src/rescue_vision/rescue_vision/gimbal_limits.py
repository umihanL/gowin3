"""Measured feedback allowance is separate from strict commanded travel."""
import math


DEFAULT_FEEDBACK_LIMIT_TOLERANCE_DEG = .2


def feedback_limit_tolerance(servo):
    value = servo.get('feedback_limit_tolerance_deg', DEFAULT_FEEDBACK_LIMIT_TOLERANCE_DEG)
    if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= .3:
        raise ValueError('feedback_limit_tolerance_deg must be finite in [0,0.3] degrees')
    return float(value)


def measured_in_limits(angle, axis, tolerance):
    lo, hi = axis.get('min_angle_deg'), axis.get('max_angle_deg')
    return (all(type(v) in (int, float) and math.isfinite(v) for v in (angle, lo, hi))
            and lo-tolerance <= angle <= hi+tolerance)


def require_measured_limits(name, counts, angle, axis, tolerance):
    if not measured_in_limits(angle, axis, tolerance):
        raise RuntimeError(
            f'{name} measured position outside calibrated limits/reference: '
            f'actual_deg={angle!r}, counts={counts!r}, '
            f'limits_deg=[{axis.get("min_angle_deg")!r},{axis.get("max_angle_deg")!r}], '
            f'feedback_limit_tolerance_deg={tolerance!r}')
    return angle
