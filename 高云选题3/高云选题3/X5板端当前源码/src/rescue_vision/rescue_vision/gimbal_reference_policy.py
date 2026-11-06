"""Narrow fault classification for operator-requested software re-referencing."""
import re


def recoverable_reference_fault(error):
    """Only a coordinate/limit mismatch may enter read-only reference recovery.

    Driver faults, emergency stops and appended shutdown errors are deliberately
    excluded. New raw samples still have to prove stationary aligned targets.
    """
    return isinstance(error, str) and re.fullmatch(
        r'(?:pan|tilt|FD8) measured position outside calibrated limits/reference: '
        r'actual_deg=-?\d+(?:\.\d+)?, counts=-?\d+, '
        r'limits_deg=\[-?\d+(?:\.\d+)?,\s*-?\d+(?:\.\d+)?\], '
        r'feedback_limit_tolerance_deg=\d+(?:\.\d+)?', error) is not None
