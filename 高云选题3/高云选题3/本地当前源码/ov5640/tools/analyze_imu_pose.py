"""Compare two stationary endpoint windows using quaternion distance, not Euler subtraction.

The fixture must actually return to the same orientation. This measures the
received orientation difference; it cannot infer sensor accuracy from freehand motion.
"""
import argparse
import json
import math
from pathlib import Path


def normalize(q):
    norm = math.sqrt(sum(x*x for x in q))
    if not 0.5 <= norm <= 1.5:
        raise ValueError('Invalid quaternion norm')
    return tuple(x/norm for x in q)


def distance(a, b):
    dot = abs(sum(x*y for x, y in zip(a, b)))
    return math.degrees(2*math.acos(min(1.0, dot)))


def mean_quaternion(window):
    anchor = window[0]
    aligned = [q if sum(x*y for x, y in zip(anchor, q)) >= 0
               else tuple(-x for x in q) for q in window]
    return normalize(tuple(sum(q[i] for q in aligned)/len(aligned) for i in range(4)))


def analyze(text, window=20):
    samples = []
    ignored = 0
    for line in text.splitlines():
        if not line.strip().startswith('samples:'):
            continue
        try:
            tokens = line.strip()[8:].split(',')
            if any(not t.strip() for t in tokens):
                raise ValueError('Empty field')
            values = tuple(map(float, tokens))
            if len(values) != 10 or not all(map(math.isfinite, values)):
                raise ValueError('Expected ten finite values')
            samples.append(normalize(values[6:10]))
        except ValueError:
            ignored += 1
    if window < 2 or len(samples) < 2*window:
        raise ValueError('Need at least two non-overlapping stationary windows of ten-channel frames')
    start, end = samples[:window], samples[-window:]
    first, last = mean_quaternion(start), mean_quaternion(end)
    return dict(valid_frames=len(samples), ignored_frames=ignored, window_frames=window,
                endpoint_orientation_difference_deg=round(distance(first, last), 5),
                start_window_max_deviation_deg=round(max(distance(first, q) for q in start), 5),
                end_window_max_deviation_deg=round(max(distance(last, q) for q in end), 5),
                note='Endpoint orientation difference, not proof of drift; both windows must be physically stationary at the same fixture orientation.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path, help='Saved ten-channel FireWater text')
    parser.add_argument('--window', type=int, default=20)
    args = parser.parse_args()
    print(json.dumps(analyze(args.input.read_text(encoding='utf-8-sig'), args.window), indent=2))
