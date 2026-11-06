"""Bounded exact-left-stamp join; proximity alone never proves exposure sync."""
from collections import OrderedDict
import math


class DepthFrameJoin:
    def __init__(self, max_age_s=.5, max_skew_s=.012, capacity=16):
        if (not math.isfinite(max_age_s) or not 0 < max_age_s <= .5
                or not math.isfinite(max_skew_s) or not 0 <= max_skew_s <= .020
                or isinstance(capacity, bool) or not isinstance(capacity, int) or not 2 <= capacity <= 32):
            raise ValueError('Invalid stereo join bounds')
        self.age_ns, self.skew_ns = round(max_age_s * 1e9), round(max_skew_s * 1e9)
        self.capacity = capacity
        self.frames = {'left': OrderedDict(), 'right': OrderedDict()}
        self.last_stamp = {'left': -1, 'right': -1}

    def push(self, eye, stamp_ns, payload, now_ns):
        if eye not in self.frames:
            raise ValueError('Unknown physical eye')
        if stamp_ns <= self.last_stamp[eye] or stamp_ns <= 0 or not 0 <= now_ns - stamp_ns <= self.age_ns:
            return False
        self.last_stamp[eye] = stamp_ns
        queue = self.frames[eye]
        queue[stamp_ns] = payload
        while len(queue) > self.capacity:
            queue.popitem(last=False)
        self.prune(now_ns)
        return True

    def prune(self, now_ns):
        for queue in self.frames.values():
            for stamp in list(queue):
                if not 0 <= now_ns - stamp <= self.age_ns:
                    del queue[stamp]

    def match(self, detection_stamp_ns, now_ns):
        self.prune(now_ns)
        if not 0 <= now_ns - detection_stamp_ns <= self.age_ns:
            return None, 'stale_detection'
        left = self.frames['left'].get(detection_stamp_ns)
        if left is None:
            return None, 'exact_left_frame_missing'
        matches = [(stamp, frame) for stamp, frame in self.frames['right'].items()
                   if abs(stamp - detection_stamp_ns) <= self.skew_ns]
        if not matches:
            return None, 'matching_right_frame_missing'
        # Two plausible exposure identities are unsuitable for precision range.
        if len(matches) != 1:
            return None, 'ambiguous_right_frame'
        right_stamp, right = matches[0]
        return (left, right, (right_stamp - detection_stamp_ns) / 1e9), 'ok'
