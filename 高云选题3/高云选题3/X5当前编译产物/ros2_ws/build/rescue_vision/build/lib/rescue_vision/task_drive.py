"""Temporal zebra confirmation and explicit, measured camera task poses."""
from collections import deque
import math


class CrosswalkLatch:
    def __init__(self, min_frames=3, min_duration_s=.15, max_gap_s=.25, clear_frames=5, cooldown_s=2.):
        self.min_frames, self.min_duration = min_frames, min_duration_s
        self.max_gap, self.clear_frames, self.cooldown = max_gap_s, clear_frames, cooldown_s
        self.hits = deque(maxlen=10)
        self.last_capture = None
        self.armed = True
        self.clear_count = 0
        self.completed_at = None

    def complete(self, now):
        self.armed = False
        self.hits.clear()
        self.clear_count = 0
        self.completed_at = now

    def observe(self, capture, detected, valid, now):
        if (not isinstance(capture, (int, float)) or not math.isfinite(capture)
                or not 0 <= now - capture <= .3
                or self.last_capture is not None and capture <= self.last_capture):
            return False
        if self.last_capture is not None and capture-self.last_capture > self.max_gap:
            self.hits.clear()
            self.clear_count = 0
        self.last_capture = capture
        if valid is not True:
            self.hits.clear(); self.clear_count = 0
            return False
        if not self.armed:
            self.clear_count = self.clear_count + 1 if detected is False else 0
            if (self.completed_at is not None and now-self.completed_at >= self.cooldown
                    and self.clear_count >= self.clear_frames):
                self.armed = True
            return False
        if detected is not True:
            self.hits.clear()
            return False
        self.hits.append(capture)
        if len(self.hits) >= self.min_frames and self.hits[-1]-self.hits[0] >= self.min_duration:
            self.armed = False
            self.completed_at = None
            self.hits.clear()
            return True
        return False


def road_pose(mount):
    if mount.get('mounting_verified') is not True or mount.get('road_pose_verified') is not True:
        return None
    values = [mount.get('road_pan_deg'), mount.get('road_tilt_deg')]
    for value, key in zip(values, ('pan_limits_deg', 'tilt_limits_deg')):
        bounds = mount.get(key, ())
        if (not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value)
                or len(bounds) != 2 or not bounds[0] <= value <= bounds[1]):
            return None
    return tuple(float(v) for v in values)
