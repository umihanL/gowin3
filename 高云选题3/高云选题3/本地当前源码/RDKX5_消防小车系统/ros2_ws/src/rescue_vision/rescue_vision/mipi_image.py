"""ROS-independent decoding and bounded, timestamp-based MIPI frame pairing.

Pairing close timestamps does not establish synchronized camera exposures.
NV12 height is the visible Y-plane height, step is the Y/UV row stride.
"""
from collections import deque
from dataclasses import dataclass
import math

import cv2
import numpy as np


def image_storage_view(data, width, height, step, encoding):
    """Validate Image storage without a pixel copy or color conversion."""
    for name, value in (("width", width), ("height", height), ("step", step)):
        if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value <= 0:
            raise ValueError(name + " must be a positive integer")
    encoding = str(encoding).lower()
    if encoding not in ("nv12", "bgr8", "rgb8", "mono8"):
        raise ValueError("Unsupported MIPI image encoding: " + encoding)
    channels = 3 if encoding in ("bgr8", "rgb8") else 1
    if step < width * channels:
        raise ValueError("Image step is shorter than a visible row")
    if encoding == "nv12" and (width % 2 or height % 2 or step % 2):
        raise ValueError("NV12 width, height and Y/UV stride must be even")
    rows = height * 3 // 2 if encoding == "nv12" else height
    try:
        raw = np.frombuffer(data, dtype=np.uint8)
    except (TypeError, ValueError) as exc:
        raise ValueError("Image data must expose a contiguous byte buffer") from exc
    if raw.size != rows * step:
        raise ValueError(f"Image data length {raw.size} differs from expected {rows * step}")
    return raw.reshape(rows, step)[:, :width * channels]


def image_to_bgr(data, width, height, step, encoding):
    """Decode uint8 ROS Image storage, honoring padding and exact byte length."""
    encoding = str(encoding).lower()
    visible = np.ascontiguousarray(image_storage_view(data, width, height, step, encoding))
    if encoding == "nv12":
        return cv2.cvtColor(visible, cv2.COLOR_YUV2BGR_NV12)
    if encoding == "mono8":
        return cv2.cvtColor(visible, cv2.COLOR_GRAY2BGR)
    visible = visible.reshape(height, width, 3)
    return cv2.cvtColor(visible, cv2.COLOR_RGB2BGR) if encoding == "rgb8" else visible.copy()


def stamp_nanoseconds(stamp):
    """Read a ROS-like stamp without float precision loss at UNIX epoch scale."""
    sec, nsec = int(stamp.sec), int(stamp.nanosec)
    if sec < 0 or not 0 <= nsec < 1_000_000_000:
        raise ValueError("Invalid image timestamp")
    return sec * 1_000_000_000 + nsec


@dataclass(frozen=True)
class StampedFrame:
    stamp_ns: int
    payload: object


class LatestStereoPairer:
    """Keep a small backlog but deliver only the newest unconsumed usable pair.

    Images are not copied or decoded here. A slow downstream timer intentionally
    skips older frames rather than replaying stale observations.
    """
    def __init__(self, max_skew_s=.012, max_age_s=.5, queue_size=8):
        if not math.isfinite(max_skew_s) or not 0 <= max_skew_s <= 1:
            raise ValueError("max_skew_s must be finite and in [0, 1]")
        if not math.isfinite(max_age_s) or not 0 < max_age_s <= 10:
            raise ValueError("max_age_s must be finite and in (0, 10]")
        if isinstance(queue_size, bool) or not isinstance(queue_size, int) or not 1 <= queue_size <= 64:
            raise ValueError("queue_size must be an integer in [1, 64]")
        self.max_skew_ns = round(max_skew_s * 1e9)
        self.max_age_ns = round(max_age_s * 1e9)
        self.queue_size = queue_size
        self.queues = {"left": deque(), "right": deque()}
        self.last_seen = {"left": -1, "right": -1}
        self.counts = dict(received=0, invalid_age=0, out_of_order=0,
                           queue_overflow=0, expired=0, skipped=0, paired=0)

    def push(self, eye, stamp_ns, payload, now_ns):
        if eye not in self.queues:
            raise ValueError("eye must be left or right")
        self.counts["received"] += 1
        if stamp_ns <= 0 or not 0 <= now_ns - stamp_ns <= self.max_age_ns:
            self.counts["invalid_age"] += 1
            return False
        if stamp_ns <= self.last_seen[eye]:
            self.counts["out_of_order"] += 1
            return False
        self.last_seen[eye] = stamp_ns
        queue = self.queues[eye]
        queue.append(StampedFrame(stamp_ns, payload))
        while len(queue) > self.queue_size:
            queue.popleft()
            self.counts["queue_overflow"] += 1
        return True

    def take_latest(self, now_ns):
        for queue in self.queues.values():
            retained = [f for f in queue if 0 <= now_ns - f.stamp_ns <= self.max_age_ns]
            self.counts["expired"] += len(queue) - len(retained)
            queue.clear()
            queue.extend(retained)
        candidates = [(left, right) for left in self.queues["left"]
                      for right in self.queues["right"]
                      if abs(left.stamp_ns - right.stamp_ns) <= self.max_skew_ns]
        if not candidates:
            return None
        left, right = max(candidates, key=lambda p: (
            min(p[0].stamp_ns, p[1].stamp_ns), -abs(p[0].stamp_ns - p[1].stamp_ns)))
        for eye, selected in (("left", left), ("right", right)):
            queue = self.queues[eye]
            while queue and queue[0].stamp_ns <= selected.stamp_ns:
                discarded = queue.popleft()
                if discarded is not selected:
                    self.counts["skipped"] += 1
        self.counts["paired"] += 1
        return left, right
