"""Per-target, fresh-frame observation policy independent of YOLO/hardware.

Distances belong to individual detections. They must come from calibrated depth
or explicitly associated measurements, never from a class's assumed size.
Outputs are requests; a navigation/safety controller owns all vehicle motion.
"""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass, field, asdict
import json
import math
from pathlib import Path


@dataclass
class Policy:
    range_min_m: float = .4
    range_max_m: float = 1.0
    min_height_px: float = 40
    min_short_side_px: float = 12
    confidence: float = .25
    window_frames: int = 5
    min_hits: int = 3
    association_iou: float = .3
    max_frame_gap_s: float = .5
    stable_pan_tilt_deg: float = 2
    alignment_tolerance_deg: float = 4
    max_obliquity_deg: float = 55
    stopped_speed_mps: float = .02
    hfov_deg: float = 72
    pan_limit_deg: float = 90
    tilt_min_deg: float = -20
    tilt_max_deg: float = 45
    gimbal_rate_deg_s: float = 20
    scan_span_deg: float = 25
    scan_period_s: float = 8
    scan_tilt_deg: float = 12
    class_priority: list[int] = field(default_factory=lambda: [0, 1, 2])

    def __post_init__(self):
        numeric = [v for v in asdict(self).values() if isinstance(v, (int, float))]
        if not all(math.isfinite(v) for v in numeric):
            raise ValueError('Policy numbers must be finite')
        if not 0 < self.range_min_m < self.range_max_m:
            raise ValueError('Invalid observation range')
        if not (isinstance(self.min_hits, int) and isinstance(self.window_frames, int)
                and 1 <= self.min_hits <= self.window_frames <= 30):
            raise ValueError('Invalid temporal confirmation window')
        if not (0 <= self.confidence <= 1 and 0 < self.association_iou <= 1):
            raise ValueError('Invalid confidence/association threshold')
        if not (20 <= self.hfov_deg <= 150 and self.tilt_min_deg < self.tilt_max_deg
                and 0 < self.pan_limit_deg <= 180 and 0 < self.max_obliquity_deg < 90):
            raise ValueError('Invalid optics/gimbal limits')
        if any(x <= 0 for x in [self.min_height_px, self.min_short_side_px,
                               self.max_frame_gap_s, self.stable_pan_tilt_deg,
                               self.alignment_tolerance_deg, self.gimbal_rate_deg_s,
                               self.scan_period_s]):
            raise ValueError('Positive observation thresholds required')
        if self.stopped_speed_mps < 0 or self.scan_span_deg < 0:
            raise ValueError('Invalid motion threshold')
        if sorted(self.class_priority) != [0, 1, 2]:
            raise ValueError('Class priorities must permute fire/firefighter/person')

    @classmethod
    def load(cls, path):
        return cls(**json.loads(Path(path).read_text(encoding='utf-8-sig')))


@dataclass
class Detection:
    class_id: int
    confidence: float
    bbox: tuple[float, float, float, float]
    detail_scale: float = 1.0  # min(1, inference downscale); upsampling adds no detail
    distance_m: float | None = None
    distance_source: str | None = None
    obliquity_deg: float | None = None

    @property
    def size(self):
        return ((self.bbox[2] - self.bbox[0]) * self.detail_scale,
                (self.bbox[3] - self.bbox[1]) * self.detail_scale)


def iou(a, b):
    intersect = max(0., min(a[2], b[2]) - max(a[0], b[0])) * max(0., min(a[3], b[3]) - max(a[1], b[1]))
    total = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - intersect
    return intersect / total if total > 0 else 0.


class Observer:
    def __init__(self, policy: Policy | None = None):
        self.policy = policy or Policy()
        self.reset()

    def reset(self):
        self.target = None
        self.history = deque(maxlen=self.policy.window_frames)
        self.last_timestamp = None
        self.last_frame_id = None
        self.anchor_gimbal = None
        self.scan_start = None
        self.scan_center = None
        self.track_serial = 0

    def clear_evidence(self):
        self.target = None
        self.history.clear()
        self.anchor_gimbal = None

    def update(self, detections, *, timestamp, frame_id, width, height,
               pan_deg=0., tilt_deg=0., speed_mps=None,
               safety_clear=False, at_observation_point=False):
        p = self.policy
        out = dict(state='hold', reason='invalid_input', ready=False,
                   request_reposition=False, vehicle_motion_authorized=False,
                   pan_deg=pan_deg, tilt_deg=tilt_deg, votes=0,
                   window=p.window_frames, target=None, frame_id=frame_id,
                   timestamp=timestamp, evidence_fresh=False)
        required = [timestamp, width, height, pan_deg, tilt_deg]
        if (not all(isinstance(v, (int, float)) and math.isfinite(v) for v in required)
                or width <= 0 or height <= 0 or not isinstance(frame_id, int)
                or abs(pan_deg) > p.pan_limit_deg
                or not p.tilt_min_deg <= tilt_deg <= p.tilt_max_deg):
            self.clear_evidence()
            return out
        duplicate = (self.last_timestamp is not None and
                     (timestamp <= self.last_timestamp or frame_id <= self.last_frame_id))
        gap = timestamp - self.last_timestamp if self.last_timestamp is not None else .05
        if gap > p.max_frame_gap_s:
            self.clear_evidence()
        if not duplicate:
            self.last_timestamp, self.last_frame_id = timestamp, frame_id
            out['evidence_fresh'] = True
        if (not safety_clear or speed_mps is None or not math.isfinite(speed_mps)
                or abs(speed_mps) > p.stopped_speed_mps or not at_observation_point):
            self.clear_evidence()
            out['reason'] = ('safety_hold' if not safety_clear else
                             'await_vehicle_stop' if speed_mps is None or not math.isfinite(speed_mps)
                             or abs(speed_mps) > p.stopped_speed_mps else 'navigate_to_observation_point')
            return out
        if (self.anchor_gimbal is not None and
                max(abs(pan_deg-self.anchor_gimbal[0]), abs(tilt_deg-self.anchor_gimbal[1])) > p.stable_pan_tilt_deg):
            self.clear_evidence()
        if duplicate:
            out['reason'] = 'duplicate_or_out_of_order_frame'
            return out  # State changes revoke evidence even without a new image.
        valid = []
        for d in detections:
            if (d.class_id not in p.class_priority or not math.isfinite(d.confidence)
                    or not p.confidence <= d.confidence <= 1 or len(d.bbox) != 4
                    or not all(math.isfinite(v) for v in d.bbox)
                    or not 0 <= d.bbox[0] < d.bbox[2] <= width
                    or not 0 <= d.bbox[1] < d.bbox[3] <= height
                    or not 0 < d.detail_scale <= 1):
                continue
            valid.append(d)
        selected = None
        if self.target:
            matches = [(iou(d.bbox, self.target.bbox), d) for d in valid if d.class_id == self.target.class_id]
            if matches:
                score, candidate = max(matches, key=lambda x: x[0])
                if score >= p.association_iou:
                    selected = candidate
        if selected is None and valid:
            self.clear_evidence()
            selected = min(valid, key=lambda d: (p.class_priority.index(d.class_id), -min(d.size), -d.confidence))
            self.track_serial += 1
        dt = min(gap, .2)
        step = p.gimbal_rate_deg_s * dt
        def command(pan, tilt):
            return (max(-p.pan_limit_deg, min(p.pan_limit_deg, pan_deg + max(-step, min(step, pan-pan_deg)))),
                    max(p.tilt_min_deg, min(p.tilt_max_deg, tilt_deg + max(-step, min(step, tilt-tilt_deg)))))
        if selected is None:
            self.history.append(False)
            # Do not authorize action while the target is missing, even if 3/5 votes remain.
            if not any(self.history):
                self.target = None
            if self.scan_start is None:
                self.scan_start, self.scan_center = timestamp, pan_deg
            phase = (timestamp-self.scan_start)/p.scan_period_s
            pan_goal = self.scan_center + p.scan_span_deg*math.sin(2*math.pi*phase)
            out['pan_deg'], out['tilt_deg'] = command(pan_goal, p.scan_tilt_deg)
            out.update(state='scan', reason='no_current_target', votes=sum(self.history))
            return out
        self.scan_start = self.scan_center = None
        self.target = selected
        if self.anchor_gimbal is None:
            self.anchor_gimbal = (pan_deg, tilt_deg)
        w, h = selected.size
        cx, cy = (selected.bbox[0]+selected.bbox[2])/2, (selected.bbox[1]+selected.bbox[3])/2
        focal = width / (2*math.tan(math.radians(p.hfov_deg)/2))
        ep = math.degrees(math.atan2(width/2-cx, focal))
        et = math.degrees(math.atan2(height/2-cy, focal))
        out['pan_deg'], out['tilt_deg'] = command(pan_deg+ep, tilt_deg+et)
        out['target'] = dict(**asdict(selected), track_id=self.track_serial,
                             effective_width_px=w, effective_height_px=h,
                             center_error_deg=[ep, et])
        # All votes refer to the same current class, image region and stable pointing.
        reason = None
        if max(abs(ep), abs(et)) > p.alignment_tolerance_deg:
            reason = 'align_target'
        elif selected.distance_m is None or not math.isfinite(selected.distance_m) or selected.distance_m <= 0 or not selected.distance_source:
            reason = 'await_associated_range'
        elif not p.range_min_m <= selected.distance_m <= p.range_max_m:
            reason = 'too_near' if selected.distance_m < p.range_min_m else 'too_far'
        elif selected.obliquity_deg is not None and (not math.isfinite(selected.obliquity_deg) or not 0 <= selected.obliquity_deg <= p.max_obliquity_deg):
            reason = 'oblique_view'
        elif h < p.min_height_px or min(w,h) < p.min_short_side_px:
            reason = 'insufficient_pixels'
        qualifies = reason is None
        self.history.append(qualifies)
        votes = sum(self.history)
        ready = qualifies and votes >= p.min_hits
        out.update(state='ready' if ready else 'confirm' if qualifies else 'align' if reason == 'align_target' else 'adjust_view',
                   reason='confirmed_current_target' if ready else reason or 'collect_fresh_frames',
                   ready=ready, votes=votes,
                   request_reposition=reason in ('too_near','too_far','oblique_view','insufficient_pixels'))
        return out
