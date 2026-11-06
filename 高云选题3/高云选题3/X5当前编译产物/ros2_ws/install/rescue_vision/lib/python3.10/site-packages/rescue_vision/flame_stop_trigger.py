"""Qualify a new stop using post-start exposures from either image half."""
from .checkpoint_alignment import confident_fire, number


class FlameStopTrigger:
    def __init__(self, config=None):
        config = config or {}
        self.frames = config.get('stop_confirm_frames', 4)
        self.duration = config.get('stop_stable_duration_s', .45)
        self.band = config.get('stop_center_band_norm', [.2, .8])
        self.margin = config.get('stop_box_margin_norm', .02)
        self.min_size = config.get('stop_min_box_wh_norm', [.015, .025])
        self.threshold = max(.90, config.get('min_confidence', .90))
        if (type(self.frames) is not int or not 3 <= self.frames <= 12
                or not number(self.duration) or not .25 <= self.duration <= 1.5
                or not isinstance(self.band, (list, tuple)) or len(self.band) != 2
                or any(not number(v) for v in self.band) or not 0 <= self.band[0] < .5 < self.band[1] <= 1
                or not number(self.margin) or not 0 <= self.margin <= .1
                or not isinstance(self.min_size, (list, tuple)) or len(self.min_size) != 2
                or any(not number(v) or not 0 < v <= .1 for v in self.min_size)):
            raise ValueError('fire_stop_trigger_config_invalid')
        self.reset()

    def reset(self):
        self.running_since = self.capture = None
        self.rows = {}
        self.choice = None
        self.ready = False
        self.reason = 'fire_stop_waiting_navigation_running'

    def observe(self, document, now, eligible, *, running):
        if not running:
            self.reset()
            return
        if self.running_since is None:
            self.running_since = now
        stamp, size = document.get('capture_stamp'), document.get('image_size_wh')
        session = document.get('registry_session')
        if (document.get('frame_valid') is not True or not number(stamp) or not 0 <= now-stamp <= .6
                or not isinstance(session, str) or not session
                or not isinstance(size, (list, tuple)) or len(size) != 2
                or any(type(v) is not int or v <= 0 for v in size)
                or not isinstance(document.get('targets'), list)):
            self.rows = {}; self.choice = None; self.ready = False
            self.reason = 'fire_stop_waiting_fresh_exposure'
            return
        if stamp <= self.running_since:
            self.rows = {}; self.choice = None; self.ready = False
            self.reason = 'fire_stop_waiting_post_start_exposure'
            return
        if self.capture is not None and stamp <= self.capture:
            return
        old, rows = self.rows, {}
        for target in document.get('targets', []):
            if not isinstance(target, dict):
                continue
            identifier = target.get('target_id')
            if type(identifier) is not int or identifier <= 0:
                continue
            identity = (session, identifier)
            box = target.get('bbox_xyxy')
            if (identity not in eligible or not confident_fire(target, self.threshold)
                    or not isinstance(box, (list, tuple)) or len(box) != 4
                    or any(not number(v) for v in box)):
                continue
            x1, y1, x2, y2 = [value/size[i%2] for i, value in enumerate(box)]
            center = (x1+x2)/2.
            if not (self.margin <= x1 < x2 <= 1-self.margin and self.margin <= y1 < y2 <= 1-self.margin
                    and x2-x1 >= self.min_size[0] and y2-y1 >= self.min_size[1]
                    and self.band[0] <= center <= self.band[1]):
                continue
            previous = old.get(identity)
            area = (x2-x1)*(y2-y1)
            continuous = bool(previous and stamp-previous['last'] <= .4
                and abs(center-previous['center']) <= .12 and .4 <= area/previous['area'] <= 2.5)
            rows[identity] = dict(first=previous['first'] if continuous else stamp, last=stamp,
                hits=previous['hits']+1 if continuous else 1, center=center, area=area)
        self.rows, self.capture = rows, stamp
        qualified = [identity for identity, row in rows.items()
                     if row['hits'] >= self.frames and stamp-row['first'] >= self.duration]
        pool = qualified or list(rows)
        self.choice = min(pool, key=lambda identity: abs(rows[identity]['center']-.5)) if pool else None
        self.ready = bool(qualified)
        self.reason = ('fire_stop_stable_target_ready' if qualified else
                       'fire_stop_confirming_stable_target' if rows else 'fire_stop_waiting_suitable_position')

    def snapshot(self):
        row = self.rows.get(self.choice, {})
        return dict(ready=self.ready, reason=self.reason,
            target_id=self.choice[1] if self.choice else None,
            registry_session=self.choice[0] if self.choice else None,
            confirm_frames=self.frames, stable_duration_s=self.duration,
            center_band_norm=list(self.band), box_margin_norm=self.margin,
            min_box_wh_norm=list(self.min_size), hits=row.get('hits', 0),
            stable_elapsed_s=max(0., row.get('last', 0)-row.get('first', 0)),
            semantics='both_halves_complete_boxes_after_navigation_started')
