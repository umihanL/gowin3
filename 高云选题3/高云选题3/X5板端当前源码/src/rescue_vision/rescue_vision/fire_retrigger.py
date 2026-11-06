"""Task-local suppression of processed flame tracks and departure retriggers.

Image overlap links a brief track interruption. Longer gaps need three fresh
unique matches at a bounded same-route viewpoint; none of these links establish
a world-space flame count, extinguishing confirmation, or range estimate.
"""
from .checkpoint_alignment import confident_fire, number


class FireRetriggerGuard:
    MAX_IDENTITIES = 4096
    MAX_ANCHORS = 32
    TRACK_GAP_S = .6
    SAME_VIEW_GAP_S = 4.
    MAX_RECENT_BOXES = 128

    def __init__(self, config=None):
        config = config or {}
        self.cooldown_s = config.get('fire_retrigger_cooldown_s', 3.)
        if not number(self.cooldown_s) or not .5 <= self.cooldown_s <= 30.:
            raise ValueError('fire_retrigger_cooldown_invalid')
        self.threshold = config.get('min_confidence', .90)
        self.reset()

    def reset(self):
        self.processed, self.anchors, self.boxes = {}, {}, {}
        self.recent_boxes, self.anchor_meta, self.alias_probes = {}, {}, {}
        self.exempt = set()
        self.capture = None
        self.cooldown_until = 0.
        self.alias_count = 0
        self.same_view_alias_count = 0
        self.truncated = False

    def remaining(self, now):
        return max(0., self.cooldown_until - now)

    def _remember(self, identity, status='completed'):
        if identity not in self.processed and len(self.processed) >= self.MAX_IDENTITIES:
            del self.processed[next(iter(self.processed))]
            self.truncated = True
        self.processed[identity] = status

    def mark_processed(self, identity, now, start_cooldown=True, status='completed'):
        if not identity:
            return
        if status not in ('completed', 'skipped'):
            raise ValueError('invalid_processed_fire_status')
        self._remember(identity, status)
        if start_cooldown:
            self.begin_cooldown(now)
        observed = self.recent_boxes.get(identity)
        if observed and 0 <= now-observed['stamp'] <= (self.SAME_VIEW_GAP_S if observed['context'] else self.TRACK_GAP_S):
            if identity not in self.anchors and len(self.anchors) >= self.MAX_ANCHORS:
                oldest = next(iter(self.anchors))
                del self.anchors[oldest]
                self.anchor_meta.pop(oldest, None)
                self.truncated = True
            self.anchors[identity] = (observed['box'], observed['stamp'])
            self.anchor_meta[identity] = observed

    def begin_cooldown(self, now):
        self.cooldown_until = now + self.cooldown_s

    @staticmethod
    def _same_short_track(old, new):
        # A strict overlap and size check avoids turning a general cooldown
        # into permanent suppression of unrelated objects in a broad image area.
        intersection = max(0., min(old[2], new[2]) - max(old[0], new[0])) * max(0., min(old[3], new[3]) - max(old[1], new[1]))
        a, b = (old[2] - old[0]) * (old[3] - old[1]), (new[2] - new[0]) * (new[3] - new[1])
        return .67 <= b / a <= 1.5 and intersection / (a + b - intersection) >= .55

    @staticmethod
    def _view_context(context, now, capture):
        if not isinstance(context, dict):
            return None
        cell, heading = context.get('pose_cell'), context.get('pose_heading_deg')
        stamp, progress = context.get('stamp'), context.get('route_progress_mm')
        if (context.get('valid') is not True or context.get('status_fresh') is not True or
                context.get('pose_valid') is not True or context.get('retained_route') is not True or
                not number(stamp) or not 0 <= now-stamp <= .3 or not 0 <= now-capture <= .3 or
                not isinstance(context.get('command_id'), str) or not context['command_id'] or
                type(context.get('route_generation')) is not int or type(context.get('task_id')) is not int or
                not isinstance(cell, (list, tuple)) or len(cell) != 2 or any(not number(v) for v in cell) or
                not number(heading) or not number(progress)):
            return None
        return dict(route=(context['command_id'], context['task_id'], context['route_generation']),
                    cell=tuple(cell), heading=heading, progress=progress)

    @staticmethod
    def _same_view(old, current, view):
        a, b = old.get('context'), current
        return bool(a and b and old['view'] == view and a['route'] == b['route'] and
            sum((a['cell'][i]-b['cell'][i])**2 for i in range(2)) <= 4. and
            abs((a['heading']-b['heading']+180.) % 360.-180.) <= 5. and
            abs(a['progress']-b['progress']) <= 60.)

    def observe(self, document, now, exempt=(), context=None):
        self.exempt = set(exempt)
        stamp, session, size = document.get('capture_stamp'), document.get('registry_session'), document.get('image_size_wh')
        if (document.get('frame_valid') is not True or not number(stamp) or not 0 <= now - stamp <= self.TRACK_GAP_S or
                not isinstance(session, str) or not session or not isinstance(size, (list, tuple)) or len(size) != 2 or
                any(type(v) is not int or v <= 0 for v in size) or not isinstance(document.get('targets'), list) or
                self.capture is not None and stamp <= self.capture):
            return False
        boxes = {}
        for target in document['targets']:
            if not isinstance(target, dict):
                continue
            identity, box = (session, target.get('target_id')), target.get('bbox_xyxy')
            if (type(identity[1]) is not int or identity[1] <= 0 or target.get('selectable') is not True or
                    not confident_fire(target, self.threshold) or not isinstance(box, (list, tuple)) or len(box) != 4 or
                    any(not number(v) for v in box) or not 0 <= box[0] < box[2] <= size[0] or not 0 <= box[1] < box[3] <= size[1]):
                continue
            boxes[identity] = tuple(value / size[i % 2] for i, value in enumerate(box))
        view = (document.get('coordinate_frame'), tuple(size))
        current_context = self._view_context(context, now, stamp) if isinstance(view[0], str) and view[0] else None
        self.recent_boxes = {key: value for key, value in self.recent_boxes.items()
                            if 0 <= stamp-value['stamp'] <= self.SAME_VIEW_GAP_S}
        for identity, box in boxes.items():
            if identity not in self.recent_boxes and len(self.recent_boxes) >= self.MAX_RECENT_BOXES:
                del self.recent_boxes[next(iter(self.recent_boxes))]
                self.truncated = True
            self.recent_boxes[identity] = dict(box=box, stamp=stamp, context=current_context, view=view)
        anchors, metadata, probes = {}, {}, {}
        unmatched = {}
        for identity, (box, last_seen) in self.anchors.items():
            if identity in boxes:
                anchors[identity] = (boxes[identity], stamp)
                metadata[identity] = self.recent_boxes[identity]
            elif 0 <= stamp-last_seen <= self.TRACK_GAP_S or (
                    0 <= stamp-last_seen <= self.SAME_VIEW_GAP_S and
                    self.anchor_meta.get(identity, {}).get('context')):
                unmatched[identity] = (box, last_seen)
        # Only unique one-to-one matches in the same registry session qualify.
        # Existing unprocessed queue entries are never absorbed into old tracks.
        links = {old: [new for new, box in boxes.items() if new[0] == old[0] and
                      new not in self.processed and new not in self.exempt and
                      self.anchor_meta.get(old, {}).get('view') == view and
                      (stamp-anchor[1] <= self.TRACK_GAP_S or
                       self._same_view(self.anchor_meta.get(old, {}), current_context, view)) and
                      self._same_short_track(anchor[0], box)] for old, anchor in unmatched.items()}
        for old, candidates in links.items():
            if len(candidates) == 1 and sum(candidates[0] in values for values in links.values()) == 1:
                identity = candidates[0]
                short = stamp-unmatched[old][1] <= self.TRACK_GAP_S
                probe = self.alias_probes.get(old)
                exposure = (document.get('capture_stamp_ns'), document.get('sequence'))
                if not probe or probe['identity'] != identity or stamp-probe['stamp'] > self.TRACK_GAP_S:
                    probe = dict(identity=identity, hits=0, stamp=stamp, exposures=set())
                if not any(type(v) is int for v in exposure) or exposure not in probe['exposures']:
                    probe['hits'] += 1
                    probe['exposures'].add(exposure)
                probe['stamp'] = stamp
                if short or probe['hits'] >= 3:
                    self._remember(identity, self.processed.get(old, 'completed'))
                    self.alias_count += 1
                    self.same_view_alias_count += int(not short)
                    anchors[identity] = (boxes[identity], stamp)
                    metadata[identity] = self.recent_boxes[identity]
                    continue
                probes[old] = probe
                anchors[old] = unmatched[old]
                metadata[old] = self.anchor_meta.get(old, {})
            else:
                anchors[old] = unmatched[old]
                metadata[old] = self.anchor_meta.get(old, {})
        self.anchors, self.anchor_meta, self.alias_probes = anchors, metadata, probes
        self.boxes, self.capture = boxes, stamp
        return True

    def allows(self, identity, now):
        return identity not in self.processed and (not self.remaining(now) or identity in self.exempt)

    def has_deferred(self, now, blocked=()):
        """A stationary checkpoint should wait only for a visible new candidate."""
        return bool(self.remaining(now) and self.capture is not None and 0 <= now - self.capture <= self.TRACK_GAP_S and
            any(identity not in self.processed and identity not in self.exempt and identity not in blocked
                for identity in self.boxes))

    def filtered(self, document, now, blocked=()):
        session, targets = document.get('registry_session'), document.get('targets')
        if not isinstance(targets, list) or not isinstance(session, str):
            return document
        blocked = set(blocked)
        return dict(document, targets=[target for target in targets if isinstance(target, dict) and type(target.get('target_id')) is int and
            (session, target.get('target_id')) not in blocked and self.allows((session, target.get('target_id')), now)])

    def snapshot(self, now):
        return dict(cooldown_s=self.cooldown_s, cooldown_remaining_s=round(self.remaining(now), 3),
            processed_identity_count=len(self.processed), short_track_alias_count=self.alias_count,
            completed_identity_count=sum(value == 'completed' for value in self.processed.values()),
            skipped_identity_count=sum(value == 'skipped' for value in self.processed.values()),
            same_view_alias_count=self.same_view_alias_count, same_view_gap_s=self.SAME_VIEW_GAP_S,
            queued_targets_exempt=True, truncated=self.truncated,
            identity_semantics='task_track_ids_and_bounded_same_view_aliases_not_physical_total')
