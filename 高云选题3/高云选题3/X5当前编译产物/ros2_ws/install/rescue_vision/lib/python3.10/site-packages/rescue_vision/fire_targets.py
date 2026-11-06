"""Conservative flame identities and explicit selection, with no ROS/hardware I/O.

IDs are local to registry_session and never reused. Ambiguous spatial matches or
even one missed observation retire the affected identity. This intentionally
requires re-selection after occlusion instead of silently choosing another fire.
"""
from collections import OrderedDict
import math
import uuid

from .legacy_aiming import bbox_iou


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def frame_key(document):
    stamp = document.get('capture_stamp_ns')
    sequence = document.get('sequence')
    if (not isinstance(stamp, int) or isinstance(stamp, bool) or stamp < 0
            or not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 0):
        raise ValueError('Exact capture_stamp_ns and sequence integers are required')
    return stamp, sequence


class FireTargetRegistry:
    def __init__(self, *, max_age_s=.5, max_motion_px=45., max_tracks=32, min_observations=2,
                 target_class='fire'):
        if target_class not in ('fire', 'firefighter'):
            raise ValueError('unsupported_target_registry_class')
        self.target_class = target_class
        self.class_id = {'fire': 0, 'firefighter': 1}[target_class]
        self.label = {'fire': '火焰', 'firefighter': '消防员'}[target_class]
        if not 0 < max_age_s <= .5 or not 0 < max_motion_px <= 200 or not 1 <= max_tracks <= 64:
            raise ValueError('Invalid target tracking bounds')
        self.max_age_s, self.max_motion_px = max_age_s, max_motion_px
        self.max_tracks, self.min_observations = max_tracks, min_observations
        self.session = uuid.uuid4().hex
        self.next_id, self.generation = 1, 0
        self.tracks, self.selected = {}, None
        self.geometry = self.key = self.received = None
        self.enabled = False
        self._frame_valid = False
        self.reason = 'mission_not_fire'
        self.commands = OrderedDict()

    def clear(self, reason, *, tracks=True):
        if self.selected is not None:
            self.generation += 1
        self.selected = None
        self.reason = reason
        if tracks:
            self.tracks.clear()
            self._frame_valid = False

    def set_enabled(self, enabled):
        if enabled is not True:
            self.clear('mission_not_fire')
        self.enabled = enabled is True

    def expire(self, now):
        if self.received is None or not _finite(now) or not 0 <= now - self.received <= self.max_age_s:
            self.clear('detection_timeout')

    def update(self, detections, *, stamp_ns, sequence, frame_id, size, now, available=True):
        if (not isinstance(frame_id, str) or not frame_id or
                not isinstance(size, (list, tuple)) or len(size) != 2
                or any(not isinstance(x, int) or isinstance(x, bool) or x <= 0 for x in size)
                or not _finite(now) or not isinstance(detections, (list, tuple))
                or any(not isinstance(item, dict) for item in detections)):
            self.clear('invalid_frame'); return
        try:
            new_key = frame_key(dict(capture_stamp_ns=stamp_ns, sequence=sequence))
        except ValueError:
            self.clear('invalid_frame'); return
        geometry = (frame_id, tuple(size))
        if not self.enabled or available is not True:
            self.clear('mission_not_fire' if not self.enabled else 'detector_unavailable'); return
        if self.geometry is not None and geometry != self.geometry:
            self.clear('camera_geometry_changed')
        if self.key is not None:
            if new_key == self.key:
                return  # Repeated publication never refreshes a capture lease.
            if stamp_ns <= self.key[0] or sequence <= self.key[1]:
                self.clear('source_restart_or_out_of_order')
        self.expire(now)
        self.geometry, self.key, self.received = geometry, new_key, now
        width, height = size
        boxes = []
        for detection in detections:
            box = detection.get('bbox_xyxy', ())
            confidence = detection.get('confidence')
            if (detection.get('class_id') != self.class_id or not _finite(confidence) or not .4 <= confidence <= 1
                    or not isinstance(box, (list, tuple)) or len(box) != 4 or not all(_finite(x) for x in box)):
                continue
            x1, y1, x2, y2 = box
            if not (2 <= x1 < x2 <= width - 2 and 2 <= y1 < y2 <= height - 2
                    and min(x2-x1, y2-y1) >= 6):
                continue
            boxes.append(dict(bbox_xyxy=list(box), confidence=float(confidence)))
        if len(boxes) > self.max_tracks:
            self.clear('too_many_targets'); return
        # Require a one-to-one association graph, not a greedy nearest match.
        edges = {}
        for identifier, track in self.tracks.items():
            old = track['bbox_xyxy']; center = ((old[0]+old[2])/2, (old[1]+old[3])/2)
            edges[identifier] = [index for index, item in enumerate(boxes)
                if math.hypot((item['bbox_xyxy'][0]+item['bbox_xyxy'][2])/2-center[0],
                              (item['bbox_xyxy'][1]+item['bbox_xyxy'][3])/2-center[1]) <= self.max_motion_px
                and bbox_iou(old, item['bbox_xyxy']) >= .1]
        owners = {index: [identifier for identifier, matches in edges.items() if index in matches]
                  for index in range(len(boxes))}
        ambiguous = {index for index, matching in owners.items()
                     if len(matching) > 1 or any(len(edges[identifier]) > 1 for identifier in matching)}
        # Two substantially overlapping fresh boxes cannot start two safe IDs.
        for first in range(len(boxes)):
            for second in range(first + 1, len(boxes)):
                if bbox_iou(boxes[first]['bbox_xyxy'], boxes[second]['bbox_xyxy']) > .25:
                    ambiguous.update((first, second))
        updated = {}
        for index, item in enumerate(boxes):
            if index in ambiguous:
                continue
            matches = owners[index]
            if len(matches) == 1 and len(edges[matches[0]]) == 1:
                identifier = matches[0]
                observations = self.tracks[identifier]['observations'] + 1
            else:
                identifier = self.next_id; self.next_id += 1
                observations = 1
            updated[identifier] = dict(item, target_id=identifier, observations=observations,
                label=self.label + ' ' + str(identifier), class_name=self.target_class,
                selectable=observations >= self.min_observations)
        self.tracks = updated
        self._frame_valid = True
        if self.selected is not None and self.selected not in updated:
            self.clear('target_ambiguous' if ambiguous else 'selected_target_lost', tracks=False)
        elif self.selected is not None:
            self.reason = 'selected_target_visible'
        elif self.reason not in ('target_ambiguous', 'selected_target_lost'):
            self.reason = 'explicit_selection_required'

    def select(self, command, *, now, ros_now):
        self.expire(now)
        if not isinstance(command, dict):
            return False, 'invalid_selection_command'
        identifier = command.get('command_id')
        issued, ttl = command.get('issued_at'), command.get('ttl_s')
        if (not isinstance(identifier, str) or not 1 <= len(identifier) <= 128
                or not _finite(issued) or not _finite(ttl) or not 0 < ttl <= 5
                or not _finite(ros_now) or not 0 <= ros_now-issued <= ttl):
            return False, 'selection_command_expired_or_invalid'
        if identifier in self.commands:
            return False, 'duplicate_selection_command'
        self.commands[identifier] = True
        while len(self.commands) > 128:
            self.commands.popitem(last=False)
        if not self.enabled:
            return False, 'mission_not_fire'
        candidates = [item for item in self.tracks.values() if item['selectable']]
        selected = None
        if sum(key in command for key in ('target_id','ordinal','selector')) != 1:
            self.clear('selection_missing_or_ambiguous', tracks=False)
            return False, self.reason
        if 'ordinal' in command:
            number = command['ordinal']
            ordered = sorted(candidates, key=lambda item: (item['bbox_xyxy'][0]+item['bbox_xyxy'][2])/2)
            # "Second" always refers to the current selectable view, left to right.
            if type(number) is int and 1 <= number <= len(ordered):
                centers = [(item['bbox_xyxy'][0]+item['bbox_xyxy'][2])/2 for item in ordered]
                if all(abs(centers[number-1]-x)>1 for i,x in enumerate(centers) if i!=number-1):
                    selected = ordered[number-1]
        elif 'target_id' in command:
            number = command['target_id']
            if isinstance(number, int) and not isinstance(number, bool):
                selected = next((item for item in candidates if item['target_id'] == number), None)
        else:
            selector = command.get('selector')
            if selector == 'only' and len(candidates) == 1:
                selected = candidates[0]
            elif selector in ('leftmost', 'rightmost', 'largest') and candidates:
                def score(item):
                    x1, y1, x2, y2 = item['bbox_xyxy']
                    return (x1+x2)/2 if selector != 'largest' else (x2-x1)*(y2-y1)
                ordered = sorted(candidates, key=score, reverse=selector != 'leftmost')
                if len(ordered) == 1 or abs(score(ordered[0])-score(ordered[1])) > 1:
                    selected = ordered[0]
        if selected is None:
            self.clear('selection_missing_or_ambiguous', tracks=False)
            return False, self.reason
        self.selected = selected['target_id']
        self.generation += 1
        self.reason = 'explicit_target_selected'
        return True, self.reason

    def snapshot(self, now):
        self.expire(now)
        frame, size = self.geometry if self.geometry is not None else ('', (0, 0))
        stamp_ns, sequence = self.key if self.key is not None else (0, 0)
        common = dict(registry_session=self.session, capture_stamp_ns=stamp_ns,
            target_class=self.target_class,
            capture_stamp=stamp_ns/1e9, sequence=sequence, coordinate_frame=frame,
            image_size_wh=list(size), physical_eye='left', selection_generation=self.generation,
            frame_valid=bool(self.enabled and self._frame_valid))
        selection = dict(common, valid=self.selected is not None, target_id=self.selected,
            bbox_xyxy=self.tracks[self.selected]['bbox_xyxy'] if self.selected is not None else None,
            reason=self.reason)
        ordered = sorted(self.tracks.values(), key=lambda item: (item['bbox_xyxy'][0]+item['bbox_xyxy'][2])/2)
        ordinal = 0
        listed = []
        for item in ordered:
            ordinal += int(item['selectable'])
            listed.append(dict(item, ordinal=ordinal if item['selectable'] else None))
        targets = dict(common, valid=self.enabled and bool(self.tracks),
            targets=listed, selected_target_id=self.selected,
            reason=self.reason, identity_note='IDs are not reused; loss or ambiguity requires new selection')
        return targets, selection


class SelectionHistory:
    """Remember exact capture keys so slow stereo results keep their own target."""
    def __init__(self, maxlen=32, max_age_s=.5):
        self.maxlen, self.max_age_s = maxlen, max_age_s
        self.frames = OrderedDict()
        self.identity = None
        self.latest_session = None
        self.latest_generation = -1
        self.retired_sessions = set()

    def observe(self, document, now):
        if not isinstance(document, dict) or not _finite(now):
            self.frames.clear(); self.identity = None; return False
        session, generation = document.get('registry_session'), document.get('selection_generation')
        target = document.get('target_id')
        if (isinstance(session, str) and session and isinstance(generation, int)
                and not isinstance(generation, bool) and generation >= 0):
            if session in self.retired_sessions:
                return False
            if session == self.latest_session and generation < self.latest_generation:
                return False
            if session != self.latest_session:
                if self.latest_session is not None:
                    self.retired_sessions.add(self.latest_session)
                # A bounded terminal state requires restarting this small node
                # after an unrealistic number of source process restarts.
                if len(self.retired_sessions) > 64:
                    self.frames.clear(); self.identity = None; return True
                self.latest_session = session
            self.latest_generation = generation
        valid = (document.get('valid') is True and isinstance(session, str) and bool(session)
                 and isinstance(generation, int) and not isinstance(generation, bool) and generation > 0
                 and isinstance(target, int) and not isinstance(target, bool) and target > 0)
        identity = (session, generation, target) if valid else None
        changed = self.identity != identity
        if changed or not valid:
            self.frames.clear()
        self.identity = identity
        if valid:
            try:
                key = frame_key(document)
                if key not in self.frames:  # Timers do not extend a frame lease.
                    self.frames[key] = (dict(document), now)
                while len(self.frames) > self.maxlen:
                    self.frames.popitem(last=False)
            except ValueError:
                self.frames.clear(); self.identity = None
        return changed

    def lookup(self, *, stamp_ns, sequence, frame_id, size, now):
        value = self.frames.get((stamp_ns, sequence))
        if value is None:
            return None
        document, received = value
        if (not _finite(now) or not 0 <= now-received <= self.max_age_s
                or document.get('coordinate_frame') != frame_id
                or tuple(document.get('image_size_wh', ())) != tuple(size)
                or document.get('physical_eye') != 'left'):
            return None
        return document


def selected_detection(detections, selection):
    box = selection.get('bbox_xyxy') if isinstance(selection, dict) else None
    if not isinstance(box, (list, tuple)) or len(box) != 4 or not all(_finite(v) for v in box):
        return None
    matches = [item for item in detections if item.class_id == 0 and _finite(float(item.confidence))
               and .4 <= item.confidence <= 1
               and len(item.bbox_xyxy) == 4 and all(_finite(float(x)) for x in item.bbox_xyxy)
               and all(abs(float(a)-float(b)) <= .01 for a, b in zip(item.bbox_xyxy, box))]
    return matches[0] if len(matches) == 1 else None
