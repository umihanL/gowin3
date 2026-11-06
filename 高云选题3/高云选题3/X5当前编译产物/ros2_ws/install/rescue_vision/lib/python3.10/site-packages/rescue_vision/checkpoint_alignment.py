"""Fresh multi-frame identities for bounded side-camera checkpoint alignment.

The image centre line indicates lateral alignment only. It supplies neither a
distance-to-fire estimate nor permission to energize a pump or move a wheel.
"""
import math


MIN_FIRE_CONFIDENCE = .90


def confident_fire(target, threshold=MIN_FIRE_CONFIDENCE):
    """Semantic search accepts a detector score, never a missing/boolean score."""
    confidence = target.get('confidence') if isinstance(target, dict) else None
    return number(confidence) and threshold <= confidence <= 1.


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


class CheckpointAlignment:
    def __init__(self, config):
        config = dict(config)
        for source, destination in dict(stable_frames='confirm_frames', settle_s='dwell_s',
                max_move_s='leg_timeout_s', capture_ttl_s='frame_ttl_s').items():
            if source in config:
                config[destination] = config[source]
        self.enabled = config.get('enabled', True) is True
        self.side = config.get('forward_image_side', 'unset')
        if self.side not in ('left', 'right', 'unset'):
            raise ValueError('checkpoint_forward_image_side_invalid')
        self.config = dict(confirm_frames=3, center_confirm_frames=3, observation_s=1.5,
            center_tolerance_norm=.05, dwell_s=2., frame_ttl_s=.6, target_lost_s=.6,
            leg_max_cm=30., max_advance_cm=60., leg_timeout_s=12., checkpoint_timeout_s=180., max_targets=16,
            min_confidence=MIN_FIRE_CONFIDENCE, feedback_wait_s=3.)
        for key in self.config:
            if key in config:
                self.config[key] = config[key]
        limits = {'confirm_frames': (3, 12), 'center_confirm_frames': (3, 12),
            'observation_s': (.5, 5), 'center_tolerance_norm': (.01, .12), 'dwell_s': (2, 10),
            'frame_ttl_s': (.1, .6), 'target_lost_s': (.2, 1.5), 'leg_max_cm': (10, 40),
            'max_advance_cm': (10, 100), 'leg_timeout_s': (3, 40), 'checkpoint_timeout_s': (20, 300),
            'max_targets': (1, 32), 'min_confidence': (MIN_FIRE_CONFIDENCE, 1.), 'feedback_wait_s': (1., 5.)}
        for key, (low, high) in limits.items():
            if not number(self.config[key]) or not low <= self.config[key] <= high:
                raise ValueError('checkpoint_alignment_invalid_' + key)
        if any(type(self.config[key]) is not int for key in ('confirm_frames', 'center_confirm_frames', 'max_targets')):
            raise ValueError('checkpoint_alignment_frame_counts_must_be_integers')
        self.active = False
        self.waypoint_id = None
        self.phase, self.reason = 'IDLE', 'not_started'
        self.started_at = self.dwell_at = self.stopped_at = self.move_at = None
        self.queue, self.seen, self.current_frame = [], {}, {}
        self.capture = self.last_frame_at = None
        self.frame_count = self.center_hits = 0
        self.current_index = None
        self.total_advance_cm = 0.
        self.pending_leg = None
        self.stop_reason = None
        self.queue_truncated = False
        self.feedback_wait_at = None
        self.feedback_wait_reason = None

    @property
    def current(self):
        return self.queue[self.current_index] if self.current_index is not None else None

    @property
    def identity(self):
        row = self.current
        return (row['registry_session'], row['target_id']) if row else None

    def begin(self, now, waypoint_id):
        self.active = True
        self.waypoint_id = waypoint_id
        self.phase, self.reason = 'CHECKPOINT_COLLECT', 'collecting_stable_checkpoint_targets'
        self.started_at = now
        self.dwell_at = self.stopped_at = self.move_at = None
        self.queue, self.seen, self.current_frame = [], {}, {}
        self.capture = self.last_frame_at = None
        self.frame_count = self.center_hits = 0
        self.current_index = None
        self.total_advance_cm = 0.
        self.pending_leg = None
        self.stop_reason = None
        self.queue_truncated = False
        self.feedback_wait_at = None
        self.feedback_wait_reason = None

    def observe(self, document, now):
        stamp, session = document.get('capture_stamp'), document.get('registry_session')
        size = document.get('image_size_wh')
        if (document.get('frame_valid') is not True or not number(stamp) or
                not 0 <= now - stamp <= self.config['frame_ttl_s'] or
                not isinstance(session, str) or not session or
                not isinstance(size, (list, tuple)) or len(size) != 2 or
                any(type(v) is not int or v <= 0 for v in size) or
                not isinstance(document.get('targets'), list)):
            return False
        if self.capture is not None and stamp <= self.capture:
            return False
        continuous = self.capture is not None and stamp - self.capture <= self.config['frame_ttl_s']
        self.capture, self.last_frame_at = stamp, now
        self.frame_count += 1
        rows = {}
        for target in document['targets']:
            box = target.get('bbox_xyxy') if isinstance(target, dict) else None
            identity = (session, target.get('target_id')) if isinstance(target, dict) else None
            if (not identity or type(identity[1]) is not int or identity[1] <= 0 or target.get('selectable') is not True or
                    not confident_fire(target, self.config['min_confidence']) or
                    not isinstance(box, (list, tuple)) or len(box) != 4 or any(not number(v) for v in box) or
                    not 0 <= box[0] < box[2] <= size[0] or not 0 <= box[1] < box[3] <= size[1]):
                continue
            if identity in rows:
                continue
            center = (box[0] + box[2]) / (2. * size[0])
            previous = self.seen.get(identity, {})
            count = previous.get('hits', 0) + 1 if continuous and identity in self.current_frame else 1
            rows[identity] = dict(registry_session=session, target_id=identity[1], center_x_norm=center,
                                  confidence=float(target['confidence']), hits=count, last_seen=stamp)
        self.seen = rows
        self.current_frame = rows
        if self.current:
            observation = rows.get(self.identity)
            if observation:
                self.current.update(center_x_norm=observation['center_x_norm'],
                                    confidence=observation['confidence'], last_seen=stamp)
            centered = observation is not None and abs(observation['center_x_norm'] - .5) <= self.config['center_tolerance_norm']
            self.center_hits = self.center_hits + 1 if continuous and centered else 1 if centered else 0
        return True

    def freeze_queue(self, now):
        if (self.frame_count < self.config['confirm_frames'] or now - self.started_at < self.config['observation_s'] or
                self.capture is None or not 0 <= now - self.capture <= self.config['frame_ttl_s']):
            return False
        stable = [row for row in self.current_frame.values() if row['hits'] >= self.config['confirm_frames']]
        if stable and self.side == 'unset':
            raise ValueError('checkpoint_forward_image_side_unset')
        tolerance = self.config['center_tolerance_norm']
        stable = [row for row in stable if abs(row['center_x_norm'] - .5) <= tolerance or
                  (row['center_x_norm'] > .5 if self.side == 'right' else row['center_x_norm'] < .5)]
        stable.sort(key=lambda row: (abs(row['center_x_norm'] - .5), row['target_id']))
        self.queue_truncated = len(stable) > self.config['max_targets']
        self.queue = [dict(row, status='queued') for row in stable[:self.config['max_targets']]]
        self.phase, self.reason = 'ALIGN_SELECT', 'checkpoint_queue_ready'
        return True

    def select(self):
        if self.current is not None:
            return self.current
        self.current_index = next((i for i, row in enumerate(self.queue) if row['status'] == 'queued'), None)
        self.center_hits = 0
        self.stopped_at = self.dwell_at = None
        if self.current:
            self.current['status'] = 'aligning'
        return self.current

    def observation(self, now):
        row = self.current_frame.get(self.identity)
        return row if row and 0 <= now - row['last_seen'] <= self.config['frame_ttl_s'] else None

    def relation(self, now):
        row = self.observation(now)
        if not row:
            return 'missing'
        error = row['center_x_norm'] - .5
        if abs(error) <= self.config['center_tolerance_norm']:
            return 'center'
        if self.side == 'unset':
            return 'unconfigured'
        return 'forward' if (error > 0) == (self.side == 'right') else 'passed'

    def finish_target(self, status, reason):
        if self.current:
            self.current.update(status=status, reason=reason)
        self.current_index = None
        self.center_hits = 0
        self.stopped_at = self.dwell_at = self.move_at = None
        self.phase, self.reason = 'ALIGN_SELECT', reason

    def reserve_leg(self, guard, now):
        reserved = float(guard['max_advance_cm'])
        self.total_advance_cm += reserved
        self.pending_leg = dict(start_cell=list(guard['start_cell']), heading_deg=float(guard['heading_deg']),
            heading_tolerance_deg=float(guard.get('heading_tolerance_deg', 15.)),
            lateral_tolerance_cm=float(guard.get('lateral_tolerance_cm', 6.)),
            reserved_cm=reserved, issued_at=now, settled=False)

    def settle_leg(self, navigation, now):
        """Refund unused reservation once, using independently confirmed stop.

        The approved path is monotonic and within 15 degrees of the initial
        heading. Projection/cos(15) bounds its length; 4 cm covers quantized
        start/end cells. Missing/out-of-corridor measurements get no refund.
        """
        leg = self.pending_leg
        if not leg or leg['settled']:
            return
        cell = navigation.get('pose_cell')
        if (navigation.get('pose_valid') is not True or navigation.get('stopped') is not True or
                navigation.get('phase') not in ('paused', 'arrived') or
                not isinstance(cell, (list, tuple)) or len(cell) != 2 or any(type(v) is not int for v in cell) or
                not number(navigation.get('stamp')) or not leg['issued_at'] < navigation['stamp'] <= now):
            return
        leg['settled'] = True
        angle = math.radians(leg['heading_deg'])
        dx, dy = [2. * (cell[i] - leg['start_cell'][i]) for i in range(2)]
        longitudinal = dx * math.cos(angle) - dy * math.sin(angle)
        lateral = abs(dx * math.sin(angle) + dy * math.cos(angle))
        if longitudinal < -4. or lateral > leg['lateral_tolerance_cm'] + 4. or not 0 <= leg['heading_tolerance_deg'] <= 15.:
            leg['settled_cm'] = leg['reserved_cm']
            return
        bound = (max(0., longitudinal) + 4.) / math.cos(math.radians(leg['heading_tolerance_deg']))
        consumed = min(leg['reserved_cm'], math.ceil(bound * 100.) / 100.)
        self.total_advance_cm -= leg['reserved_cm'] - consumed
        leg['settled_cm'] = consumed
        leg['stopped_pose_cell'] = list(cell)

    def snapshot(self, now):
        row = self.observation(now) if self.current else None
        processed = sum(item['status'] in ('completed', 'lost', 'passed', 'failed', 'skipped') for item in self.queue)
        return dict(enabled=self.enabled, active=self.active, phase=self.phase, reason=self.reason,
            forward_image_side=self.side, waypoint_id=self.waypoint_id, queue=[dict(item) for item in self.queue],
            current_target_id=self.current['target_id'] if self.current else None,
            current_registry_session=self.current['registry_session'] if self.current else None,
            confirmed_count=len(self.queue), processed_count=processed, remaining_count=len(self.queue) - processed,
            center_error_norm=row['center_x_norm'] - .5 if row else None,
            dwell_remaining_s=max(0., self.config['dwell_s'] - (now - self.dwell_at)) if self.dwell_at is not None else None,
            total_advance_cm=round(self.total_advance_cm, 2), max_advance_cm=self.config['max_advance_cm'],
            pending_leg=dict(self.pending_leg) if self.pending_leg else None,
            center_tolerance_norm=self.config['center_tolerance_norm'], confirm_frames=self.config['confirm_frames'],
            center_confirm_frames=self.config['center_confirm_frames'], dwell_s=self.config['dwell_s'],
            min_confidence=self.config['min_confidence'], feedback_wait_reason=self.feedback_wait_reason,
            feedback_wait_elapsed_s=now - self.feedback_wait_at if self.feedback_wait_at is not None else None,
            queue_truncated=self.queue_truncated, alignment_semantics='side_view_centerline_only')
