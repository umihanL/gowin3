"""A bounded batch of side-view flames, including both sides of image centre.

Only image observations live here. Signed wheel movement and restoring the
retained route remain the responsibility of the FPGA navigation owner.
An explicit spray batch confirms stationary targets without wheel centering;
the fire controller then aims the fixed camera's separate nozzle gimbal.
"""
from .checkpoint_alignment import MIN_FIRE_CONFIDENCE, confident_fire, number


class FlameCentering:
    def __init__(self, config=None):
        config = config or {}
        self.side = config.get('forward_image_side', 'right')
        if self.side not in ('right', 'left', 'unset'):
            raise ValueError('fire_centering_forward_image_side_required')
        self.tolerance = config.get('centering_tolerance_norm', .05)
        if not number(self.tolerance) or not .01 <= self.tolerance <= .08:
            raise ValueError('fire_centering_tolerance_invalid')
        self.threshold = max(MIN_FIRE_CONFIDENCE, config.get('min_confidence', MIN_FIRE_CONFIDENCE))
        self.confirm_frames = 3
        self.dwell_s = config.get('centering_dwell_s', 1.)
        if not number(self.dwell_s) or not 1. <= self.dwell_s <= 5.:
            raise ValueError('fire_centering_dwell_requires_at_least_one_second')
        self.ttl_s = .6
        self.max_targets = 32
        self.collect_wait_s = 3.
        self.intake_window_s = 6.
        self.confirm_window_s = 1.2
        self.reset()

    def reset(self):
        self.active = False
        self.phase, self.reason = 'IDLE', 'not_requested'
        self.queue = []
        self.current_index = None
        self.capture = self.capture_ns = self.sequence = self.coordinate_frame = None
        self.image_size_wh = None
        self.rows = {}
        self.started_at = self.phase_at = None
        self.center_hits = 0
        self.frame_count = 0
        self.post_stop_frame_count = 0
        self.dwell_at = self.dwell_capture = None
        self.move_direction = 'stopped'
        self.queue_truncated = False
        self.batch_at_terminal = False
        self.gimbal_only = False
        self.frozen = False
        self.collect_reason = 'collecting_post_stop_confirmations'
        self.last_high_confidence_at = None
        self.batch_counter = 0
        self.collection_closed_at = None
        self.stationary_since = None
        self.abandoned_reason = None

    def begin(self, now, terminal=False, seed=None, *, gimbal_only=False):
        self.reset()
        self.active = True
        self.started_at = self.phase_at = now
        self.stationary_since = now
        self.phase, self.reason = 'CENTER_COLLECT', 'collecting_all_visible_stable_flames'
        self.batch_at_terminal = terminal
        self.gimbal_only = gimbal_only is True
        if isinstance(seed, dict):
            # A trigger is a candidate only. It never contributes a stopped
            # confirmation or dwell exposure after the car has braked.
            saved = self._valid_rows(seed, now, max_age_s=5.)
            for identity, observation in saved.items():
                self._new_candidate(identity, observation, seed.get('image_size_wh'),
                                    seed.get('coordinate_frame'), now, seeded=True)

    @property
    def current(self):
        return self.queue[self.current_index] if self.current_index is not None else None

    @property
    def identity(self):
        row = self.current
        return (row['registry_session'], row['target_id']) if row else None

    @property
    def pending_identities(self):
        identities=set()
        for row in self.queue:
            if row['status'] in ('candidate', 'queued', 'aligning', 'centered'):
                identities.update(tuple(identity) for identity in row['observed_id_aliases'])
                identities.update(tuple(identity) for identity in row.get('_rebind_ids',[]))
        return identities

    def _valid_rows(self, document, now, max_age_s=None):
        stamp,session,size=(document.get(key) for key in ('capture_stamp','registry_session','image_size_wh'))
        if (document.get('frame_valid') is not True or not number(stamp) or not 0<=now-stamp<=(self.ttl_s if max_age_s is None else max_age_s)
                or not isinstance(session,str) or not session or not isinstance(size,(list,tuple)) or len(size)!=2
                or any(type(value) is not int or value<=0 for value in size)
                or not isinstance(document.get('targets'),list)):
            return {}
        rows={}
        for item in document['targets']:
            if not isinstance(item,dict): continue
            identity,box=(session,item.get('target_id')),item.get('bbox_xyxy')
            if (type(identity[1]) is not int or identity[1]<=0 or item.get('selectable') is not True
                    or not confident_fire(item,self.threshold) or not isinstance(box,(list,tuple)) or len(box)!=4
                    or any(not number(value) for value in box)
                    or not 0<=box[0]<box[2]<=size[0] or not 0<=box[1]<box[3]<=size[1]):
                continue
            rows[identity]=dict(registry_session=session,target_id=identity[1],bbox_xyxy=list(box),
                center_x_norm=(box[0]+box[2])/(2.*size[0]),confidence=float(item['confidence']),last_seen=stamp,hits=1)
        return rows

    def _new_candidate(self, identity, observation, size, frame, now, seeded=False):
        if len(self.queue)>=self.max_targets:
            self.queue_truncated=True
            return None
        self.batch_counter+=1
        row=dict(observation,status='candidate',batch_id='batch-%d'%self.batch_counter,
            anchor_bbox_xyxy=list(observation['bbox_xyxy']),anchor_image_size_wh=list(size),
            anchor_coordinate_frame=frame,original_identity=list(identity),original_target_id=identity[1],
            observed_id_aliases=[list(identity)],post_stop_stamps=[],last_candidate_stamp=observation['last_seen'],
            created_at=now,seeded_from_trigger=seeded)
        self.queue.append(row)
        return row

    @staticmethod
    def _near_box(old, new, size):
        a=(old[2]-old[0])*(old[3]-old[1]);b=(new[2]-new[0])*(new[3]-new[1])
        overlap=max(0.,min(old[2],new[2])-max(old[0],new[0]))*max(0.,min(old[3],new[3])-max(old[1],new[1]))
        return (.4<=b/a<=2.5 and overlap/(a+b-overlap)>=.25
            and abs((old[0]+old[2]-new[0]-new[2])/(2.*size[0]))<=.08)

    def _associate_batch(self, now, stationary, at_anchor):
        live=[row for row in self.queue if row['status'] in ('candidate','queued','aligning','centered')]
        occupied={tuple(identity):row['batch_id'] for row in self.queue for identity in row['observed_id_aliases']}
        edges={}
        for row in live:
            same=(row['registry_session'],row['target_id'])
            edges[row['batch_id']]=[same] if same in self.rows else [identity for identity,obs in self.rows.items()
                if identity[0]==row['registry_session'] and occupied.get(identity,row['batch_id'])==row['batch_id']
                and stationary and self._near_box(row['bbox_xyxy'],obs['bbox_xyxy'],self.image_size_wh)]
        owners={identity:[row for row in live if identity in edges[row['batch_id']]] for identity in self.rows}
        consumed=set()
        for row in live:
            matches=edges[row['batch_id']]
            if len(matches)!=1 or len(owners[matches[0]])!=1:
                if matches:
                    row.pop('_rebind_stamps',None);row.pop('_rebind_ids',None)
                continue
            identity=matches[0];obs=self.rows[identity];consumed.add(identity)
            same=identity==(row['registry_session'],row['target_id'])
            if not same and not stationary: continue
            row['last_candidate_stamp']=self.capture
            if stationary and self.capture>max(self.started_at,self.stationary_since):
                stamps=[stamp for stamp in row['post_stop_stamps'] if self.capture-stamp<=self.confirm_window_s]
                if self.capture not in stamps: stamps.append(self.capture)
                row['post_stop_stamps']=stamps
            if not same:
                if self.capture<=self.stationary_since: continue
                stamps=[stamp for stamp in row.get('_rebind_stamps',[]) if self.capture-stamp<=self.confirm_window_s]
                if stamps and self.capture-stamps[-1]>self.ttl_s: stamps=[]
                if self.capture not in stamps: stamps.append(self.capture)
                row['_rebind_stamps']=stamps
                aliases=row.setdefault('_rebind_ids',[])
                if list(identity) not in aliases: aliases.append(list(identity))
                if len(stamps)<self.confirm_frames: continue
                for alias in aliases:
                    if alias not in row['observed_id_aliases']: row['observed_id_aliases'].append(alias)
                if len(row['observed_id_aliases'])>128:
                    row['observed_id_aliases']=[row['observed_id_aliases'][0]]+row['observed_id_aliases'][-127:]
                row['reassociation']=dict(original_identity=list(row['original_identity']),new_identity=list(identity),
                    confirmed_at=now,capture_stamps=list(stamps),scope='unique_stationary_local_view',
                    anchor_since=self.stationary_since,
                    physical_identity_verified=False,extinguished_verified=False)
                row.pop('_rebind_stamps',None);row.pop('_rebind_ids',None)
                if row is self.current: self.reset_dwell()
            row.update(obs)
            if row['status']=='candidate' and len(row['post_stop_stamps'])>=self.confirm_frames:
                row['status']='queued'
                row['confirmed_at']=now
        if stationary and at_anchor and now<=self.started_at+self.intake_window_s:
            for identity,obs in self.rows.items():
                if identity in consumed or owners[identity]: continue
                # Known completed/skipped batch members are not new targets.
                if any(list(identity) in row['observed_id_aliases'] for row in self.queue): continue
                if any(self._near_box(row['anchor_bbox_xyxy'],obs['bbox_xyxy'],self.image_size_wh)
                       for row in self.queue if row['status'] in ('completed','skipped')): continue
                row=self._new_candidate(identity,obs,self.image_size_wh,self.coordinate_frame,now)
                if row and self.capture>max(self.started_at,self.stationary_since): row['post_stop_stamps']=[self.capture]

    def observe(self, document, now, *, stationary=True, at_anchor=True):
        if self.abandoned_reason is not None:
            return False
        stamp, session, size = (document.get(key) for key in ('capture_stamp', 'registry_session', 'image_size_wh'))
        if (document.get('frame_valid') is not True or not number(stamp) or not 0 <= now-stamp <= self.ttl_s
                or self.capture is not None and stamp <= self.capture
                or not isinstance(session, str) or not session
                or not isinstance(size, (list, tuple)) or len(size) != 2
                or any(type(value) is not int or value <= 0 for value in size)
                or not isinstance(document.get('targets'), list)):
            return False
        ns,sequence=document.get('capture_stamp_ns'),document.get('sequence')
        if (type(ns) is int and type(sequence) is int and type(self.capture_ns) is int and type(self.sequence) is int
                and (ns<=self.capture_ns or sequence<=self.sequence)):
            return False
        continuous = self.capture is not None and stamp-self.capture <= self.ttl_s
        if not stationary:
            self.stationary_since=None
            for row in self.queue:
                row.pop('_rebind_stamps',None);row.pop('_rebind_ids',None)
        elif self.stationary_since is None:
            self.stationary_since=now
        if not continuous and self.dwell_at is not None:
            self.reset_dwell()
        self.frame_count += 1
        if stationary and stamp > max(self.started_at, self.stationary_since):
            self.post_stop_frame_count += 1
        rows = self._valid_rows(document,now)
        for identity,row in rows.items():
            previous = self.rows.get(identity)
            row['hits']=previous['hits']+1 if continuous and previous else 1
        self.rows, self.capture = rows, stamp
        self.capture_ns, self.sequence = document.get('capture_stamp_ns'), document.get('sequence')
        self.coordinate_frame, self.image_size_wh = document.get('coordinate_frame'), list(size)
        if rows: self.last_high_confidence_at=stamp
        self._associate_batch(now,stationary,at_anchor)
        observation = self.observation(now)
        centered = observation is not None and abs(observation['center_x_norm']-.5) <= self.tolerance
        visible_for_dwell = observation is not None and (self.gimbal_only or centered)
        if self.dwell_at is not None and (not visible_for_dwell or not stationary): self.reset_dwell()
        self.center_hits = self.center_hits+1 if centered and continuous else 1 if centered else 0
        return True

    def freeze(self, now):
        if (self.capture is None or self.post_stop_frame_count < self.confirm_frames or not 0 <= now-self.capture <= self.ttl_s
                or now-self.started_at < .6):
            return False
        stable=[row for row in self.queue if row['status']=='queued']
        # Camera/transport latency is not time spent seeing an empty scene.
        # Compare exposure stamps so braking and delivery delay cannot retire
        # the trigger after just one or two genuinely stopped exposures.
        empty_for=max(0.,self.capture-max(self.started_at,self.last_high_confidence_at or self.started_at))
        if not stable and now-self.started_at<self.collect_wait_s and empty_for<1.2-1e-9:
            self.collect_reason='waiting_post_stop_stable_flame_evidence'
            return False
        if not stable:
            self.collect_reason=('no_flame_visible_after_verified_stop' if empty_for>=1.2-1e-9
                                 else 'flame_batch_confirmation_timeout')
            for row in self.queue:
                if row['status']=='candidate': row.update(status='skipped',reason=self.collect_reason,finished_at=now)
        else:
            self.collect_reason='stable_flames_confirmed_after_stop'
        self.frozen=True;self.collection_closed_at=now
        return True

    def abandon_unfinished(self, now, reason):
        """Close missing-camera observations without claiming a completed fire.

        This is only an image-side operation. The caller must still stop a
        local jog, verify standstill and return the measured route anchor.
        """
        if self.abandoned_reason is not None:
            return
        self.abandoned_reason=reason
        for row in self.queue:
            if row['status'] in ('candidate','queued','aligning','centered'):
                row.update(status='skipped',reason=reason,finished_at=now)
        self.current_index=None
        self.frozen=True
        self.collection_closed_at=now
        self.collect_reason=reason
        self.reset_dwell()

    def pending_candidates(self, now):
        waiting=False
        for row in self.queue:
            if row['status']!='candidate': continue
            if (now<=self.started_at+self.intake_window_s and
                    now-row['last_candidate_stamp']<=self.confirm_window_s):
                waiting=True
            else:
                row.update(status='skipped',reason='new_flame_confirmation_timeout',finished_at=now)
        return waiting

    def select(self):
        if self.current is None:
            pending = [i for i, row in enumerate(self.queue) if row['status'] == 'queued']
            self.current_index = min(pending, key=lambda i: (abs(self.queue[i]['center_x_norm']-.5), i)) if pending else None
            self.reset_dwell()
            if self.current:
                self.current['status'] = 'aligning'
        return self.current

    def observation(self, now):
        row = self.rows.get(self.identity)
        if row and 0 <= now-row['last_seen'] <= self.ttl_s:
            return dict(row,hits=max(row['hits'],len(self.current.get('post_stop_stamps',[]))))
        return None

    def relation(self, now):
        row = self.observation(now)
        if row is None:
            return 'missing'
        error = row['center_x_norm']-.5
        if abs(error) <= self.tolerance:
            return 'center'
        if self.side == 'unset':
            return 'unconfigured'
        return 'forward' if (error > 0) == (self.side == 'right') else 'reverse'

    @staticmethod
    def _anchor_match(anchor_box, anchor_size, observation_box, image_size):
        """Conservative same-view match, never a physical world identity."""
        old=[value/anchor_size[i%2] for i,value in enumerate(anchor_box)]
        new=[value/image_size[i%2] for i,value in enumerate(observation_box)]
        a,b=(old[2]-old[0])*(old[3]-old[1]),(new[2]-new[0])*(new[3]-new[1])
        intersection=max(0.,min(old[2],new[2])-max(old[0],new[0]))*max(0.,min(old[3],new[3])-max(old[1],new[1]))
        overlap=intersection/(a+b-intersection)
        return .67 <= b/a <= 1.5 and overlap >= .55,overlap

    def reassociate_at_anchor(self, now, since):
        """Rebind one lost queue track using three fresh stationary exposures.

        The caller must verify the same route's measured anchor and standstill
        and pass the beginning of that uninterrupted stationary window. Screen
        overlap never establishes an extinguished fire or physical identity.
        """
        current=self.current
        if (not current or not current.get('reacquire_attempted') or self.relation(now)!='missing'
                or not number(since) or not self.started_at <= since <= now
                or self.capture is None or not since < self.capture <= now
                or now-self.capture>self.ttl_s
                or current['anchor_coordinate_frame']!=self.coordinate_frame):
            if current: current.pop('_reassociation_probe',None)
            return False
        original=current['original_identity']
        occupied={(row['registry_session'],row['target_id']) for row in self.queue if row is not current}
        occupied.update(tuple(row['original_identity']) for row in self.queue if row is not current)
        candidates=[]
        for identity,observation in self.rows.items():
            if identity[0]!=original[0] or identity in occupied or identity==tuple(original):
                continue
            match,overlap=self._anchor_match(current['anchor_bbox_xyxy'],current['anchor_image_size_wh'],
                observation['bbox_xyxy'],self.image_size_wh)
            if not match:
                continue
            # A new ID that could also be another queued/finished flame is
            # ambiguous, even if the original ID of that other flame is gone.
            conflict=any(row is not current and row['original_identity'][0]==identity[0] and
                self._anchor_match(row['anchor_bbox_xyxy'],row['anchor_image_size_wh'],
                    observation['bbox_xyxy'],self.image_size_wh)[0] for row in self.queue)
            if not conflict: candidates.append((identity,observation,overlap))
        if len(candidates)!=1:
            current.pop('_reassociation_probe',None)
            return False
        identity,observation,overlap=candidates[0]
        probe=current.get('_reassociation_probe')
        if (not probe or probe['since']!=since or probe['identity']!=identity
                or self.capture-probe['captures'][-1]>self.ttl_s):
            probe=dict(since=since,identity=identity,captures=[],exposure_keys=[])
            current['_reassociation_probe']=probe
        if self.capture in probe['captures']:
            return False
        # capture_stamp is mandatory. If explicit exposure metadata is present,
        # a republished frame cannot gain another vote by changing its float time.
        exposure=(self.capture_ns,self.sequence)
        if any(type(value) is int for value in exposure) and exposure in probe['exposure_keys']:
            return False
        probe['captures'].append(self.capture); probe['exposure_keys'].append(exposure)
        if len(probe['captures'])<self.confirm_frames:
            return False
        evidence=dict(original_identity=list(original),new_identity=list(identity),anchor_since=since,
            confirmed_at=now,capture_stamps=list(probe['captures']),
            exposure_keys=[list(key) for key in probe['exposure_keys']],
            reference_bbox_xyxy=list(current['anchor_bbox_xyxy']),match_iou=overlap,
            coordinate_frame=self.coordinate_frame,physical_identity_verified=False,extinguished_verified=False)
        current.update(observation,reassociation=evidence)
        if list(identity) not in current['observed_id_aliases']:
            current['observed_id_aliases'].append(list(identity))
        if len(current['observed_id_aliases'])>128:
            current['observed_id_aliases']=[current['observed_id_aliases'][0]]+current['observed_id_aliases'][-127:]
        current['post_stop_stamps']=list(probe['captures'])
        current.pop('_reassociation_probe',None)
        self.reset_dwell()
        return True

    def reset_dwell(self):
        self.center_hits = 0
        self.dwell_at = self.dwell_capture = None

    def begin_dwell(self, now):
        self.dwell_at, self.dwell_capture = now, self.capture
        self.current['status'] = 'aligning' if self.gimbal_only else 'centered'

    def dwell_complete(self, now):
        observation=self.observation(now)
        return bool(self.dwell_at is not None and self.dwell_capture is not None
                    and observation and (self.gimbal_only or self.relation(now) == 'center')
                    and (observation['hits'] if self.gimbal_only else self.center_hits) >= self.confirm_frames
                    and now-self.dwell_at >= self.dwell_s
                    and self.capture-self.dwell_capture >= self.dwell_s)

    def finish_target(self, status, reason, now):
        if self.current:
            self.current.update(status=status, reason=reason, finished_at=now)
        self.current_index = None
        self.move_direction = 'stopped'
        self.reset_dwell()

    def snapshot(self, now):
        observation = self.observation(now)
        return dict(enabled=True, active=self.active, phase=self.phase, reason=self.reason,
            queue=[{key:value for key,value in row.items() if not key.startswith('_')} for row in self.queue],
            current_target_id=self.identity[1] if self.identity else None,
            current_batch_id=self.current.get('batch_id') if self.current else None,
            current_registry_session=self.identity[0] if self.identity else None,
            capture_stamp=self.capture, capture_stamp_ns=self.capture_ns, sequence=self.sequence,
            coordinate_frame=self.coordinate_frame, image_size_wh=self.image_size_wh,
            center_tolerance_norm=self.tolerance, dwell_s=self.dwell_s,
            dwell_remaining_s=(max(0., self.dwell_s-(now-self.dwell_at)) if self.dwell_at is not None
                and observation and (self.gimbal_only or self.relation(now) == 'center') else None),
            center_error_norm=observation['center_x_norm']-.5 if observation else None,
            move_direction=self.move_direction, completed_count=sum(row['status']=='completed' for row in self.queue),
            total_count=len(self.queue), queue_truncated=self.queue_truncated,
            collect_reason=self.collect_reason,
            abandoned_reason=self.abandoned_reason,
            collect_deadline=(self.started_at+self.collect_wait_s if self.started_at is not None else None),
            intake_deadline=(self.started_at+self.intake_window_s if self.started_at is not None else None),
            min_confidence=self.threshold, confirm_frames=self.confirm_frames,
            gimbal_only=self.gimbal_only,
            confirmed_count=sum('confirmed_at' in row for row in self.queue),
            unconfirmed_count=sum('confirmed_at' not in row for row in self.queue),
            sprayed_count=sum(row.get('spray_confirmed') is True for row in self.queue),
            semantics=('stationary_gimbal_aim_and_verified_spray_each_confirmed_flame' if self.gimbal_only else
                       'each_flame_stopped_at_image_vertical_center_for_one_second'))
