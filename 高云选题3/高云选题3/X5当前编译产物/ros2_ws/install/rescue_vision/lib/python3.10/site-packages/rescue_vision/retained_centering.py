"""Bounded signed jogs while the FPGA's original navigation route is paused."""
import math
from copy import deepcopy

from . import ov5640_protocol as wire
from .numbered_navigation import recent
from .retained_route import angle_error, local_coordinates
from .route_fire_patrol import off_disarmed, navigation_off_disarmed

PHASES = ('centering_starting', 'centering_moving', 'centering_stopping', 'centering_verifying')


class RetainedCentering:
    def __init__(self, navigation):
        self.navigation = navigation
        self.session = None
        self.counter = 0
        self.last_accepted_sequence = None
        self._pump_stop_recovery = None

    @property
    def active(self):
        return bool(self.session and self.session.get('active'))

    @property
    def map_limits_disabled(self):
        return self.active and self.session.get('map_limits_enabled') is False

    def reset(self):
        self.session = None
        self._pump_stop_recovery = None

    def cancel(self, reason):
        self._pump_stop_recovery = None
        if self.session:
            self.session.update(active=False, reason=reason)

    def _pump_off(self, now):
        n = self.navigation
        check = navigation_off_disarmed if n.active.get('execute_fire') is False else off_disarmed
        return check(n.owner.actuator, now)

    def _pose_reason(self, now):
        n, s = self.navigation, self.session
        pose = n.map.get('pose') or {}
        if not s:
            return 'centering_session_not_active'
        reason=n._retained_identity_reason(now)
        if reason:
            return reason
        if pose.get('route_generation') != s['route_generation']:
            return 'centering_pose_generation_changed'
        if pose.get('execution_index') != s['execution_index']:
            return 'centering_paused_execution_index_changed'
        return ''

    def _pose(self, now):
        if self._pose_reason(now):
            return None
        pose=self.navigation.map['pose']
        return dict(image_mm=[pose['x_mm'],3200-pose['y_mm']], heading_deg=pose['heading_tenths']/10., stamp=pose['stamp'])

    def _pose_evidence(self, now):
        n=self.navigation
        return dict(stamp=now,reason=self._pose_reason(now),pose=dict(n.map.get('pose') or {}),
            host=dict(n.owner.numbered.host),map_reason=n.map.get('reason'),
            map_stamp=n.map.get('stamp'),status_at=n.owner.status_at,
            host_at=n.owner.numbered.host_at,point_at=n.owner.numbered.point_at)

    def _corridor_reason(self, pose):
        s = self.session
        forward,lateral = local_coordinates(s['anchor'],pose['image_mm'])
        # The local distance envelope is relative to the actual pause anchor,
        # independent of map cells, road edges, branches and route turns.
        # Integer X/Y telemetry contributes at most 1 mm per axis to the
        # difference of two rounded poses. Only this actual-pose comparison
        # gets its projected quantization allowance; command bounds stay exact.
        quantization=s['stop_bound_quantization_mm']
        if not s['stop_bounds_mm'][0]-quantization <= forward <= s['stop_bounds_mm'][1]+quantization:
            return 'centering_local_distance_exceeded'
        if abs(lateral) > 40.:
            return 'centering_local_lateral_limit_exceeded'
        if angle_error(pose['heading_deg'],s['anchor']['heading_deg']) > 8.:
            return 'centering_local_heading_changed'
        return self._line_reason(pose)

    def _line_reason(self, pose, distance=0, *, returning=False):
        # A jog and its return use the measured anchor. Map geometry belongs
        # to global route planning and does not veto this local observation.
        return ''

    def begin(self, now):
        n, a = self.navigation, self.navigation.active or {}
        if self.active:
            if a.get('phase') != 'paused' or not n._retained_identity(now,stopped=True):
                raise ValueError('centering_existing_session_requires_paused_pose')
            return self.state(now)
        if n.owner.capabilities.get('retained_jog') is not True:
            raise ValueError('centering_retained_jog_firmware_required')
        if (a.get('phase') != 'paused' or not n._retained_identity(now,stopped=True) or
                not n.current_reference(now) or not self._pump_off(now)):
            raise ValueError('centering_begin_requires_paused_route_and_pump_off')
        pose = n.map['pose']
        anchor = dict(image_mm=[pose['x_mm'],3200-pose['y_mm']],heading_deg=pose['heading_tenths']/10.,stamp=pose['stamp'])
        heading_rad=math.radians(anchor['heading_deg'])
        stop_bound_quantization_mm=min(math.sqrt(2.),abs(math.cos(heading_rad))+abs(math.sin(heading_rad)))
        corridor = dict(backward_mm=-600.,forward_mm=600.,stop_bounds_mm=[-620.,620.],
            route_index=pose['execution_index'],anchor_route_progress_mm=a['progress_mm'],
            lateral_limit_mm=40.,heading_limit_deg=8.,map_limits_enabled=False,
            geometry_verified=False,geometry_source='measured_pause_anchor_without_map_limits',
            reason='centering_map_limits_disabled',backward_reason='centering_local_distance_limit',
            forward_reason='centering_local_distance_limit',return_policy='measured_original_anchor')
        self.counter += 1
        self.session = dict(active=True,session_id='%s:%s:%s' % (a['task_id'],a['route_generation'],self.counter),
            task_id=a['task_id'],route_generation=a['route_generation'],execution_index=pose['execution_index'],
            anchor=anchor,bounds_mm=[corridor['backward_mm'],corridor['forward_mm']],
            stop_bounds_mm=list(corridor['stop_bounds_mm']),
            stop_bound_quantization_mm=stop_bound_quantization_mm,
            corridor=corridor,map_limits_enabled=False,
            saved_progress_mm=a['progress_mm'],saved_progress_index=a['progress_index'],
            started_at=now,travelled_mm=0.,travel_budget_mm=2400.,jog_count=0,jog_complete=True,
            last_valid_pose=deepcopy(anchor),short_stop_count=0,short_stop_direction=None,
            short_stop_owner=None,jog_serial=0,
            reason='centering_anchor_captured',jog=None,query_sequence=None,next_poll=now)
        return self.state(now)

    def proposal(self, direction, now, step_mm=40, speed_mm_s=40, *, returning=False):
        base = dict(stamp=now,valid=False,reason='centering_verified_paused_pose_required')
        n, s = self.navigation, self.session
        if self._pump_stop_recovery is not None:
            return dict(base, reason='centering_waiting_fresh_pump_low')
        if (not self.active or n.active.get('phase') != 'paused' or not n._retained_identity(now,stopped=True)
                or not self._pump_off(now)):
            return base
        pose = self._pose(now)
        if not pose: return base
        if type(direction) is not int or direction not in (-1,1):
            return dict(base,reason='centering_direction_requires_minus1_or1')
        if type(step_mm) is not int or not 1 <= step_mm <= 100 or type(speed_mm_s) is not int or not 20 <= speed_mm_s <= 60:
            return dict(base,reason='centering_step_or_speed_out_of_range')
        reason = self._corridor_reason(pose)
        if reason: return dict(base,reason=reason)
        offset,lateral = local_coordinates(s['anchor'],pose['image_mm'])
        available = s['bounds_mm'][1]-offset if direction>0 else offset-s['bounds_mm'][0]
        distance = direction*min(step_mm,max(0,math.floor(available)))
        if not distance:
            key='forward_reason' if direction>0 else 'backward_reason'
            return dict(base,reason=s['corridor'].get(key,'centering_straight_corridor_exhausted'))
        reason=self._line_reason(pose,distance,returning=returning)
        if reason:return dict(base,reason=reason)
        # Every target move keeps enough travelled-distance budget to return.
        reserve = abs(offset+distance)+20.
        if s['travelled_mm']+abs(distance)+reserve > s['travel_budget_mm']:
            return dict(base,reason='centering_distance_budget_reserved_for_return')
        return dict(base,valid=True,reason='centering_retained_signed_jog_proposed',
            session_id=s['session_id'],route_generation=s['route_generation'],task_id=s['task_id'],
            distance_mm=distance,speed_mm_s=speed_mm_s,returning=returning,
            from_image_mm=pose['image_mm'],from_heading_deg=pose['heading_deg'],from_pose_stamp=pose['stamp'])

    def return_proposal(self, now):
        state = self.state(now)
        if state.get('return_ready'):
            return dict(valid=True,at_anchor=True,stamp=now,session_id=self.session['session_id'])
        if not self.active or not state.get('pose_valid'):
            return dict(valid=False,at_anchor=False,stamp=now,reason='centering_return_requires_fresh_pose')
        offset = state['offset_mm']
        if abs(offset) <= 1.:
            return dict(valid=False,at_anchor=False,stamp=now,reason='centering_return_lateral_or_heading_error')
        pose=self._pose(now)
        direction=-1 if offset>0 else 1
        radians=math.radians(pose['heading_deg'])
        unit=[direction*math.cos(radians),-direction*math.sin(radians)]
        anchor=self.session['anchor']['image_mm']
        delta=[pose['image_mm'][i]-anchor[i] for i in (0,1)]
        closest=-sum(delta[i]*unit[i] for i in (0,1))
        perpendicular_sq=max(0.,sum(value*value for value in delta)-closest*closest)
        failure=dict(valid=False,at_anchor=False,stamp=now,
            reason='centering_return_window_unreachable_on_verified_line')
        if closest<=0. or perpendicular_sq>400.:
            return failure
        half_chord=math.sqrt(400.-perpendicular_sq)
        first_step=max(1,math.ceil(closest-half_chord))
        last_step=math.floor(closest+half_chord)
        last=None
        # Return into the existing measured 20 mm acceptance window instead
        # of commanding exact zero and projecting the 20 mm braking envelope
        # beyond it. Integer steps and XY quantisation are checked explicitly.
        # Solve the actual yaw ray/circle intersection, not abs(offset)>120:
        # offset 119 mm with 10 mm lateral error needs a little over 100 mm.
        # A 100 mm intermediate jog is allowed only when the complete remaining
        # return ray reaches an integer endpoint in the acceptance circle and
        # its endpoint remains within the measured local-distance envelope.
        for step in range(first_step,last_step+1):
            endpoint=[pose['image_mm'][i]+step*unit[i] for i in (0,1)]
            if not self._return_window_contains(endpoint):
                continue
            endpoint_offset,_=local_coordinates(self.session['anchor'],endpoint)
            if not self.session['bounds_mm'][0]<=endpoint_offset<=self.session['bounds_mm'][1]:
                continue
            reason=self._line_reason(pose,direction*step,returning=True)
            if reason:
                last=dict(failure,reason=reason)
                continue
            bounded_step=min(100,step)
            last=self.proposal(direction,now,step_mm=bounded_step,speed_mm_s=40,returning=True)
            if last.get('valid') and last.get('distance_mm')==direction*bounded_step:
                return last
            if last.get('valid'):
                last=dict(failure,reason='centering_return_intermediate_step_limited')
        return last or failure

    def begin_jog(self, proposal, now):
        s, n, a = self.session, self.navigation, self.navigation.active or {}
        distance = proposal.get('distance_mm')
        returning=proposal.get('returning') is True
        target_key=None if returning else proposal.get('motion_target_key')
        if target_key is not None and (not isinstance(target_key,str) or not 1<=len(target_key)<=128):
            raise ValueError('centering_motion_target_key_invalid')
        if type(distance) is not int or distance == 0:
            raise ValueError('centering_signed_distance_required')
        checked = self.proposal(1 if distance>0 else -1,now,abs(distance),proposal.get('speed_mm_s'),
                                returning=proposal.get('returning') is True)
        if (proposal.get('valid') is not True or checked.get('valid') is not True or
                any(proposal.get(key)!=checked.get(key) for key in
                    ('session_id','route_generation','task_id','distance_mm','speed_mm_s','from_image_mm'))):
            raise ValueError('centering_proposal_changed_or_invalid')
        pose = self._pose(now)
        payload = wire.retained_jog_payload(distance,checked['speed_mm_s'])
        # A wrapping UART sequence must not look like a replay of the last jog.
        reserved = (n.owner.active or {}).get('request_sequence')
        sequence = next((i for offset in range(1,257) if
            (i := (n.owner.sequence+offset)&255) not in n.owner.pending and
            i != self.last_accepted_sequence and i != reserved),None)
        if sequence is None: raise ValueError('centering_uart_sequences_exhausted')
        sequence = n.owner.send(wire.RETAINED_JOG,payload,now=now,seq=sequence)
        owner=('return',None) if returning else ('target',target_key)
        if owner!=s['short_stop_owner']:
            s.update(short_stop_count=0,short_stop_direction=None,short_stop_owner=owner)
        s['jog_serial']+=1
        offset,_ = local_coordinates(s['anchor'],pose['image_mm'])
        deadline = now+min(8.,abs(distance)/checked['speed_mm_s']+3.)
        s.update(jog=dict(sequence=sequence,proposal=checked,start_offset_mm=offset,
            jog_id=s['jog_serial'],motion_target_key=target_key,
            target_identity=deepcopy(proposal.get('target_identity')) if not returning else None,
            target_motion_exhausted=False,motion_exhausted_token=None,
            target_offset_mm=offset+distance,start_pose=pose,issued_at=now,deadline=deadline,
            ack_at=None,stop_requested=False,terminal_at=None,status=None),
            query_sequence=None,next_poll=now,jog_complete=False,reason='centering_jog_waiting_acceptance')
        a.update(phase='centering_starting',phase_at=now,sequence=sequence)

    def write_ready(self, sequence, now):
        n,s = self.navigation,self.session
        pose=self._pose(now) if self.active else None
        return bool(self.active and s.get('jog') and s['jog']['sequence']==sequence and
            self._pump_stop_recovery is None and
            n.active.get('phase')=='centering_starting' and pose and
            n._retained_identity(now,stopped=True) and self._pump_off(now) and
            not self._corridor_reason(pose) and
            not self._line_reason(pose,s['jog']['proposal']['distance_mm'],
                                  returning=s['jog']['proposal'].get('returning') is True) and
            not n.owner.mode_transition and not n.owner.zebra_transition)

    def mark_written(self, sequence, now):
        """Record only the physical UART writer's successful complete write."""
        if self.active and self.session.get('jog') and self.session['jog']['sequence']==sequence:
            self.session['jog']['written_at']=now

    def stop_jog(self, now):
        if not self.active: raise ValueError('centering_session_not_active')
        n,s = self.navigation,self.session
        if n.active.get('phase')=='paused': return
        if n.active.get('phase') not in PHASES: raise ValueError('centering_jog_not_active')
        if s['jog']['stop_requested']: return
        s['jog']['stop_requested']=True
        sequence=n.owner.send(wire.STOP,now=now)
        s['jog']['stop_sequence']=sequence
        n.active.update(phase='centering_stopping',phase_at=now,sequence=sequence)
        s['reason']='centering_waiting_signed_jog_stop'

    def hold_pump_stop_recovery(self, token, now, deadline):
        """Authorize only bounded STOP verification, never a new motion write."""
        n, s = self.navigation, self.session
        if (not self.active or not s.get('jog') or n.active.get('execute_fire') is not False
                or n.active.get('phase') not in PHASES or not now < deadline <= now + 3.):
            raise ValueError('centering_pump_recovery_requires_owned_search_jog')
        self.stop_jog(now)
        self._pump_stop_recovery = dict(token=token, leg=n.active, jog=s['jog'],
            task_id=n.active['task_id'], generation=n.active['route_generation'],
            stop_sequence=s['jog']['stop_sequence'], started=now, deadline=deadline)

    def clear_pump_stop_recovery(self, token):
        if self._pump_stop_recovery and self._pump_stop_recovery['token'] is token:
            self._pump_stop_recovery = None

    def pump_stop_recovery_matches(self, token, now, *, include_paused=False):
        r, n, s = self._pump_stop_recovery, self.navigation, self.session
        if not r or r['token'] is not token or not self.active:
            return False
        a, v, m = n.active or {}, n.owner.actuator, n.owner.mission_state
        phases = ('centering_stopping', 'centering_verifying', 'paused') if include_paused else (
            'centering_stopping', 'centering_verifying')
        return bool(r['started'] <= now < r['deadline'] and a is r['leg']
            and a.get('task_id') == r['task_id'] and a.get('route_generation') == r['generation']
            and s.get('route_generation') == r['generation'] and s.get('jog') is r['jog']
            and a.get('execute_fire') is False and a.get('phase') in phases
            and s['jog'].get('stop_requested') and s['jog'].get('stop_sequence') == r['stop_sequence']
            and not n.owner.mode_transition and not n.owner.zebra_transition
            and n.owner.task in ('dialogue', 'semantic') and not m.get('operator_hold')
            and not m.get('emergency_stop') and not m.get('traffic_active')
            and not v.get('estop') and v.get('pump_on') is not True
            and v.get('pump_requested_on') is not True and v.get('manual_pump_active') is not True
            and (v.get('guard') or {}).get('armed') is False)

    def _failure(self, reason, now):
        s,n=self.session,self.navigation
        if 'motion_failure' not in s:
            pose=self._pose(now)
            jog=s.get('jog') or {}
            offset,lateral=local_coordinates(s['anchor'],pose['image_mm']) if pose else (None,None)
            s['motion_failure']=deepcopy(dict(stamp=now,reason=reason,
                task_id=s['task_id'],route_generation=s['route_generation'],execution_index=s['execution_index'],
                anchor=s['anchor'],bounds_mm=s['bounds_mm'],stop_bounds_mm=s['stop_bounds_mm'],corridor=s['corridor'],
                stop_bound_quantization_mm=s['stop_bound_quantization_mm'],
                actual_pose=pose,last_valid_pose=s.get('last_valid_pose'),raw_pose=n.map.get('pose'),
                actual_offset_mm=offset,lateral_mm=lateral,
                bounds_exceeded_mm=(max(0.,s['stop_bounds_mm'][0]-offset,offset-s['stop_bounds_mm'][1])
                    if offset is not None else None),
                command_bounds_exceeded_mm=(max(0.,s['bounds_mm'][0]-offset,offset-s['bounds_mm'][1])
                    if offset is not None else None),
                jog=jog,host=n.owner.numbered.host,status_at=n.owner.status_at))
        self.session['reason']=reason
        self.navigation.stop(now,reason)

    def on_reply(self, packet, sent_at, now):
        if not self.active or not self.session.get('jog'): return False
        s,n = self.session,self.navigation
        jog=s['jog']; command=packet.command&0x7f
        if command==wire.RETAINED_JOG_STATUS and packet.sequence==s.get('query_sequence'):
            s['query_sequence']=None
            try: status=wire.retained_jog_status(packet.data)
            except ValueError as exc:
                self._failure(str(exc),now); return True
            if status['accepted_sequence']!=jog['sequence']:
                # STOP removes unsent 3C frames and invalidates their writer
                # guard. A later status and unchanged measured pose may prove
                # that the cancelled command never became an accepted jog.
                pose=self._pose(now)
                stop_ack=jog.get('stop_ack_at')
                if (jog['stop_requested'] and jog['ack_at'] is None and stop_ack is not None
                        and sent_at>stop_ack and status['nav_valid'] and status['stopped']
                        and status['phase'] in (0,3,4) and not status['run'] and pose and pose['stamp']>stop_ack
                        and math.dist(pose['image_mm'],jog['start_pose']['image_mm'])<=2.
                        and angle_error(pose['heading_deg'],jog['start_pose']['heading_deg'])<=1.):
                    jog.update(status=status,status_at=now,status_requested_at=sent_at,
                               cancelled_before_accept=True)
                    if jog['terminal_at'] is None:
                        jog.update(terminal_at=sent_at,terminal_seen_at=now)
                    n.active.update(phase='centering_verifying',phase_at=now)
                return True
            if status['phase']==0:
                return True
            jog.update(status=status,status_at=now,status_requested_at=sent_at)
            if status['phase'] in (3,4) and status['stopped'] and not status['run']:
                if status['phase']==4 and not (jog['stop_requested'] and status['reason_code']==1):
                    self._failure('centering_fpga_aborted_%d'%status['reason_code'],now); return True
                if jog['terminal_at'] is None:
                    jog.update(terminal_at=sent_at,terminal_seen_at=now)
                n.active.update(phase='centering_verifying',phase_at=now)
            return True
        if command==wire.RETAINED_JOG and packet.sequence==jog['sequence']:
            if packet.data[0]: self._failure('centering_fpga_jog_rejected_%d'%packet.data[0],now)
            else:
                jog['ack_at']=now; self.last_accepted_sequence=jog['sequence']
                if not jog['stop_requested']:
                    n.active.update(phase='centering_moving',phase_at=now)
            return True
        if command==wire.STOP and packet.sequence==jog.get('stop_sequence'):
            if packet.data[0]: self._failure('centering_stop_rejected',now)
            else: jog['stop_ack_at']=now
            return True
        return False

    def tick(self, now):
        n,s = self.navigation,self.session
        if not self.active or not s.get('jog'): return
        a,jog = n.active,s['jog']
        if a.get('phase') not in PHASES: return
        if not n.feedback(now): self._failure('centering_navigation_feedback_lost',now); return
        recovery = self._pump_stop_recovery
        stopping_readback = bool(recovery and self.pump_stop_recovery_matches(recovery['token'], now))
        if not self._pump_off(now) and not stopping_readback:
            self._failure('centering_pump_off_lost',now); return
        pose=self._pose(now)
        if pose:
            s['last_valid_pose']=deepcopy(pose)
            s.pop('pose_missing_since',None)
            reason=self._corridor_reason(pose)
            if reason: self._failure(reason,now); return
            offset,_=local_coordinates(s['anchor'],pose['image_mm'])
            displacement=offset-jog['start_offset_mm']
            sign=1 if jog['proposal']['distance_mm']>0 else -1
            if not -10. <= sign*displacement <= abs(jog['proposal']['distance_mm'])+20.:
                self._failure('centering_jog_direction_or_distance_mismatch',now); return
        else:
            s.setdefault('pose_missing_since',now)
            s['pose_missing_evidence']=self._pose_evidence(now)
            if now-s['pose_missing_since']>1.2:
                s['pose_failure']=dict(s['pose_missing_evidence'],since=s['pose_missing_since'])
                self._failure('centering_same_route_pose_lost',now); return
        verify_deadline=(jog['terminal_seen_at']+1.5 if jog.get('terminal_seen_at') is not None else jog['deadline'])
        if now>verify_deadline:
            self._failure('centering_jog_timeout',now); return
        if not jog['stop_requested'] and jog['ack_at'] is None and now-jog['issued_at']>2.:
            self._failure('centering_jog_ack_timeout',now); return
        if jog['stop_requested'] and jog.get('stop_ack_at') is None and now-a['phase_at']>2.:
            self._failure('centering_stop_ack_timeout',now); return
        status=jog.get('status') or {}
        # A latch issued before terminal receipt is not endpoint evidence.
        # Wait for samples captured after the received DONE/STOP result.
        after=jog.get('terminal_seen_at')
        if (a['phase']=='centering_verifying' and after is not None and pose and
                pose['stamp']>after and n.stopped(now,after) and status.get('nav_valid') is True and
                status.get('stopped') is True and not status.get('run') and recent(jog.get('status_at'),now,.8)):
            offset,_=local_coordinates(s['anchor'],pose['image_mm'])
            previous=jog.get('stop_pose')
            if previous is None:
                jog['stop_pose']=deepcopy(pose)
                jog['stop_pose_samples']=1
                return
            if pose['stamp']<=previous['stamp'] or pose['stamp']-previous['stamp']<.1:
                return
            stable=(math.dist(pose['image_mm'],previous['image_mm'])<=2. and
                    angle_error(pose['heading_deg'],previous['heading_deg'])<=.2)
            jog['stop_pose']=deepcopy(pose)
            jog['stop_pose_samples']=jog.get('stop_pose_samples',1)+1 if stable else 1
            if jog['stop_pose_samples']<2:
                return
            returning=jog['proposal'].get('returning') is True
            endpoint_error=abs(offset-jog['target_offset_mm'])
            start=jog['start_pose'];distance=jog['proposal']['distance_mm']
            radians=math.radians(start['heading_deg'])
            planned_endpoint=[start['image_mm'][0]+distance*math.cos(radians),
                              start['image_mm'][1]-distance*math.sin(radians)]
            final_return=returning and math.dist(planned_endpoint,s['anchor']['image_mm'])<=20.
            endpoint_reached=(self._at_anchor(pose) or not final_return and endpoint_error<=15.
                if returning else endpoint_error<=15.)
            jog.update(endpoint_reached=endpoint_reached,endpoint_error_mm=endpoint_error,
                remaining_mm=jog['target_offset_mm']-offset,verified_stop_pose=deepcopy(pose))
            sign=1 if distance>0 else -1
            moved=sign*(offset-jog['start_offset_mm'])
            jog['measured_directional_progress_mm']=moved
            if (not jog.get('cancelled_before_accept') and status['phase']==3 and
                    not (returning and self._at_anchor(pose)) and moved<=.5):
                # The 15 mm endpoint tolerance cannot prove a short command
                # moved at all. Refuse zero/reverse progress before that
                # tolerance can reset the bounded short-stop counter.
                jog['endpoint_reached']=False
                self._failure('centering_jog_no_forward_progress',now); return
            short=False
            if not jog.get('cancelled_before_accept') and status['phase']==3 and not endpoint_reached:
                remaining=sign*(jog['target_offset_mm']-offset)
                # DONE closes this physical move; it does not prove the flame
                # is aligned. Only a genuine bounded short move can be retried
                # by the next fresh visual/anchor proposal, never blind replay.
                if moved>=2. and remaining>0:
                    count=s['short_stop_count']+1 if s['short_stop_direction']==sign else 1
                    s.update(short_stop_count=count,short_stop_direction=sign)
                    if count>=3:
                        if returning or not jog.get('motion_target_key'):
                            self._failure('centering_repeated_short_stop',now); return
                        # Only this named visual target has exhausted its move
                        # allowance. Its consumer may skip it after seeing this
                        # verified STOP; the original route remains retained.
                        jog.update(target_motion_exhausted=True,
                            motion_exhausted_token='%s:%s:%s' %
                                (s['session_id'],jog['jog_id'],jog['sequence']))
                    short=True
                else:
                    self._failure('centering_jog_endpoint_not_measured',now); return
            elif status['phase']==3:
                s.update(short_stop_count=0,short_stop_direction=None)
            jog['measured_short_stop']=short
            s['travelled_mm']+=abs(offset-jog['start_offset_mm']); s['jog_count']+=1
            reason=('centering_target_motion_exhausted' if jog.get('target_motion_exhausted') else
                'centering_jog_measured_short_stop' if short else 'centering_jog_measured_stop')
            s.update(jog_complete=True,reason=reason,query_sequence=None)
            a.update(pause_ack_at=after,pause_at=a.get('pause_at',now),reason=reason)
            n._finish('paused',now)
            return
        if s.get('query_sequence') is not None and now-s.get('query_sent_at',now)>.6:
            s['query_sequence']=None
        if (s.get('query_sequence') is None and now>=s['next_poll'] and
                (jog['ack_at'] is not None or jog.get('stop_ack_at') is not None)):
            s['query_sequence']=n.owner.send(wire.RETAINED_JOG_STATUS,now=now)
            s.update(query_sent_at=now,next_poll=now+.2)

    def end(self, now):
        if self._pump_stop_recovery is not None:
            raise ValueError('centering_waiting_fresh_pump_low')
        state=self.state(now)
        if not state.get('return_ready'):
            raise ValueError('centering_end_requires_measured_original_anchor')
        self.session.update(active=False,reason='centering_original_anchor_restored')
        return self.state(now)

    def state(self, now):
        s,n = self.session,self.navigation
        if not s: return dict(active=False,pose_valid=False,return_ready=False,jog_complete=False,
                              map_limits_enabled=False)
        pose=self._pose(now) if self.active else None
        offset,lateral=local_coordinates(s['anchor'],pose['image_mm']) if pose else (None,None)
        ready=bool(pose and n.active.get('phase')=='paused' and n._retained_identity(now,stopped=True)
            and self._at_anchor(pose) and s['jog_complete'])
        jog=s.get('jog') or {}
        return dict(active=self.active,session_id=s['session_id'],anchor=dict(s['anchor']),
            map_limits_enabled=s['map_limits_enabled'],
            bounds_mm=list(s['bounds_mm']),stop_bounds_mm=list(s['stop_bounds_mm']),
            stop_bound_quantization_mm=s['stop_bound_quantization_mm'],
            offset_mm=offset,lateral_mm=lateral,pose_valid=bool(pose),
            return_ready=ready,jog_complete=bool(s['jog_complete'] and n.active.get('phase')=='paused'),
            travelled_mm=round(s['travelled_mm'],3),travel_budget_mm=s['travel_budget_mm'],
            jog_count=s['jog_count'],jog_status=jog.get('status'),
            command_sequence=jog.get('sequence'),written_at=jog.get('written_at'),ack_at=jog.get('ack_at'),
            acknowledged_sequence=jog.get('sequence') if jog.get('ack_at') is not None else None,
            accepted_sequence=jog.get('sequence') if jog.get('ack_at') is not None else None,
            endpoint_reached=jog.get('endpoint_reached'),remaining_mm=jog.get('remaining_mm'),
            measured_short_stop=jog.get('measured_short_stop',False),short_stop_count=s['short_stop_count'],
            target_motion_exhausted=jog.get('target_motion_exhausted',False),
            motion_target_key=jog.get('motion_target_key'),target_identity=deepcopy(jog.get('target_identity')),
            jog_returning=(jog.get('proposal') or {}).get('returning') is True,
            motion_exhausted_token=jog.get('motion_exhausted_token'),
            stop_pose_samples=jog.get('stop_pose_samples',0),
            motion_failure=deepcopy(s.get('motion_failure')),
            pose_reason=self._pose_reason(now) if self.active else
                (s.get('pose_failure') or (n.active or {}).get('identity_failure') or {}).get('reason'),
            pose_failure=s.get('pose_failure') or (n.active or {}).get('identity_failure'),
            corridor=dict(s['corridor']),
            reason=s['reason'],route_generation=s['route_generation'],execution_index=s['execution_index'],
            saved_route_progress_mm=s['saved_progress_mm'],physical_position_verified=False)

    def _at_anchor(self, pose):
        return bool(pose and self._return_window_contains(pose['image_mm'])
            and angle_error(pose['heading_deg'],self.session['anchor']['heading_deg'])<=8.)

    def _return_window_contains(self, point):
        # The temporary jog may leave the map. Its final return must be in the
        # original route's map frame before normal navigation takes ownership.
        return bool(0 <= point[0] < 3520 and 0 <= point[1] < 3200
            and math.dist(point,self.session['anchor']['image_mm']) <= 20.)
