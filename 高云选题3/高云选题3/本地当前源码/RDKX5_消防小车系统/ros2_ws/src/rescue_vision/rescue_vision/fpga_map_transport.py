"""Low-priority, read-only map telemetry on the existing fixed-10 UART owner.

No I/O lives here. New snapshots and route downloads use idle UART slots.
Once latched, a pose's three continuation reads may run ahead of ordinary
polls, while STOP/OFF/heartbeat remain the UART owner's higher priority.
"""
from collections import deque
import copy
import struct

CAPS, POSE_LATCH, POSE_XY, POSE_ANGLE = 0x30, 0x31, 0x32, 0x33
ROUTE_HEADER, ROUTE_POINT, MAP_ID = 0x34, 0x35, 0x36
ROUTE_CURSOR = 0x37
BARRIER_HEADER, BARRIER_POINT = 0x38, 0x39
START_REFERENCE = 0x3a
REFERENCE_GENERATION = 0x3b
COMMANDS = frozenset((CAPS, POSE_LATCH, POSE_XY, POSE_ANGLE, ROUTE_HEADER,
                      ROUTE_POINT, MAP_ID, ROUTE_CURSOR, BARRIER_HEADER, BARRIER_POINT, START_REFERENCE, REFERENCE_GENERATION))
INVALID = b'\xff'*4
POSE_COMMANDS = frozenset((POSE_LATCH, POSE_XY, POSE_ANGLE, ROUTE_CURSOR))


class _ReadOnlyRouteList(list):
    """JSON/list-compatible static route data shared only by internal readers."""
    def _readonly(self, *args, **kwargs):
        raise TypeError('internal_route_snapshot_is_read_only')

    __setitem__ = __delitem__ = __iadd__ = __imul__ = _readonly
    append = clear = extend = insert = pop = remove = reverse = sort = _readonly

    def __copy__(self):
        return self

    def __deepcopy__(self, memo):
        return self


def _read_only_route_value(value):
    if isinstance(value, list):
        return _ReadOnlyRouteList(_read_only_route_value(item) for item in value)
    if isinstance(value, tuple):
        return tuple(_read_only_route_value(item) for item in value)
    return copy.deepcopy(value)


class MapTelemetry:
    def __init__(self, max_route_samples=128):
        if type(max_route_samples) is not int or not 2 <= max_route_samples <= 256:
            raise ValueError('map_sample_count_outside_2_to_256')
        self.sample_limit=max_route_samples
        self.full_route_required=False
        self.pose_period_s=.5
        self.supported=None;self.reason='waiting_for_map_capability'
        self.capability_flags=0
        self.map_signature=None;self.waiting=None
        self.last_reply_at=None
        self.last_pose_latch_at=float('-inf')
        self.pose_completed=0;self.pose_timeouts=0
        self.last_pose_duration_ms=None;self.last_pose_completed_at=None
        self.next_probe=0.;self.next_pose=0.;self.next_route=0.;self.next_barriers=0.
        self.next_map_id=0.
        self.pose={};self.route={};self.barriers={}
        self.start_reference={};self.next_reference=0.
        self.reference_work=None
        self.pose_work=None;self.route_work=None;self.barrier_work=None
        self.urgent=deque();self.download=deque();self.failures=0
        self._snapshot_route_source=None;self._snapshot_route_static={}

    def disconnected(self):
        self.supported=None;self.map_signature=None
        self.capability_flags=0
        self.waiting=None;self.urgent.clear();self.download.clear()
        self.pose_work=self.route_work=self.barrier_work=None
        self.pose={};self.route={};self.barriers={}
        self.start_reference={}
        self.reference_work=None
        self.last_reply_at=None
        self.last_pose_latch_at=float('-inf')
        self.next_probe=0.;self.next_map_id=0.;self.failures=0
        self._snapshot_route_source=None;self._snapshot_route_static={}
        self.reason='fpga_uart_unavailable'

    def expire(self, now):
        if self.waiting and now-self.waiting['sent_at']>.12:
            pending=self.waiting
            command=pending['command'];self.waiting=None;self.failures+=1
            self.reason='map_reply_timeout'
            # Pose tokens and route generations are independent snapshots.
            # A missed pose must not discard hundreds of verified route points.
            if command in POSE_COMMANDS:
                self.pose_timeouts+=1
                self.pose_work=None
                self.pose=dict(stamp=pending['sent_at'],valid=False,reason='map_reply_timeout',
                               failed_command=command)
                self.urgent=deque(req for req in self.urgent if req[0] not in
                    POSE_COMMANDS)
            elif command in (ROUTE_POINT,ROUTE_HEADER) and self.route_work:
                work=self.route_work
                key=(command,pending['data'])
                retries=work.setdefault('retries',{})
                retries[key]=retries.get(key,0)+1
                if retries[key]<=2:
                    self.download.appendleft((command,pending['data'],pending['tag']))
                else:
                    self.route_work=None;self.route={};self._drop_download('route')
            elif command in (BARRIER_HEADER,BARRIER_POINT):
                self.barrier_work=None;self._drop_download('barrier')
            elif command in (START_REFERENCE,REFERENCE_GENERATION):
                self.reference_work=None
                self.urgent=deque(req for req in self.urgent if req[0] not in
                    (START_REFERENCE,REFERENCE_GENERATION))
            if command==CAPS:
                self.next_probe=now+5.
            if self.failures>=3:
                self.supported=False;self.reason='map_telemetry_unavailable'
                self.capability_flags=0;self.map_signature=None
                self.next_probe=max(self.next_probe,now+1.)
        if self.pose_work and now-self.pose_work['stamp']>.35:
            self.pose_timeouts+=1
            # Drop only this incomplete atomic snapshot. Keeping its old
            # continuation queue would delay a new usable latch indefinitely.
            stamp=self.pose_work['stamp']
            self.pose_work=None
            self.pose=dict(stamp=stamp,valid=False,reason='map_pose_snapshot_timeout')
            self.reason='map_pose_snapshot_timeout'
            self.urgent=deque(req for req in self.urgent if req[0] not in POSE_COMMANDS)

    def next_pose_continuation(self, now):
        """Return one read from the current latch, never start a new snapshot."""
        self.expire(now)
        if self.waiting or self.supported is not True or not self.pose_work:
            return None
        token=self.pose_work['token']
        for request in self.urgent:
            if request[0] in (POSE_XY,POSE_ANGLE,ROUTE_CURSOR) and request[1][:2]==token:
                self.urgent.remove(request)
                return request
        return None

    def next_control_pose_request(self, now):
        """Bounded pose service during navigation; excludes all bulk reads.

        The UART owner keeps stop/off, heartbeat and due safety feedback ahead
        of this request. New priority latches are capped below 3 Hz, so fresh
        poses cannot monopolize ordinary telemetry or actuator feedback.
        """
        continuation=self.next_pose_continuation(now)
        if continuation:
            return continuation
        if (self.waiting or self.pose_work or self.supported is not True or
                self.map_signature is None or now<self.next_pose or
                now-self.last_pose_latch_at<.35):
            return None
        if self.pose.get('valid') is True and now-self.pose['stamp']<.35:
            return None
        self.next_pose=now+self.pose_period_s
        return POSE_LATCH,bytes(4),'pose_latch'

    def next_recovery_request(self, now):
        """One due read-only recovery probe, even after a task has stopped.

        CAPS attempts are spaced five seconds apart and missing map identity
        reads one second apart, starting at physical write. The owner's one
        queued/in-flight map slot prevents duplicate requests while waiting.
        """
        self.expire(now)
        if self.waiting:
            return None
        if self.supported is not True:
            return (CAPS,bytes(4),'caps') if now>=self.next_probe else None
        if self.map_signature is None and now>=self.next_map_id:
            return MAP_ID,bytes(4),'map_id'
        return None

    def next_request(self, now):
        self.expire(now)
        if self.waiting:return None
        recovery=self.next_recovery_request(now)
        if recovery:return recovery
        if self.supported is not True or self.map_signature is None:return None
        continuation=self.next_pose_continuation(now)
        if continuation:return continuation
        if now>=self.next_probe:
            return CAPS,bytes(4),'caps'
        if self.urgent:return self.urgent.popleft()
        if now>=self.next_pose:
            self.next_pose=now+self.pose_period_s
            return POSE_LATCH,bytes(4),'pose_latch'
        if now>=self.next_route:
            self.next_route=now+.5
            return ROUTE_HEADER,bytes(4),'route_head'
        if now>=self.next_barriers:
            self.next_barriers=now+1.
            return BARRIER_HEADER,bytes(4),'barrier_head'
        if self.capability_flags&32 and now>=self.next_reference:
            self.next_reference=now+.5
            return REFERENCE_GENERATION,bytes(4),'reference_head'
        if self.download:return self.download.popleft()
        return None

    def sent(self, sequence, request, now):
        command,data,tag=request
        self.waiting=dict(sequence=sequence,command=command,data=data,tag=tag,sent_at=now)
        if command==POSE_LATCH:
            self.last_pose_latch_at=now
        elif command==CAPS:
            self.next_probe=now+5.
        elif command==MAP_ID:
            self.next_map_id=now+1.

    def _drop_download(self, prefix):
        self.download=deque(x for x in self.download if not x[2].startswith(prefix))

    @staticmethod
    def route_header(data):
        word=int.from_bytes(data[2:4],'little')
        return dict(generation=int.from_bytes(data[:2],'little'),count=word&0xfff,
                    valid=bool(word&0x8000),planning=bool(word&0x4000))

    def receive(self, packet, now):
        pending=self.waiting
        if (not pending or packet.sequence!=pending['sequence'] or
                packet.command!=(pending['command']|0x80) or not 0<=now-pending['sent_at']<=.12):
            return False
        self.waiting=None;self.failures=0;self.last_reply_at=now
        data=packet.data;tag=pending['tag'];command=pending['command']
        if command==CAPS:
            self.supported=(len(data)==4 and data[0]==0 and data[1]==1 and data[3]==1
                            and data[2]&15==15)
            self.capability_flags=data[2] if self.supported else 0
            self.next_probe=now+5.
            self.reason='map_protocol_ready' if self.supported else 'map_firmware_not_supported'
            return True
        if data==INVALID:
            self.reason='map_snapshot_changed_retrying'
            if command in (POSE_XY,POSE_ANGLE,ROUTE_CURSOR):
                self.pose_work=None;self.urgent.clear()
                self.pose=dict(stamp=pending['sent_at'],valid=False,reason='map_snapshot_changed_retrying',
                               failed_command=command)
            elif command in (ROUTE_HEADER,ROUTE_POINT):
                self.route_work=None;self.route={};self._drop_download('route')
            elif command in (BARRIER_HEADER,BARRIER_POINT):
                self.barrier_work=None;self.barriers={};self._drop_download('barrier')
            elif command in (START_REFERENCE,REFERENCE_GENERATION):
                self.reference_work=None;self.start_reference=dict(stamp=pending['sent_at'],valid=False)
            return True
        if command==MAP_ID:
            self.map_signature=int.from_bytes(data,'little')
        elif command==START_REFERENCE:
            x,y=data[:2];heading=int.from_bytes(data[2:],'little',signed=True)
            valid=0<=x<176 and 0<=y<160 and -1800<=heading<=1800
            if self.reference_work is not None:
                self.reference_work.update(valid=valid,cell=[x,y] if valid else None,
                    heading_deg=heading/10. if valid else None)
                self.urgent.appendleft((REFERENCE_GENERATION,bytes(4),'reference_verify'))
        elif command==REFERENCE_GENERATION:
            generation=int.from_bytes(data,'little')
            if generation==0:
                self.reference_work=None
                self.start_reference=dict(stamp=pending['sent_at'],valid=False,generation=0)
            elif tag=='reference_head':
                self.reference_work=dict(generation=generation,stamp=pending['sent_at'])
                self.urgent.appendleft((START_REFERENCE,bytes(4),'start_reference'))
            elif tag=='reference_verify':
                work=self.reference_work
                if work and work['generation']==generation and now-work['stamp']<=.35:
                    self.start_reference=dict(work,source='fpga_touchscreen',is_live_pose=False)
                else:
                    self.start_reference=dict(stamp=pending['sent_at'],valid=False,reason='start_reference_changed_during_query')
                    self.next_reference=0.
                self.reference_work=None
        elif command==POSE_LATCH:
            token=data[:2]
            if not data[2]&1 or int.from_bytes(token,'little') not in range(1,65535):
                self.pose=dict(stamp=pending['sent_at'],valid=False,reason='fpga_pose_not_valid')
                self.pose_work=None
                return True
            self.pose_work=dict(stamp=pending['sent_at'],flags=data[2],task_id=data[3],token=token)
            self.urgent.extend(((POSE_XY,token+bytes(2),'pose_xy'),(POSE_ANGLE,token+bytes(2),'pose_angle')))
            if self.capability_flags&128:
                self.urgent.append((ROUTE_CURSOR,token+bytes(2),'route_cursor'))
        elif command in (POSE_XY,POSE_ANGLE,ROUTE_CURSOR):
            work=self.pose_work
            if not work or pending['data'][:2]!=work['token']:return True
            if command==POSE_XY:
                work['x_mm'],work['y_mm']=struct.unpack('<hh',data)
            elif command==POSE_ANGLE:
                work.update(heading_tenths=int.from_bytes(data[:2],'little',signed=True),mode=data[2],
                            phase=data[3]>>4,status=data[3]&15)
            else:
                generation,index=struct.unpack('<HH',data)
                if not 1<=generation<=65534 or not 0<=index<2048:
                    self.pose_work=None
                    self.pose=dict(stamp=work['stamp'],valid=False,reason='fpga_pose_cursor_invalid')
                    return True
                work.update(route_generation=generation,execution_index=index)
            complete=('x_mm' in work and 'heading_tenths' in work and
                      (not self.capability_flags&128 or 'execution_index' in work))
            if complete and 0<=now-work['stamp']<=.35:
                self.pose={k:v for k,v in work.items() if k!='token'}
                self.pose['valid']=bool(work['flags']&1)
                self.pose_completed+=1
                self.last_pose_completed_at=now
                self.last_pose_duration_ms=round((now-work['stamp'])*1000.,3)
                self.pose_work=None
        elif command==ROUTE_HEADER:
            head=self.route_header(data)
            if (not 1<=head['generation']<=65534 or not 2<=head['count']<=2048 or
                    not head['valid'] or head['planning']):
                self.route=dict(stamp=now,valid=False,**{k:v for k,v in head.items() if k!='valid'})
                self.route_work=None;self._drop_download('route')
            elif tag=='route_verify':
                work=self.route_work
                if work and work['head']==head and len(work['points'])==len(work['indices']):
                    self.route=dict(stamp=now,valid=True,generation=head['generation'],
                        total_count=head['count'],sampled=head['count']>len(work['indices']),
                        sample_indices=work['indices'],points_mm=[work['points'][i] for i in work['indices']])
                    self.route_work=None;self.reason='map_protocol_ready'
                else:
                    self.route={};self.route_work=None;self._drop_download('route');self.next_route=0.
            elif (self.route.get('valid') and self.route.get('generation')==head['generation'] and
                  self.route.get('total_count')==head['count'] and
                  (not self.full_route_required or self.route.get('sampled') is False)):
                self.route['stamp']=now
            elif (self.route_work and self.route_work['head']==head and
                  (not self.full_route_required or len(self.route_work['indices'])==head['count'])):
                pass
            else:
                self.route={};self._drop_download('route')
                n=head['count'];steps=n if self.full_route_required else min(n,self.sample_limit)
                indices=sorted(set(round(i*(n-1)/(steps-1)) for i in range(steps))) if steps>1 else list(range(n))
                self.route_work=dict(head=head,indices=indices,points={})
                self.download.extend((ROUTE_POINT,struct.pack('<HH',head['generation'],i),'route_point') for i in indices)
                self.download.append((ROUTE_HEADER,bytes(4),'route_verify'))
        elif command==ROUTE_POINT:
            work=self.route_work;generation,index=struct.unpack('<HH',pending['data'])
            x,y=struct.unpack('<HH',data)
            if work and work['head']['generation']==generation and index in work['indices']:
                if not 0<=x<=3520 or not 0<=y<=3200:
                    self.route_work=None;self.route={};self._drop_download('route')
                else:work['points'][index]=[x,y]
        elif command==BARRIER_HEADER:
            generation=int.from_bytes(data[:2],'little');mask=data[2]
            head=(generation,mask)
            if not 1<=generation<=65534 or mask&~15 or data[3]:
                self.barriers={};self.barrier_work=None;self._drop_download('barrier')
            elif tag=='barrier_verify':
                work=self.barrier_work
                if work and work['head']==head and len(work['points'])==2*len(work['slots']):
                    self.barriers=dict(stamp=now,valid=True,generation=generation,
                        segments_mm=[work['points'][(i,0)]+work['points'][(i,1)] for i in work['slots']])
                    self.barrier_work=None
                else:self.barriers={};self.barrier_work=None;self._drop_download('barrier');self.next_barriers=0.
            elif self.barriers.get('valid') and self.barriers.get('generation')==generation:
                self.barriers['stamp']=now
            elif self.barrier_work and self.barrier_work['head']==head:pass
            else:
                self.barriers={};self._drop_download('barrier')
                slots=[i for i in range(4) if mask&(1<<i)]
                self.barrier_work=dict(head=head,slots=slots,points={})
                # Eight endpoints at most; put them ahead of a long route download.
                requests=[(BARRIER_POINT,generation.to_bytes(2,'little')+bytes((i,j)),'barrier_point') for i in slots for j in (0,1)]
                self.download.extendleft(reversed(requests+[(BARRIER_HEADER,bytes(4),'barrier_verify')]))
        elif command==BARRIER_POINT:
            work=self.barrier_work;generation=int.from_bytes(pending['data'][:2],'little')
            slot,endpoint=pending['data'][2:];x,y=struct.unpack('<HH',data)
            if work and work['head'][0]==generation and slot in work['slots']:
                if not 0<=x<=3520 or not 0<=y<=3200:
                    self.barrier_work=None;self.barriers={};self._drop_download('barrier')
                else:work['points'][(slot,endpoint)]=[x,y]
        return True

    def snapshot(self, now, *, internal_read_only=False):
        """Copy dynamic evidence; optionally share immutable internal path data.

        Default callers still receive the original fully independent deep
        copy. The UART owner opts in only for its read-only consumers. A route
        is replaced atomically on verification; header refreshes mutate only
        its stamp, which is copied anew on every call along with pose/flags.
        """
        work=self.route_work
        download=dict(active=bool(work), received=len(work['points']) if work else 0,
            total=len(work['indices']) if work else 0,
            generation=work['head']['generation'] if work else None,
            complete_required=self.full_route_required)
        document=dict(stamp=self.last_reply_at,supported=self.supported,reason=self.reason,
            heading_supported=bool(self.supported and self.capability_flags&16),
            region_supported=bool(self.supported and self.capability_flags&32),
            region_preference_supported=bool(self.supported and self.capability_flags&64),
            continuous_route_supported=bool(self.supported and self.capability_flags&128),
            start_reference=self.start_reference,
            map_signature=self.map_signature,pose=self.pose,route=self.route,barriers=self.barriers,
            route_download=download,
            transport=dict(pose_completed=self.pose_completed,pose_timeouts=self.pose_timeouts,
                last_pose_completed_at=self.last_pose_completed_at,
                last_pose_duration_ms=self.last_pose_duration_ms),
            path_sampling_limit=2048 if self.full_route_required else self.sample_limit,read_only=True)
        if not internal_read_only:
            return copy.deepcopy(document)
        # Keep the source object, rather than only id(), so an allocator cannot
        # reuse an old identity and accidentally retain another route's points.
        if self.route is not self._snapshot_route_source:
            self._snapshot_route_source=self.route
            self._snapshot_route_static={key:_read_only_route_value(self.route[key])
                for key in ('points_mm','sample_indices') if key in self.route}
        route=copy.deepcopy({key:value for key,value in self.route.items()
                             if key not in ('points_mm','sample_indices')})
        route.update(self._snapshot_route_static)
        document.pop('route')
        result=copy.deepcopy(document)
        result['route']=route
        return result
