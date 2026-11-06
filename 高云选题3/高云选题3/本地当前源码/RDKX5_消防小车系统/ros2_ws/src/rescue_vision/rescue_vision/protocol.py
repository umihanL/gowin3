"""UART v1: A5 5A | version:u8 type:u8 seq:u16 len:u16 | payload | CRC16.

Little endian, CRC16/CCITT-FALSE over version through payload, max payload 64.
This is a proposed explicit protocol; FPGA firmware must implement it too.
"""
from dataclasses import dataclass
import math
import struct

CROSSWALK, CROSSING_COMPLETE, STATUS, ACK = 0x10,0x11,0x12,0x13
PERMISSION,GIMBAL,HEARTBEAT = 0x80,0x81,0x83
TELEMETRY,MANEUVER=0x20,0x84
NAV_STATUS,NAV_GOAL,NAV_CONTROL=0x21,0x85,0x86
MANUAL_STATUS,MANUAL_MOVE=0x22,0x87
NAV_STATUS_FORMAT='<IIIBBHH'
NAV_STATES={0:'idle',1:'accepted',2:'navigating',3:'paused',4:'arrived',5:'cancelled',6:'failed'}
NAV_ACTIONS={'cancel':1,'pause':2,'resume':3}
TELEMETRY_FORMAT='<IihhhHhHBII'
LENGTHS={CROSSWALK:4,CROSSING_COMPLETE:4,STATUS:8,ACK:3,PERMISSION:7,GIMBAL:8,HEARTBEAT:4,TELEMETRY:29,MANEUVER:14}
LENGTHS.update({NAV_STATUS:18,NAV_GOAL:13,NAV_CONTROL:11})
LENGTHS.update({MANUAL_STATUS:18,MANUAL_MOVE:17})


def crc16(data):
    crc=0xffff
    for value in data:
        crc ^= value << 8
        for _ in range(8):crc=((crc<<1)^0x1021)&0xffff if crc&0x8000 else (crc<<1)&0xffff
    return crc


def encode(kind,sequence,payload):
    if kind not in LENGTHS or len(payload)!=LENGTHS[kind]:raise ValueError('Unknown type or incorrect payload size')
    body=struct.pack('<BBHH',1,kind,sequence&0xffff,len(payload))+payload
    return b'\xa5\x5a'+body+struct.pack('<H',crc16(body))


@dataclass
class Packet:
    kind:int
    sequence:int
    payload:bytes


class Parser:
    def __init__(self):self.buffer=bytearray();self.bad_frames=0
    def feed(self,data):
        self.buffer.extend(data);out=[]
        if len(self.buffer)>8192:self.buffer=self.buffer[-8192:]
        while len(self.buffer)>=2:
            pos=self.buffer.find(b'\xa5\x5a')
            if pos<0:
                self.buffer=self.buffer[-1:] if self.buffer[-1]==0xa5 else bytearray();break
            if pos:del self.buffer[:pos]
            if len(self.buffer)<8:break
            version,kind,sequence,length=struct.unpack_from('<BBHH',self.buffer,2)
            if version!=1 or length>64 or LENGTHS.get(kind)!=length:
                del self.buffer[0];self.bad_frames+=1;continue
            total=10+length
            if len(self.buffer)<total:break
            if crc16(self.buffer[2:8+length])!=struct.unpack_from('<H',self.buffer,8+length)[0]:
                del self.buffer[0];self.bad_frames+=1;continue
            out.append(Packet(kind,sequence,bytes(self.buffer[8:8+length])))
            del self.buffer[:total]
        return out


def event(packet):
    if packet.kind in (CROSSWALK,CROSSING_COMPLETE):
        return dict(event_type=packet.kind,wire_sequence=packet.sequence,event_id=struct.unpack('<I',packet.payload)[0])
    if packet.kind==STATUS:
        flags,pan,tilt,reserved=struct.unpack('<HhhH',packet.payload)
        if reserved!=0 or flags&~7:raise ValueError('Unsupported STATUS flags/reserved field')
        return dict(event_type=STATUS,wire_sequence=packet.sequence,vehicle_stopped=bool(flags&1),
                    gimbal_settled=bool(flags&2),measured_angles_valid=bool(flags&4),pan_deg=pan/100,tilt_deg=tilt/100)
    if packet.kind==ACK:
        seq,result=struct.unpack('<HB',packet.payload)
        return dict(event_type=ACK,wire_sequence=packet.sequence,acknowledged_sequence=seq,acknowledgement_result=result)
    raise ValueError('Host received an outbound-only packet type')


def motion_payload(event_id,allowed,lease_ms):
    if not 0<lease_ms<=1000:raise ValueError('Motion lease must be 1..1000ms')
    return struct.pack('<IBH',event_id,bool(allowed),lease_ms)


def gimbal_payload(pan,tilt,speed,lease_ms):
    if not all(math.isfinite(x) for x in (pan,tilt,speed)) or not (0<=pan<=180 and 0<=tilt<=180 and 0<speed<=180):
        raise ValueError('Invalid gimbal command')
    if not 0<lease_ms<=1000:raise ValueError('Gimbal lease must be 1..1000ms')
    return struct.pack('<hhHH',round(pan*100),round(tilt*100),round(speed*100),lease_ms)


class CommandGate:
    """Bridge watchdog: a GO lease needs fresh ROS command and fresh MCU status."""
    def __init__(self,status_timeout=.5,max_command_age=.25):
        self.status_timeout=status_timeout;self.max_command_age=max_command_age
        self.status_time=None;self.permission=None;self.command_time=None;self.expires=None
    def observe_status(self,now):self.status_time=now
    def set_permission(self,allowed,event_id,lease_ms,now,age):
        if not 0<=age<=self.max_command_age:
            self.permission=None;return False
        motion_payload(event_id,allowed,lease_ms)
        self.permission=(bool(allowed),event_id,lease_ms);self.command_time=now
        self.expires=now+lease_ms/1000-age
        return True
    def current(self,now):
        if self.permission is None:return dict(allowed=False,event_id=0,lease_ms=300,reason='no_command')
        allowed,eid,lease=self.permission
        if self.status_time is None or not 0<=now-self.status_time<=self.status_timeout:allowed=False;reason='stale_fpga_status'
        elif now>=self.expires or not 0<=now-self.command_time<=self.max_command_age:allowed=False;reason='stale_command'
        else:reason='fresh_command'
        if allowed:
            lease=math.floor(1000*min(self.expires-now,self.status_time+self.status_timeout-now))
            if lease<1:allowed=False;reason='permission_expired'
        return dict(allowed=allowed,event_id=eid,lease_ms=max(1,lease) if allowed else 250,reason=reason)


def maneuver_payload(segment_id,turn,stop_at_end,speed_m_s,distance_m,lease_ms):
    if not (0<segment_id<=0xffffffff and turn in (0,1,2,3) and 0<lease_ms<=1000):raise ValueError('Invalid maneuver identity/type/lease')
    if not all(math.isfinite(v) for v in (speed_m_s,distance_m)) or not (.01<=speed_m_s<=1.0 and .01<=distance_m<=100):
        raise ValueError('Maneuver speed/range outside configured protocol limits')
    return struct.pack('<IBBHIH',segment_id,turn,int(stop_at_end),round(speed_m_s*1000),round(distance_m*1000),lease_ms)


def navigation_identity(session_id,goal_id):
    if not all(isinstance(x,int) and not isinstance(x,bool) and 0<x<=0xffffffff for x in (session_id,goal_id)):
        raise ValueError('Invalid navigation session/goal identity')


def navigation_goal_payload(session_id,goal_id,building_number,arrival_policy,lease_ms=1000):
    navigation_identity(session_id,goal_id)
    if not isinstance(building_number,int) or isinstance(building_number,bool) or not 1<=building_number<=65535:
        raise ValueError('Invalid building number')
    if arrival_policy not in ('manual','auto') or not isinstance(lease_ms,int) or not 0<lease_ms<=1000:
        raise ValueError('Invalid arrival policy/acceptance lease')
    return struct.pack('<IIHBH',session_id,goal_id,building_number,int(arrival_policy=='auto'),lease_ms)


def navigation_control_payload(session_id,goal_id,action,lease_ms=1000):
    navigation_identity(session_id,goal_id)
    if action not in NAV_ACTIONS or not isinstance(lease_ms,int) or not 0<lease_ms<=1000:
        raise ValueError('Invalid navigation control/lease')
    return struct.pack('<IIBH',session_id,goal_id,NAV_ACTIONS[action],lease_ms)


def manual_move_payload(session_id,goal_id,request,lease_ms=1000):
    """Proposed signed straight segment. Acceptance never grants GO permission."""
    from .manual_motion import validate
    navigation_identity(session_id,goal_id)
    motion=validate(request)
    if type(lease_ms) is not int or not 0<lease_ms<=1000:
        raise ValueError('Invalid manual acceptance window')
    return struct.pack('<IIbHHHH',session_id,goal_id,
        1 if motion['direction']=='forward' else -1,round(motion['speed_m_s']*1000),
        round(motion['distance_m']*1000),round(motion['max_duration_s']*1000),lease_ms)


def manual_status(packet,stamp):
    """bit2 is clearance in the accepted movement direction, including rear sensing."""
    if packet.kind!=MANUAL_STATUS:raise ValueError('Not MANUAL_STATUS')
    session,goal,sequence,state,flags,reserved,error=struct.unpack(NAV_STATUS_FORMAT,packet.payload)
    navigation_identity(session,goal)
    if state not in NAV_STATES or flags&~7 or reserved!=0:
        raise ValueError('Invalid manual status flags/state/reserved field')
    return dict(stamp=stamp,session_id=session,goal_id=goal,status_sequence=sequence,
        state=NAV_STATES[state],vehicle_stopped=bool(flags&1),estop=bool(flags&2),
        obstacle_clear=bool(flags&4),motion_kind='manual',building_number=0,
        error_code=error,wire_sequence=packet.sequence)


def navigation_status(packet,stamp):
    if packet.kind!=NAV_STATUS:raise ValueError('Not NAV_STATUS')
    session,goal,sequence,state,flags,building,error=struct.unpack(NAV_STATUS_FORMAT,packet.payload)
    navigation_identity(session,goal)
    if state not in NAV_STATES or flags&~3 or building<1:
        raise ValueError('Invalid navigation status flags/state/building')
    return dict(stamp=stamp,session_id=session,goal_id=goal,status_sequence=sequence,
                state=NAV_STATES[state],vehicle_stopped=bool(flags&1),estop=bool(flags&2),
                building_number=building,error_code=error,wire_sequence=packet.sequence)


class TelemetryDecoder:
    """Cumulative encoder avoids integrating duplicate or lost-packet deltas."""
    def __init__(self):self.previous=None
    def decode(self,packet,stamp):
        if packet.kind!=TELEMETRY:raise ValueError('Not a telemetry packet')
        ms,encoder_mm,rate,yaw,line,obstacle,speed,flags,junction,junction_seq,done=struct.unpack(TELEMETRY_FORMAT,packet.payload)
        if flags&~127 or junction not in range(5):raise ValueError('Unsupported telemetry flags/junction')
        current=(ms,encoder_mm);delta=0.;valid=False;reason='initial_packet'
        if self.previous:
            dt=(ms-self.previous[0])&0xffffffff
            delta_mm=encoder_mm-self.previous[1]
            if dt==0:return None
            if dt>0x80000000:reason='mcu_clock_reset_or_out_of_order'
            elif dt>300:reason='telemetry_gap'
            elif abs(delta_mm)>(dt*2.+10):reason='implausible_encoder_jump'
            else:valid=bool(flags&1);delta=delta_mm/1000.;reason='ok' if valid else 'encoder_invalid'
        self.previous=current
        return dict(stamp=stamp,mcu_time_ms=ms,wire_sequence=packet.sequence,odometry_valid=valid,
            delta_s_m=delta if valid else 0.,encoder_distance_m=encoder_mm/1000.,yaw_rate_rad_s=rate/1000.,
            gyro_valid=bool(flags&64),yaw_valid=bool(flags&2),yaw_rad=yaw/1000.,line_valid=bool(flags&4),
            line_error_m=line/1000.,obstacle_distance_m=obstacle/1000. if flags&8 and obstacle!=65535 else None,
            speed_m_s=speed/1000.,estop=bool(flags&16),vehicle_stopped=bool(flags&32),junction_type=junction,
            junction_seq=junction_seq,maneuver_done_id=done,odometry_reason=reason)
