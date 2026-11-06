"""UART bridge with CRC framing, acknowledgements and expiring motion leases."""
import json
import math
import struct
import time
import secrets
from .node_common import Node
from std_msgs.msg import String
from rescue_interfaces.msg import FpgaEvent,GimbalCommand,MotionPermission,ManeuverCommand
from . import protocol as wire
from .node_common import param,age,publish_json,spin
from .manual_motion import start_guard as manual_start_guard, runtime_guard as manual_runtime_guard


class FpgaBridge(Node):
    def __init__(self):
        super().__init__('fpga_bridge')
        self.dry=param(self,'dry_run',True);self.port=param(self,'port','');self.serial=None
        self.gimbal_enabled=bool(param(self,'gimbal_enabled',True))
        self.maneuver_enabled=bool(param(self,'maneuver_enabled',True))
        self.manual_motion_enabled=bool(param(self,'manual_motion_enabled',False))
        self.manual=None;self.manual_seen=set()
        self.manual_safety={'mission':{},'actuator':{},'telemetry':{},'navigation':{}}
        self.baud=int(param(self,'baud',115200));self.parser=wire.Parser();self.gate=wire.CommandGate()
        self.sequence=0;self.pending={};self.target=None;self.acked_target=None;self.last_gimbal=None;self.link_error=None
        self.generation=0;self.gimbal_expires=0.
        self.telemetry=wire.TelemetryDecoder();self.segments={};self.segment_records={}
        self.segment_counter=secrets.randbits(30)+1
        self.estop=False;self.telemetry_time=None
        self.event_pub=self.create_publisher(FpgaEvent,'/fpga/events',20)
        self.tx_pub=self.create_publisher(String,'/fpga/tx',20);self.link_pub=self.create_publisher(String,'/fpga/link',10)
        self.telemetry_pub=self.create_publisher(String,'/fpga/telemetry',20)
        self.navigation_pub=self.create_publisher(String,'/fpga/navigation_status',20)
        self.navigation_status_seen={}
        self.navigation_identity=None
        self.navigation_latest_status=None
        self.create_subscription(MotionPermission,'/control/motion_permission',self.permission,10)
        self.create_subscription(GimbalCommand,'/control/gimbal',self.gimbal,10)
        self.create_subscription(ManeuverCommand,'/control/maneuver',self.maneuver,10)
        self.create_subscription(String,'/fpga/navigation_command',self.navigation_command,10)
        for key,topic in (('mission','/mission/state'),('actuator','/actuators/state'),('navigation','/navigation/state')):
            self.create_subscription(String,topic,lambda msg,k=key:self.manual_safety_update(k,msg),10)
        if not self.dry:
            if not self.port:raise ValueError('Set the physical UART device; no automatic serial-port selection')
            import serial
            self.serial=serial.Serial(self.port,self.baud,timeout=0,write_timeout=.05)
            self.serial.reset_input_buffer();self.serial.reset_output_buffer()
        # UART is fixed at startup. Dry-run has no receive source, so avoid an
        # otherwise empty 50 Hz callback; the permission/STOP heartbeat remains.
        self.receive_timer=(self.create_timer(.02,self.receive) if self.serial is not None else None)
        self.create_timer(.1,self.tick)
    def send(self,kind,payload,metadata=None):
        self.sequence=(self.sequence+1)&0xffff;data=wire.encode(kind,self.sequence,payload)
        if self.serial:
            try:
                if self.serial.write(data)!=len(data):raise OSError('Partial serial write')
                self.link_error=None
            except Exception as exc:
                self.link_error=str(exc);self.gate.status_time=None;self.get_logger().error(self.link_error);return None
        publish_json(self.tx_pub,dict(kind=kind,sequence=self.sequence,hex=data.hex(),dry_run=self.dry,metadata=metadata))
        return self.sequence
    def permission(self,msg):
        try:self.gate.set_permission(msg.allow_motion,msg.event_id,msg.lease_ms,time.monotonic(),age(self,msg.header.stamp))
        except ValueError as exc:self.get_logger().error(str(exc))
    def gimbal(self,msg):
        if not self.gimbal_enabled:return
        command_age=age(self,msg.header.stamp)
        if not 0<=command_age<=.25:return
        if msg.owner not in ('traffic','fire','mission_home'):return
        remaining=math.floor(msg.lease_ms-1000*command_age)
        if remaining<1:return
        try:payload=wire.gimbal_payload(msg.pan_deg,msg.tilt_deg,msg.speed_deg_s,remaining)
        except ValueError as exc:self.get_logger().error(str(exc));return
        target=(round(msg.pan_deg,2),round(msg.tilt_deg,2))
        if target!=self.target:self.acked_target=None;self.target=target;self.generation+=1
        self.last_gimbal=time.monotonic()
        self.gimbal_expires=self.last_gimbal+msg.lease_ms/1000-age(self,msg.header.stamp)
        seq=self.send(wire.GIMBAL,payload,dict(owner=msg.owner,pan_deg=msg.pan_deg,tilt_deg=msg.tilt_deg))
        if seq is not None:
            self.pending[seq]=(target,self.last_gimbal,self.generation)
            self.pending={s:r for s,r in self.pending.items() if time.monotonic()-r[1]<2.0}
    def maneuver(self,msg):
        if not self.maneuver_enabled:return
        # Legacy segment commands must never compete with the current manual
        # goal, even when a non-task launch leaves maneuver_enabled at default.
        # A pause/cancel request alone does not release ownership: require the
        # FPGA's terminal status and measured stopped confirmation.
        if self.manual and not self.manual.get('terminal_stopped'):return
        delay=age(self,msg.header.stamp)
        remaining=math.floor(msg.lease_ms-delay*1000)
        if not 0<=delay<=.25 or remaining<1 or not self.gate.current(time.monotonic())['allowed'] or self.estop:return
        if self.telemetry_time is None or time.monotonic()-self.telemetry_time>.3:return
        key=(msg.command_id,msg.route_revision,msg.edge_id)
        if not msg.command_id or not msg.edge_id:return
        if key not in self.segments:
            if len(self.segments)>4096:self.get_logger().error('Maneuver history full; stop and restart mission');return
            self.segment_counter=(self.segment_counter+1)&0xffffffff or 1
            self.segments[key]=self.segment_counter
            self.segment_records[self.segment_counter]=key
        try:payload=wire.maneuver_payload(self.segments[key],msg.turn,msg.stop_at_end,msg.speed_limit_m_s,msg.distance_m,remaining)
        except ValueError as exc:self.get_logger().error(str(exc));return
        self.send(wire.MANEUVER,payload,dict(command_id=msg.command_id,edge_id=msg.edge_id,route_revision=msg.route_revision,
            segment_id=self.segments[key],same_id_renews_only=True))
    def navigation_command(self,msg):
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict):raise ValueError('Navigation command must be object')
            stamp=data.get('stamp');now=self.get_clock().now().nanoseconds/1e9
            if isinstance(stamp,bool) or not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or not 0<=now-stamp<=.3:
                raise ValueError('Stale navigation command')
            session,goal=data.get('session_id'),data.get('goal_id')
            action=data.get('action')
            identity=(session,goal)
            # A goal may be accepted while STOP is held. FPGA must never turn goal acceptance into permission to drive.
            if action=='navigate':
                if self.manual and not self.manual.get('terminal_stopped'):
                    raise ValueError('Cancel manual movement and await stopped confirmation before navigation')
                payload=wire.navigation_goal_payload(session,goal,int(data['building_number']),data['arrival_policy'])
                kind=wire.NAV_GOAL
            elif action=='manual_move':
                if not self.manual_motion_enabled or self.dry:
                    raise ValueError('manual_motion_not_enabled_on_real_uart')
                payload=wire.manual_move_payload(session,goal,data)
                kind=wire.MANUAL_MOVE
                if identity in self.manual_seen:
                    if not self.manual or self.manual['identity']!=identity or self.manual['payload']!=payload:
                        raise ValueError('Repeated manual identity cannot change or restart movement')
                    if self.manual.get('cancelled') or self.manual.get('terminal_stopped') or now>=self.manual['deadline']:
                        raise ValueError('Manual movement already stopped or expired')
                else:
                    if len(self.manual_seen)>=4096:
                        raise ValueError('Manual history full; restart while stopped')
                    if self.navigation_identity is not None:
                        previous=self.navigation_latest_status or {}
                        if (previous.get('state') not in ('arrived','cancelled','failed') or
                                previous.get('vehicle_stopped') is not True or
                                not 0<=now-previous.get('stamp',0)<=.3):
                            raise ValueError('Previous FPGA goal must be stopped and terminal before manual movement')
                    # The navigation owner has already accepted this new identity,
                    # so independently check the remaining physical preconditions.
                    safety=dict(self.manual_safety)
                    safety['navigation']={'stamp':now,'mode':'preview'}
                    reason=manual_start_guard(now,**safety,link=dict(stamp=now,dry_run=self.dry,
                        status_fresh=self.gate.status_time is not None and 0<=time.monotonic()-self.gate.status_time<.5,
                        manual_motion_enabled=self.manual_motion_enabled,error=self.link_error))
                    if reason:raise ValueError(reason)
                    deadline=data.get('deadline')
                    if type(deadline) not in (int,float) or not math.isfinite(deadline) or not now<deadline<=now+data['max_duration_s']:
                        raise ValueError('Invalid manual deadline')
                    self.manual=dict(identity=identity,payload=payload,deadline=deadline,cancelled=False,
                        terminal_stopped=False,status=None,cancel_sent=None)
                    self.manual_seen.add(identity)
            else:
                if self.manual and identity==self.manual['identity']:
                    if action=='resume':raise ValueError('Manual movement cannot resume')
                    if action in ('pause','cancel'):self.manual['cancelled']=True
                payload=wire.navigation_control_payload(session,goal,action);kind=wire.NAV_CONTROL
            sequence=self.send(kind,payload,dict(action=action,session_id=session,goal_id=goal,
                building_id=data.get('building_id'),building_number=data.get('building_number'),
                arrival_policy=data.get('arrival_policy'),motion_permission_separate=True))
            if action=='navigate' and sequence is not None:
                self.navigation_identity=(session,goal,int(data['building_number']))
                self.navigation_latest_status=None
                self.manual=None
            elif action=='manual_move' and sequence is not None:
                self.navigation_identity=(session,goal,0)
        except (ValueError,TypeError,KeyError) as exc:
            self.get_logger().warning('FPGA navigation command rejected: '+str(exc))
    def manual_safety_update(self,key,msg):
        try:
            data=json.loads(msg.data)
            self.manual_safety[key]=data if isinstance(data,dict) else {}
        except (ValueError,TypeError):self.manual_safety[key]={}

    def manual_permission_guard(self,now):
        """Hard stop latches; retransmitting a goal never extends its deadline."""
        if not self.manual:return ''
        m=self.manual
        reason=manual_runtime_guard(now,**{k:v for k,v in self.manual_safety.items() if k!='navigation'})
        status=m.get('status') or {}
        if now>=m['deadline']:reason='manual_deadline_expired'
        if not status or not 0<=now-status.get('stamp',0)<=.3:
            reason=reason or 'manual_direction_status_unavailable'
        elif status.get('obstacle_clear') is not True or status.get('estop') or status.get('error_code'):
            reason=reason or 'manual_direction_blocked'
        elif status.get('state') not in ('accepted','navigating'):
            reason=reason or 'manual_not_running'
        if m.get('cancelled') or m.get('terminal_stopped'):
            return reason or 'manual_stopped'
        # Before the first acceptance, just hold STOP. Other failures latch cancel.
        if reason and (reason!='manual_direction_status_unavailable' or status):
            m['cancelled']=True
            self.send(wire.NAV_CONTROL,wire.navigation_control_payload(*m['identity'],'cancel'),
                dict(reason=reason,motion_kind='manual'))
        return reason
    def receive(self):
        if self.serial is None:return
        try:data=self.serial.read(min(self.serial.in_waiting,2048))
        except Exception as exc:self.link_error=str(exc);self.gate.status_time=None;return
        for packet in self.parser.feed(data):
            if packet.kind in (wire.NAV_STATUS,wire.MANUAL_STATUS):
                try:
                    decoder=wire.manual_status if packet.kind==wire.MANUAL_STATUS else wire.navigation_status
                    status=decoder(packet,self.get_clock().now().nanoseconds/1e9)
                    if (status['session_id'],status['goal_id'],status['building_number'])!=self.navigation_identity:continue
                    key=(status['session_id'],status['goal_id'])
                    previous=self.navigation_status_seen.get(key)
                    current=status['status_sequence']
                    if previous is not None and not 0<((current-previous)&0xffffffff)<0x80000000:continue
                    self.navigation_status_seen[key]=current
                    self.navigation_latest_status=status
                    if len(self.navigation_status_seen)>4096:self.navigation_status_seen={key:current}
                    self.estop=status['estop'];self.telemetry_time=time.monotonic()
                    self.gate.observe_status(time.monotonic())
                    if packet.kind==wire.MANUAL_STATUS and self.manual:
                        self.manual['status']=status
                        if status['state'] in ('arrived','cancelled','failed') and status['vehicle_stopped']:
                            self.manual['terminal_stopped']=True
                    publish_json(self.navigation_pub,status)
                except ValueError as exc:self.get_logger().warning(str(exc))
                continue
            if packet.kind==wire.TELEMETRY:
                try:
                    data=self.telemetry.decode(packet,self.get_clock().now().nanoseconds/1e9)
                    if data is None:continue
                    self.estop=data['estop'];self.telemetry_time=time.monotonic()
                    self.manual_safety['telemetry']=data
                    record=self.segment_records.get(data['maneuver_done_id'])
                    data.update(maneuver_done=record is not None,maneuver_command_id=record[0] if record else None,
                        maneuver_edge_id=record[2] if record else None,maneuver_route_revision=record[1] if record else None)
                    publish_json(self.telemetry_pub,data)
                except ValueError as exc:self.get_logger().warning(str(exc))
                continue
            try:d=wire.event(packet)
            except ValueError as exc:self.get_logger().warning(str(exc));continue
            if packet.kind==wire.STATUS:self.gate.observe_status(time.monotonic())
            if packet.kind==wire.ACK:
                record=self.pending.pop(d['acknowledged_sequence'],None)
                if record and record[0]==self.target and record[2]==self.generation and time.monotonic()-record[1]<1.0:
                    self.acked_target=record[0] if d['acknowledgement_result']==0 else None
            msg=FpgaEvent();msg.header.stamp=self.get_clock().now().to_msg()
            for key,value in d.items():setattr(msg,key,value)
            self.event_pub.publish(msg)
    def tick(self):
        now=time.monotonic();cmd=self.gate.current(now)
        stamp=self.get_clock().now().nanoseconds/1e9
        manual_reason=self.manual_permission_guard(stamp)
        if manual_reason:cmd.update(allowed=False,reason=manual_reason)
        if self.estop:cmd.update(allowed=False,reason='fpga_estop')
        self.send(wire.PERMISSION,wire.motion_payload(cmd['event_id'],cmd['allowed'],cmd['lease_ms']),cmd)
        fresh=self.gate.status_time is not None and now-self.gate.status_time<.5
        ack=self.target is not None and self.acked_target==self.target and fresh and now<self.gimbal_expires
        publish_json(self.link_pub,dict(stamp=stamp,dry_run=self.dry,status_fresh=fresh,gimbal_acknowledged=ack,
            manual_motion_enabled=self.manual_motion_enabled,manual_stop_reason=manual_reason,
            target_deg=list(self.target) if self.target else None,error=self.link_error,crc_rejects=self.parser.bad_frames))
    def destroy_node(self):
        if self.serial:
            self.send(wire.PERMISSION,wire.motion_payload(0,False,300),{'reason':'shutdown'})
            self.serial.close()
        return super().destroy_node()

def main():spin(FpgaBridge)
