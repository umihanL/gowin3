"""One owner of gimbal commands and motion permission; traffic preempts aiming."""
import json
import math
import re
from collections import deque, OrderedDict
import time
from cv_bridge import CvBridge
from .node_common import Node
from std_msgs.msg import String
from rescue_interfaces.msg import StereoPair,FpgaEvent,GimbalCommand,MotionPermission,ManeuverCommand
from .node_common import IMAGE_QOS,param,read_json,age,publish_json,spin
from .traffic import TrafficDetector,TrafficController
from .auto_fire_gate import AutoArmFence


class Mission(Node):
    def __init__(self):
        super().__init__('mission');self.bridge=CvBridge()
        traffic=read_json(param(self,'traffic_config',''))
        self.detector=TrafficDetector(traffic);self.traffic=TrafficController(traffic)
        self.mount=read_json(param(self,'mount_config',''))
        self.task_profiles=bool(param(self,'task_profiles_enabled',False))
        self.fixed_camera_observation=bool(param(self,'fixed_camera_observation',False))
        self.extension_enabled=bool(param(self,'extension_enabled',False))
        self.extension={};self.extension_received=None
        self.navigation_owner=str(param(self,'navigation_owner','x5'))
        if self.navigation_owner not in ('x5','fpga'):raise ValueError('invalid navigation owner')
        self.arrival_policy=str(param(self,'arrival_policy','manual'))
        if self.arrival_policy not in ('manual','auto','scan'):raise ValueError('invalid arrival policy')
        self.auto_fire={};self.auto_received=None
        self.auto_authority=AutoArmFence()
        self.camera_pose='unconfigured'
        self.local_traffic_id=0x80000000
        self.traffic_requests=deque(maxlen=256)
        self.gimbal_transport=str(param(self,'gimbal_transport','fpga'))
        if self.gimbal_transport not in ('fpga','x5'):raise ValueError('gimbal_transport must be fpga or x5')
        self.mode=str(param(self,'initial_mode','idle'));self.observation=None;self.fpga=None;self.fpga_time=None;self.link={};self.link_time=None
        # Explicit inspect startup enables real vision, while all stopped,
        # mounting and actuator gates remain required for physical movement.
        if self.mode not in ('idle','standby','inspect'):raise ValueError('initial_mode must be idle/standby/inspect')
        self.observation_pose_id=None
        self.operator_hold=False
        self.navigation=None;self.nav_received=None;self.arrival_notified=None
        self.navigation_command_id=None;self.navigation_requested_at=0.;self.arrivals_seen=OrderedDict()
        self.scan_hold_run=None;self.scan_motion_allowed=False;self.scan_wait_reason='not_requested'
        self.target=None;self.move_end=0.;self.fire=None;self.fire_time=None
        self.state_pub=self.create_publisher(String,'/mission/state',10)
        self.permission_pub=self.create_publisher(MotionPermission,'/control/motion_permission',10)
        self.gimbal_pub=self.create_publisher(GimbalCommand,'/control/gimbal',10)
        self.maneuver_pub=self.create_publisher(ManeuverCommand,'/control/maneuver',10)
        self.speech_pub=self.create_publisher(String,'/speech/say',10)
        self.traffic_pub=self.create_publisher(String,'/traffic/state',10)
        self.auto_pub=self.create_publisher(String,'/fire/auto/request',10)
        self.create_subscription(FpgaEvent,'/fpga/events',self.event,20)
        self.create_subscription(String,'/actuators/state' if self.gimbal_transport=='x5' else '/fpga/link',self.link_state,10)
        self.create_subscription(String,'/mission/request',self.request,10)
        self.create_subscription(String,'/fire/proposal',self.fire_proposal,10)
        self.create_subscription(String,'/navigation/proposal',self.navigation_proposal,10)
        self.create_subscription(String,'/navigation/state',self.navigation_state,10)
        self.create_subscription(String,'/fire/auto/state',self.auto_state,10)
        self.create_subscription(String,'/fire/auto/request',self.auto_request,10)
        if self.extension_enabled:
            self.create_subscription(String,'/extension/state',self.extension_state,10)
        if self.task_profiles:
            self.create_subscription(String,'/traffic/observation',self.traffic_observation,10)
        # Task profiles use road_vision's observations. Their image callback is
        # inactive, so avoid registering an unused reader for full stereo pairs.
        if not self.task_profiles:
            self.create_subscription(StereoPair,'/stereo/pair',self.image,IMAGE_QOS)
        self.create_timer(.1,self.tick)
    def event(self,msg):
        now=time.monotonic()
        if not 0<=age(self,msg.header.stamp)<=.5:return
        if msg.event_type==FpgaEvent.STATUS:self.fpga=msg;self.fpga_time=now
        elif msg.event_type==FpgaEvent.CROSSWALK:
            if self.extension_enabled and self.extension.get('task')!='traffic':return
            if not self.task_profiles or self.mode in ('standby','patrol','navigate'):
                self.begin_crosswalk(msg.event_id,now)
        elif msg.event_type==FpgaEvent.CROSSING_COMPLETE:
            if self.traffic.on_crossing_complete(msg.event_id):
                self.target=None;self.observation=None;self.camera_pose='unconfigured'
    def begin_crosswalk(self,event_id,now):
        # Wire-compatible name: FPGA CROSSWALK now denotes a route intersection.
        # Local camera pixels no longer generate this event.
        if self.traffic.on_crosswalk(event_id,now):
            self.observation=None;self.target=None;self.fire=None;self.observation_pose_id=None
            self.camera_pose='traffic'
            self.publish_permission(False,event_id,'crosswalk_stop_requested')
    def extension_state(self,msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict):raise ValueError('expected object')
            previous=self.extension.get('task')
            self.extension=value;self.extension_received=time.monotonic()
            if value.get('task')=='traffic' and previous!='traffic':
                self.local_traffic_id=(self.local_traffic_id+1)&0xffffffff or 0x80000001
                self.mode='standby';self.operator_hold=False
                self.begin_crosswalk(self.local_traffic_id,time.monotonic())
        except (ValueError,TypeError):self.extension={};self.extension_received=None
    def extension_fresh(self,now):
        return bool(self.extension_enabled and self.extension_received is not None and
                    0<=now-self.extension_received<=.5 and
                    0<=self.get_clock().now().nanoseconds/1e9-self.extension.get('stamp',0)<=.5)
    def traffic_observation(self,msg):
        try:
            value=json.loads(msg.data);elapsed=self.get_clock().now().nanoseconds/1e9-float(value['capture_ros_s'])
            if value.get('observation_only') is True or value.get('control_eligible') is False:return
            if not 0<=elapsed<=.3 or self.traffic.event_id is None:return
            value['capture_time']=time.monotonic()-elapsed
            self.observation=value
        except (ValueError,TypeError,KeyError):self.observation=None
    def link_state(self,msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict):raise ValueError('Link status must be an object')
            if self.gimbal_transport=='x5':
                stamp=value.get('stamp')
                delay=self.get_clock().now().nanoseconds/1e9-stamp if isinstance(stamp,(int,float)) else math.inf
                if not 0<=delay<=.5:raise ValueError('Stale local actuator status')
            self.link=value;self.link_time=time.monotonic()
        except ValueError:self.link={};self.link_time=None
    def request(self,msg):
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict):raise ValueError('Mission request must be an object')
            mode=data.get('mode')
            auto_run=data.get('auto_run_id')
            if auto_run is not None:
                stamp=self.get_clock().now().nanoseconds/1e9
                if mode=='standby' and auto_run==self.auto_authority.active_run:
                    self.auto_authority.retire(auto_run)
                elif not self.auto_authority.authorized(auto_run,stamp):
                    raise ValueError('Retired automatic mission request')
            if 'arrival_policy' in data:
                if data['arrival_policy'] not in ('auto','manual','scan'):raise ValueError('invalid arrival policy')
                self.arrival_policy=data['arrival_policy']
                if mode is None:return
            if mode=='traffic':
                # Explicit intersection observation replaces the removed zebra
                # vision trigger. A request never changes standby into driving.
                request_id=data.get('request_id');issued=data.get('issued_at');ttl=data.get('ttl_s')
                wall=self.get_clock().now().nanoseconds/1e9
                if (not isinstance(request_id,str) or not re.fullmatch(r'[A-Za-z0-9_:.-]{1,100}',request_id)
                        or type(issued) not in (int,float) or type(ttl) not in (int,float)
                        or not math.isfinite(issued) or not math.isfinite(ttl)
                        or not 0<ttl<=5 or not 0<=wall-issued<=ttl):
                    raise ValueError('invalid_or_expired_traffic_request')
                if request_id in self.traffic_requests:return
                self.traffic_requests.append(request_id)
                if self.traffic.event_id is not None:return
                if self.navigation_fresh(time.monotonic()) and (self.navigation or {}).get('estop'):
                    raise ValueError('fpga_estop')
                if self.mode not in ('patrol','navigate'):self.mode='standby'
                self.operator_hold=False
                self.local_traffic_id=(self.local_traffic_id+1)&0xffffffff or 0x80000001
                self.begin_crosswalk(self.local_traffic_id,time.monotonic())
                return
            if mode=='reset_traffic':
                now=time.monotonic()
                if self.stopped(now):
                    event_id=self.traffic.event_id
                    self.traffic.reset();self.mode='idle';self.operator_hold=True;self.target=None
                    self.observation=None;self.observation_pose_id=None;self.fire=None
                    self.camera_pose='unconfigured'
                    self.publish_permission(False,event_id,'operator_reset_traffic')
                return
            if mode not in ('idle','standby','patrol','inspect','fire','navigate'):raise ValueError('invalid mission mode')
            if mode=='navigate' and data.get('command_id') is not None:
                cid=data['command_id']
                if not isinstance(cid,str) or not 1<=len(cid)<=100:raise ValueError('invalid_navigation_command_id')
                if cid!=self.navigation_command_id:
                    self.navigation=None;self.nav_received=None
                    self.navigation_command_id=cid
                    self.navigation_requested_at=self.get_clock().now().nanoseconds/1e9
                self.scan_hold_run=None
            elif auto_run is None and mode in ('idle','standby','inspect','fire'):
                self.scan_hold_run=None
            if mode=='standby' and self.traffic.event_id is not None:
                # Task completion cancels a pending crossing without granting GO.
                self.publish_permission(False,self.traffic.event_id,'task_finished_standby')
                self.traffic.reset()
                self.target=None;self.camera_pose='unconfigured'
            if mode!=self.mode:
                # Preserve the observation pose during inspect -> selected fire.
                if not (self.mode=='inspect' and mode=='fire'):self.target=None
                self.fire=None;self.observation=None;self.observation_pose_id=None
            self.mode=mode
            self.operator_hold=mode=='idle'
            if mode not in ('patrol','navigate'):
                reason=('crossing_pending_observation_hold' if self.traffic.event_id is not None
                        else 'operator_hold' if self.operator_hold else 'mission_'+mode)
                self.publish_permission(False,self.traffic.event_id,reason)
            if mode=='inspect' and self.stopped(time.monotonic()):self.observation_pose_id=data.get('observation_pose_id')
        except (ValueError,TypeError) as exc:self.get_logger().warning(str(exc))
    def fire_proposal(self,msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict):raise ValueError('Fire proposal must be an object')
            self.fire=value;self.fire_time=time.monotonic()
        except ValueError:self.fire=None
    def navigation_proposal(self,msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict):raise ValueError('Navigation proposal must be an object')
            if self.navigation_owner=='fpga' and value.get('owner')!='fpga':raise ValueError('Wrong navigation owner')
            if self.navigation_command_id is not None and value.get('command_id')!=self.navigation_command_id:return
            self.navigation=value;self.nav_received=time.monotonic()
        except ValueError:self.navigation=None
    def navigation_state(self,msg):
        # Numbered FPGA completion must belong to the currently requested job.
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict) or value.get('owner')!='fpga':return
            cid=value.get('command_id')
            if not isinstance(cid,str) or cid!=self.navigation_command_id:return
            stamp=value.get('stamp');wall=self.get_clock().now().nanoseconds/1e9
            if type(stamp) not in (int,float) or not math.isfinite(stamp) or not 0<=wall-stamp<=.3:return
            if value.get('phase') in ('failed','cancelled','fault','rejected','expired'):
                self.remember_arrival(cid)
            self.navigation=value;self.nav_received=time.monotonic()
        except (ValueError,TypeError):return
    def remember_arrival(self,cid):
        self.arrivals_seen[cid]=self.get_clock().now().nanoseconds/1e9
        while len(self.arrivals_seen)>256:self.arrivals_seen.popitem(last=False)
    def arrival_eligible(self,now):
        n=self.navigation or {};cid=n.get('command_id')
        if (not isinstance(cid,str) or not cid or cid in self.arrivals_seen or
                not self.navigation_fresh(now) or n.get('arrived') is not True or n.get('valid') is not True or
                not self.stopped(now) or n.get('estop') or
                n.get('phase') in ('failed','cancelled','fault','rejected','expired')):return False
        if n.get('motion_kind') in ('numbered_point', 'semantic_region'):
            if n.get('arrival_policy') not in ('manual','scan'):return False
            if n.get('task_profile')=='extension1' and n.get('arrival_policy')!='manual':return False
            if n.get('task_profile')=='extension2' and n.get('arrival_policy')!='manual':return False
            stamp=n.get('arrival_stamp');wall=self.get_clock().now().nanoseconds/1e9
            return bool(cid==self.navigation_command_id and n.get('status_fresh') is True and
                n.get('vehicle_stopped') is True and type(stamp) in (int,float) and math.isfinite(stamp) and
                self.navigation_requested_at<=stamp<=wall and wall-stamp<=2.)
        return self.navigation_command_id is None or cid==self.navigation_command_id
    def auto_state(self,msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict):raise ValueError('expected object')
            self.auto_fire=value;self.auto_received=time.monotonic()
        except (ValueError,TypeError):self.auto_fire={};self.auto_received=None
    def auto_request(self,msg):
        try:
            data=json.loads(msg.data)
            self.auto_authority.request(data,self.get_clock().now().nanoseconds/1e9)
            if data.get('action')=='scan':self.scan_hold_run=data['command_id']
        except (ValueError,TypeError):pass
    def scan_hardware_status(self,now):
        if not self.stopped(now):return False,'scan_waiting_for_measured_vehicle_stop'
        # A body-fixed camera observes the current view. Rotating the nozzle
        # does not reveal any new pixels and requires no observation authority.
        if self.fixed_camera_observation:return True,'fixed_camera_observation_ready'
        if not self.mount.get('mounting_verified',False):return False,'scan_waiting_for_mount_verification'
        if self.gimbal_transport!='x5':return False,'scan_waiting_for_x5_gimbal'
        if self.link_time is None or not 0<=now-self.link_time<=.5:return False,'scan_waiting_for_actuator_status'
        link=self.link
        stamp=link.get('stamp');wall=self.get_clock().now().nanoseconds/1e9
        if type(stamp) not in (int,float) or not math.isfinite(stamp) or not 0<=wall-stamp<=.5:
            return False,'scan_waiting_for_actuator_status'
        if link.get('enabled') is not True or link.get('dry_run') is not False:return False,'scan_waiting_for_real_actuator'
        if link.get('estop') or link.get('fault'):return False,'scan_actuator_fault_or_estop'
        if link.get('status_fresh') is not True or link.get('feedback_fresh') is not True:return False,'scan_waiting_for_fresh_encoder_feedback'
        angles=link.get('actual_angles_deg')
        if (not isinstance(angles,(list,tuple)) or len(angles)!=2 or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in angles)):
            return False,'scan_waiting_for_measured_gimbal_angles'
        return True,'scan_hardware_ready'
    def navigation_ready(self,now):
        n=self.navigation
        if not n or self.nav_received is None or now-self.nav_received>.3:return False
        return bool(self.navigation_fresh(now) and n.get('valid') and n.get('ready_to_move') and not n.get('arrived'))
    def navigation_fresh(self,now):
        n=self.navigation or {};stamp=n.get('stamp')
        if type(stamp) not in (int,float) or not math.isfinite(stamp):return False
        if self.navigation_owner=='fpga' and n.get('owner')!='fpga':return False
        return bool(self.nav_received is not None and 0<=now-self.nav_received<=.3
                    and 0<=self.get_clock().now().nanoseconds/1e9-stamp<=.3)
    def send_maneuver(self):
        n=self.navigation
        msg=ManeuverCommand();msg.header.stamp=self.get_clock().now().to_msg()
        msg.command_id=str(n['command_id']);msg.edge_id=str(n['edge_id']);msg.route_revision=int(n.get('route_revision',0))
        msg.turn={'left':1,'right':2,'straight':0}[n['turn']]
        msg.stop_at_end=bool(n.get('stop_distance_m') is not None or n.get('requires_stop'))
        msg.speed_limit_m_s=float(n['speed_limit_m_s']);msg.distance_m=float(n['distance_m']);msg.lease_ms=200
        self.maneuver_pub.publish(msg)
    def image(self,msg):
        if self.task_profiles:return
        if self.traffic.event_id is None:return
        elapsed=age(self,msg.header.stamp)
        if not 0<=elapsed<=.3:self.observation=None;return
        frame=self.bridge.imgmsg_to_cv2(msg.left,'bgr8')
        self.observation=self.detector.detect(frame,capture_time=time.monotonic()-elapsed,frame_id=msg.sequence)
    def stopped(self,now):
        if self.navigation_owner=='fpga' and self.extension_enabled:
            # The fixed UART owner supplies the vehicle's physical stop fact.
            # A previous navigation ID is only relevant to arrival attribution;
            # it must not hide a newer measured stop during fire inspection.
            return bool(self.extension_fresh(now) and self.extension.get('real_uart') is True
                and self.extension.get('status_fresh') is True
                and self.extension.get('vehicle_stopped') is True)
        if self.navigation_owner=='fpga' and self.navigation_command_id is not None:
            n=self.navigation or {}
            return bool(self.navigation_fresh(now) and n.get('command_id')==self.navigation_command_id and
                n.get('status_fresh') is True and n.get('vehicle_stopped') is True and not n.get('estop'))
        if self.fpga is not None and self.fpga_time is not None and 0<=now-self.fpga_time<.5:
            return bool(self.fpga.vehicle_stopped)
        n=self.navigation or {}
        return bool(self.navigation_owner=='fpga' and self.navigation_fresh(now)
            and n.get('owner')=='fpga'
            and n.get('status_fresh') and n.get('vehicle_stopped') is True)
    def ready(self,now):
        if (self.extension_fresh(now) and self.extension.get('task')=='traffic' and
                self.extension.get('fixed_traffic_camera_verified') is True):
            return self.stopped(now)
        if not self.mount.get('mounting_verified',False) or self.target is None or now<self.move_end:return False
        if self.link_time is None or now-self.link_time>.5 or not self.link.get('status_fresh'):return False
        if self.gimbal_transport=='x5' and not self.link.get('gimbal_settled_estimate',False):return False
        if (self.gimbal_transport=='x5' and self.link.get('feedback_required') and
                not (self.link.get('feedback_fresh') and self.link.get('gimbal_settled_measured'))):return False
        actual=self.link.get('target_deg')
        return (self.link.get('gimbal_acknowledged',False) and actual is not None
                and max(abs(actual[i]-self.target[i]) for i in (0,1))<.02)
    def measured_hold_target(self,now):
        """Seed explicit aiming at the measured pose; never jump to a home."""
        if not self.mount.get('mounting_verified',False):return None,'aim_waiting_for_mount_verification'
        if self.gimbal_transport!='x5':return None,'aim_waiting_for_x5_gimbal'
        if self.link_time is None or not 0<=now-self.link_time<=.5:return None,'aim_waiting_for_actuator_status'
        link=self.link
        if link.get('enabled') is not True or link.get('dry_run') is not False:return None,'aim_waiting_for_real_actuator'
        if link.get('error') or link.get('fault') or link.get('estop'):return None,'aim_actuator_fault_or_estop'
        if link.get('status_fresh') is not True or link.get('feedback_fresh') is not True:return None,'aim_waiting_for_fresh_encoder_feedback'
        if link.get('reference_confirmed_this_start') is not True:return None,'aim_waiting_for_measured_reference'
        angles=link.get('actual_angles_deg')
        if (not isinstance(angles,(list,tuple)) or len(angles)!=2 or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in angles)):
            return None,'aim_waiting_for_measured_gimbal_angles'
        bounds=(self.mount['pan_limits_deg'],self.mount['tilt_limits_deg'])
        if not all(bounds[i][0]<=angles[i]<=bounds[i][1] for i in (0,1)):
            return None,'aim_measured_pose_outside_limits'
        return tuple(angles),'aim_initializing_at_measured_pose'
    def command(self,target,owner,now):
        if self.operator_hold:return
        if self.navigation_fresh(now) and (self.navigation or {}).get('estop'):return
        if not self.mount.get('mounting_verified',False):return
        bounds=(self.mount['pan_limits_deg'],self.mount['tilt_limits_deg'])
        if not all(math.isfinite(target[i]) and bounds[i][0]<=target[i]<=bounds[i][1] for i in (0,1)):
            self.get_logger().error('Gimbal target outside measured limits');return
        speed=float(self.mount['speed_deg_s'])
        if not 0<speed<=180:raise ValueError('Invalid gimbal speed')
        target=tuple(round(float(v),2) for v in target)
        if target!=self.target:
            if self.target is None:
                travel=max(b[1]-b[0] for b in bounds)/speed
            else:travel=max(abs(target[i]-self.target[i]) for i in (0,1))/speed
            self.move_end=now+travel+float(self.mount['extra_settle_s'])
            self.target=target
        msg=GimbalCommand();msg.header.stamp=self.get_clock().now().to_msg();msg.owner=owner
        msg.pan_deg=target[0];msg.tilt_deg=target[1];msg.speed_deg_s=speed;msg.lease_ms=300
        self.gimbal_pub.publish(msg)
    def publish_permission(self,allowed,event_id,reason,remaining_ms=250):
        msg=MotionPermission();msg.header.stamp=self.get_clock().now().to_msg()
        msg.event_id=int(event_id or 0);msg.allow_motion=bool(allowed and remaining_ms>=1)
        msg.lease_ms=max(1,min(250,int(remaining_ms))) if msg.allow_motion else 250;msg.reason=reason
        self.permission_pub.publish(msg)
    def tick(self):
        now=time.monotonic();ready=self.ready(now);stopped=self.stopped(now)
        self.scan_motion_allowed=False
        scan_hardware_ready,self.scan_wait_reason=self.scan_hardware_status(now)
        ext_fresh=self.extension_fresh(now)
        ext_task=self.extension.get('task') if ext_fresh else None
        if self.extension_enabled and ext_task!='traffic' and self.traffic.event_id is not None:
            self.traffic.reset();self.observation=None
        if not stopped:self.observation_pose_id=None
        traffic=self.traffic.tick(now,self.observation,vehicle_stopped=stopped,gimbal_ready=ready)
        allowed=False;reason='mission_'+self.mode
        if traffic['traffic_active']:
            driving=self.mode in ('patrol','navigate')
            allowed=driving and traffic['move_allowed'] and self.mount.get('mounting_verified',False)
            reason=traffic['reason'] if driving else 'crossing_pending_observation_hold'
            scan_stopped=(traffic.get('stop_wait_complete') is True if traffic.get('stop_policy')=='delay' else stopped)
            if (scan_stopped or traffic['state']=='RELEASED') and traffic['state'] not in ('TIMEOUT','WAIT_STOP'):
                self.camera_pose='traffic'
                if not (ext_fresh and self.extension.get('fixed_traffic_camera_verified') is True):
                    self.command((self.mount['pan_home_deg']+self.mount['pan_sign']*traffic['desired_pan_deg'],
                                  self.mount['tilt_home_deg']+self.mount['tilt_sign']*traffic['desired_tilt_deg']),'traffic',now)
        elif self.mode=='patrol':allowed=True
        elif self.mode=='navigate':
            allowed=self.navigation_ready(now);reason='navigation_ready' if allowed else 'navigation_not_ready'
            n=self.navigation
            if self.arrival_eligible(now):
                self.mode='inspect';self.target=None;self.observation_pose_id=None
                self.remember_arrival(n['command_id'])
                if n.get('motion_kind') in ('numbered_point', 'semantic_region'):self.arrival_policy=n['arrival_policy']
                if self.arrival_notified!=n.get('command_id'):
                    self.arrival_notified=n.get('command_id')
                    # The task router sets the policy with the goal; a later
                    # spoken mode change supersedes that goal's old snapshot.
                    manual=n.get('motion_kind')=='manual'
                    self.speech_pub.publish(String(data=('已到达区域搜索航点并停稳，继续检查火源。' if n.get('motion_kind')=='semantic_region' else 'FPGA报告短距离移动完成并停车，等待下一条明确指令。' if manual else
                        'FPGA报告已到达并停稳。正在进行YOLO识别并核对楼牌，'+('开始找火扫描；找到后等待选择编号和明确喷水指令。' if self.arrival_policy=='scan' else '开始一轮自动找火灭火。' if self.arrival_policy=='auto' else '等待你分析火场或选择火焰；喷水需要单独指令。'))))
                    if self.arrival_policy in ('auto','scan') and not manual:
                        stamp=self.get_clock().now().nanoseconds/1e9
                        if self.arrival_policy=='scan':self.scan_hold_run='arrival:'+str(n['command_id'])
                        publish_json(self.auto_pub,dict(action='scan' if self.arrival_policy=='scan' else 'start',command_id='arrival:'+str(n['command_id']),issued_at=stamp,ttl_s=3.))
        elif self.mode in ('fire','inspect') and stopped:
            self.camera_pose='observe'
            auto=self.auto_fire
            scanning=(self.mode=='inspect' and self.auto_received is not None and 0<=now-self.auto_received<.5
                      and auto.get('active') and auto.get('phase')=='SEARCH')
            if self.fixed_camera_observation and self.mode=='inspect':
                self.camera_pose='fixed_observe'
                reason='fixed_camera_observing_current_view'
            elif self.scan_hold_run is not None:
                authorized=self.auto_authority.authorized(self.scan_hold_run,self.get_clock().now().nanoseconds/1e9)
                same_run=auto.get('command_id')==self.scan_hold_run and auto.get('scan_only') is True
                self.scan_motion_allowed=bool(scanning and same_run and authorized and scan_hardware_ready)
                self.camera_pose='scan' if self.scan_motion_allowed else 'observe_wait_manual'
                if self.scan_motion_allowed:
                    offset=float(auto.get('pan_offset_deg',0.))
                    desired=(self.mount['pan_home_deg']+self.mount['pan_sign']*offset,self.mount['tilt_home_deg'])
                    limits=(self.mount['pan_limits_deg'],self.mount['tilt_limits_deg'])
                    if not all(math.isfinite(desired[i]) and limits[i][0]<=desired[i]<=limits[i][1] for i in (0,1)):
                        self.scan_motion_allowed=False;scan_hardware_ready=False
                        self.scan_wait_reason='scan_pose_outside_measured_limits';reason=self.scan_wait_reason
                    else:self.command(desired,'mission_home',now)
                elif not scan_hardware_ready:reason=self.scan_wait_reason
                elif not same_run:reason='scan_waiting_for_controller'
                else:reason='scan_finished_waiting_for_manual_selection' if not auto.get('active') else 'scan_waiting_for_authority'
            elif scanning:
                offset=float(auto.get('pan_offset_deg',0.))
                self.command((self.mount['pan_home_deg']+self.mount['pan_sign']*offset,self.mount['tilt_home_deg']),'mission_home',now)
            elif self.extension_enabled and self.mode=='inspect':
                # Task selection and arrival only turn on perception. An
                # operator must choose a target before either axis can move.
                self.camera_pose='observe_wait_manual'
            elif self.target is None:
                if self.extension_enabled:
                    initial,reason=self.measured_hold_target(now)
                    if initial is not None:
                        self.target=tuple(round(float(v),2) for v in initial)
                        self.move_end=now+float(self.mount['extra_settle_s'])
                        self.command(initial,'fire',now)
                else:self.command((self.mount['pan_home_deg'],self.mount['tilt_home_deg']),'mission_home',now)
            else:
                target=self.target
                if self.mode=='fire' and ready and self.fire and self.fire_time is not None and now-self.fire_time<.5 and self.fire.get('valid'):
                    capture=self.fire.get('capture_stamp',0);frame_age=self.get_clock().now().nanoseconds/1e9-capture
                    proposal=self.fire.get('servo_plan',{}).get('command')
                    if proposal and 0<=frame_age<=.5:target=(proposal['pan_target_deg'],proposal['tilt_target_deg'])
                self.command(target,'fire' if self.mode=='fire' else 'mission_home',now)
        # FPGA owns the map and driving. No removed crosswalk image or road
        # gimbal pose is required to navigate. Patrol still needs fresh real
        # chassis feedback; deleting a visual gate must not create blind GO.
        if self.task_profiles and self.mode=='patrol':
            fresh_status=self.fpga_time is not None and 0<=now-self.fpga_time<.5
            navigation_status=(self.navigation_owner=='fpga' and self.navigation_fresh(now)
                and (self.navigation or {}).get('status_fresh') is True)
            if not (fresh_status or navigation_status):
                allowed=False;reason='fpga_status_unavailable'
        if not traffic['traffic_active'] and self.mode in ('idle','standby','patrol','navigate'):
            self.camera_pose='unconfigured'
        if self.operator_hold:
            allowed=False
            reason='crossing_pending_observation_hold' if traffic['traffic_active'] else 'operator_hold'
        emergency_stop=bool(self.navigation_fresh(now) and (self.navigation or {}).get('estop'))
        if emergency_stop:allowed=False;reason='fpga_estop'
        if self.task_profiles and self.navigation_owner!='fpga' and (self.fpga_time is None or not 0<=now-self.fpga_time<.5):
            allowed=False;reason='fpga_status_unavailable'
        if self.mode=='navigate' and not self.navigation_ready(now):allowed=False;reason='navigation_not_ready'
        if self.extension_enabled and ext_fresh:
            active=self.extension.get('active') or {}
            if (active.get('phase') in ('moving','seeking') and not self.operator_hold and
                    not emergency_stop and not traffic['traffic_active'] and self.extension.get('status_fresh') is True):
                allowed=True;reason='fpga_extension_'+active['phase']
            elif self.mode=='navigate':
                # Region/point navigation belongs to the same UART owner but
                # is not stored in the legacy manual/seek `active` slot.
                n=self.navigation or {}
                same_navigation=bool(self.navigation_command_id and self.navigation_fresh(now)
                    and n.get('command_id')==self.navigation_command_id and n.get('owner')=='fpga'
                    and n.get('status_fresh') is True and self.extension.get('status_fresh') is True)
                allowed=bool(same_navigation and self.navigation_ready(now) and not self.operator_hold
                    and not emergency_stop and not traffic['traffic_active'])
                if not self.operator_hold and not emergency_stop and not traffic['traffic_active']:
                    reason=('fpga_extension_navigation_'+str(n.get('phase','waiting')) if same_navigation
                            else 'fpga_extension_'+str(active.get('phase','idle')))
        traffic['published_at']=self.get_clock().now().nanoseconds/1e9
        remaining=math.floor((traffic['valid_until_s']-now)*1000) if allowed and traffic['traffic_active'] else 250
        self.publish_permission(allowed,traffic['crosswalk_id'],reason,remaining)
        if allowed and self.mode=='navigate' and self.navigation_owner=='x5':self.send_maneuver()
        publish_json(self.traffic_pub,traffic)
        publish_json(self.state_pub,dict(stamp=self.get_clock().now().nanoseconds/1e9,
            mode=self.mode,traffic_active=traffic['traffic_active'],traffic_state=traffic['state'],
            camera_pose=self.camera_pose,task_profiles_enabled=self.task_profiles,
            fixed_camera_observation=self.fixed_camera_observation,
            extension_enabled=self.extension_enabled,extension_task=ext_task,
            fixed_traffic_camera_verified=bool(ext_fresh and self.extension.get('fixed_traffic_camera_verified')),
            navigation_owner=self.navigation_owner,arrival_policy=self.arrival_policy,
            navigation_command_id=self.navigation_command_id,arrival_consumed=self.arrival_notified,
            scan_only=self.scan_hold_run is not None,scan_motion_allowed=self.scan_motion_allowed,
            scan_hardware_ready=scan_hardware_ready,scan_wait_reason=self.scan_wait_reason,
            emergency_stop=emergency_stop,
            auto_fire_active=bool(self.auto_received is not None and 0<=now-self.auto_received<.5 and self.auto_fire.get('active')),
            traffic_stop_policy=traffic.get('stop_policy'),traffic_wait_s=self.traffic.cfg.get('stop_delay_s'),
            yolo_requested=(ext_task!='traffic' if self.extension_enabled else self.mode in ('inspect','fire')) and not traffic['traffic_active'],
            motion_allowed=bool(allowed),vehicle_stopped=stopped,gimbal_ready=self.ready(now),
            vehicle_stop_source=('extension_uart' if self.extension_enabled and self.navigation_owner=='fpga'
                                 else 'navigation_or_fpga_event'),
            mounting_verified=self.mount.get('mounting_verified',False),gimbal_target_deg=self.target,
            actual_angles_deg=(self.link.get('actual_angles_deg') if self.gimbal_transport=='x5' and
                self.link_time is not None and 0<=now-self.link_time<.5 and self.link.get('feedback_fresh') else
                [self.fpga.pan_deg,self.fpga.tilt_deg] if self.gimbal_transport!='x5' and self.fpga and
                self.fpga.measured_angles_valid and self.fpga_time and now-self.fpga_time<.5 else None),
            observation_pose_id=self.observation_pose_id,operator_hold=self.operator_hold,
            gimbal_transport=self.gimbal_transport,
            pump_control_implemented=self.gimbal_transport=='x5',
            pump_armed=(self.link.get('guard',{}).get('armed',False) if self.gimbal_transport=='x5' else False),
            reason=reason))

def main():spin(Mission)
