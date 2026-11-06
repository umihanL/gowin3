"""Live ROS 2 + Linux PTY UART test. No real camera, GPIO, robot or BPU.

Run after colcon build, source install/setup.bash, in the dedicated ROS container.
"""
from pathlib import Path
import json
import os
import pty
import select
import signal
import struct
import subprocess
import sys
import tempfile
import time
import tty
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from rescue_interfaces.msg import StereoPair,DetectionArray
from rescue_vision import protocol as wire
from rescue_vision.node_common import IMAGE_QOS

ROOT=Path(__file__).resolve().parents[1]
os.environ['ROS_DOMAIN_ID']='171';os.environ['ROS_LOCALHOST_ONLY']='1'


def board(color):
    a=np.full((240,320,3),185,np.uint8)
    if color=='blank':return a
    cv2.rectangle(a,(140,60),(180,160),(12,12,12),-1)
    if color in ('red','both'):cv2.circle(a,(160,87),7,(0,0,255),-1)
    if color in ('green','both'):cv2.circle(a,(160,133),7,(0,255,0),-1)
    return a


def main():
    logs=ROOT/'logs/ros_integration';logs.mkdir(parents=True,exist_ok=True)
    master,slave=pty.openpty();tty.setraw(slave);device=os.ttyname(slave);os.set_blocking(master,False)
    tmp=Path(tempfile.mkdtemp(prefix='rescue_ros_test_'))
    mount=json.loads((ROOT/'config/mount.template.json').read_text());mount.update(mounting_verified=True,speed_deg_s=180.,extra_settle_s=.02)
    (tmp/'mount.json').write_text(json.dumps(mount))
    processes=[];streams=[];checks=[];parser=wire.Parser();packets=[];targets=[];seq=0
    last_status=last_image=0.;status_enabled=True;forced_moving=False;permission_until=0.;last_permission=False
    states=[];links=[];goals=[];detections=[];processed_frames=[]
    def start(exe,params):
        command=['ros2','run','rescue_vision',exe,'--ros-args']
        for k,v in params.items():command+=['-p',f'{k}:={str(v).lower() if isinstance(v,bool) else v}']
        log=(logs/(exe+'.log')).open('w');streams.append(log)
        processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    def send(kind,payload):
        nonlocal seq
        seq=(seq+1)&0xffff;os.write(master,wire.encode(kind,seq,payload))
    rclpy.init();node=Node('integration_harness');bridge=CvBridge()
    camera=node.create_publisher(StereoPair,'/stereo/pair',IMAGE_QOS)
    semantic=node.create_publisher(String,'/semantic/request',10)
    mission_request=node.create_publisher(String,'/mission/request',10)
    node.create_subscription(String,'/traffic/state',lambda x:states.append(json.loads(x.data)),10)
    node.create_subscription(String,'/fpga/link',lambda x:links.append(json.loads(x.data)),10)
    node.create_subscription(String,'/navigation/semantic_goal',lambda x:goals.append(json.loads(x.data)),10)
    node.create_subscription(DetectionArray,'/perception/detections',detections.append,IMAGE_QOS)
    node.create_subscription(StereoPair,'/vision/frame',processed_frames.append,IMAGE_QOS)
    image_seq=0
    def pump(color='blank',duration=.1):
        nonlocal last_status,last_image,image_seq,permission_until,last_permission
        end=time.monotonic()+duration
        while time.monotonic()<end:
            now=time.monotonic()
            if status_enabled and now-last_status>.045:
                moving=forced_moving and last_permission and now<permission_until
                send(wire.STATUS,struct.pack('<HhhH',(0 if moving else 1)|2,0,0,0));last_status=now
            readable,_,_=select.select([master],[],[],0)
            if readable:
                for packet in parser.feed(os.read(master,8192)):
                    packets.append((now,packet))
                    if packet.kind==wire.GIMBAL:
                        pan,tilt,speed,lease=struct.unpack('<hhHH',packet.payload);targets.append((pan/100,tilt/100))
                        send(wire.ACK,struct.pack('<HB',packet.sequence,0))
                    if packet.kind==wire.PERMISSION:
                        eid,allowed,lease=struct.unpack('<IBH',packet.payload)
                        last_permission=bool(allowed);permission_until=now+lease/1000
            if now-last_image>.095:
                msg=StereoPair();msg.sequence=image_seq;image_seq+=1
                msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='stereo_rig'
                msg.left=bridge.cv2_to_imgmsg(board(color),'bgr8');msg.right=bridge.cv2_to_imgmsg(board(color),'bgr8')
                msg.synchronization_verified=False;msg.skew_s=float('nan');camera.publish(msg);last_image=now
            rclpy.spin_once(node,timeout_sec=.01)
            if any(p.poll() is not None for p in processes):raise AssertionError('ROS node exited; inspect logs/ros_integration')
    def until(predicate,color,timeout,label):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            pump(color,.08)
            if predicate():checks.append(label);return
        raise AssertionError(label+' timed out; last states='+repr(states[-3:]))
    report={'passed':False,'real_hardware':False,'bpu_executed':False,'checks':checks}
    try:
        start('stereo_camera',{'enabled':False})
        start('perception',{'detector_enabled':False,'stereo_calibration':ROOT/'config/stereo.template.json'})
        start('fire_aim',{'servo_config':ROOT/'config/servo.template.json','stereo_calibration':ROOT/'config/stereo.template.json'})
        start('mission',{'traffic_config':ROOT/'config/traffic.json','mount_config':tmp/'mount.json'})
        start('fpga_bridge',{'dry_run':False,'port':device})
        start('scene',{'ocr_enabled':False,'buildings_config':ROOT/'config/buildings.json'})
        until(lambda:links and links[-1]['status_fresh'] and detections,'blank',15,'six_nodes_running_and_uart_status_received')
        assert not detections[-1].detector_available
        assert detections[-1].header.frame_id=='left_camera_optical'
        assert processed_frames[-1].left.header.frame_id=='left_camera_optical'
        assert processed_frames[-1].right.header.frame_id=='right_camera_optical'
        checks.append('unavailable_detector_is_explicit_and_left_right_headers_preserved')
        semantic.publish(String(data=json.dumps({'text':'去3号楼观察'},ensure_ascii=False)))
        until(lambda:goals,'blank',3,'semantic_request_delivered')
        assert goals[-1]['building_id']=='C' and not goals[-1]['can_execute'];checks.append('semantic_goal_is_building_C_not_claimed_navigation')
        # Deliberately corrupt one frame; it must not create a crosswalk event.
        invalid=bytearray(wire.encode(wire.CROSSWALK,777,struct.pack('<I',99)));invalid[-1]^=1;os.write(master,invalid)
        pump('blank',.3);assert states[-1]['state']=='IDLE';checks.append('bad_crc_crosswalk_rejected')
        send(wire.CROSSWALK,struct.pack('<I',42))
        until(lambda:len(set(targets))>=2,'blank',8,'crosswalk_stop_and_bounded_scan_commands_reach_uart')
        assert not last_permission
        until(lambda:states[-1]['state']=='RED_HOLD','red',5,'red_lamp_holds_stop')
        send(wire.CROSSWALK,struct.pack('<I',42));pump('red',.3)
        assert states[-1]['state']=='RED_HOLD';checks.append('repeated_event_does_not_restart_scan')
        pump('green',.18);assert not last_permission;checks.append('short_green_flash_does_not_release')
        pump('red',.2)
        until(lambda:last_permission and states[-1]['state']=='RELEASED','green',4,'stable_green_sends_expiring_uart_go')
        forced_moving=True;pump('green',.7)
        assert last_permission and states[-1]['state']=='RELEASED';checks.append('motion_does_not_deadlock_green_and_gimbal_lease_renews')
        mission_request.publish(String(data=json.dumps({'mode':'idle'})))
        until(lambda:not last_permission,'green',2,'explicit_idle_overrides_green_permission')
        forced_moving=False;pump('green',.5);assert not last_permission
        mission_request.publish(String(data=json.dumps({'mode':'patrol'})))
        until(lambda:last_permission and states[-1]['state']=='RELEASED','green',8,'resume_reacquires_fresh_green')
        until(lambda:not last_permission,'red',2,'red_revokes_uart_go')
        forced_moving=False
        until(lambda:last_permission and states[-1]['state']=='RELEASED','green',4,'green_can_reacquire_after_stop')
        send(wire.CROSSING_COMPLETE,struct.pack('<I',42))
        until(lambda:states[-1]['state']=='IDLE','green',2,'matching_crossing_complete_releases_gimbal_owner')
        # UART bridge must stop if MCU heartbeats vanish even when ROS asks GO.
        from rescue_interfaces.msg import MotionPermission
        pub=node.create_publisher(MotionPermission,'/control/motion_permission',10)
        # Patrol requests GO, but stale MCU status must still force bridge STOP.
        status_enabled=False;pump('blank',.8)
        assert not last_permission and not links[-1]['status_fresh'];checks.append('mcu_status_loss_forces_stop')
        report.update(passed=True,uart_packets=len(packets),distinct_gimbal_targets=sorted(set(targets)),
            model='synthetic LED images; YOLO disabled; all ROS nodes instantiated',test_transport='Linux PTY using actual pyserial bridge')
    except Exception as exc:
        report['error']=repr(exc);raise
    finally:
        (ROOT/'reports/ros_integration.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
        for stream in streams:stream.close()
        node.destroy_node();rclpy.shutdown();os.close(master);os.close(slave)
        print(json.dumps(report,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
