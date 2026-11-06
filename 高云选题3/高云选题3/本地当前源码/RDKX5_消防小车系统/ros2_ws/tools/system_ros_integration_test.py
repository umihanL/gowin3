"""Real ROS 2 / pyserial / Linux PTY system-chain integration (no robot hardware).

Run inside the built ROS container after sourcing install/setup.bash.
The voice case injects a recognized transcript intent; this is NOT a microphone/ASR test.
"""
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import tempfile
import time
import tty

os.environ['ROS_DOMAIN_ID']='178'
os.environ['ROS_LOCALHOST_ONLY']='1'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rescue_vision import protocol as wire

ROOT=Path(__file__).resolve().parents[1]


def main():
    logs=ROOT/'logs/system_ros_integration';logs.mkdir(parents=True,exist_ok=True)
    reports=ROOT/'reports';reports.mkdir(exist_ok=True)
    temp=Path(tempfile.mkdtemp(prefix='rescue_navigation_test_'))
    config=json.loads((ROOT/'config/navigation.json').read_text())
    config.update(map_registration_verified=True,map_registration_id='PTY_ONLY_SYNTHETIC_FIXTURE',
                  execution_adapter_verified=True,initial_pose_verified=True,
                  initial_pose_xy_yaw=[2.6,.64,math.pi/2])
    (temp/'navigation.json').write_text(json.dumps(config))
    master,slave=pty.openpty();tty.setraw(slave);device=os.ttyname(slave);os.set_blocking(master,False)
    processes=[];streams=[];checks=[];packets=[];tx=[];telemetry=[];states=[];proposals=[];missions=[];results=[];speech=[];links=[]
    parser=wire.Parser();seq=0;last_status=last_telemetry=0.;encoder_mm=0;fraction_mm=0.
    telemetry_send_gaps=[]
    telemetry_enabled=True;status_enabled=True;estop=False;advance_to_mm=None;done_id=0
    last_permission=False;lease_expires=0.;last_segment_id=0
    rclpy.init();node=Node('system_integration_harness')
    intent=node.create_publisher(String,'/interaction/command',10)
    for topic,collection in [('/fpga/tx',tx),('/fpga/telemetry',telemetry),('/navigation/state',states),
                             ('/navigation/proposal',proposals),('/mission/state',missions),
                             ('/interaction/result',results),('/fpga/link',links)]:
        node.create_subscription(String,topic,lambda msg,dest=collection:dest.append(json.loads(msg.data)),30)
    node.create_subscription(String,'/speech/say',lambda msg:speech.append(msg.data),10)
    def start(exe,params,suffix=''):
        command=['ros2','run','rescue_vision',exe,'--ros-args']
        for key,value in params.items():command+=['-p',f'{key}:={str(value).lower() if isinstance(value,bool) else value}']
        log=(logs/(exe+suffix+'.log')).open('w');streams.append(log)
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process);return process
    def stop(process,keep_telemetry_running=False):
        # A stopped interaction process is expected to exit; remove it from the
        # pump's unexpected-exit check before waiting for the ros2 wrapper.
        processes.remove(process)
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        if keep_telemetry_running:
            # Waiting synchronously used to pause this same thread's simulated
            # MCU stream. On X5 a slow ros2-wrapper exit exceeded the real
            # odometry gap limit and correctly invalidated the starting pose.
            deadline=time.monotonic()+3
            while process.poll() is None and time.monotonic()<deadline:pump(.02)
            if process.poll() is None:os.killpg(process.pid,signal.SIGKILL)
        try:process.wait(timeout=3)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
    def send(kind,payload):
        nonlocal seq
        seq=(seq+1)&0xffff;os.write(master,wire.encode(kind,seq,payload))
    def pump(duration=.1):
        nonlocal last_status,last_telemetry,encoder_mm,fraction_mm,last_permission,lease_expires,last_segment_id,done_id
        end=time.monotonic()+duration
        while time.monotonic()<end:
            now=time.monotonic()
            readable,_,_=select.select([master],[],[],0)
            if readable:
                for packet in parser.feed(os.read(master,16384)):
                    packets.append((now,packet))
                    if packet.kind==wire.PERMISSION:
                        _,allowed,lease=struct.unpack('<IBH',packet.payload)
                        last_permission=bool(allowed);lease_expires=now+lease/1000.
                    elif packet.kind==wire.MANEUVER:
                        last_segment_id=struct.unpack('<IBBHIH',packet.payload)[0]
                    elif packet.kind==wire.GIMBAL:
                        send(wire.ACK,struct.pack('<HB',packet.sequence,0))
            moving=advance_to_mm is not None and encoder_mm<advance_to_mm and last_permission and now<lease_expires and not estop
            if status_enabled and now-last_status>=.045:
                send(wire.STATUS,struct.pack('<HhhH',(0 if moving else 1)|2,0,0,0));last_status=now
            if telemetry_enabled and now-last_telemetry>=.045:
                if last_telemetry:telemetry_send_gaps.append(now-last_telemetry)
                dt=min(.1,now-last_telemetry) if last_telemetry else .05
                if moving:
                    fraction_mm+=150.*dt;increment=int(fraction_mm);fraction_mm-=increment
                    encoder_mm=min(advance_to_mm,encoder_mm+increment)
                    if encoder_mm>=advance_to_mm:done_id=last_segment_id;moving=False
                flags=1|4|64|(0 if moving else 32)|(16 if estop else 0)
                payload=struct.pack(wire.TELEMETRY_FORMAT,int(now*1000)&0xffffffff,encoder_mm,0,0,0,65535,
                                    150 if moving else 0,flags,0,0,done_id)
                send(wire.TELEMETRY,payload);last_telemetry=now
            rclpy.spin_once(node,timeout_sec=.005)
            if any(p.poll() is not None for p in processes):raise AssertionError('ROS node exited; see logs/system_ros_integration')
    def until(predicate,timeout,label):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            pump(.06)
            if predicate():checks.append(label);return
        raise AssertionError(label+' timed out; proposals='+repr(proposals[-2:])+'; mission='+repr(missions[-1:]))
    def command(cid,action='navigate',source='text',text='去3号楼观察',issued_offset=0.):
        payload=dict(command_id=cid,action=action,source=source,text=text,
                     issued_at=node.get_clock().now().nanoseconds/1e9+issued_offset,ttl_s=5.)
        intent.publish(String(data=json.dumps(payload,ensure_ascii=False)))
    def maneuver_packets(after=0.):return [(t,p) for t,p in packets if t>=after and p.kind==wire.MANEUVER]
    report=dict(passed=False,domain_id=178,real_hardware=False,bpu_executed=False,microphone_asr_tested=False,
                transport='Real rclpy nodes + pyserial + Linux PTY',checks=checks,
                configuration='Temporary verified synthetic starting fixture; production config remains disabled')
    try:
        start('navigation',{'navigation_config':temp/'navigation.json','navigation_map':ROOT/'config/navigation_map.json'})
        start('mission',{'traffic_config':ROOT/'config/traffic.json','mount_config':ROOT/'config/mount.template.json'})
        start('fpga_bridge',{'dry_run':False,'port':device})
        interaction=start('interaction',{'buildings_config':ROOT/'config/buildings.json','navigation_execute_enabled':False},'_preview')
        until(lambda:states and states[-1].get('localization',{}).get('valid') and links and links[-1]['status_fresh'] and intent.get_subscription_count()>0,
              15,'four_ros_nodes_and_valid_pty_telemetry')
        command('preview-3')
        until(lambda:any(r.get('accepted') and r.get('command_id')=='preview-3' for r in results)
              and states and (states[-1].get('route') or {}).get('building_id')=='03',4,'text_semantic_building_resolved_and_route_published')
        pump(.5)
        assert proposals[-1]['mode']=='preview' and not proposals[-1]['ready_to_move']
        assert not maneuver_packets() and not last_permission
        checks.append('default_preview_emits_no_uart_maneuver_or_go')
        assert any('预览' in sentence for sentence in speech);checks.append('preview_speech_text_published')
        switch_gap_start=len(telemetry_send_gaps)
        stop(interaction,keep_telemetry_running=True);pump(.4)
        start('interaction',{'buildings_config':ROOT/'config/buildings.json','navigation_execute_enabled':True},'_execute')
        until(lambda:intent.get_subscription_count()>0 and states[-1].get('localization',{}).get('valid'),
              8,'execute_interaction_discovered_with_fresh_localization')
        pump(.5)
        switch_gaps=telemetry_send_gaps[switch_gap_start:]
        report['interaction_restart_max_telemetry_gap_s']=max(switch_gaps,default=0.)
        assert switch_gaps and max(switch_gaps)<.3, 'Harness paused MCU stream during interaction restart'
        assert states[-1]['localization']['valid'], states[-1]['localization']
        checks.append('interaction_restart_preserves_valid_mcu_stream_and_localization')
        command('execute-3')
        until(lambda:any(r.get('accepted') and r.get('command_id')=='execute-3' for r in results)
              and proposals and proposals[-1].get('command_id')=='execute-3' and proposals[-1]['ready_to_move']
              and bool(maneuver_packets()),6,'interaction_to_navigation_mission_and_uart_maneuver')
        first=dict(proposals[-1]);first_records=[x for x in tx if x['kind']==wire.MANEUVER and x['metadata']['edge_id']==first['edge_id']]
        until(lambda:len([x for x in tx if x['kind']==wire.MANEUVER and x['metadata']['edge_id']==first['edge_id']])>=4,3,'maneuver_lease_renewals_observed')
        first_records=[x for x in tx if x['kind']==wire.MANEUVER and x['metadata']['edge_id']==first['edge_id']]
        ids={x['metadata']['segment_id'] for x in first_records}
        assert len(ids)==1 and all(x['metadata']['same_id_renews_only'] for x in first_records)
        first_id=next(iter(ids));checks.append('same_route_segment_keeps_single_wire_id')
        advance_to_mm=round(first['distance_m']*1000)
        until(lambda:any(x.get('maneuver_done_id')==first_id and x.get('maneuver_done') for x in telemetry),first['distance_m']/.15+5,
              'simulated_fpga_distance_completion_received')
        mapped=next(x for x in reversed(telemetry) if x.get('maneuver_done_id')==first_id)
        assert (mapped['maneuver_command_id'],mapped['maneuver_edge_id'],mapped['maneuver_route_revision'])==('execute-3',first['edge_id'],first['route_revision'])
        checks.append('done_id_maps_command_edge_and_route_revision')
        until(lambda:proposals[-1].get('edge_index')==1 and proposals[-1]['ready_to_move'],4,'matching_completion_advances_route_once')
        advance_to_mm=None;pump(.4)
        assert proposals[-1]['edge_index']==1;checks.append('repeated_done_does_not_skip_next_edge')
        second=dict(proposals[-1])
        until(lambda:any(x['kind']==wire.MANEUVER and x['metadata']['edge_id']==second['edge_id'] for x in tx),3,'next_maneuver_reaches_uart')
        second_id=next(x['metadata']['segment_id'] for x in reversed(tx) if x['kind']==wire.MANEUVER and x['metadata']['edge_id']==second['edge_id'])
        command('voice-stop','stop','voice','停止')
        until(lambda:missions[-1].get('operator_hold') and not last_permission and states[-1].get('paused'),3,'voice_transcript_stop_preempts_navigation')
        quiet=time.monotonic();pump(.45)
        assert not maneuver_packets(quiet) and not last_permission;checks.append('stop_hold_blocks_maneuver_renewal')
        command('voice-resume','resume','voice','继续')
        until(lambda:last_permission and not states[-1].get('paused') and proposals[-1]['ready_to_move'],3,'voice_transcript_resume_rechecks_navigation')
        pump(.4)
        latest=next(x for x in reversed(tx) if x['kind']==wire.MANEUVER)
        assert latest['metadata']['segment_id']==second_id;checks.append('resume_preserves_current_segment_identity')
        command('expired-command','navigate','text','去3号楼观察',issued_offset=-20.)
        until(lambda:any(not r.get('accepted') and 'Expired' in r.get('reason','') for r in results),2,'expired_interaction_command_rejected')
        estop=True
        until(lambda:not last_permission and proposals[-1].get('reason')=='estop',3,'mcu_estop_overrides_valid_navigation')
        quiet=time.monotonic();pump(.35)
        assert not maneuver_packets(quiet);checks.append('estop_blocks_uart_maneuvers')
        estop=False
        until(lambda:last_permission and proposals[-1]['ready_to_move'],3,'fresh_cleared_estop_allows_existing_task')
        direct_navigation=node.create_publisher(String,'/navigation/request',10)
        pump(.5)
        direct_navigation.publish(String(data=json.dumps(dict(command_id='bad-goal',action='navigate',
            building_id='missing-building',mode='execute',confirmed=True,
            issued_at=node.get_clock().now().nanoseconds/1e9,ttl_s=5.))))
        until(lambda:states[-1].get('paused') and states[-1].get('last_request_result',{}).get('accepted') is False
              and not last_permission,3,'rejected_navigation_pauses_previous_goal')
        pump(.5)
        assert states[-1]['reason'].startswith('request_rejected:') and not last_permission
        checks.append('periodic_state_preserves_navigation_rejection')
        command('recover-rejected-goal','resume','text','继续')
        until(lambda:last_permission and proposals[-1]['ready_to_move'],3,'explicit_resume_after_rejection_rechecks_gates')
        # Keep STATUS fresh while stopping odometry: proves navigation independently watches telemetry.
        telemetry_enabled=False
        until(lambda:not last_permission and not proposals[-1]['ready_to_move'],3,'stale_odometry_revokes_navigation_go')
        assert links[-1]['status_fresh'];checks.append('fresh_status_cannot_override_stale_localization')
        status_enabled=False;pump(.7)
        assert not last_permission and not links[-1]['status_fresh'];checks.append('status_watchdog_also_holds_stop')
        report.update(passed=True,checks_count=len(checks),uart_packets=len(packets),first_segment=first,
                      first_wire_id=first_id,second_wire_id=second_id,encoder_distance_m=encoder_mm/1000.,
                      final_proposal=proposals[-1],notes=['Only first route segment executed in software PTY model.',
                      'No physical vehicle, MCU firmware, steering dynamics, microphone or speech speaker evaluated.'])
    except Exception as exc:
        report.update(error=repr(exc),last_proposals=proposals[-3:],last_missions=missions[-3:],last_results=results[-3:])
        raise
    finally:
        (reports/'system_ros_integration.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        for process in list(processes):stop(process)
        for stream in streams:stream.close()
        node.destroy_node();rclpy.shutdown();os.close(master);os.close(slave)
        print(json.dumps(report,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
