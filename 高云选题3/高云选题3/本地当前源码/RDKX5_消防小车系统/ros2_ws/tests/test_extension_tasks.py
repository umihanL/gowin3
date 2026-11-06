"""Offline contracts of the real FPGA wire/task boundary; no hardware I/O."""
import json
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rescue_vision'))
from rescue_vision import ov5640_protocol as p
from rescue_vision.extension_control import ExtensionControl, CenterConfirmation, start_guard
from rescue_vision.task_interaction import parse_task_text, TaskRouter
from rescue_vision.vision_task_gate import VisionTaskGate


def request(text, now=100., cid='test-1'):
    return dict(parse_task_text(text),command_id=cid,source='web',issued_at=now,ttl_s=3.)


def ready_core():
    core=ExtensionControl(True,True,-1)
    core.protocol_ready=True
    core.capabilities=dict(distance=True,velocity=True,traffic=True,route_turn=True,zebra_switch=True)
    core.status=dict(phase=0,vehicle_stopped=True,enabled=False,mode_selected=True,zebra_wait=False,zebra_detection_enabled=True)
    core.status_at=100.
    core.route_catalog=json.loads((ROOT/'config/extension_map.json').read_text(encoding='utf-8-sig'))
    return core


def ack(core, seq, now=100.05, result=0):
    command=core.pending[seq][0]
    core.receive(p.Packet(seq,command|0x80,bytes((result,0,0,0))),now)


def settle_mode(core, now=100.075):
    core.status_at=now
    core.status_request_at=now
    core.status.update(phase=0,vehicle_stopped=True)
    core.tick(now)


def targets(stamp=100., target=1, x=.5, session='camera1', valid=True):
    return dict(valid=valid,capture_stamp=stamp,registry_session=session,image_size_wh=[640,480],
        targets=[dict(target_id=target,selectable=True,bbox_xyxy=[x*640-20,220,x*640+20,260])])


class WireTests(unittest.TestCase):
    def test_actual_fixed10_vector_and_fragmented_noise(self):
        frame=p.encode(7,p.MOVE,p.move_payload('reverse',.2))
        self.assertEqual(frame[:9].hex(),'a55a01071038ff6400')
        self.assertEqual(len(frame),10)
        parser=p.Parser()
        self.assertEqual(parser.feed(b'noise'+frame[:5]),[])
        found=parser.feed(frame[5:]+b'\xa5')
        self.assertEqual(found,[p.Packet(7,p.MOVE,b'\x38\xff\x64\0')])
        self.assertEqual(parser.feed(frame[1:]),found)

    def test_bad_checksum_resync_and_old_crc_not_accepted(self):
        frame=p.encode(4,p.EXT_STATUS)
        bad=frame[:-1]+bytes((frame[-1]^1,))
        parser=p.Parser()
        self.assertEqual(parser.feed(bad+frame),[p.Packet(4,p.EXT_STATUS,bytes(4))])
        self.assertEqual(parser.bad_frames,1)

    def test_no_nonfinite_or_overlimit_distance(self):
        for distance in (float('nan'),float('inf'),0,.049,10.001,True):
            with self.assertRaises(ValueError):p.move_payload('forward',distance)


class StateMachineTests(unittest.TestCase):
    def test_default_vision_independent_of_uart(self):
        core=ExtensionControl()
        core.request(request('开启拓展任务一'),100.)
        self.assertEqual(core.task,'vision')
        self.assertEqual(core.outbox,[])
        self.assertFalse(core.state(100.)['can_move'])
        self.assertEqual(core.effects[-1],('mission',{'mode':'inspect'}))

    def test_old_firmware_and_old_status_fail_closed(self):
        core=ready_core();core.protocol_ready=False
        with self.assertRaisesRegex(ValueError,'firmware'):core.request(request('前进50厘米'),100.)
        core.protocol_ready=True;core.status_at=99.
        with self.assertRaisesRegex(ValueError,'feedback'):core.request(request('前进50厘米'),100.)

    def test_mode_ack_precedes_motion_and_ack_does_not_complete(self):
        core=ready_core();core.request(request('后退20厘米'),100.)
        self.assertEqual([f[4] for f in core.outbox],[p.TASK_MODE])
        a=core.active
        ack(core,a['mode_sequence'])
        self.assertEqual(a['phase'],'waiting_mode_stop')
        self.assertNotIn(p.MOVE,[f[4] for f in core.outbox])
        settle_mode(core)
        self.assertEqual(core.outbox[-1][4],p.MOVE)
        ack(core,a['request_sequence'],100.1)
        self.assertEqual(a['phase'],'moving')
        self.assertFalse(core.last_result['completed'])
        self.assertFalse(core.state(100.1)['can_move'])

    def test_done_event_requires_matching_task_and_new_measured_stop(self):
        core=ready_core();core.request(request('前进20厘米'),100.)
        a=core.active;ack(core,a['mode_sequence']);settle_mode(core);ack(core,a['request_sequence'],100.1)
        core.receive(p.Packet(0,p.DONE,bytes((a['request_sequence']+1,3,0,0))),100.2)
        self.assertEqual(a['phase'],'moving')
        core.receive(p.Packet(0,p.DONE,bytes((a['request_sequence'],3,0,0))),100.2)
        core.tick(100.21)
        self.assertEqual(a['phase'],'stopping')
        core.status_at=100.25;core.status['vehicle_stopped']=False;core.tick(100.25)
        self.assertEqual(a['phase'],'stopping')
        core.status_at=100.3;core.status_request_at=100.25;core.status['vehicle_stopped']=True;core.tick(100.3)
        self.assertEqual(a['phase'],'completed')
        self.assertTrue(core.last_result['completed'])

    def test_stop_cannot_be_undone_by_late_mode_ack(self):
        core=ready_core();core.request(request('前进20厘米'),100.)
        a=core.active;core.stop(100.01,'operator_stop')
        ack(core,a['mode_sequence'],100.02)
        self.assertEqual([f[4] for f in core.outbox],[p.TASK_MODE,p.STOP])
        self.assertEqual(a['phase'],'stopping')

    def test_link_loss_stops_and_lease_is_not_renewed(self):
        core=ready_core();core.request(request('前进20厘米'),100.)
        a=core.active;ack(core,a['mode_sequence']);settle_mode(core);ack(core,a['request_sequence'],100.1)
        core.tick(100.7)
        self.assertEqual(a['phase'],'stopping')
        self.assertEqual(a['stop_reason'],'fpga_feedback_lost')
        self.assertIn(p.STOP,[f[4] for f in core.outbox])

    def test_delayed_pre_stop_status_does_not_prove_stop(self):
        core=ready_core();core.request(request('前进20厘米'),100.)
        a=core.active;ack(core,a['mode_sequence']);settle_mode(core);ack(core,a['request_sequence'],100.1)
        status_seq=core.send(p.EXT_STATUS,now=100.15)
        core.stop(100.2,'operator_stop')
        core.receive(p.Packet(status_seq,p.EXT_STATUS|0x80,bytes((0,10,0,0))),100.25)
        core.tick(100.25)
        self.assertEqual(a['phase'],'stopping')
        core.tick(102.3)
        self.assertEqual(core.last_result['reason'],'stop_requested_no_measured_stop')
        self.assertFalse(core.state(102.3)['can_move'])

    def test_late_wrong_opcode_ack_does_not_delete_current_request(self):
        core=ready_core();seq=core.send(p.EXT_STATUS,now=100.)
        core.receive(p.Packet(seq,p.MOVE|0x80,bytes(4)),100.1)
        self.assertIn(seq,core.pending)
        core.receive(p.Packet(seq,p.EXT_STATUS|0x80,bytes((0,10,0,0))),100.2)
        self.assertEqual(core.status_at,100.2)

    def test_switch_to_traffic_preserves_new_owner_when_old_move_stops(self):
        core=ready_core();core.request(request('前进20厘米'),100.)
        a=core.active;ack(core,a['mode_sequence']);settle_mode(core);ack(core,a['request_sequence'],100.1)
        core.request(request('开启交通任务',100.2,'traffic'),100.2)
        core.effects=[]
        core.status_at=100.3;core.status_request_at=100.25;core.tick(100.3)
        self.assertEqual(core.task,'traffic')
        self.assertNotIn(('mission',{'mode':'inspect'}),core.effects)

    def test_wait_light_level_restarts_green_confirmation_if_event_lost(self):
        core=ready_core();core.task='traffic'
        seq=core.send(p.EXT_STATUS,now=100.)
        core.receive(p.Packet(seq,p.EXT_STATUS|0x80,bytes((6,30,0,0))),100.1)
        self.assertEqual(core.effects[0],('mission',{'mode':'reset_traffic'}))
        self.assertFalse(core.traffic_cycle_seen)
        core.observe_traffic(dict(published_at=100.15,move_allowed=True,state='RELEASED'),100.15)
        self.assertEqual(core.outbox[-1][5],0)

    def test_traffic_task_mode_waits_for_stop_and_drive_is_separate(self):
        core=ready_core();core.request(request('开启交通任务'),100.)
        self.assertEqual([f[4] for f in core.outbox],[p.STOP])
        self.assertFalse(core.state(100.)['can_move'])
        core.status_request_at=100.01;core.status_at=100.02;core.tick(100.02)
        transition=core.mode_transition
        self.assertEqual(transition['phase'],'waiting_ack')
        ack(core,transition['sequence'],100.04)
        self.assertEqual(core.fpga_task_mode,3)
        self.assertFalse(core.state(100.04)['can_move'])
        core.status_request_at=100.15;core.status_at=100.17;core.tick(100.17)
        self.assertIsNone(core.mode_transition)
        self.assertNotIn(3,[f[4] for f in core.outbox])
        core.request(request('开始交通行驶',100.18,'drive'),100.18)
        self.assertEqual(core.outbox[-1][4],2)
        ack(core,core.active['speed_sequence'],100.20)
        self.assertEqual(core.outbox[-1][4],3)
        self.assertEqual(core.outbox[-1][5],0)

    def test_acknowledged_stop_is_not_repeated_resetting_encoder_stillness(self):
        core=ready_core();core.request(request('前进20厘米'),100.)
        a=core.active;ack(core,a['mode_sequence']);settle_mode(core);ack(core,a['request_sequence'],100.1)
        core.stop(100.2,'operator_stop');ack(core,a['stop_sequence'],100.21)
        core.outbox=[]
        core.tick(100.8)
        self.assertNotIn(p.STOP,[f[4] for f in core.outbox])

    def test_route_requires_explicit_reference_and_upload_ack_chain(self):
        core=ready_core()
        with self.assertRaisesRegex(ValueError,'confirm_physical_start'):
            core.request(request('一直前进在第1个路口左转'),100.)
        core.request(request('确认起点X104.22厘米Y40厘米朝向90度',cid='ref'),100.)
        core.request(request('一直前进在第1个路口左转'),100.)
        self.assertFalse(core.reference['confirmed'])
        a=core.active;ack(core,a['mode_sequence']);settle_mode(core)
        self.assertEqual(core.outbox[-1][4],p.SET_START)
        now=100.08
        while a['phase']=='uploading_route':
            ack(core,a['route_upload_sequence'],now);now+=.01
        self.assertEqual(a['phase'],'planning')
        self.assertNotIn(0x03,[f[4] for f in core.outbox])
        core.status.update(phase=8);core.status_at=now
        core.tick(now)
        self.assertEqual(core.outbox[-1][4],0x03)
        self.assertEqual(core.outbox[-1][5:7],bytes((1,a['request_sequence'])))

    def test_seek_requires_camera_then_stops_without_auto_gpio(self):
        core=ready_core()
        with self.assertRaisesRegex(ValueError,'exposure'):core.request(request('前进直到火焰居中'),100.)
        core.observe_targets(targets(),100.)
        core.request(request('前进直到火焰居中'),100.)
        a=core.active;ack(core,a['mode_sequence']);settle_mode(core);ack(core,a['request_sequence'],100.1)
        for stamp in (100.11,100.16,100.21):core.observe_targets(targets(stamp),stamp)
        self.assertEqual(a['phase'],'stopping')
        self.assertEqual(a['stop_reason'],'fire_horizontal_center_confirmed')
        self.assertFalse(core.state(100.21)['aim_locked'])
        core.status_at=100.22;core.status_request_at=100.215;core.tick(100.22)
        self.assertEqual(a['phase'],'completed')
        self.assertNotIn('interaction',[key for key,_ in core.effects])

    def test_seek_lost_camera_or_passed_center_stops(self):
        for failure in ('lost','passed'):
            core=ready_core();core.observe_targets(targets(),100.)
            core.request(request('前进直到火焰居中'),100.)
            a=core.active;ack(core,a['mode_sequence']);settle_mode(core);ack(core,a['request_sequence'],100.1)
            if failure=='passed':core.observe_targets(targets(100.11,x=.2),100.11)
            else:
                core.status_at=100.55;core.tick(100.55)
            self.assertEqual(a['phase'],'stopping')

    def test_automatic_spray_after_seek_is_rejected_before_movement(self):
        core=ready_core();core.observe_targets(targets(),100.)
        with self.assertRaisesRegex(ValueError,'manual_spray_command_required'):
            core.request(request('前进直到火焰居中然后自动灭火'),100.)
        self.assertIsNone(core.active)
        self.assertEqual(core.outbox,[])

    def test_unsourced_preferred_target_cannot_bypass_completed_seek(self):
        req=request('开始自动灭火')
        req.update(preferred_target_id=1,preferred_registry_session='camera1')
        actuator=dict(stamp=100.,dry_run=False,enabled=True,pump_enabled=True,pump_on=False,guard=dict(armed=False))
        with self.assertRaisesRegex(ValueError,'manual_spray_command_required'):
            TaskRouter(extension_enabled=True).route(req,100.,extension=ready_core().state(100.),actuator=actuator)

    def test_preview_green_cannot_release_and_other_tasks_ignore_zebra(self):
        core=ready_core()
        core.receive(p.Packet(0,p.ZEBRA,b'\1\1\0\0'),100.)
        self.assertEqual(core.effects,[])
        core.task='traffic';core.status['zebra_wait']=True
        core.traffic_wait_started=97.;core.traffic_cycle_seen=True
        core.observe_traffic(dict(status='green',stamp=100.),100.)
        self.assertEqual(core.outbox,[])
        core.observe_traffic(dict(published_at=100.,move_allowed=True,state='RELEASED'),100.)
        self.assertEqual(core.outbox[-1][4:9],bytes((p.LIGHT,2,0,0,0)))


class VisionAndGrammarTests(unittest.TestCase):
    def test_distinct_frames_same_id_only(self):
        core=CenterConfirmation()
        self.assertFalse(core.observe(targets(),100.))
        self.assertFalse(core.observe(targets(),100.01))
        self.assertEqual(core.hits,1)
        self.assertFalse(core.observe(targets(100.1,target=2),100.1))
        self.assertEqual(core.hits,0)
        self.assertFalse(core.observe(targets(100.15),100.15))
        self.assertFalse(core.observe(targets(100.20),100.20))
        self.assertTrue(core.observe(targets(100.25),100.25))

    def test_extension_semantic_yolo_on_traffic_off_and_stale_off(self):
        gate=VisionTaskGate(enabled=True)
        gate.update(dict(stamp=100.,mode='navigate',extension_enabled=True,extension_task='semantic',traffic_active=False),100.,10.)
        self.assertTrue(gate.allowed(100.,10.1))
        gate.update(dict(stamp=100.1,mode='inspect',extension_enabled=True,extension_task='traffic',traffic_active=False),100.1,10.1)
        self.assertFalse(gate.allowed(100.1,10.1))

    def test_new_grammar_is_explicit_and_negation_never_moves(self):
        for text,action in [('开启拓展任务一','extension_vision'),('前进1000厘米','manual_move'),
                ('一直前进然后在第3个路口左拐','route_turn'),('前进直到火焰居中','seek_fire'),
                ('前进直到火焰居中然后自动灭火','seek_fire'),('开启交通任务','traffic_task')]:
            self.assertEqual(parse_task_text(text)['action'],action)
        for text in ('不要前进直到火焰居中','如果前进到第三个路口然后左转','前进50厘米然后喷水'):
            self.assertEqual(parse_task_text(text)['action'],'dialogue')

    def test_router_outputs_real_extension_not_old_crc_navigation(self):
        router=TaskRouter(extension_enabled=True)
        out=router.route(request('前进100厘米'),100.,extension=ready_core().state(100.))
        self.assertEqual(out['extension']['distance_m'],1.)
        self.assertIsNone(out['navigation'])
        self.assertFalse(out.get('completed',False))
        stop=router.route(request('停车',cid='stop'),100.)
        self.assertEqual(stop['extension']['action'],'stop')
        self.assertEqual(stop['actuator']['action'],'disarm')

    def test_legacy_unregistered_building_not_sent_to_crc_bridge(self):
        router=TaskRouter({'buildings':[dict(building_id='A',number='1')]},extension_enabled=True,navigation_execute_enabled=True)
        with self.assertRaisesRegex(ValueError,'not_registered'):
            router.route(dict(action='navigate',building_id='A',command_id='building',source='web',issued_at=100.,ttl_s=3.),100.)


if __name__=='__main__':unittest.main()
