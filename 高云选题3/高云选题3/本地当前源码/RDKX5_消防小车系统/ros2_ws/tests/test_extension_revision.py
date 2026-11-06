"""Task separation and actual FPGA zebra-toggle acknowledgement/readback."""
import unittest
from test_extension_tasks import ready_core,request,ack
from rescue_vision import ov5640_protocol as p
from rescue_vision.extension_control import ExtensionControl,zebra_switch_guard
from rescue_vision.task_interaction import TaskRouter,parse_task_text
from rescue_vision.vision_task_gate import VisionTaskGate


class ExtensionRevisionTests(unittest.TestCase):
    def test_wire_capability_and_actual_flag(self):
        self.assertTrue(p.capabilities(bytes((0,1,31,3)))['zebra_switch'])
        self.assertFalse(p.capabilities(bytes((0,1,15,3)))['zebra_switch'])
        self.assertTrue(p.status(bytes((6,30,0,0)))['zebra_detection_enabled'])
        self.assertFalse(p.status(bytes((6,14,0,0)))['zebra_detection_enabled'])
        for flag in (64,128):
            with self.assertRaises(ValueError):p.status(bytes((0,flag,0,0)))

    def test_ack_alone_does_not_update_zebra_switch(self):
        core=ready_core();core.status['zebra_detection_enabled']=False
        core.request(request('开启FPGA斑马线识别'),100.)
        frame=core.outbox[-1]
        self.assertEqual(frame[4:9],bytes((p.ZEBRA_ENABLE,1,0,0,0)))
        ack(core,core.zebra_transition['sequence'],100.05)
        self.assertFalse(core.state(100.05)['zebra_detection_enabled'])
        core.tick(100.06)
        self.assertFalse(core.last_result['completed'])
        seq=core.send(p.EXT_STATUS,now=100.1)
        core.receive(p.Packet(seq,p.EXT_STATUS|0x80,bytes((0,26,0,0))),100.12)
        core.tick(100.12)
        self.assertTrue(core.state(100.12)['zebra_detection_enabled'])
        self.assertTrue(core.last_result['completed'])
        self.assertIsNone(core.zebra_transition)

    def test_old_status_reply_cannot_confirm_toggle(self):
        core=ready_core();core.status['zebra_detection_enabled']=False
        seq=core.send(p.EXT_STATUS,now=99.99)
        core.request(request('开启FPGA斑马线识别'),100.)
        ack(core,core.zebra_transition['sequence'],100.05)
        core.receive(p.Packet(seq,p.EXT_STATUS|0x80,bytes((0,26,0,0))),100.10)
        core.tick(100.10)
        self.assertIsNotNone(core.zebra_transition)
        self.assertFalse(core.last_result['completed'])

    def test_timeout_or_nack_is_not_success(self):
        for result in (0,4):
            core=ready_core();core.request(request('关闭FPGA斑马线识别'),100.)
            ack(core,core.zebra_transition['sequence'],100.05,result)
            core.tick(102.1)
            self.assertIsNone(core.zebra_transition)
            self.assertFalse(core.last_result['completed'])
            self.assertFalse(core.last_result['accepted'])

    def test_real_flag_can_change_from_local_fpga_touch(self):
        core=ready_core()
        seq=core.send(p.EXT_STATUS,now=100.)
        core.receive(p.Packet(seq,p.EXT_STATUS|0x80,bytes((0,10,0,0))),100.01)
        self.assertFalse(core.state(100.01)['zebra_detection_enabled'])
        seq=core.send(p.EXT_STATUS,now=100.2)
        core.receive(p.Packet(seq,p.EXT_STATUS|0x80,bytes((0,26,0,0))),100.21)
        self.assertTrue(core.state(100.21)['zebra_detection_enabled'])
        self.assertIsNone(core.state(101.)['zebra_detection_enabled'])
        core.capabilities['zebra_switch']=False
        self.assertIsNone(core.state(100.21)['zebra_detection_enabled'])

    def test_waiting_light_can_disable_but_never_release(self):
        core=ready_core();core.task='traffic';core.fpga_task_mode=3
        core.status.update(phase=6,zebra_wait=True)
        core.active=dict(action='traffic_drive',phase='moving',request_sequence=5)
        self.assertEqual(zebra_switch_guard(core.state(100.),100.),'')
        core.request(request('关闭斑马线检测'),100.)
        ack(core,core.zebra_transition['sequence'],100.05)
        seq=core.send(p.EXT_STATUS,now=100.1)
        core.receive(p.Packet(seq,p.EXT_STATUS|0x80,bytes((6,14,0,0))),100.12)
        core.traffic_cycle_seen=True;core.traffic_wait_started=97.
        core.observe_traffic(dict(published_at=100.15,state='RELEASED',move_allowed=True),100.15)
        self.assertEqual(core.outbox[-1][4:9],bytes((p.LIGHT,0,0,0,0)))
        self.assertEqual(core.status['phase'],6)

    def test_no_switch_while_moving_and_no_drive_with_off_unknown(self):
        core=ready_core();core.status['vehicle_stopped']=False
        with self.assertRaisesRegex(ValueError,'measured_stop'):
            core.request(request('关闭斑马线检测'),100.)
        for enabled in (False,None):
            core=ready_core();core.task='traffic';core.fpga_task_mode=3
            core.status['zebra_detection_enabled']=enabled
            with self.assertRaisesRegex(ValueError,'enable_fpga_zebra'):
                core.request(request('开始交通行驶'),100.)

    def test_off_then_on_requires_new_traffic_cycle_not_old_green(self):
        core=ready_core();core.task='traffic';core.status.update(phase=6,zebra_wait=True)
        core.traffic_wait_latched=True;core.traffic_cycle_seen=True;core.traffic_wait_started=90.
        seq=core.send(p.EXT_STATUS,now=100.)
        core.receive(p.Packet(seq,p.EXT_STATUS|0x80,bytes((6,14,0,0))),100.01)
        self.assertFalse(core.traffic_cycle_seen)
        seq=core.send(p.EXT_STATUS,now=100.2)
        core.receive(p.Packet(seq,p.EXT_STATUS|0x80,bytes((6,30,0,0))),100.21)
        self.assertEqual(core.traffic_wait_started,100.21)
        core.observe_traffic(dict(published_at=100.22,state='RELEASED',move_allowed=True),100.22)
        self.assertEqual(core.outbox[-1][5],0)
        self.assertIn(('mission',{'mode':'reset_traffic'}),core.effects)

    def test_both_extensions_allow_dialogue_without_fpga_uart(self):
        core=ExtensionControl();router=TaskRouter(extension_enabled=True)
        core.request(request('开启拓展任务一'),100.)
        self.assertTrue(core.state(100.)['dialogue_enabled'])
        for action in ('dialogue','scene','report'):
            req=dict(action=action,text='当前有几团火焰',command_id=action,source='web',issued_at=100.)
            self.assertIsNotNone(router.route(req,100.,extension=core.state(100.))['question'])
        core.request(request('开启拓展任务二',cid='two'),100.)
        state=core.state(100.)
        self.assertEqual(state['task'],'dialogue')
        self.assertTrue(state['dialogue_enabled'])
        self.assertFalse(state['real_uart'])
        self.assertEqual(core.outbox,[])
        routed=router.route(request('当前有几团火焰',cid='q'),100.,extension=state)
        self.assertIsNotNone(routed['question'])
        self.assertIsNone(routed['navigation'])
        with self.assertRaisesRegex(ValueError,'real_extension_uart'):
            router.route(request('前进20厘米',cid='move'),100.,extension=state)

    def test_stop_actions_never_blocked_by_mode_or_stale_extension(self):
        for text in ('停车','停止喷水','任务结束'):
            req=request(text,cid=text.encode().hex());req['source']='voice'
            routed=TaskRouter(extension_enabled=True).route(req,100.,extension={})
            self.assertTrue(routed['accepted'])
            self.assertEqual(routed['actuator']['action'],'disarm')

    def test_task2_yolo_remains_enabled(self):
        gate=VisionTaskGate(True)
        gate.update(dict(stamp=100.,mode='inspect',extension_enabled=True,extension_task='dialogue',traffic_active=False),100.,10.)
        self.assertTrue(gate.allowed(100.,10.1))

    def test_text_switch_cannot_smuggle_opposite_boolean(self):
        req=request('关闭斑马线检测');req['enabled']=True
        with self.assertRaisesRegex(ValueError,'text_zebra_switch_mismatch'):
            TaskRouter(extension_enabled=True).route(req,100.,extension=ready_core().state(100.))


if __name__=='__main__':unittest.main()
