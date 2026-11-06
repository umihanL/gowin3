"""HTTP queue must not turn missing FPGA evidence into a real motion command."""
import importlib.util
import http.client
import json
from pathlib import Path
import threading
import unittest
from unittest.mock import Mock

WS = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('extension_presentation', WS / 'tools/presentation_server.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
from rescue_vision.extension_control import ExtensionControl
from rescue_vision.task_interaction import TaskRouter
from rescue_vision import ov5640_protocol as wire


class ExtensionPresentationTests(unittest.TestCase):
    def setUp(self):
        self.now = 1000.
        self.state = module.PresentationState(monotonic=lambda: self.now, wall=lambda: self.now)
        self.state.ros.update(available=True, router_ready=True)
        self.state.update('mission', {'stamp': self.now, 'mode': 'inspect'})
        self.evidence = dict(stamp=self.now, real_uart=True, protocol_ready=True, status_fresh=True,
                             vehicle_stopped=True, can_move=True,
                             capabilities=dict(distance=True, velocity=True, route_turn=True),
                             reference={'confirmed': True})

    def ready(self):
        self.state.update('extension', self.evidence)

    def test_visual_and_dialogue_do_not_require_fpga(self):
        self.state.enqueue('开启拓展任务一', 'vision')
        self.state.enqueue('你好介绍一下自己', 'question')
        sent=[]
        self.state.drain(lambda kind, value: sent.append(value), lambda: self.now)
        self.assertEqual([value['action'] for value in sent], ['extension_vision', 'dialogue'])

    def test_movement_denied_without_new_protocol_not_legacy_ready_flag(self):
        self.state.update('fpga_link', dict(stamp=self.now, dry_run=False, status_fresh=True))
        with self.assertRaisesRegex(module.ApiError, 'extension_status_unavailable'):
            self.state.enqueue('前进20厘米', 'move')

    def test_movement_is_rechecked_after_queue_and_not_retried(self):
        self.ready()
        self.state.enqueue('前进50厘米', 'move')
        self.evidence['can_move'] = False
        self.evidence['reason'] = 'fpga_physical_host_mode_required'
        self.ready()
        self.state.drain(lambda *args: self.fail('must not publish unavailable movement'), lambda: self.now)
        self.assertEqual(self.state.commands['move']['status'], 'rejected')
        self.assertEqual(self.state.commands['move']['reason'], 'fpga_physical_host_mode_required')

    def test_router_acceptance_is_not_hardware_completion(self):
        self.ready()
        self.state.enqueue('前进20厘米', 'move')
        sent=[]
        self.state.drain(lambda kind, value: sent.append(value), lambda: self.now)
        self.assertEqual(sent[0]['action'], 'manual_move')
        self.assertEqual(self.state.commands['move']['status'], 'submitted')
        self.assertNotIn('completed', self.state.commands['move'])

    def test_fresh_ros_receipt_does_not_refresh_old_source(self):
        self.evidence['stamp'] = self.now-.7
        self.ready()
        with self.assertRaisesRegex(module.ApiError, 'real_extension_uart_required'):
            self.state.enqueue('前进20厘米', 'move')

    def test_route_requires_matching_firmware_capability_and_new_reference(self):
        self.evidence['capabilities']['route_turn'] = False
        self.ready()
        with self.assertRaisesRegex(module.ApiError, 'fpga_capability_not_implemented_route_turn'):
            self.state.enqueue('一直前进在第3个路口左转', 'route')
        self.evidence['capabilities']['route_turn'] = True
        self.evidence['reference']['confirmed'] = False
        self.ready()
        with self.assertRaisesRegex(module.ApiError, 'confirm_physical_start_before_junction_route'):
            self.state.enqueue('一直前进在第3个路口左转', 'route')

    def test_reference_is_exact_explicit_and_needs_stopped_board(self):
        self.ready()
        record=self.state.enqueue('确认起点X104.22厘米Y40厘米朝向90度', 'ref')
        self.assertEqual(record['action'], 'set_reference')
        self.evidence['vehicle_stopped']=False
        self.ready()
        self.state.drain(lambda *args: self.fail('lost stop must block reference'), lambda: self.now)
        self.assertEqual(self.state.commands['ref']['reason'], 'measured_stop_required')

    def test_seek_automatic_fire_requires_actual_pump_and_manual_seek_does_not(self):
        self.ready()
        self.state.enqueue('前进直到火焰居中', 'manual')
        with self.assertRaisesRegex(module.ApiError, 'real_actuator_feedback_required'):
            self.state.enqueue('前进直到火焰居中然后自动灭火', 'automatic')

    def test_finish_traffic_preempts_queued_motion_even_when_mission_expired(self):
        self.ready()
        self.state.enqueue('前进20厘米', 'move')
        self.state.latest.pop('mission')
        self.state.enqueue('结束交通任务', 'finish')
        sent=[]
        self.state.drain(lambda kind, value: sent.append(value), lambda: self.now)
        self.assertEqual([value['action'] for value in sent], ['finish_task'])
        self.assertEqual(self.state.commands['move']['reason'], 'superseded_by_stop')


class ExtensionHttpIntegrationTests(unittest.TestCase):
    """Production HTTP → parser → router → core, with explicit offline UART fixtures."""
    def setUp(self):
        self.now=1000.
        self.core=ExtensionControl(real_uart=True,side_camera_verified=True,screen_x_per_forward_sign=-1)
        self.core.protocol_ready=True
        self.core.capabilities=dict(distance=True,velocity=True,route_turn=True,traffic=True,zebra_switch=True)
        self.core.status=dict(phase=0,vehicle_stopped=True,enabled=False,mode_selected=True,zebra_wait=False,zebra_detection_enabled=True)
        self.core.status_at=self.now
        self.core.route_catalog=json.loads((WS/'config/extension_map.json').read_text(encoding='utf-8-sig'))
        self.router=TaskRouter(extension_enabled=True)
        self.state=module.PresentationState(monotonic=lambda:self.now,wall=lambda:self.now)
        self.state.ros.update(available=True,router_ready=True)
        self.refresh()
        self.server=module.BoundedServer(('127.0.0.1',0),module.BaseHTTPRequestHandler)
        self.host='127.0.0.1:'+str(self.server.server_address[1])
        self.server.RequestHandlerClass=module.handler_class(self.state,WS/'web',Mock(),{self.host})
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join()

    def refresh(self):
        self.state.update('extension',self.core.state(self.now))
        self.state.update('mission',dict(stamp=self.now,mode='inspect',vehicle_stopped=True))
        self.actuator=dict(stamp=self.now,dry_run=False,enabled=True,pump_enabled=True,pump_on=False,guard={'armed':False})
        self.state.update('actuators',self.actuator)

    def post(self,text,identifier):
        conn=http.client.HTTPConnection('127.0.0.1',self.server.server_address[1],timeout=3)
        conn.request('POST','/api/command',json.dumps(dict(text=text,request_id=identifier)),
            {'Content-Type':'application/json','Origin':'http://'+self.host,'Host':self.host})
        response=conn.getresponse();status=response.status;result=json.loads(response.read());conn.close()
        return status,result

    def dispatch(self):
        def publish(kind,value):
            self.assertEqual(kind,'command')
            result=self.router.route(value,self.now,mission=dict(stamp=self.now,mode='inspect'),
                extension=self.core.state(self.now),actuator=self.actuator)
            self.state.update('interaction',result)
            if result.get('extension'):
                self.core.request(result['extension'],self.now)
        self.state.drain(publish,lambda:self.now)
        self.refresh()

    def test_independent_traffic_start_finish_never_issues_semantic_movement(self):
        self.assertEqual(self.post('开启交通任务','traffic')[0],202)
        self.dispatch()
        self.assertEqual(self.core.task,'traffic')
        self.assertIsNone(self.core.active)
        self.assertTrue(any(effect[0]=='mission' and effect[1].get('mode')=='traffic' for effect in self.core.effects))
        self.assertEqual(self.post('结束交通任务','finish')[0],202)
        self.dispatch()
        self.assertEqual(self.core.task,'vision')
        self.assertIsNone(self.core.active)
        self.assertFalse(any(frame[4] in (wire.MOVE,wire.VELOCITY,0x03) for frame in self.core.outbox))
        self.assertFalse(self.state.snapshot()['synthetic'])

    def test_traffic_drive_needs_confirmed_mode_then_speed_ack_before_drive(self):
        status,reason=self.post('开始交通行驶','wrong-mode')
        self.assertEqual(status,409)
        self.assertEqual(reason['error'],'select_traffic_task_and_wait_ready_before_drive')
        self.core.task='traffic';self.core.fpga_task_mode=3;self.refresh()
        self.assertEqual(self.post('开始交通行驶','drive')[0],202)
        self.dispatch()
        self.assertEqual(self.core.task,'traffic')
        self.assertEqual(self.core.active['action'],'traffic_drive')
        self.assertEqual(self.core.active['phase'],'waiting_traffic_speed')
        self.assertEqual([frame[4] for frame in self.core.outbox],[0x02])
        seq=self.core.active['speed_sequence']
        self.core.receive(wire.Packet(seq,0x82,bytes(4)),self.now+.05)
        self.assertEqual(self.core.outbox[-1][4],0x03)
        self.assertEqual(self.core.outbox[-1][5],0)
        self.assertFalse(self.core.last_result['completed'])

    def test_traffic_drive_rechecks_transition_and_link_at_dispatch(self):
        self.core.task='traffic';self.core.fpga_task_mode=3;self.refresh()
        self.assertEqual(self.post('开始交通行驶','drive')[0],202)
        self.core.mode_transition={'phase':'waiting_stop','mode':3,'started_at':self.now}
        self.refresh()
        self.state.drain(lambda *args:self.fail('mode transition cannot drive'),lambda:self.now)
        self.assertEqual(self.state.commands['drive']['status'],'rejected')
        self.core.mode_transition=None;self.core.status_at=self.now-1;self.refresh()
        self.assertEqual(self.post('开始交通行驶','stale')[0],409)

    def test_traffic_observation_is_independent_but_drive_needs_actual_zebra_on(self):
        self.core.status['zebra_detection_enabled']=None
        self.refresh()
        self.assertEqual(self.post('开启交通任务','observe')[0],202)
        self.core.task='traffic';self.core.fpga_task_mode=3
        for i,value in enumerate((None,False)):
            self.core.status['zebra_detection_enabled']=value;self.refresh()
            status,result=self.post('开始交通行驶','blocked'+str(i))
            self.assertEqual(status,409)
            self.assertEqual(result['error'],'enable_fpga_zebra_detection_before_traffic_drive')

    def test_zebra_ack_is_not_actual_state_and_later_readback_is_required(self):
        self.core.status['zebra_detection_enabled']=False;self.refresh()
        status,result=self.post('开启FPGA斑马线识别','zebra-on')
        self.assertEqual(status,202);self.assertEqual(result['action'],'set_zebra_detection')
        self.dispatch()
        self.assertFalse(self.core.state(self.now)['zebra_detection_enabled'])
        sequence=self.core.zebra_transition['sequence']
        self.core.receive(wire.Packet(sequence,wire.ZEBRA_ENABLE|0x80,bytes(4)),self.now+.01)
        self.assertEqual(self.core.zebra_transition['phase'],'waiting_readback')
        self.assertFalse(self.core.state(self.now+.01)['zebra_detection_enabled'])
        status_seq=self.core.send(wire.EXT_STATUS,now=self.now+.02)
        self.core.receive(wire.Packet(status_seq,wire.EXT_STATUS|0x80,bytes((0,26,0,0))),self.now+.03)
        self.core.tick(self.now+.04)
        self.assertTrue(self.core.state(self.now+.04)['zebra_detection_enabled'])
        self.assertTrue(self.core.last_result['completed'])
        self.assertIsNone(self.core.zebra_transition)

    def test_zebra_switch_is_rechecked_if_vehicle_starts_after_queue(self):
        self.assertEqual(self.post('关闭FPGA斑马线识别','zebra-off')[0],202)
        self.core.status['vehicle_stopped']=False;self.refresh()
        self.state.drain(lambda *args:self.fail('moving vehicle cannot change detector'),lambda:self.now)
        self.assertEqual(self.state.commands['zebra-off']['reason'],'measured_stop_required_for_zebra_switch')

    def test_stopped_red_wait_can_disable_detection_without_light_or_drive(self):
        self.core.task='traffic';self.core.fpga_task_mode=3
        self.core.active=dict(command_id='traffic-driving',action='traffic_drive',phase='moving',request_sequence=90)
        self.core.status.update(phase=6,zebra_wait=True,vehicle_stopped=True)
        self.refresh()
        self.assertFalse(self.core.state(self.now)['can_move'])
        self.assertEqual(self.post('关闭FPGA斑马线识别','disable-waiting')[0],202)
        self.dispatch()
        self.assertEqual([frame[4] for frame in self.core.outbox],[wire.ZEBRA_ENABLE])
        self.assertEqual(self.core.active['command_id'],'traffic-driving')

    def test_extension_two_enables_dialogue_without_uart_but_does_not_enable_movement(self):
        self.core=ExtensionControl(real_uart=False);self.refresh()
        self.assertEqual(self.post('开启拓展任务二','dialogue-mode')[0],202)
        self.dispatch()
        self.assertEqual(self.core.task,'dialogue')
        self.assertTrue(self.core.state(self.now)['dialogue_enabled'])
        self.assertEqual(self.core.outbox,[])
        self.assertEqual(self.post('前进20厘米','still-no-uart')[0],409)
        self.assertEqual(self.post('当前画面有几个火焰','question-two')[0],202)

    def test_reference_then_first_left_reaches_correct_wire_without_legacy_uart(self):
        self.assertNotIn('fpga_link',self.state.latest)
        self.assertEqual(self.post('确认起点X104.22厘米Y40厘米朝向90度','reference')[0],202)
        self.dispatch()
        self.assertTrue(self.core.reference['confirmed'])
        status,queued=self.post('一直前进在第1个路口左转','route')
        self.assertEqual(status,202);self.assertEqual(queued['action'],'route_turn')
        self.dispatch()
        self.assertEqual(self.core.active['route']['requested_junction'],1)
        self.assertEqual(self.core.active['route']['direction'],'left')
        self.assertEqual(self.core.active['phase'],'waiting_mode')
        self.assertFalse(self.core.reference['confirmed'])
        self.assertEqual([frame[4] for frame in self.core.outbox],[wire.TASK_MODE])
        self.assertFalse(self.core.last_result['completed'])
        self.assertTrue(self.state.commands['route']['receipt']['accepted'])
        self.assertNotIn('completed',self.state.commands['route']['receipt'])

    def test_distance_and_seek_sentences_use_extension_not_old_crc_guard(self):
        for i,text in enumerate(('前进20厘米','后退20厘米','前进直到火焰居中','前进直到火焰居中然后自动灭火')):
            with self.subTest(text=text):
                status,result=self.post(text,'motion'+str(i))
                self.assertEqual(status,202,result)
                self.assertIn(result['action'],('manual_move','seek_fire'))
        self.assertNotIn('fpga_link',self.state.latest)

    def test_visual_dialogue_without_fpga_cannot_sneak_movement(self):
        self.core=ExtensionControl(real_uart=False)
        self.refresh()
        self.assertEqual(self.post('开启拓展任务一','vision')[0],202)
        self.dispatch()
        self.assertEqual(self.core.task,'vision');self.assertEqual(self.core.outbox,[])
        self.assertEqual(self.post('前进20厘米','motion')[0],409)
        self.assertEqual(self.post('当前画面有几个火焰','question')[0],202)


if __name__ == '__main__':
    unittest.main()
