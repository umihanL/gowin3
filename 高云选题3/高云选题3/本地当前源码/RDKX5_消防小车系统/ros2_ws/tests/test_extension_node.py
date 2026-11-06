"""Exercise ROS adapter methods with isolated transport fakes, no board needed."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rescue_vision'))
from rescue_vision.extension_control import ExtensionControl
from rescue_vision import ov5640_protocol as wire


class Serial:
    def __init__(self):self.writes=[];self.closed=False;self.in_waiting=0
    def write(self,data):self.writes.append(data);return len(data)
    def read(self,size):return b''
    def close(self):self.closed=True
    def reset_input_buffer(self):self.reset_done=True


class ExtensionAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        modules={name:types.ModuleType(name) for name in ('rclpy','rclpy.node','std_msgs','std_msgs.msg',
            'rescue_interfaces','rescue_interfaces.msg','rescue_vision.node_common')}
        modules['rclpy.node'].Node=type('Node',(),{'destroy_node':lambda self:None,
            'get_logger':lambda self:types.SimpleNamespace(info=lambda text:None)})
        modules['std_msgs.msg'].String=object
        modules['rescue_interfaces.msg'].FpgaEvent=object
        common=modules['rescue_vision.node_common']
        common.param=lambda n,k,d:d
        common.publish_json=lambda p,v:None
        common.spin=lambda n:None
        common.read_json=lambda p:{}
        spec=importlib.util.spec_from_file_location('rescue_vision._extension_adapter_test',ROOT/'src/rescue_vision/rescue_vision/extension_task_node.py')
        cls.module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,modules):spec.loader.exec_module(cls.module)

    def setUp(self):
        self.node=self.module.ExtensionTask.__new__(self.module.ExtensionTask)
        self.node.core=ExtensionControl(True)
        self.node.serial=Serial();self.node.error=None
        self.node.tx_queue=[];self.node.in_flight=None
        self.node.outputs={};self.node.parser=wire.Parser()
        self.node.now=lambda:100.

    def test_only_one_frame_in_flight_and_polls_coalesced(self):
        node=self.node
        node.core.send(wire.EXT_STATUS,now=100.)
        node.core.send(wire.HEARTBEAT,now=100.)
        node.flush()
        self.assertEqual(len(node.serial.writes),1)
        self.assertEqual(len(node.tx_queue),1)
        for _ in range(5):node.core.send(wire.HEARTBEAT,now=100.)
        node.flush()
        self.assertEqual(len(node.serial.writes),1)
        self.assertEqual(len(node.tx_queue),1)
        node.in_flight=None;node.flush()
        self.assertEqual(len(node.serial.writes),2)

    def test_stop_discards_old_motion_mode_green_and_preserves_new_mode(self):
        node=self.node
        node.core.send(wire.EXT_STATUS,now=100.);node.flush()
        for command,data in ((0x02,b'\x78\0\0\0'),(0x04,b'\x78\0\0\0'),
                             (wire.VELOCITY,wire.velocity_payload(60)),(wire.LIGHT,b'\2\0\0\0'),(wire.TASK_MODE,b'\3\0\0\0')):
            node.core.send(command,data,now=100.)
        node.flush()
        node.core.stop(100.,'user_stop')
        node.core.send(wire.TASK_MODE,b'\1\0\0\0',now=100.)
        node.flush()
        self.assertEqual([f[4] for f in node.tx_queue],[wire.STOP,wire.TASK_MODE])
        self.assertEqual(node.tx_queue[-1][5],1)

    def test_shutdown_sends_stop_even_with_missing_previous_ack(self):
        node=self.node
        node.core.send(wire.EXT_STATUS,now=100.);node.flush()
        serial=node.serial
        with patch.object(self.module.time,'sleep',lambda _:None):node.destroy_node()
        self.assertTrue(serial.closed)
        self.assertEqual(serial.writes[-1][4],wire.STOP)

    def test_operator_stop_discards_queued_patrol_resume(self):
        node=self.node
        node.core.send(wire.EXT_STATUS,now=100.);node.flush()
        node.core.send(wire.RESUME,now=100.);node.flush()
        self.assertIn(wire.RESUME,[f[4] for f in node.tx_queue])
        node.core.stop(100.,'operator_stop');node.flush()
        self.assertNotIn(wire.RESUME,[f[4] for f in node.tx_queue])

    def test_patrol_resume_rechecks_off_at_uart_write_not_only_when_queued(self):
        import test_route_fire_patrol as patrol
        for on in (False,True,None):
            c=patrol.running();patrol.pause(c)
            c.outbox.clear();c.effects.clear()
            c.numbered.resume_after_fire(100.34)
            c.actuator['pump_on']=on
            node=self.node;node.core=c;node.now=lambda:100.35
            node.serial=Serial();node.tx_queue=[];node.in_flight=None
            node.outputs=dict(mission=object(),auto=object(),actuator=object())
            node.pump_bridge=self.module.FPGAPumpBridge(c)
            node.flush()
            writes=[f[4] for f in node.serial.writes]
            self.assertEqual(wire.RESUME in writes,on is False)
            if on is not False:
                self.assertEqual(c.numbered.active['phase'],'stopping')

    def test_pump_status_required_before_and_during_movement(self):
        node=self.node;node.actuator={}
        self.assertTrue(node.actuator_guard(100.))
        node.actuator=dict(stamp=100.,pump_on=False,guard=dict(armed=False))
        self.assertEqual(node.actuator_guard(100.),'')
        node.actuator['guard']['armed']=True
        self.assertTrue(node.actuator_guard(100.))

    def test_retained_area_resume_reaches_uart_and_stale_numbered_done_cannot_hide_guard_rejection(self):
        from test_continuous_area_navigation import ContinuousHarness
        for pump_on in (False, True):
            h = ContinuousHarness();h.launch();h.sample(15);h.pause(15)
            c = h.c
            c.outbox.clear();c.effects.clear()
            c.numbered.active = dict(phase='arrived')
            h.n.resume('continuous-test',h.now+.003)
            c.actuator['pump_on'] = pump_on
            node = self.node;node.core = c;node.now = lambda: h.now+.004
            node.serial = Serial();node.tx_queue = [];node.in_flight = None
            node.outputs = dict(mission=object(), auto=object(), actuator=object())
            node.pump_bridge = self.module.FPGAPumpBridge(c)
            node.flush()
            self.assertEqual(wire.RESUME in [f[4] for f in node.serial.writes], not pump_on)
            if pump_on:
                self.assertEqual(h.n.active['phase'], 'stopping')
                self.assertEqual(h.n.active['reason'], 'continuous_resume_write_guard_rejected')
    def test_region_observe_uses_independent_low_but_fire_keeps_axis_fault_gate(self):
        node = self.node
        node.actuator = dict(stamp=100., pump_on=False, pump_feedback_fresh=True,
            pump_backend='fpga_uart', pump_pin='B13', guard=dict(armed=False),
            error='tilt measured position outside calibrated limits/reference')
        self.assertEqual(node.actuator_guard(100., navigation_only=True), '')
        self.assertEqual(node.actuator_guard(100.), 'pump_off_fresh_actuator_feedback_required')
        for value in (True, None):
            node.actuator['pump_on'] = value
            self.assertEqual(node.actuator_guard(100., navigation_only=True),
                             'pump_off_fresh_actuator_feedback_required')

    def centering_adapter(self):
        from test_retained_centering import CenteringHarness
        h=CenteringHarness()
        h.c.outbox.clear(); h.c.effects.clear()
        node=self.node; node.core=h.c; node.now=lambda:h.now+.006
        node.serial=Serial(); node.tx_queue=[]; node.in_flight=None
        node.outputs=dict(mission=object(),auto=object(),actuator=object())
        node.pump_bridge=self.module.FPGAPumpBridge(h.c)
        return h,node

    def test_retained_jog_rechecks_measured_scope_and_pump_at_actual_uart_write(self):
        for change in ('none','pump','cursor','mode'):
            with self.subTest(change=change):
                h,node=self.centering_adapter()
                sequence=h.begin_jog(acknowledge=False)
                if change=='pump': h.c.actuator['pump_on']=True
                elif change=='cursor': h.n.map['pose']['execution_index']+=1
                elif change=='mode': h.c.mode_transition=dict(phase='waiting_idle')
                node.flush()
                commands=[frame[4] for frame in node.serial.writes]
                self.assertEqual(wire.RETAINED_JOG in commands,change=='none')
                if change=='none':
                    self.assertEqual(node.serial.writes[0][3],sequence)
                    self.assertEqual(node.in_flight[:2],(sequence,wire.RETAINED_JOG))
                    state=h.n.centering.state(node.now())
                    self.assertEqual(state['command_sequence'],sequence)
                    self.assertEqual(state['written_at'],node.now())
                    self.assertIsNone(state['acknowledged_sequence'])
                else:
                    self.assertEqual(h.n.active['phase'],'stopping')
                    self.assertEqual(h.n.active['reason'],'fire_centering_write_guard_rejected')
                    # The allocator may reuse the unsent sequence for STOP;
                    # no pending reply may still represent the rejected 3C.
                    self.assertNotEqual(h.c.pending.get(sequence,(None,))[0],wire.RETAINED_JOG)
                    node.flush()
                    self.assertIn(wire.STOP,[frame[4] for frame in node.serial.writes])
                    self.assertNotIn(wire.RETAINED_JOG,[frame[4] for frame in node.serial.writes])

    def test_failed_retained_jog_write_never_records_written_evidence(self):
        h,node=self.centering_adapter()
        h.begin_jog(acknowledge=False)
        centering=h.n.centering
        with patch.object(centering,'mark_written') as written:
            with patch.object(node.serial,'write',return_value=0):
                node.flush()
            written.assert_not_called()
        self.assertIsNone(node.serial)
        self.assertIn('partial_uart_write',node.error)

    def test_fire_trace_logs_transitions_not_every_camera_frame(self):
        node=self.node;lines=[]
        node.get_logger=lambda:types.SimpleNamespace(info=lines.append)
        node.trace_fire_transition({'stamp':100.,'region_search':{'active':False}})
        self.assertEqual(lines,[])
        region=dict(active=True,command_id='patrol-1',phase='CENTER_CORRECT',reason='forward',
            centering=dict(current_batch_id='batch-1',current_target_id=8,
                queue=[dict(batch_id='batch-1',target_id=8,status='queued')],capture_stamp=100.),
            navigation=dict(phase='centering_starting',centering=dict(command_sequence=12)))
        state=dict(stamp=100.,region_search=region)
        node.trace_fire_transition(state)
        for stamp in (100.1,100.2,100.3):
            state['stamp']=stamp;region['centering']['capture_stamp']=stamp
            node.trace_fire_transition(state)
        self.assertEqual(len(lines),1)
        motion=region['navigation']['centering']
        motion['written_at']=100.4;node.trace_fire_transition(state)
        motion.update(ack_at=100.5,acknowledged_sequence=12);node.trace_fire_transition(state)
        region['centering']['queue'][0]['status']='completed';node.trace_fire_transition(state)
        region.update(active=False,phase='COMPLETE');node.trace_fire_transition(state)
        node.trace_fire_transition(state)
        self.assertEqual(len(lines),5)
        events=[json.loads(line.split(' ',1)[1]) for line in lines]
        self.assertIsNone(events[0]['motion']['written_at'])
        self.assertEqual(events[1]['motion']['written_at'],100.4)
        self.assertEqual(events[2]['motion']['acknowledged_sequence'],12)
        self.assertEqual(events[-1]['phase'],'COMPLETE')

    def test_stop_removes_unwritten_retained_jog_from_both_outbox_and_transport_queue(self):
        for already_queued in (False,True):
            with self.subTest(already_queued=already_queued):
                h,node=self.centering_adapter()
                h.c.send(wire.EXT_STATUS,now=h.now);node.flush()
                jog_sequence=h.begin_jog(acknowledge=False)
                if already_queued:
                    node.flush()
                    self.assertEqual([frame[4] for frame in node.tx_queue],[wire.RETAINED_JOG])
                h.n.stop_centering_jog(h.now+.005)
                node.flush()
                self.assertNotIn(wire.RETAINED_JOG,[frame[4] for frame in node.tx_queue])
                self.assertFalse(h.n.centering_write_ready(jog_sequence,h.now+.006))
                self.assertEqual(node.tx_queue[0][4],wire.STOP)
                node.in_flight=None;node.flush()
                self.assertEqual([frame[4] for frame in node.serial.writes],[wire.EXT_STATUS,wire.STOP])
                # Queue cancellation is an ordinary local stop, not a falsely
                # reported transport or navigation failure.
                self.assertEqual(h.n.active['phase'],'centering_stopping')

    def test_retained_jog_status_poll_is_coalesced_to_latest_sequence(self):
        node=self.node
        node.core.send(wire.EXT_STATUS,now=100.);node.flush()
        for _ in range(4): latest=node.core.send(wire.RETAINED_JOG_STATUS,now=100.)
        node.flush()
        self.assertEqual(len(node.tx_queue),1)
        self.assertEqual((node.tx_queue[0][3],node.tx_queue[0][4]),(latest,wire.RETAINED_JOG_STATUS))
        for _ in range(4): latest=node.core.send(wire.RETAINED_JOG_STATUS,now=100.)
        node.flush()
        self.assertEqual(len(node.tx_queue),1)
        self.assertEqual(node.tx_queue[0][3],latest)
        node.in_flight=None;node.flush()
        self.assertEqual([frame[4] for frame in node.serial.writes],[wire.EXT_STATUS,wire.RETAINED_JOG_STATUS])
        self.assertEqual(node.serial.writes[-1][3],latest)

    def recovery_adapter(self):
        node=self.node
        node.core.route_catalog={'config_marker':'retained'}
        node._uart_core_template=copy.deepcopy(node.core)
        node._uart_core_template.real_uart=False
        node._uart_enabled=True;node._uart_port='/dev/ttyS2';node._uart_closed=False
        node._uart_retry_at=0.;node._uart_retry_delay=.5;node._uart_recovering=False
        node._uart_accept_after=float('-inf')
        node.map_telemetry=self.module.MapTelemetry()
        node.actuator={};node.last_publish=float('inf')
        node.outputs=dict(mission=object(),auto=object(),actuator=object())
        return node

    def test_write_timeout_drops_old_routes_pump_queue_and_all_evidence(self):
        node=self.recovery_adapter();old=node.core;serial=node.serial
        old.active=dict(phase='moving',command_id='old-move',request_sequence=7)
        old.reference=dict(confirmed=True)
        old.used_motion={7};old.seen={'old-request'}
        old.capabilities={'retained_jog':True};old.protocol_ready=True
        old.area.navigation.map={'start_reference':{'generation':9},'pose':{'valid':True}}
        node.pump().seen['old-on']=99.
        node.map_telemetry.pose={'valid':True}
        old.send(wire.START,now=100.)
        old.send(wire.RESUME,now=100.)
        old.send(wire.PUMP_ON,now=100.)
        with patch.object(serial,'write',side_effect=TimeoutError('Write timeout')):
            with patch.object(self.module.time,'monotonic',return_value=10.):node.flush()
        self.assertTrue(serial.closed);self.assertIsNone(node.serial)
        self.assertFalse(node.core.real_uart);self.assertFalse(node.core.protocol_ready)
        self.assertIsNone(node.core.active);self.assertIsNone(node.core.area.navigation.active)
        self.assertFalse(node.core.reference['confirmed']);self.assertEqual(node.core.capabilities,{})
        self.assertEqual(node.core.pending,{});self.assertEqual(node.core.outbox,[]);self.assertEqual(node.tx_queue,[])
        self.assertEqual(node.pump().pending,{});self.assertEqual(node.map_telemetry.pose,{})
        self.assertEqual(node.core.route_catalog,{'config_marker':'retained'})
        self.assertIn(7,node.core.used_motion);self.assertIn('old-request',node.core.seen)
        self.assertIn('old-on',node.pump().seen)
        self.assertEqual(node.core.area.consumed_panel_reference,9)
        self.assertEqual(node._uart_retry_at,10.5)

    def test_uart_reopen_uses_monotonic_bounded_backoff_and_never_old_actions(self):
        node=self.recovery_adapter()
        with patch.object(self.module.time,'monotonic',return_value=10.):
            node._uart_disconnected('write_failed',100.)
        factory=unittest.mock.Mock(side_effect=OSError('temporarily unavailable'))
        with patch.dict(sys.modules,{'serial':types.SimpleNamespace(Serial=factory)}):
            with patch.object(self.module.time,'monotonic',return_value=10.1):
                node._try_uart_open(2000000000.)
            self.assertEqual(factory.call_count,0)
            for stamp,delay in ((10.5,1.),(11.5,2.),(13.5,4.),(17.5,8.),(25.5,8.)):
                with patch.object(self.module.time,'monotonic',return_value=stamp):node._try_uart_open(1.)
                self.assertEqual(node._uart_retry_at,stamp+delay)
        opened=Serial();factory=unittest.mock.Mock(return_value=opened)
        with patch.dict(sys.modules,{'serial':types.SimpleNamespace(Serial=factory)}):
            with patch.object(self.module.time,'monotonic',return_value=33.5):node._try_uart_open(200.)
        factory.assert_called_once_with('/dev/ttyS2',115200,timeout=0,write_timeout=.2,exclusive=True)
        self.assertTrue(opened.reset_done)
        self.assertEqual([frame[4] for frame in node.core.outbox],[wire.STOP])
        self.assertTrue(node._uart_recovering);self.assertFalse(node.core.protocol_ready)
        node.now=lambda:200.;node.flush()
        self.assertEqual([frame[4] for frame in opened.writes],[wire.STOP])

    def test_read_exception_uses_same_disconnect_and_does_not_reopen_during_shutdown(self):
        node=self.recovery_adapter();serial=node.serial
        with patch.object(serial,'read',side_effect=OSError('device disconnected')):
            with patch.object(self.module.time,'monotonic',return_value=10.):node.tick()
        self.assertTrue(serial.closed);self.assertIsNone(node.serial)
        self.assertTrue(node._uart_recovering)
        self.assertIn('uart_read_failed',node.error)
        node._uart_closed=True
        factory=unittest.mock.Mock()
        with patch.dict(sys.modules,{'serial':types.SimpleNamespace(Serial=factory)}):
            with patch.object(self.module.time,'monotonic',return_value=100.):node._try_uart_open(101.)
        factory.assert_not_called()

    def test_recovery_requires_stop_ack_and_new_feedback_and_blocks_old_requests(self):
        node=self.recovery_adapter()
        with patch.object(self.module.time,'monotonic',return_value=10.):node._uart_disconnected('test',100.)
        with patch.dict(sys.modules,{'serial':types.SimpleNamespace(Serial=lambda *args,**kw:Serial())}):
            with patch.object(self.module.time,'monotonic',return_value=10.5):node._try_uart_open(100.5)
        node.now=lambda:100.5;node.flush()
        node.request(types.SimpleNamespace(data=json.dumps(dict(action='extension_dialogue',command_id='during-recovery',issued_at=100.5))))
        self.assertFalse(node.core.last_result['accepted'])
        node.pump_request(types.SimpleNamespace(data=json.dumps(dict(action='set_pump',enabled=True,command_id='old-pump',issued_at=100.5,duration_ms=500))))
        self.assertFalse(node.pump().last_result['accepted'])
        def reply(command,data,stamp):
            seq=node.core.send(command,now=stamp)
            node.core.receive(wire.Packet(seq,command|128,data),stamp+.001)
        reply(wire.CAPABILITIES,bytes((0,2,95,2)),100.51) # v2, no pump hardware capability.
        for command,data in ((wire.HEARTBEAT,bytes((0,0,19,0))),
                             (wire.POINT_STATUS,bytes(4)),(wire.EXT_STATUS,bytes((0,10,0,0)))):
            reply(command,data,100.52)
        with patch.object(self.module.time,'monotonic',return_value=10.6):node._uart_recovery_tick(100.6)
        self.assertTrue(node._uart_recovering)
        self.assertTrue(node.core.state(100.6)['can_move'])
        self.assertFalse(node.extension_state(100.6)['can_move'])
        self.assertEqual(node.extension_state(100.6)['reason'],'uart_reconnecting_wait_for_fresh_feedback')
        # Exercise real tick parsing of the actual written STOP ACK.
        seq=node._uart_stop_sequence
        node.serial.read=lambda size:wire.encode(seq,wire.STOP|128,bytes(4))
        node.now=lambda:100.61
        with patch.object(self.module.time,'monotonic',return_value=10.61):node.tick()
        self.assertEqual(node._uart_stop_ack_at,100.61)
        self.assertTrue(node._uart_recovering) # Earlier samples cannot prove a post-STOP standstill.
        for command,data in ((wire.HEARTBEAT,bytes((0,0,19,0))),
                             (wire.POINT_STATUS,bytes(4)),(wire.EXT_STATUS,bytes((0,10,0,0)))):
            reply(command,data,100.7)
        with patch.object(self.module.time,'monotonic',return_value=10.71):node._uart_recovery_tick(100.71)
        self.assertFalse(node._uart_recovering);self.assertIsNone(node.error)
        self.assertTrue(node.extension_state(100.71)['can_move'])
        self.assertIsNone(node.core.active);self.assertIsNone(node.core.area.navigation.active)
        node.now=lambda:100.72
        node.request(types.SimpleNamespace(data=json.dumps(dict(action='extension_dialogue',command_id='delayed-old-request',issued_at=100.5))))
        self.assertEqual(node.core.last_result['reason'],'uart_reconnected_new_command_required')
        self.assertNotIn(wire.RESUME,[frame[4] for frame in node.serial.writes])
        self.assertNotIn(wire.START,[frame[4] for frame in node.serial.writes])

    def test_uncommissioned_actuator_accepts_real_off_without_allowing_pump_writes(self):
        from rescue_vision.fpga_pump_io import FPGAPumpTransport
        sent=[]
        transport=FPGAPumpTransport(dict(backend='fpga_uart', enabled=False,
            pin='B13', active_high=True), sent.append, dry_run=True, wall_clock=lambda:100.)
        def update(**changes):
            feedback=dict(stamp=100.,feedback_stamp=100.,feedback_fresh=True,
                real_uart=True,protocol_ready=True,pin='B13',active_high=True,
                pump_on=False,pump_ready=True)
            feedback.update(changes)
            transport.observe(feedback)
            self.node.actuator=dict(stamp=100.,guard=dict(armed=False),**transport.snapshot())
        update()
        self.assertEqual(self.node.actuator_guard(100.),'')
        self.assertFalse(self.node.actuator['pump_ready'])
        for changes in (dict(pump_on=True),dict(real_uart=False),dict(feedback_stamp=99.),
                        dict(protocol_ready=False),dict(error='disconnected')):
            update(**changes)
            self.assertEqual(self.node.actuator_guard(100.),'pump_off_fresh_actuator_feedback_required')
        transport.set(False)
        self.assertEqual(sent,[])


if __name__=='__main__':unittest.main()
