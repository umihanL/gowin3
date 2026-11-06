"""Pulse protocol and sole-UART scheduling, no hardware or ROS runtime."""
import unittest
import types
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/rescue_vision'))
from rescue_vision import ov5640_protocol as wire
from rescue_vision.extension_control import ExtensionControl
from rescue_vision.fpga_pump_bridge import FPGAPumpBridge
import test_extension_node as fixture
import test_extension_map_scheduler as scheduler
import test_numbered_navigation as numbered


class PumpBridgeTests(unittest.TestCase):
    def setUp(self):
        self.core = ExtensionControl(True)
        self.core.protocol_ready = True
        self.core.capabilities = wire.capabilities(bytes((0, 1, 63, 1)))
        self.core.status_at = 100.
        self.core.status = dict(vehicle_stopped=True, mode_selected=True)
        self.bridge = FPGAPumpBridge(self.core)
        self.bridge.initialized = True
        self.bridge.feedback_at = 100.
        self.bridge.sample = wire.pump_status(bytes((30, 0, 0, 0)))

    def request(self, **values):
        return dict(action='set_pump', enabled=True, command_id='burst-1',
                    issued_at=100., duration_ms=400, **values)

    def transmitted(self, command, now):
        raw = next(raw for raw in self.core.outbox if raw[4] == command)
        return self.bridge.before_write(raw, now)

    def reply(self, raw, data, now):
        packet = wire.Parser().feed(wire.encode(raw[3], raw[4] | 0x80, data))[0]
        return self.bridge.receive(packet, now)

    def test_old_firmware_remains_usable_without_pump_capability(self):
        self.assertFalse(wire.capabilities(bytes((0, 1, 31, 1)))['pump_b13'])
        self.assertTrue(self.core.capabilities['pump_b13'])
        for flag in (64, 128):
            with self.assertRaises(ValueError):wire.capabilities(bytes((0, 1, flag, 1)))

    def test_payload_rejects_out_of_range_or_bool_duration(self):
        self.assertEqual(wire.pump_payload(1000), b'\xe8\x03\0\0')
        for duration in (True, 0, 1001, 400., None):
            with self.assertRaises(ValueError):wire.pump_payload(duration)

    def test_status_busy_invalid_and_conflicting_flags_are_not_feedback(self):
        for value in (b'\xff'*4, bytes((31, 0, 1, 0)), bytes((30, 0, 1, 0)),
                      bytes((62, 0, 0, 0)), bytes((15, 7, 0, 0))):
            with self.assertRaises(ValueError):wire.pump_status(value)
        self.assertTrue(wire.pump_status(bytes((15, 0, 100, 0)))['pump_on'])

    def test_queue_delay_is_subtracted_from_pulse_not_added_to_it(self):
        self.bridge.request(self.request(), 100.01)
        raw = self.transmitted(wire.PUMP_ON, 100.1)
        self.assertLessEqual(int.from_bytes(raw[5:7], 'little'), 300)
        self.assertGreaterEqual(int.from_bytes(raw[5:7], 'little'), 299)
        self.assertIsNone(self.bridge.state(100.1)['pump_on'])

    def test_expired_queued_on_is_dropped(self):
        self.bridge.request(self.request(), 100.)
        self.assertIsNone(self.transmitted(wire.PUMP_ON, 100.5))
        self.assertFalse(self.bridge.last_result['accepted'])

    def test_state_change_while_on_waits_in_queue_invalidates_request(self):
        self.bridge.request(self.request(), 100.)
        self.core.mode_transition = dict(phase='waiting_stop')
        self.assertIsNone(self.transmitted(wire.PUMP_ON, 100.01))

    def test_same_timestamp_on_cannot_cross_off_barrier(self):
        self.bridge.off(100., 'stop')
        with self.assertRaisesRegex(ValueError, 'stale'):
            self.bridge.request(self.request(), 100.)

    def test_readback_transition_is_bounded_and_not_actual_feedback(self):
        self.bridge.request(self.request(), 100.)
        self.transmitted(wire.PUMP_ON, 100.02)
        current = self.bridge.state(100.03)
        self.assertTrue(current['pending_readback'])
        self.assertEqual(current['pending_command_id'], 'burst-1')
        self.assertIsNone(current['pump_on'])
        self.assertFalse(current['pump_ready'])
        self.assertFalse(self.bridge.state(100.301)['pending_readback'])

    def test_feedback_command_identity_requires_readback_of_each_written_generation(self):
        self.bridge.request(self.request(),100.)
        on=self.transmitted(wire.PUMP_ON,100.02)
        self.reply(on,bytes(4),100.03)
        self.assertIsNone(self.bridge.state(100.03)['feedback_command_id'])
        self.bridge.tick(100.04)
        poll=self.transmitted(wire.PUMP_STATUS,100.04)
        self.reply(poll,bytes((15,0,100,0)),100.05)
        self.assertEqual(self.bridge.state(100.05)['feedback_command_id'],'burst-1')
        self.bridge.off(100.1,'manual-stop')
        self.assertEqual(self.bridge.state(100.1)['feedback_command_id'],'burst-1')
        off=self.transmitted(wire.PUMP_OFF,100.11)
        self.reply(off,bytes(4),100.12)
        self.assertIsNone(self.bridge.state(100.12)['feedback_command_id'])
        self.core.outbox.clear();self.bridge.tick(100.13)
        poll=self.transmitted(wire.PUMP_STATUS,100.13)
        self.reply(poll,bytes((30,0,0,0)),100.14)
        self.assertEqual(self.bridge.state(100.14)['feedback_command_id'],'manual-stop')

    def test_local_queued_pump_on_blocks_motion_before_new_actuator_snapshot_arrives(self):
        fixture.ExtensionAdapterTests.setUpClass()
        cls=fixture.ExtensionAdapterTests.module.ExtensionTask
        node=cls.__new__(cls)
        node.pump_bridge=self.bridge
        node.actuator=dict(stamp=100.,pump_on=False,guard=dict(armed=False))
        self.assertEqual(node.actuator_guard(100.),'')
        self.bridge.request(self.request(),100.)
        self.assertEqual(node.actuator_guard(100.),'pump_output_pending_or_not_confirmed_off')
        on=self.transmitted(wire.PUMP_ON,100.02)
        self.reply(on,bytes(4),100.03)
        self.assertEqual(node.actuator_guard(100.03),'pump_output_pending_or_not_confirmed_off')

    def test_stop_invalidates_prior_query_generation(self):
        self.bridge.sample = wire.pump_status(bytes((15, 0, 100, 0)))
        self.bridge.tick(100.)
        poll = self.transmitted(wire.PUMP_STATUS, 100.)
        self.bridge.before_write(wire.encode(200, wire.STOP), 100.01)
        self.reply(poll, bytes((30, 0, 0, 0)), 100.02)
        self.assertIsNone(self.bridge.state(100.03)['pump_on'])

    def guarded_node(self):
        fixture.ExtensionAdapterTests.setUpClass()
        cls = fixture.ExtensionAdapterTests.module.ExtensionTask
        node = cls.__new__(cls)
        node.pump_bridge = self.bridge
        node.actuator = dict(stamp=100., pump_on=False, guard=dict(armed=False))
        return node

    def test_navigation_disarm_off_does_not_cancel_with_fresh_low(self):
        node = self.guarded_node()
        self.bridge.off(100.01, 'navigation-disarm')
        self.assertEqual(node.actuator_guard(100.01), '')
        off = self.transmitted(wire.PUMP_OFF, 100.02)
        self.assertEqual(node.actuator_guard(100.03), '')
        state = self.bridge.state(100.03)
        self.assertIs(state['pump_on'], False)
        self.assertEqual(state['feedback_stamp'], 100.)
        self.assertIsNone(state['feedback_command_id'])
        self.assertFalse(state['pump_ready'])
        self.reply(off, bytes(4), 100.04)
        self.assertFalse(self.bridge.state(100.04)['pump_ready'])
        self.core.outbox.clear(); self.bridge.tick(100.05)
        poll = self.transmitted(wire.PUMP_STATUS, 100.05)
        self.reply(poll, bytes((30, 0, 0, 0)), 100.06)
        self.assertTrue(self.bridge.state(100.06)['pump_ready'])
        self.assertEqual(self.bridge.state(100.06)['feedback_command_id'], 'navigation-disarm')

    def test_navigation_mode_lockout_off_keeps_original_low_ttl(self):
        node = self.guarded_node()
        self.bridge.sample = wire.pump_status(bytes((46, 6, 0, 0)))
        self.bridge.initialized = False
        self.bridge.tick(100.01)
        self.transmitted(wire.PUMP_OFF, 100.02)
        self.assertEqual(node.actuator_guard(100.03), '')
        self.assertEqual(self.bridge.state(100.03)['feedback_stamp'], 100.)
        self.assertEqual(node.actuator_guard(100.61), 'pump_output_pending_or_not_confirmed_off')

    def test_off_after_sent_on_cannot_reuse_old_low(self):
        node = self.guarded_node()
        self.bridge.request(self.request(), 100.)
        self.transmitted(wire.PUMP_ON, 100.01)
        self.bridge.off(100.02, 'stop-after-on')
        self.transmitted(wire.PUMP_OFF, 100.03)
        self.assertIsNone(self.bridge.state(100.04)['pump_on'])
        self.assertEqual(node.actuator_guard(100.04), 'pump_output_pending_or_not_confirmed_off')

    def test_redundant_off_cannot_extend_low_ttl_or_accept_older_query(self):
        self.bridge.tick(100.)
        poll = self.transmitted(wire.PUMP_STATUS, 100.)
        self.bridge.off(100.4, 'redundant-off')
        self.transmitted(wire.PUMP_OFF, 100.41)
        self.reply(poll, bytes((30, 0, 0, 0)), 100.5)
        self.assertEqual(self.bridge.state(100.5)['feedback_stamp'], 100.)
        self.assertIsNone(self.bridge.state(100.5)['feedback_command_id'])
        self.assertIsNone(self.bridge.state(100.61)['pump_on'])

    def test_stop_and_resume_preserve_measured_low_without_claiming_new_feedback(self):
        for command in (wire.STOP, wire.RESUME):
            with self.subTest(command=command):
                self.setUp()
                self.bridge.before_write(wire.encode(200, command), 100.01)
                state = self.bridge.state(100.02)
                self.assertIs(state['pump_on'], False)
                self.assertEqual(state['feedback_stamp'], 100.)
                self.assertIsNone(state['feedback_command_id'])
                self.assertIsNone(self.bridge.state(100.61)['pump_on'])

    def test_resume_invalidates_prior_query_without_inventing_off(self):
        self.bridge.sample = wire.pump_status(bytes((15, 0, 100, 0)))
        self.bridge.tick(100.)
        poll = self.transmitted(wire.PUMP_STATUS, 100.)
        self.assertTrue(self.bridge.state(100.)['pump_on'])
        resume = wire.encode(201, wire.RESUME)
        self.assertEqual(self.bridge.before_write(resume, 100.01), resume)
        self.reply(poll, bytes((30, 0, 0, 0)), 100.02)
        self.assertIsNone(self.bridge.state(100.03)['pump_on'])
        self.assertTrue(self.bridge.sample['pump_on'])
        self.assertFalse(self.bridge.state(100.03)['pump_ready'])

    def test_resume_discards_queued_on_and_blocks_older_callback(self):
        self.bridge.request(self.request(), 100.)
        self.bridge.before_write(wire.encode(202, wire.RESUME), 100.01)
        self.assertIsNone(self.transmitted(wire.PUMP_ON, 100.02))
        old = self.request(); old['command_id'] = 'after-resume-old-burst'
        with self.assertRaisesRegex(ValueError, 'stale'):
            self.bridge.request(old, 100.03)

    def test_busy_navigation_without_verified_patrol_pause_cannot_enable_pump(self):
        self.core.numbered.active = dict(phase='paused', command_id='route', task_id=1)
        self.assertFalse(self.core.state(100.)['can_move'])
        self.assertFalse(self.bridge.vehicle_ready(100.))
        with self.assertRaisesRegex(ValueError, 'idle_vehicle'):
            self.bridge.request(self.request(), 100.)

    def paused_patrol(self):
        """Actual route/three exposures/STOP ACK/post-ACK query progression."""
        self.core = numbered.running(profile='dialogue')
        self.core.capabilities['pump_b13'] = True
        self.core.actuator = dict(stamp=100.18, pump_on=False, pump_feedback_fresh=True,
                                  guard=dict(armed=False))
        self.core.numbered.active['patrol_fire'] = True
        self.core.numbered.patrol.start(True, 100.09)
        for now in (100.1, 100.13, 100.16):
            self.core.observe_targets(dict(capture_stamp=now, valid=True, frame_valid=True,
                registry_session='cam-a', image_size_wh=[1280,1088],
                targets=[dict(target_id=7,selectable=True,bbox_xyxy=[100,100,200,200])]),now)
        self.assertEqual(self.core.numbered.active['phase'],'pausing')
        seq = self.core.numbered.active['pause_sequence']
        self.core.receive(wire.Packet(seq,wire.STOP|0x80,bytes(4)),100.17)
        numbered.telemetry(self.core,100.18,motion=2,run=False,stopped=True)
        self.assertTrue(self.core.numbered.pause_verified(100.2))
        self.assertEqual(self.core.numbered.patrol.phase,'EXTINGUISHING')
        self.bridge = FPGAPumpBridge(self.core)
        self.bridge.initialized=True
        self.bridge.feedback_at=100.2
        self.bridge.sample=wire.pump_status(bytes((30,0,0,0)))

    def test_verified_patrol_pause_allows_pump_without_allowing_general_motion(self):
        self.paused_patrol()
        self.assertFalse(self.core.state(100.2)['can_move'])
        self.assertTrue(self.bridge.vehicle_ready(100.2))
        value=self.request();value['issued_at']=100.2
        self.bridge.request(value,100.2)
        self.assertIsNotNone(self.transmitted(wire.PUMP_ON,100.21))

    def test_patrol_permission_is_rechecked_against_actual_pause_session_at_write(self):
        changes = (
            lambda: self.core.numbered.host.update(task_id=255),
            lambda: self.core.numbered.host.update(run=True),
            lambda: self.core.status.update(vehicle_stopped=False),
            lambda: self.core.numbered.active.update(phase='resuming'),
            lambda: setattr(self.core.numbered.patrol,'phase','COOLDOWN'),
            lambda: setattr(self.core,'mode_transition',dict(phase='waiting_stop')),
            lambda: setattr(self.core,'zebra_transition',dict(phase='waiting_readback')),
            lambda: setattr(self.core,'status_at',99.),
        )
        for index, change in enumerate(changes):
            with self.subTest(index=index):
                self.paused_patrol()
                value=self.request();value['issued_at']=100.2
                self.bridge.request(value,100.2)
                change()
                self.assertFalse(self.bridge.vehicle_ready(100.21))
                self.assertIsNone(self.transmitted(wire.PUMP_ON,100.21))

    def test_delayed_readback_does_not_extend_freshness(self):
        self.bridge.tick(100.)
        poll = self.transmitted(wire.PUMP_STATUS, 100.)
        self.reply(poll, bytes((30, 0, 0, 0)), 100.5)
        self.assertTrue(self.bridge.state(100.5)['feedback_fresh'])
        self.assertEqual(self.bridge.state(100.5)['feedback_stamp'], 100.)
        self.assertIsNone(self.bridge.state(100.61)['pump_on'])

    def test_mode_change_lockout_reinitializes_off_without_replaying_on(self):
        self.bridge.tick(100.)
        poll = self.transmitted(wire.PUMP_STATUS, 100.)
        self.core.outbox.clear()
        self.reply(poll, bytes((46, 6, 0, 0)), 100.02)
        self.bridge.tick(100.02)
        self.assertEqual([f[4] for f in self.core.outbox], [wire.PUMP_OFF])
        raw=self.transmitted(wire.PUMP_OFF,100.02)
        self.reply(raw,bytes(4),100.03)
        self.core.outbox.clear();self.bridge.tick(100.04)
        poll=self.transmitted(wire.PUMP_STATUS,100.04)
        self.reply(poll,bytes((30,0,0,0)),100.05)
        self.assertTrue(self.bridge.state(100.05)['pump_ready'])
        self.assertNotIn(wire.PUMP_ON,[f[4] for f in self.core.outbox])

    def test_other_lockout_causes_do_not_automatically_clear(self):
        for reason in (1,2,3,4,5):
            self.setUp()
            self.bridge.tick(100.)
            poll=self.transmitted(wire.PUMP_STATUS,100.)
            self.core.outbox.clear()
            self.reply(poll,bytes((46,reason,0,0)),100.02)
            self.bridge.tick(100.02)
            self.assertNotIn(wire.PUMP_OFF,[f[4] for f in self.core.outbox])

    def test_off_cancels_queued_on_and_old_callback_cannot_restart(self):
        self.bridge.request(self.request(), 100.)
        self.bridge.off(100.02, 'stop')
        self.assertIsNone(self.transmitted(wire.PUMP_ON, 100.03))
        old = self.request();old['command_id'] = 'late-old'
        with self.assertRaisesRegex(ValueError, 'stale'):
            self.bridge.request(old, 100.04)

    def test_ack_is_not_actual_on_state(self):
        self.bridge.request(self.request(), 100.)
        raw = self.transmitted(wire.PUMP_ON, 100.)
        self.reply(raw, bytes(4), 100.01)
        self.assertTrue(self.bridge.state(100.02)['ack_accepted'])
        self.assertIsNone(self.bridge.state(100.02)['pump_on'])
        self.bridge.tick(100.02)
        poll = self.transmitted(wire.PUMP_STATUS, 100.02)
        self.reply(poll, bytes((15, 0, 100, 0)), 100.03)
        self.assertTrue(self.bridge.state(100.04)['pump_on'])
        self.assertFalse(self.bridge.state(100.04)['pump_ready'])

    def test_old_status_reply_does_not_overwrite_new_control_generation(self):
        self.bridge.tick(100.)
        poll = self.transmitted(wire.PUMP_STATUS, 100.)
        self.bridge.request(self.request(), 100.)
        self.transmitted(wire.PUMP_ON, 100.01)
        self.reply(poll, bytes((30, 0, 0, 0)), 100.02)
        self.assertIsNone(self.bridge.state(100.02)['pump_on'])

    def test_on_ack_timeout_never_retries_on(self):
        self.bridge.request(self.request(), 100.)
        self.transmitted(wire.PUMP_ON, 100.)
        self.core.outbox.clear()
        self.bridge.tick(100.7)
        self.assertNotIn(wire.PUMP_ON, [raw[4] for raw in self.core.outbox])
        self.assertIn('no_retry', self.bridge.last_result['reason'])

    def test_unknown_offline_feedback_is_none_not_false(self):
        self.assertFalse(self.bridge.state(100.1)['pump_on'])
        self.assertIsNone(self.bridge.state(101.)['pump_on'])
        self.core.real_uart = False
        self.assertIsNone(self.bridge.state(100.1)['pump_on'])

    def test_reject_navigation_in_progress_and_duplicate_on(self):
        self.bridge.request(self.request(), 100.)
        with self.assertRaisesRegex(ValueError, 'duplicate'):
            self.bridge.request(self.request(), 100.)
        self.core.mode_transition = dict(phase='waiting_stop')
        value=self.request();value['command_id']='other'
        with self.assertRaisesRegex(ValueError, 'idle_vehicle'):
            self.bridge.request(value, 100.)


class PumpFirmware(scheduler.FirmwareSerial):
    def __init__(self, clock):
        super().__init__(clock)
        self.on_until = 0.
        self.drop_pump_status = False

    def write(self, raw):
        packet=wire.Parser().feed(raw)[0]
        if packet.command == wire.CAPABILITIES:
            self.writes.append((self.clock[0], packet.command))
            self.rx.extend(wire.encode(packet.sequence, packet.command|0x80, bytes((0,1,63,1))))
            return len(raw)
        if packet.command in (wire.PUMP_ON,wire.PUMP_OFF,wire.PUMP_STATUS):
            self.writes.append((self.clock[0], packet.command))
            data=bytes(4)
            if packet.command == wire.PUMP_ON:
                self.on_until=self.clock[0]+int.from_bytes(packet.data[:2],'little')/1000
            elif packet.command == wire.PUMP_OFF:
                self.on_until=0.
            else:
                if self.drop_pump_status:return len(raw)
                remaining=max(0, int((self.on_until-self.clock[0])*1000))
                data=bytes((15 if remaining else 30,0))+remaining.to_bytes(2,'little')
            self.rx.extend(wire.encode(packet.sequence, packet.command|0x80, data))
            return len(raw)
        return super().write(raw)


class PumpSchedulingTests(scheduler.MapSchedulingTests):
    def node(self):
        n=super().node()
        n.serial=PumpFirmware(self.clock)
        return n

    def test_real_frame_handshake_and_lost_status_preserve_heartbeat_and_off(self):
        self.clock=[100.];n=self.node()
        for _ in range(80):n.tick();self.clock[0]+=.02
        self.assertTrue(n.pump().state(self.clock[0])['pump_ready'])
        self.assertTrue(any(cmd==wire.PUMP_OFF for _,cmd in n.serial.writes))
        n.serial.drop_pump_status=True
        for _ in range(65):n.tick();self.clock[0]+=.02
        beats=[t for t,c in n.serial.writes if c==wire.HEARTBEAT]
        self.assertLess(max(b-a for a,b in zip(beats,beats[1:])),.6)
        self.assertIsNone(n.pump().state(self.clock[0])['pump_on'])
        requested=self.clock[0];n.pump().off(requested)
        for _ in range(10):n.tick();self.clock[0]+=.02
        self.assertTrue(any(c==wire.PUMP_OFF and requested<=t<=requested+.16 for t,c in n.serial.writes))

    def test_off_prioritized_over_queued_on(self):
        self.clock=[100.];n=self.node()
        seed=PumpBridgeTests();seed.setUp();n.core=seed.core;n.pump_bridge=seed.bridge
        n.in_flight=(240,wire.HOST_STATUS,100.)
        n.pump().request(seed.request(),100.)
        n.flush()
        n.pump().off(100.01,'off')
        n.flush()
        self.assertEqual(n.tx_queue[0][4],wire.PUMP_OFF)
        self.assertNotIn(wire.PUMP_ON,[frame[4] for frame in n.tx_queue])

    def test_region_command_survives_disarm_callback_at_next_ros_tick(self):
        from test_area_navigation import Harness
        h = Harness(); h.start(execute_fire=False)
        self.clock = [h.now+.002]; n = self.node()
        n.core = h.c
        n.outputs = dict(mission=object(), auto=object(), actuator=object())
        n.actuator = h.c.actuator
        n.pump_bridge = FPGAPumpBridge(h.c)
        n.pump_bridge.initialized = True
        n.pump_bridge.feedback_at = h.now-.01
        n.pump_bridge.sample = wire.pump_status(bytes((30, 0, 0, 0)))
        # Interaction publishes search + disarm on independent topics. The
        # delayed disarm reaches this owner after the region has been accepted.
        n.pump_bridge.off(self.clock[0], 'region-disarm')
        n.flush()
        self.assertEqual(n.serial.writes[-1][1], wire.PUMP_OFF)
        self.clock[0] += .02
        n.tick()
        self.assertTrue(h.c.area.search.active)
        self.assertNotEqual(h.c.area.navigation.active['phase'], 'stopping')
        self.assertNotIn('pump_output_pending', h.c.area.search.reason)
        self.assertNotIn(wire.STOP, [command for _, command in n.serial.writes])


if __name__ == '__main__':unittest.main()
