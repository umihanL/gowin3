"""Finite-pulse pump contracts, using local callbacks only; no GPIO/UART/ROS."""
from copy import deepcopy
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision.fpga_pump_io import FPGAPumpTransport, validate_pump
from rescue_vision import local_actuator_io


def pump_config(enabled=True):
    return dict(backend='fpga_uart', enabled=enabled, pin='B13', active_high=True,
                max_duration_ms=1000, feedback_timeout_s=.6)


def state(now=10., on=False, ready=True, **extra):
    return dict(stamp=now, feedback_stamp=now, feedback_fresh=True, real_uart=True,
                protocol_ready=True, pump_on=on, pump_ready=ready,
                pin='B13', active_high=True, remaining_ms=200 if on else 0, **extra)


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.sent = []
        self.io = FPGAPumpTransport(pump_config(), self.sent.append, dry_run=False,
                                    wall_clock=lambda: self.now)

    def test_submission_is_not_output_feedback_and_on_is_never_renewed(self):
        self.assertIsNone(self.io.snapshot()['pump_on'])
        self.io.observe(state())
        self.io.set(True, duration_ms=400)
        self.assertTrue(self.io.snapshot()['pump_requested_on'])
        self.assertFalse(self.io.snapshot()['pump_on'])
        self.io.set(True, duration_ms=900)
        self.assertEqual(len(self.sent), 1)
        self.assertEqual(self.sent[0]['duration_ms'], 400)
        self.now = 10.1
        self.io.observe(state(self.now, on=True, ready=False))
        self.assertTrue(self.io.snapshot()['pump_on'])
        self.assertFalse(self.io.snapshot()['pump_ready'])
        self.assertTrue(self.io.snapshot()['pump_feedback_fresh'])

    def test_off_request_does_not_claim_output_low_before_feedback(self):
        self.io.observe(state(on=True, ready=False))
        self.io.set(False)
        self.assertTrue(self.io.snapshot()['pump_on'])
        self.assertFalse(self.io.snapshot()['pump_requested_on'])
        self.assertFalse(self.sent[-1]['enabled'])
        self.now += .1
        self.io.observe(state(self.now))
        self.assertFalse(self.io.snapshot()['pump_on'])

    def test_stale_or_disconnected_readback_is_unknown_never_false(self):
        self.io.observe(state())
        self.now = 10.601
        self.assertIsNone(self.io.snapshot()['pump_on'])
        with self.assertRaisesRegex(RuntimeError, 'fresh_off_ready'):
            self.io.set(True)
        self.io.set(False, force=True)
        self.assertFalse(self.sent[-1]['enabled'])
        s = state(self.now); s['real_uart'] = False
        self.io.observe(s)
        self.assertIsNone(self.io.snapshot()['pump_on'])

    def test_old_readback_does_not_replace_new_and_restart_invalidates(self):
        self.io.observe(state(10., on=True, ready=False))
        self.io.observe(state(9.9))
        self.assertTrue(self.io.snapshot()['pump_on'])
        self.io.observe(None)
        self.assertIsNone(self.io.snapshot()['pump_on'])

    def test_disabled_pump_and_invalid_duration_cannot_send_on(self):
        self.io.observe(state())
        for duration in (0, 1001, True, 2.5):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                self.io.set(True, duration_ms=duration)
        self.assertEqual(self.sent, [])
        self.io.config['enabled'] = False
        with self.assertRaises(RuntimeError):
            self.io.set(True)
        self.io.set(False)
        self.assertFalse(self.sent[-1]['enabled'])

    def test_wrong_pin_or_unverified_protocol_cannot_admit_on(self):
        for key, value in (('pin', '16'), ('active_high', False), ('protocol_ready', False),
                           ('feedback_fresh', False), ('pump_ready', False)):
            s = state(); s[key] = value
            self.io.observe(s)
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                self.io.set(True)
        self.assertEqual(self.sent, [])

    def test_finite_bursts_need_explicit_off_between_and_close_requests_off(self):
        self.io.observe(state())
        self.io.set(True, duration_ms=300)
        first = self.sent[-1]['command_id']
        self.io.set(False)
        self.io.set(True, duration_ms=100)
        self.assertEqual([x['enabled'] for x in self.sent], [True, False, True])
        self.assertNotEqual(first, self.sent[-1]['command_id'])
        self.io.close()
        self.assertFalse(self.sent[-1]['enabled'])
        with self.assertRaises(RuntimeError):
            self.io.set(True)

    def test_dry_run_preserves_real_readback_but_never_emits(self):
        io = FPGAPumpTransport(pump_config(), self.sent.append, dry_run=True,
                              wall_clock=lambda: self.now)
        self.assertIsNone(io.snapshot()['pump_on'])
        io.observe(state())
        io.set(True); io.set(False); io.close()
        self.assertEqual(self.sent, [])
        self.assertFalse(io.snapshot()['pump_on'])
        self.assertTrue(io.snapshot()['pump_feedback_fresh'])
        self.assertFalse(io.snapshot()['pump_ready'])

    def test_dry_run_does_not_treat_simulated_or_invalid_feedback_as_off(self):
        for key, value in (('real_uart', False), ('protocol_ready', False),
                           ('pin', '16'), ('active_high', False),
                           ('feedback_fresh', False), ('feedback_stamp', 9.),
                           ('feedback_stamp', 11.), ('error', 'disconnected')):
            io = FPGAPumpTransport(pump_config(), self.sent.append, dry_run=True,
                                  wall_clock=lambda: self.now)
            readback = state(); readback[key] = value
            io.observe(readback)
            with self.subTest(key=key, value=value):
                self.assertIsNone(io.snapshot()['pump_on'])
                self.assertFalse(io.snapshot()['pump_feedback_fresh'])
                self.assertFalse(io.snapshot()['pump_ready'])

    def test_disabled_output_reports_real_high_without_claiming_ready(self):
        for dry_run in (False, True):
            io = FPGAPumpTransport(pump_config(enabled=False), self.sent.append,
                                  dry_run=dry_run, wall_clock=lambda: self.now)
            io.observe(state(on=True, ready=False))
            self.assertTrue(io.snapshot()['pump_on'])
            self.assertFalse(io.snapshot()['pump_ready'])
            io.observe(state())
            self.assertFalse(io.snapshot()['pump_on'])
            self.assertFalse(io.snapshot()['pump_ready'])
            with self.assertRaises(RuntimeError):
                io.set(True)
        self.assertEqual(self.sent, [])

    def test_fpga_config_cannot_assign_local_gpio(self):
        cfg = pump_config(); cfg['gpio_chip'] = '/dev/gpiochip0'
        with self.assertRaises(ValueError):
            validate_pump(cfg)
        cfg = pump_config(); cfg['active_high'] = False
        with self.assertRaises(ValueError):
            validate_pump(cfg)

    def test_factory_never_allocates_x5_gpio_for_fpga_pump(self):
        cfg = json.loads((ROOT/'config/actuator_io.template.json').read_text(encoding='utf-8'))
        cfg.update(dry_run=False, pump=pump_config())
        cfg['servo']['calibration_confirmed'] = True
        self.io.observe(state())
        with patch.object(local_actuator_io, '_verify_pwm_controller'), \
             patch.object(local_actuator_io, '_ExclusiveOwner'), \
             patch.object(local_actuator_io, '_HardwarePWM'), \
             patch.object(local_actuator_io, '_PumpGPIO', side_effect=AssertionError('GPIO touched')):
            adapter = local_actuator_io.create_actuator_io(cfg, fpga_pump=self.io)
            self.assertFalse(adapter.gimbal.config['pump']['enabled'])
            self.assertTrue(adapter.config['pump']['enabled'])
            adapter.set_pump(True, duration_ms=200)
            self.assertEqual(self.sent[-1]['duration_ms'], 200)
            adapter.disable_gimbal()
            self.assertFalse(self.sent[-1]['enabled'])
            adapter.close()


class NodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from test_gimbal_self_test_node import module
        cls.mod = module()

    def setUp(self):
        cfg = json.loads((ROOT/'config/actuator_io.template.json').read_text(encoding='utf-8'))
        cfg.update(dry_run=False, pump=pump_config())
        self.mod.LocalActuator.parameters = dict(
            control_config=dict(enabled=True, pump_enabled=True, external_pump_timeout_verified=False),
            io_config=cfg, dry_run=False)
        self.gimbal = SimpleNamespace(config=cfg, requires_measured_feedback=False,
            set_pump=Mock(side_effect=AssertionError('Local GPIO pump called')),
            disable_gimbal=Mock(), set_gimbal=Mock(), close=Mock())
        with patch.object(self.mod, 'create_actuator_io', return_value=self.gimbal):
            self.node = self.mod.LocalActuator()
        self.sent = []
        self.node.fpga_pump.emit = self.sent.append
        self.node.now = 10.

    def observe(self, value):
        self.node.pump_state(SimpleNamespace(data=json.dumps(value)))

    def test_actual_state_stays_unknown_after_start_and_off_command(self):
        self.assertIsNone(self.node.pump_on)
        self.node.write_pump(False)
        self.assertIsNone(self.node.pump_on)
        self.assertTrue(self.node.self_test_context(10.)['pump_on'])
        self.observe(state())
        self.assertFalse(self.node.pump_on)
        self.assertFalse(self.node.self_test_context(10.)['pump_on'])

    def test_guard_pulse_duration_is_used_and_stop_does_not_fake_low(self):
        self.observe(state())
        self.node.guard.burst_deadline = 10.4
        with patch.object(self.mod.time, 'monotonic', return_value=10.):
            self.node.write_pump(True)
        self.assertGreaterEqual(self.sent[-1]['duration_ms'], 399)
        self.assertLessEqual(self.sent[-1]['duration_ms'], 400)
        self.assertFalse(self.node.pump_on)
        self.observe(state(on=True, ready=False))
        self.node.stop_outputs('test_stop')
        self.assertTrue(self.node.pump_on)
        self.assertFalse(self.sent[-1]['enabled'])
        self.gimbal.disable_gimbal.assert_called_once()

    def test_manual_disarm_issues_fresh_off_even_when_feedback_missing(self):
        self.node.write_pump(False)
        before = self.sent[-1]['command_id']
        self.node.request(SimpleNamespace(data=json.dumps(dict(action='disarm'))))
        self.assertNotEqual(before, self.sent[-1]['command_id'])
        self.assertFalse(self.sent[-1]['enabled'])
        self.assertIsNone(self.node.pump_on)

    def test_expired_guard_duration_cannot_be_rounded_into_late_on(self):
        self.observe(state())
        self.node.guard.burst_deadline = 10.
        with patch.object(self.mod.time, 'monotonic', return_value=10.01):
            self.node.write_pump(True)
        self.assertFalse(self.sent[-1]['enabled'])
        self.assertFalse(self.node.fpga_pump.requested_on)

    def test_status_emits_unknown_on_stale_feedback_without_latching_io_fault(self):
        self.observe(state())
        self.node.now = 10.7
        with patch.object(self.mod.time, 'monotonic', return_value=10.7):
            self.node.tick()
        out = self.node.output[-1]
        self.assertIsNone(out['pump_on'])
        self.assertFalse(out['pump_feedback_fresh'])
        self.assertFalse(out['pump_ready'])
        self.assertEqual(out['pump_pin'], 'B13')
        self.assertEqual(out['error'], '')

    def test_lost_pump_feedback_disarms_active_burst_and_sends_off(self):
        self.node.fpga_pump.config['feedback_timeout_s'] = .1
        self.observe(state())
        self.node.guard.arm(9.6, 5.)
        self.node.guard.update(9.6, mission_ok=True, command_ok=True, aim_ok=True)
        self.node.guard.update(10., mission_ok=True, command_ok=True, aim_ok=True)
        with patch.object(self.mod.time, 'monotonic', return_value=10.):
            self.node.write_pump(True)
        self.assertTrue(self.sent[-1]['enabled'])
        self.node.now = 10.15
        self.node.mission = dict(mode='fire', vehicle_stopped=True)
        self.node.mission_at = self.node.aim_at = 10.15
        self.node.requested = self.node.commanded = (90., 90.)
        self.node.command_owner = 'fire'; self.node.pwm_active = True
        self.node.command_deadline = 11.; self.node.settled_after = 9.
        self.node.aim = dict(capture_stamp=10.15, valid=True, coordinate_frame='undistorted_left_optical',
            aim=dict(coordinate_frame='undistorted_left_optical', image_size_wh=[1280,1088], camera_calibration_id='fixture'),
            image_locked=True, pump_candidate=True, landing_compensated=True,
            landing_calibration_id='fixture', servo_reference_target_deg=[90.,90.])
        with patch.object(self.mod.time, 'monotonic', return_value=10.15):
            self.node.tick()
        self.assertFalse(self.node.guard.armed)
        self.assertFalse(self.node.fpga_pump.requested_on)
        self.assertFalse(self.sent[-1]['enabled'])
        self.assertIsNone(self.node.output[-1]['pump_on'])


class BridgeTransitionTests(unittest.TestCase):
    """Real bridge generations, real transport, actual actuator callbacks."""
    @classmethod
    def setUpClass(cls):
        from test_gimbal_self_test_node import module
        cls.mod = module()

    def setUp(self):
        NodeTests.setUp(self)
        from rescue_vision.extension_control import ExtensionControl
        from rescue_vision.fpga_pump_bridge import FPGAPumpBridge
        from rescue_vision import ov5640_protocol as wire
        self.wire = wire
        self.core = ExtensionControl(True)
        self.core.protocol_ready = True
        self.core.capabilities = wire.capabilities(bytes((0,1,63,1)))
        self.core.status_at = 100.
        self.core.status = dict(vehicle_stopped=True, mode_selected=True)
        self.bridge = FPGAPumpBridge(self.core)
        self.bridge.initialized = True
        self.bridge.feedback_at = 100.
        self.bridge.sample = wire.pump_status(bytes((30,0,0,0)))
        self.node.now = 100.
        def emit(value):
            self.sent.append(value)
            self.bridge.request(value, self.node.now)
        self.node.fpga_pump.emit = emit
        self.observe()
        self.node.guard.arm(99.6, 5.)
        self.node.guard.update(99.6, mission_ok=True, command_ok=True, aim_ok=True)
        self.node.guard.update(100., mission_ok=True, command_ok=True, aim_ok=True)
        with patch.object(self.mod.time, 'monotonic', return_value=100.):
            self.node.write_pump(True)

    def observe(self):
        NodeTests.observe(self, self.bridge.state(self.node.now))

    def transmit(self, opcode, now):
        self.node.now = now
        raw = next(frame for frame in self.core.outbox if frame[4] == opcode)
        self.core.outbox.remove(raw)
        return self.bridge.before_write(raw, now)

    def reply(self, raw, data, now):
        self.node.now = now
        packet = self.wire.Parser().feed(self.wire.encode(raw[3], raw[4]|0x80, data))[0]
        self.bridge.receive(packet, now)

    def tick(self, now):
        self.node.now = now; self.core.status_at = now
        self.observe()
        self.node.mission = dict(mode='fire', vehicle_stopped=True)
        self.node.mission_at = self.node.aim_at = now
        self.node.requested = self.node.commanded = (90., 90.)
        self.node.command_owner = 'fire'; self.node.pwm_active = True
        self.node.command_deadline = now + .4; self.node.settled_after = now - 1.
        self.node.aim = dict(capture_stamp=now, valid=True, coordinate_frame='undistorted_left_optical',
            aim=dict(coordinate_frame='undistorted_left_optical', image_size_wh=[1280,1088], camera_calibration_id='fixture'),
            image_locked=True, pump_candidate=True, landing_compensated=True,
            landing_calibration_id='fixture', servo_reference_target_deg=[90.,90.])
        with patch.object(self.mod.time, 'monotonic', return_value=now):
            self.node.tick()

    def readback(self, data, now):
        self.bridge.last_poll = -float('inf')
        self.bridge.tick(now)
        query = self.transmit(self.wire.PUMP_STATUS, now)
        self.reply(query, data, now+.01)
        self.observe()

    def test_real_on_ack_and_status_gap_does_not_cancel_valid_burst(self):
        frame = self.transmit(self.wire.PUMP_ON, 100.04)
        self.tick(100.04)
        self.assertTrue(self.node.guard.armed)
        self.assertTrue(self.node.guard.pump_on)
        self.assertIsNone(self.node.pump_on)
        self.assertTrue(self.node.fpga_pump.snapshot()['pump_readback_pending'])
        self.assertEqual([v['enabled'] for v in self.sent], [True])
        self.reply(frame, bytes(4), 100.05)
        self.tick(100.06)
        self.assertTrue(self.node.guard.armed)
        self.assertIsNone(self.node.pump_on)
        self.readback(bytes((15,0,100,0)), 100.08)
        self.tick(100.1)
        self.assertTrue(self.node.pump_on)
        self.assertTrue(self.node.guard.armed)
        self.assertFalse(self.node.fpga_pump.snapshot()['pump_ready'])

    def test_off_generation_gap_keeps_existing_cooldown_but_never_new_arm(self):
        frame = self.transmit(self.wire.PUMP_ON, 100.01)
        self.reply(frame, bytes(4), 100.02)
        self.readback(bytes((15,0,100,0)), 100.03)
        self.tick(100.41)
        self.assertEqual(self.node.guard.state, 'COOLDOWN')
        off = self.transmit(self.wire.PUMP_OFF, 100.42)
        self.tick(100.43)
        self.assertTrue(self.node.guard.armed)
        self.assertEqual(self.node.guard.state, 'COOLDOWN')
        self.assertIsNone(self.node.pump_on)
        self.assertFalse(self.node.fpga_pump.snapshot()['pump_ready'])
        with patch.object(self.mod.time, 'monotonic', return_value=100.43):
            self.node.request(SimpleNamespace(data=json.dumps(dict(action='arm_pump'))))
        self.assertIn('fresh_off_ready', self.node.last_request)
        self.reply(off, bytes(4), 100.44)
        self.readback(bytes((30,0,0,0)), 100.45)
        self.tick(100.46)
        self.assertTrue(self.node.guard.armed)
        self.assertFalse(self.node.pump_on)

    def test_readback_wait_is_bounded_from_local_issued_time(self):
        self.transmit(self.wire.PUMP_ON, 100.2)
        self.tick(100.25)
        self.assertTrue(self.node.guard.armed)
        self.tick(100.301)
        self.assertFalse(self.node.guard.armed)
        self.assertFalse(self.sent[-1]['enabled'])
        self.assertIsNone(self.node.pump_on)

    def test_rejected_ack_and_disconnect_do_not_receive_pending_grace(self):
        frame = self.transmit(self.wire.PUMP_ON, 100.01)
        self.reply(frame, bytes((2,0,0,0)), 100.02)
        self.tick(100.03)
        self.assertFalse(self.node.guard.armed)
        self.assertFalse(self.sent[-1]['enabled'])
        self.assertFalse(self.node.fpga_pump.snapshot()['pump_readback_pending'])

    def test_connection_loss_immediately_cancels_existing_burst(self):
        self.transmit(self.wire.PUMP_ON, 100.01)
        self.core.real_uart = False
        self.tick(100.02)
        self.assertFalse(self.node.guard.armed)
        self.assertFalse(self.sent[-1]['enabled'])
        self.assertIsNone(self.node.pump_on)

    def test_manual_disarm_reissues_off_to_clear_real_bridge_lockout(self):
        self.node.now = 100.01
        self.node.write_pump(False)
        first = self.sent[-1]['command_id']
        self.transmit(self.wire.PUMP_OFF, 100.02)
        self.bridge.sample = self.wire.pump_status(bytes((46,6,0,0)))
        self.bridge.feedback_at = 100.03
        self.node.now = 100.03
        self.observe()
        self.assertFalse(self.node.pump_on)
        self.assertFalse(self.node.fpga_pump.snapshot()['pump_ready'])
        self.node.request(SimpleNamespace(data=json.dumps(dict(action='disarm'))))
        self.assertNotEqual(first, self.sent[-1]['command_id'])
        off = self.transmit(self.wire.PUMP_OFF, 100.04)
        self.reply(off, bytes(4), 100.05)
        self.readback(bytes((30,0,0,0)), 100.06)
        self.assertTrue(self.node.fpga_pump.snapshot()['pump_ready'])


if __name__ == '__main__':
    unittest.main()
