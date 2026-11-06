"""Offline gimbal interlock tests; no owner lock, UART or GPIO is opened."""
import json
import pathlib
import struct
import sys
import time
import unittest
from unittest import mock

WORKSPACE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / 'src/rescue_vision'))
from rescue_vision.pd42_protocol import Command, ProtocolError, parse_response
from rescue_vision.pd42_actuator_io import PD42ActuatorIO, validate, angle_to_counts, counts_to_angle


def response(command, data=b'', address=1):
    body = bytes([0xC5, address, int(command), 1]) + data
    return parse_response(body + bytes([sum(body) & 255, 0x5C]))


class MotorFixture:
    def __init__(self, name, events):
        self.name = name
        self.events = events
        self.counts = 0
        self.target_counts = 0
        self.rpm = 0
        self.status = 0
        self.enabled = True
        self.enable_echo_override = None
        self.fail_position = False
        self.on_position = None
        self.on_read = None

    def exchange(self, packet):
        command = Command(packet[2])
        self.events.append((self.name, command, packet))
        if command in (Command.READ_POSITION, Command.READ_SPEED, Command.READ_STATUS, Command.READ_ENABLED,
                       Command.READ_SYSTEM):
            if self.on_read:
                self.on_read(command)
        if command == Command.READ_POSITION:
            return response(command, struct.pack('>i', self.counts), packet[1])
        if command == Command.READ_SPEED:
            return response(command, struct.pack('>h', self.rpm), packet[1])
        if command == Command.READ_STATUS:
            return response(command, bytes([self.status]), packet[1])
        if command == Command.READ_ENABLED:
            return response(command, bytes([0 if self.enabled else 1]), packet[1])
        if command == Command.READ_SYSTEM:
            data = struct.pack('>fhfffhiiiIBBBB', 24., 15, 4., 2., 4., self.rpm,
                               self.target_counts, self.counts, self.target_counts-self.counts,
                               0, int(not self.enabled), 1, 0, 0)
            return response(command, data, packet[1])
        if command == Command.ENABLE:
            self.enabled = packet[3] == 0
            echoed = self.enabled if self.enable_echo_override is None else self.enable_echo_override
            return response(command, bytes([0 if echoed else 1]), packet[1])
        if command == Command.ABSOLUTE_POSITION:
            if self.on_position:
                self.on_position()
            if self.fail_position:
                raise ProtocolError('fixture lost response after motion write')
            # ACK does not change measured position; tests set measurements explicitly.
            return response(command, address=packet[1])
        raise AssertionError('Unexpected fixture command: ' + str(command))


class PumpFixture:
    def __init__(self, events):
        self.events = events
        self.enabled = False

    def set(self, enabled):
        self.enabled = enabled
        self.events.append(('pump', enabled, None))


class AdapterTests(unittest.TestCase):
    def test_locked_axis_cannot_leave_reference(self):
        io, pan, tilt, events = self.fixture()
        io.servo['tilt']['motion_enabled'] = False
        with self.assertRaisesRegex(ValueError, 'physical direction not verified'):
            io.set_gimbal(91, 91)
        self.assertFalse(events)
        io.set_gimbal(91, 90)

    def config(self):
        return json.loads((WORKSPACE/'config/actuator_io.pd42.template.json').read_text(encoding='utf-8-sig'))

    def fixture(self):
        cfg = self.config()
        for name, port in [('pan', '/dev/ttyS1'), ('tilt', '/dev/ttyS2')]:
            cfg['servo'][name]['port'] = port
            cfg['servo'][name]['reference_counts'] = 0
        cfg['servo']['enable_on_command'] = True
        for flag in ('reference_confirmed_this_start', 'load_supported_verified', 'calibration_confirmed'):
            cfg['servo'][flag] = True
        # Constructor is intentionally dry-run: it cannot import/open hardware.
        io = PD42ActuatorIO(cfg)
        io.dry_run = False
        io.config['dry_run'] = False
        events = []
        pan, tilt = MotorFixture('pan', events), MotorFixture('tilt', events)
        io._ports = {'/dev/ttyS1': pan, '/dev/ttyS2': tilt}
        return io, pan, tilt, events

    def test_template_does_not_open_uart_or_invent_feedback(self):
        with mock.patch('rescue_vision.pd42_actuator_io.PD42SerialTransport') as factory:
            io = PD42ActuatorIO(self.config())
            io.set_gimbal(90, 90)
            result = io.feedback()
            self.assertFalse(result['feedback_fresh'])
            self.assertIsNone(result['actual_angles_deg'])
            self.assertFalse(result['settled_measured'])
            self.assertFalse(result['hardware_written'])
            factory.assert_not_called()

    def test_acceleration_above_protocol_limit_fails_before_hardware(self):
        cfg = self.config()
        cfg['servo']['pan']['acceleration'] = 201
        with self.assertRaises(ValueError):
            validate(cfg)

    def test_two_ttl_axes_cannot_share_one_tx_rx_port(self):
        cfg = self.config()
        for axis in ('pan', 'tilt'):
            cfg['servo'][axis]['port'] = '/dev/ttyS1'
        with self.assertRaises(ValueError):
            validate(cfg)

    def test_real_mode_requires_calibration_reference_and_load_confirmation(self):
        cfg = self.config()
        cfg['dry_run'] = False
        with self.assertRaises(ValueError):
            validate(cfg)

    def test_angle_reference_sign_and_gear_are_used(self):
        axis = self.config()['servo']['pan']
        axis.update(reference_counts=1000, reference_angle_deg=90, gear_ratio=2, direction=-1)
        self.assertEqual(angle_to_counts(135, axis), -11800)
        self.assertEqual(counts_to_angle(-11800, axis), 135)
        with self.assertRaises(ValueError):
            angle_to_counts(200, axis)

    def test_initial_poll_reads_only_and_never_enables(self):
        io, pan, tilt, events = self.fixture()
        pan.enabled = tilt.enabled = False
        io._cycle()
        self.assertEqual(len(events), 8)
        self.assertTrue(all(cmd.value < 0xF0 for _name, cmd, _frame in events))
        self.assertTrue(io.feedback()['feedback_fresh'])
        self.assertFalse(io.feedback()['command_accepted'])

    def test_target_coalesces_to_latest_before_io(self):
        io, pan, tilt, events = self.fixture()
        io.set_gimbal(96, 96, lease_s=.5)
        io.set_gimbal(102, 108, lease_s=.5)
        io._cycle()
        motions = [event for event in events if event[1] == Command.ABSOLUTE_POSITION]
        self.assertEqual(len(motions), 2)
        self.assertEqual(int.from_bytes(motions[0][2][7:11], 'big'), angle_to_counts(102, io.servo['pan']))
        self.assertEqual(int.from_bytes(motions[1][2][7:11], 'big'), angle_to_counts(108, io.servo['tilt']))
        self.assertTrue(io.feedback()['command_accepted'])
        self.assertFalse(io.feedback()['settled_measured'])

    def test_accepted_ack_and_old_reading_cannot_claim_settled(self):
        io, pan, tilt, events = self.fixture()
        io.set_gimbal(90, 90, lease_s=.5)
        io._cycle()
        self.assertFalse(io.feedback()['settled_measured'])
        time.sleep(.02)  # Windows Python 3.10 monotonic clock has coarse ticks.
        io._cycle()
        self.assertTrue(io.feedback()['settled_measured'])
        pan.rpm = 1
        io._cycle()
        self.assertFalse(io.feedback()['settled_measured'])

    def test_expired_target_is_never_written(self):
        io, pan, tilt, events = self.fixture()
        io.set_gimbal(96, 96, lease_s=.5)
        io._goal['deadline'] = time.monotonic()-1
        io._cycle()
        self.assertFalse(any(e[1] == Command.ABSOLUTE_POSITION for e in events))

    def test_slow_feedback_cannot_authorize_motion_with_renewed_lease(self):
        io, pan, tilt, events = self.fixture()
        io.servo['feedback_timeout_s'] = .1
        io.set_gimbal(96, 96, lease_s=.5)
        def slow(_command):
            time.sleep(.027)
        pan.on_read = tilt.on_read = slow
        with self.assertRaises(RuntimeError):
            io._cycle()
        self.assertFalse(any(e[1] == Command.ABSOLUTE_POSITION for e in events))

    def test_disable_ack_echo_mismatch_is_not_accepted(self):
        io, pan, tilt, events = self.fixture()
        io._active = True
        pan.enable_echo_override = True
        with self.assertRaises(RuntimeError):
            io._disable_axes()

    def test_enable_ack_echo_mismatch_never_starts_position(self):
        io, pan, tilt, events = self.fixture()
        pan.enabled = False
        pan.enable_echo_override = False
        io.set_gimbal(96, 96, lease_s=.5)
        with self.assertRaises(RuntimeError):
            io._cycle()
        self.assertFalse(any(e[1] == Command.ABSOLUTE_POSITION for e in events))

    def test_disabled_stale_target_on_second_axis_blocks_both_axes_before_any_write(self):
        io, pan, tilt, events = self.fixture()
        tilt.enabled = False
        tilt.target_counts = 1000
        io.set_gimbal(96, 96, lease_s=.5)
        with self.assertRaisesRegex(RuntimeError, 'retained target differs'):
            io._cycle()
        self.assertFalse(any(e[1] in (Command.ENABLE, Command.ABSOLUTE_POSITION) for e in events))

    def test_disabled_aligned_targets_enable_and_move_but_active_loop_has_no_extra_reads(self):
        io, pan, tilt, events = self.fixture()
        pan.enabled = tilt.enabled = False
        io.set_gimbal(96, 96, lease_s=.5)
        io._cycle()
        self.assertEqual(sum(e[1] == Command.READ_SYSTEM for e in events), 4)
        self.assertEqual(sum(e[1] == Command.ABSOLUTE_POSITION for e in events), 2)
        events.clear()
        io._cycle()
        self.assertFalse(any(e[1] == Command.READ_SYSTEM for e in events))

    def test_pre_enable_target_read_failure_or_axis_drift_never_enables(self):
        for kind in ('timeout', 'drift', 'target_changed'):
            with self.subTest(kind=kind):
                io, pan, tilt, events = self.fixture()
                tilt.enabled = False
                reads = []
                def on_read(command):
                    if command == Command.READ_SYSTEM:
                        reads.append(command)
                        if kind == 'timeout':
                            raise TimeoutError('retained target unavailable')
                        if len(reads) == 2:
                            if kind == 'drift': tilt.counts = tilt.target_counts = 10
                            else: tilt.target_counts += 1
                tilt.on_read = on_read
                io.set_gimbal(96, 96, lease_s=.5)
                with self.assertRaises((RuntimeError, TimeoutError)):
                    io._cycle()
                self.assertFalse(any(e[1] in (Command.ENABLE, Command.ABSOLUTE_POSITION) for e in events))

    def test_expiry_during_pre_enable_cannot_enable_either_axis(self):
        io, pan, tilt, events = self.fixture()
        tilt.enabled = False
        io.set_gimbal(96, 96, lease_s=.5)
        def expire(command):
            if command == Command.READ_SYSTEM:
                io._goal['deadline'] = time.monotonic()-1
        tilt.on_read = expire
        io._cycle()
        self.assertFalse(any(e[1] in (Command.ENABLE, Command.ABSOLUTE_POSITION) for e in events))

    def test_lease_expiry_switches_pump_off_before_disabling_axes(self):
        io, pan, tilt, events = self.fixture()
        pump = PumpFixture(events)
        io._pump = pump
        io.config['pump']['enabled'] = True
        io.set_gimbal(90, 90, lease_s=.5)
        io._cycle()
        time.sleep(.02)
        io._cycle()
        io.set_pump(True)
        self.assertTrue(pump.enabled)
        events.clear()
        io._goal['deadline'] = time.monotonic()-1
        io._cycle()
        self.assertFalse(pump.enabled)
        pump_off = next(i for i, e in enumerate(events) if e[0] == 'pump' and e[1] is False)
        disabled = next(i for i, e in enumerate(events) if e[1] == Command.ENABLE and e[2][3] == 1)
        self.assertLess(pump_off, disabled)

    def test_stop_during_first_axis_prevents_second_axis_motion_and_disables(self):
        io, pan, tilt, events = self.fixture()
        io.set_gimbal(96, 96, lease_s=.5)
        pan.on_position = io.disable_gimbal
        io._cycle()
        self.assertFalse(any(e[0] == 'tilt' and e[1] == Command.ABSOLUTE_POSITION for e in events))
        self.assertFalse(io._active)
        self.assertTrue(any(e[1] == Command.ENABLE and e[2][3] == 1 for e in events))

    def test_retarget_switches_pump_off_before_worker_can_move(self):
        io, pan, tilt, events = self.fixture()
        io._pump = PumpFixture(events)
        io.config['pump']['enabled'] = True
        io.set_gimbal(90, 90, lease_s=.5)
        io._cycle()
        time.sleep(.02)
        io._cycle()
        io.set_pump(True)
        events.clear()
        io.set_gimbal(96, 96, lease_s=.5)
        self.assertFalse(io._pump.enabled)
        self.assertFalse(io.feedback()['command_accepted'])
        self.assertEqual(events[0][:2], ('pump', False))
        self.assertFalse(any(e[1] == Command.ABSOLUTE_POSITION for e in events))

    def test_failed_disable_keeps_uncertain_active_state_and_latches_fault(self):
        io, pan, tilt, events = self.fixture()
        io._active = True
        pan.enable_echo_override = True
        with self.assertRaises(RuntimeError):
            io._disable_axes()
        self.assertTrue(io._active)
        self.assertIn('Disable could not be confirmed', io.feedback()['error'])
        self.assertFalse(io.feedback()['settled_measured'])
        with self.assertRaises(RuntimeError):
            io.set_gimbal(90, 90)

    def test_run_fault_latches_and_unknown_motion_gets_disable_attempt(self):
        io, pan, tilt, events = self.fixture()
        io.set_gimbal(96, 96, lease_s=.5)
        pan.fail_position = True
        io._run()
        self.assertIn('fixture lost response', io.feedback()['error'])
        self.assertFalse(io.feedback()['feedback_fresh'])
        self.assertTrue(any(e[1] == Command.ENABLE and e[2][3] == 1 for e in events))
        with self.assertRaises(RuntimeError):
            io.set_gimbal(90, 90)


if __name__ == '__main__':
    unittest.main(verbosity=2)
