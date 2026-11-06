"""Real mixed adapter against two protocol fixtures; never opens UART/GPIO."""
import json
import pathlib
import sys
import time
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision import zdt_xs_protocol as zdt
from rescue_vision.mixed_actuator_io import (MixedActuatorIO, validate, joint_to_motor_degrees,
                                           motor_to_joint_angle)
from rescue_vision.pd42_protocol import Command
from rescue_vision.local_actuator_io import create_actuator_io
from test_pd42_actuator_io import MotorFixture
from test_zdt_xs_protocol import LIVE_CONFIG, LIVE_OPTIONS, LIVE_VERSION


def snapshot():
    return dict(version=zdt.parse_response(LIVE_VERSION).value,
                config=zdt.parse_response(LIVE_CONFIG).value,
                options=zdt.parse_response(LIVE_OPTIONS).value)


class ZDTFixture:
    def __init__(self, events):
        self.events = events
        self.counts = 0
        self.target_counts = 0
        self.realtime_target_counts = 0
        self.rpm = 0
        self.enabled = True
        self.arrived = True
        self.options = LIVE_OPTIONS
        self.fault = False
        self.fail_position = False
        self.enable_ignored = False
        self.on_position = None
        self.on_read = None

    def exchange(self, packet):
        _, cmd = zdt.validate_request(packet)
        self.events.append(('pan', cmd, packet))
        if cmd.value < 0x80 and self.on_read:
            self.on_read(cmd)
        reads = {zdt.Command.READ_VERSION: LIVE_VERSION, zdt.Command.READ_CONFIG: LIVE_CONFIG,
                 zdt.Command.READ_OPTIONS: self.options}
        if cmd in reads:
            return zdt.parse_response(reads[cmd], 1, cmd)
        if cmd in (zdt.Command.READ_POSITION, zdt.Command.READ_SPEED, zdt.Command.READ_TARGET_POSITION,
                   zdt.Command.READ_REALTIME_TARGET_POSITION):
            val = {zdt.Command.READ_POSITION:self.counts, zdt.Command.READ_SPEED:self.rpm,
                   zdt.Command.READ_TARGET_POSITION:self.target_counts,
                   zdt.Command.READ_REALTIME_TARGET_POSITION:self.realtime_target_counts}[cmd]
            payload = bytes((int(val < 0),))+abs(val).to_bytes(2 if cmd == zdt.Command.READ_SPEED else 4, 'big')
        elif cmd == zdt.Command.READ_STATE:
            payload = bytes((int(self.enabled) | int(self.arrived)*2 | int(self.fault)*4,))
        elif cmd == zdt.Command.ENABLE:
            if not self.enable_ignored:
                self.enabled = bool(packet[3])
            payload = bytes((zdt.SUCCESS,))
        elif cmd == zdt.Command.POSITION_EMM:
            if self.on_position:
                self.on_position()
            if self.fail_position:
                raise TimeoutError('ZDT reply lost after write')
            payload = bytes((zdt.SUCCESS,))
        else:
            raise AssertionError('Unexpected ZDT command '+str(cmd))
        return zdt.parse_response(bytes((1, cmd))+payload+bytes((0x6b,)), 1, cmd)


def mixed_fixture():
    cfg = json.loads((ROOT/'config/actuator_io.mixed.template.json').read_text(encoding='utf-8'))
    for name in ('pan', 'tilt'):
        cfg['servo'][name]['reference_counts'] = 0
    for flag in ('enable_on_command', 'reference_confirmed_this_start', 'load_supported_verified', 'calibration_confirmed'):
        cfg['servo'][flag] = True
    cfg['servo']['pan']['zdt_verified_snapshot'] = snapshot()
    io = MixedActuatorIO(cfg)
    io.dry_run = False  # Constructor remains dry: all I/O below is in-memory.
    events = []
    pan, tilt = ZDTFixture(events), MotorFixture('tilt', events)
    io._ports = {'/dev/ttyS1': tilt, '/dev/ttyS3': pan}
    return io, pan, tilt, events


class MixedTests(unittest.TestCase):
    def test_new_roles_keep_motor_serial_wiring_and_axis_limits(self):
        io, pan, tilt, _ = mixed_fixture()
        self.assertIs(io._ports['/dev/ttyS3'], pan)
        self.assertIs(io._ports['/dev/ttyS1'], tilt)
        self.assertEqual(io.servo['pan']['protocol'], 'zdt_xs')
        self.assertEqual(io.servo['tilt']['protocol'], 'pd42')
        self.assertEqual((io.servo['pan']['min_angle_deg'], io.servo['pan']['max_angle_deg']), (30.,150.))
        self.assertEqual((io.servo['tilt']['min_angle_deg'], io.servo['tilt']['max_angle_deg']), (40.,140.))
        pan.counts = round(5*65536/360)
        tilt.counts = round(-3*51200/360)
        io._cycle()
        measured = io.feedback()['actual_angles_deg']
        self.assertAlmostEqual(measured[0], 95., places=2)
        self.assertAlmostEqual(measured[1], 87., places=2)

    def test_former_axis_configuration_rejected_before_serial_open(self):
        io, *_ = mixed_fixture()
        cfg = io.config
        cfg['servo']['pan'], cfg['servo']['tilt'] = cfg['servo']['tilt'], cfg['servo']['pan']
        cfg['dry_run'] = False
        with mock.patch('rescue_vision.mixed_actuator_io.ZDTSerialTransport') as serial:
            with self.assertRaisesRegex(ValueError, 'pan protocol must be zdt_xs'):
                MixedActuatorIO(cfg)
            serial.assert_not_called()

    def test_template_dry_run_has_no_transport_or_fake_feedback(self):
        cfg = json.loads((ROOT/'config/actuator_io.mixed.template.json').read_text(encoding='utf-8'))
        with mock.patch('rescue_vision.mixed_actuator_io.ZDTSerialTransport') as serial:
            io = create_actuator_io(cfg)
            self.assertEqual(io.backend, 'mixed_serial')
            self.assertFalse(io.feedback()['feedback_fresh'])
            self.assertIsNone(io.feedback()['actual_angles_deg'])
            serial.assert_not_called()

    def test_mixed_commands_use_distinct_protocols_and_absolute_units(self):
        io, _, _, events = mixed_fixture()
        io.set_gimbal(91, 92, speed_deg_s=6, lease_s=.5)
        io._cycle()
        pan = [e for e in events if e[0] == 'pan' and e[1] == zdt.Command.POSITION_EMM]
        tilt = [e for e in events if e[0] == 'tilt' and e[1] == Command.ABSOLUTE_POSITION]
        self.assertEqual(len(pan), 1)
        self.assertEqual(len(tilt), 1)
        self.assertEqual(int.from_bytes(pan[0][2][6:10], 'big'), 9)  # pan 1 degree / .1125 degree microstep
        self.assertEqual(pan[0][2][-3], 1)  # coordinate zero absolute; NOT repeated relative increments
        self.assertEqual(int.from_bytes(tilt[0][2][7:11], 'big', signed=True), round(2*51200/360))
        self.assertEqual(io.feedback()['axis_protocols'], ['zdt_xs', 'pd42'])
        self.assertTrue(io.feedback()['command_accepted'])
        self.assertFalse(io.feedback()['settled_measured'])

    def test_startup_reads_only(self):
        io, pan, tilt, events = mixed_fixture()
        pan.enabled = tilt.enabled = False
        io._cycle()
        self.assertTrue(io.feedback()['feedback_fresh'])
        self.assertFalse(any(e[1] in (Command.ENABLE, Command.ABSOLUTE_POSITION,
                                     zdt.Command.ENABLE, zdt.Command.POSITION_EMM) for e in events))

    def test_reference_gearing_direction_are_applied(self):
        io, _, _, _ = mixed_fixture()
        a = dict(io.servo['pan'], reference_counts=16384, gear_ratio=2., direction=-1)
        self.assertEqual(joint_to_motor_degrees(95, a, 'emm'), 80.)
        counts = round(80/360*65536)
        self.assertAlmostEqual(motor_to_joint_angle(counts, a, 'emm'), 95., places=2)

    def test_two_measured_axes_required_after_ack(self):
        io, pan, tilt, _ = mixed_fixture()
        io.set_gimbal(91, 91, speed_deg_s=6, lease_s=.5)
        io._cycle()
        pan.counts = round(65536/360)
        time.sleep(.02)
        io._cycle()
        self.assertFalse(io.feedback()['settled_measured'])
        tilt.counts = round(51200/360)
        time.sleep(.02)
        io._cycle()
        self.assertTrue(io.feedback()['settled_measured'])

    def test_unknown_option_bit_remains_blocking(self):
        io, pan, _, events = mixed_fixture()
        pan.options = bytes.fromhex('01 1a 04 26 6b')
        io.set_gimbal(91, 91, speed_deg_s=6, lease_s=.5)
        with self.assertRaises(RuntimeError):
            io._cycle()
        self.assertFalse(any(e[1] in (Command.ABSOLUTE_POSITION, zdt.Command.POSITION_EMM) for e in events))
        cfg = io.config
        cfg['servo']['pan']['zdt_verified_snapshot']['options'] = zdt.parse_response(pan.options).value
        with self.assertRaises(ValueError):
            validate(cfg)

    def test_zdt_failure_attempts_disable_on_both_axes_and_never_replays(self):
        io, pan, tilt, events = mixed_fixture()
        pan.fail_position = True
        io.set_gimbal(91, 91, speed_deg_s=6, lease_s=.5)
        io._run()
        self.assertFalse(pan.enabled)
        self.assertFalse(tilt.enabled)
        self.assertFalse(io.feedback()['feedback_fresh'])
        self.assertIn('reply lost', io.feedback()['error'])
        self.assertEqual(sum(e[1] == zdt.Command.POSITION_EMM for e in events), 1)

    def test_stop_during_pan_cancels_tilt_motion_and_disables_both(self):
        io, pan, tilt, events = mixed_fixture()
        pan.on_position = io.disable_gimbal
        io.set_gimbal(91, 91, speed_deg_s=6, lease_s=.5)
        io._cycle()
        self.assertFalse(any(e[1] == Command.ABSOLUTE_POSITION for e in events))
        self.assertFalse(pan.enabled or tilt.enabled)

    def test_enable_ack_without_measured_enable_cannot_move(self):
        io, pan, _, events = mixed_fixture()
        pan.enabled = False
        pan.enable_ignored = True
        io.set_gimbal(91, 91, speed_deg_s=6, lease_s=.5)
        with self.assertRaises(RuntimeError):
            io._cycle()
        self.assertFalse(any(e[1] == zdt.Command.POSITION_EMM for e in events))

    def test_disabled_zdt_old_target_blocks_both_axes_even_with_arrived_flag(self):
        io, pan, tilt, events = mixed_fixture()
        pan.enabled = False
        pan.counts = -7531  # Recorded after manual/external repositioning, target retained at 17.
        pan.target_counts = pan.realtime_target_counts = 17
        io.set_gimbal(91, 91, speed_deg_s=6, lease_s=.5)
        with self.assertRaisesRegex(RuntimeError, 'retained target differs'):
            io._cycle()
        self.assertFalse(any(e[1] in (Command.ENABLE, Command.ABSOLUTE_POSITION,
                                     zdt.Command.ENABLE, zdt.Command.POSITION_EMM) for e in events))

    def test_disabled_zdt_aligned_target_enables_then_moves(self):
        io, pan, tilt, events = mixed_fixture()
        pan.enabled = False
        pan.counts = 19
        pan.target_counts = pan.realtime_target_counts = 17
        io.set_gimbal(91, 91, speed_deg_s=6, lease_s=.5)
        io._cycle()
        before_enable = next(i for i,e in enumerate(events) if e[1] == zdt.Command.ENABLE)
        self.assertTrue(any(e[1] == zdt.Command.READ_TARGET_POSITION for e in events[:before_enable]))
        self.assertTrue(any(e[1] == zdt.Command.POSITION_EMM for e in events))

    def test_zdt_pre_enable_timeout_drift_or_realtime_disagreement_blocks_all_motion(self):
        for kind in ('timeout', 'drift', 'realtime'):
            with self.subTest(kind=kind):
                io, pan, tilt, events = mixed_fixture()
                pan.enabled = False
                def on_read(command):
                    if command == zdt.Command.READ_TARGET_POSITION:
                        if kind == 'timeout': raise TimeoutError('target missing')
                        if kind == 'drift': pan.counts = 10
                        if kind == 'realtime': pan.realtime_target_counts = 8
                pan.on_read = on_read
                io.set_gimbal(91, 91, speed_deg_s=6, lease_s=.5)
                with self.assertRaises((RuntimeError, TimeoutError)):
                    io._cycle()
                self.assertFalse(any(e[1] in (Command.ENABLE, Command.ABSOLUTE_POSITION,
                                             zdt.Command.ENABLE, zdt.Command.POSITION_EMM) for e in events))

    def test_stale_lease_stops_mixed_axes(self):
        io, pan, tilt, _ = mixed_fixture()
        io.set_gimbal(91, 91, speed_deg_s=6, lease_s=.5)
        io._cycle()
        io._goal['deadline'] = time.monotonic()-1
        io._cycle()
        self.assertFalse(pan.enabled or tilt.enabled)

    def test_real_without_zdt_snapshot_fails_before_open(self):
        io, _, _, _ = mixed_fixture()
        cfg = io.config
        cfg['dry_run'] = False
        cfg['servo']['pan']['zdt_verified_snapshot'] = None
        with mock.patch('rescue_vision.mixed_actuator_io.ZDTSerialTransport') as serial:
            with self.assertRaises(ValueError):
                MixedActuatorIO(cfg)
            serial.assert_not_called()


if __name__ == '__main__':
    unittest.main(verbosity=2)
