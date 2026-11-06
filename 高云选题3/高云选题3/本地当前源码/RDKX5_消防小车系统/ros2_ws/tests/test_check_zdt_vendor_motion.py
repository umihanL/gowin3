"""Bounded vendor-example diagnostics against decoded UART fixtures, no devices."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


SPEC = importlib.util.spec_from_file_location(
    'check_zdt_vendor_motion', Path(__file__).resolve().parents[1] / 'tools/check_zdt_vendor_motion.py')
check = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(check)
wire = check.wire


class Clock:
    def __init__(self): self.t = 0.
    def __call__(self): return self.t
    def sleep(self, dt): self.t += dt


class MotorFixture:
    """Emm mode0 target accumulator; hardware state is independent of ACK."""
    def __init__(self, clock, *, enabled=True, moving=True, origin=1000,
                 target_offset=0, fault=None, options=0x426):
        self.clock = clock
        self.enabled, self.moving = enabled, moving
        self.position = origin
        self.target = origin + target_offset
        self.realtime_target = self.target
        self.target_accumulator = float(self.target)
        self.frames = []
        self.timeout_s = .15
        self.motion_count = 0
        self.options_reads = 0
        self.options = options
        self.fault = fault
        self.after_motion = False

    def exchange(self, request):
        wire.validate_request(request)
        self.frames.append(bytes(request))
        self.clock.t += .005
        command = wire.Command(request[1])
        if command == wire.Command.POSITION_EMM:
            self.motion_count += 1
            self.after_motion = True
            # The vendor example uses relative-to-previous-target, not live encoder.
            if request[10:] != bytes((0, 0, 0x6b)):
                raise AssertionError('The diagnostic changed relative motion semantics')
            pulses = int.from_bytes(request[6:10], 'big') * (-1 if request[2] else 1)
            if abs(pulses) != 8 or request[3:6] != bytes((0, 1, 1)):
                raise AssertionError('Unbounded or faster motion frame')
            self.target_accumulator += pulses * .1125 / check.COUNT_DEG
            self.target = round(self.target_accumulator)
            self.realtime_target = self.target
            if self.moving:
                self.position = self.target
            if self.fault == 'wrong_target':
                self.position += 22
                self.target += 22
                self.realtime_target = self.target
            elif self.fault == 'overshoot':
                self.position += 100
            elif self.fault == 'uncertain_ack':
                raise TimeoutError('uncertain_FD_ack')
            elif self.fault == 'interrupt':
                raise KeyboardInterrupt('fixture interruption')
            frame = bytes((1, command, 2, 0x6b))
        elif command == wire.Command.ENABLE:
            self.enabled = bool(request[3])
            frame = bytes((1, command, 2, 0x6b))
        elif command == wire.Command.READ_VERSION:
            frame = bytes.fromhex('01 1f 00 ce 23 14 6b')
        elif command == wire.Command.READ_CONFIG:
            data = check.CONFIG
            if self.fault == 'config_changed' and self.after_motion:
                data = data[:-1] + bytes((data[-1] + 1,))
            frame = bytes((1, command)) + data + b'\x6b'
        elif command == wire.Command.READ_OPTIONS:
            self.options_reads += 1
            value = self.options
            if self.fault == 'options_changed' and self.after_motion:
                value = 0x406
            frame = bytes((1, command)) + value.to_bytes(2, 'big') + b'\x6b'
        elif command == wire.Command.READ_STATE:
            state = 3 if self.enabled else 2
            if self.fault == 'driver_fault' and self.after_motion:
                state |= 4
            frame = bytes((1, command, state, 0x6b))
        else:
            if self.fault == 'feedback_timeout' and self.after_motion and self.enabled:
                self.clock.t += self.timeout_s
                raise TimeoutError('read_timeout')
            values = {
                wire.Command.READ_POSITION: self.position,
                wire.Command.READ_TARGET_POSITION: self.target,
                wire.Command.READ_REALTIME_TARGET_POSITION: self.realtime_target,
                wire.Command.READ_POSITION_ERROR: self.target - self.position,
                wire.Command.READ_SPEED: 0,
            }
            value = values[command]
            size = 2 if command == wire.Command.READ_SPEED else 4
            frame = bytes((1, command, int(value < 0))) + abs(value).to_bytes(size, 'big') + b'\x6b'
        return wire.parse_response(frame, expected_address=1, expected_command=command)

    def controls(self):
        return [f for f in self.frames if f[1] in (wire.Command.ENABLE, wire.Command.POSITION_EMM)]


class VendorMotionTests(unittest.TestCase):
    def run_check(self, **kwargs):
        clock = Clock()
        motor = MotorFixture(clock, **kwargs)
        report = {}
        checker = check.VendorMotionCheck(motor, report, clock, clock.sleep)
        return motor, checker, report

    def test_mode0_eight_pulses_each_direction_restore_original_target(self):
        # Small pre-existing following error must not turn mode0 into mode2.
        motor, checker, report = self.run_check(target_offset=5)
        checker.execute()
        moves = [f for f in motor.frames if f[1] == wire.Command.POSITION_EMM]
        self.assertEqual(moves, list(check.FRAMES))
        self.assertTrue(report['completed'])
        self.assertEqual(motor.target, 1005)
        self.assertEqual(report['restored_target_error_counts'], 0)
        self.assertLessEqual(abs(report['return_residual_deg']), .10)
        self.assertTrue(motor.enabled)
        self.assertFalse(any(f[1] == wire.Command.ENABLE for f in motor.frames))
        self.assertTrue(all(s['stable_samples'] == 3 for s in report['segments']))
        self.assertFalse(report['production_compatibility_changed'])

    def test_initially_disabled_requires_fresh_baseline_and_restores_disabled(self):
        motor, checker, report = self.run_check(enabled=False, target_offset=5)
        checker.execute()
        self.assertTrue(report['completed'])
        self.assertEqual([f[3] for f in motor.frames if f[1] == wire.Command.ENABLE], [1, 0])
        self.assertTrue(report['cleanup']['disabled_readback'])
        self.assertFalse(motor.enabled)

    def test_ack_and_arrived_bit_without_real_displacement_cannot_pass(self):
        motor, checker, report = self.run_check(moving=False)
        with self.assertRaisesRegex(RuntimeError, 'deadline'):
            checker.execute()
        self.assertFalse(report['completed'])
        self.assertEqual(motor.motion_count, 1)
        self.assertEqual(motor.controls()[-1], wire.enable_request(1, False))
        self.assertTrue(report['cleanup']['disabled_readback'])
        self.assertLess(checker.clock.t, 8.)

    def test_stale_retained_target_is_rejected_before_enable_or_motion(self):
        motor, checker, report = self.run_check(enabled=False, target_offset=100)
        with self.assertRaisesRegex(RuntimeError, 'retained_target'):
            checker.execute()
        self.assertEqual(motor.controls(), [])
        self.assertNotIn('cleanup', report)

    def test_realtime_target_disagreement_is_rejected_without_actuation(self):
        motor, checker, report = self.run_check(enabled=False)
        motor.realtime_target += 4
        with self.assertRaisesRegex(RuntimeError, 'retained_target'):
            checker.execute()
        self.assertEqual(motor.controls(), [])

    def test_configuration_change_after_first_move_never_sends_return(self):
        for fault in ('config_changed', 'options_changed'):
            with self.subTest(fault=fault):
                motor, checker, report = self.run_check(fault=fault)
                with self.assertRaises(RuntimeError):
                    checker.execute()
                self.assertEqual(motor.motion_count, 1)
                self.assertFalse(report['completed'])
                self.assertTrue(report['cleanup']['disabled_readback'])

    def test_uncertain_write_and_interruption_do_not_retry_or_return(self):
        for fault, error in [('uncertain_ack', TimeoutError), ('interrupt', KeyboardInterrupt)]:
            with self.subTest(fault=fault):
                motor, checker, report = self.run_check(fault=fault)
                with self.assertRaises(error):
                    checker.execute()
                self.assertEqual(motor.motion_count, 1)
                self.assertEqual(sum(f[1] == wire.Command.ENABLE for f in motor.frames), 1)
                self.assertTrue(report['cleanup']['disabled_readback'])

    def test_feedback_timeout_stops_without_return(self):
        motor, checker, report = self.run_check(fault='feedback_timeout')
        with self.assertRaisesRegex(TimeoutError, 'read_timeout'):
            checker.execute()
        self.assertEqual(motor.motion_count, 1)
        self.assertTrue(report['cleanup']['disabled_readback'])

    def test_wrong_target_cannot_pass_even_when_motor_follows_it(self):
        motor, checker, report = self.run_check(fault='wrong_target')
        with self.assertRaisesRegex(RuntimeError, 'deadline'):
            checker.execute()
        self.assertEqual(motor.motion_count, 1)
        self.assertTrue(report['cleanup']['disabled_readback'])

    def test_overshoot_and_driver_fault_stop_after_first_command(self):
        for fault, message in [('overshoot', 'excursion'), ('driver_fault', 'fault_limit')]:
            with self.subTest(fault=fault):
                motor, checker, report = self.run_check(fault=fault)
                with self.assertRaisesRegex(RuntimeError, message):
                    checker.execute()
                self.assertEqual(motor.motion_count, 1)
                self.assertTrue(report['cleanup']['disabled_readback'])

    def test_nonmatching_options_refused_before_controls(self):
        motor, checker, report = self.run_check(options=0x826)
        with self.assertRaisesRegex(RuntimeError, 'observed_configuration'):
            checker.execute()
        self.assertEqual(motor.controls(), [])

    def test_offline_main_never_opens_uart_or_claims_completion(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'plan.json'
            with patch.object(check, 'ZDTSerialTransport') as port, contextlib.redirect_stdout(io.StringIO()):
                result = check.main(['--report', str(path)])
            port.assert_not_called()
            self.assertEqual(result, 0)
            report = json.loads(path.read_text(encoding='utf-8'))
            self.assertFalse(report['hardware_requested'])
            self.assertFalse(report['completed'])

    def test_execution_requires_explicit_unloaded_and_finite_clearance(self):
        for args in (['--execute'], ['--execute', '--unloaded'],
                     ['--execute', '--unloaded', '--clearance-deg', '1.99'],
                     ['--execute', '--unloaded', '--clearance-deg', 'nan'],
                     ['--execute', '--unloaded', '--clearance-deg', 'inf']):
            with self.subTest(args=args), tempfile.TemporaryDirectory() as directory:
                with patch.object(check, 'ZDTSerialTransport') as port, contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit):
                        check.main(args + ['--report', str(Path(directory) / 'result.json')])
                port.assert_not_called()


if __name__ == '__main__':
    unittest.main()
