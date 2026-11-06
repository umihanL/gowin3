"""Explicit offline UART fixtures: no port is opened by these tests."""
import contextlib
import importlib.util
import io
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

SPEC=importlib.util.spec_from_file_location('pd42_small_motion',
    Path(__file__).resolve().parents[1]/'tools/check_pd42_small_motion.py')
check=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(check)
from rescue_vision.pd42_protocol import Command, parse_response


class Clock:
    def __init__(self):self.t=0.
    def __call__(self):return self.t
    def sleep(self,duration):self.t+=duration


class MotorFixture:
    def __init__(self,*,origin=26830,enabled=True,arrive=True,fail_motion=False):
        self.position=origin;self.enabled=enabled;self.arrive=arrive
        self.fail_motion=fail_motion;self.frames=[];self.timeout_s=.12
    def exchange(self,request):
        self.frames.append(request);command=Command(request[2])
        if command==Command.ABSOLUTE_POSITION:
            if self.fail_motion:raise TimeoutError('uncertain_F2_ack')
            direction,acceleration,rpm,magnitude=struct.unpack('>BBHI',request[3:-2])
            assert acceleration==1 and rpm==1
            if self.arrive:self.position=(-1 if direction else 1)*magnitude
            data=b''
        elif command==Command.ENABLE:
            assert request[3]==1, 'must never enable'
            self.enabled=False;data=b'\x01'
        else:
            data={Command.READ_VERSION:b'\x12\x12',Command.READ_POSITION:struct.pack('>i',self.position),
                  Command.READ_SPEED:b'\x00\x00',Command.READ_STALL:b'\x00',
                  Command.READ_ENABLED:bytes((0 if self.enabled else 1,)),Command.READ_STATUS:b'\x00'}[command]
        raw=bytes((0xc5,1,command,1))+data
        return parse_response(raw+bytes((sum(raw)&255,0x5c)),1,command)
    def close(self):pass


class SmallMotionTests(unittest.TestCase):
    def test_default_never_opens_uart_or_invents_absolute_targets(self):
        import json
        with tempfile.TemporaryDirectory() as d,patch.object(check,'PD42SerialTransport') as connection,contextlib.redirect_stdout(io.StringIO()):
            path=Path(d)/'dry.json'
            self.assertEqual(check.main(['--report',str(path)]),0)
            report=json.loads(path.read_text(encoding='utf-8'))
            self.assertIsNone(report['plan']['absolute_targets_counts'])
            self.assertFalse(report['hardware_access_requested'])
        connection.assert_not_called()

    def test_execute_requires_unloaded_and_finite_two_degree_clearance(self):
        for args in (['--execute'],['--execute','--unloaded'],
                     ['--execute','--unloaded','--clearance-deg','1.99'],
                     ['--execute','--unloaded','--clearance-deg','nan'],
                     ['--execute','--unloaded','--clearance-deg','inf']):
            with self.subTest(args=args),patch.object(check,'PD42SerialTransport') as port,contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                check.main(args)
            port.assert_not_called()

    def test_absolute_plan_preserves_origin_sign_and_rejects_overflow(self):
        self.assertEqual(check.targets_from_origin(-200),[-58,-342,-200])
        for origin in (2**31-1,-2**31,True,1.5):
            with self.assertRaises(check.AcceptanceFault):check.targets_from_origin(origin)

    def test_three_absolute_moves_use_real_origin_and_three_stable_encoder_samples(self):
        motor=MotorFixture();clock=Clock();report={}
        checker=check.SmallMotionCheck(motor,report,clock,clock.sleep);checker.run()
        moves=[f for f in motor.frames if f[2]==Command.ABSOLUTE_POSITION]
        self.assertEqual(len(moves),3)
        self.assertEqual(report['absolute_targets_counts'],[26972,26688,26830])
        self.assertEqual(report['final_position_counts'],26830)
        self.assertTrue(report['completed'])
        self.assertTrue(all(s['stable_samples']==3 and s['elapsed_s']<=4 for s in report['segments']))
        self.assertFalse(any(f[2] in (Command.ENABLE,Command.STOP,Command.CLEAR_STATUS) for f in motor.frames))

    def test_not_enabled_never_sends_motion_or_enable(self):
        motor=MotorFixture(enabled=False);clock=Clock()
        with self.assertRaisesRegex(check.AcceptanceFault,'not_enabled'):
            check.SmallMotionCheck(motor,{},clock,clock.sleep).run()
        self.assertTrue(all(f[2]<0xf0 for f in motor.frames))

    def test_missing_arrival_times_out_once_without_reissuing_or_returning(self):
        motor=MotorFixture(arrive=False);clock=Clock();report={}
        checker=check.SmallMotionCheck(motor,report,clock,clock.sleep)
        with self.assertRaisesRegex(check.AcceptanceFault,'deadline'):
            checker.run()
        checker.fault_stop_disable()
        self.assertEqual(sum(f[2]==Command.ABSOLUTE_POSITION for f in motor.frames),1)
        self.assertEqual(sum(f[2]==Command.ENABLE for f in motor.frames),1)
        self.assertFalse(any(f[2]==Command.STOP for f in motor.frames))
        self.assertTrue(report['fault_cleanup']['disabled_readback'])
        self.assertLessEqual(clock.t,4.09)

    def test_excursion_checked_before_more_reads_and_no_second_motion(self):
        class Overshoot(MotorFixture):
            def exchange(self,request):
                result=super().exchange(request)
                if request[2]==Command.ABSOLUTE_POSITION:self.position+=check.degrees_to_counts(1.)
                return result
        motor=Overshoot();clock=Clock();report={}
        checker=check.SmallMotionCheck(motor,report,clock,clock.sleep)
        with self.assertRaisesRegex(check.AcceptanceFault,'excursion'):
            checker.run()
        self.assertEqual(motor.frames[-1][2],Command.READ_POSITION)
        checker.fault_stop_disable()
        self.assertEqual(report['motion_commands_sent'],1)

    def test_uncertain_motion_ack_is_not_retried_and_disables_once(self):
        motor=MotorFixture(fail_motion=True);clock=Clock();report={}
        checker=check.SmallMotionCheck(motor,report,clock,clock.sleep)
        with self.assertRaises(TimeoutError):checker.run()
        checker.fault_stop_disable()
        self.assertEqual(report['motion_commands_sent'],1)
        self.assertEqual(report['disable_commands_sent'],1)
        self.assertTrue(report['fault_cleanup']['disabled_readback'])

    def test_keyboard_interrupt_after_possible_motion_uses_fail_safe_disable(self):
        class Interrupted(MotorFixture):
            def exchange(self,request):
                result=super().exchange(request)
                if request[2]==Command.ABSOLUTE_POSITION:raise KeyboardInterrupt('offline_interrupt_fixture')
                return result
        motor=Interrupted();clock=Clock()
        original_init=check.SmallMotionCheck.__init__
        def initialize(instance,transport,report):original_init(instance,transport,report,clock,clock.sleep)
        with tempfile.TemporaryDirectory() as d,patch.object(check,'PD42SerialTransport',return_value=motor) as connection,patch.object(check.SmallMotionCheck,'__init__',initialize),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(check.main(['--execute','--unloaded','--clearance-deg','2','--report',str(Path(d)/'report.json')]),1)
        connection.assert_called_once_with('/dev/ttyS1',baudrate=115200,timeout_s=.12)
        self.assertFalse(motor.enabled)
        self.assertEqual(sum(f[2]==Command.ENABLE for f in motor.frames),1)

    def test_preflight_failure_does_not_change_existing_motor_state(self):
        motor=MotorFixture(enabled=False);clock=Clock()
        original_init=check.SmallMotionCheck.__init__
        def initialize(instance,transport,report):original_init(instance,transport,report,clock,clock.sleep)
        with tempfile.TemporaryDirectory() as d,patch.object(check,'PD42SerialTransport',return_value=motor),patch.object(check.SmallMotionCheck,'__init__',initialize),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(check.main(['--execute','--unloaded','--clearance-deg','2','--report',str(Path(d)/'report.json')]),1)
        self.assertTrue(all(f[2]<0xf0 for f in motor.frames))


if __name__=='__main__':unittest.main()
