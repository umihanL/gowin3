"""Offline ZDT wire fixtures; these tests never open any serial device."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SPEC=importlib.util.spec_from_file_location('zdt_small_motion',Path(__file__).resolve().parents[1]/'tools/check_zdt_small_motion.py')
check=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(check)
wire=check.wire
LIVE_CONFIG=bytes.fromhex('01 42 21 15 19 02 02 02 00 10 01 00 04 B0 0B B8 13 88 05 07 01 00 01 01 00 08 08 98 07 D0 00 08 6B')


class Clock:
    def __init__(self):self.t=0.
    def __call__(self):return self.t
    def sleep(self,duration):self.t+=duration


class MotorFixture:
    def __init__(self,*,origin=21200,options=6,enabled=True,arrive=True,fail_motion=False):
        self.position=origin;self.target=origin;self.enabled=enabled;self.options=options
        self.arrive=arrive;self.fail_motion=fail_motion;self.frames=[];self.timeout_s=.2
    def exchange(self,request):
        self.frames.append(request);command=wire.Command(request[1])
        wire.validate_request(request)
        if command==wire.Command.POSITION_EMM:
            if self.fail_motion:raise TimeoutError('uncertain_FD_ack')
            assert request[-3:]==bytes((2,0,0x6b))
            assert request[3:6]==bytes((0,1,1))
            pulses=int.from_bytes(request[6:10],'big')*(-1 if request[2] else 1)
            self.target=self.position+round(pulses*check.PULSE_DEG*65536/360)
            if self.arrive:self.position=self.target
            frame=bytes((1,command,2,0x6b))
        elif command==wire.Command.ENABLE:
            assert request[3]==0,'must never auto-enable'
            self.enabled=False;frame=bytes((1,command,2,0x6b))
        elif command==wire.Command.READ_VERSION:frame=bytes.fromhex('01 1f 00 ce 23 14 6b')
        elif command==wire.Command.READ_CONFIG:frame=LIVE_CONFIG
        elif command==wire.Command.READ_OPTIONS:frame=bytes((1,command))+self.options.to_bytes(2,'big')+b'\x6b'
        elif command==wire.Command.READ_STATE:frame=bytes((1,command,3 if self.enabled else 2,0x6b))
        else:
            value= self.position if command==wire.Command.READ_POSITION else self.target if command in (
                wire.Command.READ_TARGET_POSITION,wire.Command.READ_REALTIME_TARGET_POSITION) else 0
            size=2 if command==wire.Command.READ_SPEED else 4
            frame=bytes((1,command,int(value<0)))+abs(value).to_bytes(size,'big')+b'\x6b'
        return wire.parse_response(frame,expected_address=1,expected_command=command)
    def close(self):pass


class ZDTMotionTests(unittest.TestCase):
    def test_offline_default_never_opens_uart_or_invents_positions(self):
        with tempfile.TemporaryDirectory() as d,patch.object(check,'ZDTSerialTransport') as port,contextlib.redirect_stdout(io.StringIO()):
            path=Path(d)/'plan.json';self.assertEqual(check.main(['--report',str(path)]),0)
            report=json.loads(path.read_text(encoding='utf-8'))
            self.assertFalse(report['hardware_access_requested']);self.assertIsNone(report['plan']['absolute_targets'])
        port.assert_not_called()

    def test_execute_requires_unloaded_and_finite_clearance(self):
        for args in (['--execute'],['--execute','--unloaded'],['--execute','--unloaded','--clearance-deg','1.9'],
                     ['--execute','--unloaded','--clearance-deg','nan'],['--execute','--unloaded','--clearance-deg','inf']):
            with self.subTest(args=args),patch.object(check,'ZDTSerialTransport') as port,contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):check.main(args)
            port.assert_not_called()

    def test_pulse_conversion_and_bounded_quantization_use_current_position(self):
        first=check.relative_plan(1000,1000,1.)
        self.assertEqual(first['pulses'],9);self.assertAlmostEqual(first['expected_offset_deg'],1.0125)
        for current in range(700,1301):
            for target in (-1.,0.,1.):
                r=check.relative_plan(1000,current,target)
                self.assertLessEqual(abs(r['expected_offset_deg']),1.02)
                self.assertLessEqual(abs(r['expected_offset_deg']-target),.12)
        with self.assertRaises(check.AcceptanceFault):check.relative_plan(1000,1400,0.)

    def test_unknown_0406_is_rejected_before_any_motion(self):
        motor=MotorFixture(options=0x406);clock=Clock();report={}
        with self.assertRaisesRegex(wire.ProtocolError,'not verified'):
            check.SmallMotionCheck(motor,report,clock,clock.sleep).run()
        self.assertTrue(all(f[1]<0xf0 for f in motor.frames))
        self.assertFalse(report['normalized_options']['layout_verified'])

    def test_three_moves_keep_enabled_and_report_residual_and_both_targets(self):
        motor=MotorFixture();clock=Clock();report={}
        check.SmallMotionCheck(motor,report,clock,clock.sleep).run()
        moves=[f for f in motor.frames if f[1]==wire.Command.POSITION_EMM]
        self.assertEqual(len(moves),3)
        self.assertEqual([int.from_bytes(f[6:10],'big') for f in moves],[9,18,9])
        self.assertEqual([f[2] for f in moves],[0,1,0])
        self.assertTrue(report['completed']);self.assertTrue(motor.enabled)
        self.assertTrue(report['retained_targets_consistent'])
        self.assertLessEqual(abs(report['measured_return_error_deg']),.2)
        self.assertTrue(all(s['stable_samples']==3 and s['elapsed_s']<=4 for s in report['segments']))
        self.assertFalse(any(f[1] in (wire.Command.ENABLE,wire.Command.STOP) for f in motor.frames))

    def test_retained_target_far_away_is_rejected_without_actuation(self):
        motor=MotorFixture();motor.target+=65536;clock=Clock()
        with self.assertRaisesRegex(check.AcceptanceFault,'retained_target'):
            check.SmallMotionCheck(motor,{},clock,clock.sleep).run()
        self.assertTrue(all(f[1]<0xf0 for f in motor.frames))

    def test_motion_timeout_not_retried_disables_once_no_return(self):
        motor=MotorFixture(fail_motion=True);clock=Clock();report={}
        checker=check.SmallMotionCheck(motor,report,clock,clock.sleep)
        with self.assertRaises(TimeoutError):checker.run()
        checker.fault_disable()
        self.assertEqual(report['motion_commands_sent'],1);self.assertEqual(report['disable_commands_sent'],1)
        self.assertTrue(report['fault_cleanup']['disabled_readback'])

    def test_ack_without_arrival_expires_without_next_motion(self):
        motor=MotorFixture(arrive=False);clock=Clock();report={}
        checker=check.SmallMotionCheck(motor,report,clock,clock.sleep)
        with self.assertRaisesRegex(check.AcceptanceFault,'deadline'):checker.run()
        self.assertEqual(report['motion_commands_sent'],1)
        self.assertLessEqual(clock.t,4.09)

    def test_normal_transient_one_degree_error_is_not_treated_as_arrival_or_fault(self):
        class Transition(MotorFixture):
            def __init__(self):super().__init__();self.error_reads=0
            def exchange(self,request):
                response=super().exchange(request)
                if request[1]==wire.Command.POSITION_EMM:self.error_reads=2
                if request[1]==wire.Command.READ_POSITION_ERROR and self.error_reads:
                    self.error_reads-=1
                    return wire.parse_response(bytes.fromhex('01 37 00 00 00 00 b6 6b'))
                return response
        motor=Transition();clock=Clock();report={}
        check.SmallMotionCheck(motor,report,clock,clock.sleep).run()
        self.assertTrue(report['completed'])
        for index in range(1,4):
            samples=[s for s in report['samples'] if s['stage']=='segment_%d'%index]
            self.assertEqual(len(samples),5)
            self.assertGreater(samples[0]['position_error_deg'],.9)
            self.assertEqual(samples[-1]['position_error_deg'],0)

    def test_large_following_error_rejects_without_another_segment(self):
        class BadError(MotorFixture):
            def __init__(self):super().__init__();self.moving=False
            def exchange(self,request):
                response=super().exchange(request)
                if request[1]==wire.Command.POSITION_EMM:self.moving=True
                if request[1]==wire.Command.READ_POSITION_ERROR and self.moving:
                    return wire.parse_response(bytes.fromhex('01 37 00 00 00 02 22 6b'))
                return response
        motor=BadError();clock=Clock();report={}
        with self.assertRaisesRegex(check.AcceptanceFault,'exceeds_2_5'):
            check.SmallMotionCheck(motor,report,clock,clock.sleep).run()
        self.assertEqual(report['motion_commands_sent'],1)

    def test_overshoot_is_checked_before_more_reads(self):
        class Overshoot(MotorFixture):
            def exchange(self,request):
                response=super().exchange(request)
                if request[1]==wire.Command.POSITION_EMM:self.position+=200
                return response
        motor=Overshoot();clock=Clock();report={}
        with self.assertRaisesRegex(check.AcceptanceFault,'excursion'):
            check.SmallMotionCheck(motor,report,clock,clock.sleep).run()
        self.assertEqual(motor.frames[-1][1],wire.Command.READ_POSITION)
        self.assertEqual(report['motion_commands_sent'],1)

    def test_preflight_does_not_disable_but_interrupted_motion_does(self):
        class Interrupted(MotorFixture):
            def exchange(self,request):
                response=super().exchange(request)
                if request[1]==wire.Command.POSITION_EMM:raise KeyboardInterrupt('offline_fixture')
                return response
        for motor,expected_disable in ((MotorFixture(enabled=False),0),(Interrupted(),1)):
            clock=Clock();original_init=check.SmallMotionCheck.__init__
            def initialize(instance,transport,report):original_init(instance,transport,report,clock,clock.sleep)
            with tempfile.TemporaryDirectory() as d,patch.object(check,'ZDTSerialTransport',return_value=motor) as port,patch.object(check.SmallMotionCheck,'__init__',initialize),contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(check.main(['--execute','--unloaded','--clearance-deg','2','--report',str(Path(d)/'out.json')]),1)
            port.assert_called_once_with('/dev/ttyS3',baudrate=115200,timeout_s=.2)
            self.assertEqual(sum(f[1]==wire.Command.ENABLE for f in motor.frames),expected_disable)


if __name__=='__main__':unittest.main()
