"""Offline single-pulse acceptance fixtures, with no serial access."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SPEC=importlib.util.spec_from_file_location('probe_zdt_single',Path(__file__).resolve().parents[1]/'tools/probe_zdt_single_microstep.py')
check=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(check)
wire=check.wire
CONFIG=bytes.fromhex('01 42 21 15 19 02 02 02 00 10 01 00 04 B0 0B B8 13 88 05 07 01 00 01 01 00 08 08 98 07 D0 00 08 6B')


class Clock:
    def __init__(self):self.t=0.
    def monotonic(self):return self.t
    def sleep(self,seconds):self.t+=seconds


class ProbeFixture:
    def __init__(self,*,enabled=False,arrive=True,options=0x426,config=CONFIG,
                 mismatch=False,lost_motion_ack=False,disable_failure=False):
        self.position=-5;self.target=-6;self.enabled=enabled;self.arrive=arrive
        self.options=options;self.config=config;self.mismatch=mismatch
        self.lost_motion_ack=lost_motion_ack;self.disable_failure=disable_failure;self.frames=[]
    def exchange(self,request):
        wire.validate_request(request);self.frames.append(request);command=request[1]
        if command==0xd5:
            raise AssertionError('Probe must never repeat D5: hardware showed mode selection can auto-enable')
        elif command==wire.Command.ENABLE:
            self.enabled=bool(request[3])
            if not self.enabled and self.disable_failure:raise TimeoutError('disable_not_confirmed')
            frame=bytes((1,command,2,0x6b))
        elif command==wire.Command.POSITION_EMM:
            assert request[-3:]==bytes((2,0,0x6b))
            assert request[3:6]==bytes((0,1,1))
            assert int.from_bytes(request[6:10],'big')==1,'only a single microstep allowed'
            delta=round(check.STEP*65536/360)*(-1 if request[2] else 1)
            self.target=self.position+delta
            if self.arrive:self.position=self.target
            if self.mismatch:self.target+=100
            if self.lost_motion_ack:raise TimeoutError('FD_outcome_uncertain')
            frame=bytes((1,command,2,0x6b))
        elif command==wire.Command.READ_VERSION:frame=bytes.fromhex('01 1f 00 ce 23 14 6b')
        elif command==wire.Command.READ_CONFIG:frame=self.config
        elif command==wire.Command.READ_OPTIONS:frame=bytes((1,command))+self.options.to_bytes(2,'big')+b'\x6b'
        elif command==wire.Command.READ_STATE:frame=bytes((1,command,3 if self.enabled else 2,0x6b))
        else:
            value=self.position if command==wire.Command.READ_POSITION else self.target if command in (
                wire.Command.READ_TARGET_POSITION,wire.Command.READ_REALTIME_TARGET_POSITION) else 0
            size=2 if command==wire.Command.READ_SPEED else 4
            frame=bytes((1,command,int(value<0)))+abs(value).to_bytes(size,'big')+b'\x6b'
        return wire.parse_response(frame,expected_address=1,expected_command=command)


def run(link,report):
    clock=Clock()
    with patch.object(check.time,'monotonic',clock.monotonic),patch.object(check.time,'sleep',clock.sleep):
        check.run_probe(link,report)
    return clock


class SingleMicrostepTests(unittest.TestCase):
    def test_default_dryrun_never_opens_uart(self):
        with tempfile.TemporaryDirectory() as d,patch.object(check,'ZDTSerialTransport') as port,contextlib.redirect_stdout(io.StringIO()):
            output=Path(d)/'dry.json'
            self.assertEqual(check.main(['--report',str(output)]),0)
            doc=json.loads(output.read_text(encoding='utf-8'))
            self.assertFalse(doc['completed']);self.assertEqual(doc['motion_commands_attempted'],0)
        port.assert_not_called()

    def test_success_proves_signed_actual_motion_and_returns_without_automatic_disable(self):
        link=ProbeFixture();report={'motion_commands_attempted':0};run(link,report)
        self.assertTrue(report['completed']);self.assertTrue(link.enabled)
        self.assertEqual(report['motion_commands_attempted'],2)
        segments=report['segments'];self.assertEqual(len(segments),2)
        self.assertGreater(segments[0]['measured_delta_deg'],.05)
        self.assertLess(segments[1]['measured_delta_deg'],-.05)
        self.assertTrue(all(s['stable_samples']==3 for s in segments))
        self.assertLessEqual(abs(report['return_residual_deg']),.06)
        self.assertTrue(all(s['unverified_extra_options']==0x400 for s in report['configuration_samples']))
        writes=[f for f in link.frames if f[1]>=0xd0]
        self.assertEqual([f[1] for f in writes],[wire.Command.ENABLE,wire.Command.POSITION_EMM,wire.Command.POSITION_EMM])
        self.assertEqual(writes[0],bytes.fromhex('01 f3 ab 01 00 6b'))

    def test_ack_without_motion_never_passes_or_sends_return(self):
        link=ProbeFixture(arrive=False);report={'motion_commands_attempted':0}
        with self.assertRaisesRegex(RuntimeError,'displacement_not_confirmed'):run(link,report)
        self.assertEqual(sum(f[1]==wire.Command.POSITION_EMM for f in link.frames),1)
        self.assertEqual(sum(f[1]==wire.Command.ENABLE and f[3]==0 for f in link.frames),1)
        self.assertFalse(report.get('completed',False));self.assertFalse(report['fault_state']['enabled'])

    def test_unknown_options_or_configuration_fail_before_any_write(self):
        changed=bytearray(CONFIG);changed[9]=32
        for link in (ProbeFixture(options=0x446),ProbeFixture(options=0x404),ProbeFixture(config=bytes(changed))):
            with self.subTest(config=link.config.hex(),options=link.options):
                with self.assertRaisesRegex(RuntimeError,'explicitly_observed_configuration'):
                    run(link,{'motion_commands_attempted':0})
                self.assertTrue(all(f[1]<0xd0 for f in link.frames))

    def test_target_position_mismatch_rejects_and_disables_once(self):
        link=ProbeFixture(mismatch=True);report={'motion_commands_attempted':0}
        with self.assertRaisesRegex(RuntimeError,'displacement_not_confirmed'):run(link,report)
        self.assertEqual(report['motion_commands_attempted'],1)
        self.assertEqual(sum(f[1]==wire.Command.ENABLE and f[3]==0 for f in link.frames),1)
        self.assertTrue(report['fault_disable_ack']);self.assertFalse(link.enabled)

    def test_unknown_initial_retained_target_does_not_select_or_enable(self):
        link=ProbeFixture();link.target=1000
        with self.assertRaisesRegex(RuntimeError,'stale_target'):run(link,{'motion_commands_attempted':0})
        self.assertTrue(all(f[1]<0xd0 for f in link.frames))

    def test_disabled_hand_turned_position_rejects_all_writes_including_stop_and_enable(self):
        link=ProbeFixture(enabled=False);link.position=-9765;link.target=-6
        report={'motion_commands_attempted':0}
        with self.assertRaisesRegex(RuntimeError,'stale_target'):run(link,report)
        self.assertFalse(link.enabled)
        self.assertEqual(report['motion_commands_attempted'],0)
        self.assertTrue(all(f[1] in wire.READ_COMMANDS for f in link.frames))
        self.assertNotIn('fault_disable_ack',report)

    def test_lost_motion_ack_and_lost_disable_ack_are_not_retried(self):
        link=ProbeFixture(lost_motion_ack=True,disable_failure=True);report={'motion_commands_attempted':0}
        with self.assertRaises(TimeoutError):run(link,report)
        self.assertEqual(sum(f[1]==wire.Command.POSITION_EMM for f in link.frames),1)
        self.assertEqual(sum(f[1]==wire.Command.ENABLE and f[3]==0 for f in link.frames),1)
        self.assertIn('cleanup_error',report)

    def test_already_enabled_motor_is_not_reenabled(self):
        link=ProbeFixture(enabled=True);report={'motion_commands_attempted':0};run(link,report)
        self.assertTrue(report['completed'])
        self.assertFalse(any(f[1]==wire.Command.ENABLE for f in link.frames))

    def test_zero_integer_rpm_with_slow_encoder_creep_cannot_trigger_early_return(self):
        class Creeping(ProbeFixture):
            def __init__(self):super().__init__();self.step=0;self.positions=[];self.first_step_position_reads=0;self.reads_at_return=None
            def exchange(self,request):
                command=request[1]
                if command==wire.Command.POSITION_EMM:
                    before=self.position;self.step+=1
                    if self.step==2:self.reads_at_return=self.first_step_position_reads
                    response=super().exchange(request)
                    if self.step==1:
                        self.position=before
                        # Every sample satisfies the former +/-0.055deg target
                        # tolerance and speed is integer zero, yet shaft moves.
                        self.positions=[before+n for n in (11,14,17,20,20,20)]
                    return response
                if command==wire.Command.READ_POSITION and self.step==1:
                    self.first_step_position_reads+=1
                    if self.positions:self.position=self.positions.pop(0)
                return super().exchange(request)
        link=Creeping();report={'motion_commands_attempted':0};run(link,report)
        self.assertTrue(report['completed'])
        samples=[s for s in report['samples'] if s['label']=='step_1']
        self.assertGreaterEqual(len(samples),6)
        self.assertGreaterEqual(link.reads_at_return,6)
        self.assertEqual([s['position_counts'] for s in samples[:3]],[6,9,12])
        self.assertTrue(all(s['speed']==0 for s in samples))
        self.assertLessEqual(max(s['position'] for s in samples[-3:])-min(s['position'] for s in samples[-3:]),.015)

    def test_small_but_unsettled_following_error_needs_tighter_error_threshold(self):
        class FollowingError(ProbeFixture):
            def __init__(self):super().__init__();self.error_reads=0;self.step=0
            def exchange(self,request):
                response=super().exchange(request)
                if request[1]==wire.Command.POSITION_EMM:
                    self.step+=1;self.error_reads=3 if self.step==1 else 0
                if request[1]==wire.Command.READ_POSITION_ERROR and self.error_reads:
                    self.error_reads-=1
                    # 6 encoder counts=0.03296deg passed old .055 but exceeds .025.
                    return wire.parse_response(bytes.fromhex('01 37 00 00 00 00 06 6b'))
                return response
        link=FollowingError();report={'motion_commands_attempted':0};run(link,report)
        self.assertTrue(report['completed'])
        samples=[s for s in report['samples'] if s['label']=='step_1']
        self.assertGreaterEqual(len(samples),6)
        self.assertTrue(all(abs(s['error'])<=.025 for s in samples[-3:]))


if __name__=='__main__':unittest.main()
