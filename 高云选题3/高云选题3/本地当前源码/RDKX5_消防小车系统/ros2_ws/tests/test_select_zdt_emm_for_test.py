"""Offline validation of temporary D5 selection; no hardware port is opened."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SPEC=importlib.util.spec_from_file_location('select_zdt_emm',Path(__file__).resolve().parents[1]/'tools/select_zdt_emm_for_test.py')
check=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(check)
wire=check.wire
CONFIG=bytes.fromhex('01 42 21 15 19 02 02 02 00 10 01 00 04 B0 0B B8 13 88 05 07 01 00 01 01 00 08 08 98 07 D0 00 08 6B')


class SelectionFixture:
    def __init__(self,options=0x426,*,target=-6,config=CONFIG,lost_ack=False,post_config=None,disable_failure=False):
        self.options=options;self.target=target;self.config=config;self.post_config=post_config
        self.lost_ack=lost_ack;self.disable_failure=disable_failure;self.enabled=True;self.frames=[]
    def exchange(self,request):
        self.frames.append(request);cmd=request[1]
        wire.validate_request(request)
        if cmd==0xd5:
            assert request==bytes.fromhex('01 d5 69 00 01 6b')
            self.options&=~0x400
            if self.post_config is not None:self.config=self.post_config
            if self.lost_ack:raise TimeoutError('selection_may_have_executed')
            frame=bytes.fromhex('01 d5 02 6b')
        elif cmd==wire.Command.ENABLE:
            assert request==bytes.fromhex('01 f3 ab 00 00 6b')
            if self.disable_failure:raise TimeoutError('disable_unconfirmed')
            self.enabled=False;frame=bytes.fromhex('01 f3 02 6b')
        elif cmd==wire.Command.READ_VERSION:frame=bytes.fromhex('01 1f 00 ce 23 14 6b')
        elif cmd==wire.Command.READ_CONFIG:frame=self.config
        elif cmd==wire.Command.READ_OPTIONS:frame=bytes((1,cmd))+self.options.to_bytes(2,'big')+b'\x6b'
        elif cmd==wire.Command.READ_STATE:frame=bytes((1,cmd,3 if self.enabled else 2,0x6b))
        else:
            value=-5 if cmd==wire.Command.READ_POSITION else self.target if cmd in (
                wire.Command.READ_TARGET_POSITION,wire.Command.READ_REALTIME_TARGET_POSITION) else 0
            size=2 if cmd==wire.Command.READ_SPEED else 4
            frame=bytes((1,cmd,int(value<0)))+abs(value).to_bytes(size,'big')+b'\x6b'
        return wire.parse_response(frame,expected_address=1,expected_command=cmd)


class SelectionTests(unittest.TestCase):
    def test_dryrun_never_opens_uart(self):
        with tempfile.TemporaryDirectory() as d,patch.object(check,'ZDTSerialTransport') as port,contextlib.redirect_stdout(io.StringIO()):
            path=Path(d)/'dry.json'
            self.assertEqual(check.main(['--report',str(path)]),0)
            doc=json.loads(path.read_text(encoding='utf-8'))
            self.assertFalse(doc['completed']);self.assertEqual(doc['motion_commands_sent'],0)
        port.assert_not_called()

    def test_execute_requires_unloaded(self):
        with patch.object(check,'ZDTSerialTransport') as port,contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            check.main(['--execute','--report','unused-report.json'])
        port.assert_not_called()

    def test_success_0426_to_0026_has_one_nonpersistent_selection_and_unchanged_config(self):
        link=SelectionFixture();report={}
        with patch.object(check.time,'sleep'):check.select_ordinary_emm(link,report)
        self.assertTrue(report['completed']);self.assertTrue(link.enabled)
        self.assertEqual(report['options_before'],'04 26');self.assertEqual(report['options_after'],'00 26')
        self.assertEqual(report['normalized_options']['raw'],0x26)
        self.assertFalse(report['persistent_configuration_written'])
        writes=[f for f in link.frames if f[1]>=0xd0]
        self.assertEqual(writes,[bytes.fromhex('01 d5 69 00 01 6b')])
        self.assertEqual(len(report['samples']),4)
        self.assertEqual(len({s['config_hex'] for s in report['samples']}),1)
        for sample in report['samples']:
            self.assertAlmostEqual(sample['values']['position'],-5*360/65536)

    def test_unknown_or_conflicting_mode_options_rejected_before_any_change(self):
        for options in (0x404,0x427,0x800,0x046):
            with self.subTest(options=hex(options)):
                link=SelectionFixture(options=options)
                with patch.object(check.time,'sleep'),self.assertRaises((RuntimeError,wire.ProtocolError)):
                    check.select_ordinary_emm(link,{})
                self.assertTrue(all(f[1]<0xd0 for f in link.frames),'invalid options must fail before D5')

    def test_retained_target_rejects_before_any_change(self):
        link=SelectionFixture(target=6000)
        with self.assertRaisesRegex(RuntimeError,'retained_target'):check.select_ordinary_emm(link,{})
        self.assertTrue(all(f[1]<0xd0 for f in link.frames))

    def test_non_ack_config_rejects_before_any_change(self):
        config=bytearray(CONFIG);config[22]=0
        link=SelectionFixture(config=bytes(config))
        with self.assertRaises((RuntimeError,wire.ProtocolError)):check.select_ordinary_emm(link,{})
        self.assertTrue(all(f[1]<0xd0 for f in link.frames))

    def test_lost_selection_ack_never_retries_or_moves_and_disables_once(self):
        link=SelectionFixture(lost_ack=True);report={}
        with self.assertRaises(TimeoutError):check.select_ordinary_emm(link,report)
        self.assertEqual([f[1] for f in link.frames if f[1]>=0xd0],[0xd5,wire.Command.ENABLE])
        self.assertTrue(report['fault_disable_ack']);self.assertFalse(report['fault_state']['enabled'])
        self.assertNotIn('completed',report)

    def test_lost_cleanup_ack_is_recorded_without_retry(self):
        link=SelectionFixture(lost_ack=True,disable_failure=True);report={}
        with self.assertRaises(TimeoutError):check.select_ordinary_emm(link,report)
        self.assertEqual(sum(f[1]==wire.Command.ENABLE for f in link.frames),1)
        self.assertEqual(sum(f[1]==0xd5 for f in link.frames),1)
        self.assertIn('cleanup_error',report)

    def test_changed_configuration_after_selection_never_accepts_or_restores_automatically(self):
        config=bytearray(CONFIG);config[13]^=1
        link=SelectionFixture(post_config=bytes(config));report={}
        with patch.object(check.time,'sleep'),self.assertRaisesRegex(RuntimeError,'configuration_changed'):
            check.select_ordinary_emm(link,report)
        self.assertEqual([f[1] for f in link.frames if f[1]>=0xd0],[0xd5,wire.Command.ENABLE])
        self.assertFalse(link.enabled)


if __name__=='__main__':unittest.main()
