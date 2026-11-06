"""Real-reference startup and cancellation using protocol fixtures, no UART."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision import live_mixed_actuator_io as live
from rescue_vision import zdt_xs_protocol as zdt, zdt_vendor_bench as vendor
from rescue_vision.local_actuator_io import create_actuator_io
from rescue_vision.fpga_pump_io import FPGAPumpIO
from test_pd42_actuator_io import MotorFixture
from test_mixed_actuator_io import ZDTFixture


def config():
    cfg = json.loads((ROOT/'config/actuator_io.mixed.template.json').read_text(encoding='utf-8'))
    cfg['dry_run'] = False
    cfg['pump'] = dict(enabled=False)
    s = cfg['servo']
    s.update(live_readback_until_reference=True, calibration_confirmed=True,
             load_supported_verified=True, position_tolerance_deg=.5,
             timeout_s=.15, feedback_timeout_s=.6, enable_on_command=True)
    s['pan'].update(min_angle_deg=-90., max_angle_deg=90., reference_angle_deg=0., reference_counts=999)
    s['tilt'].update(min_angle_deg=0., max_angle_deg=60., reference_angle_deg=0., reference_counts=888)
    cfg['commissioning'] = dict(tilt_physical_direction_verified=True,
        tilt_direction_confirmation=dict(positive_motor_direction='up'))
    return cfg


class PanFixture(ZDTFixture):
    def exchange(self, packet):
        _, cmd = zdt.validate_request(packet)
        if cmd == zdt.Command.READ_ENCODER:
            self.events.append(('pan', cmd, packet))
            absolute = getattr(self, 'absolute_counts', self.counts % 65536)
            return zdt.parse_response(bytes((1, cmd))+absolute.to_bytes(2, 'big')+b'\x6b', 1, cmd)
        if cmd in (zdt.Command.READ_VERSION, zdt.Command.READ_CONFIG,
                   zdt.Command.READ_OPTIONS, zdt.Command.READ_POSITION_ERROR):
            self.events.append(('pan', cmd, packet))
            payload = {zdt.Command.READ_VERSION: bytes.fromhex('00 ce 23 14'),
                zdt.Command.READ_CONFIG: vendor.CONFIG,
                zdt.Command.READ_OPTIONS: bytes.fromhex('04 26'),
                zdt.Command.READ_POSITION_ERROR: b'\0'*5}[cmd]
            return zdt.parse_response(bytes((1, cmd))+payload+b'\x6b', 1, cmd)
        return super().exchange(packet)

    def close(self):
        self.events.append(('close_pan', None, None))


class TiltFixture(MotorFixture):
    def close(self):
        self.events.append(('close_tilt', None, None))


class DelegateFixture:
    def __init__(self, cfg):
        self.config = live.validate_mixed(cfg)
        self.commands = []
        self.closed = False

    def feedback(self):
        return dict(reference_confirmed_this_start=True, feedback_fresh=True,
                    feedback_age_s=.01, actual_angles_deg=[0., 0.], hardware_written=False,
                    motor_counts=[self.config['servo'][n]['reference_counts'] for n in ('pan', 'tilt')],
                    motor_rpm=[0, 0], error='')

    def reference_handoff_reason(self): return ''

    def detach_for_reference(self): self.closed = True

    def close(self):
        self.closed = True


def fixture():
    io = live.LiveMixedActuatorIO(config(), start_worker=False)
    events = []
    pan, tilt = PanFixture(events), TiltFixture('tilt', events)
    pan.counts = -7
    pan.target_counts = pan.realtime_target_counts = -8
    tilt.counts = tilt.target_counts = 27200
    io._owner = mock.Mock()
    io._ports = {'/dev/ttyS1':tilt, '/dev/ttyS3':pan}
    return io, pan, tilt, events


class LiveReferenceTests(unittest.TestCase):
    def capture(self, io, **extra):
        with mock.patch.object(live.Path, 'read_text', return_value='current-boot'), \
             mock.patch.object(live, 'MixedActuatorIO', DelegateFixture):
            return io.commit_reference(operator_zero_confirmed=True, supported_load=True,
                stationary_confirmed=True, tilt_positive_direction='up', **extra)

    def test_factory_explicit_live_pending_routes_without_changing_dry_templates(self):
        cfg = config()
        with mock.patch.object(live, 'LiveMixedActuatorIO') as factory:
            create_actuator_io(cfg)
            factory.assert_called_once()
        cfg['dry_run'] = True
        with mock.patch.object(live, 'LiveMixedActuatorIO') as factory:
            result = create_actuator_io(cfg)
            self.assertTrue(result.dry_run)
            factory.assert_not_called()

    def test_historical_snapshot_cannot_block_reads_or_claim_known_joint_angles(self):
        cfg = config()
        cfg['servo']['pan']['zdt_fd8_snapshot'] = {'invalid':'old snapshot'}
        result = live.pending_config(cfg)
        self.assertFalse(result['dry_run'])
        self.assertIsNone(result['servo']['pan']['reference_counts'])
        self.assertNotIn('zdt_fd8_snapshot', result['servo']['pan'])
        io, _, _, events = fixture()
        for _ in range(3):io._sample()
        state = io.feedback()
        self.assertTrue(state['raw_feedback_fresh'])
        self.assertTrue(state['reference_capture_ready'])
        self.assertEqual(state['motor_counts'], [-7,27200])
        self.assertEqual(state['motor_rpm'], [0,0])
        self.assertIsNone(state['actual_angles_deg'])
        self.assertFalse(state['feedback_fresh'])
        self.assertFalse(state['reference_confirmed_this_start'])
        self.assertFalse(state['hardware_written'])
        self.assertTrue(all(int(cmd)<0x80 for _,cmd,_ in events))

    def test_pending_stop_and_close_do_not_disable_installed_load(self):
        io, pan, tilt, events = fixture()
        io._sample()
        io.disable_gimbal()
        io.close()
        self.assertTrue(pan.enabled)
        self.assertTrue(tilt.enabled)
        self.assertTrue(all(cmd is None or int(cmd)<0x80 for _,cmd,_ in events))

    def test_motion_requires_reference_and_never_queues_pending_targets(self):
        io, _, _, events = fixture()
        with self.assertRaisesRegex(RuntimeError, 'zero is not confirmed'):
            io.set_gimbal(0,30)
        with self.assertRaises(RuntimeError):io.set_pump(True)
        self.assertFalse(events)
        self.assertFalse(io.commands)

    def test_explicit_capture_keeps_installation_and_uses_current_measured_counts(self):
        io, pan, tilt, events = fixture()
        result = self.capture(io)
        self.assertEqual(result['boot_id'], 'current-boot')
        self.assertEqual(result['config']['servo']['pan']['reference_counts'], -7)
        self.assertEqual(result['config']['servo']['tilt']['reference_counts'],27200)
        self.assertEqual(result['config']['servo']['pan']['min_angle_deg'],-90.)
        self.assertEqual(result['config']['servo']['tilt']['max_angle_deg'],60.)
        self.assertEqual(result['config']['servo']['tilt']['direction'],1)
        self.assertFalse(result['motion_commands_sent'])
        self.assertTrue(io.feedback()['reference_confirmed_this_start'])
        self.assertTrue(io.feedback()['reference_capture_ready'])
        self.assertTrue(all(cmd is None or int(cmd)<0x80 for _,cmd,_ in events))
        self.assertTrue(pan.enabled);self.assertTrue(tilt.enabled)
        first_delegate = io._delegate
        pan.counts = pan.target_counts = pan.realtime_target_counts = 100
        tilt.counts = tilt.target_counts = 27400
        def reopen():
            io._owner = mock.Mock()
            io._ports = {'/dev/ttyS1': tilt, '/dev/ttyS3': pan}
        with mock.patch.object(io, '_open', side_effect=reopen):
            second = self.capture(io)
        self.assertTrue(first_delegate.closed)
        self.assertNotEqual(second['reference_id'], result['reference_id'])
        self.assertEqual(second['config']['servo']['pan']['reference_counts'], 100)
        self.assertEqual(second['config']['servo']['tilt']['reference_counts'], 27400)
        self.assertTrue(pan.enabled); self.assertTrue(tilt.enabled)
        self.assertTrue(all(cmd is None or int(cmd)<0x80 for _,cmd,_ in events))

    def test_confirmed_reference_factory_keeps_repeat_capable_owner_on_restart(self):
        io, _, _, _ = fixture()
        result = self.capture(io)
        self.assertFalse(result['config']['servo']['live_readback_until_reference'])
        with mock.patch.object(live, 'MixedActuatorIO', DelegateFixture):
            restarted = create_actuator_io(result['config'])
        self.assertIsInstance(restarted, live.LiveMixedActuatorIO)
        self.assertTrue(restarted.feedback()['reference_capture_supported'])
        self.assertTrue(restarted.feedback()['reference_capture_ready'])
        restarted.close()

    def test_busy_delegate_rejects_recapture_without_losing_existing_reference(self):
        io, _, _, _ = fixture()
        self.capture(io)
        previous = deepcopy(io.config)
        io._delegate.reference_handoff_reason = lambda: 'another_gimbal_owner_active'
        io._delegate.detach_for_reference = mock.Mock(side_effect=RuntimeError('another_gimbal_owner_active'))
        self.assertFalse(io.feedback()['reference_capture_ready'])
        with self.assertRaisesRegex(RuntimeError, 'another_gimbal_owner_active'): self.capture(io)
        self.assertEqual(io.config, previous)
        self.assertIsNotNone(io._delegate)

    def test_reference_limit_fault_restores_raw_readback_but_never_confirms_zero_automatically(self):
        io, pan, tilt, events = fixture()
        first = self.capture(io)
        error = ('tilt measured position outside calibrated limits/reference: actual_deg=-8.79609375, '
                 'counts=26238, limits_deg=[0.0,60.0], feedback_limit_tolerance_deg=0.2')
        old = io._delegate
        old.feedback = lambda: dict(error=error, feedback_fresh=False)
        io._recover_readback(error)
        self.assertTrue(old.closed)
        self.assertIsNone(io._delegate)
        self.assertEqual(io.config['servo']['reference_id'], first['reference_id'])
        self.assertFalse(io.feedback()['reference_recovery_permitted'])
        pan.counts = pan.target_counts = pan.realtime_target_counts = 2
        tilt.counts = tilt.target_counts = 26238
        io._ports = {'/dev/ttyS1': tilt, '/dev/ttyS3': pan}
        io._owner = mock.Mock()
        for _ in range(3): io._sample()
        state = io.feedback()
        self.assertTrue(state['reference_recovery_permitted'])
        self.assertEqual(state['reference_recovery_error'], error)
        self.assertFalse(state['reference_confirmed_this_start'])
        self.assertIsNone(state['actual_angles_deg'])
        second = self.capture(io)
        self.assertEqual(second['recovered_error'], error)
        self.assertEqual(second['config']['servo']['tilt']['reference_counts'], 26238)
        self.assertTrue(all(cmd is None or int(cmd)<0x80 for _,cmd,_ in events))

    def test_driver_fault_returns_to_readonly_without_clear_enable_or_replay(self):
        io, _, _, _ = fixture()
        self.capture(io)
        delegate = io._delegate
        delegate.feedback = lambda: dict(error='tilt driver fault: 3', feedback_fresh=False)
        state = io.feedback()
        self.assertFalse(state['reference_capture_ready'])
        io._recovery_thread.join(1.)
        self.assertIsNone(io._delegate)
        self.assertTrue(delegate.closed)
        self.assertTrue(io._restore_attempted)
        self.assertEqual(io._recovery_error, 'tilt driver fault: 3')
        self.assertEqual(io._restore_state, 'operator_recovery_required')

    def restore_fixture(self, *, allowed=True):
        captured, _, _, _ = fixture()
        cfg = self.capture(captured)['config']
        cfg['servo'].update(reference_confirmed_this_start=False, live_readback_until_reference=True,
            absolute_reference_restore_allowed=allowed)
        io = live.LiveMixedActuatorIO(cfg, start_worker=False)
        events = []
        pan, tilt = PanFixture(events), TiltFixture('tilt', events)
        # Both runtime turn counters may reset; saved physical origins do not.
        pan.counts = pan.target_counts = pan.realtime_target_counts = 0
        pan.absolute_counts = (-7+5461) % 65536  # physical pan about +30 degrees
        tilt.counts = tilt.target_counts = 27200+5120  # physical tilt +36 degrees
        io._ports = {'/dev/ttyS1': tilt, '/dev/ttyS3': pan}
        io._owner = mock.Mock()
        for _ in range(3): io._sample()
        now = time.monotonic()
        for index, row in enumerate(io._rows): row['started'] = now-.1+index*.04
        return io, pan, tilt, events

    def attempt_restore(self, io):
        with mock.patch.object(live.Path, 'read_text', return_value='new-boot'), \
             mock.patch.object(live, 'MixedActuatorIO', DelegateFixture):
            io._attempt_absolute_restore()

    def test_saved_absolute_origin_restores_nonzero_pose_without_movement_or_changing_origin(self):
        io, pan, tilt, events = self.restore_fixture()
        original = deepcopy(io.servo['absolute_reference'])
        self.attempt_restore(io)
        result = io.take_restored_reference()
        self.assertEqual(result['reference_id'], original['reference_id'])
        self.assertEqual(result['boot_id'], 'new-boot')
        self.assertEqual(result['config']['servo']['absolute_reference'], original)
        self.assertEqual(result['config']['servo']['pan']['reference_counts'], -5461)
        self.assertEqual(result['config']['servo']['tilt']['reference_counts'], 27200)
        self.assertAlmostEqual(result['restored_angles_deg'][0], 30., places=2)
        self.assertAlmostEqual(result['restored_angles_deg'][1], 36., places=2)
        self.assertFalse(result['motion_commands_sent'])
        self.assertFalse(io.feedback()['reference_confirmed_this_start'])
        self.assertEqual(io.feedback()['absolute_reference_restore_state'], 'awaiting_persistence')
        with self.assertRaisesRegex(RuntimeError, 'persistence'): io.set_gimbal(0, 30)
        self.assertIsNone(io.take_restored_reference())
        io.finish_reference_restore(result['reference_id'], True)
        self.assertTrue(io.feedback()['reference_confirmed_this_start'])
        self.assertEqual(io.feedback()['absolute_reference_restore_state'], 'restored')
        self.assertTrue(pan.enabled and tilt.enabled)
        self.assertTrue(all(cmd is None or int(cmd)<0x80 for _,cmd,_ in events))

    def test_untrusted_anchor_or_changed_geometry_never_auto_restores(self):
        for change in ('permission', 'geometry'):
            io, _, _, events = self.restore_fixture(allowed=change != 'permission')
            if change == 'geometry': io.servo['pan']['direction'] *= -1
            self.attempt_restore(io)
            self.assertIsNone(io.take_restored_reference())
            self.assertIsNone(io._delegate)
            self.assertFalse(io.feedback()['reference_confirmed_this_start'])
            self.assertTrue(all(cmd is None or int(cmd)<0x80 for _,cmd,_ in events))
            if change == 'geometry':
                self.assertIn('installation_changed', io.feedback()['absolute_reference_restore_reason'])

    def test_failed_restore_persistence_does_not_open_motion_or_repeat_delivery(self):
        io, _, _, _ = self.restore_fixture()
        self.attempt_restore(io)
        result = io.take_restored_reference()
        io.finish_reference_restore(result['reference_id'], False, 'disk full')
        self.assertEqual(io.feedback()['absolute_reference_restore_state'], 'failed')
        self.assertFalse(io.feedback()['reference_confirmed_this_start'])
        self.assertIsNone(io.take_restored_reference())
        with self.assertRaisesRegex(RuntimeError, 'persistence'): io.set_gimbal(0, 30)

    def test_restore_publication_and_motion_gate_are_atomic(self):
        io, _, _, _ = fixture()
        self.capture(io)
        io._delegate.set_gimbal = mock.Mock()
        io._delegate.hold_gimbal = mock.Mock()
        io._delegate.set_pump = mock.Mock()
        for action in (lambda: io.set_gimbal(0, 30), io.hold_gimbal, lambda: io.set_pump(True)):
            entered, completed = threading.Event(), threading.Event()
            errors=[]
            def run():
                entered.set()
                try: action()
                except RuntimeError as exc: errors.append(str(exc))
                finally: completed.set()
            with io._mutex:
                io._restore_pending_accept = False
                thread=threading.Thread(target=run)
                thread.start(); self.assertTrue(entered.wait(.5))
                self.assertFalse(completed.wait(.02))
                io._restore_pending_accept = True
            thread.join(1.)
            self.assertFalse(thread.is_alive())
            self.assertTrue(errors)
        io._delegate.set_gimbal.assert_not_called()
        io._delegate.hold_gimbal.assert_not_called()
        io._delegate.set_pump.assert_not_called()

    def test_fpga_pump_wrapper_preserves_pump_config_in_automatic_restore(self):
        io, _, _, _ = self.restore_fixture()
        cfg=json.loads((ROOT/'config/actuator_io.mixed.template.json').read_text(encoding='utf-8'))['pump']
        wrapper=FPGAPumpIO(io, mock.Mock(), cfg)
        self.attempt_restore(io)
        result=wrapper.take_restored_reference()
        self.assertEqual(result['config']['pump'], wrapper.config['pump'])
        self.assertEqual(result['config']['pump']['pin'], 'B13')

    def test_missing_operator_confirmation_fails_before_uart_or_reference(self):
        io, _, _, events = fixture()
        for key in ('operator_zero_confirmed','supported_load','stationary_confirmed'):
            args=dict(operator_zero_confirmed=True,supported_load=True,stationary_confirmed=True)
            args[key]=False
            with self.assertRaisesRegex(ValueError,'Explicit'):io.commit_reference(**args)
        self.assertFalse(events)
        self.assertFalse(io.feedback()['reference_confirmed_this_start'])

    def test_unknown_direction_cannot_be_confirmed_by_api_argument(self):
        io, _, _, events = fixture()
        io.config['commissioning']['tilt_direction_confirmation'] = {}
        with self.assertRaisesRegex(ValueError,'consistent tilt direction'):self.capture(io)
        self.assertFalse(events)

    def test_legacy_observed_direction_is_accepted_without_another_physical_test(self):
        io,_,_,_=fixture()
        commissioning=io.config['commissioning']
        commissioning.pop('tilt_direction_confirmation')
        commissioning.update(tilt_positive_direction='up',
            tilt_direction_evidence='operator observed positive PD42 probe',
            tilt_direction_verified_at='2026-11-02T10:00:00Z')
        self.assertEqual(self.capture(io)['config']['servo']['tilt']['direction'],1)

    def test_unverified_installation_never_offers_zero_capture_even_with_live_counts(self):
        io,_,_,_=fixture()
        for _ in range(3):io._sample()
        io.config['commissioning']['tilt_physical_direction_verified']=False
        state=io.feedback()
        self.assertTrue(state['raw_feedback_fresh'])
        self.assertFalse(state['reference_capture_ready'])
        self.assertEqual(state['reference_capture_reason'],'tilt_physical_direction_unconfirmed_or_inconsistent')

    def test_unaligned_retained_target_rejects_without_enable_or_rebase(self):
        for axis in ('pan','tilt'):
            io,pan,tilt,events=fixture()
            if axis=='pan':pan.target_counts=1000
            else:tilt.target_counts=28200
            with self.assertRaisesRegex(ValueError,'target'):self.capture(io)
            self.assertFalse(io.feedback()['reference_confirmed_this_start'])
            self.assertTrue(all(int(cmd)<0x80 for _,cmd,_ in events))

    def test_stop_during_capture_prevents_delegate_and_keeps_only_raw_counts(self):
        io,pan,tilt,events=fixture()
        cancel=threading.Event()
        tilt.on_read=lambda _command:cancel.set()
        with self.assertRaisesRegex(RuntimeError,'cancelled'):self.capture(io,cancel_event=cancel)
        self.assertIsNone(io._delegate)
        self.assertIsNone(io.feedback()['actual_angles_deg'])
        self.assertTrue(all(int(cmd)<0x80 for _,cmd,_ in events))

    def test_stale_raw_feedback_cannot_offer_reference_capture(self):
        io,_,_,_=fixture()
        for _ in range(3):io._sample()
        io._rows[-1]['started']=time.monotonic()-10
        state=io.feedback()
        self.assertFalse(state['raw_feedback_fresh'])
        self.assertFalse(state['reference_capture_ready'])
        self.assertIsNone(state['motor_counts'])
        self.assertEqual(state['reference_capture_reason'],'raw_encoder_feedback_stale')

    def test_readback_errors_recover_without_latching_node_fault_or_writing_motion(self):
        io,_,_,_=fixture()
        io._read_error='UART unavailable'
        self.assertEqual(io.feedback()['error'],'')
        self.assertEqual(io.feedback()['readback_error'],'UART unavailable')
        for _ in range(3):io._sample()
        self.assertEqual(io.feedback()['readback_error'],'')
        self.assertTrue(io.feedback()['reference_capture_ready'])

    def test_worker_retries_only_readback_after_port_failure_without_exiting_node(self):
        io,_,_,events=fixture()
        attempts=[]
        def sample():
            attempts.append(1)
            if len(attempts)==1:
                raise OSError('USB or UART not ready')
            io._quit.set()
        io._sample=sample
        with mock.patch.object(io._quit,'wait',return_value=False):io._run()
        self.assertEqual(len(attempts),2)
        self.assertIn('UART not ready',io.feedback()['readback_error'])
        self.assertEqual(io.feedback()['error'],'')
        self.assertTrue(all(cmd is None for _,cmd,_ in events))

    def test_uart_close_failure_retains_global_ownership(self):
        io,pan,_,_=fixture()
        owner=io._owner
        pan.close=mock.Mock(side_effect=OSError('descriptor busy'))
        with self.assertRaisesRegex(RuntimeError,'ownership release failed'):io._release()
        owner.close.assert_not_called()
        self.assertIs(io._ports['/dev/ttyS3'],pan)

    def test_cancel_during_delegate_creation_closes_it_without_activation(self):
        io,_,_,events=fixture()
        cancel=threading.Event()
        created=[]
        def make(cfg):
            delegate=DelegateFixture(cfg)
            created.append(delegate)
            cancel.set()
            return delegate
        with mock.patch.object(live.Path,'read_text',return_value='current-boot'),mock.patch.object(live,'MixedActuatorIO',side_effect=make):
            with self.assertRaisesRegex(RuntimeError,'cancelled'):
                io.commit_reference(operator_zero_confirmed=True,supported_load=True,
                    stationary_confirmed=True,tilt_positive_direction='up',cancel_event=cancel)
        self.assertIsNone(io._delegate)
        self.assertTrue(created[0].closed)
        self.assertTrue(all(cmd is None or int(cmd)<0x80 for _,cmd,_ in events))

    def test_false_installation_record_does_not_become_confirmed_on_capture(self):
        io,_,_,events=fixture()
        io.servo['calibration_confirmed']=False
        with self.assertRaisesRegex(ValueError,'already be confirmed'):self.capture(io)
        self.assertIsNone(io._delegate)
        self.assertTrue(all(cmd is None or int(cmd)<0x80 for _,cmd,_ in events))

    def test_fpga_wrapper_updates_reference_without_replacing_pump_settings(self):
        io,_,_,_=fixture()
        pump_cfg=json.loads((ROOT/'config/actuator_io.mixed.template.json').read_text(encoding='utf-8'))['pump']
        wrapper=FPGAPumpIO(io,mock.Mock(),pump_cfg)
        with mock.patch.object(live.Path,'read_text',return_value='current-boot'),mock.patch.object(live,'MixedActuatorIO',DelegateFixture):
            result=wrapper.commit_reference(operator_zero_confirmed=True,supported_load=True,stationary_confirmed=True,tilt_positive_direction='up')
        self.assertEqual(result['config']['pump'],wrapper.config['pump'])
        self.assertEqual(result['config']['pump']['pin'],'B13')
        self.assertTrue(wrapper.config['servo']['reference_confirmed_this_start'])


if __name__=='__main__':unittest.main()
