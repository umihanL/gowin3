"""The empirical FD8 option enables only bounded, measured bench steps."""
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
sys.path.insert(0, str(ROOT / 'tests'))
from rescue_vision.pd42_test import PD42Bench
from rescue_vision import zdt_xs_protocol as wire
from rescue_vision import zdt_vendor_bench as profile
from test_pd42_test import evidence
from test_check_zdt_vendor_motion import MotorFixture, Clock


class BenchMotor(MotorFixture):
    def __init__(self, clock, **kwargs):
        super().__init__(clock, **kwargs)
        self.closed = False
        self.config_mutated = False
        self.disable_during_enable_verification = False
        self.enabled_state_reads = None
    def exchange(self, frame):
        cmd = wire.Command(frame[1])
        if cmd == wire.Command.ENABLE and frame[3]:
            self.enabled_state_reads = 0
        if cmd == wire.Command.READ_STATE and self.enabled_state_reads is not None:
            self.enabled_state_reads += 1
            if self.disable_during_enable_verification and self.enabled_state_reads == 2:
                self.enabled = False
        if cmd == wire.Command.READ_VOLTAGE:
            self.frames.append(frame)
            return wire.parse_response(bytes.fromhex('01 24 2d14 6b'), 1, cmd)
        if cmd == wire.Command.READ_CONFIG and self.config_mutated:
            self.frames.append(frame)
            data = bytearray(profile.CONFIG)
            data[10] ^= 1  # Same decoded units, different unvalidated configuration.
            return wire.parse_response(bytes((1, cmd)) + data + b'\x6b', 1, cmd)
        return super().exchange(frame)
    def close(self): self.closed = True


class BoundedZDTTests(unittest.TestCase):
    def setup(self, **kwargs):
        self.clock = Clock()
        self.motor = BenchMotor(self.clock, **dict({'enabled': False}, **kwargs))
        self.core = PD42Bench(clock=lambda: self.clock.t,
                              zdt_factory=lambda *a, **k: self.motor)
        self.n = 0
    def request(self, action):
        self.n += 1
        self.core.handle(dict(action=action, motor_id='motor2', request_id=str(self.n),
                              issued_at=self.clock.t, ttl_s=3.), self.clock.t, evidence(self.clock.t))
    def tick(self): self.core.tick(self.clock.t, evidence(self.clock.t))
    def ready(self, **kwargs):
        self.setup(**kwargs);self.request('connect');self.request('enable')
        self.assertEqual(self.core.last_result, 'completed', self.core.error)
    def snapshot(self): return self.core.snapshot(self.clock.t)
    def complete(self):
        self.tick();self.assertEqual(self.core.last_result, 'accepted')
        self.tick();self.assertEqual(self.core.last_result, 'accepted')
        self.tick();self.assertEqual(self.core.last_result, 'completed', self.core.error)
    def test_connect_is_read_only_generic_layout_unverified_but_exact_bench_available(self):
        self.setup();self.request('connect');s = self.snapshot()
        self.assertTrue(s['connected']);self.assertTrue(s['can_enable'])
        self.assertFalse(s['motion_configuration_valid']);self.assertFalse(s['can_run'])
        self.assertTrue(s['position_test_ready']);self.assertTrue(s['limited_position_only'])
        self.assertEqual(s['motion_configuration_reason'], 'zdt_undocumented_options')
        self.assertIn('0x0426', s['motion_configuration_detail'])
        self.assertEqual((s['step_deg'], s['limit_deg'], s['test_speed_raw']), (.9, .9, 1))
        self.assertIsNone(s['test_speed_rpm']);self.assertEqual(self.motor.controls(), [])
    def test_exact_forward_and_return_packets_and_three_sample_measured_arrival(self):
        self.ready();origin = self.motor.target
        self.request('step_positive');self.assertEqual(self.motor.frames[-1], profile.frame(1))
        self.assertAlmostEqual(self.snapshot()['target_deg'], .9, delta=.01)
        self.assertEqual(self.core.last_result, 'accepted');self.complete()
        self.request('return_origin');self.assertEqual(self.motor.frames[-1], profile.frame(-1));self.complete()
        self.assertEqual(self.motor.target, origin)
        self.assertEqual(self.core.zdt_offset, 0)
        self.assertTrue(self.snapshot()['can_move']);self.assertFalse(self.snapshot()['can_run'])
    def test_reverse_first_is_bounded_and_return_uses_prior_target_not_measured_position(self):
        self.ready(target_offset=5);origin = self.motor.target
        self.request('step_negative');self.complete()
        self.assertLess(self.motor.target, origin)
        self.request('return_origin');self.complete();self.assertEqual(self.motor.target, origin)
    def test_repeated_step_cannot_expand_session_and_run_never_sends_velocity(self):
        self.ready();self.request('step_positive');self.complete()
        self.request('step_positive');self.assertEqual(self.core.reason, 'zdt_limited_position_boundary')
        for action in ('run_positive', 'run_negative'):
            self.request(action);self.assertEqual(self.core.reason, 'zdt_limited_position_only')
        self.assertEqual(self.motor.motion_count, 1)
        self.assertFalse(any(f[1] == wire.Command.VELOCITY for f in self.motor.frames))
    def test_stale_retained_target_cannot_enable_or_reset_coordinates(self):
        self.setup(target_offset=200);self.request('connect')
        self.assertTrue(self.snapshot()['connected']);self.assertFalse(self.snapshot()['can_enable'])
        self.assertEqual(self.snapshot()['position_test_reason'], 'zdt_retained_target_mismatch')
        self.request('enable');self.assertEqual(self.core.reason, 'zdt_retained_target_mismatch')
        self.assertEqual(self.motor.controls(), [])
    def test_target_changes_after_connect_block_enable_without_control(self):
        self.setup();self.request('connect');self.motor.target += 100
        self.request('enable');self.assertEqual(self.core.reason, 'zdt_retained_target_mismatch')
        self.assertEqual(self.motor.controls(), [])
    def test_disable_during_postenable_stability_check_never_reports_enabled(self):
        self.setup();self.request('connect')
        self.motor.disable_during_enable_verification = True
        self.request('enable')
        self.assertEqual(self.core.last_result, 'error')
        self.assertIn('motor_test_enabled_feedback_required', self.core.error)
        self.assertFalse(self.motor.enabled);self.assertIsNone(self.core.enable_deadline)
        self.assertFalse(self.snapshot()['can_move'])
        self.assertEqual(self.motor.motion_count, 0)
        self.assertEqual([f[3] for f in self.motor.controls()], [1, 0])
    def test_options0406_and_other_config_bytes_do_not_gain_empirical_permission(self):
        for mutate in ('options', 'config'):
            self.setup()
            if mutate == 'options':self.motor.options = 0x406
            else:self.motor.config_mutated = True
            self.request('connect')
            self.assertFalse(self.snapshot()['can_enable']);self.assertFalse(self.snapshot()['limited_position_only'])
            self.request('enable');self.assertEqual(self.motor.controls(), [])
    def test_raw_configuration_change_with_same_decoded_units_blocks_movement(self):
        self.ready();self.motor.config_mutated = True
        self.request('step_positive');self.assertEqual(self.core.reason, 'zdt_configuration_changed_reconnect')
        self.assertEqual(self.motor.motion_count, 0);self.assertFalse(self.snapshot()['can_move'])
    def test_arrived_and_zero_speed_with_no_real_movement_timeout_and_disable(self):
        self.ready(moving=False);self.request('step_positive')
        self.tick();self.tick();self.tick();self.assertEqual(self.core.last_result, 'accepted')
        self.clock.t += 6.;self.tick()
        self.assertEqual(self.core.last_result, 'error');self.assertFalse(self.motor.enabled)
        self.assertEqual(self.motor.motion_count, 1)
    def test_wrong_target_and_excursion_never_report_arrival(self):
        for fault in ('wrong_target', 'overshoot'):
            self.ready(fault=fault);self.request('step_positive')
            for _ in range(3): self.tick()
            if self.core.last_result != 'error':self.clock.t += 6.;self.tick()
            self.assertEqual(self.core.last_result, 'error', fault)
            self.assertFalse(self.motor.enabled);self.assertEqual(self.motor.motion_count, 1)
    def test_lost_ack_never_retries_motion_and_disables(self):
        self.ready(fault='uncertain_ack');self.request('step_positive')
        self.assertEqual(self.core.last_result, 'error');self.assertFalse(self.motor.enabled)
        self.assertEqual(self.motor.motion_count, 1)
    def test_late_arrival_cannot_complete_even_when_third_good_sample_is_ready(self):
        for timing in ('before_poll', 'during_poll'):
            self.ready();self.request('step_positive');self.tick();self.tick()
            self.clock.t = self.core.move_deadline + (.01 if timing == 'before_poll' else -.01)
            self.tick()
            self.assertEqual(self.core.last_result, 'error', timing)
            self.assertIn('motor_test_arrival_timeout', self.core.error)
            self.assertFalse(self.motor.enabled);self.assertEqual(self.motor.motion_count, 1)
    def test_stop_ends_move_without_returning_or_zeroing(self):
        self.ready();self.request('step_positive');self.request('stop')
        self.assertFalse(self.motor.enabled);self.assertIsNone(self.core.target_counts)
        self.assertEqual(self.motor.motion_count, 1)


if __name__ == '__main__': unittest.main()
