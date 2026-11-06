"""Protocol-real fixtures for complete-target ZDT control; no UART/GPIO opens."""
import copy
import pathlib
import sys
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision import zdt_xs_protocol as zdt, zdt_fd8_control as fd8, zdt_vendor_bench as vendor
from rescue_vision.mixed_actuator_io import MixedActuatorIO, validate
from rescue_vision.pd42_protocol import Command as PD
from test_mixed_actuator_io import mixed_fixture, ZDTFixture
from test_pd42_actuator_io import MotorFixture
from test_zdt_xs_protocol import LIVE_VERSION


class Clock:
    now = 100.
    def __call__(self): return self.now
    def tick(self, dt=.04): self.now += dt


class FD8Motor(ZDTFixture):
    def __init__(self, events):
        super().__init__(events)
        self.options = bytes.fromhex('01 1a 04 26 6b')
        self.counts = 3
        self.target_counts = self.realtime_target_counts = 9
        self.actual_pulse_position = 9.
        self.corrupt_config = False
        self.error_override = None

    def exchange(self, packet):
        _, cmd = zdt.validate_request(packet)
        if cmd == zdt.Command.READ_CONFIG and self.corrupt_config:
            raw = bytearray(bytes((1, 0x42))+vendor.CONFIG+bytes((0x6b,)))
            raw[12] ^= 1  # Valid packet but different configuration byte.
            return zdt.parse_response(bytes(raw))
        if cmd == zdt.Command.READ_POSITION_ERROR:
            self.events.append(('pan', cmd, packet))
            value = self.target_counts-self.counts if self.error_override is None else self.error_override
            return zdt.parse_response(bytes((1, cmd, int(value < 0)))+abs(value).to_bytes(4,'big')+b'\x6b')
        result = super().exchange(packet)
        if cmd == zdt.Command.POSITION_EMM:
            direction = -1 if packet[2] else 1
            pulses = int.from_bytes(packet[6:10], 'big')
            if packet != fd8.motion_frame(direction, pulses):
                raise AssertionError('Only mode 0, encoded speed 1 / acceleration 1 permitted')
            self.actual_pulse_position += direction*pulses*fd8.STEP_COUNTS
            self.target_counts = self.realtime_target_counts = round(self.actual_pulse_position)
            self.rpm = direction
            self.arrived = False
        return result

    def arrive(self):
        self.counts = self.target_counts
        self.rpm = 0
        self.arrived = True


class TiltMotor(MotorFixture):
    def exchange(self, packet):
        result = super().exchange(packet)
        if packet[2] == PD.ABSOLUTE_POSITION:
            self.counts = self.target_counts = int.from_bytes(packet[7:11], 'big', signed=True)
        return result


def config():
    io, *_ = mixed_fixture()
    cfg = copy.deepcopy(io.config)
    cfg['servo']['position_tolerance_deg'] = .5
    pan, tilt = cfg['servo']['pan'], cfg['servo']['tilt']
    pan.update(min_angle_deg=-90, max_angle_deg=90, reference_angle_deg=0, reference_counts=3,
               gear_ratio=1, direction=1, zdt_verified_snapshot=None)
    tilt.update(min_angle_deg=0, max_angle_deg=60, reference_angle_deg=0, reference_counts=20792,
                gear_ratio=1, direction=1)
    pan['zdt_fd8_snapshot'] = dict(profile=fd8.PROFILE,
        version=zdt.parse_response(LIVE_VERSION).value, config_data_hex=vendor.CONFIG.hex(' '),
        options_raw_hex='04 26', reference_position_counts=3,
        retained_target_counts=9, realtime_target_counts=9)
    return cfg


def fixture():
    io = MixedActuatorIO(config())  # Dry constructor avoids owner/serial threads.
    io.dry_run = False
    events = []
    pan, tilt = FD8Motor(events), TiltMotor('tilt', events)
    tilt.counts = tilt.target_counts = 20792
    io._ports = {'/dev/ttyS3':pan, '/dev/ttyS1':tilt}
    return io, pan, tilt, events


def motion(events):
    return [event for event in events if event[0] == 'pan' and event[1] == zdt.Command.POSITION_EMM]


class FD8Tests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.timer = mock.patch('rescue_vision.mixed_actuator_io.time.monotonic', self.clock)
        self.timer.start()
        self.addCleanup(self.timer.stop)

    def stable(self, io, n=3):
        for _ in range(n):
            io._cycle()
            self.clock.tick()

    def test_startup_only_reads_and_never_rebases_enabled_motors(self):
        io, pan, tilt, events = fixture()
        self.stable(io)
        self.assertEqual(pan.counts, 3)
        self.assertEqual(pan.target_counts, 9)
        self.assertEqual(tilt.counts, 20792)
        self.assertFalse(any(e[1] in (zdt.Command.ENABLE, zdt.Command.POSITION_EMM,
                                     PD.ENABLE, PD.ABSOLUTE_POSITION) for e in events))
        self.assertEqual(io.feedback()['pan_stable_samples'], 3)
        self.assertFalse(io.feedback()['pan_arbitrary_position_protocol_verified'])

    def test_exact_profile_required_and_unknown_bits_are_not_globally_masked(self):
        for field, value in (('config_data_hex',vendor.CONFIG[:-1].hex()),
                             ('options_raw_hex','04 06'),('reference_position_counts',4)):
            cfg = config()
            cfg['servo']['pan']['zdt_fd8_snapshot'][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError): validate(cfg)
        cfg = config()
        cfg['servo']['pan']['max_angle_deg'] = 180
        with self.assertRaisesRegex(ValueError,'180 degrees total'): validate(cfg)

    def test_live_configuration_change_blocks_writes(self):
        io, pan, _, events = fixture()
        pan.corrupt_config = True
        io.set_gimbal(1,0,speed_deg_s=6,lease_s=.5)
        with self.assertRaisesRegex(RuntimeError,'config/options changed'): io._cycle()
        self.assertFalse(motion(events))

    def test_repeated_same_goal_renew_does_not_stack_or_clear_pending(self):
        io, pan, _, events = fixture()
        self.stable(io)
        io.set_gimbal(5.4,0,speed_deg_s=6,lease_s=.5)
        io._cycle()
        pending = io._fd8.pending
        for _ in range(5):
            self.clock.tick()
            io.set_gimbal(5.4,0,speed_deg_s=6,lease_s=.5)
            io._cycle()
        self.assertIs(io._fd8.pending,pending)
        self.assertEqual(len(motion(events)),1)
        self.assertFalse(io.feedback()['settled_measured'])

    def test_complete_target_uses_one_packet_and_three_actual_arrival_samples(self):
        io, pan, _, events = fixture()
        self.stable(io)
        io.set_gimbal(5.4,2,speed_deg_s=6,lease_s=.5)
        io._cycle()
        self.assertEqual(len(motion(events)),1)
        self.assertEqual(motion(events)[0][2], fd8.motion_frame(1, 48))
        pan.arrive()
        for sample in range(3):
            self.clock.tick()
            io.set_gimbal(5.4,2,speed_deg_s=6,lease_s=.5)
            io._cycle()
            self.assertEqual(len(motion(events)),1)
            if sample < 2: self.assertIsNotNone(io._fd8.pending)
        self.clock.tick()
        io._cycle()
        self.assertEqual(len(motion(events)),1)
        self.assertTrue(io.feedback()['settled_measured'])
        self.assertEqual(io.feedback()['requested_target_deg'],[5.4,2.])
        self.assertNotEqual(io.feedback()['target_deg'],[5.4,2.])
        self.assertAlmostEqual(io.feedback()['actual_angles_deg'][0],5.4,delta=.05)
        self.assertEqual(sum(e[1] == PD.ABSOLUTE_POSITION for e in events),1)

    def test_retarget_waits_for_outstanding_step_before_reversing(self):
        io, pan, _, events = fixture()
        self.stable(io)
        io.set_gimbal(5.4,0,speed_deg_s=6,lease_s=.5)
        io._cycle()
        io.set_gimbal(-1.8,0,speed_deg_s=6,lease_s=.5)
        io._cycle()
        self.assertEqual(len(motion(events)),1)
        pan.arrive()
        self.stable(io)
        self.assertEqual(len(motion(events)),2)
        self.assertEqual(motion(events)[-1][2],fd8.motion_frame(-1,64))

    def test_lease_loss_disables_both_without_readback_or_replay(self):
        io, pan, tilt, events = fixture()
        self.stable(io)
        io.set_gimbal(2,0,speed_deg_s=6,lease_s=.5)
        io._cycle()
        self.clock.tick(.51)
        with self.assertRaisesRegex(RuntimeError,'lost lease'): io._cycle()
        self.assertFalse(pan.enabled or tilt.enabled)
        self.assertEqual(len(motion(events)),1)

    def test_four_second_no_progress_budget_is_not_extended_by_renewals(self):
        io, _, _, events = fixture()
        self.stable(io)
        io.set_gimbal(2,0,speed_deg_s=6,lease_s=.5)
        io._cycle()
        for _ in range(9):
            self.clock.tick(.4)
            io.set_gimbal(2,0,speed_deg_s=6,lease_s=.5)
            io._cycle()
        self.clock.tick(.41)
        io.set_gimbal(2,0,speed_deg_s=6,lease_s=.5)
        with self.assertRaisesRegex(RuntimeError,'no progress'): io._cycle()
        self.assertEqual(len(motion(events)),1)

    def test_pulse_resolution_reaches_both_boundaries_without_crossing_limits(self):
        io, *_ = fixture()
        # Captured actual=3, retained=9 shifts the physical lattice by 0.033deg.
        io.set_gimbal(90,0)
        self.assertLessEqual(io._goal['target'][0],90)
        self.assertLess(90-io._goal['target'][0],fd8.PULSE_DEG)
        io.set_gimbal(-90,0)
        self.assertGreaterEqual(io._goal['target'][0],-90)
        with self.assertRaisesRegex(ValueError,'exceeds measured'): io.set_gimbal(180,0)

    def test_reachable_endpoints_use_capture_and_direction_without_clamping(self):
        cfg=config();axis=cfg['servo']['pan'];snap=axis['zdt_fd8_snapshot']
        before=copy.deepcopy((axis,snap))
        result=fd8.reachable_range(axis,snap)
        self.assertEqual(result['step_deg'],.1125)
        self.assertEqual(result['step_indices'],[-800,799])
        self.assertAlmostEqual(result['limits_deg'][0],-89.967041015625)
        self.assertAlmostEqual(result['limits_deg'][1],89.920458984375)
        self.assertEqual((axis,snap),before)
        reverse=dict(axis,direction=-1)
        reversed_result=fd8.reachable_range(reverse,snap)
        self.assertAlmostEqual(reversed_result['limits_deg'][0],-result['limits_deg'][1])
        self.assertAlmostEqual(reversed_result['limits_deg'][1],-result['limits_deg'][0])
        new_reference=dict(axis,reference_counts=109)
        new_capture=dict(snap,retained_target_counts=109)
        self.assertEqual(fd8.reachable_range(new_reference,new_capture)['limits_deg'],[-90.,90.])

    def test_range_with_no_lattice_point_is_explicitly_unreachable(self):
        cfg=config();axis=dict(cfg['servo']['pan'],min_angle_deg=-.01,max_angle_deg=.01)
        snap=axis['zdt_fd8_snapshot']
        self.assertIsNone(fd8.reachable_range(axis,snap)['limits_deg'])
        with self.assertRaisesRegex(ValueError,'no realizable lattice'):
            fd8.quantized_target(0,axis,snap,.5)

    def test_disabled_old_target_and_false_arrival_do_not_authorize_motion(self):
        io, pan, _, events = fixture()
        pan.enabled = False
        pan.counts = 100
        io._cycle()
        self.assertEqual(io._fd8.stable, 0)
        self.assertFalse(pan.enabled)
        self.assertFalse(motion(events))
        self.assertEqual(io.feedback()['pan_feedback_diagnostic']['last_observation']['outcome'],
                         'waiting_stationary_feedback')

    def test_stationary_target_change_still_faults_with_original_sample(self):
        for field in ('target_counts', 'realtime_target_counts'):
            io, pan, _, events = fixture()
            self.stable(io)
            setattr(pan, field, getattr(pan, field)+3)
            io.set_gimbal(2,0,speed_deg_s=6,lease_s=.5)
            with self.subTest(field=field), self.assertRaisesRegex(RuntimeError,'reference/retained target changed'):
                io._cycle()
            self.assertFalse(motion(events))
            diagnostic = io.feedback()['pan_feedback_diagnostic']['last_unstable_observation']
            self.assertEqual(diagnostic['expected_counts'], 9)
            self.assertEqual(diagnostic['outcome'], 'retained_target_changed')
            self.assertEqual(diagnostic['sample']['retained_target_counts' if field=='target_counts' else 'realtime_target_counts'], 12)

    def test_unchanged_targets_with_transient_motion_wait_without_writes(self):
        for disturbance in ('rpm', 'following_error', 'encoder_pair_jitter'):
            io, pan, _, events = fixture()
            self.stable(io)
            if disturbance == 'rpm': pan.rpm = 1
            elif disturbance == 'following_error': pan.counts = 50
            else:
                reads = []
                def jitter(cmd):
                    if cmd == zdt.Command.READ_POSITION:
                        reads.append(1); pan.counts = 3 if len(reads)%2 else 10
                pan.on_read = jitter
            for _ in range(3):
                io.set_gimbal(2,0,speed_deg_s=6,lease_s=.5)
                io._cycle(); self.clock.tick()
            with self.subTest(disturbance=disturbance):
                self.assertEqual(io._fd8.stable, 0)
                self.assertIsNone(io._fd8.pending)
                self.assertFalse(motion(events))
                self.assertFalse(any(e[1] == PD.ABSOLUTE_POSITION for e in events))
                self.assertEqual(io._fd8.expected, 9)
                self.assertEqual(io.feedback()['pan_feedback_diagnostic']['last_unstable_observation']['outcome'],
                                 'waiting_stationary_feedback')
            # Recovery must collect three new good samples, not reuse the old
            # stable count or reset the hardware reference to the jitter value.
            pan.on_read = None; pan.rpm = 0; pan.counts = 9
            for _ in range(2):
                io.set_gimbal(2,0,speed_deg_s=6,lease_s=.5)
                io._cycle(); self.clock.tick()
                self.assertFalse(motion(events))
            io.set_gimbal(2,0,speed_deg_s=6,lease_s=.5); io._cycle()
            self.assertEqual(len(motion(events)), 1)
            self.assertIsNotNone(io.feedback()['pan_feedback_diagnostic']['last_unstable_observation'])

    def test_real_overspeed_or_following_error_still_faults_before_new_step(self):
        for disturbance in ('speed', 'error'):
            io, pan, _, events = fixture()
            self.stable(io)
            if disturbance == 'speed': pan.rpm = 3
            else: pan.error_override = 250
            io.set_gimbal(2,0,speed_deg_s=6,lease_s=.5)
            with self.subTest(disturbance=disturbance), self.assertRaisesRegex(RuntimeError, 'overspeed or excessive following error'):
                io._cycle()
            self.assertFalse(motion(events))
            self.assertEqual(io.feedback()['pan_feedback_diagnostic']['last_unstable_observation']['outcome'],
                             'overspeed_or_following_error')

    def test_ack_without_position_and_targets_is_not_completion(self):
        io, pan, _, events = fixture()
        self.stable(io)
        io.set_gimbal(2,0,speed_deg_s=6,lease_s=.5)
        io._cycle()
        pan.rpm = 0
        pan.arrived = True
        self.stable(io)
        self.assertEqual(len(motion(events)),1)
        self.assertFalse(io.feedback()['settled_measured'])

    def test_arrived_bit_is_required_even_at_matching_position(self):
        io, pan, _, events = fixture()
        self.stable(io)
        io.set_gimbal(2,0,speed_deg_s=6,lease_s=.5)
        io._cycle()
        pan.arrive()
        pan.arrived = False
        self.stable(io)
        self.assertEqual(len(motion(events)),1)

    def test_stop_during_pan_write_cancels_followup_steps(self):
        io, pan, tilt, events = fixture()
        self.stable(io)
        pan.on_position = io.disable_gimbal
        io.set_gimbal(2,0,speed_deg_s=6,lease_s=.5)
        io._cycle()
        self.assertFalse(pan.enabled or tilt.enabled)
        self.assertEqual(len(motion(events)),1)

    def test_lost_ack_is_never_replayed_and_worker_disables(self):
        io, pan, tilt, events = fixture()
        self.stable(io)
        pan.fail_position = True
        io.set_gimbal(2,0,speed_deg_s=6,lease_s=.5)
        io._run()
        self.assertEqual(len(motion(events)),1)
        self.assertFalse(pan.enabled or tilt.enabled)
        self.assertIn('reply lost',io.feedback()['error'])

    def test_slow_feedback_prevents_motion(self):
        io, pan, _, events = fixture()
        def delay(cmd):
            if cmd == zdt.Command.READ_TARGET_POSITION: self.clock.tick(.41)
        pan.on_read = delay
        io.set_gimbal(2,0,speed_deg_s=6,lease_s=.5)
        with self.assertRaisesRegex(RuntimeError,'real-time budget'): io._cycle()
        self.assertFalse(motion(events))

    def test_metadata_latency_is_not_encoder_age_but_every_cycle_still_verifies_it(self):
        io, pan, tilt, events = fixture()
        metadata = {zdt.Command.READ_VERSION, zdt.Command.READ_CONFIG, zdt.Command.READ_OPTIONS}
        def delay(cmd):
            self.clock.tick(.15 if cmd in metadata else .03)
        pan.on_read = delay
        tilt.on_read = lambda cmd: self.clock.tick(.02)
        start = self.clock()
        io._cycle()
        # Six timed pan dynamic queries plus both PD42 queries. The separate
        # error query in this fixture has no delay. Do not stamp completion.
        self.assertAlmostEqual(io._received, start + .45)
        self.assertAlmostEqual(io.feedback()['feedback_age_s'], .22)
        self.assertTrue(io.feedback()['feedback_fresh'])
        io._cycle()
        for cmd in metadata:
            self.assertEqual(sum(e[0] == 'pan' and e[1] == cmd for e in events), 2)
        self.assertFalse(motion(events))

    def test_combined_pan_and_tilt_dynamic_age_still_blocks_motion(self):
        io, pan, tilt, events = fixture()
        self.stable(io)
        pan.on_read = lambda cmd: self.clock.tick(.12) if cmd == zdt.Command.READ_TARGET_POSITION else None
        tilt.on_read = lambda cmd: self.clock.tick(.30) if cmd == PD.READ_STATUS else None
        io.set_gimbal(2,0,speed_deg_s=6,lease_s=.5)
        with self.assertRaisesRegex(RuntimeError, 'Mixed FD8 feedback exceeded real-time budget'):
            io._cycle()
        self.assertFalse(motion(events))
        self.assertFalse(any(e[1] == PD.ABSOLUTE_POSITION for e in events))

    def test_metadata_wait_does_not_extend_command_lease(self):
        io, pan, _, events = fixture()
        self.stable(io)
        pan.on_read = lambda cmd: self.clock.tick(.51) if cmd == zdt.Command.READ_OPTIONS else None
        io.set_gimbal(2,0,speed_deg_s=6,lease_s=.5)
        io._cycle()
        self.assertTrue(io.feedback()['feedback_fresh'])
        self.assertFalse(motion(events))
        self.assertFalse(any(e[1] == PD.ABSOLUTE_POSITION for e in events))

    def test_compact_tilt_snapshot_keeps_position_speed_enable_and_run_state(self):
        io, _, tilt, events = fixture()
        tilt.counts += 142
        tilt.rpm, tilt.status, tilt.enabled = 2, 2, False
        value = io._read_axis('tilt', io.servo['tilt'])
        self.assertEqual(value['counts'], tilt.counts)
        self.assertAlmostEqual(value['angle'], 142*360/51200)
        self.assertEqual((value['rpm'], value['enabled'], value['status']), (2,False,2))
        self.assertEqual([e[1] for e in events], [PD.READ_SYSTEM, PD.READ_STATUS])

    def test_compact_tilt_snapshot_still_checks_independent_fault_state(self):
        for status in (3,4,5):
            io, _, tilt, events = fixture()
            tilt.status = status
            with self.subTest(status=status), self.assertRaisesRegex(RuntimeError, 'driver fault'):
                io._read_axis('tilt', io.servo['tilt'])
            self.assertFalse(any(e[1] == PD.ABSOLUTE_POSITION for e in events))

    def test_compact_tilt_snapshot_rejects_system_stall_even_if_status_is_idle(self):
        io, _, tilt, _ = fixture()
        original = tilt.exchange
        def stalled(packet):
            reply = original(packet)
            if packet[2] == PD.READ_SYSTEM:
                reply.value['stalled'] = True
            return reply
        tilt.exchange = stalled
        with self.assertRaisesRegex(RuntimeError, 'driver fault'):
            io._read_axis('tilt', io.servo['tilt'])

    def test_full_180_configured_span_uses_three_complete_target_packets_and_returns(self):
        io, pan, _, events = fixture()
        self.stable(io)
        # The exact endpoints depend on the captured encoder/retained offset.
        # These are all reachable within the explicit +/-90 mechanical limits.
        observed = []
        for target in (89., -89.9, 0.):
            for _ in range(1000):
                io.set_gimbal(target,0,speed_deg_s=6,lease_s=.5)
                self.clock.tick()
                if io._fd8.pending is not None: pan.arrive()
                io._cycle()
                angle = (pan.counts-3)*vendor.COUNT_DEG
                observed.append(angle)
                self.assertGreaterEqual(angle,-90)
                self.assertLessEqual(angle,90)
                if io.feedback()['settled_measured']: break
            else:
                self.fail('Complete target traversal failed to converge')
            self.assertAlmostEqual(angle,target,delta=.5)
        self.assertGreater(max(observed),89.)
        self.assertLess(min(observed),-89.8)
        self.assertEqual(len(motion(events)),3)
        self.assertGreater(max(int.from_bytes(e[2][6:10], 'big') for e in motion(events)),1500)
        self.assertTrue(all(e[2][3:6] == bytes.fromhex('00 01 01') and e[2][10:12] == b'\0\0'
                            for e in motion(events)))

    def test_changed_following_error_or_targets_prevent_next_step(self):
        for failure in ('error','target','realtime','overspeed'):
            io, pan, _, events = fixture()
            self.stable(io)
            io.set_gimbal(4,0,speed_deg_s=6,lease_s=.5)
            io._cycle()
            pan.arrive()
            if failure == 'error': pan.error_override = 30
            if failure == 'target': pan.target_counts += 3
            if failure == 'realtime': pan.realtime_target_counts += 3
            if failure == 'overspeed': pan.rpm = 3
            if failure in ('overspeed','target','realtime'):
                with self.assertRaises(RuntimeError): io._cycle()
            else:
                self.stable(io)
            self.assertEqual(len(motion(events)),1, failure)

    def test_disabled_aligned_motor_only_enables_after_explicit_goal_and_stability(self):
        io, pan, tilt, events = fixture()
        pan.enabled = tilt.enabled = False
        self.stable(io)
        self.assertFalse(pan.enabled or tilt.enabled)
        io.set_gimbal(.9,0,speed_deg_s=6,lease_s=.5)
        io._cycle()
        self.assertTrue(pan.enabled and tilt.enabled)
        self.assertEqual(len(motion(events)),0)
        self.stable(io)
        self.assertEqual(len(motion(events)),1)

    def test_lease_expiring_mid_read_disables_before_motion(self):
        io, pan, tilt, events = fixture()
        self.stable(io)
        io.set_gimbal(0,0,speed_deg_s=6,lease_s=.5)
        io._cycle()
        def delay(cmd):
            if cmd == zdt.Command.READ_TARGET_POSITION: self.clock.tick(.08)
        pan.on_read = delay
        io.set_gimbal(2,0,speed_deg_s=6,lease_s=.05)
        io._cycle()
        self.assertFalse(pan.enabled or tilt.enabled)
        self.assertFalse(motion(events))


class CompleteTargetTrajectoryTests(unittest.TestCase):
    def stepper(self, start_deg=0):
        axis = config()['servo']['pan']
        axis['reference_counts'] = 0
        snap = dict(axis['zdt_fd8_snapshot'], retained_target_counts=round(start_deg/fd8.COUNT_DEG))
        stepper = fd8.FD8Stepper(axis,snap)
        stepper.stable = 3
        return stepper

    def sample(self, stepper, actual_deg, *, realtime_deg=None):
        actual = round(actual_deg/fd8.COUNT_DEG)
        target = round(stepper.pending['expected'])
        realtime = target if realtime_deg is None else round(realtime_deg/fd8.COUNT_DEG)
        return dict(position_before_counts=actual,position_counts=actual,
            retained_target_counts=target,realtime_target_counts=realtime,
            position_error_deg=(target-actual)*fd8.COUNT_DEG,speed_rpm=1,
            enabled=True,arrived=False)

    def test_single_90_and_180_degree_targets_use_full_pulse_count(self):
        for start, target, pulses in ((0,90,800),(-90,90,1600),(90,-90,1600)):
            stepper = self.stepper(start)
            direction, expected = stepper.plan_step(target,.5)
            packet = stepper.mark_written(direction,expected,100)
            self.assertEqual(packet,fd8.motion_frame(direction,pulses))
            self.assertEqual(packet[3:6],bytes.fromhex('00 01 01'))
            self.assertEqual(packet[10:12],b'\0\0')
            self.assertEqual(stepper.steps,1)
            self.assertEqual(fd8.angle_for_counts(expected,stepper.axis),target)
            self.assertGreater(stepper.pending['deadline'],104)

    def test_large_remaining_error_and_moving_realtime_target_are_normal(self):
        stepper = self.stepper()
        stepper.mark_written(*stepper.plan_step(90,.5),100)
        deadline = stepper.pending['deadline']
        for now, actual in ((100.1,.6),(103,18),(106,36),(109,54),(112,72)):
            self.assertFalse(stepper.observe(self.sample(stepper,actual,realtime_deg=actual+1),now))
            self.assertEqual(stepper.pending['deadline'],deadline)
            self.assertIsNone(stepper.plan_step(-90,.5))
        arrival = self.sample(stepper,90)
        arrival.update(speed_rpm=0,arrived=True)
        self.assertFalse(stepper.observe(arrival,115))
        self.assertFalse(stepper.observe(arrival,115.05))
        self.assertTrue(stepper.observe(arrival,115.1))
        self.assertEqual(stepper.steps,1)

    def test_full_target_stall_faults_without_resend(self):
        stepper = self.stepper()
        stepper.mark_written(*stepper.plan_step(90,.5),100)
        stepper.observe(self.sample(stepper,18),103)
        with self.assertRaisesRegex(RuntimeError,'no progress'):
            stepper.observe(self.sample(stepper,18),107.01)
        self.assertEqual(stepper.steps,1)

    def test_continuous_slow_progress_cannot_extend_hard_deadline(self):
        stepper = self.stepper()
        stepper.mark_written(*stepper.plan_step(90,.5),100)
        deadline = stepper.pending['deadline']
        for seconds in range(3,34,3):
            stepper.observe(self.sample(stepper,seconds),100+seconds)
        with self.assertRaisesRegex(RuntimeError,'arrival timeout'):
            stepper.observe(self.sample(stepper,34),deadline+.01)
        self.assertEqual(stepper.steps,1)

    def test_wrong_scaling_or_direction_is_rejected_inside_mechanical_limits(self):
        for corruption in ('retained_target','actual_reverse','realtime_reverse'):
            stepper = self.stepper()
            stepper.mark_written(*stepper.plan_step(45,.5),100)
            sample = self.sample(stepper,1,realtime_deg=2)
            if corruption == 'retained_target': sample['retained_target_counts'] *= 2
            elif corruption == 'actual_reverse': sample['position_counts'] = -200
            else: sample['realtime_target_counts'] = -200
            with self.subTest(corruption=corruption),self.assertRaisesRegex(RuntimeError,'target|interval'):
                stepper.observe(sample,100.1)
            self.assertEqual(stepper.steps,1)

    def test_legacy_profile_migration_preserves_captured_reference(self):
        cfg = config()
        cfg['servo']['pan']['zdt_fd8_snapshot']['profile'] = fd8.LEGACY_PROFILE
        original = copy.deepcopy(cfg)
        checked = validate(cfg)
        migrated = checked['servo']['pan']['zdt_fd8_snapshot']
        self.assertEqual(migrated['profile'],fd8.PROFILE)
        expected = copy.deepcopy(original['servo']['pan']['zdt_fd8_snapshot'])
        expected['profile'] = fd8.PROFILE
        self.assertEqual(migrated,expected)
        self.assertEqual(cfg,original)


if __name__ == '__main__': unittest.main(verbosity=2)
