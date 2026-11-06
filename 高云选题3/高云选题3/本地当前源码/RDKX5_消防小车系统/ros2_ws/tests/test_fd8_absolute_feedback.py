"""A saved mechanical origin cannot conceal a changed running coordinate."""
import unittest
from unittest import mock

from test_zdt_fd8_control import Clock, fixture, motion
from rescue_vision import zdt_xs_protocol as zdt


class AbsoluteFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.timer = mock.patch('rescue_vision.mixed_actuator_io.time.monotonic', self.clock)
        self.timer.start()
        self.addCleanup(self.timer.stop)

    def fixture(self, *, encoder_direction=1):
        io, pan, tilt, events = fixture()
        io.servo['pan']['zdt_fd8_snapshot']['startup_encoder_offset_counts'] = (3-1000) % 65536
        original = pan.exchange

        def exchange(packet):
            _, cmd = zdt.validate_request(packet)
            if cmd == zdt.Command.READ_ENCODER:
                events.append(('pan', cmd, packet))
                value = (1000+encoder_direction*(pan.counts-3)) % 65536
                return zdt.parse_response(bytes((1, cmd))+value.to_bytes(2, 'big')+b'\x6b', 1, cmd)
            return original(packet)

        pan.exchange = exchange
        return io, pan, tilt, events

    def stable(self, io):
        for _ in range(3):
            io._cycle()
            self.clock.tick(.05)

    def test_legacy_without_saved_encoder_offset_does_not_add_queries(self):
        io, _, _, events = fixture()
        self.stable(io)
        self.assertFalse(any(row[0] == 'pan' and row[1] == zdt.Command.READ_ENCODER for row in events))

    def test_matching_encoder_keeps_exact_step_protocol_and_three_arrival_samples(self):
        io, pan, _, events = self.fixture()
        self.stable(io)
        io.set_gimbal(1.8, 0., speed_deg_s=6, lease_s=.5)
        io._cycle()
        pan.arrive()
        self.stable(io)
        self.assertEqual(len(motion(events)), 1)
        self.assertEqual(sum(row[0] == 'pan' and row[1] == zdt.Command.READ_ENCODER for row in events), 7)
        self.assertIn('absolute_counts', io._feedback[0]['fd8_sample'])

    def test_reversed_encoder_sign_faults_before_a_second_relative_step(self):
        io, pan, tilt, events = self.fixture(encoder_direction=-1)
        self.stable(io)
        io.set_gimbal(1.8, 0., speed_deg_s=6, lease_s=.5)
        io._cycle()
        pan.arrive()
        io._run()
        self.assertIn('absolute encoder/runtime position mismatch', io._fault)
        self.assertEqual(len(motion(events)), 1)
        self.assertFalse(pan.enabled or tilt.enabled)

    def read_interval(self, before, after, absolute, *, offset=0):
        io, pan, _, _ = fixture()
        axis = io.servo['pan']
        axis['zdt_fd8_snapshot']['startup_encoder_offset_counts'] = offset
        positions = iter((before, after))
        original = pan.exchange

        def exchange(packet):
            _, cmd = zdt.validate_request(packet)
            if cmd == zdt.Command.READ_POSITION:
                value = next(positions)
                payload = bytes((int(value < 0),))+abs(value).to_bytes(4, 'big')
                return zdt.parse_response(bytes((1, cmd))+payload+b'\x6b', 1, cmd)
            if cmd == zdt.Command.READ_ENCODER:
                return zdt.parse_response(bytes((1, cmd))+absolute.to_bytes(2, 'big')+b'\x6b', 1, cmd)
            return original(packet)

        pan.exchange = exchange
        return io._read_fd8_dynamic_axis(axis)

    def test_serial_motion_envelope_accepts_intermediate_encoder_read(self):
        for before, after in ((0, 100), (100, 0)):
            sample = self.read_interval(before, after, 50)
            self.assertEqual(sample['fd8_sample']['absolute_counts'], 50)

    def test_single_turn_wrap_maps_to_signed_running_position(self):
        self.assertEqual(self.read_interval(-5, 5, 65535)['counts'], 5)
        self.assertEqual(self.read_interval(-105, -95, 65436)['counts'], -95)

    def test_coordinate_shift_outside_existing_feedback_tolerance_is_not_rebased(self):
        self.read_interval(0, 0, 36)  # 36 counts is just below 0.2 degrees.
        with self.assertRaisesRegex(RuntimeError, 'absolute encoder/runtime position mismatch'):
            self.read_interval(0, 0, 37)


if __name__ == '__main__':
    unittest.main(verbosity=2)
