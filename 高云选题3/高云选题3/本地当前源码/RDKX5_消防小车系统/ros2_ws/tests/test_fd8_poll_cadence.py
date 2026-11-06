"""Exercise the real worker scheduler with measured-arrival protocol fixtures."""
import unittest
from unittest import mock

from test_zdt_fd8_control import Clock, fixture, motion
from test_mixed_actuator_io import mixed_fixture
from rescue_vision import zdt_vendor_bench as vendor, zdt_fd8_control as target_control


class FD8PollCadenceTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.timer = mock.patch('rescue_vision.mixed_actuator_io.time.monotonic', self.clock)
        self.timer.start()
        self.addCleanup(self.timer.stop)

    def test_fast_cadence_only_while_an_fd8_goal_is_outstanding(self):
        io, _, _, _ = fixture()
        io.servo['poll_hz'] = 8
        self.assertEqual(io._poll_interval_s(), .125)
        io.set_gimbal(5.4, 0, speed_deg_s=6, lease_s=.5)
        self.assertEqual(io._poll_interval_s(), .05)
        self.assertEqual(io.feedback()['feedback_poll_target_hz'], 20)
        io._accepted_generation = io._goal['generation']
        self.assertEqual(io._poll_interval_s(), .125)
        io._accepted_generation = None
        self.clock.tick(.51)
        self.assertEqual(io._poll_interval_s(), .125)
        io._goal = None
        self.assertEqual(io._poll_interval_s(), .125)
        general, _, _, _ = mixed_fixture()
        general.servo['poll_hz'] = 8
        general.set_gimbal(91, 91, speed_deg_s=6, lease_s=.5)
        self.assertEqual(general._poll_interval_s(), .125)

    def run_move(self, *, legacy=False):
        self.clock.now = 100.
        io, pan, _, events = fixture()
        io.servo['poll_hz'] = 8
        for _ in range(3):
            io._cycle()
            self.clock.tick(.125)
        start = self.clock()
        writes = []
        pan.on_position = lambda: writes.append(self.clock())
        io.set_gimbal(5.4, 0, speed_deg_s=6, lease_s=.5)
        clock = self.clock

        class Scheduler:
            stopped = False
            completed = None
            waits = []

            def is_set(self):
                return self.stopped

            def wait(self, seconds):
                self.waits.append(seconds)
                clock.tick(seconds)
                # Simulate the full packet's travel at the unchanged 1 RPM;
                # arrival still comes from encoder feedback, never the ACK.
                pending = io._fd8.pending
                travel = pending['pulses']*target_control.PULSE_DEG/6 if pending else 0
                if pending and clock()-writes[-1] >= travel-1e-9:
                    pan.arrive()
                if io.feedback()['settled_measured']:
                    self.completed = clock()
                    self.stopped = True
                    return
                if len(self.waits) > 200:
                    raise AssertionError('scheduled bounded move did not converge')
                io.set_gimbal(5.4, 0, speed_deg_s=6, lease_s=.5)

        scheduler = Scheduler()
        io._quit = scheduler
        if legacy:
            io._poll_interval_s = lambda: .125
        io._run()
        self.assertEqual(io._fault, '')
        self.assertIsNotNone(scheduler.completed)
        self.assertEqual(len(motion(events)), 1)
        self.assertEqual(motion(events)[0][2], target_control.motion_frame(1,48))
        self.assertAlmostEqual((pan.counts-3)*vendor.COUNT_DEG, 5.4, delta=.05)
        self.assertGreaterEqual(scheduler.completed-writes[0], .9+.1-1e-9)
        return scheduler.completed-start, scheduler.waits

    def test_worker_finishes_same_motion_sooner_without_changing_packets(self):
        previous, _ = self.run_move(legacy=True)
        optimized, waits = self.run_move()
        self.assertLess(optimized, previous)
        self.assertIn(.05, waits)
        self.assertIn(.125, waits)  # Return to normal polling after arrival.

    def test_worker_does_not_sleep_again_after_slow_serial_cycle(self):
        io, _, _, _ = fixture()
        io.set_gimbal(5.4, 0, speed_deg_s=6, lease_s=.5)
        waits = []
        io._cycle = lambda: self.clock.tick(.08)
        io._quit = mock.Mock()
        io._quit.is_set.side_effect = [False, True]
        io._quit.wait.side_effect = waits.append
        io._run()
        self.assertEqual(waits, [0.])


if __name__ == '__main__':
    unittest.main(verbosity=2)
