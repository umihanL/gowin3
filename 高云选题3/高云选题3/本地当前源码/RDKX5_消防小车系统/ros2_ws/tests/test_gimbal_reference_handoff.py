"""Software zero capture must not release a loaded axis or erase faults."""
import unittest
from unittest import mock

import test_pd42_actuator_io as pd42_tests
from rescue_vision.gimbal_reference_policy import recoverable_reference_fault
from rescue_vision.gimbal_limits import require_measured_limits


LIMIT_ERROR = ('tilt measured position outside calibrated limits/reference: actual_deg=-8.79609375, '
               'counts=26238, limits_deg=[0.0,60.0], feedback_limit_tolerance_deg=0.2')


class ReferenceHandoffTests(unittest.TestCase):
    def fixture(self):
        io, pan, tilt, events = pd42_tests.AdapterTests().fixture()
        io.config['pump']['enabled'] = False
        for motor in (pan, tilt):
            motor.close = mock.Mock()
        io._owner = mock.Mock()
        return io, pan, tilt, events

    def test_idle_holding_handoff_preserves_enable_and_sends_no_write(self):
        io, pan, tilt, events = self.fixture()
        io._cycle()
        io._holding = dict(target=[90., 90.], generation=0)
        io._active = True
        owner = io._owner
        events.clear()
        io.detach_for_reference()
        # Exercise the worker finally branch too: it must not drop torque.
        io._run()
        self.assertTrue(io.closed)
        self.assertTrue(pan.enabled and tilt.enabled)
        self.assertEqual(events, [])
        owner.close.assert_called_once()
        pan.close.assert_called_once(); tilt.close.assert_called_once()
        self.assertFalse(io._motion_gate(dict(generation=0), 0.))
        self.assertEqual(events, [])

    def test_active_target_and_moving_or_stale_feedback_refuse_handoff(self):
        for condition in ('goal', 'moving', 'stale'):
            io, pan, _, _ = self.fixture()
            io._cycle()
            if condition == 'goal': io.set_gimbal(96., 90., lease_s=.5)
            elif condition == 'moving': pan.rpm = 1; io._cycle()
            else: io._received = 0.
            with self.assertRaises(RuntimeError): io.detach_for_reference()
            self.assertFalse(io.closed)
            self.assertFalse(io._reference_handoff)
            pan.close.assert_not_called()

    def test_only_exact_coordinate_fault_enters_readonly_recovery(self):
        self.assertTrue(recoverable_reference_fault(LIMIT_ERROR))
        for name in ('pan', 'tilt', 'FD8'):
            with self.assertRaises(RuntimeError) as error:
                require_measured_limits(name, 26238, -8.79609375,
                    dict(min_angle_deg=0., max_angle_deg=60.), .2)
            self.assertTrue(recoverable_reference_fault(str(error.exception)))
        for error in ('tilt driver fault: 3', LIMIT_ERROR+'; Disable could not be confirmed',
                      'emergency_stop', '', None):
            self.assertFalse(recoverable_reference_fault(error))
        io, pan, tilt, events = self.fixture()
        io._fault = LIMIT_ERROR
        io.detach_for_reference()
        self.assertEqual(events, [])
        self.assertTrue(pan.enabled and tilt.enabled)
        io, pan, _, events = self.fixture()
        io._fault = 'tilt driver fault: 3'
        with self.assertRaisesRegex(RuntimeError, 'driver fault'): io.detach_for_reference()
        self.assertFalse(io.closed)
        pan.close.assert_not_called()
        self.assertEqual(events, [])

    def test_join_failure_and_descriptor_failure_keep_ownership(self):
        for resource in ('worker', 'port'):
            io, pan, _, _ = self.fixture()
            io._cycle()
            owner = io._owner
            if resource == 'worker':
                io._thread = mock.Mock()
                io._thread.is_alive.return_value = True
            else:
                pan.close.side_effect = OSError('descriptor busy')
            with self.assertRaises((RuntimeError, OSError)): io.detach_for_reference()
            owner.close.assert_not_called()
            self.assertFalse(io.closed)

    def test_fault_recovery_waits_for_inflight_fault_stop_before_preserving_torque(self):
        io, pan, tilt, events = self.fixture()
        io._fault = LIMIT_ERROR
        io._active = True
        io._quit.set()
        io._thread = mock.Mock()
        io._thread.is_alive.return_value = False
        # The real worker has published its fault but not executed finally yet.
        io._thread.join.side_effect = lambda **kw: io._run()
        io.detach_for_reference()
        self.assertFalse(pan.enabled or tilt.enabled)
        self.assertEqual([int(row[1]) for row in events], [0xFA, 0xFA])


if __name__ == '__main__': unittest.main()
