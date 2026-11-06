"""Pump requests are bounded even while the printed flame remains visible."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rescue_vision'))
from rescue_vision.local_actuator_guard import PumpGuard


class PumpGuardTests(unittest.TestCase):
    def tick(self, guard, now, **overrides):
        gates = dict(mission_ok=True, command_ok=True, aim_ok=True)
        gates.update(overrides)
        return guard.update(now, **gates)

    def begin_burst(self, guard, start=10):
        self.assertTrue(guard.arm(start, 20)['arm_accepted'])
        self.assertFalse(self.tick(guard, start)['pump_on'])
        self.assertTrue(self.tick(guard, start+.351)['pump_on'])
        return start+.351

    def test_never_armed_always_off(self):
        guard = PumpGuard()
        for now in (0, 1, 10, 100):
            state = self.tick(guard, now)
            self.assertFalse(state['pump_on'])
            self.assertFalse(state['armed'])
            self.assertEqual(state['burst_count'], 0)

    def test_stable_lock_burst_cooldown_and_new_dwell(self):
        guard = PumpGuard()
        start = self.begin_burst(guard)
        self.assertTrue(self.tick(guard, start+.39)['pump_on'])
        state = self.tick(guard, start+.401)
        self.assertEqual(state['state'], 'COOLDOWN')
        self.assertFalse(state['pump_on'])
        self.assertFalse(self.tick(guard, start+1.39)['pump_on'])
        self.assertFalse(self.tick(guard, start+1.402)['pump_on'])
        self.assertTrue(self.tick(guard, start+1.753)['pump_on'])
        self.assertEqual(guard.burst_count, 2)

    def test_each_gate_loss_cancels_and_recovery_never_auto_sprays(self):
        for gate in ('mission_ok', 'command_ok', 'aim_ok'):
            for during_burst in (False, True):
                with self.subTest(gate=gate, during_burst=during_burst):
                    guard = PumpGuard()
                    if during_burst:
                        self.begin_burst(guard)
                        now = 10.4
                    else:
                        guard.arm(10, 20)
                        self.tick(guard, 10)
                        now = 10.2
                    state = self.tick(guard, now, **{gate: False})
                    self.assertFalse(state['pump_on'])
                    self.assertFalse(state['armed'])
                    self.assertFalse(self.tick(guard, 11)['pump_on'])
                    self.assertFalse(self.tick(guard, 20)['pump_on'])

    def test_expiry_clips_burst_and_cannot_resume(self):
        guard = PumpGuard()
        guard.arm(0, .5)
        self.tick(guard, 0)
        self.assertTrue(self.tick(guard, .36)['pump_on'])
        self.assertEqual(guard.burst_deadline, .5)
        state = self.tick(guard, .5)
        self.assertEqual(state['reason'], 'arm_lease_expired')
        self.assertFalse(state['pump_on'])
        self.assertFalse(self.tick(guard, 1)['pump_on'])

    def test_repeated_arm_cannot_extend_or_reset_budget(self):
        guard = PumpGuard()
        self.begin_burst(guard)
        original = guard.snapshot()
        result = guard.arm(10.4, 30)
        self.assertFalse(result['arm_accepted'])
        self.assertEqual(result['arm_deadline'], original['arm_deadline'])
        self.assertEqual(result['burst_count'], 1)
        self.assertTrue(result['pump_on'])
        overdue = guard.arm(11, 30)
        self.assertFalse(overdue['pump_on'])
        self.assertFalse(overdue['arm_accepted'])
        self.assertEqual(overdue['state'], 'COOLDOWN')
        self.assertFalse(guard.arm(30, 30)['arm_accepted'])
        self.assertFalse(guard.pump_on)

    def test_printed_flame_cannot_trigger_unlimited_bursts(self):
        guard = PumpGuard(dict(burst_s=.1, cooldown_s=.5, stable_lock_s=.1))
        guard.arm(0, 30)
        rising_edges, previous = 0, False
        for frame in range(3001):
            state = self.tick(guard, frame/100)
            rising_edges += int(state['pump_on'] and not previous)
            previous = state['pump_on']
        self.assertEqual(rising_edges, 3)
        self.assertEqual(state['state'], 'EXHAUSTED')
        self.assertTrue(state['requires_disarm'])
        self.assertFalse(guard.arm(31, 30)['arm_accepted'])
        guard.disarm()
        self.assertTrue(guard.arm(31, 30)['arm_accepted'])
        self.assertEqual(guard.burst_count, 0)

    def test_delayed_tick_cannot_reuse_elapsed_cooldown(self):
        guard = PumpGuard()
        start = self.begin_burst(guard)
        state = self.tick(guard, start+5)
        self.assertFalse(state['pump_on'])
        self.assertEqual(state['cooldown_deadline'], start+6)
        self.assertFalse(self.tick(guard, start+5.99)['pump_on'])

    def test_maximum_requested_continuous_burst_one_second(self):
        guard = PumpGuard(dict(burst_s=1))
        start = self.begin_burst(guard)
        deadline = guard.burst_deadline
        self.assertAlmostEqual(deadline-start, 1)
        self.assertTrue(self.tick(guard, deadline-.001)['pump_on'])
        self.assertFalse(self.tick(guard, deadline)['pump_on'])

    def test_estop_shutdown_disarm_and_clock_reverse(self):
        for operation in ('estop', 'disarm', 'rollback'):
            guard = PumpGuard()
            self.begin_burst(guard)
            if operation == 'estop':
                state = self.tick(guard, 10.4, estop=True)
            elif operation == 'disarm':
                state = guard.disarm('shutdown')
            else:
                state = self.tick(guard, 9)
                self.assertEqual(state['state'], 'FAULT')
                self.assertFalse(guard.arm(11, 5)['arm_accepted'])
            self.assertFalse(state['pump_on'])
            self.assertFalse(state['armed'])
            self.assertFalse(self.tick(guard, 12)['pump_on'])

    def test_invalid_parameters_strictly_rejected(self):
        for config in (dict(burst_s=0), dict(burst_s=1.001), dict(burst_s=float('nan')),
                       dict(cooldown_s=.49), dict(cooldown_s=float('inf')),
                       dict(max_bursts=4), dict(max_bursts=True), dict(max_bursts=1.2),
                       dict(stable_lock_s=-.1), dict(stable_lock_s='0.35'), dict(extra=1)):
            with self.subTest(config=config), self.assertRaises(ValueError):
                PumpGuard(config)
        for lease in (0, 30.01, float('nan'), float('inf'), True, '30', 10**400):
            guard = PumpGuard()
            self.begin_burst(guard)
            with self.subTest(lease=lease), self.assertRaises(ValueError):
                guard.arm(10.4, lease)
            self.assertFalse(guard.pump_on)

    def test_invalid_time_and_missing_verification_fail_closed(self):
        for bad in (float('nan'), float('inf'), -1, True, '12', 10**400):
            guard = PumpGuard()
            self.begin_burst(guard)
            with self.subTest(now=bad), self.assertRaises(ValueError):
                self.tick(guard, bad)
            self.assertFalse(guard.pump_on)
            self.assertEqual(guard.state, 'FAULT')
        for value in (None, 1, 'true'):
            guard = PumpGuard()
            self.begin_burst(guard)
            with self.assertRaises(ValueError):
                self.tick(guard, 11, aim_ok=value)
            self.assertFalse(guard.pump_on)


if __name__ == '__main__':
    unittest.main()
