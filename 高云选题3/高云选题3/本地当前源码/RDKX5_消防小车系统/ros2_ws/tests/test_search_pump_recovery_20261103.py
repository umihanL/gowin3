"""Unknown LOW must hold/STOP, never become implicit motion authorization."""
import sys
import unittest
from types import SimpleNamespace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rescue_vision'))
from rescue_vision.search_pump_recovery import SearchPumpRecovery


class Navigation:
    def __init__(self, phase='planning'):
        self.active = dict(command_id='leg-1', task_id=7, execute_fire=False,
            continuous_route=True, route_generation=11, phase=phase,
            phase_at=10., deadline=60.)
        self.measured_stopped = phase != 'running'
        self.calls = []

    def stopped(self, now, after=None):
        return self.measured_stopped

    def pause(self, command_id, now):
        self.calls.append(('pause', command_id))
        self.active.update(phase='pausing', pause_at=now, sequence=23)

    def resume(self, command_id, now):
        self.calls.append(('resume', command_id))
        self.active['phase'] = 'resuming'


def snapshots(now=10., fresh=False):
    a = dict(stamp=now, pump_on=False if fresh else None, pump_feedback_fresh=fresh,
             pump_backend='fpga_uart', pump_pin='B13', guard=dict(armed=False),
             pump_requested_on=False, manual_pump_active=False, estop=False)
    p = dict(backend='fpga_uart', pin='B13', feedback_fresh=fresh,
             pump_on=False if fresh else None, feedback_stamp=now if fresh else None, error='')
    return a, p


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.r = SearchPumpRecovery()
        self.n = Navigation()
        self.a, self.p = snapshots()

    def step(self, now=10., reason='pump_output_pending_or_not_confirmed_off', **kwargs):
        return self.r.step(self.n, self.a, self.p, reason, now, **kwargs)

    def test_unknown_low_holds_prestart_without_motion_or_fake_low(self):
        d = self.step()
        self.assertTrue(d.hold_prestart and d.hold_search and d.request_off)
        self.assertEqual(self.n.calls, [])
        self.assertIsNone(self.p['pump_on'])
        self.assertFalse(self.step(10.2).request_off)

    def test_fresh_bridge_without_fresh_local_actuator_does_not_release(self):
        self.step()
        self.a, self.p = snapshots(9., True)
        self.assertTrue(self.step(10.4, '').hold_prestart)

    def test_confirmed_low_releases_same_prestart_and_preserves_deadline(self):
        self.step()
        self.n.active.update(phase='starting', phase_at=10.2)
        self.a, self.p = snapshots(10.8, True)
        d = self.step(10.8, '')
        self.assertFalse(d.hold_search)
        self.assertFalse(self.r.active)
        self.assertEqual(self.n.calls, [])
        self.assertAlmostEqual(self.n.active['phase_at'], 10.8)
        self.assertAlmostEqual(self.n.active['deadline'], 60.8)

    def test_unknown_low_running_pauses_then_requires_measured_stop(self):
        self.n = Navigation('running')
        self.assertTrue(self.step().hold_search)
        self.assertEqual(self.n.calls, [('pause', 'leg-1')])
        self.a, self.p = snapshots(10.4, True)
        self.n.active['phase'] = 'paused'
        self.assertTrue(self.step(10.4, '').hold_search)
        self.n.measured_stopped = True
        d = self.step(10.5, '')
        self.assertTrue(d.resumed)
        self.assertEqual(self.n.calls, [('pause', 'leg-1'), ('resume', 'leg-1')])

    def test_never_resumes_existing_fire_or_operator_pause(self):
        self.n = Navigation('paused')
        self.step()
        self.a, self.p = snapshots(10.4, True)
        self.step(10.4, '')
        self.assertEqual(self.n.calls, [])

    def test_pending_on_actual_on_armed_manual_and_estop_abort(self):
        mutations = [('pump_on', True), ('pump_requested_on', True),
                     ('manual_pump_active', True), ('estop', True), ('guard', {'armed': True})]
        for key, value in mutations:
            with self.subTest(key=key):
                self.a, self.p = snapshots()
                self.a[key] = value
                self.assertTrue(self.step().abort_reason)
                self.assertFalse(self.r.active)
        self.a, self.p = snapshots()
        self.assertTrue(self.step(pending_on=True).abort_reason)
        self.p['pump_on'] = True
        self.assertTrue(self.step().abort_reason)

    def test_axis_error_does_not_replace_independent_pump_low(self):
        self.a['error'] = 'PD42 timeout'
        self.step()
        self.a, self.p = snapshots(10.2, True)
        self.a['error'] = 'PD42 timeout'
        self.assertFalse(self.step(10.2, '').hold_search)

    def test_timeout_returns_original_reason_never_resumes(self):
        self.n = Navigation('running')
        self.step()
        d = self.step(13.)
        self.assertIn('timeout:pump_output_pending_or_not_confirmed_off', d.abort_reason)
        self.assertEqual(self.n.calls, [('pause', 'leg-1')])

    def test_operator_stop_and_replaced_task_never_resume(self):
        self.n = Navigation('running')
        self.step()
        self.n.active['phase'] = 'stopping'
        self.assertFalse(self.step(10.2).handled)
        self.assertFalse(self.r.active)
        self.n = Navigation('running')
        self.step()
        self.n.active = dict(self.n.active, command_id='other-leg', phase='paused')
        self.assertFalse(self.step(10.4).handled)
        self.assertEqual(self.n.calls, [('pause', 'leg-1')])

    def test_route_change_and_external_pause_identity_abort(self):
        self.n = Navigation('running')
        self.step()
        self.n.active['route_generation'] = 12
        self.assertIn('route_changed', self.step(10.1).abort_reason)
        self.n = Navigation('running')
        self.step()
        self.n.active['pause_at'] = 10.05
        self.assertIn('ownership_lost', self.step(10.1).abort_reason)

    def test_centering_and_unowned_start_keep_explicit_stop_path(self):
        for phase in ('starting', 'centering_moving', 'centering_starting'):
            with self.subTest(phase=phase):
                self.n = Navigation(phase)
                self.assertTrue(self.step().abort_reason)
                self.assertEqual(self.n.calls, [])

    def test_firefighting_and_unrelated_failures_not_handled(self):
        self.n.active['execute_fire'] = True
        self.assertFalse(self.step().handled)
        self.n.active['execute_fire'] = False
        self.assertFalse(self.step(reason='navigation_feedback_lost').handled)

    def test_mode_change_or_loss_of_measured_stop_never_extends_hold(self):
        self.step()
        self.n.owner = SimpleNamespace(task='vision', mode_transition=None, zebra_transition=None)
        self.assertIn('mode_changed', self.step(10.1).abort_reason)
        self.n = Navigation()
        self.step()
        self.n.measured_stopped = False
        self.assertIn('stop_lost', self.step(10.2).abort_reason)

    def test_fresh_flag_on_pre_recovery_low_cannot_release(self):
        self.step()
        self.a, self.p = snapshots(10.3, True)
        self.p['feedback_stamp'] = 9.9
        self.assertTrue(self.step(10.3, '').hold_prestart)

    def test_explicitly_tracked_unwritten_start_is_held_but_written_start_is_not(self):
        self.n = Navigation('starting')
        self.n.active.update(start_write_tracking=True, sequence=17)
        self.assertTrue(self.step().hold_prestart)
        self.r.reset()
        self.n.active.update(start_written_at=9.99, start_written_sequence=17)
        self.assertTrue(self.step().abort_reason)


if __name__ == '__main__':
    unittest.main()
