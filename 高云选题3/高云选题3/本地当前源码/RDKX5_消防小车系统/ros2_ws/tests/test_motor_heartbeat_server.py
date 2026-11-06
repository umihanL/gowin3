"""Renewals cannot start motion, survive Stop, or extend an unrelated shaft run."""
import unittest
from test_presentation_server import module, Clock


class MotorHeartbeatTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.state = module.PresentationState(monotonic=self.clock, wall=self.clock)
        self.state.ros.update(available=True, motor_test_ready=True)
        self.refresh()
        self.runs = {}
        for motor in ('motor1', 'motor2'):
            # Build the actual console ledger without requiring ROS or serial I/O.
            record = self.state.enqueue(request_id=motor, motor_test='run_positive', motor_id=motor)
            self.runs[motor] = record['command_id']
        self.state.drain(lambda *_: None, self.clock)
        self.motor_feedback()

    def refresh(self):
        for topic, data in (
            ('mission', dict(mode='inspect')),
            ('actuators', dict(dry_run=True, enabled=False)),
            ('fpga_link', dict(dry_run=True)),
        ):
            self.state.update(topic, dict(data, stamp=self.clock()))

    def motor_feedback(self, **changes):
        motors = {motor: dict(stamp=self.clock(), connected=True, feedback_fresh=True,
            continuous_running=True, run_id=run_id) for motor, run_id in self.runs.items()}
        motors['motor2'].update(changes)
        self.state.update('motor_test', dict(stamp=self.clock(), motors=motors))

    def test_two_independent_latest_renewals_without_command_or_event_growth(self):
        sizes = len(self.state.commands), len(self.state.events)
        for _ in range(30):
            for motor, run in self.runs.items(): self.state.motor_heartbeat(motor, run)
        self.assertEqual(len(self.state.motor_heartbeats), 2)
        self.assertEqual(sizes, (len(self.state.commands), len(self.state.events)))
        calls = []
        self.state.drain(lambda k, v: calls.append((k, v)), self.clock)
        self.assertEqual(len(calls), 2)
        for kind, payload in calls:
            self.assertEqual(kind, 'motor_test')
            self.assertEqual(payload['action'], 'keepalive')
            self.assertEqual(payload['run_id'], self.runs[payload['motor_id']])
            self.assertLessEqual(payload['ttl_s'], 1.)
        self.state.drain(lambda *_: self.fail('heartbeat retried'), self.clock)

    def test_stop_revokes_one_shaft_before_dispatch_and_preserves_other(self):
        for motor, run in self.runs.items(): self.state.motor_heartbeat(motor, run)
        self.state.enqueue(request_id='stop2', motor_test='stop', motor_id='motor2')
        with self.assertRaisesRegex(module.ApiError, 'ended_run'):
            self.state.motor_heartbeat('motor2', self.runs['motor2'])
        calls = []
        self.state.drain(lambda k, v: calls.append(v), self.clock)
        self.assertEqual([(v['motor_id'], v['action']) for v in calls],
                         [('motor2', 'stop'), ('motor1', 'keepalive')])

    def test_old_heartbeat_never_crosses_new_run_or_wrong_motor(self):
        for motor, run in (('motor1', self.runs['motor2']), ('motor2', 'unknown')):
            with self.assertRaises(module.ApiError): self.state.motor_heartbeat(motor, run)
        self.motor_feedback(run_id='new-driver-run')
        with self.assertRaisesRegex(module.ApiError, 'live_run_required'):
            self.state.motor_heartbeat('motor2', self.runs['motor2'])

    def test_global_stop_revokes_both_runs_before_ros_feedback_catches_up(self):
        self.state.ros['router_ready'] = True
        for motor, run in self.runs.items(): self.state.motor_heartbeat(motor, run)
        self.state.enqueue(text='停止喷水', request_id='global-stop')
        self.assertEqual(self.state.motor_heartbeats, {})
        for motor, run in self.runs.items():
            with self.assertRaisesRegex(module.ApiError, 'ended_run'):
                self.state.motor_heartbeat(motor, run)

    def test_pending_confirmation_can_renew_but_finished_run_cannot(self):
        self.motor_feedback(continuous_running=False, continuous_pending=True)
        self.state.motor_heartbeat('motor2', self.runs['motor2'])
        self.motor_feedback(continuous_running=False, continuous_pending=False, run_id=None)
        self.state.drain(lambda *_: self.fail('run finished before renewal'), self.clock)

    def test_fresh_aggregate_does_not_hide_stale_or_missing_worker_feedback(self):
        for changes in (dict(stamp=self.clock()-2), dict(feedback_fresh=False),
                        dict(connected=False), dict(continuous_running=False), dict(synthetic=True)):
            self.motor_feedback(**changes)
            with self.assertRaises(module.ApiError): self.state.motor_heartbeat('motor2', self.runs['motor2'])
        self.state.update('motor_test', dict(stamp=self.clock(), motors=None))
        with self.assertRaises(module.ApiError): self.state.motor_heartbeat('motor2', self.runs['motor2'])

    def test_queued_heartbeat_expires_even_after_new_feedback_arrives(self):
        self.state.motor_heartbeat('motor2', self.runs['motor2'])
        self.clock.value += 1.1
        self.refresh(); self.motor_feedback()
        self.state.drain(lambda *_: self.fail('expired heartbeat'), self.clock)

    def test_guard_rechecked_at_publish_and_failure_not_retried(self):
        self.state.motor_heartbeat('motor2', self.runs['motor2'])
        self.state.update('fpga_link', dict(stamp=self.clock(), dry_run=False))
        self.state.drain(lambda *_: self.fail('UART owner changed'), self.clock)
        self.refresh()
        self.state.motor_heartbeat('motor2', self.runs['motor2'])
        def fail(*_): raise RuntimeError('ROS publish failed')
        self.state.drain(fail, self.clock)
        self.state.drain(lambda *_: self.fail('retry'), self.clock)

    def test_heartbeat_not_accepted_on_ordinary_command_path(self):
        with self.assertRaisesRegex(module.ApiError, 'unsupported_motor_test_action'):
            self.state.enqueue(request_id='bad', motor_test='keepalive', motor_id='motor2')


if __name__ == '__main__':
    unittest.main()
