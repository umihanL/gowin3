"""Manual pulse HTTP boundary, queue safety and correlated real-node feedback."""
import unittest

import test_numbered_interaction as fixture


class ManualPumpApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.NumberedHttpTests.setUpClass()
        cls.module = fixture.NumberedHttpTests.module

    setUp = fixture.NumberedHttpTests.setUp
    tearDown = fixture.NumberedHttpTests.tearDown
    post = fixture.NumberedHttpTests.post

    def ready(self, **changes):
        self.state.ros['pump_manual_ready'] = True
        value = dict(stamp=100., can_start=True, ready_reason='ready', phase='idle')
        value.update(changes)
        self.state.update('pump_manual', value)

    def request(self, **changes):
        value = dict(request_id='pulse1', action='start', duration_ms=400, operator_confirmed=True)
        value.update(changes)
        return value

    def submit(self, payload=None):
        return self.post(self.request() if payload is None else payload, '/api/pump-manual')

    def test_one_shot_request_has_bounded_duration_and_separate_ros_owner(self):
        self.ready()
        request = self.request()
        request.pop('duration_ms')
        status, record = self.submit(request)
        self.assertEqual(status, 202, record)
        self.assertEqual(record['pump_manual']['duration_ms'], 400)
        sent = []
        self.state.drain(lambda kind, value: sent.append((kind, value)), lambda: 100.)
        self.assertEqual(sent, [('pump_manual', dict(action='start', duration_ms=400,
            operator_confirmed=True, command_id=record['command_id'], issued_at=100., ttl_s=2.))])
        self.assertEqual(self.state.commands['pulse1']['status'], 'submitted')
        self.assertNotIn('pump_manual_status', self.state.commands['pulse1'])
        self.assertEqual(self.state.snapshot()['topics']['pump_manual']['topic'], '/pump/manual/state')

    def test_duration_confirmation_and_injected_fields_fail_closed(self):
        self.ready()
        cases = [dict(duration_ms=value) for value in (99, 1001, True, 400., '400', None, float('nan'))]
        cases += [dict(operator_confirmed=value) for value in (False, 1, 'true', None)]
        cases += [dict(action='spray'), dict(issued_at=100.), dict(ttl_s=2.),
                  dict(command_id='injected'), dict(source='web'), dict(continuous=True)]
        for changes in cases:
            with self.subTest(changes=changes):
                self.assertEqual(self.submit(self.request(**changes))[0], 400)
        unconfirmed = self.request()
        unconfirmed.pop('operator_confirmed')
        self.assertEqual(self.submit(unconfirmed)[0], 400)
        self.assertEqual(self.submit(dict(action='stop', request_id='stop', duration_ms=400))[0], 400)
        self.assertEqual(self.submit(dict(action='stop', request_id='stop', operator_confirmed=True))[0], 400)
        self.assertEqual(self.submit(dict(action='start', operator_confirmed=True))[0], 400)
        self.assertFalse(self.state.pending)
        for value in (100, 1000):
            self.assertEqual(self.submit(self.request(request_id='bound' + str(value), duration_ms=value))[0], 202)

    def test_start_requires_fresh_node_readiness_and_subscriber(self):
        self.assertEqual(self.submit()[0], 409)
        self.ready()
        self.state.ros['pump_manual_ready'] = False
        self.assertEqual(self.submit()[0], 409)
        for changes in (dict(stamp=98.), dict(stamp=102.), dict(stamp=None),
                        dict(can_start=False, ready_reason='pump_cooldown'), dict(can_start='true')):
            with self.subTest(changes=changes):
                self.ready(**changes)
                status, result = self.submit()
                self.assertEqual(status, 409, result)
        self.assertFalse(self.state.pending)
        self.ready()
        # The dedicated owner supplies readiness; automatic fire calibration
        # cannot be silently overridden through a general text command.
        self.assertEqual(self.submit()[0], 202)
        self.assertEqual(self.post(dict(text='开始喷水', request_id='automatic'), '/api/command')[0], 409)

    def test_queue_rechecks_readiness_and_expires_start_without_retry(self):
        self.ready()
        self.submit()
        self.ready(can_start=False, ready_reason='vehicle_not_stopped')
        self.state.drain(lambda *_: self.fail('readiness changed after queueing'), lambda: 100.)
        self.assertEqual(self.state.commands['pulse1']['reason'], 'vehicle_not_stopped')
        self.ready()
        self.submit(self.request(request_id='pulse2'))
        self.state.monotonic = lambda: 102.01
        self.state.drain(lambda *_: self.fail('expired pulse dispatched'), lambda: 102.01)
        self.assertEqual(self.state.commands['pulse2']['status'], 'expired')
        self.assertEqual(self.submit(self.request(request_id='pulse2'))[1]['status'], 'expired')
        self.assertFalse(self.state.pending)

    def test_stop_preempts_queued_pulse_and_preserves_other_owners(self):
        self.ready()
        self.submit()
        self.state.ros['motor_test_ready'] = True
        self.state.enqueue(request_id='motor-stop', motor_test='stop', motor_id='motor1')
        self.state.latest.pop('pump_manual')
        status, record = self.submit(dict(request_id='pulse-stop', action='stop'))
        self.assertEqual(status, 202, record)
        self.assertEqual(self.state.commands['pulse1']['reason'], 'superseded_by_pump_manual_stop')
        sent = []
        self.state.drain(lambda kind, value: sent.append((kind, value)), lambda: 100.)
        self.assertEqual([kind for kind, _ in sent], ['pump_manual', 'motor_test'])
        self.assertEqual(sent[0][1], dict(action='stop', command_id=record['command_id'], issued_at=100., ttl_s=2.))
        self.assertEqual(self.submit()[1]['status'], 'rejected')
        self.assertFalse(self.state.pending)

    def test_stop_survives_queue_age_and_uses_reserved_capacity(self):
        self.ready()
        for index in range(8):
            self.state.enqueue(text='你好', request_id='task' + str(index))
        self.state.ros.update(listen_ready=True, audio_test_ready=True, motor_test_ready=True,
            gimbal_self_test_ready=True, gimbal_manual_ready=True)
        self.state.enqueue(request_id='listen-stop', listen=False)
        self.state.enqueue(request_id='audio-stop', audio_test=False)
        self.state.enqueue(request_id='motor1-stop', motor_test='stop', motor_id='motor1')
        self.state.enqueue(request_id='motor2-stop', motor_test='stop', motor_id='motor2')
        self.state.enqueue(request_id='gimbal-test-stop', gimbal_test={'action': 'stop'})
        self.state.enqueue(request_id='gimbal-manual-stop', gimbal_manual={'action': 'stop'})
        self.assertEqual(self.submit(dict(request_id='pulse-stop', action='stop'))[0], 202)
        self.assertEqual(len(self.state.pending), 15)
        self.assertEqual({item[0] for item in self.state.pending}, set(self.state.commands))
        self.state.monotonic = lambda: 110.
        sent = []
        self.state.drain(lambda kind, value: sent.append((kind, value)), lambda: 110.)
        self.assertEqual(len(sent), 1)
        self.assertEqual((sent[0][0], sent[0][1]['action']), ('pump_manual', 'stop'))

    def test_same_id_is_idempotent_and_changed_payload_is_rejected(self):
        self.ready()
        _, first = self.submit()
        _, repeated = self.submit()
        self.assertEqual(first['command_id'], repeated['command_id'])
        self.assertEqual(len(self.state.pending), 1)
        self.assertEqual(self.submit(self.request(duration_ms=500))[0], 409)
        self.assertEqual(self.submit(dict(request_id='pulse1', action='stop'))[0], 409)
        default = self.request()
        default.pop('duration_ms')
        self.assertEqual(self.submit(default)[1]['command_id'], first['command_id'])
        sent = []
        self.state.drain(lambda *args: sent.append(args), lambda: 100.)
        self.state.drain(lambda *args: sent.append(args), lambda: 100.)
        self.submit()
        self.assertEqual(len(sent), 1)
        self.assertFalse(self.state.pending)

    def test_feedback_requires_fresh_exact_request_and_keeps_terminal_result(self):
        self.ready()
        _, record = self.submit()
        self.state.drain(lambda *_: None, lambda: 100.)
        feedback = dict(stamp=100., last_request_id=record['command_id'], last_request_action='start',
            request_stamp=100., last_result='completed', last_request_reason='pump_off_confirmed', phase='completed')
        for changes in (dict(last_request_id='other'), dict(stamp=98.), dict(request_stamp=98.),
                        dict(last_request_action='stop'), dict(synthetic=True)):
            self.state.update('pump_manual', dict(feedback, **changes))
            self.assertEqual(self.state.commands['pulse1']['status'], 'submitted')
        self.state.update('pump_manual', feedback)
        self.assertEqual(self.state.commands['pulse1']['pump_manual_result'], 'completed')
        self.state.update('pump_manual', dict(feedback, last_result='accepted', phase='spraying'))
        self.assertEqual(self.state.commands['pulse1']['pump_manual_result'], 'completed')
        self.assertEqual(self.state.commands['pulse1']['pump_manual_status']['phase'], 'completed')

    def test_rejected_second_start_does_not_replace_current_run_progress(self):
        self.ready()
        _, first = self.submit()
        _, second = self.submit(self.request(request_id='pulse2'))
        self.state.drain(lambda *_: None, lambda: 100.)
        feedback = dict(stamp=100., request_id=first['command_id'], phase='spraying', reason='spraying',
            last_request_id=second['command_id'], last_request_action='start', request_stamp=100.,
            last_result='rejected', last_request_reason='manual_pulse_busy')
        self.state.update('pump_manual', feedback)
        self.assertEqual(self.state.commands['pulse2']['status'], 'rejected')
        self.assertEqual(self.state.commands['pulse1']['status'], 'observed')
        self.assertEqual(self.state.commands['pulse1']['pump_manual_status']['phase'], 'spraying')
        self.state.wall = lambda: 101.2
        self.state.update('pump_manual', dict(feedback, stamp=101.2, phase='completed', reason='pump_off_confirmed'))
        self.assertEqual(self.state.commands['pulse1']['pump_manual_result'], 'completed')
        self.assertEqual(self.state.commands['pulse2']['reason'], 'manual_pulse_busy')

    def test_publish_failure_and_receipt_timeout_never_retry(self):
        self.ready()
        self.submit()

        def failed_publish(*_):
            raise RuntimeError('no transport')

        self.state.drain(failed_publish, lambda: 100.)
        self.assertEqual(self.submit()[1]['reason'], 'publish_failed:RuntimeError')
        self.assertFalse(self.state.pending)
        self.submit(self.request(request_id='pulse2'))
        self.state.drain(lambda *_: None, lambda: 100.)
        self.state.wall = lambda: 106.
        self.state.snapshot()
        self.assertEqual(self.submit(self.request(request_id='pulse2'))[1]['status'], 'receipt_timeout')
        self.assertFalse(self.state.pending)

    def test_delayed_stop_completion_and_duplicate_rejection_preserve_run_evidence(self):
        self.ready()
        _, first = self.submit()
        self.state.drain(lambda *_: None, lambda: 100.)
        feedback = dict(stamp=100., request_id=first['command_id'], phase='spraying', reason='spraying',
            last_request_id=first['command_id'], last_request_action='start', request_stamp=100.,
            last_result='accepted', last_request_reason='accepted')
        self.state.update('pump_manual', feedback)
        self.state.update('pump_manual', dict(feedback, last_result='rejected',
            last_request_reason='manual_pump_unique_request_required'))
        self.assertEqual(self.state.commands['pulse1']['status'], 'observed')
        self.assertEqual(self.state.commands['pulse1']['reason'], 'spraying')
        _, stop = self.submit(dict(action='stop', request_id='pulse-stop'))
        self.state.drain(lambda *_: None, lambda: 100.)
        self.state.wall = lambda: 101.2
        feedback.update(stamp=101.2, phase='cancelled', reason='operator_stop',
            last_request_id=stop['command_id'], last_request_action='stop',
            last_result='cancelled', last_request_reason='pump_off_confirmed')
        self.state.update('pump_manual', feedback)
        self.assertEqual(self.state.commands['pulse-stop']['pump_manual_result'], 'cancelled')
        self.assertEqual(self.state.commands['pulse1']['pump_manual_result'], 'cancelled')


if __name__ == '__main__':
    unittest.main()
