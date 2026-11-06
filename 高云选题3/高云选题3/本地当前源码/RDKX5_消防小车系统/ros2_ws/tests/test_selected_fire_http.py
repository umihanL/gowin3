"""Selected-fire HTTP admission and queued dispatch, without ROS or hardware I/O."""
import copy
import http.client
from http.server import BaseHTTPRequestHandler
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock


WS = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('selected_fire_presentation_server',
                                            WS / 'tools/presentation_server.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

TEXT = '对选定火焰自动瞄准喷水并关水'
IDENTITY = dict(target_id=2, registry_session='session', selection_generation=3)


class Clock:
    def __init__(self):
        self.value = 100.

    def __call__(self):
        return self.value


def ready_evidence(now):
    """Real-feedback-shaped messages; initial alignment is deliberately incomplete."""
    return dict(
        mission=dict(stamp=now, mode='fire', vehicle_stopped=True, gimbal_ready=False,
                     mounting_verified=True, gimbal_transport='x5'),
        fire_selection=dict(capture_stamp=now, valid=True, **IDENTITY),
        fire_proposal=dict(capture_stamp=now, valid=True, aim_mode='fixed_camera_3d',
                           calibration_ready=True, landing_compensated=True,
                           water_impact_calibration_verified=True,
                           landing_calibration_id='measured', geometry_locked=False,
                           pump_candidate=False, **IDENTITY),
        actuators=dict(stamp=now, enabled=True, dry_run=False, status_fresh=True,
                       feedback_fresh=True, reference_confirmed_this_start=True,
                       actual_angles_deg=[0., 0.], pump_enabled=True, pump_ready=True,
                       pump_on=False, pump_feedback_fresh=True,
                       guard=dict(armed=False, state='DISARMED')),
        extension=dict(stamp=now, real_uart=True, status_fresh=True,
                       vehicle_stopped=True, busy=False, task='dialogue',
                       dialogue_enabled=True))


class SelectedFireHttpTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.clock = Clock()
        self.state = module.PresentationState(monotonic=self.clock, wall=self.clock)
        self.state.ros.update(available=True, router_ready=True)
        self.set_evidence()
        self.server = module.BoundedServer(('127.0.0.1', 0), BaseHTTPRequestHandler)
        self.port = self.server.server_address[1]
        self.host = '127.0.0.1:' + str(self.port)
        self.proxy = Mock()
        self.server.RequestHandlerClass = module.handler_class(
            self.state, Path(self.temp.name), self.proxy, {self.host})
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.close_server)

    def close_server(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def set_evidence(self, changes=None):
        evidence = ready_evidence(self.clock())
        for topic, fields in (changes or {}).items():
            evidence[topic].update(fields)
        for topic, data in evidence.items():
            self.state.update(topic, data)

    def post(self, payload):
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=3)
        try:
            connection.request('POST', '/api/command', json.dumps(payload).encode(),
                               {'Host': self.host, 'Content-Type': 'application/json',
                                'Origin': 'http://' + self.host})
            response = connection.getresponse()
            return response.status, json.loads(response.read())
        finally:
            connection.close()

    def payload(self, request_id='selected', **changes):
        return dict(dict(text=TEXT, request_id=request_id,
                         target_expectation=copy.deepcopy(IDENTITY)), **changes)

    def assert_rejected(self, payload, expected_reason, status=409):
        code, data = self.post(payload)
        self.assertEqual(code, status, data)
        self.assertEqual(data['error'], expected_reason, data)
        self.assertFalse(self.state.pending)

    def test_http_queues_before_alignment_and_dispatch_preserves_click_identity_once(self):
        code, record = self.post(self.payload())
        self.assertEqual(code, 202, record)
        self.assertEqual(record['action'], 'extinguish_selected')
        self.assertEqual(record['status'], 'queued')
        self.assertEqual(record['target_expectation'], IDENTITY)
        self.assertNotIn('accepted', record)
        self.assertNotIn('receipt', record)
        self.assertNotIn('execution', record)
        self.assertEqual(len(self.state.pending), 1)
        self.assertFalse(self.proxy.mock_calls)
        published = []
        self.state.drain(lambda kind, value: published.append((kind, value)), self.clock)
        self.assertEqual(len(published), 1)
        kind, value = published[0]
        self.assertEqual(kind, 'command')
        self.assertEqual(value['action'], 'extinguish_selected')
        self.assertEqual(value['source'], 'web')
        self.assertEqual(value['text'], TEXT)
        self.assertEqual(value['command_id'], record['command_id'])
        self.assertEqual(value['issued_at'], self.clock())
        self.assertEqual(value['ttl_s'], 3.)
        self.assertNotIn('target_expectation', value)
        for key, expected in IDENTITY.items():
            self.assertEqual(value['expected_' + key], expected)
        self.assertEqual(self.state.commands['selected']['status'], 'submitted')
        self.assertNotIn('receipt', self.state.commands['selected'])
        self.state.drain(lambda *args: self.fail('a second dispatch must not occur'), self.clock)

    def test_target_expectation_remains_optional_for_existing_text_clients(self):
        code, record = self.post(dict(text=TEXT, request_id='legacy'))
        self.assertEqual(code, 202, record)
        published = []
        self.state.drain(lambda kind, value: published.append(value), self.clock)
        self.assertEqual(len(published), 1)
        self.assertEqual(published[0]['action'], 'extinguish_selected')
        self.assertFalse(any(key.startswith('expected_') for key in published[0]))

    def test_http_rejects_invalid_expectation_shape_including_explicit_null(self):
        invalid = [None, True, 1, 'session', [], {}, {'target_id': 2},
                   dict(IDENTITY, pump_on=True)]
        invalid.extend({k: v for k, v in IDENTITY.items() if k != omitted}
                       for omitted in IDENTITY)
        for expectation in invalid:
            with self.subTest(expectation=expectation):
                code, data = self.post(self.payload(target_expectation=expectation))
                self.assertEqual(code, 400, data)
                self.assertIn('error', data)
                self.assertFalse(self.state.pending)

    def test_http_rejects_identity_type_coercions_and_unbounded_session(self):
        for key in ('target_id', 'selection_generation'):
            for value in (True, False, None, 0, -1, 2., '2', [], {}):
                with self.subTest(key=key, value=value):
                    self.assert_rejected(self.payload(target_expectation=dict(IDENTITY, **{key: value})),
                                         'invalid_selected_fire_identity', 400)
        for session in (None, True, 2, '', 'x' * 161, [], {}):
            with self.subTest(session=session):
                self.assert_rejected(self.payload(target_expectation=dict(IDENTITY, registry_session=session)),
                                     'invalid_selected_fire_identity', 400)

    def test_http_forbids_target_expectation_on_other_actions(self):
        for text in ('锁定火焰2', '寻找火焰', '开始喷水', '停止喷水', '报告当前场景'):
            with self.subTest(text=text):
                self.assert_rejected(self.payload(text=text),
                                     'target_expectation_only_for_selected_fire_round', 400)
        code, data = self.post(self.payload(text='停止喷水', target_expectation=None))
        self.assertEqual(code, 400, data)
        self.assertFalse(self.state.pending)

    def test_http_rejects_flattened_identity_and_arbitrary_action_fields(self):
        for extra in (dict(action='extinguish_selected'), dict(expected_target_id=2),
                      dict(expected_registry_session='session'),
                      dict(expected_selection_generation=3), dict(pump_on=True)):
            with self.subTest(extra=extra):
                self.assert_rejected(self.payload(**extra), 'only_text_and_request_id_allowed', 400)

    def test_same_request_id_is_idempotent_and_changed_identity_or_text_is_rejected(self):
        code, original = self.post(self.payload())
        self.assertEqual(code, 202, original)
        reordered = {key: IDENTITY[key] for key in reversed(IDENTITY)}
        code, duplicate = self.post(self.payload(target_expectation=reordered))
        self.assertEqual(code, 202, duplicate)
        self.assertEqual(duplicate['command_id'], original['command_id'])
        self.assertEqual(len(self.state.pending), 1)
        self.state.drain(lambda *args: None, self.clock)
        code, duplicate = self.post(self.payload())
        self.assertEqual(code, 202, duplicate)
        self.assertEqual(duplicate['status'], 'submitted')
        self.assertEqual(duplicate['command_id'], original['command_id'])
        self.assertFalse(self.state.pending)
        for changes in (dict(target_id=4), dict(registry_session='next-session'),
                        dict(selection_generation=4)):
            with self.subTest(changes=changes):
                self.assert_rejected(self.payload(target_expectation=dict(IDENTITY, **changes)),
                                     'request_id_reused_with_different_payload')
        self.assert_rejected(dict(text=TEXT, request_id='selected'),
                             'request_id_reused_with_different_payload')
        self.assert_rejected(self.payload(text='对选定火焰执行一轮灭火'),
                             'request_id_reused_with_different_payload')

    def test_click_identity_must_match_current_selection_before_enqueue(self):
        for changes in (dict(target_id=9), dict(registry_session='new-session'),
                        dict(selection_generation=4)):
            with self.subTest(changes=changes):
                self.set_evidence({'fire_selection': changes, 'fire_proposal': changes})
                self.assert_rejected(self.payload(), 'selected_fire_changed_before_submit')

    def test_selection_changes_while_queued_are_rejected_without_publish(self):
        for index, changes in enumerate((dict(target_id=9), dict(registry_session='new-session'),
                                         dict(selection_generation=4))):
            with self.subTest(changes=changes):
                self.set_evidence()
                request_id = 'changed-' + str(index)
                code, record = self.post(self.payload(request_id))
                self.assertEqual(code, 202, record)
                self.set_evidence({'fire_selection': changes, 'fire_proposal': changes})
                self.state.drain(lambda *args: self.fail('changed target must not be published'), self.clock)
                saved = self.state.commands[request_id]
                self.assertEqual(saved['status'], 'rejected')
                self.assertEqual(saved['reason'], 'selected_fire_changed_before_submit')
                self.assertNotIn('submitted_at', saved)
                self.assertNotIn('receipt', saved)

    def test_missing_calibration_is_rejected_with_specific_reason(self):
        cases = [('mission', dict(mounting_verified=False), 'fixed_camera_mounting_unverified'),
                 ('fire_proposal', dict(calibration_ready=False), 'fixed_camera_uncalibrated'),
                 ('fire_proposal', dict(landing_compensated=False), 'fixed_camera_water_impact_unmeasured'),
                 ('fire_proposal', dict(water_impact_calibration_verified=False),
                  'fixed_camera_water_impact_unmeasured'),
                 ('fire_proposal', dict(landing_calibration_id=''), 'fixed_camera_water_impact_unmeasured')]
        for topic, fields, reason in cases:
            with self.subTest(topic=topic, fields=fields):
                self.set_evidence({topic: fields})
                self.assert_rejected(self.payload(), reason)

    def test_stale_feedback_is_rejected_even_when_messages_just_arrived(self):
        cases = [('actuators', dict(stamp=self.clock() - .6), 'real_actuator_feedback_required'),
                 ('actuators', dict(feedback_fresh=False), 'region_real_gimbal_reference_required'),
                 ('actuators', dict(pump_feedback_fresh=False), 'pump_off_fresh_actuator_feedback_required'),
                 ('fire_selection', dict(capture_stamp=self.clock() - .6), 'fresh_explicit_fire_selection_required'),
                 ('fire_proposal', dict(capture_stamp=self.clock() - .5), 'one_click_fire_geometry_required'),
                 ('extension', dict(stamp=self.clock() - .7), 'real_extension_uart_required')]
        for topic, fields, reason in cases:
            with self.subTest(topic=topic, fields=fields):
                self.set_evidence({topic: fields})
                self.assert_rejected(self.payload(), reason)

    def test_expired_evidence_is_rejected_at_http_boundary(self):
        for topic, timestamp in (('actuators', 'stamp'), ('fire_selection', 'capture_stamp'),
                                 ('fire_proposal', 'capture_stamp')):
            with self.subTest(topic=topic):
                self.set_evidence({topic: {timestamp: self.clock() - 2.}})
                self.assert_rejected(self.payload(), 'selected_fire_evidence_stale')

    def test_queued_round_rechecks_calibration_and_feedback_before_publish(self):
        cases = [('fire_proposal', dict(calibration_ready=False), 'fixed_camera_uncalibrated'),
                 ('actuators', dict(pump_feedback_fresh=False), 'pump_off_fresh_actuator_feedback_required')]
        for index, (topic, fields, reason) in enumerate(cases):
            with self.subTest(topic=topic):
                self.set_evidence()
                request_id = 'lost-evidence-' + str(index)
                code, record = self.post(self.payload(request_id))
                self.assertEqual(code, 202, record)
                self.set_evidence({topic: fields})
                self.state.drain(lambda *args: self.fail('lost evidence must prevent dispatch'), self.clock)
                self.assertEqual(self.state.commands[request_id]['status'], 'rejected')
                self.assertEqual(self.state.commands[request_id]['reason'], reason)


if __name__ == '__main__':
    unittest.main()
