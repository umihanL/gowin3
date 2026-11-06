"""Explicit text/voice pulse requests share the live manual-pump owner."""
import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision.task_interaction import TaskRouter, parse_task_text
from rescue_vision.voice import prepare_spoken_command
from rescue_vision.voice_verification import needs_independent_verification, verify_command
import test_manual_pump_api as http_fixture
import test_voice_capture_lifecycle as voice_fixture


def request(text='手动试喷', **changes):
    value = dict(parse_task_text(text), command_id='pulse-text', source='web', issued_at=100., ttl_s=2.)
    value.update(changes)
    return value


class ManualPumpCommandTests(unittest.TestCase):
    def test_exact_phrases_and_integer_durations(self):
        for text, duration in [('手动试喷', 400), ('手动试喷400毫秒', 400), ('手动试喷四百毫秒', 400),
                               ('手动试喷四百五十毫秒', 450), ('手动试喷一千毫秒', 1000),
                               ('小车，请手动试喷一百毫秒。', 100)]:
            self.assertEqual(parse_task_text(text), dict(text=text, action='manual_pump_test', duration_ms=duration))
        for text in ('不要手动试喷', '手动试喷？', '“手动试喷”', '如果手动试喷',
                     '手动试喷40.0毫秒', '手动试喷40 . 0毫秒', '手动试喷400.0毫秒', '手动试喷然后前进', '手动试喷吗'):
            self.assertEqual(parse_task_text(text)['action'], 'dialogue', text)
        for text in ('手动试喷99毫秒', '手动试喷1001毫秒'):
            with self.assertRaises(ValueError): parse_task_text(text)

    def test_router_outputs_only_the_dedicated_finite_pulse(self):
        state = dict(stamp=100., can_start=True)
        out = TaskRouter().route(request(), 100., pump_manual=state)
        self.assertEqual(out['pump_manual'], dict(action='start', duration_ms=400, operator_confirmed=True,
            command_id='pulse-text', issued_at=100., ttl_s=2.))
        for key in ('auto', 'actuator', 'mission', 'navigation', 'extension'):
            self.assertIsNone(out[key])
        self.assertFalse(TaskRouter().route(request(command_id='unique'), 100., pump_manual=state).get('completed', False))
        router = TaskRouter()
        router.route(request(), 100., pump_manual=state)
        self.assertFalse(router.route(request(), 100., pump_manual=state)['accepted'])

    def test_freshness_explicit_text_and_exact_duration_cannot_be_bypassed(self):
        for state in (None, {}, dict(stamp=98., can_start=True), dict(stamp=102., can_start=True),
                      dict(stamp=100., can_start=False, ready_reason='vehicle_not_stopped'),
                      dict(stamp=100., can_start=True, synthetic=True)):
            with self.assertRaises(ValueError): TaskRouter().route(request(), 100., pump_manual=state)
        for changes in (dict(text=''), dict(source='test'), dict(text='不要手动试喷'),
                        dict(duration_ms=500), dict(duration_ms=True), dict(duration_ms=400.),
                        dict(issued_at=97.9, ttl_s=5.)):
            with self.assertRaises(ValueError):
                TaskRouter().route(dict(request(), **changes), 100., pump_manual=dict(stamp=100., can_start=True))

    def test_voice_requires_independent_same_duration_agreement_even_with_confidence(self):
        for scored in (True, False):
            self.assertTrue(needs_independent_verification('手动试喷', dict(confidence_available=scored), None))
        proof = verify_command('手动试喷400毫秒', dict(text='手动试喷四百毫秒', result=[dict(conf=.99)]), None)
        self.assertTrue(proof['verified'])
        self.assertFalse(verify_command('手动试喷400毫秒', dict(text='手动试喷五百毫秒', result=[dict(conf=.99)]), None)['verified'])
        for evidence in (None, {}, dict(verified=False)):
            with self.assertRaisesRegex(ValueError, 'manual_pump_independent_asr_required'):
                TaskRouter().route(request(source='voice', voice_verification=evidence), 100.,
                    pump_manual=dict(stamp=100., can_start=True))
        self.assertIn('pump_manual', TaskRouter().route(request(source='voice', voice_verification=proof),
            100., pump_manual=dict(stamp=100., can_start=True)))

    def test_stop_remains_available_without_readiness_or_wake_word(self):
        self.assertEqual(parse_task_text('停止试喷')['action'], 'stop_spray')
        self.assertEqual(prepare_spoken_command('停止试喷', 'continuous', True), '停止试喷')
        out = TaskRouter(extension_enabled=True).route(request('停止试喷', source='voice', issued_at=0.), 100.)
        self.assertEqual(out['actuator']['action'], 'disarm')
        self.assertEqual(out['auto']['action'], 'cancel')


class ManualPumpTextHttpTests(unittest.TestCase):
    setUpClass = classmethod(http_fixture.ManualPumpApiTests.setUpClass.__func__)
    setUp = http_fixture.ManualPumpApiTests.setUp
    tearDown = http_fixture.ManualPumpApiTests.tearDown
    post = http_fixture.ManualPumpApiTests.post
    ready = http_fixture.ManualPumpApiTests.ready

    def test_text_http_rechecks_same_gate_and_matches_node_feedback(self):
        payload = dict(text='手动试喷四百毫秒', request_id='text-pulse')
        self.assertEqual(self.post(payload, '/api/command')[0], 409)
        self.ready()
        code, queued = self.post(payload, '/api/command')
        self.assertEqual(code, 202, queued)
        sent = []
        self.state.drain(lambda kind, value: sent.append((kind, value)), lambda: 100.)
        self.assertEqual(sent[0][0], 'command')
        self.assertEqual(sent[0][1]['ttl_s'], 2.)
        out = TaskRouter().route(sent[0][1], 100., pump_manual=self.state.envelope('pump_manual')['data'])
        self.assertEqual(out['pump_manual']['command_id'], queued['command_id'])
        self.state.update('pump_manual', dict(stamp=100., request_stamp=100.,
            last_request_id=queued['command_id'], last_request_action='start', last_result='completed',
            last_request_reason='confirmed_low', request_id=queued['command_id'], phase='completed'))
        self.assertEqual(self.state.commands['text-pulse']['pump_manual_result'], 'completed')
        self.state.update('interaction', dict(command_id=queued['command_id'], accepted=True))
        self.assertEqual(self.state.commands['text-pulse']['status'], 'observed')
        payload['request_id'] = 'text-pulse2'
        self.ready()
        self.assertEqual(self.post(payload, '/api/command')[0], 202)
        self.ready(can_start=False, ready_reason='vehicle_not_stopped')
        self.state.drain(lambda *_: self.fail('text pulse bypassed changed readiness'), lambda: 100.)
        self.assertEqual(self.state.commands['text-pulse2']['reason'], 'vehicle_not_stopped')


class ManualPumpVoiceIngressTests(unittest.TestCase):
    setUpClass = classmethod(voice_fixture.VoiceCaptureLifecycleTests.setUpClass.__func__)
    setUp = voice_fixture.VoiceCaptureLifecycleTests.setUp
    make_voice = voice_fixture.VoiceCaptureLifecycleTests.make_voice

    def test_real_voice_ingress_preserves_proof_and_short_lease(self):
        voice = self.make_voice()
        voice.verification_backend = Mock()
        decoder = Mock()
        decoder.done.is_set.return_value = True
        decoder.combined.return_value = dict(text='手动试喷四百毫秒', result=[dict(conf=.99)])
        with patch.object(self.module, 'StreamingASR', return_value=decoder), patch.object(voice, 'wait_decoder'):
            proof = voice.verify_voice_action('手动试喷400毫秒', dict(confidence_available=True), [b'\0\0' * 1600], 100000000.)
        self.assertTrue(proof['verified'], proof)
        self.assertEqual(voice.accept_text('手动试喷', 'voice', 10.)['reason'], 'manual_pump_independent_asr_required')
        self.assertEqual(voice.command_pub.values, [])
        self.assertEqual(voice.accept_text('手动试喷', 'voice', 10., verification=proof)['state'], 'command_submitted')
        command = voice.command_pub.values[-1]
        self.assertEqual(command['voice_verification'], proof)
        self.assertEqual(command['ttl_s'], 2.)
        out = TaskRouter().route(command, 10., pump_manual=dict(stamp=10., can_start=True))
        self.assertEqual(out['pump_manual']['duration_ms'], 400)

    def test_router_node_wires_manual_topic_and_dispatches_once(self):
        spec = importlib.util.spec_from_file_location('rescue_vision._manual_pump_router_test',
            ROOT / 'src/rescue_vision/rescue_vision/task_interaction_node.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with patch.object(voice_fixture.FakeNode, 'create_subscription') as subscriptions:
            node = module.TaskInteraction()
        self.assertIn('/pump/manual/state', [call.args[1] for call in subscriptions.call_args_list])
        self.assertIn('/pump/manual/request', node.publishers)
        node.output_pubs['speech'] = types.SimpleNamespace(publish=lambda value: None)
        node.update('pump_manual', types.SimpleNamespace(data=json.dumps(dict(stamp=10., can_start=True))))
        node.dispatch(request(issued_at=10.), 10.)
        values = node.publishers['/pump/manual/request'].values
        self.assertEqual(len(values), 1)
        self.assertEqual(values[0]['duration_ms'], 400)
        node.dispatch(request(issued_at=10.), 10.)
        self.assertEqual(len(values), 1)


if __name__ == '__main__':
    unittest.main()
