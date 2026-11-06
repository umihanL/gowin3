"""Numbered destinations: actual ingress, task routing, and ASR parameter agreement."""
import http.client
import importlib.util
import json
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision.task_interaction import parse_task_text, TaskRouter
from rescue_vision.voice_verification import verify_command, needs_independent_verification


def extension(task='dialogue', **changes):
    value = dict(stamp=100., real_uart=True, status_fresh=True, protocol_ready=True,
                 task=task, dialogue_enabled=True, vehicle_stopped=True,
                 can_confirm_point_start=True, can_navigate_point=True,
                 point_reference=dict(point_id=2, confirmed=True, heading_confirmed=True,
                                      current_heading_deg=90.), point_navigation_reason='ready')
    value.update(changes)
    return value


def command(text, **changes):
    value = dict(parse_task_text(text), source='voice', command_id='numbered-1', issued_at=100., ttl_s=3.,
                 voice_verification=dict(verified=True))
    value.update(changes)
    return value


class NumberedInteractionTests(unittest.TestCase):
    def router(self):
        return TaskRouter(extension_enabled=True, navigation_execute_enabled=True)

    def test_chinese_and_numeric_points_are_canonical(self):
        for text in ('从二号点位导航到六号点位', '从2号点到6号点', '从2号点位移动到6号点位'):
            parsed = parse_task_text(text)
            self.assertEqual((parsed['action'], parsed['start_point'], parsed['target_point']), ('navigate_point', 2, 6))
        self.assertEqual(parse_task_text('去三号点位')['target_point'], 3)
        self.assertEqual(parse_task_text('前往9号点')['target_point'], 9)
        self.assertEqual(parse_task_text('瞄准2号火源')['target_id'], 2)

    def test_negation_questions_and_compounds_cannot_move(self):
        for text in ('不要从2号点到6号点', '从2号点到6号点可以吗', '如果从2号点到6号点',
                     '确认实际起点为2号点'):
            self.assertEqual(parse_task_text(text)['action'], 'dialogue', text)
        with self.assertRaisesRegex(ValueError, 'numbered_navigation_separate_actions'):
            parse_task_text('去6号点然后喷水')
        for text in ('去0号点', '去十号点', '从2号点到2号点'):
            with self.assertRaises(ValueError): parse_task_text(text)

    def test_building_and_point_names_are_not_interchanged(self):
        self.assertEqual(parse_task_text('去3号楼', {'buildings': [{'building_id': 'B3', 'number': 3}]})['action'], 'navigate')
        self.assertEqual(parse_task_text('去3号点')['action'], 'navigate_point')

    def test_numbered_voice_routes_to_single_extension_owner(self):
        for task, policy in (('vision', 'manual'), ('dialogue', 'manual'), ('semantic', 'manual')):
            result = self.router().route(command('从2号点到6号点'), 100., extension=extension(task))
            self.assertEqual(result['extension']['arrival_policy'], policy)
            self.assertEqual(result['extension']['target_point'], 6)
            self.assertIsNone(result['navigation'])
            self.assertIsNone(result['mission'])  # Core accepts first, then changes task state.
            self.assertEqual(result['actuator']['action'], 'disarm')
            self.assertEqual(result['auto']['action'], 'cancel')
            self.assertIn('等待执行端确认', result['speech'])
            self.assertNotIn('已提交FPGA规划与执行', result['speech'])

    def test_missing_offline_or_stale_navigation_is_not_queued(self):
        for state in ({}, extension(stamp=99.), extension(real_uart=False),
                      extension(status_fresh=False), extension(can_navigate_point=False)):
            with self.assertRaises(ValueError):
                self.router().route(command('去6号点', source='web'), 100., extension=state)
        with self.assertRaisesRegex(ValueError, 'not_enabled'):
            TaskRouter(extension_enabled=False).route(command('去6号点'), 100., extension=extension())
        self.assertTrue(TaskRouter(extension_enabled=True, navigation_execute_enabled=False).route(
            command('去6号点'), 100., extension=extension())['accepted'])

    def test_text_parameters_cannot_be_substituted(self):
        for changes in ({'target_point': 7}, {'start_point': 3}):
            with self.assertRaisesRegex(ValueError, 'text_point_mismatch'):
                self.router().route(command('从2号点到6号点', **changes), 100., extension=extension())
        with self.assertRaisesRegex(ValueError, 'observe_or_scan'):
            self.router().route(command('去6号点', arrival_policy='auto'), 100., extension=extension())

    def test_confirmation_is_console_only_and_does_not_start(self):
        req = dict(action='confirm_point_start', point_id=2, confirmed=True, start_heading_deg=90.,
                   source='web', command_id='point-confirm', issued_at=100., ttl_s=3.)
        result = self.router().route(req, 100., extension=extension())
        self.assertEqual(result['extension']['action'], 'confirm_point_start')
        self.assertIsNone(result['mission'])
        self.assertIsNone(result['navigation'])
        for changes in ({'source': 'voice'}, {'confirmed': False}, {'point_id': True},
                        {'start_heading_deg':None}, {'start_heading_deg':True},
                        {'start_heading_deg':float('nan')}, {'start_heading_deg':181.}):
            with self.assertRaises(ValueError):
                self.router().route(dict(req, **changes), 100., extension=extension())

    def test_independent_asr_must_agree_on_both_points(self):
        self.assertTrue(needs_independent_verification('从2号点到6号点', {'confidence_available': False}, None))
        yes = verify_command('从二号点到六号点', dict(text='从2号点导航到6号点', result=[{'conf': .95}]), None)
        self.assertTrue(yes['verified'])
        for text in ('从3号点到6号点', '从2号点到7号点', '不要去6号点'):
            no = verify_command('从二号点到六号点', dict(text=text, result=[{'conf': .95}]), None)
            self.assertFalse(no['verified'], text)


class NumberedHttpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('numbered_http_under_test', ROOT/'tools/presentation_server.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def setUp(self):
        self.state = self.module.PresentationState(monotonic=lambda: 100., wall=lambda: 100.)
        self.state.ros.update(available=True, router_ready=True)
        self.state.update('extension', extension())
        self.state.update('mission', dict(stamp=100., mode='inspect', vehicle_stopped=True))
        self.server = self.module.BoundedServer(('127.0.0.1', 0), self.module.BaseHTTPRequestHandler)
        self.host = '127.0.0.1:' + str(self.server.server_address[1])
        self.server.RequestHandlerClass = self.module.handler_class(self.state, ROOT/'web', Mock(), {self.host})
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join()

    def post(self, payload, path='/api/point-navigation'):
        connection = http.client.HTTPConnection('127.0.0.1', self.server.server_address[1], timeout=3.)
        connection.request('POST', path, json.dumps(payload),
                           {'Content-Type': 'application/json', 'Host': self.host, 'Origin': 'http://' + self.host})
        response = connection.getresponse()
        status, body = response.status, json.loads(response.read())
        connection.close()
        return status, body

    def test_confirmation_and_navigation_reach_real_queue(self):
        for action, values in (('confirm_point_start', dict(point_id=2, confirmed=True, start_heading_deg=90.)),
                               ('navigate_point', dict(start_point=2, target_point=6))):
            status, result = self.post(dict(request_id=action, action=action, **values))
            self.assertEqual(status, 202, result)
            self.assertEqual(result['status'], 'queued')
            self.assertFalse(result['synthetic'])
        published = []
        self.state.drain(lambda kind, value: published.append((kind, value)), lambda: 100.)
        self.assertEqual(len(published), 2)
        for kind, value in published:
            self.assertEqual(kind, 'command'); self.assertEqual(value['source'], 'web')
            self.assertEqual(value['issued_at'], 100.)
            self.assertTrue(TaskRouter(extension_enabled=True, navigation_execute_enabled=True).route(
                value, 100., extension=extension())['accepted'])

    def test_cannot_override_source_policy_or_send_unknown_action(self):
        for extra in ({'source': 'voice'}, {'arrival_policy': 'auto'}, {'issued_at': 100.}):
            status, _ = self.post(dict(request_id='bad', action='navigate_point', target_point=6, **extra))
            self.assertEqual(status, 400)
        self.assertEqual(self.post(dict(request_id='bad', action='start_spray'))[0], 400)
        self.assertEqual(self.post(dict(request_id='bad', action='confirm_point_start', point_id=2, confirmed=False))[0], 400)

    def test_existing_text_entry_parses_numbered_points_without_qwen(self):
        status, result = self.post(dict(request_id='text', text='从二号点位导航到六号点位'), '/api/command')
        self.assertEqual(status, 202, result)
        self.assertEqual(result['action'], 'navigate_point')
        published = []
        self.state.drain(lambda kind, value: published.append(value), lambda: 100.)
        self.assertEqual(len(published), 1)
        self.assertEqual(published[0]['action'], 'navigate_point')
        self.assertEqual(published[0]['start_point'], 2)
        self.assertEqual(published[0]['target_point'], 6)
        self.assertNotIn('manual_point_command', published[0])
        self.assertEqual(published[0]['text'], '从二号点位导航到六号点位')

    def test_changed_payload_and_link_loss_do_not_dispatch(self):
        payload = dict(request_id='one', action='navigate_point', target_point=6)
        self.assertEqual(self.post(payload)[0], 202)
        self.assertEqual(self.post(dict(payload, target_point=7))[0], 409)
        self.state.update('extension', extension(real_uart=False))
        self.state.drain(lambda *_: self.fail('must not publish after link loss'), lambda: 100.)
        self.assertEqual(self.state.commands['one']['status'], 'rejected')


if __name__ == '__main__':
    unittest.main()
