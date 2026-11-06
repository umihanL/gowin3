"""HTTP boundary, no-IO sandbox, freshness and one-shot ROS dispatch checks."""
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch, Mock

WS = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('presentation_server', WS / 'tools/presentation_server.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Clock:
    def __init__(self):
        self.value = 1000.
    def __call__(self):
        return self.value


class ExecutorLifecycleTests(unittest.TestCase):
    def test_waits_without_repeated_subscription_registration(self):
        stop = threading.Event()
        executor, node = Mock(), object()
        iterations = []

        def spin_once(timeout_sec):
            iterations.append(timeout_sec)
            if len(iterations) == 4:
                stop.set()

        executor.spin_once.side_effect = spin_once
        module.spin_until_stopped(node, stop, executor, lambda: True)
        self.assertEqual(iterations, [.1] * 4)
        executor.add_node.assert_called_once_with(node)
        executor.remove_node.assert_called_once_with(node)
        executor.shutdown.assert_called_once_with()

    def test_executor_cleanup_on_callback_error(self):
        executor, node = Mock(), object()
        executor.spin_once.side_effect = RuntimeError('callback failure')
        with self.assertRaisesRegex(RuntimeError, 'callback failure'):
            module.spin_until_stopped(node, threading.Event(), executor, lambda: True)
        executor.remove_node.assert_called_once_with(node)
        executor.shutdown.assert_called_once_with()

    def test_ros_context_shutdown_exits_without_spinning(self):
        executor, node = Mock(), object()
        module.spin_until_stopped(node, threading.Event(), executor, lambda: False)
        executor.spin_once.assert_not_called()
        executor.remove_node.assert_called_once_with(node)
        executor.shutdown.assert_called_once_with()


class FrozenBundleTests(unittest.TestCase):
    def test_frozen_assets_resolve_to_bundle_not_source_or_cwd(self):
        with tempfile.TemporaryDirectory() as directory:
            bundle = Path(directory)
            (bundle / 'web').mkdir()
            (bundle / 'config').mkdir()
            (bundle / 'web/presentation.html').write_text('bundled-page', encoding='utf-8')
            (bundle / 'config/buildings.json').write_text('{"buildings":[]}', encoding='utf-8')
            with patch.object(sys, 'frozen', True, create=True), patch.object(sys, '_MEIPASS', directory, create=True):
                root = module.resource_root()
                self.assertEqual(root, bundle.resolve())
                self.assertEqual((root / 'web/presentation.html').read_text(encoding='utf-8'), 'bundled-page')
                self.assertEqual(json.loads((root / 'config/buildings.json').read_text(encoding='utf-8')), {'buildings': []})
            self.assertEqual(module.resource_root(), WS)

    def test_frozen_live_launch_rejected_before_ros_initialization(self):
        with patch.object(sys, 'frozen', True, create=True), patch.object(sys, 'argv', ['presentation_backend']), \
                patch.object(module, 'PreviewProxy', side_effect=AssertionError('must not create live proxy')), \
                patch.object(module, 'ros_loop', side_effect=AssertionError('must not start ROS')), \
                patch('sys.stderr', new_callable=io.StringIO) as output:
            with self.assertRaises(SystemExit) as caught:
                module.main()
            self.assertEqual(caught.exception.code, 2)
            self.assertIn('Run this ROS service on X5', output.getvalue())


class StateTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.state = module.PresentationState(monotonic=self.clock, wall=self.clock)
        self.state.ros.update(available=True, router_ready=True, listen_ready=True)
        self.state.update('mission', {'stamp': self.clock(), 'mode': 'standby'})
        self.state.update('actuators', dict(stamp=self.clock(), dry_run=False, enabled=True, pump_enabled=True))

    def test_search_only_uses_observation_gate_without_enabling_gimbal_or_water(self):
        self.state.update('extension', dict(stamp=self.clock(), region_observe_ready=True,
            region_ready=False, region_reason='region_real_gimbal_reference_required'))
        self.state.update('actuators', dict(stamp=self.clock(), dry_run=False, enabled=False, pump_enabled=False))
        search = dict(action='search_region', region_id='C', execute_fire=False)
        self.assertEqual(self.state.execution_ready(search), (True, ''))
        self.assertEqual(self.state.execution_ready(dict(search, execute_fire=True)),
                         (False, 'region_real_gimbal_reference_required'))
        for malformed in (None, 0, 1, '', 'false', 'true'):
            self.assertEqual(self.state.execution_ready(dict(search, execute_fire=malformed)),
                             (False, 'region_execute_fire_must_be_boolean'))
        self.clock.value += 20
        self.assertEqual(self.state.execution_ready(search), (False, 'extension_status_unavailable'))

    def test_full_region_mode_retains_pump_gate(self):
        self.state.update('extension', dict(stamp=self.clock(), region_ready=True, region_observe_ready=True))
        self.state.update('actuators', dict(stamp=self.clock(), dry_run=False, enabled=True, pump_enabled=False))
        self.assertEqual(self.state.execution_ready(dict(action='search_region', execute_fire=True)),
                         (False, 'real_pump_not_enabled'))

    def test_region_observation_readiness_is_rechecked_before_dispatch(self):
        self.state.update('extension', dict(stamp=self.clock(), region_observe_ready=True, region_ready=False))
        record = self.state.enqueue('去C区域搜索火焰', 'observe-regression')
        self.assertEqual(record['status'], 'queued')
        self.state.update('extension', dict(stamp=self.clock(), region_observe_ready=False,
            region_observe_reason='region_start_pose_and_heading_required'))
        self.state.drain(lambda *args: self.fail('must not publish after readiness lost'), self.clock)
        self.assertEqual(self.state.commands['observe-regression']['status'], 'rejected')
        self.assertEqual(self.state.commands['observe-regression']['reason'], 'region_start_pose_and_heading_required')

    def test_no_publish_from_enqueue_or_http_state_read(self):
        record = self.state.enqueue('寻找火焰', 'req1')
        self.assertEqual(record['status'], 'queued')
        self.assertNotIn('receipt', record)
        self.assertEqual(self.state.snapshot()['commands'][0]['status'], 'queued')
        seen = []
        self.state.drain(lambda kind, data: seen.append((kind, data)), self.clock)
        self.assertEqual(len(seen), 1)
        self.assertEqual(seen[0][1]['action'], 'inspect')
        self.assertEqual(seen[0][1]['source'], 'web')
        self.assertEqual(seen[0][1]['ttl_s'], 3.)
        self.assertEqual(self.state.commands['req1']['status'], 'submitted')
        self.state.drain(lambda *args: self.fail('repeated dispatch'), self.clock)

    def test_repeated_request_id_never_requeues(self):
        one = self.state.enqueue('寻找火焰', 'req1')
        self.state.drain(lambda *args: None, self.clock)
        two = self.state.enqueue('寻找火焰', 'req1')
        self.assertEqual(one['command_id'], two['command_id'])
        self.assertEqual(len(self.state.pending), 0)
        with self.assertRaisesRegex(module.ApiError, 'different_payload'):
            self.state.enqueue('开始喷水', 'req1')

    def test_executor_rejection_survives_router_and_dispatch_callback_order(self):
        record = self.state.enqueue('寻找火焰', 'executor')
        result = dict(command_id=record['command_id'], stamp=self.clock(), accepted=False,
                      completed=False, reason='pump_off_fresh_actuator_feedback_required',
                      irrelevant_large_payload={'ignored': True})
        def publish(kind, value):
            self.state.update('extension', dict(stamp=self.clock(), last_result=result))
            self.state.update('interaction', dict(command_id=record['command_id'], accepted=True))
        self.state.drain(publish, self.clock)
        saved = self.state.commands['executor']
        self.assertEqual(saved['status'], 'execution')
        self.assertFalse(saved['execution']['accepted'])
        self.assertNotIn('irrelevant_large_payload', saved['execution'])
        self.clock.value += 10
        self.assertEqual(self.state.snapshot()['commands'][0]['status'], 'execution')
        self.assertEqual(len(self.state.pending), 0)

    def test_retained_executor_result_deduplicates_heartbeats_and_preserves_terminal_result(self):
        record = self.state.enqueue('寻找火焰', 'executor')
        accepted = dict(command_id=record['command_id'], stamp=self.clock(), accepted=True, completed=False)
        self.state.update('extension', dict(stamp=self.clock(), last_result=accepted))
        rejected = dict(accepted, stamp=self.clock() + 1, accepted=False, reason='blocked')
        self.clock.value += 1
        self.state.update('extension', dict(stamp=self.clock(), last_result=rejected))
        for _ in range(3):
            self.clock.value += 10
            self.state.update('extension', dict(stamp=self.clock(), last_result=rejected))
        self.state.update('extension', dict(stamp=self.clock(), last_result=accepted))
        self.assertEqual(len([e for e in self.state.events if e['kind'] == 'extension_result']), 2)
        self.assertEqual(self.state.commands['executor']['execution'], rejected)

    def test_late_retained_result_replaces_timeout_but_invalid_outcomes_do_not_guess(self):
        record = self.state.enqueue('寻找火焰', 'executor')
        self.state.drain(lambda *args: None, self.clock)
        self.clock.value += 10
        self.assertEqual(self.state.snapshot()['commands'][0]['status'], 'receipt_timeout')
        for result in [dict(stamp=1001., accepted=False),
                       dict(command_id=record['command_id'], stamp=999., accepted=False),
                       dict(command_id=record['command_id'], stamp=1001., accepted=False, synthetic=True),
                       dict(command_id=record['command_id'], stamp=1011., accepted=False),
                       dict(command_id=record['command_id'], stamp=1001., accepted='false')]:
            self.state.update('extension', dict(stamp=self.clock(), last_result=result))
        self.assertNotIn('execution', self.state.commands['executor'])
        result = dict(command_id=record['command_id'], stamp=1001., accepted=False, reason='blocked')
        self.state.update('extension', dict(stamp=self.clock(), last_result=result))
        self.assertEqual(self.state.commands['executor']['execution'], result)
        self.assertNotIn('reason', self.state.commands['executor'])
        self.assertEqual(len(self.state.pending), 0)

    def test_voice_executor_result_has_compact_event_without_web_command(self):
        result = dict(command_id='voice-command', stamp=900., accepted=False, reason='blocked')
        self.state.update('extension', dict(stamp=self.clock(), last_result=result))
        self.assertEqual(self.state.events[-1]['kind'], 'extension_result')
        self.assertEqual(self.state.events[-1]['data'], result)
        self.assertFalse(self.state.commands)

    def test_navigation_parser_receipt_and_executor_receipt_have_separate_timeout(self):
        record = self.state.enqueue('寻找火焰', 'executor')
        self.state.commands['executor']['action'] = 'navigate_point'
        self.state.drain(lambda *args: None, self.clock)
        self.state.update('interaction', dict(command_id=record['command_id'], action='navigate_point',
                                              accepted=True, stamp=self.clock()))
        self.clock.value += 6
        saved = self.state.snapshot()['commands'][0]
        self.assertEqual(saved['status'], 'execution_timeout')
        self.assertEqual(saved['reason'], 'navigation_execution_receipt_missing')
        self.assertTrue(saved['receipt']['accepted'])
        self.state.update('extension', dict(stamp=self.clock(), last_result=dict(
            command_id=record['command_id'], stamp=self.clock(), accepted=False, reason='blocked')))
        self.assertEqual(self.state.snapshot()['commands'][0]['status'], 'execution')
        self.assertFalse(self.state.commands['executor']['execution']['accepted'])
        self.assertFalse(self.state.pending)

    def test_tts_availability_transition_is_event_but_heartbeat_timestamp_is_not(self):
        self.state.update('voice', dict(stamp=self.clock(), state='idle', tts_state='ready'))
        before = len(self.state.events)
        self.state.update('voice', dict(stamp=self.clock() + 1, state='idle', tts_state='ready'))
        self.assertEqual(len(self.state.events), before)
        self.state.update('voice', dict(stamp=self.clock() + 1, state='idle', tts_state='unavailable',
                                       tts_error='audio_output_device_missing', tts_device_ready=False))
        self.assertEqual(len(self.state.events), before + 1)

    def test_source_timestamp_stale_despite_recent_reception(self):
        self.state.update('mission', {'stamp': 1., 'mode': 'inspect'})
        self.assertFalse(self.state.envelope('mission')['fresh'])
        with self.assertRaisesRegex(module.ApiError, 'mission_state_stale'):
            self.state.enqueue('寻找火焰', 'req1')
        self.assertEqual(self.state.enqueue('停止喷水', 'req2')['status'], 'queued')

    def test_future_and_nested_capture_timestamps(self):
        self.state.update('traffic_preview', {'stamp': {'sec': 1001, 'nanosec': 0}, 'status': 'red'})
        self.assertFalse(self.state.envelope('traffic_preview')['fresh'])
        self.state.update('traffic_preview', {'capture_ros_s': 999.9, 'status': 'red'})
        self.assertTrue(self.state.envelope('traffic_preview')['fresh'])
        self.clock.value += 2
        self.assertFalse(self.state.envelope('traffic_preview')['fresh'])

    def test_executor_rechecks_router_and_freshness(self):
        self.state.enqueue('寻找火焰', 'req1')
        self.state.ros['router_ready'] = False
        self.state.drain(lambda *args: self.fail('must not publish'), self.clock)
        self.assertEqual(self.state.commands['req1']['status'], 'rejected')

    def test_expired_queued_request_is_never_published(self):
        self.state.enqueue('寻找火焰', 'req1')
        self.clock.value += 4
        self.state.drain(lambda *args: self.fail('expired dispatch'), self.clock)
        self.assertEqual(self.state.commands['req1']['status'], 'expired')

    def test_stop_preempts_full_queue_and_cancels_unsent_arms(self):
        for n in range(8):
            self.state.enqueue('开始喷水', 'req' + str(n))
        stop = self.state.enqueue('停止喷水', 'stop')
        self.assertEqual(len(self.state.pending), 1)
        self.assertTrue(all(v['reason'] == 'superseded_by_stop' for k, v in self.state.commands.items() if k != 'stop'))
        seen = []
        self.state.drain(lambda kind, value: seen.append(value), self.clock)
        self.assertEqual([v['action'] for v in seen], ['stop_spray'])

    def test_bad_text_and_bounded_queue(self):
        for text in ('', 'x'*201, None):
            with self.assertRaises(module.ApiError):
                self.state.enqueue(text, 'bad')
        for n in range(8):
            self.state.enqueue('报告当前场景', str(n))
        with self.assertRaisesRegex(module.ApiError, 'queue_full'):
            self.state.enqueue('寻找火焰', 'overflow')

    def test_scene_answer_receipt_correlated_and_no_false_execution(self):
        record = self.state.enqueue('分析当前火场', 'req1')
        self.state.drain(lambda *args: None, self.clock)
        self.state.update('interaction', dict(command_id='other', accepted=True))
        self.assertEqual(self.state.commands['req1']['status'], 'submitted')
        self.state.update('interaction', dict(command_id=record['command_id'], accepted=False, reason='mission_stale'))
        self.assertEqual(self.state.commands['req1']['status'], 'receipt')
        self.assertFalse(self.state.commands['req1']['receipt']['accepted'])
        self.state.update('scene_answer', dict(command_id=record['command_id'], text='不能确定', accepted=True))
        self.assertEqual(self.state.commands['req1']['answer']['text'], '不能确定')
        self.assertFalse(self.state.snapshot()['control']['accepted_is_execution'])

    def test_listen_has_no_fabricated_ack_or_timeout(self):
        self.state.update('voice', dict(stamp=1000., microphone_enabled=True))
        self.state.enqueue(request_id='listen', listen=True)
        seen = []
        self.state.drain(lambda kind, data: seen.append((kind, data['enabled'])), self.clock)
        self.assertEqual(seen, [('listen', True)])
        self.clock.value += 8
        self.assertEqual(self.state.snapshot()['commands'][0]['status'], 'submitted')
        self.assertNotIn('receipt', self.state.commands['listen'])

    def test_listen_stop_preempts_only_its_pending_start_and_is_idempotent(self):
        self.state.ros['audio_test_ready'] = True
        self.state.update('voice', dict(stamp=self.clock(), microphone_enabled=True))
        self.state.enqueue('寻找火焰', 'task')
        self.state.enqueue(request_id='diagnostic', audio_test=True)
        self.state.enqueue(request_id='start', listen=True)
        one = self.state.enqueue(request_id='end', listen=False, cancel_start_request_id='start')
        two = self.state.enqueue(request_id='end', listen=False, cancel_start_request_id='start')
        self.assertEqual(one['command_id'], two['command_id'])
        self.assertEqual(self.state.commands['start']['reason'], 'superseded_by_listen_stop')
        self.assertEqual(self.state.commands['diagnostic']['status'], 'queued')
        seen = []
        self.state.drain(lambda kind, value: seen.append((kind, value)), self.clock)
        self.assertEqual([kind for kind, _ in seen], ['listen', 'command', 'audio_test'])
        self.assertFalse(seen[0][1]['enabled'])
        self.assertNotIn('receipt', self.state.commands['end'])

    def test_stop_before_delayed_start_never_opens_microphone(self):
        self.state.update('voice', dict(stamp=self.clock(), microphone_enabled=True))
        self.state.enqueue(request_id='end', listen=False, cancel_start_request_id='late-start')
        self.state.drain(lambda *args: None, self.clock)
        with self.assertRaisesRegex(module.ApiError, 'superseded_by_listen_stop'):
            self.state.enqueue(request_id='late-start', listen=True)
        self.assertEqual(len(self.state.pending), 0)
        # A deliberate later session with a new ID still works.
        self.state.enqueue(request_id='next-start', listen=True)

    def test_listen_stop_has_reserved_capacity_and_does_not_need_voice_freshness(self):
        for i in range(8):
            self.state.enqueue('报告当前场景', 'task-'+str(i))
        self.state.ros.update(router_ready=False, listen_ready=True)
        self.state.latest.clear()
        # Session ledger is full, independently of the ordinary pending queue.
        for i in range(1016):
            self.state.commands['old-'+str(i)] = dict(kind='command', action='scene', status='submitted')
        self.state.enqueue(request_id='end', listen=False)
        self.assertEqual(len(self.state.pending), 9)
        self.assertEqual(self.state.pending[0][0], 'end')
        self.assertTrue(all(self.state.commands['task-'+str(i)]['status'] == 'queued' for i in range(8)))
        calls = []
        self.state.drain(lambda kind, value: calls.append((kind, value['enabled'])), self.clock)
        self.assertEqual(calls, [('listen', False)])

    def test_listen_cancel_id_is_scoped_and_cannot_change_on_replay(self):
        self.state.enqueue(request_id='end', listen=False, cancel_start_request_id='start')
        with self.assertRaisesRegex(module.ApiError, 'different_payload'):
            self.state.enqueue(request_id='end', listen=False, cancel_start_request_id='other-start')
        for payload in (dict(listen=True), dict(audio_test=False), dict(text='停止喷水')):
            with self.subTest(payload=payload), self.assertRaisesRegex(module.ApiError, 'only_for_listen_stop'):
                self.state.enqueue(request_id='bad', cancel_start_request_id='start', **payload)
        # A referenced ID is never interpreted as a different kind of command.
        self.state.enqueue('寻找火焰', 'start')
        self.assertEqual(self.state.commands['start']['status'], 'queued')

    def test_listen_stop_slot_coexists_with_all_other_owners_at_queue_capacity(self):
        self.state.ros.update(audio_test_ready=True, motor_test_ready=True, gimbal_self_test_ready=True)
        for i in range(8):
            self.state.enqueue('报告当前场景', 'task-'+str(i))
        self.state.enqueue(request_id='test-end', audio_test=False)
        self.state.enqueue(request_id='motor1-end', motor_test='stop', motor_id='motor1')
        self.state.enqueue(request_id='motor2-end', motor_test='stop', motor_id='motor2')
        self.state.enqueue(request_id='gimbal-end', gimbal_test={'action': 'stop'})
        self.state.enqueue(request_id='listen-end', listen=False)
        self.assertEqual(len(self.state.pending), 13)
        self.state.enqueue(request_id='listen-end-again', listen=False)
        self.assertEqual(len(self.state.pending), 13)
        self.assertEqual(self.state.commands['listen-end']['reason'], 'superseded_by_listen_stop')
        for key in ('test-end', 'motor1-end', 'motor2-end', 'gimbal-end'):
            self.assertEqual(self.state.commands[key]['status'], 'queued')

    def test_audio_test_works_without_router_or_mission_and_never_becomes_task(self):
        self.state.ros.update(router_ready=False, listen_ready=False, audio_test_ready=True)
        self.state.latest.pop('mission')
        self.state.update('voice', dict(stamp=self.clock(), microphone_enabled=True))
        record = self.state.enqueue(request_id='test-start', audio_test=True)
        self.assertEqual(record['status'], 'queued')
        self.assertNotIn('audio_test_status', record)
        snapshot = self.state.snapshot()
        self.assertTrue(snapshot['control']['audio_test_ready'])
        self.assertFalse(snapshot['control']['command_ready'])
        seen = []
        self.state.drain(lambda kind, value: seen.append((kind, value)), self.clock)
        self.assertEqual(len(seen), 1)
        kind, value = seen[0]
        self.assertEqual(kind, 'audio_test')
        self.assertEqual(value, dict(enabled=True, command_id=record['command_id'],
                                    source='web', issued_at=self.clock(), ttl_s=3.))
        self.clock.value += 8
        self.assertEqual(self.state.snapshot()['commands'][0]['status'], 'submitted')

    def test_audio_test_start_requires_fresh_enabled_voice_and_rechecks_discovery(self):
        self.state.ros['audio_test_ready'] = True
        for payload in (dict(stamp=self.clock(), microphone_enabled=False),
                        dict(stamp=self.clock()-13, microphone_enabled=True)):
            self.state.update('voice', payload)
            with self.assertRaisesRegex(module.ApiError, 'microphone_disabled_or_voice_status_stale'):
                self.state.enqueue(request_id='start', audio_test=True)
        self.state.update('voice', dict(stamp=self.clock(), microphone_enabled=True))
        self.state.enqueue(request_id='start', audio_test=True)
        self.state.ros['audio_test_ready'] = False
        self.state.drain(lambda *args: self.fail('lost test subscriber must not receive request'), self.clock)
        self.assertEqual(self.state.commands['start']['reason'], 'audio_test_subscriber_unavailable')

    def test_audio_test_stop_cancels_only_test_starts_and_precedes_other_tasks(self):
        self.state.ros['audio_test_ready'] = True
        self.state.update('voice', dict(stamp=self.clock(), microphone_enabled=True))
        self.state.enqueue('寻找火焰', 'task')
        self.state.enqueue(request_id='normal-listen', listen=True)
        self.state.enqueue(request_id='test-start', audio_test=True)
        self.state.enqueue(request_id='test-stop', audio_test=False)
        self.assertEqual(self.state.commands['test-start']['reason'], 'superseded_by_audio_test_stop')
        self.assertEqual(self.state.commands['task']['status'], 'queued')
        self.assertEqual(self.state.commands['normal-listen']['status'], 'queued')
        seen = []
        self.state.drain(lambda kind, value: seen.append((kind, value)), self.clock)
        self.assertEqual([kind for kind, _ in seen], ['audio_test', 'command', 'listen'])
        self.assertFalse(seen[0][1]['enabled'])

    def test_audio_test_stop_has_reserved_slot_without_dropping_full_task_queue(self):
        self.state.ros['audio_test_ready'] = True
        for i in range(8):
            self.state.enqueue('报告当前场景', 'task-'+str(i))
        self.state.enqueue(request_id='test-stop', audio_test=False)
        self.assertEqual(len(self.state.pending), 9)
        self.assertTrue(all(self.state.commands['task-'+str(i)]['status'] == 'queued' for i in range(8)))
        seen = []
        self.state.drain(lambda kind, value: seen.append(kind), self.clock)
        self.state.drain(lambda kind, value: seen.append(kind), self.clock)
        self.assertEqual(seen, ['audio_test'] + ['command']*8)

    def test_audio_test_stop_requires_subscriber_but_not_voice_or_router_freshness(self):
        self.state.ros.update(router_ready=False, audio_test_ready=True)
        self.state.latest.clear()
        self.state.enqueue(request_id='stop', audio_test=False)
        seen = []
        self.state.drain(lambda kind, value: seen.append((kind, value['enabled'])), self.clock)
        self.assertEqual(seen, [('audio_test', False)])
        self.state.ros['audio_test_ready'] = False
        with self.assertRaisesRegex(module.ApiError, 'audio_test_subscriber_unavailable'):
            self.state.enqueue(request_id='stop-disconnected', audio_test=False)

    def test_audio_test_request_kinds_are_exclusive_and_ids_cannot_change_kind(self):
        self.state.ros['audio_test_ready'] = True
        self.state.update('voice', dict(stamp=self.clock(), microphone_enabled=True))
        for arguments in (dict(text='寻找火焰', audio_test=True), dict(listen=True, audio_test=True),
                          dict(audio_test=1), dict(audio_test='false'), dict()):
            with self.subTest(arguments=arguments), self.assertRaises(module.ApiError):
                self.state.enqueue(request_id='invalid', **arguments)
        first = self.state.enqueue(request_id='one', audio_test=True)
        repeated = self.state.enqueue(request_id='one', audio_test=True)
        self.assertEqual(first['command_id'], repeated['command_id'])
        self.assertEqual(len(self.state.pending), 1)
        for arguments in (dict(audio_test=False), dict(listen=True), dict(text='寻找火焰')):
            with self.assertRaisesRegex(module.ApiError, 'different_payload'):
                self.state.enqueue(request_id='one', **arguments)

    def test_audio_test_feedback_is_real_correlated_fresh_without_meter_event_flood(self):
        self.state.ros['audio_test_ready'] = True
        self.state.update('voice', dict(stamp=self.clock(), microphone_enabled=True))
        record = self.state.enqueue(request_id='test', audio_test=True)
        self.state.drain(lambda *args: None, self.clock)
        events_before = len(self.state.events)
        for i in range(50):
            payload = dict(stamp=self.clock(), capture_stamp=self.clock()-8,
                           command_id=record['command_id'], state='capturing', level_dbfs=-50+i/2)
            self.state.update('audio_test', payload)
        self.assertEqual(len(self.state.events), events_before)
        self.assertTrue(self.state.envelope('audio_test')['fresh'])
        self.assertEqual(self.state.commands['test']['status'], 'observed')
        self.assertEqual(self.state.commands['test']['audio_test_status'], payload)
        self.assertNotIn('receipt', self.state.commands['test'])
        self.clock.value += 4.01
        self.assertFalse(self.state.envelope('audio_test')['fresh'])
        self.assertFalse(self.state.update('audio_test', dict(synthetic=True, state='capturing')))

    def test_audio_test_pending_request_expires_without_auto_retry(self):
        self.state.ros['audio_test_ready'] = True
        self.state.update('voice', dict(stamp=self.clock(), microphone_enabled=True))
        self.state.enqueue(request_id='test', audio_test=True)
        self.clock.value += 3.01
        self.state.drain(lambda *args: self.fail('expired request must not record'), self.clock)
        self.assertEqual(self.state.commands['test']['status'], 'expired')

    def test_rejected_audio_request_does_not_reject_or_overwrite_running_session(self):
        self.state.ros['audio_test_ready'] = True
        self.state.update('voice', dict(stamp=self.clock(), microphone_enabled=True))
        first = self.state.enqueue(request_id='session', audio_test=True)
        self.state.drain(lambda *args: None, self.clock)
        progress = dict(stamp=self.clock(), state='listening', session_id=first['command_id'],
                        command_id=first['command_id'], request_id=first['command_id'], request_accepted=True)
        self.state.update('audio_test', progress)
        second = self.state.enqueue(request_id='busy-start', audio_test=True)
        self.state.drain(lambda *args: None, self.clock)
        rejection = dict(progress, request_id=second['command_id'], request_accepted=False,
                         request_rejected=True, request_reason='microphone_busy')
        self.state.update('audio_test', rejection)
        self.assertEqual(self.state.commands['busy-start']['status'], 'rejected')
        self.assertEqual(self.state.commands['busy-start']['reason'], 'microphone_busy')
        self.assertEqual(self.state.commands['session']['status'], 'observed')
        self.assertEqual(self.state.commands['session']['audio_test_status'], progress)
        self.assertNotIn('audio_test_session_id', self.state.commands['busy-start'])
        self.state.update('audio_test', dict(progress, state='completed', result_stamp=self.clock()))
        self.assertEqual(self.state.commands['session']['audio_test_status']['state'], 'completed')
        self.assertEqual(self.state.commands['busy-start']['status'], 'rejected')

    def test_audio_stop_receives_own_feedback_and_final_session_result(self):
        self.state.ros['audio_test_ready'] = True
        self.state.update('voice', dict(stamp=self.clock(), microphone_enabled=True))
        start = self.state.enqueue(request_id='start', audio_test=True)
        self.state.drain(lambda *args: None, self.clock)
        session_id = start['command_id']
        progress = dict(stamp=self.clock(), state='listening', session_id=session_id,
                        command_id=session_id, request_id=session_id, request_accepted=True)
        self.state.update('audio_test', progress)
        stop = self.state.enqueue(request_id='stop', audio_test=False)
        self.state.drain(lambda *args: None, self.clock)
        self.state.update('audio_test', dict(progress, request_id=stop['command_id'],
                                          request_action='stop', capture_active=True))
        self.assertEqual(self.state.commands['stop']['status'], 'observed')
        self.assertTrue(self.state.commands['stop']['audio_test_status']['capture_active'])
        # The correlated stop remains associated even if a later node status
        # names only its original session (or a second accepted stop).
        final = dict(progress, state='completed', capture_active=False,
                     result_stamp=self.clock(), text='今天进行语音测试')
        self.state.update('audio_test', final)
        for key in ('start', 'stop'):
            self.assertEqual(self.state.commands[key]['status'], 'observed')
            self.assertEqual(self.state.commands[key]['audio_test_status'], final)

    def test_duplicate_audio_request_rejection_preserves_previously_observed_result(self):
        self.state.ros['audio_test_ready'] = True
        self.state.update('voice', dict(stamp=self.clock(), microphone_enabled=True))
        start = self.state.enqueue(request_id='start', audio_test=True)
        self.state.drain(lambda *args: None, self.clock)
        result = dict(stamp=self.clock(), state='completed', session_id=start['command_id'],
                      command_id=start['command_id'], request_id=start['command_id'],
                      request_accepted=True, result_stamp=self.clock(), text='真实测试文字')
        self.state.update('audio_test', result)
        rejection = dict(result, request_accepted=False, request_rejected=True,
                         request_reason='duplicate_audio_test_request')
        self.state.update('audio_test', rejection)
        record = self.state.commands['start']
        self.assertEqual(record['status'], 'observed')
        self.assertEqual(record['audio_test_status'], result)
        self.assertEqual(record['audio_test_rejection'], rejection)

    def test_unknown_audio_rejection_does_not_attach_to_command_id_session(self):
        self.state.ros['audio_test_ready'] = True
        self.state.update('voice', dict(stamp=self.clock(), microphone_enabled=True))
        start = self.state.enqueue(request_id='start', audio_test=True)
        self.state.drain(lambda *args: None, self.clock)
        self.state.update('audio_test', dict(stamp=self.clock(), state='listening',
            command_id=start['command_id'], request_id='not-from-this-hmi',
            request_rejected=True, request_reason='expired_or_future_audio_test_request'))
        self.assertEqual(self.state.commands['start']['status'], 'submitted')
        self.assertNotIn('audio_test_status', self.state.commands['start'])

    def test_immediate_audio_rejection_is_not_overwritten_after_dispatch(self):
        self.state.ros['audio_test_ready'] = True
        self.state.update('voice', dict(stamp=self.clock(), microphone_enabled=True))
        self.state.enqueue(request_id='start', audio_test=True)
        def reject(kind, value):
            self.state.update('audio_test', dict(stamp=self.clock(), state='error',
                command_id=value['command_id'], request_id=value['command_id'],
                request_rejected=True, request_reason='microphone_busy'))
        self.state.drain(reject, self.clock)
        self.assertEqual(self.state.commands['start']['status'], 'rejected')
        self.assertEqual(self.state.commands['start']['reason'], 'microphone_busy')

    def test_emergency_stop_still_cancels_pending_audio_test_and_car_requests(self):
        self.state.ros['audio_test_ready'] = True
        self.state.update('voice', dict(stamp=self.clock(), microphone_enabled=True))
        self.state.enqueue(request_id='test', audio_test=True)
        self.state.enqueue('开始喷水', 'spray')
        self.state.enqueue('停止喷水', 'stop')
        self.assertEqual(self.state.commands['test']['reason'], 'superseded_by_stop')
        self.assertEqual(self.state.commands['spray']['reason'], 'superseded_by_stop')
        seen = []
        self.state.drain(lambda kind, value: seen.append((kind, value['action'])), self.clock)
        self.assertEqual(seen, [('command', 'stop_spray')])

    def test_failed_publish_is_not_retried(self):
        self.state.enqueue('寻找火焰', 'req1')
        def fail(*args):
            raise RuntimeError('unavailable')
        self.state.drain(fail, self.clock)
        self.assertEqual(self.state.commands['req1']['status'], 'rejected')
        self.state.drain(lambda *args: self.fail('retry'), self.clock)

    def test_bounded_events_invalid_json_values(self):
        self.assertFalse(self.state.update('scene', {'stamp': float('nan')}))
        self.assertFalse(self.state.update('scene', {'text': 'x' * 65536}))
        for i in range(200):
            self.state.update('speech', {'text': str(i)})
        self.assertEqual(len(self.state.events), 100)

    def test_disconnected_state_has_no_fabricated_data_or_receipt(self):
        state = module.PresentationState()
        snap = state.snapshot()
        self.assertEqual(snap['mode'], 'live')
        self.assertFalse(snap['synthetic'])
        self.assertFalse(snap['control']['requires_token'])
        self.assertFalse(snap['control']['command_ready'])
        self.assertTrue(all(v['data'] is None for v in snap['topics'].values()))
        self.assertEqual(snap['events'], [])
        self.assertEqual(snap['buildings'], [])
        with self.assertRaisesRegex(module.ApiError, 'ros_unavailable'):
            state.enqueue('寻找火焰', 'one')
        self.assertEqual(state.snapshot()['commands'], [])

    def test_synthetic_topics_are_rejected(self):
        self.assertFalse(self.state.update('mission', {'mode': 'fire', 'synthetic': True}))
        self.assertEqual(self.state.envelope('mission')['data']['mode'], 'standby')

    def test_real_navigation_requires_live_uart_at_enqueue_and_dispatch(self):
        self.state.buildings = {'buildings': [{'building_id': 'A', 'number': '01', 'name': '安居楼'}]}
        with self.assertRaisesRegex(module.ApiError, 'real_fpga_feedback_required'):
            self.state.enqueue('去1号楼', 'one')
        self.state.update('fpga_link', dict(dry_run=True, status_fresh=True))
        with self.assertRaisesRegex(module.ApiError, 'real_uart_not_enabled'):
            self.state.enqueue('去1号楼', 'one')
        self.state.update('fpga_link', dict(dry_run=False, status_fresh=True))
        self.state.enqueue('去1号楼', 'one')
        self.state.update('fpga_link', dict(dry_run=False, status_fresh=False))
        self.state.drain(lambda *args: self.fail('link lost before dispatch'), self.clock)
        self.assertEqual(self.state.commands['one']['reason'], 'real_fpga_feedback_required')
        self.assertEqual(self.state.snapshot()['buildings'], [{'id': 'A', 'number': '01', 'name': '安居楼'}])

    def test_live_fixed_camera_evidence_does_not_unlock_navigation_or_pump(self):
        self.state.buildings = {'buildings': [{'building_id': 'A', 'number': '01'}]}
        self.state.update('actuators', dict(stamp=self.clock(), dry_run=True, enabled=False))
        self.state.update('road_preview', dict(stamp=self.clock(), detected=True,
            observation_only=True, control_eligible=False))
        self.state.update('traffic_preview', dict(stamp=self.clock(), status='GREEN',
            observation_only=True, control_eligible=False))
        snapshot = self.state.snapshot()
        self.assertTrue(snapshot['topics']['traffic_preview']['fresh'])
        self.assertIsNone(snapshot['topics']['traffic_observation']['data'])
        self.assertIsNone(snapshot['topics']['motion_permission']['data'])
        with self.assertRaisesRegex(module.ApiError, 'real_fpga_feedback_required'):
            self.state.enqueue('去1号楼', 'fixed-camera-nav')
        with self.assertRaisesRegex(module.ApiError, 'real_actuator_not_enabled'):
            self.state.enqueue('开始喷水', 'fixed-camera-pump')

    def test_stop_during_drain_cancels_all_not_yet_published_commands(self):
        self.state.enqueue('寻找火焰', 'inspect')
        self.state.enqueue('开始自动灭火', 'auto')
        actions = []
        def publish(kind, value):
            actions.append(value['action'])
            if value['action'] == 'inspect':
                self.state.enqueue('停止喷水', 'stop')
        self.state.drain(publish, self.clock)
        self.assertEqual(actions, ['inspect', 'stop_spray'])
        self.assertEqual(self.state.commands['auto']['status'], 'rejected')

    def test_bare_navigation_publishes_manual_policy_even_after_auto_mission(self):
        self.state.buildings = {'buildings': [{'building_id': 'A', 'number': '01'}]}
        self.state.update('mission', dict(stamp=self.clock(), mode='inspect', arrival_policy='auto'))
        self.state.update('fpga_link', dict(dry_run=False, status_fresh=True))
        self.state.update('actuators', dict(stamp=self.clock(), dry_run=True))
        self.state.enqueue('去1号楼', 'one')
        seen = []
        self.state.drain(lambda kind, value: seen.append(value), self.clock)
        self.assertEqual(seen[0]['arrival_policy'], 'manual')

    def test_real_pump_required_but_stop_always_available(self):
        self.state.update('actuators', dict(stamp=self.clock(), dry_run=True, enabled=True, pump_enabled=True))
        for text in ('开始喷水', '开始自动灭火', '到达后自动灭火'):
            with self.subTest(text=text):
                with self.assertRaisesRegex(module.ApiError, 'real_actuator_not_enabled'):
                    self.state.enqueue(text, 'one')
        self.assertEqual(self.state.enqueue('停止喷水', 'stop')['status'], 'queued')
        self.state.update('actuators', dict(stamp=self.clock(), dry_run=False, enabled=True, pump_enabled=False))
        with self.assertRaisesRegex(module.ApiError, 'real_pump_not_enabled'):
            self.state.enqueue('开始喷水', 'one')
        self.state.update('actuators', dict(stamp=self.clock(), dry_run=False, enabled=True, pump_enabled=True))
        self.state.enqueue('开始喷水', 'one')
        self.state.update('actuators', dict(stamp=self.clock(), dry_run=True, enabled=True, pump_enabled=True))
        self.state.drain(lambda kind, data: self.assertEqual(data['action'], 'stop_spray'), self.clock)
        self.assertEqual(self.state.commands['one']['status'], 'rejected')

    def test_removed_demo_flag_is_rejected_before_startup(self):
        with patch.object(sys, 'argv', ['presentation_server.py', '--demo']), \
                patch('sys.stderr', new_callable=io.StringIO):
            with self.assertRaises(SystemExit) as caught:
                module.main()
            self.assertEqual(caught.exception.code, 2)


class MotorTestStateTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.state = module.PresentationState(monotonic=self.clock, wall=self.clock)
        self.state.ros.update(available=True, motor_test_ready=True)
        self.refresh()

    def refresh(self):
        self.state.update('mission', dict(stamp=self.clock(), mode='inspect', traffic_active=False,
            auto_fire_active=False, emergency_stop=False, pump_armed=False, motion_allowed=False))
        self.state.update('actuators', dict(stamp=self.clock(), dry_run=True, enabled=False,
            hardware_written=False, pump_on=False, pump_enabled=False, estop=False))
        self.state.update('fpga_link', dict(stamp=self.clock(), dry_run=True))

    def test_motor_uses_own_node_and_exact_bounded_wire_schema_without_router(self):
        record = self.state.enqueue(request_id='one', motor_test='connect')
        self.assertEqual(record['status'], 'queued')
        self.assertTrue(self.state.snapshot()['control']['motor_test_ready'])
        self.assertFalse(self.state.snapshot()['control']['command_ready'])
        calls = []
        self.state.drain(lambda kind, data: calls.append((kind, data)), self.clock)
        self.assertEqual(calls, [('motor_test', dict(action='connect', request_id=record['command_id'],
            issued_at=self.clock(), ttl_s=3., motor_id='motor1'))])
        self.assertEqual(self.state.commands['one']['status'], 'submitted')
        self.assertNotIn('motor_test_status', self.state.commands['one'])
        self.assertEqual(self.state.enqueue(request_id='one', motor_test='connect')['command_id'], record['command_id'])
        self.state.drain(lambda *args: self.fail('duplicate must not republish'), self.clock)
        with self.assertRaisesRegex(module.ApiError, 'different_payload'):
            self.state.enqueue(request_id='one', motor_test='step_positive')

    def test_motor_readiness_rechecks_freshness_and_real_uart_ownership(self):
        self.state.enqueue(request_id='one', motor_test='enable')
        self.state.update('fpga_link', dict(stamp=self.clock(), dry_run=False))
        self.state.drain(lambda *args: self.fail('real FPGA owns the UART'), self.clock)
        self.assertEqual(self.state.commands['one']['status'], 'rejected')
        self.refresh()
        self.clock.value += 2
        with self.assertRaisesRegex(module.ApiError, 'motor_test_system_status_stale'):
            self.state.enqueue(request_id='stale', motor_test='step_positive')
        self.refresh()
        self.state.update('actuators', dict(stamp=self.clock(), dry_run=False, enabled=True))
        with self.assertRaises(module.ApiError):
            self.state.enqueue(request_id='production', motor_test='connect')

    def test_stop_and_disconnect_work_without_mission_but_need_node(self):
        self.state.latest.clear()
        for action in ('stop', 'disconnect'):
            self.state.enqueue(request_id=action, motor_test=action)
            calls=[]
            self.state.drain(lambda kind,data: calls.append((kind,data['action'])), self.clock)
            self.assertEqual(calls, [('motor_test',action)])
        self.state.ros['motor_test_ready'] = False
        with self.assertRaisesRegex(module.ApiError,'motor_test_subscriber_unavailable'):
            self.state.enqueue(request_id='no-node', motor_test='stop')

    def test_motor_stop_cancels_only_motor_requests_and_reserves_its_own_slot(self):
        self.state.ros.update(router_ready=True, audio_test_ready=True)
        self.state.enqueue(request_id='move', motor_test='step_positive')
        for n in range(7):self.state.enqueue('报告当前场景', 'task'+str(n))
        self.state.enqueue(request_id='audio-stop', audio_test=False)
        self.state.enqueue(request_id='motor-stop', motor_test='stop')
        self.assertEqual(self.state.commands['move']['reason'],'superseded_by_motor_test_stop')
        self.assertTrue(all(self.state.commands['task'+str(n)]['status']=='queued' for n in range(7)))
        self.assertEqual(len(self.state.pending),9)
        calls=[]
        self.state.drain(lambda kind,data: calls.append(kind),self.clock)
        self.state.drain(lambda kind,data: calls.append(kind),self.clock)
        self.assertEqual(calls[:2],['motor_test','audio_test'])
        self.assertEqual(calls.count('command'),7)

    def test_feedback_is_correlated_fresh_and_not_completion_from_acceptance(self):
        record=self.state.enqueue(request_id='move',motor_test='step_positive')
        self.state.drain(lambda *args:None,self.clock)
        feedback=dict(stamp=self.clock(),last_request_id=record['command_id'],
            last_action='step_positive',last_result='accepted',reason='position_acknowledged')
        self.state.update('motor_test',dict(feedback,last_request_id='other'))
        self.assertEqual(self.state.commands['move']['status'],'submitted')
        self.state.update('motor_test',dict(feedback,stamp=self.clock()-2))
        self.assertEqual(self.state.commands['move']['status'],'submitted')
        self.state.update('motor_test',feedback)
        self.assertEqual(self.state.commands['move']['status'],'observed')
        self.assertEqual(self.state.commands['move']['motor_test_status']['last_result'],'accepted')
        self.state.update('motor_test',dict(feedback,last_result='completed',reason='measured_arrival'))
        self.state.update('motor_test',feedback)
        self.assertEqual(self.state.commands['move']['motor_test_status']['last_result'],'completed')

    def test_both_diagnostic_stops_fit_a_full_queue_without_silent_deque_eviction(self):
        self.state.ros.update(router_ready=True,audio_test_ready=True)
        for n in range(8):self.state.enqueue('报告当前场景','task'+str(n))
        self.state.enqueue(request_id='audio-stop',audio_test=False)
        self.state.enqueue(request_id='motor-stop',motor_test='stop')
        self.assertEqual(len(self.state.pending),10)
        self.assertEqual({item[0] for item in self.state.pending},
                         {'task'+str(n) for n in range(8)}|{'audio-stop','motor-stop'})
        self.state.enqueue(request_id='motor-disconnect',motor_test='disconnect')
        self.assertEqual(len(self.state.pending),10)
        self.assertEqual(self.state.commands['motor-stop']['status'],'rejected')
        calls=[]
        self.state.drain(lambda kind,data:calls.append(kind),self.clock)
        self.state.drain(lambda kind,data:calls.append(kind),self.clock)
        self.assertEqual(calls,['motor_test','audio_test']+['command']*8)

    def test_motor_result_ledger_preserves_concurrent_rejection_and_terminal_completion(self):
        first=self.state.enqueue(request_id='move',motor_test='step_positive')
        second=self.state.enqueue(request_id='read',motor_test='read')
        self.state.drain(lambda *args:None,self.clock)
        events=[dict(request_id=first['command_id'],action='step_positive',result='accepted',
                     reason='queued',stamp=self.clock()),
                dict(request_id=second['command_id'],action='read',result='rejected',
                     reason='motor_test_worker_busy',stamp=self.clock()),
                dict(request_id=first['command_id'],action='step_positive',result='completed',
                     reason='measured_arrival',stamp=self.clock())]
        # Last result names the completed move, but the rejected read must not disappear.
        self.state.update('motor_test',dict(stamp=self.clock(),last_request_id=first['command_id'],
            last_action='step_positive',last_result='completed',recent_requests=events))
        self.assertEqual(self.state.commands['move']['motor_test_status']['last_result'],'completed')
        self.assertEqual(self.state.commands['read']['status'],'rejected')
        self.assertEqual(self.state.commands['read']['reason'],'motor_test_worker_busy')
        self.state.update('motor_test',dict(stamp=self.clock(),recent_requests=events[:1]))
        self.assertEqual(self.state.commands['move']['motor_test_status']['last_result'],'completed')

    def test_motor_ledger_ignores_stale_future_mismatched_or_malformed_events(self):
        record=self.state.enqueue(request_id='read',motor_test='read')
        self.state.drain(lambda *args:None,self.clock)
        event=dict(request_id=record['command_id'],action='read',result='completed',stamp=self.clock())
        for item in (dict(event,stamp=self.clock()-2),dict(event,stamp=self.clock()+2),
                     dict(event,action='enable'),dict(event,request_id='other'),
                     dict(event,stamp=None),dict(event,stamp=float('inf')),None):
            self.state.update('motor_test',dict(stamp=self.clock(),recent_requests=[item]))
            self.assertEqual(self.state.commands['read']['status'],'submitted')
        # A fresh summary must not revalidate the old ledger entry it describes.
        self.state.update('motor_test',dict(stamp=self.clock(),last_request_id=record['command_id'],
            last_action='read',last_result='completed',recent_requests=[dict(event,stamp=self.clock()-2)]))
        self.assertEqual(self.state.commands['read']['status'],'submitted')

    def test_immediate_rejection_is_not_overwritten_and_lost_receipt_never_retries(self):
        self.state.enqueue(request_id='move',motor_test='step_positive')
        def reject(kind,data):
            self.state.update('motor_test',dict(stamp=self.clock(),last_request_id=data['request_id'],
                last_action=data['action'],last_result='rejected',reason='not_enabled'))
        self.state.drain(reject,self.clock)
        self.assertEqual(self.state.commands['move']['status'],'rejected')
        self.state.enqueue(request_id='read',motor_test='read')
        self.state.drain(lambda *args:None,self.clock)
        self.clock.value+=6
        self.state.snapshot()
        self.assertEqual(self.state.commands['read']['status'],'receipt_timeout')
        self.state.drain(lambda *args:self.fail('timeout cannot retry'),self.clock)

    def test_expired_motor_requests_are_not_published(self):
        self.state.enqueue(request_id='move',motor_test='step_negative')
        self.clock.value+=3.01
        self.refresh()
        self.state.drain(lambda *args:self.fail('expired movement request'),self.clock)
        self.assertEqual(self.state.commands['move']['status'],'expired')

    def test_motor2_is_explicit_on_wire_and_id_cannot_change_target(self):
        record=self.state.enqueue(request_id='m2',motor_test='connect',motor_id='motor2')
        self.assertEqual(record['motor_id'],'motor2')
        calls=[]
        self.state.drain(lambda kind,data:calls.append((kind,data)),self.clock)
        self.assertEqual(calls,[('motor_test',dict(action='connect',request_id=record['command_id'],
            issued_at=self.clock(),ttl_s=3.,motor_id='motor2'))])
        self.assertEqual(self.state.enqueue(request_id='m2',motor_test='connect',motor_id='motor2')['command_id'],
                         record['command_id'])
        with self.assertRaisesRegex(module.ApiError,'different_payload'):
            self.state.enqueue(request_id='m2',motor_test='connect',motor_id='motor1')
        for invalid in ('motor3','/dev/ttyS0',True,2,{}):
            with self.subTest(motor_id=invalid),self.assertRaisesRegex(module.ApiError,'unsupported_motor_id'):
                self.state.enqueue(request_id='bad',motor_test='connect',motor_id=invalid)

    def test_stop_cancels_only_requested_motor_queue_and_global_stop_cancels_both(self):
        self.state.ros['router_ready']=True
        self.state.enqueue(request_id='m1',motor_test='step_positive',motor_id='motor1')
        self.state.enqueue(request_id='m2',motor_test='step_negative',motor_id='motor2')
        self.state.enqueue(request_id='m2stop',motor_test='stop',motor_id='motor2')
        self.assertEqual(self.state.commands['m2']['status'],'rejected')
        self.assertEqual(self.state.commands['m1']['status'],'queued')
        self.assertEqual([item[0] for item in self.state.pending],['m2stop','m1'])
        self.state.enqueue('停止喷水','all-stop')
        self.assertEqual(self.state.commands['m1']['status'],'rejected')
        self.assertEqual(self.state.commands['m2stop']['status'],'rejected')
        self.assertEqual([item[0] for item in self.state.pending],['all-stop'])

    def test_each_motor_has_stop_queue_slot_alongside_audio_and_eight_tasks(self):
        self.state.ros.update(router_ready=True,audio_test_ready=True)
        for n in range(8):self.state.enqueue('报告当前场景','task'+str(n))
        self.state.enqueue(request_id='audio-stop',audio_test=False)
        self.state.enqueue(request_id='m1stop',motor_test='stop',motor_id='motor1')
        self.state.enqueue(request_id='m2stop',motor_test='stop',motor_id='motor2')
        self.assertEqual(len(self.state.pending),11)
        self.assertTrue(all(self.state.commands['task'+str(n)]['status']=='queued' for n in range(8)))
        self.state.enqueue(request_id='m2disconnect',motor_test='disconnect',motor_id='motor2')
        self.assertEqual(len(self.state.pending),11)
        self.assertEqual(self.state.commands['m1stop']['status'],'queued')
        self.assertEqual(self.state.commands['m2stop']['status'],'rejected')

    def test_motor_feedback_matches_request_motor_not_current_connected_motor(self):
        record=self.state.enqueue(request_id='m2',motor_test='read',motor_id='motor2')
        self.state.drain(lambda *args:None,self.clock)
        result=dict(stamp=self.clock(),last_request_id=record['command_id'],last_action='read',
                    last_result='completed')
        # An old single-motor or wrong-motor node cannot complete motor2's request.
        for data in (result,dict(result,motor_id='motor1'),dict(result,motor_id='motor2',last_motor_id='motor1')):
            self.state.update('motor_test',data)
            self.assertEqual(self.state.commands['m2']['status'],'submitted')
        self.state.update('motor_test',dict(result,motor_id='motor1',last_motor_id='motor2',
            last_result='rejected',reason='motor_test_different_motor_connected'))
        self.assertEqual(self.state.commands['m2']['status'],'rejected')
        self.assertEqual(self.state.commands['m2']['motor_test_status']['motor_id'],'motor2')

    def test_motor_ledger_identity_cannot_complete_other_motor(self):
        record=self.state.enqueue(request_id='m2',motor_test='read',motor_id='motor2')
        self.state.drain(lambda *args:None,self.clock)
        event=dict(stamp=self.clock(),request_id=record['command_id'],action='read',result='completed')
        for item in (event,dict(event,motor_id='motor1'),dict(event,motor_id=None)):
            self.state.update('motor_test',dict(stamp=self.clock(),motor_id='motor2',recent_requests=[item]))
            self.assertEqual(self.state.commands['m2']['status'],'submitted')
        self.state.update('motor_test',dict(stamp=self.clock(),motor_id='motor1',
            recent_requests=[dict(event,motor_id='motor2')]))
        self.assertEqual(self.state.commands['m2']['status'],'observed')
        self.assertEqual(self.state.commands['m2']['motor_test_status']['motor_id'],'motor2')


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        for name in ('presentation.html', 'presentation.css', 'presentation.js'):
            (root / name).write_text('fixture', encoding='utf-8')
        self.state = module.PresentationState()
        self.server = module.BoundedServer(('127.0.0.1', 0), BaseHTTPRequestHandler)
        self.port = self.server.server_address[1]
        self.host = '127.0.0.1:' + str(self.port)
        self.proxy = Mock()
        self.server.RequestHandlerClass = module.handler_class(self.state, root, self.proxy, {self.host})
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
        self.temp.cleanup()

    def request(self, method, path, payload=None, headers=None):
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=3)
        body = json.dumps(payload).encode() if payload is not None else None
        default = {'Host': self.host, 'Content-Type': 'application/json', 'Origin': 'http://' + self.host}
        default.update(headers or {})
        conn.request(method, path, body, default)
        response = conn.getresponse()
        status, response_headers, data = response.status, dict(response.getheaders()), response.read()
        conn.close()
        return status, response_headers, data

    def test_region_observation_http_is_separate_from_automatic_extinguishing(self):
        now = self.state.wall()
        self.state.ros.update(available=True, router_ready=True)
        self.state.update('mission', dict(stamp=now, mode='inspect'))
        self.state.update('extension', dict(stamp=now, region_observe_ready=True, region_ready=False,
            region_reason='region_real_gimbal_reference_required'))
        self.state.update('actuators', dict(stamp=now, dry_run=False, enabled=False, pump_enabled=False))
        status, _, body = self.request('POST', '/api/command',
            dict(text='去C区域搜索火焰', request_id='observe-c'))
        self.assertEqual(status, 202, body)
        sent = []
        self.state.drain(lambda kind, message: sent.append(message), self.state.wall)
        self.assertEqual(len(sent), 1)
        self.assertIs(sent[0]['execute_fire'], False)
        self.assertEqual(sent[0]['region_id'], 'C')
        status, _, body = self.request('POST', '/api/command',
            dict(text='在C区域搜寻火源并执行灭火', request_id='extinguish-c'))
        self.assertEqual(status, 409)
        self.assertEqual(json.loads(body)['error'], 'region_real_gimbal_reference_required')
        self.assertIs(json.loads(body)['synthetic'], False)
        self.assertEqual(len(self.state.pending), 0)

    def test_direct_control_still_enforces_origin_and_rebinding_boundaries(self):
        for headers, expected in (({'Origin': 'http://evil.invalid'}, 403),
                                  ({'Origin': ''}, 403), ({'Host': 'evil.invalid'}, 403),
                                  ({'Sec-Fetch-Site': 'cross-site'}, 403)):
            with self.subTest(headers=headers):
                status, _, _ = self.request('POST', '/api/session', {}, headers)
                self.assertEqual(status, expected)
        self.assertEqual(self.request('POST', '/api/session', {})[0], 200)

    def test_motor_heartbeat_has_exact_schema_same_origin_and_no_motion_endpoint(self):
        payload = {'motor_id': 'motor2', 'run_id': 'presentation-test'}
        with patch.object(self.state, 'motor_heartbeat', return_value={'queued': True}) as heartbeat:
            self.assertEqual(self.request('POST', '/api/motor-heartbeat', payload)[0], 202)
            heartbeat.assert_called_once_with(**payload)
            for invalid in ({}, dict(payload, action='run_positive'), {'motor_id': 'motor2'}):
                self.assertEqual(self.request('POST', '/api/motor-heartbeat', invalid)[0], 400)
            self.assertEqual(self.request('POST', '/api/motor-heartbeat', payload,
                                         {'Origin': 'http://evil.invalid'})[0], 403)
            heartbeat.assert_called_once()

    def test_socket_local_address_accepts_new_literal_without_startup_discovery(self):
        # Deliberately omit the actual listening address from the static set,
        # equivalent to Ethernet receiving its address after service startup.
        self.server.RequestHandlerClass = module.handler_class(
            self.state, self.temp.name, self.proxy, set(), allow_socket_address=True)
        self.assertEqual(self.request('GET', '/api/state')[0], 200)
        self.assertEqual(self.request('POST', '/api/session', {})[0], 200)
        self.server.RequestHandlerClass = module.handler_class(
            self.state, self.temp.name, self.proxy, set(), allow_socket_address=False)
        self.assertEqual(self.request('GET', '/api/state')[0], 403)

    def test_socket_address_does_not_authorize_other_hosts_ports_or_origins(self):
        self.server.RequestHandlerClass = module.handler_class(
            self.state, self.temp.name, self.proxy, set(), allow_socket_address=True)
        bad_hosts = ('evil.invalid:'+str(self.port), 'localhost:'+str(self.port),
            '127.0.0.2:'+str(self.port), '127.0.0.1:'+str(self.port+1),
            '127.0.0.1', '127.000.000.001:'+str(self.port),
            '2130706433:'+str(self.port), 'user@'+self.host, self.host+'.evil.invalid')
        for host in bad_hosts:
            with self.subTest(host=host):
                self.assertEqual(self.request('POST','/api/session',{},
                    {'Host':host,'Origin':'http://'+host})[0],403)
        for headers in ({'Origin':'http://evil.invalid'}, {'Origin':'null'},
                        {'Origin':'http://127.0.0.1:'+str(self.port+1)},
                        {'Sec-Fetch-Site':'cross-site'}):
            self.assertEqual(self.request('POST','/api/session',{},headers)[0],403)

    def test_read_state_does_not_touch_preview_and_static_allowlist(self):
        status, headers, data = self.request('GET', '/api/state')
        self.assertEqual(status, 200)
        self.assertFalse(json.loads(data)['ros']['available'])
        self.assertEqual(headers['Cache-Control'], 'no-store')
        self.assertEqual(headers['X-Frame-Options'], 'DENY')
        for path in ('/../config/voice.json', '/control.token', '/tools/presentation_server.py'):
            self.assertEqual(self.request('GET', path)[0], 404)

    def test_no_command_field_smuggling_and_no_live_demo(self):
        self.assertEqual(self.request('POST', '/api/command',
            {'text': '寻找火焰', 'request_id': 'one', 'action': 'start_spray'})[0], 400)
        self.assertEqual(self.request('POST', '/api/demo', {'scenario': 'fire'})[0], 404)
        self.assertEqual(self.request('POST', '/api/stream', {'enabled': 'yes'})[0], 400)
        self.assertEqual(self.request('POST', '/api/session', {'value': 'x' * 3000})[0], 413)
        self.assertEqual(len(self.state.pending), 0)

    def test_http_command_only_queues_real_receipt_later(self):
        self.state.ros.update(available=True, router_ready=True)
        self.state.update('mission', {'stamp': module.time.time(), 'mode': 'inspect'})
        status, _, data = self.request('POST', '/api/command', {'text': '寻找火焰', 'request_id': 'one'})
        self.assertEqual(status, 202)
        self.assertEqual(json.loads(data)['status'], 'queued')
        self.assertNotIn('accepted', json.loads(data))
        self.assertEqual(len(self.state.pending), 1)

    def test_audio_test_http_auth_schema_and_queue_only(self):
        self.state.ros.update(available=True, router_ready=False, audio_test_ready=True)
        self.state.update('voice', dict(stamp=module.time.time(), microphone_enabled=True))
        payload = dict(enabled=True, request_id='audio')
        for headers, expected in (({'Origin': 'http://evil.invalid'}, 403),
                                  ({'Sec-Fetch-Site': 'cross-site'}, 403)):
            self.assertEqual(self.request('POST', '/api/audio-test', payload, headers)[0], expected)
        for invalid in (dict(payload, text='开始喷水'), dict(payload, enabled=None),
                        dict(payload, enabled=1), dict(payload, enabled='true'), {'enabled': True}):
            self.assertEqual(self.request('POST', '/api/audio-test', invalid)[0], 400)
        self.assertEqual(len(self.state.pending), 0)
        code, _, raw = self.request('POST', '/api/audio-test', payload)
        data = json.loads(raw)
        self.assertEqual(code, 202)
        self.assertEqual(data['kind'], 'audio_test')
        self.assertEqual(data['status'], 'queued')
        self.assertNotIn('accepted', data)
        self.assertNotIn('audio_test_status', data)
        self.assertEqual(len(self.state.pending), 1)

    def test_http_listen_end_before_start_preserves_stop_with_no_fake_ack(self):
        self.state.ros.update(available=True, listen_ready=True)
        self.state.update('voice', dict(stamp=module.time.time(), microphone_enabled=True))
        payload = dict(enabled=False, request_id='end', cancel_start_request_id='start')
        code, _, raw = self.request('POST', '/api/listen', payload)
        self.assertEqual(code, 202)
        self.assertEqual(json.loads(raw)['status'], 'queued')
        self.assertNotIn('receipt', json.loads(raw))
        self.assertEqual(self.request('POST', '/api/listen', payload)[0], 202)
        self.assertEqual(self.request('POST', '/api/listen', dict(enabled=True, request_id='start'))[0], 409)
        self.assertEqual(len(self.state.pending), 1)
        for bad in (dict(enabled=True, request_id='invalid', cancel_start_request_id='start'),
                    dict(payload, cancel_start_request_id=3), dict(payload, extra=True)):
            self.assertEqual(self.request('POST', '/api/listen', bad)[0], 400)

    def test_road_frame_get_is_readonly_and_head_or_cross_site_cannot_start_encoding(self):
        self.state.preview['enabled'] = True
        cache = self.state.road_visualization
        self.assertFalse(cache.wanted())
        self.assertEqual(self.request('HEAD', '/api/road-frame')[0], 200)
        self.assertFalse(cache.wanted())
        self.assertEqual(self.request('GET', '/api/road-frame', headers={'Sec-Fetch-Site': 'cross-site'})[0], 403)
        self.assertFalse(cache.wanted())
        status, headers, packet = self.request('GET', '/api/road-frame', headers={'Authorization': ''})
        self.assertEqual(status, 200)
        self.assertEqual(headers['Cache-Control'], 'no-store')
        count = struct.unpack('!I', packet[:4])[0]
        metadata = json.loads(packet[4:4+count])
        self.assertEqual(metadata['state'], 'waiting')
        self.assertTrue(cache.wanted())
        self.assertEqual(len(self.state.pending), 0)
        self.assertEqual(packet[4+count:], b'')
        self.assertNotIn('jpeg_base64', json.dumps(self.state.snapshot()))
        self.proxy.frame.assert_not_called()
        self.state.preview['enabled'] = False
        _, _, packet = self.request('GET', '/api/road-frame')
        count = struct.unpack('!I', packet[:4])[0]
        self.assertEqual(json.loads(packet[4:4+count])['state'], 'paused')
        self.assertFalse(cache.wanted())

    def test_motor_http_strict_actions_and_origin_without_control_token(self):
        self.state.ros.update(available=True,motor_test_ready=True)
        now=module.time.time()
        self.state.update('mission',dict(stamp=now,mode='inspect'))
        self.state.update('actuators',dict(stamp=now,dry_run=True,enabled=False))
        self.state.update('fpga_link',dict(stamp=now,dry_run=True))
        for payload in ({'action':'home','request_id':'bad'}, {'action':'enable'},
                        {'action':True,'request_id':'bad'}, {'action':None,'request_id':'bad'},
                        {'action':'step_positive','request_id':'bad','degrees':90},
                        {'action':'connect','request_id':'bad','port':'/dev/ttyS0'}):
            with self.subTest(payload=payload):
                self.assertEqual(self.request('POST','/api/motor-test',payload)[0],400)
        valid={'action':'connect','request_id':'one'}
        self.assertEqual(self.request('POST','/api/motor-test',valid,{'Origin':'http://evil.invalid'})[0],403)
        code,_,raw=self.request('POST','/api/motor-test',valid)
        data=json.loads(raw)
        self.assertEqual(code,202);self.assertEqual(data['kind'],'motor_test')
        self.assertEqual(data['status'],'queued');self.assertNotIn('motor_test_status',data)
        self.assertEqual(len(self.state.pending),1)

    def test_motor_http_accepts_fixed_motor2_or_legacy_motor1_but_no_port_or_unknown_id(self):
        self.state.ros.update(available=True,motor_test_ready=True)
        now=module.time.time()
        self.state.update('mission',dict(stamp=now,mode='inspect'))
        self.state.update('actuators',dict(stamp=now,dry_run=True,enabled=False))
        self.state.update('fpga_link',dict(stamp=now,dry_run=True))
        for invalid in (None,True,2,'motor3','/dev/ttyS0',{},[]):
            with self.subTest(invalid=invalid):
                self.assertEqual(self.request('POST','/api/motor-test',
                    dict(action='connect',request_id='bad',motor_id=invalid))[0],400)
        self.assertEqual(self.request('POST','/api/motor-test',
            dict(action='connect',request_id='bad',motor_id='motor2',port='/dev/ttyS3'))[0],400)
        for payload,expected in ((dict(action='connect',request_id='m1'),'motor1'),
                                 (dict(action='connect',request_id='m2',motor_id='motor2'),'motor2')):
            code,_,raw=self.request('POST','/api/motor-test',payload)
            self.assertEqual(code,202)
            self.assertEqual(json.loads(raw)['motor_id'],expected)
        self.assertEqual(self.request('POST','/api/motor-test',
            dict(action='connect',request_id='m2',motor_id='motor1'))[0],409)

    def test_frame_only_forwards_upstream_bytes_without_fixture_fallback(self):
        metadata = module.compact(dict(transport_version=1, state='live', frame={'token': '1'}))
        packet = struct.pack('>I', len(metadata)) + metadata + b'JPEG_AS_RECEIVED'
        self.proxy.frame.return_value = packet
        status, _, data = self.request('GET', '/api/frame?after=old')
        self.assertEqual(status, 200)
        self.assertEqual(data, packet)
        self.proxy.frame.assert_called_once_with('old')
        self.proxy.frame.side_effect = module.ApiError(503, 'preview_unavailable')
        status, _, data = self.request('GET', '/api/frame')
        self.assertEqual(status, 503)
        self.assertEqual(json.loads(data)['error'], 'preview_unavailable')
        self.assertEqual(self.request('GET', '/presentation-demo.jpg')[0], 404)
        self.proxy.stream.return_value = {'stream': {'enabled': True}, 'state': 'waiting'}
        self.assertEqual(self.request('POST', '/api/stream', {'enabled': True})[0], 200)
        self.proxy.stream.assert_called_once_with(True)


class ProxyTests(unittest.TestCase):
    def test_preview_exact_bytes_persistent_connection_and_metadata_no_lease(self):
        requests, client_ports = [], []
        rawmeta = module.compact({'transport_version': 1, 'frame': {'token': '123'}, 'state': 'live'})
        packet = struct.pack('>I', len(rawmeta)) + rawmeta + b'\xff\xd8RAW_NO_REENCODE\xff\xd9'
        class Upstream(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'
            def log_message(self, *args): pass
            def do_GET(self):
                requests.append(self.path)
                client_ports.append(self.client_address[1])
                data = b'{"stream":{"enabled":true},"state":"live"}' if 'state' in self.path else packet
                self.send_response(200)
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
        server = ThreadingHTTPServer(('127.0.0.1', 0), Upstream)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        proxy = module.PreviewProxy(server.server_address[1])
        try:
            self.assertTrue(proxy.metadata()['stream']['enabled'])
            self.assertEqual(proxy.frame('a:b c'), packet)
            self.assertEqual(proxy.frame('last'), packet)
            self.assertEqual(requests, ['/api/state?images=0', '/api/frame?after=a%3Ab%20c', '/api/frame?after=last'])
            self.assertEqual(len(set(client_ports)), 1)
        finally:
            proxy.close()
            server.shutdown()
            server.server_close()
            thread.join()

    def test_synthetic_upstream_is_rejected(self):
        proxy = module.PreviewProxy()
        try:
            metadata = module.compact(dict(transport_version=1, synthetic=True, state='live'))
            with patch.object(proxy, 'request', return_value=struct.pack('>I', len(metadata)) + metadata):
                with self.assertRaisesRegex(module.ApiError, 'real_preview_required'):
                    proxy.frame('')
            with patch.object(proxy, 'request', return_value=metadata):
                with self.assertRaisesRegex(module.ApiError, 'real_preview_required'):
                    proxy.metadata()
            for fields in ({'mode': 'demo'}, {'mode': 'recorded_demo'}, {'frame': {'synthetic': True}}):
                metadata = module.compact(dict(transport_version=1, **fields))
                with patch.object(proxy, 'request', return_value=struct.pack('>I', len(metadata)) + metadata):
                    with self.assertRaisesRegex(module.ApiError, 'real_preview_required'):
                        proxy.frame('')
        finally:
            proxy.close()

    def test_invalid_packet_rejected(self):
        proxy = module.PreviewProxy()
        try:
            for packet in (b'x', struct.pack('>I', 70000), struct.pack('>I', 2) + b'{}',
                           struct.pack('>I', 5) + b'notjs'):
                with patch.object(proxy, 'request', return_value=packet):
                    with self.assertRaises(module.ApiError):
                        proxy.frame('')
        finally:
            proxy.close()


if __name__ == '__main__':
    unittest.main()
