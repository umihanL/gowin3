import unittest
from rescue_vision.task_interaction import TaskRouter, parse_task_text


class TaskInteractionTest(unittest.TestCase):
    def setUp(self):
        self.router = TaskRouter({'buildings': [{'building_id': 'A', 'number': '1'}]})
        self.now = 100.
        self.identity = dict(target_id=2, selection_generation=3, registry_session='session')
        self.state = dict(mission=dict(stamp=100., mode='fire', vehicle_stopped=True, gimbal_ready=True),
            proposal=dict(capture_stamp=100., valid=True, pump_candidate=True, **self.identity),
            selection=dict(capture_stamp=100., valid=True, **self.identity),
            actuator=dict(stamp=100., estop=False, guard=dict(state='DISARMED', armed=False)))

    def request(self, action, **kwargs):
        return dict(action=action, command_id='test1', source='text', issued_at=100., ttl_s=5., **kwargs)

    def test_web_navigation_cannot_silently_fall_back_to_preview(self):
        request = self.request('navigate', building_id='A')
        request['source'] = 'web'
        with self.assertRaisesRegex(ValueError, 'real_navigation_not_enabled'):
            self.router.route(request, 100.)
        live = TaskRouter(self.router.buildings, navigation_execute_enabled=True)
        result = live.route(request, 100.)
        self.assertEqual(result['navigation']['mode'], 'execute')
        self.assertFalse(result['navigation'].get('arrived', False))

    def test_web_pump_and_auto_reject_dry_run_but_stop_remains_available(self):
        for action in ('start_spray', 'auto_fire', 'set_arrival_policy'):
            with self.subTest(action=action):
                request = self.request(action, arrival_policy='auto')
                request['source'] = 'web'
                state = {k: dict(v) for k, v in self.state.items()}
                state['actuator'].update(dry_run=True, enabled=True, pump_enabled=True)
                with self.assertRaisesRegex(ValueError, 'real_actuator_not_enabled'):
                    TaskRouter().route(request, 100., **state)
        request = self.request('stop_spray'); request['source'] = 'web'
        self.assertEqual(TaskRouter().route(request, 100.)['actuator']['action'], 'disarm')

    def test_web_real_pump_still_requires_calibrated_selection(self):
        request = self.request('start_spray'); request['source'] = 'web'
        self.state['actuator'].update(dry_run=False, enabled=True, pump_enabled=True)
        self.assertEqual(self.router.route(request, 100., **self.state)['actuator']['action'], 'arm_pump')
        self.state['proposal']['pump_candidate'] = False
        with self.assertRaisesRegex(ValueError, 'fresh_calibrated_fire_lock_required'):
            TaskRouter().route(request, 100., **self.state)

    def test_web_bare_navigation_does_not_inherit_automatic_spray_policy(self):
        request = self.request('navigate', building_id='A', text='去1号楼')
        request['source'] = 'web'
        self.state['mission']['arrival_policy'] = 'auto'
        self.state['actuator']['dry_run'] = True
        result = TaskRouter(self.router.buildings, navigation_execute_enabled=True).route(request, 100., **self.state)
        self.assertEqual(result['navigation']['arrival_policy'], 'manual')
        self.assertEqual(result['mission']['arrival_policy'], 'manual')

    def test_explicit_separate_actions(self):
        self.assertEqual(parse_task_text('开启喷水')['action'], 'start_spray')
        self.assertEqual(parse_task_text('锁定火焰2')['target_id'], 2)
        self.assertEqual(parse_task_text('锁定二号火焰')['target_id'], 2)
        self.assertEqual(parse_task_text('锁定火焰2然后喷水')['action'], 'dialogue')
        self.assertEqual(parse_task_text('不要开启喷水')['action'], 'dialogue')
        self.assertEqual(parse_task_text('可以开始喷水吗')['action'], 'dialogue')
        self.assertEqual(parse_task_text('锁定第二个火焰')['ordinal'], 2)

    def test_documented_common_command_table(self):
        commands = {
            '开启识别': 'inspect', '开始识别': 'inspect', 'YOLO识别': 'inspect',
            '开启YOLO识别': 'inspect', '开启yolo识别': 'inspect', '开始Yolo识别': 'inspect',
            '观察': 'inspect', '开始观察': 'inspect', '小车，请开启识别。': 'inspect',
            '开启识别火焰、消防员以及居民': 'inspect', '开启火焰识别': 'inspect',
            '开始巡逻': 'patrol', '巡逻': 'patrol', '继续巡逻': 'patrol', '继续行驶': 'patrol',
            '开启场景理解': 'scene', '开始场景理解': 'scene', '场景理解': 'scene',
            '开启AI对话': 'scene', '报告当前场景': 'report', '报告场景': 'report',
            '锁定最左边的火焰': 'select_fire', '锁定右边火焰': 'select_fire',
            '锁定火焰2': 'select_fire', '锁定二号火焰': 'select_fire',
            '开启喷水': 'start_spray', '开始喷水': 'start_spray', '开启喷水灭火': 'start_spray',
            '停止喷水': 'stop_spray', '关闭喷水': 'stop_spray', '停水': 'stop_spray',
            '停车': 'stop', '紧急停车': 'stop',
            '现在有几个人': 'dialogue', '有没有消防员': 'dialogue', '灭火完成了吗': 'dialogue',
        }
        for text, expected in commands.items():
            with self.subTest(text=text):
                parsed = parse_task_text(text, self.router.buildings)
                self.assertEqual(parsed['action'], expected)
        routed = TaskRouter().route(self.request('inspect', text='开启识别'), 100.)
        self.assertEqual(routed['mission']['mode'], 'inspect')

    def test_negative_and_conditional_commands_never_actuate(self):
        for text in ('别开启识别', '不要喷水', '不要开启识别', '不要停止喷水', '别巡逻',
                     '禁止开启喷水', '不能锁定火焰2', '如果开始识别', '可以开启识别吗',
                     '识别和喷水有什么区别', '开启识别然后喷水'):
            with self.subTest(text=text):
                parsed = parse_task_text(text)
                self.assertEqual(parsed['action'], 'dialogue')
                out = TaskRouter().route(self.request('dialogue', text=text), 100.)
                self.assertIsNone(out['mission']); self.assertIsNone(out['actuator'])

    def test_stop_is_idempotent_and_accepts_stale_clock(self):
        request = self.request('stop_spray'); request['issued_at'] = -10
        for _ in range(2):
            out = self.router.route(request, 100.)
            self.assertEqual(out['actuator']['action'], 'disarm')
            self.assertEqual(out['actuator']['issued_at'], 100.)
            self.assertIsNone(out['mission'])

    def test_start_requires_fresh_valid_matched_target(self):
        out = self.router.route(self.request('start_spray'), self.now, **self.state)
        self.assertEqual(out['actuator']['action'], 'arm_pump')
        self.assertEqual(out['actuator']['target_id'], 2)
        self.assertLessEqual(out['actuator']['lease_s'], 10.)
        self.assertIsNone(out['mission'])

    def test_start_denies_stale_or_different_selection(self):
        for key, field, value in [('selection', 'capture_stamp', 99.), ('proposal', 'target_id', 8),
                                  ('proposal', 'registry_session', 'restart'), ('proposal', 'selection_generation', 4),
                                  ('mission', 'mode', 'inspect'), ('mission', 'vehicle_stopped', False),
                                  ('mission', 'gimbal_ready', False), ('proposal', 'pump_candidate', False)]:
            with self.subTest(key=key, field=field):
                router = TaskRouter(); state = {k: dict(v) for k, v in self.state.items()}
                state[key][field] = value
                with self.assertRaises(ValueError):
                    router.route(self.request('start_spray'), 100., **state)

    def test_duplicate_and_cooldown(self):
        request = self.request('start_spray')
        self.router.route(request, 100., **self.state)
        self.assertEqual(self.router.route(request, 100., **self.state)['reason'], 'duplicate')
        request['command_id'] = 'test2'
        with self.assertRaisesRegex(ValueError, 'cooldown'):
            self.router.route(request, 100.1, **self.state)
        self.state['actuator']['guard'] = {'state': 'COOLDOWN', 'armed': False}
        with self.assertRaisesRegex(ValueError, 'cooldown'):
            TaskRouter().route(request, 100., **self.state)

    def test_select_disarms_and_does_not_start_spray(self):
        out = self.router.route(self.request('select_fire', target_id=2), 100., **self.state)
        self.assertEqual(out['mission']['mode'], 'fire')
        self.assertEqual(out['fire_select']['target_id'], 2)
        self.assertEqual(out['actuator']['action'], 'disarm')
        with self.assertRaises(ValueError):
            TaskRouter().route(self.request('select_fire'), 100., **self.state)

    def test_no_text_action_or_target_smuggling(self):
        for request in (self.request('start_spray', text='不要喷水'),
                        self.request('select_fire', text='锁定火焰2', target_id=9)):
            with self.assertRaises(ValueError):
                TaskRouter().route(request, 100., **self.state)

    def test_dialogue_has_no_actuator_or_mission_output(self):
        out = self.router.route(self.request('dialogue', text='你觉得该怎么喷水？'), 100.)
        self.assertIsNone(out['actuator']); self.assertIsNone(out['mission'])
        self.assertIsNotNone(out['question'])

    def test_navigation_default_preview(self):
        out = self.router.route(self.request('navigate', building_id='A'), 100.)
        self.assertEqual(out['navigation']['mode'], 'preview')
        self.assertEqual(parse_task_text('去一号楼观察', self.router.buildings)['building_id'], 'A')

    def test_traffic_pending_observation_is_explicitly_deferred(self):
        for action in ('inspect',):
            out = TaskRouter().route(self.request(action), 100.,
                                     mission=dict(stamp=100., mode='patrol', traffic_active=True))
            self.assertTrue(out['accepted'])
            self.assertEqual(out['mission']['mode'], 'inspect')
            self.assertEqual(out['actuator']['action'], 'disarm')
            self.assertEqual(out['reason'], 'traffic_pending_observation_deferred')
            self.assertIn('楼面识别暂缓', out['speech'])
            self.assertIn('继续巡逻完成过街', out['speech'])
            self.assertIsNone(out['question'])

    def test_destination_names_and_arrival_policy_resolve_without_route(self):
        buildings = {'buildings': [{'building_id': 'C', 'number': '03', 'name': '春晖楼'}]}
        for text, expected in [('去三号楼观察', None), ('前往春晖楼', None),
                               ('小车去春晖楼自动灭火', 'auto'), ('去3号楼等待语音指令', 'manual')]:
            with self.subTest(text=text):
                parsed = parse_task_text(text, buildings)
                self.assertEqual(parsed['building_id'], 'C')
                self.assertEqual(parsed.get('arrival_policy'), expected)
                out = TaskRouter(buildings, navigation_execute_enabled=True).route(
                    dict(parsed, command_id='goal', source='voice', issued_at=100., ttl_s=5.), 100.)
                self.assertEqual(out['navigation']['owner'], 'fpga')
                self.assertEqual(out['navigation']['building_number'], '03')
                self.assertEqual(out['navigation']['arrival_policy'], expected or 'manual')
                self.assertNotIn('route', out['navigation'])
                self.assertEqual(out['auto']['action'], 'cancel')
        for text in ('去陌生楼', '去十四号楼', '去春晖楼再去三号楼'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_task_text(text, buildings)

    def test_arrival_policy_commands_do_not_navigate_or_start_spray(self):
        for text, policy in [('到达后自动找火灭火', 'auto'), ('等待语音指令', 'manual')]:
            parsed = parse_task_text(text)
            out = TaskRouter().route(dict(parsed, command_id='policy', source='voice', issued_at=100.), 100.)
            self.assertEqual(out['mission'], dict(action='set_arrival_policy', arrival_policy=policy))
            self.assertEqual(out['actuator']['action'], 'disarm')
            self.assertIsNone(out['navigation'])
            self.assertEqual(out['auto']['action'], 'cancel')

    def test_finish_cancels_auto_and_navigation_then_road_standby(self):
        parsed = parse_task_text('小车任务结束')
        out = TaskRouter().route(dict(parsed, command_id='finish', source='voice', issued_at=100.), 100.)
        self.assertEqual(out['mission']['mode'], 'standby')
        self.assertEqual(out['actuator']['action'], 'disarm')
        self.assertEqual(out['navigation']['action'], 'cancel')
        self.assertEqual(out['auto']['action'], 'cancel')

    def test_ordinal_is_distinct_from_track_number(self):
        ordinal = parse_task_text('瞄准第二个火焰')
        numbered = parse_task_text('瞄准二号火焰')
        self.assertNotIn('target_id', ordinal)
        self.assertEqual(numbered['target_id'], 2)
        self.assertNotIn('ordinal', numbered)
        out = TaskRouter().route(dict(ordinal, command_id='ordinal', source='voice', issued_at=100.),
                                 100., **self.state)
        self.assertEqual(out['fire_select']['ordinal'], 2)
        self.assertNotIn('target_id', out['fire_select'])
        self.assertEqual(out['auto']['action'], 'cancel')
        with self.assertRaisesRegex(ValueError, 'exactly_one'):
            TaskRouter().route(self.request('select_fire', target_id=2, ordinal=1), 100., **self.state)

    def test_scene_analysis_does_not_interrupt_auto_or_arm_outputs(self):
        parsed = parse_task_text('对当前火场进行分析')
        out = TaskRouter().route(dict(parsed, command_id='analysis', source='voice', issued_at=100.),
                                 100., **self.state)
        for key in ('mission', 'navigation', 'actuator', 'auto', 'fire_select'):
            self.assertIsNone(out[key])
        self.assertEqual(out['question']['text'], '请分析当前场景')

    def test_auto_start_is_explicit_and_operator_stop_always_cancels_it(self):
        parsed = parse_task_text('开始自动找火灭火')
        out = TaskRouter().route(dict(parsed, command_id='auto', source='voice', issued_at=100.), 100.)
        self.assertEqual(out['auto']['action'], 'start')
        self.assertEqual(out['mission']['mode'], 'inspect')
        self.assertEqual(out['actuator']['action'], 'disarm')
        stop = self.request('stop_spray')
        stop['issued_at'] = -100.
        stopped = TaskRouter().route(stop, 100.)
        self.assertEqual(stopped['auto']['action'], 'cancel')
        self.assertEqual(stopped['actuator']['action'], 'disarm')
        for phrase in ('不要自动找火灭火', '到达后自动灭火吗', '任务结束了吗'):
            self.assertEqual(parse_task_text(phrase)['action'], 'dialogue')

    def test_arrival_policy_cannot_be_smuggled_in_text_request(self):
        request = self.request('navigate', text='去一号楼观察', building_id='A', arrival_policy='auto')
        with self.assertRaisesRegex(ValueError, 'text_arrival_policy'):
            self.router.route(request, 100.)


if __name__ == '__main__':
    unittest.main()
