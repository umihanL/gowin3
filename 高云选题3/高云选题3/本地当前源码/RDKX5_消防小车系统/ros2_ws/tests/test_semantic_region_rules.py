"""Region intent, exact ASR agreement and retired patrol rejection; no hardware."""
import unittest
from test_numbered_interaction import command, extension
from rescue_vision.task_interaction import parse_task_text, TaskRouter, validate_point_request
from rescue_vision.voice_verification import verify_command, needs_independent_verification


TEXT = '在A区域搜寻火源并执行灭火'


class SemanticRegionRuleTests(unittest.TestCase):
    def test_equivalent_region_phrases_are_canonical(self):
        for text in (TEXT, '请在a区搜索火焰并自动灭火', '前往诶区域寻找火源然后进行灭火'):
            self.assertEqual({k:v for k,v in parse_task_text(text).items() if k != 'text'},
                             dict(action='search_region', region_id='A', execute_fire=True))
        for letter in 'BCD':
            self.assertEqual(parse_task_text(TEXT.replace('A', letter))['region_id'], letter)

    def test_search_only_phrases_preserve_absence_of_water_authority(self):
        for text in ('去C区域搜索火焰', '请在c区寻找火源', '前往西区域搜寻火焰'):
            self.assertEqual({k:v for k,v in parse_task_text(text).items() if k != 'text'},
                             dict(action='search_region', region_id='C', execute_fire=False))

    def test_unknown_region_and_extra_motion_are_rejected(self):
        for text in (TEXT.replace('A', 'E'), TEXT+'然后后退20厘米',
                     '在A或B区域搜寻火源并执行灭火', '在A区域搜寻火源并连续喷水'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_task_text(text)

    def test_questions_quotes_and_negatives_are_read_only(self):
        for text in ('不要'+TEXT, TEXT+'可以吗', '如果'+TEXT, '“'+TEXT+'”', '解释'+TEXT):
            self.assertEqual(parse_task_text(text)['action'], 'dialogue', text)

    def test_dispatch_submits_intent_without_immediate_water_or_movement(self):
        result = TaskRouter(extension_enabled=True).route(command(TEXT),100.,extension=extension(region_ready=True))
        self.assertEqual(result['extension']['action'], 'search_region')
        self.assertEqual(result['extension']['region_id'], 'A')
        self.assertTrue(result['extension']['execute_fire'])
        self.assertEqual(result['actuator']['action'], 'disarm')
        self.assertEqual(result['auto']['action'], 'cancel')
        self.assertIsNone(result['navigation'])
        self.assertIn('等待执行端确认', result['speech'])

    def test_search_only_dispatch_uses_observation_readiness_with_unready_gimbal(self):
        result = TaskRouter(extension_enabled=True).route(command('去C区域搜索火焰'),100.,
            extension=extension(region_observe_ready=True,region_ready=False,
                                region_reason='region_real_gimbal_reference_required'),
            actuator=dict(stamp=100.,dry_run=True,enabled=False,pump_enabled=False))
        self.assertIs(result['extension']['execute_fire'],False)
        self.assertIs(result['rule_navigation']['execute_fire'],False)
        self.assertEqual(result['extension']['region_id'],'C')
        self.assertEqual(result['actuator']['action'],'disarm')
        self.assertEqual(result['auto']['action'],'cancel')
        self.assertIn('90%',result['speech'])
        self.assertIn('停稳观察2秒',result['speech'])
        self.assertIn('继续后续检查点',result['speech'])
        self.assertNotIn('等待你的下一步指令',result['speech'])
        self.assertNotIn('短喷',result['speech'])

    def test_full_readiness_cannot_substitute_for_search_only_feedback(self):
        for changes in (dict(region_observe_ready=False),dict(region_observe_ready=None),
                        dict(real_uart=False),dict(stamp=98.),dict(status_fresh=False)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                TaskRouter(extension_enabled=True).route(command('去C区域搜索火焰'),100.,
                    extension=extension(**dict(dict(region_ready=True,region_observe_ready=True),**changes)))

    def test_search_only_cannot_be_promoted_or_encoded_as_non_boolean(self):
        for value in (True,0,None,'false'):
            with self.subTest(value=value),self.assertRaises(ValueError):
                TaskRouter(extension_enabled=True).route(command('去C区域搜索火焰',execute_fire=value),100.,
                    extension=extension(region_ready=True,region_observe_ready=True))

    def test_boundary_compares_region_and_automatic_authority(self):
        for changes in (dict(region_id='B'), dict(execute_fire=False), dict(execute_fire=1)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                TaskRouter(extension_enabled=True).route(dict(command(TEXT), **changes),100.,extension=extension(region_ready=True))

    def test_real_feedback_and_explicit_ready_state_required(self):
        for changes in (dict(region_ready=False), dict(real_uart=False), dict(status_fresh=False),
                        dict(stamp=98.), dict(task='vision')):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                TaskRouter(extension_enabled=True).route(command(TEXT, source='web'),100.,
                    extension=extension(**dict(dict(region_ready=True), **changes)))
        for changes in (dict(text=''), dict(source='test'), dict(voice_verification={}), dict(voice_verification='verified')):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                TaskRouter(extension_enabled=True).route(dict(command(TEXT), **changes),100.,extension=extension(region_ready=True))

    def test_independent_asr_compares_region_and_fire_authority(self):
        self.assertTrue(needs_independent_verification(TEXT,dict(confidence_available=False),None))
        self.assertTrue(needs_independent_verification(TEXT,dict(confidence_available=True),None))
        self.assertTrue(verify_command(TEXT,dict(text='请在诶区域寻找火焰并自动灭火',result=[dict(conf=.95)]),None)['verified'])
        for text in (TEXT.replace('A','B'), '在A区域搜索火源', '去4号点'):
            self.assertFalse(verify_command(TEXT,dict(text=text,result=[dict(conf=.95)]),None)['verified'])
        self.assertTrue(verify_command('去C区域搜索火焰',dict(text='前往西区寻找火源',result=[dict(conf=.95)]),None)['verified'])
        self.assertFalse(verify_command('去C区域搜索火焰',dict(text='去C区域搜索火焰并灭火',result=[dict(conf=.95)]),None)['verified'])

    def test_old_patrol_phrases_and_fields_are_rejected(self):
        for text in ('我现在在8号点，车头朝左，请前往4号点，沿途自动灭火。',
                     '从八号点到四号点进行前进，沿途灭火导航'):
            with self.assertRaisesRegex(ValueError, 'retired_patrol'):
                parse_task_text(text)
        for changes in (dict(patrol_fire=True,fire_ignore_s=3.), dict(patrol_fire=False), dict(fire_ignore_s=3.)):
            with self.subTest(changes=changes), self.assertRaisesRegex(ValueError, 'retired_patrol'):
                TaskRouter(extension_enabled=True).route(command('去4号点', **changes),100.,extension=extension())
            with self.assertRaisesRegex(ValueError, 'retired_patrol'):
                validate_point_request(dict(action='navigate_point',target_point=4,**changes))

    def test_numbered_navigation_remains_manual_and_old_stop_remains_stop(self):
        result=TaskRouter(extension_enabled=True).route(command('去4号点'),100.,extension=extension())
        self.assertEqual(result['extension']['arrival_policy'],'manual')
        self.assertNotIn('execute_fire',result['extension'])
        self.assertEqual(parse_task_text('停止沿途灭火')['action'],'stop')
        self.assertEqual(parse_task_text('停止区域搜索')['action'],'stop')
        stopped=TaskRouter(extension_enabled=True).route(command('停车',patrol_fire=True,fire_ignore_s=3.),100.,extension=extension())
        self.assertEqual(stopped['actuator']['action'],'disarm')
        self.assertEqual(stopped['extension']['action'],'stop')


if __name__ == '__main__':
    unittest.main()
