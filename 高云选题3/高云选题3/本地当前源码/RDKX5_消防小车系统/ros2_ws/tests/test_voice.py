"""Control grammar and narration tests; real audio test is recorded separately."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/rescue_vision'))
from rescue_vision.voice import IntentParser, asr_text, scene_to_speech, SpeechDeduplicator, prepare_spoken_command


class VoiceTests(unittest.TestCase):
    def setUp(self):
        self.parser = IntentParser([{'number': str(i), 'building_id': chr(64+i)} for i in range(1, 14)])

    def test_number_normalization_and_single_goal(self):
        for text, expected in [('小车去三号楼观察', 'C'), ('请去3号楼', 'C'),
                               ('前往十三号楼巡检', 'M'), ('到两栋观察', 'B')]:
            cmd = self.parser.parse(text, 10)
            self.assertEqual(cmd['building_id'], expected)
            self.assertEqual(cmd['action'], 'navigate')
            self.assertEqual(cmd['capture_stamp'], 10)
            self.assertEqual(cmd['ttl_s'], 5)
        with self.assertRaises(ValueError): self.parser.parse('去十四号楼', 10)

    def test_negative_ambiguous_question_rejected(self):
        for text in ['不要去三号楼', '不要停车', '能去3号楼吗', '去3号楼或者4号楼',
                     '先去3号楼再去4号楼', '导航到300号楼', '如果发现火焰就停车', '播放歌曲']:
            with self.subTest(text=text), self.assertRaises(ValueError): self.parser.parse(text, 10)

    def test_stop_and_local_intents(self):
        for text, action in [('立即停车', 'stop'), ('去3号楼然后紧急停车', 'stop'),
                              ('报告当前场景', 'report'), ('观察当前场景', 'inspect'), ('继续行驶', 'resume')]:
            self.assertEqual(self.parser.parse(text, 10)['action'], action)

    def test_asr_confidence_gate(self):
        text, confidence = asr_text({'text': '停 车', 'result': [{'conf': .9}, {'conf': .8}]})
        self.assertEqual(self.parser.parse(text, 10)['action'], 'stop')
        self.assertAlmostEqual(confidence, .85)
        for result in [{'text': '停车'}, {'text': '停车', 'result': [{'conf': .1}]},
                       {'text': '停车', 'result': [{'conf': float('nan')}]}]:
            with self.assertRaises(ValueError): asr_text(result)

    def test_scene_evidence_scope_and_counts(self):
        report = dict(stamp=10., valid=True, observation_scope='current_view',
                      building_number='03', building_name='春晖楼', fire_status='observed',
                      visible_person_count=2, visible_firefighter_count=1, casualty_count=9)
        text = scene_to_speech(report, 11)
        self.assertIn('03号楼', text)
        self.assertIn('2名普通人物', text)
        self.assertIn('1名消防员', text)
        self.assertNotIn('9', text)
        self.assertIn('无法仅凭图像确认', text)
        self.assertIn('过期', scene_to_speech(report, 15))
        report['valid'] = False
        self.assertNotIn('春晖楼', scene_to_speech(report, 11))

    def test_unassigned_view_is_not_assigned_to_building(self):
        report = dict(stamp=10, valid=False, building_name='错误楼',
                      unassigned_current_view=dict(fire_observed=False, visible_person_count=2,
                                                   visible_firefighter_count=0))
        text = scene_to_speech(report, 11)
        self.assertIn('归属尚未确认', text)
        self.assertIn('本次画面未检出', text)
        self.assertNotIn('错误楼', text)
        report['detector_available'] = False
        self.assertNotIn('2名', scene_to_speech(report, 11))

    def test_dedup_cooldown(self):
        dedup = SpeechDeduplicator(10)
        self.assertTrue(dedup.accept('当前看到两个人', 1))
        self.assertFalse(dedup.accept('当前看到两个人', 2))
        self.assertTrue(dedup.accept('当前看到三个人', 2))
        self.assertTrue(dedup.accept('当前看到两个人', 11))

    def test_continuous_voice_requires_wake_but_exact_stop_is_direct(self):
        from rescue_vision.task_interaction import parse_task_text
        for text in ('开始喷水', '开启自动灭火', '任务结束', '瞄准第二个火焰'):
            with self.subTest(text=text), self.assertRaisesRegex(ValueError, 'wake_word'):
                prepare_spoken_command(text, 'continuous')
        for text in ('消防小车，寻找火焰', '小车，对当前火场进行分析', '停止喷水', '紧急停车'):
            accepted = prepare_spoken_command(text, 'continuous')
            self.assertEqual(accepted, text)
            self.assertNotEqual(parse_task_text(accepted)['action'], 'dialogue')
        for text in ('停止喷水吗', '不要停止喷水'):
            with self.subTest(text=text), self.assertRaisesRegex(ValueError, 'wake_word'):
                prepare_spoken_command(text, 'continuous')
        negative = prepare_spoken_command('小车，不要喷水', 'continuous')
        self.assertEqual(parse_task_text(negative)['action'], 'dialogue')
        self.assertEqual(prepare_spoken_command('寻找火焰', 'push_to_talk'), '寻找火焰')

    def test_voice_and_text_use_same_new_task_grammar(self):
        from rescue_vision.task_interaction import parse_task_text
        buildings = {'buildings': [{'number': '03', 'building_id': 'C', 'name': '春晖楼'}]}
        for text, action in [('消防小车去春晖楼', 'navigate'), ('小车瞄准第二个火焰', 'select_fire'),
                              ('小车开启自动灭火', 'auto_fire'), ('小车任务结束', 'finish_task')]:
            self.assertEqual(parse_task_text(prepare_spoken_command(text, 'continuous'), buildings)['action'], action)


if __name__ == '__main__':
    unittest.main()
