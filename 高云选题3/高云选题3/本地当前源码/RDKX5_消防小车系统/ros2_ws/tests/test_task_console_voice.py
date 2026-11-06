"""Task-first voice variants retain exact parameters and read-only ambiguity."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.task_interaction import parse_task_text, TaskRouter
from rescue_vision.voice_verification import verify_command, needs_independent_verification


class TaskConsoleVoiceTests(unittest.TestCase):
    def test_natural_point_phrases_keep_both_endpoints(self):
        for text in ('帮我从二号点导航到六号点', '请从第2号点位出发前往第6号点位',
                     '导航从2号位置到6号导航点', '请帮我从二号点出发去六号点'):
            value=parse_task_text(text)
            self.assertEqual((value['action'],value['start_point'],value['target_point']),
                             ('navigate_point',2,6),text)
        for text in ('请帮我导航去6号点', '小车，前往第六号导航点', '帮我到六号位置'):
            self.assertEqual(parse_task_text(text)['target_point'],6,text)

    def test_no_guessed_digits_negation_or_extra_actions(self):
        for text in ('帮我不要去六号点', '能否帮我导航到六号点',
                     '帮我去六号店', '我想知道从二号点到六号点多远',
                     '帮我从二号点到六号点如果可以的话'):
            self.assertEqual(parse_task_text(text)['action'],'dialogue',text)
        for text, reason in (('帮我去六号点然后喷水','numbered_navigation_separate_actions'),
                             ('帮我去六号点还是七号点','numbered_navigation_needs_clear_points')):
            with self.assertRaisesRegex(ValueError,reason):parse_task_text(text)
        for text in ('请帮我去十号点','导航从二号点到二号点'):
            with self.assertRaises(ValueError):parse_task_text(text)

    def test_visual_aim_does_not_imply_spray(self):
        self.assertEqual(parse_task_text('自动瞄准2号火源')['target_id'],2)
        self.assertEqual(parse_task_text('自动瞄准第2团火源')['ordinal'],2)
        self.assertEqual(parse_task_text('自动瞄准最左边的火源')['selector'],'leftmost')
        self.assertEqual(parse_task_text('自动瞄准2号火源然后喷水')['action'],'dialogue')
        for text in ('停止导航','取消导航','停止瞄准','停止自动瞄准'):
            self.assertEqual(parse_task_text(text)['action'],'stop')

    def test_independent_asr_matches_meaning_not_wording(self):
        result=lambda text:dict(text=text,result=[dict(conf=.95)])
        self.assertTrue(verify_command('帮我从二号点前往六号点',result('从2号点导航到6号点'),None)['verified'])
        self.assertFalse(verify_command('帮我从二号点前往六号点',result('从2号点导航到7号点'),None)['verified'])
        self.assertTrue(verify_command('自动瞄准2号火源',result('瞄准二号火焰'),None)['verified'])

    def test_actual_sensevoice_navigation_and_natural_terminal_stop(self):
        # Exact text captured on X5: it used to reach Qwen as ordinary dialogue.
        for text in ('你现在在8号点位，请导航前往至4号点位。',
                     '你现在在八号点位，然后请前往四号点位到达后停止。',
                     '当前你在8号位置上，接下来请你导航前往4号位置，到达以后停下来。',
                     '我现在的位置是8号点，请行驶至4号点后停下车，谢谢。'):
            value=parse_task_text(text)
            self.assertEqual((value['action'],value['start_point'],value['target_point']),('navigate_point',8,4),text)
            self.assertTrue(value['start_declared'])
            self.assertNotIn('start_heading_deg',value)
            self.assertTrue(needs_independent_verification(text,{'confidence_available':False},None))

    def test_map_heading_is_explicit_and_independently_verified(self):
        for direction, expected in (('右',0.),('上',90.),('左',180.),('下',-90.),('9.5度',9.5),('-90度',-90.)):
            text=f'你现在在8号点位，车头朝{direction}，请导航前往至4号点位。'
            value=parse_task_text(text)
            self.assertEqual(value['start_heading_deg'],expected,text)
            verified=verify_command(text,dict(text=f'我现在在八号点位朝向{expected}度请前往四号点位',result=[{'conf':.95}]),None)
            self.assertTrue(verified['verified'],verified)
        primary='你现在在8号点位，车头朝右，请前往4号点位'
        for secondary in ('你现在在8号点位请前往4号点位',
                          '你现在在8号点位车头朝左请前往4号点位',
                          '你现在在8号点位车头朝右请前往5号点位'):
            self.assertFalse(verify_command(primary,dict(text=secondary,result=[{'conf':.95}]),None)['verified'])
        with self.assertRaisesRegex(ValueError,'start_heading_requires_minus180_to180'):
            parse_task_text('你现在在8号点位车头朝181度请前往4号点位')

    def test_unclear_navigation_does_not_fall_through_to_qwen(self):
        for text in ('你现在在8号点位请前往几号点位','你现在在八号点位前往四号点位或者五号点位',
                     '从八号点位导航前往','你现在在8号点位，请导航前往至四号店'):
            with self.assertRaisesRegex(ValueError,'numbered_navigation_needs_clear_points'):
                parse_task_text(text)
        for text in ('不要从8号点位前往4号点位', '你现在在8号点位请前往4号点位？',
                     '你现在在8号点位请前往4号点位吗', '“你现在在8号点位请前往4号点位”',
                     '如果你现在在8号点位请前往4号点位'):
            self.assertEqual(parse_task_text(text)['action'],'dialogue',text)

    def test_heading_cannot_be_changed_after_parse(self):
        parsed=parse_task_text('你现在在8号点位车头朝右请前往4号点位')
        request=dict(parsed,command_id='heading',source='voice',issued_at=100.,start_heading_deg=180.)
        with self.assertRaisesRegex(ValueError,'text_point_mismatch'):
            TaskRouter(extension_enabled=True).route(request,100.)

    def test_stale_dialogue_label_cannot_send_navigation_to_model(self):
        for text, reason in (('你现在在8号点位，请导航前往至4号点位。','text_action_mismatch'),
                             ('你现在在8号点位请前往几号点位','numbered_navigation_needs_clear_points')):
            with self.assertRaisesRegex(ValueError,reason):
                TaskRouter().route(dict(text=text,action='dialogue',source='voice',
                                        command_id='stale-route',issued_at=100.),100.)


if __name__=='__main__':unittest.main()
