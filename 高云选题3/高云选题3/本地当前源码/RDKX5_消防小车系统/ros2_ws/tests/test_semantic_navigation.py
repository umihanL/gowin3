"""Numbered navigation grammar regression; no model imports or network calls."""
import unittest
from rescue_vision.task_interaction import parse_task_text

class RuleNavigationTests(unittest.TestCase):
    def test_original_and_spoken_variants_match_same_command(self):
        for text in ('你现在在8号点位，车头朝左，请导航前往至4号点位。',
                     '你现在在8号点车头方向朝左，请前往4号点，并且停下。',
                     '小车目前位于8号点位，车头朝向左方，请帮我开往4号点位',
                     '我当前位置是8号点位，车头朝地图左侧，请前往4号点后停下',
                     '你 现在 在 八 号 点 位 车头 朝 左 请 前往 四 号 点 位'):
            with self.subTest(text=text):
                parsed=parse_task_text(text)
                self.assertEqual(parsed,dict(text=text,action='navigate_point',start_point=8,
                    target_point=4,start_heading_deg=180.,start_declared=True))

    def test_route_is_not_a_declaration_and_context_is_not_invented(self):
        route=parse_task_text('从8号点车头朝左前往4号点后停下')
        self.assertNotIn('start_declared',route)
        for text in ('前往4号点','从当前位置出发请前往4号点后停下','从这里去4号点',
                     '车头方向朝左请前往4号点'):
            parsed=parse_task_text(text)
            self.assertEqual(parsed['target_point'],4)
            self.assertNotIn('start_point',parsed)
        parsed=parse_task_text('你现在在8号点请前往4号点')
        self.assertNotIn('start_heading_deg',parsed)

    def test_explicit_chinese_and_decimal_heading_preserve_value(self):
        for wording,value in [('负九十',-90.),('一百八十',180.),('九点五',9.5),('9.5',9.5),('-9.5',-9.5)]:
            text=f'你现在在8号点车头朝向{wording}度请前往4号点'
            self.assertEqual(parse_task_text(text)['start_heading_deg'],value)

    def test_negation_question_quote_and_ambiguous_actions_do_not_navigate(self):
        for text in ('不要从8号点前往4号点','从8号点前往4号点吗？','如果从8号点去4号点',
                     '解释“从8号点去4号点”','从8号点去4号点或者3号点','从8号点去4号点然后喷水',
                     '我现在在8号点请前往4号点并且瞄准1号火焰'):
            with self.subTest(text=text):
                try:parsed=parse_task_text(text)
                except ValueError:continue
                self.assertNotEqual(parsed['action'],'navigate_point')

    def test_invalid_point_and_heading_are_not_coerced(self):
        for text in ('从0号点去4号点','从8号点去10号点','从8号点去8号点',
                     '你现在在8号点车头朝向190度请前往4号点'):
            with self.subTest(text=text), self.assertRaises(ValueError):parse_task_text(text)

    def test_scene_questions_remain_dialogue_or_scene(self):
        self.assertEqual(parse_task_text('分析当前火场')['action'],'scene')
        self.assertEqual(parse_task_text('当前场景里有几团火焰')['action'],'dialogue')

    def test_unsupported_navigation_wording_is_rejected_without_qwen_fallback(self):
        for text in ('带我去4号点', '你先开到4号点', '请你把车开往4号点位'):
            with self.subTest(text=text), self.assertRaisesRegex(ValueError, 'numbered_navigation_needs_clear_points'):
                parse_task_text(text)
        self.assertEqual(parse_task_text('为什么8号点到4号点要这样走')['action'], 'dialogue')

if __name__=='__main__':unittest.main()
