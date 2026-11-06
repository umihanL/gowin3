import json
import unittest
from unittest.mock import MagicMock, patch

from rescue_vision.task_scene import (LocalIntentModel, build_scene, render_dialogue,
                                     fallback_decision, compact_facts, freeze_scene_snapshot)
from rescue_vision.edge_dialogue import system_answer, status_evidence, safe_model_error, ungrounded_scene_analysis


class EdgeDialogueTests(unittest.TestCase):
    def setUp(self):
        audit = patch('rescue_vision.task_scene.REPLY_AUDIT_LOG')
        self.audit_log = audit.start()
        self.addCleanup(audit.stop)

    def model(self, **values):
        return LocalIntentModel(dict(enabled=True, mode='generate', timeout_s=75.,
            answer_timeout_s=80., model='qwen2.5-0.5b-instruct', **values))

    def call(self, value, question='解释一下机器视觉原理', report=None):
        model = self.model()
        with patch.object(model, '_request', return_value=value):
            return model.respond(question, report or {})

    def test_precision_questions_do_not_wait_for_model(self):
        model = self.model()
        for question in ('现在有几个人', '当前楼号是什么', '楼牌是什么', '灭火完成了吗', '有没有消防员', '报告当前场景'):
            self.assertFalse(model.requires_worker(question), question)
            with patch.object(model, '_request', side_effect=AssertionError('must be immediate')):
                result = model.respond(question, {})
            self.assertEqual(result['backend'], 'structured_rules')
            self.assertFalse(result['general_chat_available'])

    def test_unknown_analysis_and_explanation_use_real_model_path(self):
        for question in ('你好', '解释机器视觉原理', '请分析当前场景', '有什么建议'):
            self.assertTrue(self.model().requires_worker(question), question)

    def test_general_explanation_keywords_do_not_require_live_scene(self):
        for question in ('消防员的职责是什么', '解释一下机器视觉是什么',
                         '帮我写一段消防小车的演示开场白', '现在给我解释图像处理原理', '为 什么 天空 是 蓝色'):
            model = self.model()
            self.assertTrue(model.requires_worker(question), question)
            with patch.object(model, '_request', return_value=dict(intent='knowledge',
                    reply='可以通过更多光照和视角的样本提高检测稳健性。', observation_hint='none')) as request:
                result = model.respond(question, {'valid': False, 'stale': True})
            self.assertEqual(result['backend'], 'edge_llm_generate', question)
            self.assertEqual(result['response_scope'], 'general_knowledge')
            self.assertEqual(request.call_args.kwargs['schema']['properties']['intent']['const'], 'knowledge')
            self.assertNotIn('快照', request.call_args.args[1])
            self.assertIn('本地中文助手', request.call_args.args[0])

    def test_project_technical_explanations_use_audited_references_not_small_model(self):
        for question, expected in (('为什么火焰检测会漏检', '目标像素太少'),
                ('解释一下ROS2节点是什么', '一个或多个节点'), ('现在给我解释YOLO原理', '边界框'),
                ('FPGA和X5怎么分工', 'UART'), ('为 什么 火焰 检测 会 漏检', '实拍样本')):
            model = self.model()
            self.assertFalse(model.requires_worker(question), question)
            with patch.object(model, '_request', side_effect=AssertionError('must use audited reference')):
                result = model.respond(question, {})
            self.assertEqual(result['backend'], 'domain_reference')
            self.assertIn(expected, render_dialogue(question, result, {}, 100.))
            self.assertFalse(result['general_chat_available'])

    def test_status_capabilities_and_model_identity_are_immediate(self):
        for question, scope in (('当前硬件连接状态', 'system_status'), ('麦克风和音箱能用吗', 'system_status'),
                ('现在导航到哪里了', 'system_status'), ('你好，介绍一下自己', 'system_status'),
                ('可以问什么', 'capabilities'), ('今天天气怎么样', 'unavailable_live_information')):
            model = self.model()
            self.assertFalse(model.requires_worker(question), question)
            with patch.object(model, '_request', side_effect=AssertionError('not a model question')):
                result = model.respond(question, {})
            self.assertEqual(result['response_scope'], scope)
            text = render_dialogue(question, result, {}, 100.)
            self.assertTrue(text)
            self.assertNotIn('先停车', text)
        self.assertIn('Qwen2.5-0.5B', render_dialogue('你是谁', fallback_decision('你是谁'), {}, 100.))

    def test_spaced_asr_multi_class_question_preserves_both_counts(self):
        question = '看 到 几个 人，有 没有 消防员'
        report = build_scene(dict(capture_stamp=100., detector_available=True,
            detections=[dict(class_name=name, confidence=.9) for name in ('person', 'person', 'firefighter')]), 100.)
        decision = self.model().respond(question, report)
        text = render_dialogue(question, decision, report, 100.)
        self.assertIn('2名普通人物', text)
        self.assertIn('1名消防员', text)

    def test_system_status_has_own_freshness_and_does_not_claim_wireless_pairing(self):
        report = dict(valid=False, stale=True, system_status=dict(
            voice=dict(_received_stamp=100., stamp=100., microphone_enabled=True,
                capture_active=False, tts_enabled=False),
            fpga_link=dict(_received_stamp=100., dry_run=True, status_fresh=True)))
        text = system_answer('当前硬件连接状态', report, 100.)
        self.assertIn('麦克风配置已启用', text)
        self.assertIn('语音播报未启用', text)
        self.assertIn('不等于已验证无线配对', text)
        self.assertIn('模拟模式', text)
        self.assertIn('暂无新鲜状态', system_answer('FPGA连接状态', report, 102.))
        self.assertEqual(status_evidence(dict(_received_stamp=100., stamp=50.), 100., 12.), {})

    def test_navigation_preview_or_invalid_arrival_cannot_claim_real_arrival(self):
        navigation = dict(_received_stamp=100., stamp=100., building_number='03', mode='preview',
            proposal=dict(valid=True, status_fresh=True, arrived=True, vehicle_stopped=True))
        report = dict(system_status=dict(navigation=navigation))
        self.assertIn('未执行实车导航', system_answer('现在导航到哪里了', report, 100.))
        navigation['mode'] = 'execute'
        navigation['proposal']['vehicle_stopped'] = False
        self.assertIn('尚无有效的到达且停稳确认', system_answer('现在导航到哪里了', report, 100.))
        navigation['proposal']['vehicle_stopped'] = True
        self.assertIn('有效回执确认已到达03号楼', system_answer('现在导航到哪里了', report, 100.))

    def test_generated_claim_of_internet_search_is_rejected(self):
        result = self.call(dict(intent='knowledge', reply='我已经联网查询到答案。', observation_hint='none'))
        self.assertEqual(result['backend'], 'structured_rules')
        self.assertEqual(result['model_error'], 'ungrounded_generated_claim')

    def test_known_errors_are_diagnostic_codes_without_arbitrary_details(self):
        self.assertEqual(safe_model_error(ValueError('ungrounded_generated_claim')), 'ungrounded_generated_claim')
        self.assertEqual(safe_model_error(ValueError('secret server body')), 'ValueError')
        self.assertEqual(safe_model_error(TimeoutError('http://private/token')), 'TimeoutError')
        fallback = fallback_decision('请用中文说一句鼓励我的话', 'invalid_generation_schema')
        self.assertEqual(fallback['response_scope'], 'general_knowledge')
        self.assertEqual(fallback['model_error'], 'invalid_generation_schema')
        self.assertIn('暂未生成有效回答', render_dialogue('请用中文说一句鼓励我的话', fallback, {}, 100.))

    def test_disabled_generation_does_not_network(self):
        model = LocalIntentModel({'enabled': False, 'mode': 'generate'})
        with patch.object(model, '_request', side_effect=AssertionError('network forbidden')):
            self.assertEqual(model.respond('你好', {})['backend'], 'structured_rules')

    def test_knowledge_is_explicitly_not_live_evidence(self):
        result = self.call(dict(intent='knowledge', reply='机器视觉利用图像和算法识别目标或测量特征。', observation_hint='none'))
        self.assertEqual(result['backend'], 'edge_llm_generate')
        self.assertTrue(result['general_chat_available'])
        text = render_dialogue('解释机器视觉原理', result, {}, 100.)
        self.assertIn('通用知识解释', text)
        self.assertLessEqual(len(text), 350)

    def test_stereo_knowledge_is_curated_not_generated(self):
        for model in (self.model(), LocalIntentModel({'enabled': False}),
                      LocalIntentModel({'enabled': True, 'mode': 'intent'})):
            for question in ('解释双目测距原理', '视差是什么', '双目距离公式是什么', '立体视觉测距原理'):
                with patch.object(model, '_request', side_effect=AssertionError('reference must not call model')):
                    result = model.respond(question, {'visible_person_count': 99})
                self.assertEqual(result['backend'], 'domain_reference')
                self.assertFalse(result['general_chat_available'])
                self.assertIsNone(result['model_error'])
                text = render_dialogue(question, result, {}, 100.)
                self.assertTrue(text.startswith('本地技术参考'))
                self.assertIn('Z=fB/d', text)
                self.assertIn('像素', text); self.assertIn('米', text)
                self.assertIn('标定或匹配未通过', text)
                self.assertNotIn('99', text)
                self.assertLessEqual(len(text), 350)

    def test_disabled_node_fallback_also_returns_reference(self):
        result = fallback_decision('双目测距原理')
        self.assertEqual(result['backend'], 'domain_reference')
        self.assertFalse(result['general_chat_available'])
        self.assertIn('匹配左右同一点', render_dialogue('双目测距原理', result, {}, 100.))

    def test_live_stereo_distance_is_unknown_not_a_formula_or_fake_measurement(self):
        for question in ('当前双目距离是多少', '现在双目测到多远', '当前火焰的双目距离是多少', '当前视差是多少'):
            for model in (self.model(), LocalIntentModel({'enabled': False})):
                with patch.object(model, '_request', side_effect=AssertionError('unverified distance must not call model')):
                    result = model.respond(question, {'stereo_distance': 999})
                self.assertEqual(result['backend'], 'structured_rules')
                self.assertFalse(result['general_chat_available'])
                text = render_dialogue(question, result, {}, 100.)
                self.assertIn('尚未接入', text)
                self.assertIn('不能报告可靠的实测距离', text)
                self.assertNotIn('999', text)
                self.assertNotIn('Z=fB/d', text)

    def test_machine_vision_explanation_still_calls_language_model(self):
        model = self.model()
        with patch.object(model, '_request', return_value=dict(intent='knowledge',
                reply='机器视觉通过图像处理与学习算法获取环境信息。', observation_hint='none')) as request:
            result = model.respond('解释机器视觉原理', {})
        request.assert_called_once()
        self.assertEqual(result['backend'], 'edge_llm_generate')
        self.assertTrue(result['general_chat_available'])

    def test_generated_scene_claims_are_discarded(self):
        result = self.call(dict(intent='scene', reply='已经扑灭，现场有99名受困者。', observation_hint='inspect_people'),
            question='分析当前场景')
        report = build_scene(dict(capture_stamp=100., detector_available=True,
            detections=[dict(class_name='person', confidence=.9)]), 100.)
        text = render_dialogue('分析当前场景', result, report, 100.)
        self.assertIn('1名普通人物', text)
        self.assertNotIn('99', text)
        self.assertNotIn('已经扑灭', text)
        self.assertEqual(result['backend'], 'structured_rules')

    def test_scene_empty_reply_selects_observation_hint_only(self):
        result = self.call(dict(intent='scene', reply='', observation_hint='inspect_people'), question='分析当前场景')
        report = build_scene(dict(capture_stamp=100., detector_available=True,
            detections=[dict(class_name='person', confidence=.9)]), 100.)
        text = render_dialogue('分析当前场景', result, report, 100.)
        self.assertEqual(result['backend'], 'edge_llm_generate')
        self.assertIn('1名普通人物', text)
        self.assertIn('未执行控制', text)

    def test_scene_cannot_escape_into_knowledge_reply(self):
        result = self.call(dict(intent='knowledge', reply='3号楼有8个人。', observation_hint='none'), question='分析当前场景')
        text = render_dialogue('分析当前场景', result, {}, 100.)
        self.assertNotIn('8个人', text)
        self.assertIn('没有新鲜有效', text)

    def test_unverified_knowledge_action_claim_falls_back(self):
        result = self.call(dict(intent='knowledge', reply='我已经开启水泵。', observation_hint='none'))
        self.assertEqual(result['backend'], 'structured_rules')
        self.assertFalse(result['general_chat_available'])

    def test_trailing_fence_noise_removed_without_changing_claim(self):
        result = self.call(dict(intent='knowledge', reply='双目依据视差计算深度。``````', observation_hint='none'))
        self.assertEqual(result['reply'], '双目依据视差计算深度。')
        self.assertEqual(result['backend'], 'edge_llm_generate')
        result = self.call(dict(intent='knowledge', reply='``````', observation_hint='none'))
        self.assertEqual(result['backend'], 'structured_rules')

    def test_character_limit_retains_only_complete_sentence(self):
        complete = '通过左右匹配同点的视差计算深度。'
        raw = complete + '未' * (180-len(complete))
        result = self.call(dict(intent='knowledge', reply=raw, observation_hint='none'))
        self.assertEqual(result['backend'], 'edge_llm_generate')
        self.assertEqual(result['reply'], complete)
        for prefix in ('未完成的回复', '深度3.14159'):
            raw = prefix + '未' * (180-len(prefix))
            result = self.call(dict(intent='knowledge', reply=raw, observation_hint='none'))
            self.assertEqual(result['backend'], 'structured_rules')
        raw = '字'*179+'。'
        result = self.call(dict(intent='knowledge', reply=raw, observation_hint='none'))
        self.assertEqual(result['reply'], raw)

    def test_compact_snapshot_preserves_unknown_vs_zero_and_does_not_mutate_report(self):
        report = dict(stale=False, detector_available=True, visible_fire_count=0,
            visible_person_count=None, visible_firefighter_count=2, building_association_verified=False,
            pump_on=False, actuator_dry_run=True, water_flow_verified=False, extinguished_verified=False)
        text = compact_facts(report)
        self.assertIn('有效画面检出：火焰图案0处、普通人物数量未知、消防员图案2名', text)
        self.assertIn('楼牌状态未知；目标与楼栋归属未验证', text)
        self.assertIn('模拟泵指令关闭；实际出水未验证；扑灭未验证', text)
        self.assertIn('楼牌已确认；目标与楼栋归属未验证', compact_facts(dict(report, building_sign_confirmed=True)))
        self.assertIsNone(report['visible_person_count'])
        self.assertIn('无有效画面：火焰图案数量未知', compact_facts(dict(report, stale=True)))
        self.assertIn('泵指令未知', compact_facts({}))

    def test_strict_schema_rejects_actions_unknown_hint_and_long_text(self):
        valid = dict(intent='knowledge', reply='你好。', observation_hint='none')
        for value in (dict(valid, action='arm_pump'), dict(valid, observation_hint='open_pump'),
                      dict(valid, intent=['scene']), dict(valid, reply='字'*181),
                      dict(valid, observation_hint='widen_view'), dict(valid, reply=''), {}, []):
            result = self.call(value)
            self.assertEqual(result['backend'], 'structured_rules')
            self.assertIsNotNone(result['model_error'])

    def test_timeout_and_bad_json_report_fallback_error(self):
        model = self.model()
        for error in (TimeoutError(), json.JSONDecodeError('invalid', '', 0)):
            with patch.object(model, '_request', side_effect=error):
                result = model.respond('分析当前场景', {})
            self.assertEqual(result['model_error'], type(error).__name__)
            self.assertEqual(result['backend'], 'structured_rules')

    def test_http_request_is_loopback_bounded_and_schema_constrained(self):
        model = self.model()
        inner = json.dumps(dict(reply='你好。'))
        body = json.dumps({'choices': [{'message': {'content': inner}}]}).encode()
        response = MagicMock()
        response.__enter__.return_value.read.return_value = body
        opener = MagicMock(); opener.open.return_value = response
        with patch('urllib.request.build_opener', return_value=opener):
            result = model.respond('你好', {'visible_person_count': 2})
        request = opener.open.call_args.args[0]
        payload = json.loads(request.data)
        self.assertEqual(opener.open.call_args.kwargs['timeout'], 75.)
        self.assertEqual(payload['response_format']['type'], 'json_schema')
        self.assertEqual(payload['max_tokens'], 256)
        self.assertEqual(payload['model'], 'qwen2.5-0.5b-instruct')
        self.assertEqual(result['backend'], 'edge_llm_generate')
        self.assertEqual(payload['messages'][-1]['content'], '你好')
        self.assertNotIn('visible_person_count', request.data.decode())
        schema = payload['response_format']['json_schema']['schema']['properties']
        self.assertEqual(set(schema), {'reply'})
        self.assertEqual(schema['reply']['maxLength'], 90)

    def test_qwen3_generation_has_explicit_nonthinking_sampling_only(self):
        for name in ('qwen3-1.7b', 'qwen2.5-1.5b-instruct'):
            model = self.analysis_model()
            model.model = name
            response = MagicMock()
            response.__enter__.return_value.read.return_value = json.dumps({'choices': [
                {'finish_reason':'stop','message': {'content': json.dumps({'reply':'你好。'})}}]}).encode()
            opener = MagicMock(); opener.open.return_value = response
            with patch('urllib.request.build_opener', return_value=opener):
                model.respond('你好', {})
            payload = json.loads(opener.open.call_args.args[0].data)
            expected = dict(top_p=.8, top_k=20, min_p=0., presence_penalty=1.5)
            for key, value in expected.items():
                if name.startswith('qwen3-'):
                    self.assertEqual(payload[key], value)
                else:
                    self.assertNotIn(key, payload)

    def test_current_view_decoding_forbids_free_facts_and_knowledge(self):
        model = self.model()
        for question in ('请分析当前场景', '请解释这里的情况', '请分析周围', '附近有什么建议'):
            with patch.object(model, '_request', return_value=dict(intent='scene', reply='', observation_hint='none')) as request:
                result = model.respond(question, {'visible_person_count': 2, 'stale':False, 'detector_available':True})
            self.assertEqual(result['backend'], 'edge_llm_generate')
            schema = request.call_args.kwargs['schema']['properties']
            self.assertNotEqual(schema['intent'].get('const'), 'knowledge')
            self.assertNotIn('knowledge', schema['intent'].get('enum', []))
            self.assertEqual(schema['reply']['const'], '')
            self.assertIn('\n快照：', request.call_args.args[1])
            self.assertIn('普通人物2名', request.call_args.args[1])
            self.assertNotIn('visible_person_count', request.call_args.args[1])

    def test_spray_explanation_keeps_known_intent_and_relevant_hint(self):
        question = '为什么喷水停止不等于扑灭'
        model = self.model()
        valid = dict(intent='spray', reply='', observation_hint='verify_spray')
        with patch.object(model, '_request', return_value=valid) as request:
            result = model.respond(question, {})
        self.assertEqual(result['backend'], 'edge_llm_generate')
        props = request.call_args.kwargs['schema']['properties']
        self.assertEqual(props['intent']['const'], 'spray')
        self.assertEqual(props['observation_hint']['enum'], ['none', 'verify_spray'])
        for invalid in (dict(valid, intent='scene'), dict(valid, observation_hint='verify_building')):
            with patch.object(model, '_request', return_value=invalid):
                result = model.respond(question, {})
            self.assertEqual(result['backend'], 'structured_rules')
            self.assertEqual(result['intent'], 'spray')

    def test_complete_json_with_length_finish_reason_is_rejected(self):
        model = self.model()
        content = json.dumps(dict(intent='knowledge', reply='你好。', observation_hint='none'))
        response = MagicMock()
        response.__enter__.return_value.read.return_value = json.dumps({'choices': [
            {'finish_reason': 'length', 'message': {'content': content}}]}).encode()
        opener = MagicMock(); opener.open.return_value = response
        with patch('urllib.request.build_opener', return_value=opener):
            result = model.respond('你好', {})
        self.assertEqual(result['backend'], 'structured_rules')
        self.assertEqual(result['model_error'], 'generation_truncated_or_incomplete')

    def test_duplicate_json_fields_are_not_silently_accepted(self):
        model = self.model()
        content = '{"intent":"scene","intent":"knowledge","reply":"你好","observation_hint":"none"}'
        response = MagicMock()
        response.__enter__.return_value.read.return_value = json.dumps({'choices': [{'message': {'content': content}}]}).encode()
        opener = MagicMock(); opener.open.return_value = response
        with patch('urllib.request.build_opener', return_value=opener):
            result = model.respond('你好', {})
        self.assertEqual(result['backend'], 'structured_rules')

    def test_config_bounds(self):
        for config in ({'timeout_s': 91}, {'answer_timeout_s': 91}, {'max_tokens': True},
                       {'max_tokens': 5000}, {'mode': 'execute'}, {'temperature': float('nan')}):
            with self.assertRaises(ValueError):
                LocalIntentModel(config)

    def analysis_report(self, *, stamp=100., sequence=7):
        report = build_scene(dict(capture_stamp=stamp, sequence=sequence, coordinate_frame='left', detector_available=True,
            detections=[dict(class_name=name, confidence=.9) for name in ('fire', 'fire', 'person', 'firefighter')]),
            stamp+.1, mission=dict(stamp=stamp+.1, mode='inspect'))
        report['_question_stamp'] = stamp+.1
        return report

    def analysis_model(self):
        return self.model(scene_analysis_enabled=True, reply_max_chars=160,
            brief_reply_max_chars=48, analysis_reply_max_chars=120)

    def test_configured_longer_dialogue_and_brief_budget_are_independent(self):
        model = self.analysis_model()
        for question, limit in (('你好', 48), ('解释一下机器视觉的应用方向', 160)):
            with patch.object(model, '_request', return_value=dict(intent='knowledge',
                    reply='机器视觉可用于检测与测量。', observation_hint='none')) as request:
                result = model.respond(question, {})
            self.assertEqual(result['backend'], 'edge_llm_generate')
            self.assertEqual(request.call_args.kwargs['schema']['properties']['reply']['maxLength'], limit)
            self.assertIn(str(limit), request.call_args.args[0])

    def test_analysis_snapshot_keeps_original_counts_when_camera_changes(self):
        report = self.analysis_report()
        model = self.analysis_model()
        advice = '建议先复核人物区域，再比较火焰边界；图案不能证明真实危险，需补充观察。'
        with patch.object(model, '_request', return_value=dict(intent='scene', reply=advice, observation_hint='none')):
            result = model.respond('分析当前场景', report)
        report['visible_fire_count'] = 99
        current = self.analysis_report(stamp=130., sequence=300)
        current['visible_person_count'] = 20
        text = render_dialogue('分析当前场景', result, current, 130.)
        self.assertEqual(result['response_scope'], 'historical_snapshot_analysis')
        self.assertIn('30.0秒', text)
        self.assertIn('2处火焰图案', text)
        self.assertIn('1名普通人物', text)
        self.assertNotIn('20名', text)
        self.assertNotIn('99', text)
        self.assertIn('不代表此刻画面', text)
        self.assertIn(advice, text)
        self.assertIn('未执行控制', text)
        snapshot = result['analysis_snapshot']
        self.assertEqual(snapshot['capture_stamp'], 100.)
        self.assertEqual(snapshot['requested_stamp'], 100.1)
        self.assertEqual(snapshot['source_sequence'], 7)
        self.assertEqual(snapshot['coordinate_frame'], 'left')
        self.assertEqual(snapshot['mission_mode'], 'inspect')
        self.assertFalse(snapshot['control_use_allowed'])

    def test_snapshot_retains_delayed_sign_at_request_and_never_links_counts(self):
        report = self.analysis_report()
        report['recent_building_observation'] = dict(confirmed=True, capture_stamp=98.,
            number='03', name='安居楼', source_sequence=6, unverified_pose=True)
        report['building_sign_confirmed'] = True
        model = self.analysis_model()
        with patch.object(model, '_request', return_value=dict(intent='scene',
                reply='建议复核楼牌与目标的对应关系，再补充其他视角。', observation_hint='none')):
            result = model.respond('分析当前场景', report)
        text = render_dialogue('分析当前场景', result, {}, 120.)
        self.assertIn('03号楼安居楼', text)
        self.assertIn('尚未确认目标都属于这栋楼', text)
        self.assertEqual(result['analysis_snapshot']['facts']['recent_building_observation']['capture_stamp'], 98.)

    def test_absent_or_stale_snapshot_cannot_invoke_analysis(self):
        model = self.analysis_model()
        report = self.analysis_report()
        for bad in ({}, dict(report, stale=True), dict(report, _question_stamp=102.),
                    dict(report, visible_person_count=None), dict(report, stamp=float('nan'))):
            with patch.object(model, '_request', side_effect=AssertionError('must not infer from unavailable evidence')):
                result = model.respond('分析当前场景', bad)
            self.assertEqual(result['backend'], 'structured_rules')
            self.assertEqual(result['model_error'], 'scene_snapshot_unavailable')

    def test_analysis_rejects_invented_counts_locations_success_or_actuation(self):
        model = self.analysis_model()
        for reply in ('有99个人。', '该图里有三团火焰。', '目标在3号楼。', '距离目标0.4米。',
                      '已经扑灭火灾。', '居民确实受困。', '我正在开启水泵。',
                      '建议立即前进。', '请打开水枪。', '可见大量火焰。'):
            with patch.object(model, '_request', return_value=dict(intent='scene', reply=reply, observation_hint='none')):
                result = model.respond('分析当前场景', self.analysis_report())
            self.assertEqual(result['backend'], 'structured_rules', reply)
            self.assertEqual(result['model_error'], 'ungrounded_scene_analysis', reply)
            self.assertNotIn(reply, render_dialogue('分析当前场景', result, self.analysis_report(), 100.1))

    def test_analysis_output_only_asks_cpu_model_for_reply(self):
        model = self.analysis_model()
        reply = '建议先复核人物区域，再补充其他观察角度。'
        response = MagicMock()
        response.__enter__.return_value.read.return_value = json.dumps({'choices': [{'finish_reason':'stop',
            'message': {'content': json.dumps({'reply':reply})}}]}).encode()
        opener = MagicMock(); opener.open.return_value = response
        with patch('urllib.request.build_opener', return_value=opener):
            result = model.respond('分析当前场景', self.analysis_report())
        payload = json.loads(opener.open.call_args.args[0].data)
        self.assertEqual(set(payload['response_format']['json_schema']['schema']['properties']), {'reply'})
        self.assertEqual(payload['response_format']['json_schema']['schema']['properties']['reply']['maxLength'], 120)
        self.assertEqual(result['intent'], 'scene')
        self.assertEqual(result['response_scope'], 'historical_snapshot_analysis')
        self.assertEqual(result['reply'], reply)

    def test_snapshot_age_rejects_clock_jump_and_excessive_history(self):
        decision = dict(intent='scene', reply='建议复核人物区域。', backend='edge_llm_generate',
            response_scope='historical_snapshot_analysis', analysis_snapshot=freeze_scene_snapshot(self.analysis_report()))
        for now in (99., 221.):
            text = render_dialogue('分析当前场景', decision, {}, now)
            self.assertIn('已过期或时间异常', text)
            self.assertNotIn('2处火焰', text)

    def test_upgraded_model_keeps_fast_facts_and_no_scene_data_in_chat(self):
        model = self.analysis_model()
        for question in ('当前几个人', '报告当前场景', '水泵关了吗', '当前楼号是什么',
                         '分析一下场景里有几团火焰'):
            self.assertFalse(model.requires_worker(question))
        self.assertTrue(model.requires_worker('分析当前场景有几个人，应该优先观察什么'))
        with patch.object(model, '_request', return_value=dict(intent='knowledge', reply='你好。', observation_hint='none')) as request:
            model.respond('你好', self.analysis_report())
        self.assertNotIn('快照', request.call_args.args[1])

    def test_exact_class_counts_ignore_polite_scene_prefix(self):
        model = self.analysis_model()
        report = self.analysis_report()
        for question, intent, expected, absent in (
                ('分析一下场景里有几团火焰', 'fire', '2处火焰图案', ('普通人物', '消防员', '楼号')),
                ('分析一下当前场景有几个人', 'people', '1名普通人物', ('火焰图案', '消防员', '楼号')),
                ('分析一下场景里有几个消防员', 'firefighters', '1名消防员图案', ('火焰图案', '普通人物', '楼号'))):
            with patch.object(model, '_request', side_effect=AssertionError('exact count must be immediate')):
                result = model.respond(question, report)
            self.assertEqual(result['intent'], intent)
            self.assertEqual(result['backend'], 'structured_rules')
            text = render_dialogue(question, result, report, 100.1)
            self.assertIn(expected, text)
            for word in absent:
                self.assertNotIn(word, text)

    def test_count_questions_keep_multi_class_and_advice_as_scene(self):
        from rescue_vision.task_scene import question_intent
        for question in ('分析场景里有几团火焰和几个人', '有几团火焰，还有没有消防员',
                         '场景有多少人，应该优先复核什么', '场景里有几个消防员，有什么建议'):
            self.assertEqual(question_intent(question), 'scene', question)

    def test_uncertainty_and_confirmation_advice_are_not_positive_claims(self):
        model = self.analysis_model()
        reply = '建议先确认目标归属；仅凭图案无法确认受困，需要补充观察。'
        with patch.object(model, '_request', return_value=dict(intent='scene', reply=reply, observation_hint='none')):
            result = model.respond('分析当前场景', self.analysis_report())
        self.assertEqual(result['backend'], 'edge_llm_generate')
        self.assertEqual(result['reply'], reply)

    def test_analysis_rejects_synonymous_fire_and_people_outcomes(self):
        model = self.analysis_model()
        for reply in ('火已经熄灭，人员安全。', '扑救完成，可以放心。', '灭火已成功。',
                      '人员被困，需要救援。', '人员已全部救出。', '居民已经获救。',
                      '大家都安全。', '危险已解除。', '受困人员已经脱离危险。',
                      '未确认楼号，但火已熄灭。', '尚不能确认火已熄灭，人员已经救出。',
                      '无法确认火已熄灭且居民已经获救。', '无法确认楼号火已熄灭。'):
            self.assertTrue(ungrounded_scene_analysis(reply), reply)
            with patch.object(model, '_request', return_value=dict(intent='scene', reply=reply, observation_hint='none')):
                result = model.respond('分析当前场景', self.analysis_report())
            self.assertEqual(result['backend'], 'structured_rules', reply)
            self.assertEqual(result['model_error'], 'ungrounded_scene_analysis', reply)

    def test_analysis_allows_uncertain_outcomes_with_local_scope(self):
        model = self.analysis_model()
        for reply in ('尚不能确认已熄灭，需复核火焰区域。', '尚不能确认已扑灭，建议补充观察。',
                      '人员是否受困仍需核验。', '尚不能确认是否安全。',
                      '居民获救与否需要核实。', '建议保障人员安全。',
                      '如果火已经熄灭，建议继续复查。'):
            self.assertFalse(ungrounded_scene_analysis(reply), reply)
            with patch.object(model, '_request', return_value=dict(intent='scene', reply=reply, observation_hint='none')):
                result = model.respond('分析当前场景', self.analysis_report())
            self.assertEqual(result['backend'], 'edge_llm_generate', reply)

    def test_conditional_review_topics_are_allowed_without_claiming_observations(self):
        model = self.analysis_model()
        for reply in ('建议先复核火势变化，再确认人员是否受困。',
                      '优先确认火势，并检查是否有被困人员。',
                      '建议先检查画面清晰度，确认是否存在人员受困。',
                      '应核实火情，确认人员安全情况。', '优先复核火情和消防设施'):
            self.assertFalse(ungrounded_scene_analysis(reply), reply)
            with patch.object(model, '_request', return_value=dict(intent='scene', reply=reply, observation_hint='none')):
                result = model.respond('分析当前场景，应该优先复核什么', self.analysis_report())
            self.assertEqual(result['backend'], 'edge_llm_generate', reply)

    def test_current_view_review_wording_does_not_become_a_live_claim(self):
        model = self.analysis_model()
        for reply in ('优先确认当前画面是否覆盖目标区域，再核对楼牌是否清晰；尚不能判断人员是否受困。',
                      '建议核对当前楼牌是否清晰，再复核人物区域。',
                      '建议检查当前是否有人，人员是否受困仍需核验。'):
            self.assertFalse(ungrounded_scene_analysis(reply), reply)
            with patch.object(model, '_request', return_value=dict(intent='scene', reply=reply, observation_hint='none')):
                result = model.respond('分析当前场景，应该优先复核什么', self.analysis_report())
            self.assertEqual(result['backend'], 'edge_llm_generate', reply)
        for reply in ('建议复核楼牌，当前现场有大量火焰。', '当前人员已安全。',
                      '当前有火焰，建议复核楼牌。', '建议确认当前楼牌，但现场人员已获救。'):
            self.assertTrue(ungrounded_scene_analysis(reply), reply)

    def test_natural_snapshot_preserves_true_false_unknown_without_private_codes(self):
        report = self.analysis_report()
        flags = ('building_sign_confirmed', 'building_association_verified',
                 'pump_on', 'actuator_dry_run', 'water_flow_verified', 'extinguished_verified')
        affirmative = compact_facts(dict(report, **dict.fromkeys(flags, True)))
        self.assertIn('楼牌已确认；目标与楼栋归属已验证', affirmative)
        self.assertIn('模拟泵指令开启；实际出水已验证；扑灭已验证', affirmative)
        negative = compact_facts(dict(report, **dict.fromkeys(flags, False)))
        self.assertIn('楼牌未确认；目标与楼栋归属未验证', negative)
        self.assertIn('水泵控制指令关闭；实际出水未验证；扑灭未验证', negative)
        unknown = compact_facts(dict(report, **dict.fromkeys(flags, None)))
        self.assertIn('楼牌状态未知；目标与楼栋归属未知', unknown)
        self.assertIn('输出模式未知，泵指令未知；实际出水验证状态未知；扑灭验证状态未知', unknown)
        for text in (affirmative, negative, unknown):
            for private_code in ('楼证', '扑灭证', '出水证', '新鲜1', '模拟1'):
                self.assertNotIn(private_code, text)
        self.assertEqual(report['visible_fire_count'], 2)

    def test_zero_unknown_and_expired_counts_are_never_no_fire_or_safe(self):
        report = self.analysis_report()
        zero = compact_facts(dict(report, visible_fire_count=0, visible_person_count=0, visible_firefighter_count=0))
        self.assertIn('检出：火焰图案0处、普通人物0名、消防员图案0名', zero)
        for text in (zero, compact_facts(dict(report, stale=True)), compact_facts(dict(report, detector_available=None)),
                     compact_facts(dict(report, visible_fire_count=True, visible_person_count=-1, visible_firefighter_count=1000))):
            self.assertNotIn('没有火', text)
            self.assertNotIn('无火', text)
            self.assertNotIn('安全', text)
            self.assertNotIn('扑灭已验证', text)
        for changed in (dict(report, stale=True), dict(report, detector_available=None)):
            text = compact_facts(changed)
            self.assertIn('火焰图案数量未知', text)
            self.assertNotIn('火焰图案2处', text)

    def test_review_advice_cannot_mask_following_positive_facts(self):
        for reply in ('建议复核火势，但火势已经失控。', '建议复核火势，火势很大。',
                      '建议检查楼牌，但确认人员安全。', '建议检查人员状态，已确认全部获救。',
                      '优先复核火势，并已确认人员安全。', '建议核实左侧的火焰。'):
            self.assertTrue(ungrounded_scene_analysis(reply), reply)

    def test_review_list_allows_nominal_extinction_effect_not_asserted_success(self):
        model = self.analysis_model()
        for reply in ('优先复核楼牌归属、模拟泵状态及扑灭效果',
                      '建议复核楼牌归属、模拟泵状态和灭火结果。',
                      '应检查扑灭效果，再核实楼牌。'):
            self.assertFalse(ungrounded_scene_analysis(reply), reply)
            with patch.object(model, '_request', return_value=dict(intent='scene', reply=reply, observation_hint='none')):
                result = model.respond('分析当前场景，应该优先复核什么', self.analysis_report())
            self.assertEqual(result['backend'], 'edge_llm_generate', reply)
        for reply in ('火已扑灭，人员安全。', '优先复核楼牌归属、模拟泵状态，火已扑灭。',
                      '优先复核楼牌归属及火已扑灭。', '建议复核扑灭效果，人员已安全。',
                      '优先复核楼牌及火已扑灭效果很好。', '扑灭效果很好，人员安全。'):
            self.assertTrue(ungrounded_scene_analysis(reply), reply)

    def test_rejected_reply_is_bounded_json_in_local_audit_only(self):
        model = self.analysis_model()
        reply = '已经扑灭。\n' + '字'*500
        with patch.object(model, '_request', return_value=dict(intent='scene', reply=reply, observation_hint='none')):
            result = model.respond('分析当前场景', self.analysis_report())
        self.audit_log.warning.assert_called_once()
        event = json.loads(self.audit_log.warning.call_args.args[1])
        self.assertEqual(event['question'], '分析当前场景')
        self.assertEqual(event['reply'], reply[:320])
        self.assertTrue(event['reply_truncated'])
        self.assertNotIn('\n', self.audit_log.warning.call_args.args[1])
        self.assertNotIn('raw_reply', result)
        self.assertNotIn('已经扑灭', render_dialogue('分析当前场景', result, self.analysis_report(), 100.1))

    def test_upgraded_config_bounds_reject_wrong_types_and_unbounded_lengths(self):
        for config in ({'scene_analysis_enabled': 'true'}, {'reply_max_chars': 321},
                       {'brief_reply_max_chars': True}, {'analysis_reply_max_chars': 0}):
            with self.assertRaises(ValueError):
                LocalIntentModel(config)


if __name__ == '__main__':
    unittest.main()
