"""Production ROS router under fake transport: numbered navigation is rule-only."""
import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rescue_vision'))
from rescue_vision.task_interaction import TaskRouter,parse_task_text

class Publisher:
    def __init__(self):self.values=[]
    def publish(self,value):self.values.append(value)

class RuleNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        modules={n:types.ModuleType(n) for n in ('rclpy','rclpy.node','std_msgs','std_msgs.msg','rescue_vision.node_common')}
        modules['rclpy.node'].Node=object;modules['std_msgs.msg'].String=types.SimpleNamespace
        common=modules['rescue_vision.node_common'];common.param=lambda n,k,d:d;common.read_json=lambda p:{}
        common.publish_json=lambda p,v:p.publish(v);common.spin=lambda n:None
        spec=importlib.util.spec_from_file_location('rescue_vision._rules_node_test',ROOT/'src/rescue_vision/rescue_vision/task_interaction_node.py')
        cls.module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,modules):spec.loader.exec_module(cls.module)

    def setUp(self):
        self.clock=100.
        timer=patch.object(self.module.time,'monotonic',lambda:self.clock)
        timer.start();self.addCleanup(timer.stop)
        n=self.module.TaskInteraction.__new__(self.module.TaskInteraction);self.node=n
        n.now=lambda:self.clock;n.router=TaskRouter(extension_enabled=True);n.cache={};n.result=Publisher()
        n.output_pubs={k:Publisher() for k in ('auto','actuator','extension','mission','navigation','fire_select','question','speech')}
        self.refresh()

    def refresh(self,**changes):
        extension=dict(stamp=self.clock,real_uart=True,status_fresh=True,task='dialogue',dialogue_enabled=True,
            vehicle_stopped=True,can_confirm_point_start=True,can_navigate_point=True,fpga_task_mode=1,
            point_reference=dict(confirmed=True,point_id=8,heading_confirmed=True,current_heading_deg=180.))
        extension.update(changes)
        self.node.cache['extension']=(extension,self.clock)
        self.node.cache['mission']=(dict(mode='inspect'),self.clock)

    def request(self,**changes):
        text='你现在在8号点位，车头朝左，请导航前往至4号点位。'
        value=dict(parse_task_text(text),source='web',command_id='rules-1',issued_at=self.clock,ttl_s=3.)
        value.update(changes);return value

    def receive(self,request):
        self.node.receive(types.SimpleNamespace(data=json.dumps(request)))
        return self.node.result.values[-1]

    def assert_no_navigation(self):
        self.assertFalse(any(v.get('action')=='navigate_point' for v in self.node.output_pubs['extension'].values))

    def test_navigation_routes_immediately_with_rule_receipt_and_no_model(self):
        with patch('urllib.request.build_opener',side_effect=AssertionError('navigation called network')):
            result=self.receive(self.request())
        self.assertTrue(result['accepted'],result)
        self.assertEqual(result['navigation_parser'],'rules')
        self.assertEqual(result['rule_navigation'],dict(original_text=self.request()['text'],
            start_point=8,target_point=4,start_heading_deg=180.,model_called=False))
        self.assertIn('规则解析',result['speech']);self.assertIn('等待执行端确认',result['speech'])
        self.assertEqual(len(self.node.output_pubs['extension'].values),1)
        self.assertFalse(self.node.output_pubs['question'].values)
        self.assertFalse(hasattr(self.node,'semantic_worker'))

    def test_old_dialogue_or_semantic_action_is_reparsed_without_qwen(self):
        for action in ('dialogue','semantic_navigate'):
            result=self.receive(self.request(action=action,command_id=action))
            self.assertTrue(result['accepted'],result)
            self.assertEqual(result['action'],'navigate_point')
        self.assertFalse(self.node.output_pubs['question'].values)

    def test_voice_requires_independent_asr_receipt(self):
        result=self.receive(self.request(source='voice'))
        self.assertEqual(result['reason'],'numbered_navigation_independent_asr_required');self.assert_no_navigation()
        result=self.receive(self.request(source='voice',command_id='verified',voice_verification=dict(verified=True)))
        self.assertTrue(result['accepted'],result)

    def test_stale_missing_or_not_ready_hardware_never_routes(self):
        for changes in (dict(real_uart=False),dict(status_fresh=False),dict(can_confirm_point_start=False)):
            self.node.router.seen.clear();self.refresh(**changes)
            result=self.receive(self.request());self.assertFalse(result['accepted']);self.assert_no_navigation()
        self.node.router.seen.clear();self.refresh();self.clock+=1.
        result=self.receive(self.request());self.assertFalse(result['accepted']);self.assert_no_navigation()

    def test_missing_heading_uses_only_matching_confirmed_context(self):
        text='你现在在8号点位，请导航前往至4号点位'
        req=dict(parse_task_text(text),source='web',command_id='context',issued_at=self.clock)
        result=self.receive(req);self.assertTrue(result['accepted'],result)
        self.assertEqual(result['rule_navigation']['start_heading_deg'],180.)
        for reference in (dict(confirmed=False),dict(confirmed=True,point_id=7,heading_confirmed=True,current_heading_deg=180),
                          dict(confirmed=True,point_id=8,heading_confirmed=False,current_heading_deg=180)):
            self.refresh(point_reference=reference);self.node.router.seen.clear()
            result=self.receive(req);self.assertEqual(result['reason'],'numbered_start_heading_required')
            self.assertEqual(result['navigation_parser'],'rules')
            self.assertFalse(result['rule_navigation']['model_called'])
            self.assertEqual(result['rule_navigation']['original_text'],text)
            self.assertIn('请补充车头',result['speech'])
            self.assertIn('未下发导航',result['speech'])

    def test_target_only_and_current_position_leave_reference_resolution_to_live_state(self):
        text='从当前位置出发，请前往4号点后停下'
        result=self.receive(dict(parse_task_text(text),source='web',command_id='current',issued_at=self.clock))
        self.assertTrue(result['accepted'],result)
        self.assertEqual(result['rule_navigation']['start_point'],8)
        self.assertNotIn('start_point',self.node.output_pubs['extension'].values[-1])

    def test_tampered_points_or_heading_are_rejected_by_text_reparse(self):
        for change in (dict(target_point=3),dict(start_point=4),dict(start_heading_deg=0)):
            self.node.router.seen.clear();result=self.receive(self.request(**change))
            self.assertEqual(result['reason'],'text_point_mismatch');self.assert_no_navigation()

    def test_duplicate_and_expired_commands_do_not_route_twice(self):
        self.receive(self.request());result=self.receive(self.request())
        self.assertEqual(result['reason'],'duplicate')
        result=self.receive(self.request(command_id='expired',issued_at=90.))
        self.assertEqual(result['reason'],'expired_or_future_command')
        self.assertEqual(len(self.node.output_pubs['extension'].values),1)

    def test_navigation_does_not_wait_for_ordinary_qwen_and_question_stays_question(self):
        # No scene-model busy subscription can delay deterministic navigation.
        result=self.receive(self.request());self.assertTrue(result['accepted'])
        text='分析当前火场'
        result=self.receive(dict(parse_task_text(text),source='web',command_id='scene',issued_at=self.clock))
        self.assertTrue(result['accepted'],result)
        self.assertTrue(self.node.output_pubs['question'].values)

    def test_stop_remains_immediate_without_model_state(self):
        result=self.receive(dict(action='stop',text='停车',source='voice',command_id='stop',issued_at=0.))
        self.assertTrue(result['accepted']);self.assertEqual(result['action'],'stop')

    def region_request(self, **changes):
        value=dict(parse_task_text('在A区域搜寻火源并执行灭火'),source='web',
                   command_id='region-1',issued_at=self.clock,ttl_s=3.)
        value.update(changes)
        return value

    def test_region_rejection_speaks_chinese_and_retains_rule_receipt(self):
        for reason, phrase in (
                ('mark_start_and_heading_on_fpga_or_hmi', '实际起点和车头方向'),
                ('region_approach_calibration_verified_required', '标定尚未验收'),
                ('region_approach_time_sync_verified_required', '时间对应关系'),
                ('semantic_region_map_identity_mismatch', '地图不一致')):
            self.node.router.seen.clear()
            self.refresh(region_ready=False,region_reason=reason)
            request=self.region_request()
            result=self.receive(request)
            self.assertFalse(result['accepted'])
            self.assertEqual(result['reason'],reason)
            self.assertEqual(result['action'],'search_region')
            self.assertEqual(result['rule_navigation'],dict(original_text=request['text'],
                region_id='A',execute_fire=True,model_called=False))
            self.assertIn(phrase,result['speech'])
            self.assertIn('未下发区域任务',result['speech'])
            self.assertFalse(self.node.output_pubs['extension'].values)
            self.assertFalse(self.node.output_pubs['question'].values)
            self.assertEqual(self.node.output_pubs['speech'].values[-1].data,result['speech'])

    def test_region_voice_missing_proof_is_spoken_without_task_dispatch(self):
        self.refresh(region_ready=True)
        result=self.receive(self.region_request(source='voice'))
        self.assertEqual(result['reason'],'region_search_independent_asr_required')
        self.assertIn('独立语音核对',result['speech'])
        self.assertFalse(self.node.output_pubs['extension'].values)

    def test_legacy_dialogue_region_is_reparsed_without_qwen(self):
        self.refresh(region_ready=True)
        result=self.receive(self.region_request(action='dialogue'))
        self.assertTrue(result['accepted'],result)
        self.assertEqual(result['action'],'search_region')
        self.assertFalse(self.node.output_pubs['question'].values)
        self.assertEqual(self.node.output_pubs['extension'].values[-1]['region_id'],'A')

    def test_retired_patrol_command_explains_new_region_entry(self):
        result=self.receive(self.request(patrol_fire=True))
        self.assertEqual(result['reason'],'retired_patrol_fire_use_region_search')
        self.assertIn('已停用',result['speech'])
        self.assertIn('在A区域搜寻火源并执行灭火',result['speech'])
        self.assertFalse(self.node.output_pubs['extension'].values)

    def test_ambiguous_or_multiple_actions_never_dispatch_movement(self):
        for text in ('从8号点前往4号点然后喷水','从8号点去4号点或者3号点','从8号点前往4号点吗？'):
            result=self.receive(dict(action='semantic_navigate',text=text,source='web',command_id='bad'+str(len(text)),issued_at=self.clock))
            self.assert_no_navigation()

if __name__=='__main__':unittest.main()
