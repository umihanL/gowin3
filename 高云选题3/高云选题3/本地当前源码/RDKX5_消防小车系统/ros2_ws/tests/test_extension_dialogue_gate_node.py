"""Direct ROS question ingress must obey the selected task, including late answers."""
import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rescue_vision'))
from rescue_vision.task_scene import LocalIntentModel


class Publisher:
    def __init__(self):self.values=[]
    def publish(self,value):self.values.append(value)


class DialogueGateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        modules={name:types.ModuleType(name) for name in ('rclpy','rclpy.node','std_msgs','std_msgs.msg',
            'rescue_interfaces','rescue_interfaces.msg','rescue_vision.node_common')}
        modules['rclpy.node'].Node=object
        modules['std_msgs.msg'].String=types.SimpleNamespace
        modules['rescue_interfaces.msg'].DetectionArray=object
        common=modules['rescue_vision.node_common']
        common.IMAGE_QOS=None;common.param=lambda n,k,d:d;common.read_json=lambda p:{}
        common.seconds=lambda s:0.;common.spin=lambda n:None
        common.publish_json=lambda p,v:p.publish(v)
        spec=importlib.util.spec_from_file_location('rescue_vision._dialogue_gate_test',ROOT/'src/rescue_vision/rescue_vision/task_scene_node.py')
        cls.module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,modules):spec.loader.exec_module(cls.module)

    def setUp(self):
        node=self.module.TaskScene.__new__(self.module.TaskScene)
        node.extension_enabled=True;node.evidence={};node.pending=None;node.seen={}
        node.now=lambda:100.
        node.answer_pub=Publisher();node.speech_pub=Publisher();node.report_pub=Publisher()
        node.dialogue_status_pub=Publisher();node.dialogue_status_value=None;node.dialogue_status_published=0
        node.model=Mock(model='testmodel',enabled=True,answer_timeout=3.)
        node.worker=Mock();node.scene=lambda:{}
        self.node=node

    def mode(self,task,enabled):
        self.node.evidence['extension']=(dict(stamp=100.,task=task,dialogue_enabled=enabled),100.)

    def test_direct_question_cannot_bypass_traffic_gate(self):
        self.mode('traffic',False)
        self.node.question(types.SimpleNamespace(data=json.dumps(dict(command_id='question',text='讲一个故事',stamp=100.,ttl_s=3.))))
        self.assertFalse(self.node.answer_pub.values[-1]['accepted'])
        self.node.model.requires_worker.assert_not_called()
        self.node.worker.submit.assert_not_called()

    def test_task2_offline_accepts_question_without_fpga(self):
        self.mode('dialogue',True)
        self.node.model.requires_worker.return_value=True
        self.node.question(types.SimpleNamespace(data=json.dumps(dict(command_id='question',text='讲一个故事',stamp=100.,ttl_s=3.))))
        self.node.worker.submit.assert_called_once()
        self.assertEqual(self.node.pending['question']['command_id'],'question')

    def test_task1_offline_accepts_scene_question_without_fpga(self):
        self.mode('vision',True)
        self.node.model.requires_worker.return_value=True
        self.node.question(types.SimpleNamespace(data=json.dumps(dict(command_id='scene-one',text='分析当前火场',stamp=100.,ttl_s=3.))))
        self.node.worker.submit.assert_called_once()
        self.assertEqual(self.node.pending['question']['command_id'],'scene-one')

    def test_task1_full_model_answer_is_published_to_speech_without_shortening(self):
        self.mode('vision',True)
        text='以下基于提问时快照：'+('建议先复核火源位置和人员情况，保持稳定观察。'*20)+'以上为观察建议，未执行控制。'
        self.assertGreater(len(text),350);self.assertLess(len(text),900)
        with patch.object(self.module,'render_dialogue',return_value=text):
            self.node.answer(dict(command_id='scene-one',text='分析当前火场'),
                             dict(intent='scene',backend='local_qwen',observation_hint='none'))
        answer=self.node.answer_pub.values[-1]
        self.assertTrue(answer['accepted'])
        self.assertEqual(answer['text'],text)
        self.assertEqual(self.node.speech_pub.values[-1].data,text)
        self.assertFalse(answer['command_execution_allowed'])

    def test_switch_to_traffic_discards_inflight_result_and_speech(self):
        self.mode('traffic',False)
        future=Mock();future.done.return_value=True
        self.node.pending=dict(future=future,question=dict(command_id='question',text='讲一个故事'),
            started=0.,deadline=1000.,expired=False)
        self.node.tick()
        future.cancel.assert_called_once()
        future.result.assert_not_called()
        self.assertEqual(self.node.answer_pub.values[-1]['reason'],'dialogue_task_changed_result_discarded')
        self.assertFalse(self.node.speech_pub.values)

    def test_direct_late_answer_never_speaks_in_traffic(self):
        self.mode('traffic',False)
        self.node.answer(dict(command_id='question',text='有几团火焰'),{})
        self.assertFalse(self.node.answer_pub.values[-1]['accepted'])
        self.assertFalse(self.node.speech_pub.values)


if __name__=='__main__':unittest.main()
