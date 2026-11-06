"""Real ROS constructors/topic contracts. Isolated domain 83/84; no audio hardware."""
import json
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('RESCUE_RUN_ROS_TEST') == '1', 'explicit isolated ROS test only')
class TaskDialogueBoardTests(unittest.TestCase):
    def setUp(self):
        self.assertIn(os.environ.get('ROS_DOMAIN_ID'), ('83', '84'))
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String
        from rescue_vision.task_interaction_node import TaskInteraction
        from rescue_vision.task_scene_node import TaskScene
        from rescue_vision.voice_node import Voice
        from rescue_interfaces.msg import DetectionArray, Detection
        from rescue_vision.node_common import IMAGE_QOS
        self.rclpy, self.String, self.DetectionArray, self.Detection = rclpy, String, DetectionArray, Detection
        self.temporary = tempfile.TemporaryDirectory(prefix='task-dialogue-test-')
        self.addCleanup(self.temporary.cleanup)
        workspace = Path(self.temporary.name); (workspace/'config').mkdir()
        buildings = workspace/'config/buildings.json'
        buildings.write_text(json.dumps({'buildings': [{'building_id': 'A', 'number': '1'}]}))
        voice = workspace/'config/voice.json'
        voice.write_text(json.dumps(dict(microphone_enabled=False, tts_enabled=False, auto_scene_speech=False,
            model_dir='models/absent', speech_cooldown_s=0, speech_queue_limit=1)))
        self.process_guard = patch('subprocess.Popen', side_effect=AssertionError('No audio/process hardware in test'))
        self.process_guard.start(); self.addCleanup(self.process_guard.stop)
        rclpy.init(args=['--ros-args', '-p', 'workspace_dir:='+str(workspace), '-p', 'voice_config:='+str(voice),
                        '-p', 'buildings_config:='+str(buildings), '-p', 'task_commands:=true'])
        self.nodes = []
        self.executor = SingleThreadedExecutor()
        self.addCleanup(self.cleanup_ros)
        self.voice = Voice(); self.nodes.append(self.voice)
        self.router = TaskInteraction(); self.nodes.append(self.router)
        self.scene = TaskScene(); self.nodes.append(self.scene)
        self.probe = Node('task_dialogue_probe'); self.nodes.append(self.probe)
        for node in self.nodes:
            self.executor.add_node(node)
        self.received = {key: [] for key in ('result', 'mission', 'actuator', 'answer', 'command')}
        for key, topic in (('result', '/interaction/result'), ('mission', '/mission/request'),
                           ('actuator', '/actuators/request'), ('answer', '/scene/answer'), ('command', '/interaction/command')):
            self.probe.create_subscription(String, topic, lambda msg, k=key: self.received[k].append(json.loads(msg.data)), 10)
        self.text_pub = self.probe.create_publisher(String, '/voice/text', 10)
        self.objects_pub = self.probe.create_publisher(DetectionArray, '/perception/detections', IMAGE_QOS)
        self.actuator_pub = self.probe.create_publisher(String, '/actuators/state', 10)
        self.pump(.5)

    def cleanup_ros(self):
        for node in reversed(self.nodes):
            self.executor.remove_node(node); node.destroy_node()
        self.executor.shutdown()
        if self.rclpy.ok():
            self.rclpy.shutdown()

    def pump(self, duration=.3):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.02)

    def text(self, value):
        self.text_pub.publish(self.String(data=value)); self.pump(.4)

    def test_voice_task_mode_and_stop_without_hardware(self):
        self.text('开始巡逻')
        self.assertEqual(self.received['mission'][-1]['mode'], 'patrol')
        self.assertEqual(self.received['actuator'][-1]['action'], 'disarm')
        self.text('开启识别')
        self.assertTrue(self.received['result'][-1]['accepted'])
        self.assertEqual(self.received['command'][-1]['action'], 'inspect')
        self.assertEqual(self.received['mission'][-1]['mode'], 'inspect')
        self.text('开启喷水')
        self.assertFalse(self.received['result'][-1]['accepted'])
        self.assertEqual(self.received['result'][-1]['command_id'], self.received['command'][-1]['command_id'])
        self.assertEqual(self.received['command'][-1]['action'], 'start_spray')
        self.assertNotIn('arm_pump', [x['action'] for x in self.received['actuator']])
        self.text('停止喷水')
        self.assertTrue(self.received['result'][-1]['accepted'])
        self.assertEqual(self.received['actuator'][-1]['action'], 'disarm')
        self.assertFalse(self.voice.config['microphone_enabled']); self.assertFalse(self.voice.config['tts_enabled'])

    def test_scene_topic_from_actual_detection_message(self):
        msg = self.DetectionArray(); msg.header.stamp = self.probe.get_clock().now().to_msg()
        msg.header.frame_id = 'undistorted_left_optical'; msg.sequence = 3; msg.width = 1280; msg.height = 1088
        msg.detector_available = True
        for i, name in enumerate(('fire', 'person', 'firefighter')):
            item = self.Detection(); item.class_id = i; item.class_name = name; item.confidence = .9
            item.bbox_xyxy = [10., 10., 20., 20.]; msg.detections.append(item)
        self.objects_pub.publish(msg); self.pump(.1)
        self.text('有几个人受困？')
        self.assertTrue(self.received['answer'])
        answer = self.received['answer'][-1]
        self.assertEqual(answer['backend'], 'structured_rules')
        self.assertEqual(answer['facts']['visible_person_count'], 1)
        self.assertIn('无法仅凭本帧确认', answer['text'])
        self.assertFalse(answer['command_execution_allowed'])
        self.assertFalse(self.received['mission']); self.assertFalse(self.received['actuator'])


if __name__ == '__main__':
    unittest.main()
