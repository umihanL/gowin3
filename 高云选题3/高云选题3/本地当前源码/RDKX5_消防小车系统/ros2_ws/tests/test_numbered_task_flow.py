"""Console -> router -> fixed-frame replies -> actual Mission arrival callbacks."""
import json
import queue
import time
import unittest
import test_numbered_arrival_scan as scan_fixture
import test_numbered_navigation as hardware_fixture
import test_numbered_interaction as console_fixture
from rescue_vision.task_interaction import TaskRouter
from rescue_vision.auto_fire import AutoFireRound
from rescue_vision.edge_dialogue import system_answer


class NumberedTaskFlowTests(unittest.TestCase):
    setUp = scan_fixture.NumberedArrivalTests.setUp

    @classmethod
    def setUpClass(cls):
        # Mission's fixture patches the process-wide time.monotonic. Import
        # the console before that patch: queue binds monotonic at import time,
        # and importing it under the virtual clock breaks later ASR timeouts.
        console_fixture.NumberedHttpTests.setUpClass()
        cls.console_module = console_fixture.NumberedHttpTests.module
        cls.real_queue_clock = queue.time

    @classmethod
    def tearDownClass(cls):
        if queue.time is not cls.real_queue_clock or queue.time is not time.monotonic:
            raise AssertionError('Numbered flow fixture leaked its virtual clock into queue')

    def start_console_navigation(self, profile):
        self.core = hardware_fixture.ready()
        self.core.task = profile
        self.router = TaskRouter(extension_enabled=True)
        self.node.extension_enabled = True
        self.node.now = 100.01
        self.console = self.console_module.PresentationState(
            monotonic=lambda: self.node.now, wall=lambda: self.node.now)
        self.console.ros.update(available=True, router_ready=True)
        self.refresh()
        self.console.enqueue(request_id='confirm', point_command=dict(action='confirm_point_start', point_id=2, confirmed=True, start_heading_deg=90))
        self.console.drain(self.publish_command, lambda: self.node.now)
        self.node.now = 100.02
        self.refresh()
        # This fixture covers the explicit map-button -> FPGA path. Natural
        # speech/text goes through the asynchronous Qwen node covered separately.
        self.console.enqueue(point_command=dict(action='navigate_point', start_point=2, target_point=3), request_id='navigate')
        self.console.drain(self.publish_command, lambda: self.node.now)
        self.assertEqual(self.core.numbered.active['phase'], 'setting_start')
        self.assertEqual(self.node.mode, 'navigate')
        self.assertFalse(self.node.auto_pub.values)
        hardware_fixture.ack(self.core, 100.03)
        hardware_fixture.ack(self.core, 100.035, bytes((0,0x84,3,0)))
        task = self.core.numbered.active['task_id']
        hardware_fixture.ack(self.core, 100.04, bytes((0, 3, 1, task)))
        hardware_fixture.query(self.core, hardware_fixture.wire.POINT_STATUS, bytes((0, 3, 2, 0)), 100.05)
        hardware_fixture.query(self.core, hardware_fixture.wire.HEARTBEAT, bytes((0, 0, 19, task)), 100.06)
        hardware_fixture.ack(self.core, 100.07)
        hardware_fixture.telemetry(self.core, 100.08)
        self.observe_navigation(100.09)
        self.assertEqual(self.node.mode, 'navigate')
        self.assertFalse(self.node.auto_pub.values)

    def refresh(self):
        self.console.update('extension', self.core.state(self.node.now))
        self.console.update('mission', dict(stamp=self.node.now, mode=self.node.mode, vehicle_stopped=True))

    def publish_command(self, kind, value):
        self.assertEqual(kind, 'command')
        routed = self.router.route(value, self.node.now, extension=self.core.state(self.node.now))
        self.assertTrue(routed['accepted'])
        self.assertIsNone(routed['navigation'])
        if routed['extension']:
            self.core.request(routed['extension'], self.node.now)
        effects, self.core.effects = self.core.effects, []
        for name, payload in effects:
            if name == 'mission':
                self.node.request(scan_fixture.Message(data=json.dumps(payload)))

    def observe_navigation(self, now):
        self.node.now = now
        self.node.navigation_state(scan_fixture.Message(data=json.dumps(self.core.numbered.state(now))))
        self.node.tick()

    def finish(self):
        hardware_fixture.done(self.core, 100.2)
        # Event alone must neither enter inspect nor scan.
        self.observe_navigation(100.201)
        self.assertEqual(self.node.mode, 'navigate')
        self.assertFalse(self.node.auto_pub.values)
        hardware_fixture.telemetry(self.core, 100.21, motion=3, run=False, stopped=True)
        self.observe_navigation(100.22)
        self.assertEqual(self.node.mode, 'inspect')
        self.assertTrue(self.node.state_pub.values[-1]['yolo_requested'])

    def test_extension1_to_arrival_then_operator_question(self):
        self.start_console_navigation('vision')
        self.finish()
        self.assertFalse(self.node.auto_pub.values)
        self.refresh()
        self.console.enqueue(text='分析当前火场', request_id='analysis')
        replies = []
        self.console.drain(lambda kind, value: replies.append(self.router.route(value, self.node.now,
            extension=self.core.state(self.node.now))), lambda: self.node.now)
        self.assertEqual(len(replies), 1)
        self.assertIsNotNone(replies[0]['question'])
        self.assertIsNone(replies[0]['actuator'])

    def test_extension2_to_arrival_enables_yolo_without_scan_selection_or_spray(self):
        self.start_console_navigation('dialogue')
        self.finish()
        self.assertEqual(self.node.auto_pub.values, [])
        self.assertEqual(self.core.numbered.active['arrival_policy'], 'manual')
        self.observe_navigation(100.23)
        self.assertEqual(self.node.auto_pub.values, [])

    def test_wrong_completion_or_failed_status_does_not_change_task(self):
        self.start_console_navigation('dialogue')
        task = self.core.numbered.active['task_id']
        hardware_fixture.done(self.core, 100.2, task=task + 1)
        hardware_fixture.telemetry(self.core, 100.21, motion=3, run=False, stopped=True)
        self.observe_navigation(100.22)
        self.assertEqual(self.node.mode, 'navigate')
        self.assertFalse(self.node.auto_pub.values)

    def test_scene_system_answer_uses_actual_numbered_status(self):
        self.start_console_navigation('dialogue')
        state = self.core.state(100.09)
        state['_received_stamp'] = 100.09
        report = dict(system_status=dict(extension=state))
        answer = system_answer('现在导航到哪里了', report, 100.09)
        self.assertIn('3号点', answer)
        self.assertIn('正在执行导航', answer)
        self.assertNotIn('当前没有正在执行', answer)
        self.finish()
        state = self.core.state(100.22); state['_received_stamp'] = 100.22
        report['system_status']['extension'] = state
        self.assertIn('到达3号点及停稳', system_answer('现在导航到哪里了', report, 100.22))
        self.assertNotIn('到达3号点及停稳', system_answer('现在导航到哪里了', report, 102.))


if __name__ == '__main__':
    unittest.main()
