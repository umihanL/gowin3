"""Patrol's initial Mission transition, real callbacks with fake ROS transport."""
import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

import test_traffic_task_control as fixture
from rescue_vision.auto_fire import AutoFireRound

ROOT = Path(__file__).resolve().parents[1]


class PatrolMissionOrderingTests(unittest.TestCase):
    def setUp(self):
        fixture.TrafficTaskTests.setUp(self)
        modules = {name: types.ModuleType(name) for name in
            ('rclpy', 'rclpy.node', 'std_msgs', 'std_msgs.msg', 'rescue_vision.node_common')}
        modules['rclpy.node'].Node = fixture.Node
        modules['std_msgs.msg'].String = fixture.Message
        common = modules['rescue_vision.node_common']
        common.param = lambda node, name, default: default
        common.publish_json = lambda pub, value: pub.publish(value)
        common.spin = lambda _: None
        spec = importlib.util.spec_from_file_location('rescue_vision._patrol_auto_order_test',
            ROOT/'src/rescue_vision/rescue_vision/auto_fire_node.py')
        self.auto_module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(self.auto_module)
        self.auto_node = self.auto_module.AutoFire.__new__(self.auto_module.AutoFire)
        self.auto_node.controller = AutoFireRound()
        self.auto_node.now = lambda: self.node.now
        self.auto_node.pubs = {key: fixture.Publisher() for key in
                              ('mission', 'actuator', 'select', 'speech', 'question')}
        self.auto_node.state_pub = fixture.Publisher()
        self.node.mode = 'navigate'

    def start(self, patrol=True):
        request = dict(action='start', command_id='route-round-1', issued_at=100.,
                       ttl_s=2., return_to_standby=not patrol)
        self.auto_node.emit(self.auto_node.controller.request(request, 100.))
        return request

    def tick(self, now, mode='navigate', **changes):
        self.node.now = now
        evidence = dict(stamp=now, mode=mode, vehicle_stopped=True, gimbal_ready=False)
        evidence.update(changes)
        outputs = self.auto_node.controller.tick(now, mission=evidence)
        self.auto_node.emit(outputs)
        return outputs

    def test_inspect_before_grant_is_rejected_then_reasserted_with_same_authority(self):
        request = self.start()
        first = self.auto_node.pubs['mission'].values[-1]
        self.assertEqual(first['auto_run_id'], request['command_id'])
        self.node.request(fixture.Message(data=json.dumps(first)))
        self.assertEqual(self.node.mode, 'navigate')
        self.assertIsNone(self.node.auto_authority.active_run)
        self.assertEqual(self.tick(100.1), [])
        self.node.auto_request(fixture.Message(data=json.dumps(request)))
        self.assertTrue(self.node.auto_authority.authorized(request['command_id'],100.1))
        self.tick(100.21)
        repeated = self.auto_node.pubs['mission'].values[-1]
        self.assertEqual(repeated['auto_run_id'], request['command_id'])
        self.node.request(fixture.Message(data=json.dumps(repeated)))
        self.assertEqual(self.node.mode, 'inspect')
        self.tick(100.3, mode='inspect')
        before = len(self.auto_node.pubs['mission'].values)
        self.tick(100.6, mode='navigate')
        self.assertEqual(len(self.auto_node.pubs['mission'].values), before)
        self.assertEqual(self.auto_node.controller.reason, 'mission_changed')

    def test_retries_are_rate_limited_and_never_grant_authority(self):
        self.start()
        for now in (100.1,100.21,100.3,100.42,100.5,100.63,100.7,100.84,100.95):
            outputs=self.tick(now)
            for key, value in outputs:
                if key == 'mission':
                    self.node.request(fixture.Message(data=json.dumps(
                        self.auto_node.pubs['mission'].values[-1])))
            self.assertIsNone(self.node.auto_authority.active_run)
            self.assertEqual(self.node.mode,'navigate')
        self.assertEqual(len(self.auto_node.pubs['mission'].values),5)  # initial + four retries
        self.tick(101.)
        self.assertFalse(self.auto_node.controller.active)
        self.assertEqual(self.auto_node.controller.reason,'initial_inspect_not_confirmed')
        self.assertEqual(len(self.auto_node.pubs['mission'].values),5)

    def test_hold_traffic_and_estop_never_reassert_inspect(self):
        for key in ('operator_hold','traffic_active','emergency_stop'):
            with self.subTest(key=key):
                self.auto_node.controller=AutoFireRound()
                self.auto_node.pubs['mission'].values.clear()
                self.start()
                self.tick(100.21,**{key:True})
                self.assertEqual(len(self.auto_node.pubs['mission'].values),1)
                self.assertEqual(self.auto_node.controller.reason,'higher_priority_task')

    def test_default_round_does_not_gain_patrol_retry_behavior(self):
        self.start(patrol=False)
        self.assertEqual(self.tick(100.21),[])
        self.tick(100.6)
        self.assertEqual(self.auto_node.controller.reason,'mission_changed')
        self.assertEqual(len(self.auto_node.pubs['mission'].values),1)


if __name__=='__main__':
    unittest.main()
