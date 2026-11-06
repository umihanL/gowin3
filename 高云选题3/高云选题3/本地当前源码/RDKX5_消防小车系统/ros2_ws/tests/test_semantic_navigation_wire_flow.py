"""Rules -> production ROS router -> numbered UART handshake, without hardware."""
import json
import struct
import types
import unittest
from unittest.mock import patch
import test_numbered_navigation as hardware_fixture
import test_semantic_navigation_node as node_fixture
from rescue_vision.task_interaction import parse_task_text
wire=hardware_fixture.wire

class ExtensionPublisher(node_fixture.Publisher):
    def __init__(self,owner):
        super().__init__();self.owner=owner
    def publish(self,value):
        super().publish(value);self.owner.core.request(value,self.owner.clock)

class RuleNavigationWireFlowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        node_fixture.RuleNodeTests.setUpClass();cls.module=node_fixture.RuleNodeTests.module
    def setUp(self):
        self.core=hardware_fixture.ready();self.core.task='dialogue';self.core.outbox.clear()
        node_fixture.RuleNodeTests.setUp(self)
        self.clock=100.01;self.refresh()
        self.node.output_pubs['extension']=ExtensionPublisher(self)
    def refresh(self):
        self.node.cache['extension']=(self.core.state(self.clock),self.clock)
        self.node.cache['mission']=(dict(mode='inspect'),self.clock)
    def receive(self,request):
        self.node.receive(types.SimpleNamespace(data=json.dumps(request)))
        return self.node.result.values[-1]
    def confirm_reference(self,heading=180):
        self.receive(dict(action='confirm_point_start',point_id=8,confirmed=True,start_heading_deg=heading,
            source='web',manual_point_command=True,command_id='reference',issued_at=self.clock,ttl_s=3.))
        self.assertTrue(self.node.result.values[-1]['accepted'],self.node.result.values[-1])
        self.assertTrue(self.core.numbered.reference['confirmed']);self.assertFalse(self.core.outbox);self.refresh()
    def navigate(self,text):
        request=dict(parse_task_text(text),source='voice',voice_verification=dict(verified=True),
            command_id='speech-navigation',issued_at=self.clock,ttl_s=3.)
        with patch('urllib.request.build_opener',side_effect=AssertionError('navigation used model')):
            return self.receive(request)

    def assert_handshake(self, expected_heading, expected_little_endian):
        self.assertTrue(self.node.result.values[-1]['accepted'], self.node.result.values[-1])
        self.assertFalse(self.node.output_pubs['question'].values, 'Navigation must not turn into chat')
        self.assertEqual(self.core.numbered.active['phase'], 'setting_start')
        start_cell = self.core.numbered.points[8]['planner_cell']
        self.assertEqual(self.core.outbox[-1][4:9], bytes((wire.SET_START, *start_cell, 0, 0)))
        self.assertFalse(any(frame[4] == wire.START for frame in self.core.outbox))

        hardware_fixture.ack(self.core, 100.06)
        self.assertEqual(wire.SET_START_HEADING, 0x24)
        self.assertEqual(self.core.outbox[-1][4:9], bytes((0x24,)) + expected_little_endian + bytes(2))
        self.assertEqual(struct.unpack('<h', self.core.outbox[-1][5:7])[0], expected_heading * 10)
        self.assertFalse(any(frame[4] == wire.SELECT_POINT for frame in self.core.outbox))

        hardware_fixture.ack(self.core, 100.07, b'\x00' + expected_little_endian + b'\x00')
        task = self.core.numbered.active['task_id']
        self.assertEqual(self.core.outbox[-1][4:9], bytes((wire.SELECT_POINT, 4, task, 0, 0)))
        hardware_fixture.ack(self.core, 100.08, bytes((0, 4, 1, task)))
        self.assertEqual(self.core.numbered.active['phase'], 'planning')
        self.assertFalse(any(frame[4] == wire.START for frame in self.core.outbox))
        hardware_fixture.query(self.core, wire.POINT_STATUS, bytes((0, 4, 2, 0)), 100.09)
        hardware_fixture.query(self.core, wire.HEARTBEAT, bytes((0, 0, 19, task)), 100.10)
        self.assertEqual(self.core.outbox[-1][4:9], bytes((wire.START, 1, task, 0, 0)))
        hardware_fixture.ack(self.core, 100.11)
        self.assertEqual(self.core.numbered.active['phase'], 'running')

        motion_commands = (wire.SET_START, wire.SET_START_HEADING, wire.SELECT_POINT, wire.START)
        self.assertEqual([frame[4] for frame in self.core.outbox if frame[4] in motion_commands],
                         list(motion_commands))
        self.assertEqual(self.core.numbered.active['start_heading_deg'], expected_heading)

    def test_original_voice_left_heading_reaches_positive_little_endian_handshake(self):
        result=self.navigate('你现在在8号点位，车头朝左，请导航前往至4号点位。')
        self.assertTrue(result['accepted'],result);self.assertFalse(result['rule_navigation']['model_called'])
        self.assert_handshake(180,b'\x08\x07')
    def test_missing_heading_uses_confirmed_reference(self):
        self.confirm_reference(180)
        result=self.navigate('你现在在8号点位，请导航前往至4号点位。')
        self.assertTrue(result['accepted'],result);self.assert_handshake(180,b'\x08\x07')
    def test_spoken_negative_heading_uses_signed_little_endian(self):
        result=self.navigate('你现在在8号点位，车头朝向负九十度，请导航前往至4号点位。')
        self.assertTrue(result['accepted'],result);self.assert_handshake(-90,b'\x7c\xfc')
    def test_current_position_uses_only_confirmed_reference(self):
        self.confirm_reference(180)
        result=self.navigate('从当前位置请前往4号点后停下')
        self.assertTrue(result['accepted'],result);self.assert_handshake(180,b'\x08\x07')
    def test_missing_heading_or_mismatched_route_start_never_sends_set_start(self):
        result=self.navigate('你现在在8号点位，请前往4号点位。')
        self.assertFalse(result['accepted']);self.assertFalse(self.core.outbox)
        self.node.router.seen.clear();self.confirm_reference(180)
        result=self.navigate('从7号点车头朝左前往4号点')
        self.assertFalse(self.core.numbered.active);self.assertFalse(self.core.outbox)
    def test_mutated_payload_is_rejected_before_any_uart_frame(self):
        request=dict(parse_task_text('你现在在8号点车头朝左请前往4号点'),source='web',
            command_id='mutated',issued_at=self.clock,ttl_s=3.,target_point=3)
        result=self.receive(request)
        self.assertEqual(result['reason'],'text_point_mismatch');self.assertFalse(self.core.outbox)

if __name__=='__main__':unittest.main()
