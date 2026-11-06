"""Operator task workflow: real gates with fake transports; no board I/O."""
import json
import unittest
import test_numbered_navigation as nav_fixture
import test_numbered_interaction as ingress
import test_traffic_task_control as mission_fixture
from rescue_vision.task_interaction import parse_task_text, TaskRouter
from rescue_vision.voice_verification import verify_command


class OperatorFlowTests(unittest.TestCase):
    def test_explicit_location_and_navigation_sentence(self):
        for text in ('你现在在二号位置，请前往六号位置后停下',
                     '当前位于第2号点位前往6号点后停车',
                     '小车现在在2号位置，请你导航到6号位置并停车'):
            value=parse_task_text(text)
            self.assertEqual(value['action'],'navigate_point')
            self.assertEqual((value['start_point'],value['target_point']),(2,6))
            self.assertTrue(value['start_declared'])
        with self.assertRaisesRegex(ValueError, 'numbered_navigation_separate_actions'):
            parse_task_text('你现在在2号位置请前往6号位置然后喷水')
        for text in ('如果你现在在2号位置请前往6号位置后停下',
                     '你现在在2号位置请前往6号位置后停下吗'):
            self.assertEqual(parse_task_text(text)['action'],'dialogue')

    def test_independent_asr_must_agree_on_location_declaration(self):
        result=lambda text:dict(text=text,result=[dict(conf=.95)])
        self.assertTrue(verify_command('你现在在二号位置请前往六号位置后停下',
            result('现在在2号位置前往6号位置后停下'),None)['verified'])
        for text in ('现在在3号位置前往6号位置后停下','从2号点到6号点'):
            self.assertFalse(verify_command('现在在2号位置前往6号位置后停下',result(text),None)['verified'])

    def test_voice_can_select_task_from_standby_without_fpga(self):
        for text in ('开启任务1','开始拓展任务二'):
            result=TaskRouter(extension_enabled=True).route(ingress.command(text),100.,extension={})
            self.assertTrue(result['accepted'])
            self.assertIn('等待' if result['action']=='extension_dialogue' else 'YOLO',result['speech'])
            self.assertIsNone(result['question'])
            self.assertEqual(result['actuator']['action'],'disarm')
            self.assertEqual(result['auto']['action'],'cancel')

    def test_declared_start_is_operator_input_but_cannot_skip_live_stop(self):
        text='你现在在2号位置，车头朝地图左侧，请前往6号位置后停下'
        request=ingress.command(text)
        state=ingress.extension(can_navigate_point=False,
            point_reference=dict(confirmed=False),point_navigation_reason='confirm_physical_numbered_start')
        with self.assertRaisesRegex(ValueError, 'numbered_start_heading_required'):
            TaskRouter(extension_enabled=True).route(
                ingress.command('你现在在2号位置，请前往6号位置后停下'),100.01,extension=state)
        out=TaskRouter(extension_enabled=True).route(request,100.01,extension=state)
        self.assertEqual(out['extension']['arrival_policy'],'manual')
        core=nav_fixture.ready()
        core.task='dialogue'
        core.request(out['extension'],100.01)
        self.assertEqual(core.numbered.active['start_source'],'operator_statement')
        self.assertEqual(core.numbered.active['start_heading_deg'],180.)
        self.assertFalse(core.numbered.active['physical_position_verified'])
        self.assertEqual(core.numbered.active['phase'],'setting_start')
        self.assertFalse(any(f[4]==nav_fixture.wire.START for f in core.outbox))
        for changes in ({'vehicle_stopped':False},{'phase':1,'vehicle_stopped':False}):
            other=nav_fixture.ready();other.status.update(changes)
            with self.assertRaisesRegex(ValueError,'stop'):
                other.request(out['extension'],100.01)

    def test_declared_start_cannot_be_injected_into_another_sentence(self):
        router=TaskRouter(extension_enabled=True)
        with self.assertRaisesRegex(ValueError,'text_point_mismatch'):
            router.route(ingress.command('从2号点到6号点',start_declared=True),100.,extension=ingress.extension())
        core=nav_fixture.ready()
        bad=ingress.command('从2号点到6号点',start_declared=True)
        with self.assertRaisesRegex(ValueError,'explicit_start_statement'):
            core.request(bad,100.01)

    def test_legacy_auto_spray_shortcuts_rejected_for_extensions(self):
        for text in ('开启自动灭火','到达后自动灭火','一直前进直到火焰居中然后自动灭火'):
            with self.assertRaisesRegex(ValueError,'manual_spray_command_required'):
                TaskRouter(extension_enabled=True).route(ingress.command(text),100.,extension=ingress.extension())
        stopped=TaskRouter(extension_enabled=True).route(ingress.command('停止喷水'),100.,extension={})
        self.assertEqual(stopped['actuator']['action'],'disarm')


class ObservationAndAimTests(unittest.TestCase):
    setUp=mission_fixture.TrafficTaskTests.setUp

    def ready_observation(self):
        node=self.node
        node.extension_enabled=True
        node.mount['mounting_verified']=True
        node.event(mission_fixture.Message(event_type=mission_fixture.Message.STATUS,vehicle_stopped=True))
        node.link_state(mission_fixture.Message(data=json.dumps(dict(stamp=100.,enabled=True,dry_run=False,
            status_fresh=True,feedback_fresh=True,reference_confirmed_this_start=True,
            actual_angles_deg=[72.,84.]))))
        return node

    def test_inspect_never_homes_even_with_working_actuators(self):
        node=self.ready_observation()
        node.request(mission_fixture.Message(data='{"mode":"inspect"}'))
        for _ in range(3):node.tick()
        self.assertEqual(node.gimbal_pub.values,[])
        self.assertTrue(node.state_pub.values[-1]['yolo_requested'])

    def test_explicit_fire_starts_at_measured_pose(self):
        node=self.ready_observation()
        node.request(mission_fixture.Message(data='{"mode":"fire"}'))
        node.tick()
        self.assertEqual(len(node.gimbal_pub.values),1)
        msg=node.gimbal_pub.values[0]
        self.assertEqual((msg.owner,msg.pan_deg,msg.tilt_deg),('fire',72.,84.))
        self.assertEqual(node.target,(72.,84.))
        self.assertFalse(node.auto_pub.values)

    def test_no_pose_seed_on_unverified_reference_or_stale_feedback(self):
        for bad in ({'reference_confirmed_this_start':False},{'feedback_fresh':False},
                    {'dry_run':True},{'actual_angles_deg':[72.,float('nan')]},
                    {'actual_angles_deg':[999.,84.]}):
            node=self.ready_observation();node.link.update(bad)
            node.mode='fire';node.target=None;node.gimbal_pub.values=[]
            node.tick()
            self.assertFalse(node.gimbal_pub.values,bad)
            self.assertIsNone(node.target)


if __name__=='__main__':unittest.main()
