import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.edge_dialogue import system_answer, project_reference


class ExtensionDialogueTests(unittest.TestCase):
    def report(self, **values):
        state = dict(_received_stamp=100., stamp=100., real_uart=True, protocol_ready=True,
                     status_fresh=True, vehicle_stopped=False, task='semantic', **values)
        return dict(system_status=dict(extension=state,
                    fpga_link=dict(_received_stamp=100., stamp=100., dry_run=True)))

    def test_actual_protocol_precedes_old_disabled_bridge(self):
        text = system_answer('FPGA连接了吗', self.report(), 100.)
        self.assertIn('ov5640', text)
        self.assertNotIn('模拟模式', text)

    def test_stale_protocol_cannot_claim_connection(self):
        self.assertIn('暂无新鲜状态', system_answer('FPGA连接了吗', self.report(), 102.))

    def test_acceptance_does_not_mean_completed(self):
        r = self.report(active=dict(phase='waiting_ack', command_id='a'),
                        last_result=dict(accepted=True, completed=False, command_id='a'))
        self.assertIn('等待FPGA接收', system_answer('现在导航到哪里了', r, 100.))
        self.assertNotIn('已收到完成', system_answer('现在导航到哪里了', r, 100.))

    def test_completion_needs_same_command_and_measured_stop(self):
        r = self.report(active=dict(phase='completed', command_id='a'),
                        last_result=dict(completed=True, command_id='a'))
        self.assertNotIn('已收到完成', system_answer('现在导航到哪里了', r, 100.))
        r['system_status']['extension']['vehicle_stopped'] = True
        self.assertIn('已收到完成', system_answer('现在导航到哪里了', r, 100.))
        r['system_status']['extension']['last_result']['command_id'] = 'old'
        self.assertNotIn('已收到完成', system_answer('现在导航到哪里了', r, 100.))

    def test_other_tasks_cannot_claim_light_control(self):
        self.assertIn('尚未确认独立交通任务开启', system_answer('红绿灯现在怎么样', self.report(), 100.))

    def test_zebra_switch_requires_fresh_supported_hardware_feedback(self):
        r = self.report(capabilities={'zebra_switch': True}, zebra_detection_enabled=True)
        self.assertIn('识别开关已开启', system_answer('斑马线状态', r, 100.))
        r['system_status']['extension']['zebra_detection_enabled'] = False
        self.assertIn('识别开关已关闭', system_answer('斑马线状态', r, 100.))
        self.assertIn('开关状态未知', system_answer('斑马线状态', r, 102.))
        for field, value in [('status_fresh', False), ('real_uart', False),
                             ('protocol_ready', False), ('capabilities', {})]:
            bad = self.report(capabilities={'zebra_switch': True}, zebra_detection_enabled=True)
            bad['system_status']['extension'][field] = value
            self.assertIn('开关状态未知', system_answer('斑马线状态', bad, 100.), field)

    def test_mode_uses_extension_selection_and_no_stale_claim(self):
        r = self.report()
        r['system_status']['extension']['task'] = 'dialogue'
        self.assertIn('拓展二：对话与控制', system_answer('当前模式', r, 100.))
        self.assertIn('任务模式：未知', system_answer('当前模式', r, 102.))

    def test_project_explanation_separates_dialogue_from_motion(self):
        text = project_reference('FPGA和X5怎么分工')
        self.assertIn('拓展一提供视觉观察与语音场景问答', text)
        self.assertIn('纯对话不需要UART', text)
        self.assertIn('自主导航不再因斑马线限速', text)


if __name__ == '__main__':
    unittest.main()
