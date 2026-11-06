import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from interaction_start_status import describe


class StartupStatusTests(unittest.TestCase):
    def state(self, actuator, manual=None, fresh=True):
        return {'topics': {'actuators': {'fresh': fresh, 'data': actuator},
                           'gimbal_manual': {'fresh': fresh, 'data': manual or {}}}}

    def test_reports_current_fault_even_when_router_is_ready(self):
        text = '\n'.join(describe(self.state({'error': 'actual_deg=-16.3125',
            'enabled': True, 'dry_run': False}, {'can_start': True})))
        self.assertIn('actual_deg=-16.3125', text)
        self.assertNotIn('当前可以接受角度指令', text)
        self.assertIn('正式云台正在占用', text)

    def test_stale_fault_does_not_masquerade_as_live_fault(self):
        text = '\n'.join(describe(self.state({'error': 'OLD_ERROR',
            'enabled': True, 'dry_run': False}, fresh=False)))
        self.assertIn('尚未收到新鲜', text)
        self.assertNotIn('OLD_ERROR', text)
        self.assertNotIn('正式云台正在占用', text)

    def test_boot_reference_is_not_inferred_from_enabled_flag(self):
        text = '\n'.join(describe(self.state({'enabled': True, 'dry_run': False})))
        self.assertIn('本次开机参考尚未确认', text)

    def test_real_ready_routes_to_production_panel(self):
        text = '\n'.join(describe(self.state({'enabled': True, 'dry_run': False,
            'reference_confirmed_this_start': True}, {'can_start': True})))
        self.assertIn('当前可以接受角度指令', text)
        self.assertIn('设备调试 → 云台到角度', text)

    def test_real_pending_readback_routes_to_reference_confirmation(self):
        text = '\n'.join(describe(self.state({'enabled': True, 'dry_run': False,
            'reference_confirmed_this_start': False, 'raw_feedback_fresh': True})))
        self.assertIn('双轴真实串口回读已在线', text)
        self.assertIn('摆好机械零位后点击确认', text)
        self.assertNotIn('当前可以接受角度指令', text)

    def test_busy_does_not_claim_ready(self):
        text = '\n'.join(describe(self.state({'enabled': True, 'dry_run': False,
            'reference_confirmed_this_start': True}, {'can_start': False,
            'ready_reason': 'manual_gimbal_running'})))
        self.assertIn('manual_gimbal_running', text)
        self.assertNotIn('当前可以接受角度指令', text)


if __name__ == '__main__':
    unittest.main()
