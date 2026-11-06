"""Health report meaning: inactive features must not masquerade as verified IO."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('rescue_self_check', Path(__file__).resolve().parents[1]/'tools/self_check.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def baseline():
    values = {
        '/mission/state': {'mode': 'standby'},
        '/actuators/state': {'dry_run': True, 'enabled': False, 'error': ''},
        '/fpga/link': {'dry_run': True, 'status_fresh': False},
        '/diagnostics/camera': {'ok': True},
        '/diagnostics/perception': {'detector_active': False, 'capture_resolution': [1280, 1088],
                                  'model_resolution': [640, 640], 'jpeg_quality': 90,
                                  'enabled_classes': ['fire', 'firefighter', 'person']},
        '/diagnostics/building_ocr': {'enabled': True, 'active': False},
        '/voice/status': {'microphone_enabled': False, 'tts_enabled': False},
        '/stereo/depth_status': {'gate': 'disabled', 'status': 'disabled'},
    }
    return dict(environment={'ros_domain_id': '0', 'ros_localhost_only': '0'},
        services={unit: {'ActiveState': 'active'} for unit in MODULE.SERVICES},
        topics={name: dict(count=2, last_received_age_s=.1, value=values.get(name, {})) for name in MODULE.TOPICS},
        preview={'value': {'state': 'paused', 'stream': {'enabled': False, 'active': False}}}, llm={'ok': True})


class SelfCheckTests(unittest.TestCase):
    def checks(self, report):
        return {entry['name']: entry for entry in MODULE.evaluate(report)}

    def test_standby_is_wait_not_error_or_hardware_pass(self):
        checks = self.checks(baseline())
        self.assertNotIn('WARN', [v['status'] for v in checks.values()])
        for name in ('YOLO', 'FPGA UART', '云台/水泵', '楼牌 OCR', '语音输入/播报', '双目测距'):
            self.assertEqual(checks[name]['status'], 'WAIT')

    def test_stale_message_cannot_be_used_as_current_health(self):
        report = baseline()
        report['topics']['/diagnostics/camera']['last_received_age_s'] = 8
        checks = self.checks(report)
        self.assertEqual(checks['/diagnostics/camera']['status'], 'WARN')
        self.assertNotIn('相机输入', checks)

    def test_fixed_chassis_camera_does_not_require_old_low_head_pose(self):
        report = baseline()
        report['topics']['/diagnostics/road']['value'] = dict(fixed_camera_observation=True)
        self.assertNotIn('云台安装与低头姿态', self.checks(report))
        report['topics']['/diagnostics/road']['last_received_age_s'] = 8
        self.assertIn('云台安装与低头姿态', self.checks(report))

    def test_voice_five_second_heartbeat_not_falsely_stale(self):
        report = baseline()
        report['topics']['/voice/status']['last_received_age_s'] = 5.5
        self.assertEqual(self.checks(report)['/voice/status']['status'], 'PASS')

    def test_dry_run_false_missing_fpga_status_warns(self):
        report = baseline()
        report['topics']['/fpga/link']['value']['dry_run'] = False
        self.assertEqual(self.checks(report)['FPGA UART']['status'], 'WARN')

    def test_real_extension_uart_takes_priority_over_legacy_dry_bridge(self):
        report = baseline()
        report['topics']['/extension/state']['value'] = dict(real_uart=True, protocol_ready=True, status_fresh=True)
        self.assertEqual(self.checks(report)['FPGA UART']['status'], 'PASS')
        report['topics']['/extension/state']['value']['status_fresh'] = False
        self.assertEqual(self.checks(report)['FPGA UART']['status'], 'WARN')

    def test_enabled_motor_without_encoder_or_zero_is_not_ready(self):
        report = baseline()
        report['topics']['/actuators/state']['value'] = dict(enabled=True, dry_run=False, feedback_required=True)
        item = self.checks(report)['云台/水泵']
        self.assertEqual(item['status'], 'WAIT')
        self.assertIn('机械零位', item['message'])
        self.assertIn('编码器', item['message'])

    def test_missing_speaker_is_reported_even_when_tts_enabled(self):
        report = baseline()
        report['topics']['/voice/status']['value'] = dict(microphone_enabled=True, tts_enabled=True, tts_device_ready=False)
        self.assertEqual(self.checks(report)['语音输入/播报']['status'], 'WARN')

    def test_camera_and_detector_errors_not_hidden_by_standby(self):
        report = baseline()
        report['topics']['/diagnostics/camera']['value'] = {'ok': False, 'reason': 'no_recent_input'}
        report['topics']['/diagnostics/perception']['value']['stage_errors'] = {'capture': 'bad image'}
        checks = self.checks(report)
        self.assertEqual(checks['相机输入']['status'], 'WARN')
        self.assertEqual(checks['视觉管线']['status'], 'WARN')

    def test_missing_topic_is_wait_not_pass(self):
        report = baseline()
        del report['topics']['/mission/state']
        self.assertEqual(self.checks(report)['/mission/state']['status'], 'WAIT')

    def test_quality_mismatch_warns(self):
        report = baseline()
        report['topics']['/diagnostics/perception']['value']['jpeg_quality'] = 75
        self.assertEqual(self.checks(report)['当前画质配置']['status'], 'WARN')

    def test_wrong_domain_does_not_import_ros_or_sample(self):
        report = baseline()
        report['environment']['ros_domain_id'] = '71'
        MODULE.collect_ros(report, 1)
        self.assertIn('未在错误通信域', report['ros_error'])
        self.assertEqual(self.checks(report)['ROS 通信域']['status'], 'WARN')

    def test_service_failure_warns(self):
        report = baseline()
        report['services'][MODULE.SERVICES[0]] = {'ActiveState': 'failed'}
        self.assertEqual(self.checks(report)[MODULE.SERVICES[0]]['status'], 'WARN')

    def test_probe_uses_only_show(self):
        with patch.object(MODULE.subprocess, 'run') as run:
            run.return_value.stdout = '\n\n'.join('Id='+unit+'\nActiveState=active' for unit in MODULE.SERVICES)
            run.return_value.stderr = ''
            result = MODULE.probe_services()
        self.assertEqual(run.call_args.args[0][:2], ['systemctl', 'show'])
        self.assertTrue(all(value['ActiveState'] == 'active' for value in result.values()))


if __name__ == '__main__':
    unittest.main()
