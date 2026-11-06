"""One explicit selected-target burst, tested without hardware or model I/O."""
import unittest

from rescue_vision.selected_fire_round import selected_fire_round_reason
from rescue_vision.task_interaction import TaskRouter, parse_task_text
from rescue_vision.voice_verification import needs_independent_verification, verify_command


TEXT = '对选定火焰自动瞄准喷水并关水'


def state(now=100.):
    identity = dict(target_id=2, registry_session='session', selection_generation=3)
    return dict(
        mission=dict(stamp=now, mode='fire', vehicle_stopped=True, gimbal_ready=False,
                     mounting_verified=True, gimbal_transport='x5'),
        selection=dict(capture_stamp=now, valid=True, **identity),
        proposal=dict(capture_stamp=now, valid=True, aim_mode='fixed_camera_3d',
                      calibration_ready=True, landing_compensated=True,
                      water_impact_calibration_verified=True, landing_calibration_id='measured',
                      geometry_locked=False, pump_candidate=False, **identity),
        actuator=dict(stamp=now, enabled=True, dry_run=False, status_fresh=True,
                      feedback_fresh=True, reference_confirmed_this_start=True,
                      actual_angles_deg=[0., 0.], pump_enabled=True, pump_ready=True,
                      pump_on=False, pump_feedback_fresh=True, guard=dict(armed=False, state='DISARMED')),
        extension=dict(stamp=now, real_uart=True, status_fresh=True, vehicle_stopped=True,
                       busy=False, task='dialogue', dialogue_enabled=True))


def request(source='web', **changes):
    return dict(dict(action='extinguish_selected', text=TEXT, source=source,
                     command_id='one-click:1', issued_at=100., ttl_s=2.), **changes)


class SelectedFireRoundTests(unittest.TestCase):
    def test_exact_rule_text_and_read_only_negation(self):
        for text in (TEXT, '对选定火焰自动瞄准喷水关水', '一键瞄准喷水关水',
                     '一键瞄准喷水并关水', '对选定火焰执行一轮灭火'):
            self.assertEqual(parse_task_text(text)['action'], 'extinguish_selected', text)
        for text in ('不要' + TEXT, TEXT + '吗', '如果' + TEXT):
            self.assertEqual(parse_task_text(text)['action'], 'dialogue', text)

    def test_extension_round_can_begin_before_alignment_and_binds_selected_identity(self):
        out = TaskRouter(extension_enabled=True).route(request(), 100., **state())
        self.assertEqual(out['auto']['action'], 'start')
        self.assertEqual(out['auto']['preferred_target_id'], 2)
        self.assertEqual(out['auto']['preferred_registry_session'], 'session')
        self.assertIs(out['auto']['return_to_standby'], False)
        self.assertEqual(out['actuator']['action'], 'disarm')
        self.assertEqual(out['actuator']['preserve_auto_run_id'], 'one-click:1')
        self.assertIsNone(out['extension'])
        self.assertIsNone(out['navigation'])
        self.assertIsNone(out['mission'])
        self.assertEqual(selected_fire_round_reason(100., extension_enabled=True, **state()), '')

    def test_http_click_identity_must_be_complete_strict_and_unchanged(self):
        identity = dict(expected_target_id=2, expected_registry_session='session', expected_selection_generation=3)
        self.assertTrue(TaskRouter(extension_enabled=True).route(request(**identity), 100., **state())['accepted'])
        for mutation in (dict(expected_target_id=9), dict(expected_target_id=True),
                         dict(expected_registry_session='old'), dict(expected_selection_generation=4),
                         dict(expected_selection_generation=True)):
            value = dict(identity, **mutation)
            with self.assertRaisesRegex(ValueError, 'selected_fire_changed_before_submit'):
                TaskRouter(extension_enabled=True).route(request(**value), 100., **state())
        with self.assertRaisesRegex(ValueError, 'selected_fire_changed_before_submit'):
            TaskRouter(extension_enabled=True).route(request(expected_target_id=2), 100., **state())

    def test_missing_physical_evidence_and_busy_or_moving_state_never_arm(self):
        variants = [('mission', 'vehicle_stopped', False), ('mission', 'auto_fire_active', True),
                    ('mission', 'operator_hold', True), ('mission', 'stamp', 99.),
                    ('mission', 'mounting_verified', False), ('extension', 'busy', True),
                    ('extension', 'real_uart', False), ('extension', 'vehicle_stopped', False),
                    ('selection', 'capture_stamp', 99.), ('selection', 'valid', False),
                    ('actuator', 'dry_run', True), ('actuator', 'reference_confirmed_this_start', False),
                    ('actuator', 'actual_angles_deg', [0., float('nan')]),
                    ('actuator', 'status_fresh', False), ('actuator', 'feedback_fresh', False),
                    ('actuator', 'pump_enabled', False), ('actuator', 'pump_ready', False),
                    ('actuator', 'pump_on', True), ('actuator', 'pump_feedback_fresh', False),
                    ('actuator', 'guard', dict(armed=True)),
                    ('actuator', 'guard', dict(armed=False, state='COOLDOWN')),
                    ('proposal', 'capture_stamp', 99.), ('proposal', 'target_id', 5),
                    ('proposal', 'selection_generation', 4), ('proposal', 'calibration_ready', False),
                    ('proposal', 'landing_compensated', False),
                    ('proposal', 'water_impact_calibration_verified', False),
                    ('proposal', 'landing_calibration_id', '')]
        for item, key, value in variants:
            with self.subTest(item=item, key=key):
                context = state(); context[item][key] = value
                with self.assertRaises(ValueError):
                    TaskRouter(extension_enabled=True).route(request(), 100., **context)

    def test_voice_water_authority_needs_independent_same_action_even_high_confidence(self):
        self.assertTrue(needs_independent_verification(TEXT, dict(confidence_available=True), {}))
        accepted = verify_command(TEXT, dict(text=TEXT, result=[dict(conf=.99)]), {})
        self.assertTrue(accepted['verified'], accepted)
        denied = verify_command(TEXT, dict(text='锁定火焰2', result=[dict(conf=.99)]), {})
        self.assertFalse(denied['verified'])
        for proof in (None, {}, dict(verified=False)):
            with self.assertRaisesRegex(ValueError, 'selected_fire_round_independent_asr_required'):
                TaskRouter(extension_enabled=True).route(request('voice', voice_verification=proof), 100., **state())
        self.assertTrue(TaskRouter(extension_enabled=True).route(
            request('voice', voice_verification=accepted), 100., **state())['accepted'])

    def test_generic_auto_fire_and_payload_smuggling_still_blocked(self):
        with self.assertRaisesRegex(ValueError, 'manual_spray_command_required'):
            TaskRouter(extension_enabled=True).route(request(action='auto_fire', text=''), 100., **state())
        for text in ('', '不要' + TEXT, '锁定火焰2'):
            with self.assertRaises(ValueError):
                TaskRouter(extension_enabled=True).route(request(text=text), 100., **state())

    def test_select_fire_does_not_require_motor_readiness(self):
        context = state()
        context['actuator'] = {}
        context['mission']['mounting_verified'] = False
        out = TaskRouter(extension_enabled=True).route(
            request(action='select_fire', text='锁定火焰2', target_id=2), 100., **context)
        self.assertEqual(out['fire_select']['target_id'], 2)
        self.assertEqual(out['actuator']['action'], 'disarm')


if __name__ == '__main__':
    unittest.main()
