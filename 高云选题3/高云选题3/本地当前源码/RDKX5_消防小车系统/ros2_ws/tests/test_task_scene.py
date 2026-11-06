import unittest
import copy
from unittest.mock import patch
from rescue_vision.task_scene import build_scene, grounded_answer, LocalIntentModel, SprayHistory
from rescue_vision.mipi_building_ocr import observation_signature


class TaskSceneTest(unittest.TestCase):
    def setUp(self):
        self.detection = dict(capture_stamp=100., sequence=7, coordinate_frame='left', detector_available=True,
            detections=[dict(class_name=n, confidence=.9) for n in ('fire', 'fire', 'person', 'firefighter')])

    def test_visible_counts_not_casualties_or_total(self):
        report = build_scene(self.detection, 100.)
        self.assertEqual(report['visible_fire_count'], 2)
        self.assertEqual(report['visible_person_count'], 1)
        self.assertIsNone(report['casualty_count'])
        self.assertFalse(report['extinguished_verified'])
        self.assertFalse(report['building_association_verified'])
        answer = grounded_answer('现在几个人受困？', report, 100.)
        self.assertIn('当前画面', answer); self.assertIn('无法仅凭本帧确认', answer)

    def test_stale_and_disabled_are_unknown_not_zero(self):
        for now, available in ((102., True), (100., False)):
            self.detection['detector_available'] = available
            report = build_scene(self.detection, now)
            self.assertIsNone(report['visible_fire_count'])
            self.assertEqual(report['fire_status'], 'unknown')
            self.assertIn('没有新鲜有效', grounded_answer('报告场景', report, now))

    def test_same_frame_ocr_alone_does_not_associate_all_view_objects(self):
        ocr = dict(stamp=100., sequence=7, coordinate_frame='left', building=dict(confirmed=True, number='1', name='安居楼'))
        self.assertFalse(build_scene(self.detection, 100., ocr=ocr)['building_association_verified'])
        for key, value in [('sequence', 8), ('stamp', 99.9), ('coordinate_frame', 'right')]:
            wrong = dict(ocr); wrong[key] = value
            self.assertFalse(build_scene(self.detection, 100., ocr=wrong)['building_association_verified'])

    def ocr(self, capture=100., now=102., preview=False):
        mission = dict(stamp=now, mode='inspect', traffic_active=False,
            motion_allowed=False, gimbal_ready=not preview, gimbal_target_deg=[0, 0])
        stamp_ns = round(capture*1e9)
        matched = dict(copy.deepcopy(self.detection), capture_stamp=capture,
            capture_stamp_ns=stamp_ns)
        ocr = dict(valid=True, stamp=capture, capture_stamp=capture, processed_stamp=now,
            capture_stamp_ns=stamp_ns, sequence=7, coordinate_frame='left',
            same_capture_verified=True, unverified_pose=preview, fixed_camera_preview=preview,
            observation_signature=observation_signature(mission, now, 0., require_settled_pose=not preview),
            matched_detection=matched, object_building_association_verified=False,
            building=dict(confirmed=True, status='confirmed', number='01', name='安居楼',
                building_id='A', stamp=capture, frame_id=str(stamp_ns)))
        return ocr, mission

    def test_delayed_ocr_and_newer_counts_remain_separate(self):
        ocr, mission = self.ocr()
        current = dict(self.detection, capture_stamp=102., sequence=37,
            detections=[dict(class_name='person', confidence=.99)]*3)
        report = build_scene(current, 102., ocr=ocr, mission=mission)
        self.assertEqual(report['visible_person_count'], 3)
        self.assertEqual(report['visible_fire_count'], 0)
        self.assertEqual(report['source_sequence'], 37)
        sign = report['recent_building_observation']
        self.assertTrue(sign['confirmed'])
        self.assertEqual(sign['source_sequence'], 7)
        self.assertEqual(sign['capture_stamp'], 100.)
        self.assertEqual(sign['number'], '01')
        self.assertFalse(report['building_association_verified'])
        self.assertIsNone(report['building_number'])
        answer = grounded_answer('当前场景', report, 102.)
        self.assertIn('最近确认看到01号楼安居楼的楼牌', answer)
        self.assertIn('当前画面可见3名', answer)
        self.assertIn('尚未确认目标都属于这栋楼', answer)

    def test_recent_sign_available_when_latest_yolo_is_stale(self):
        ocr, mission = self.ocr()
        report = build_scene(self.detection, 102., ocr=ocr, mission=mission)
        self.assertIsNone(report['visible_fire_count'])
        self.assertIn('01号楼', grounded_answer('现在是几号楼', report, 102.))
        self.assertIn('没有新鲜有效', grounded_answer('当前场景', report, 102.))

    def test_sign_expiry_uses_capture_not_processing_or_answer_time(self):
        ocr, mission = self.ocr(now=103.1)
        self.assertIsNone(build_scene(self.detection, 103.1, ocr=ocr, mission=mission)['recent_building_observation'])
        ocr, mission = self.ocr()
        report = build_scene(self.detection, 102., ocr=ocr, mission=mission)
        self.assertIn('楼号尚未确认', grounded_answer('楼牌是什么', report, 103.1))

    def test_sign_invalidations_mode_traffic_and_pose_changes(self):
        for change in (dict(mode='patrol'), dict(traffic_active=True), dict(motion_allowed=True),
                       dict(gimbal_target_deg=[10, 0]), dict(stamp=99.)):
            ocr, mission = self.ocr()
            mission.update(change)
            self.assertIsNone(build_scene(self.detection, 102., ocr=ocr, mission=mission)['recent_building_observation'])
        ocr, mission = self.ocr()
        for bad in (dict(ocr, valid=False), dict(ocr, building={'confirmed': False}),
                    dict(ocr, same_capture_verified=False), dict(ocr, matched_detection=None)):
            self.assertIsNone(build_scene(self.detection, 102., ocr=bad, mission=mission)['recent_building_observation'])

    def test_sign_exact_capture_schema_mismatch_rejected(self):
        for key, value in (('capture_stamp_ns', 100_000_000_001), ('sequence', 8),
                           ('coordinate_frame', 'right'), ('capture_stamp', 100.1)):
            ocr, mission = self.ocr()
            ocr['matched_detection'][key] = value
            self.assertIsNone(build_scene(self.detection, 102., ocr=ocr, mission=mission)['recent_building_observation'])

    def test_fixed_camera_sign_marked_unverified_pose(self):
        ocr, mission = self.ocr(preview=True)
        report = build_scene(self.detection, 102., ocr=ocr, mission=mission)
        self.assertTrue(report['recent_building_observation']['unverified_pose'])
        self.assertFalse(report['building_association_verified'])
        self.assertIn('云台姿态尚未验证', grounded_answer('楼牌是什么', report, 102.))

    def test_spray_off_never_means_extinguished(self):
        report = build_scene(self.detection, 100., actuator=dict(stamp=100., pump_on=False, dry_run=False))
        text = grounded_answer('灭火完成了吗？', report, 100.)
        self.assertIn('指令为关闭', text); self.assertIn('不能证明已经扑灭', text)
        self.assertIn('状态未知', grounded_answer('水泵关了吗？', report, 102.))

    def test_dry_run_never_claims_actual_water(self):
        report = build_scene(self.detection, 100., actuator=dict(stamp=100., pump_on=True, dry_run=True))
        self.assertIn('模拟输出', grounded_answer('现在喷水了吗', report, 100.))
        self.assertFalse(report['water_flow_verified'])

    def test_no_detection_not_extinguished(self):
        self.detection['detections'] = []
        report = build_scene(self.detection, 100.)
        self.assertIn('不能排除', grounded_answer('有火焰吗？', report, 100.))
        self.assertFalse(report['extinguished_verified'])

    def test_model_disabled_never_network_and_backend_honest(self):
        with patch('urllib.request.build_opener', side_effect=AssertionError('network forbidden')):
            intent, backend, error = LocalIntentModel({'enabled': False}).classify('有几个消防员？')
        self.assertEqual(intent, 'firefighters'); self.assertEqual(backend, 'structured_rules'); self.assertIsNone(error)

    def test_model_rejects_external_endpoints(self):
        for endpoint in ('http://example.com/v1/chat/completions', 'https://127.0.0.1/v1/chat/completions',
                         'http://127.0.0.1/v1/chat/completions?url=external', 'http://user@localhost/v1/chat/completions'):
            with self.assertRaises(ValueError):
                LocalIntentModel({'endpoint': endpoint})

    def test_model_failure_falls_back_without_hallucinations(self):
        with patch('urllib.request.build_opener', side_effect=TimeoutError):
            intent, backend, error = LocalIntentModel({'enabled': True}).classify('有几个火焰？')
        self.assertEqual(intent, 'fire'); self.assertEqual(backend, 'structured_rules'); self.assertEqual(error, 'TimeoutError')

    def test_spray_history_observed_transition_and_post_review(self):
        for dry in (True, False):
            history = SprayHistory()
            history.update(dict(stamp=100., pump_on=False, dry_run=dry), 100.)
            history.update(dict(stamp=100.1, pump_on=True, dry_run=dry, armed_target=[2, 3, 'session']), 100.1)
            event = history.update(dict(stamp=100.2, pump_on=False, dry_run=dry, guard={'reason': 'operator_disarm'}), 100.2)
            self.assertEqual(event['started_stamp'], 100.1); self.assertEqual(event['ended_stamp'], 100.2)
            self.assertEqual(event['target_identity']['target_id'], 2)
            self.assertEqual(event['dry_run'], dry); self.assertFalse(event['water_flow_verified'])
            report = build_scene(self.detection, 100.2, mission={'stamp': 100.2, 'mode': 'fire'}, last_spray=event)
            self.assertEqual(report['task_state'], 'post_spray_review')
            self.assertIn('等待复查', grounded_answer('喷水完成了吗？', report, 100.2))

    def test_spray_history_no_creation_on_initial_on_or_stale(self):
        history = SprayHistory()
        self.assertIsNone(history.update(dict(stamp=100., pump_on=True), 100.))
        self.assertIsNone(history.update(dict(stamp=100.1, pump_on=False), 100.1))
        self.assertIsNone(history.update(dict(stamp=100.2, pump_on=True), 102.))
        self.assertIsNone(history.update(dict(stamp=102., pump_on=False), 102.))
        self.assertIsNone(history.update(dict(stamp=103., pump_on=True), 103.))

    def test_task_states_and_spray_control_not_flow(self):
        self.assertEqual(build_scene(self.detection, 100.)['task_state'], 'unknown')
        self.assertEqual(build_scene(self.detection, 100., mission={'stamp': 100., 'mode': 'patrol'})['task_state'], 'awaiting_observation')
        self.assertEqual(build_scene(self.detection, 100., mission={'stamp': 100., 'mode': 'inspect'})['task_state'], 'observing')
        self.assertEqual(build_scene(self.detection, 100., mission={'stamp': 100., 'mode': 'fire'})['task_state'], 'aiming')
        report = build_scene(self.detection, 100., actuator={'stamp': 100., 'pump_on': True, 'dry_run': True})
        self.assertEqual(report['task_state'], 'spraying'); self.assertFalse(report['water_flow_verified'])


if __name__ == '__main__':
    unittest.main()
