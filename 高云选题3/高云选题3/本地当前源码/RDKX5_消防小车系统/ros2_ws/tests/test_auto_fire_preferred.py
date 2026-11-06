import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.auto_fire import AutoFireRound


class RequiredFireTests(unittest.TestCase):
    def start(self, **overrides):
        controller = AutoFireRound()
        request = dict(action='start', command_id='seek-auto:123', issued_at=100., ttl_s=2.,
                       preferred_target_id=7, preferred_registry_session='capture-a')
        request.update(overrides)
        controller.request(request, 100.)
        return controller

    def observation(self, now, include_target=True, session='capture-a'):
        targets = [dict(target_id=8, selectable=True, bbox_xyxy=[0, 0, 400, 400])]
        if include_target:
            targets.append(dict(target_id=7, selectable=True, bbox_xyxy=[100, 100, 140, 180]))
        return dict(mission=dict(stamp=now, mode='inspect', vehicle_stopped=True, gimbal_ready=True),
                    targets=dict(capture_stamp=now, valid=True, registry_session=session, targets=targets))

    def test_seek_target_is_not_replaced_by_larger_flame(self):
        controller = self.start()
        controller.tick(100.3, **self.observation(100.3))
        out = controller.tick(100.5, **self.observation(100.5))
        self.assertEqual(dict(out)['select']['target_id'], 7)
        self.assertEqual(controller.chosen, ('capture-a', 7))
        self.assertEqual(controller.reason, 'selected_required_fire')

    def test_lost_target_does_not_scan_and_spray_another_flame(self):
        controller = self.start()
        controller.tick(100.3, **self.observation(100.3))
        self.assertEqual(controller.tick(100.5, **self.observation(100.5, False)), [])
        out = controller.tick(101.4, **self.observation(101.4, False))
        self.assertFalse(controller.active)
        self.assertEqual(controller.reason, 'required_fire_lost')
        self.assertEqual(dict(out)['actuator']['action'], 'disarm')
        self.assertEqual(controller.scan_index, 0)

    def test_reused_id_after_camera_restart_is_a_different_flame(self):
        controller = self.start()
        controller.tick(100.3, **self.observation(100.3))
        controller.tick(101.4, **self.observation(101.4, session='capture-b'))
        self.assertFalse(controller.active)
        self.assertEqual(controller.reason, 'required_fire_lost')

    def test_invalid_identity_is_rejected_before_start(self):
        for overrides in ({'preferred_target_id':True}, {'preferred_target_id':0},
                          {'preferred_registry_session':''}, {'preferred_registry_session':None}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                self.start(**overrides)


if __name__ == '__main__':
    unittest.main()
