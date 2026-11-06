import unittest

from rescue_vision.dialogue_workload import yield_optional_observation


class DialogueWorkloadTests(unittest.TestCase):
    def setUp(self):
        self.mission = dict(stamp=100., mode='inspect', motion_allowed=False,
            traffic_active=False, auto_fire_active=False, pump_armed=False)
        self.dialogue = dict(stamp=100., state='thinking', busy=True)

    def check(self, **kwargs):
        args = dict(mission=self.mission, mission_received=50., dialogue=self.dialogue,
            dialogue_received=50., now_ros=100.1, now_mono=50.1)
        args.update(kwargs)
        return yield_optional_observation(**args)

    def test_only_non_actuating_inspection_yields(self):
        self.assertTrue(self.check())
        for key, value in (('mode', 'fire'), ('mode', 'navigate'), ('mode', 'patrol'),
                ('mode', 'standby'), ('traffic_active', True), ('motion_allowed', True),
                ('auto_fire_active', True), ('pump_armed', True)):
            with self.subTest(key=key, value=value):
                self.assertFalse(self.check(mission=dict(self.mission, **{key: value})))

    def test_missing_or_malformed_safety_facts_never_pause_observers(self):
        for key in ('motion_allowed', 'traffic_active', 'auto_fire_active', 'pump_armed'):
            for value in (None, 0, '', []):
                self.assertFalse(self.check(mission=dict(self.mission, **{key: value})))
        self.assertFalse(self.check(mission=[]))

    def test_both_clocks_must_have_fresh_mission(self):
        self.assertFalse(self.check(mission_received=49.))
        self.assertFalse(self.check(mission=dict(self.mission, stamp=99.)))
        self.assertFalse(self.check(mission_received=51.))
        self.assertFalse(self.check(mission=dict(self.mission, stamp=101.)))

    def test_busy_publisher_failure_restores_work_without_manual_reset(self):
        # Keep the mission fresh while the dialogue publisher stops heartbeating.
        self.assertTrue(self.check())
        self.assertFalse(self.check(mission=dict(self.mission, stamp=103.),
            mission_received=53., now_ros=103.1, now_mono=53.1))
        self.assertFalse(self.check(dialogue_received=47.))
        self.assertFalse(self.check(dialogue=dict(self.dialogue, stamp=97.)))

    def test_complete_timeout_and_error_restore_work_immediately(self):
        for state in ('complete', 'error', 'idle', None):
            self.assertFalse(self.check(dialogue=dict(self.dialogue, state=state)))
        self.assertFalse(self.check(dialogue=dict(self.dialogue, busy=False)))

    def test_bad_timestamps_and_future_data_do_not_hold_pause(self):
        for value in (None, True, float('nan'), float('inf'), 101.):
            self.assertFalse(self.check(dialogue=dict(self.dialogue, stamp=value)))
        self.assertFalse(self.check(dialogue_received=51.))



if __name__ == '__main__':
    unittest.main()
