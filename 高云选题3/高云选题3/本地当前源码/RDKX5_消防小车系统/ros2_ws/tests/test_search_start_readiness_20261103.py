"""Search-only startup is map-driven; stale vision never becomes fire evidence."""
import unittest

from test_area_navigation import Harness


class SearchStartReadinessTests(unittest.TestCase):
    def test_missing_stale_or_disabled_targets_do_not_block_search_only_start(self):
        for targets in ({}, {'frame_valid': False, 'capture_stamp': 100.},
                        {'frame_valid': True, 'capture_stamp': 90.}):
            with self.subTest(targets=targets):
                h = Harness()
                h.c.area.targets = dict(targets)
                self.assertEqual(h.c.area.readiness(h.now+.002, execute_fire=False), '')
                self.assertEqual(h.c.area.readiness(h.now+.002, execute_fire=True), 'region_live_yolo_required')
                h.start(execute_fire=False)
                self.assertTrue(h.c.area.search.active)
                self.assertFalse(h.c.area.search.execute_fire)
                self.assertEqual(h.c.area.navigation.active['phase'], 'resetting')
                self.assertEqual(h.c.area.targets, targets)

    def test_missing_or_stale_idle_auto_status_does_not_block_search_only(self):
        for auto in ({}, {'stamp': 90., 'active': False}):
            with self.subTest(auto=auto):
                h = Harness()
                h.c.auto_state = dict(auto)
                self.assertEqual(h.c.area.readiness(h.now+.002, execute_fire=False), '')
                self.assertEqual(h.c.area.readiness(h.now+.002, execute_fire=True), 'region_auto_controller_not_idle')

    def test_fresh_active_auto_and_explicit_mission_auto_or_pump_armed_still_block(self):
        for owner in ('auto', 'mission', 'pump_armed'):
            h = Harness()
            h.c.mission_state['auto_fire_active'] = False
            if owner == 'auto': h.c.auto_state['active'] = True
            elif owner == 'mission': h.c.mission_state['auto_fire_active'] = True
            else: h.c.mission_state['pump_armed'] = True
            self.assertEqual(h.c.area.readiness(h.now+.002, execute_fire=False), 'region_auto_controller_not_idle')

    def test_stale_active_auto_needs_newer_explicit_idle_mission_and_measured_off(self):
        h = Harness()
        h.c.auto_state = dict(stamp=90., active=True)
        self.assertEqual(h.c.area.readiness(h.now+.002, execute_fire=False), 'region_auto_controller_not_idle')
        h.c.mission_state['auto_fire_active'] = False
        self.assertEqual(h.c.area.readiness(h.now+.002, execute_fire=False), '')
        h.c.auto_state.pop('stamp')
        self.assertEqual(h.c.area.readiness(h.now+.002, execute_fire=False), 'region_auto_controller_not_idle')

    def test_search_start_preserves_pump_unknown_requested_manual_armed_and_estop_gates(self):
        for patch in ({'pump_on': True}, {'pump_on': None}, {'pump_feedback_fresh': False},
                      {'pump_requested_on': True}, {'manual_pump_active': True},
                      {'guard': {'armed': True}}, {'guard': {}}, {'stamp': 90.}, {'estop': True}):
            with self.subTest(patch=patch):
                h = Harness()
                h.c.area.targets = {}
                h.c.auto_state = {}
                h.c.actuator.update(patch)
                self.assertEqual(h.c.area.readiness(h.now+.002, execute_fire=False), 'region_pump_off_and_disarmed_required')

    def test_search_start_preserves_real_uart_start_mode_and_priority_requirements(self):
        cases = (
            (lambda c: setattr(c, 'real_uart', False), 'real_extension_uart_and_fresh_feedback_required'),
            (lambda c: c.numbered.host.update(selected=False), 'select_fpga_communication_navigation_page'),
            (lambda c: c.area.navigation.map['start_reference'].update(valid=False), 'mark_start_and_heading_on_fpga_or_hmi'),
            (lambda c: c.mission_state.update(operator_hold=True), 'region_priority_state_not_ready'),
            (lambda c: c.mission_state.update(emergency_stop=True), 'region_priority_state_not_ready'),
        )
        for change, expected in cases:
            h = Harness()
            h.c.area.targets = {}
            h.c.auto_state = {}
            change(h.c)
            self.assertEqual(h.c.area.readiness(h.now+.002, execute_fire=False), expected)


if __name__ == '__main__':
    unittest.main()
