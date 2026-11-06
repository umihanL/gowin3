"""Automatic round and actuator authority tests, entirely without hardware/ROS."""
import unittest

from rescue_vision.auto_fire import AutoFireRound
from rescue_vision.auto_fire_gate import AutoArmFence
from rescue_vision.local_actuator_guard import PumpGuard


class AutoFireRoundTests(unittest.TestCase):
    def start(self, **kwargs):
        self.round = AutoFireRound(**kwargs)
        self.round.request(dict(action='start', command_id='run1', issued_at=100., ttl_s=2.), 100.)
        return self.round

    def evidence(self, now, mode='fire'):
        identity = dict(target_id=7, selection_generation=2, registry_session='camera-session')
        return dict(mission=dict(stamp=now, mode=mode, vehicle_stopped=True, gimbal_ready=True),
                    targets=dict(capture_stamp=now, valid=True, registry_session='camera-session',
                        targets=[dict(target_id=7, selectable=True, bbox_xyxy=[100,100,200,200])]),
                    selection=dict(capture_stamp=now, valid=True, **identity),
                    proposal=dict(capture_stamp=now, valid=True, pump_candidate=True, **identity),
                    actuator=dict(stamp=now, enabled=True, pump_enabled=True, pump_on=False,
                        auto_authorized_run_id='run1', guard=dict(state='DISARMED', armed=False)))

    def aim(self):
        self.assertEqual(self.round.tick(100.3, **self.evidence(100.3, 'inspect')), [])
        outputs = self.round.tick(100.5, **self.evidence(100.5, 'inspect'))
        self.assertEqual(self.round.phase, 'AIM')
        self.assertEqual(dict(outputs)['select']['target_id'], 7)

    def test_full_round_search_select_arm_once_review_report_standby(self):
        self.start(); self.aim()
        emitted = self.round.tick(100.6, **self.evidence(100.6))
        arm = dict(emitted)['actuator']
        self.assertEqual(arm['action'], 'arm_pump')
        self.assertEqual(arm['auto_run_id'], 'run1')
        self.assertEqual(arm['max_bursts'], 1)
        evidence = self.evidence(100.8); evidence['actuator']['pump_on'] = True
        self.assertEqual(self.round.tick(100.8, **evidence), [])
        outputs = self.round.tick(101.1, **self.evidence(101.1))
        self.assertEqual(self.round.phase, 'REVIEW')
        self.assertTrue(dict(outputs)['actuator']['retire_auto_run'])
        report = self.round.tick(102.2, **self.evidence(102.2, 'inspect'))
        self.assertEqual(dict(report)['question']['text'], '报告当前场景')
        finished = self.round.tick(102.8, **self.evidence(102.8, 'inspect'))
        self.assertEqual(dict(finished)['mission']['mode'], 'standby')
        self.assertFalse(self.round.active)
        self.assertFalse(self.round.snapshot(102.8)['extinguished_verified'])
        for now in (103, 105, 110):
            self.assertEqual(self.round.tick(now, **self.evidence(now)), [])

    def test_transient_bad_lock_does_not_consume_only_arm_attempt(self):
        self.start(); self.aim()
        bad = self.evidence(100.6); bad['proposal']['pump_candidate'] = False
        self.assertEqual(self.round.tick(100.6, **bad), [])
        self.assertEqual(self.round.phase, 'AIM')
        arm = dict(self.round.tick(100.7, **self.evidence(100.7)))['actuator']
        self.assertEqual(arm['action'], 'arm_pump')
        self.assertEqual(self.round.arm_attempt, 2)

    def test_waits_for_authority_ack_and_real_stop(self):
        self.start()
        moving = self.evidence(100.5, 'inspect'); moving['mission']['vehicle_stopped'] = False
        self.assertEqual(self.round.tick(100.5, **moving), [])
        self.assertEqual(self.round.phase, 'SEARCH')
        self.round.tick(100.6, **self.evidence(100.6, 'inspect'))
        self.round.tick(100.7, **self.evidence(100.7, 'inspect'))
        missing = self.evidence(100.8); missing['actuator'].pop('auto_authorized_run_id')
        self.assertEqual(self.round.tick(100.8, **missing), [])
        self.assertEqual(self.round.reason, 'waiting_for_actuator_auto_authority')
        self.assertEqual(self.round.arm_attempt, 0)

    def test_no_fire_scan_finishes_in_standby_without_any_arm(self):
        self.start(); outputs = []
        for now in range(101, 114):
            state = self.evidence(now, 'inspect'); state['targets']['targets'] = []
            outputs += self.round.tick(now, **state)
        self.assertFalse(self.round.active)
        self.assertEqual(self.round.reason, 'no_selectable_fire_in_scan')
        self.assertIn(('mission', {'mode': 'standby'}), outputs)
        self.assertFalse(any(k == 'actuator' and v['action'] == 'arm_pump' for k,v in outputs))

    def test_hardware_disabled_never_arms(self):
        self.start(); self.aim()
        state = self.evidence(100.6); state['actuator']['pump_enabled'] = False
        result = self.round.tick(100.6, **state)
        self.assertFalse(self.round.active)
        self.assertEqual(self.round.reason, 'pump_hardware_not_enabled')
        self.assertEqual(dict(result)['actuator']['action'], 'disarm')

    def test_cancel_and_delayed_start_never_restore_authority(self):
        self.start(); self.aim()
        cancelled = self.round.request(dict(action='cancel', issued_at=-100), 100.6)
        self.assertTrue(dict(cancelled)['actuator']['retire_auto_run'])
        self.assertEqual(self.round.tick(101, **self.evidence(101)), [])
        with self.assertRaises(ValueError):
            self.round.request(dict(action='start', command_id='delayed', issued_at=100.5, ttl_s=2), 100.7)
        self.round.request(dict(action='start', command_id='new', issued_at=101, ttl_s=2), 101)
        self.assertTrue(self.round.active)

    def test_stop_traffic_stale_selection_and_timeout_cancel(self):
        cases = [('operator_hold', {'operator_hold': True}, 'higher_priority_task'),
                 ('fpga_estop', {'emergency_stop': True}, 'higher_priority_task'),
                 ('traffic', {'traffic_active': True}, 'higher_priority_task'),
                 ('mode', {'mode': 'standby'}, 'mission_changed')]
        for name, change, reason in cases:
            with self.subTest(name=name):
                self.start(); self.aim()
                state = self.evidence(101.1); state['mission'].update(change)
                self.round.tick(101.1, **state)
                self.assertFalse(self.round.active)
                self.assertEqual(self.round.reason, reason)
        self.start(); self.aim()
        state = self.evidence(101.6); state['selection']['capture_stamp'] = 100.
        self.round.tick(101.6, **state)
        self.assertEqual(self.round.reason, 'selected_fire_lost')
        self.start(timeout_s=5.)
        self.round.tick(105, **self.evidence(105, 'inspect'))
        self.assertEqual(self.round.reason, 'automatic_round_timeout')

    def test_malformed_targets_are_ignored_and_invalid_time_closes_round(self):
        self.start()
        self.round.tick(100.3, **self.evidence(100.3, 'inspect'))
        state = self.evidence(100.5, 'inspect')
        state['targets']['targets'] = [None, {}, {'selectable': True},
             dict(target_id=True, selectable=True, bbox_xyxy=[1,2,3,4]),
             dict(target_id=3, selectable=True, bbox_xyxy=[1,2,float('nan'),4])]
        self.assertEqual(self.round.tick(100.5, **state), [])
        outputs = self.round.tick(float('nan'), **state)
        self.assertFalse(self.round.active)
        self.assertEqual(dict(outputs)['actuator']['issued_at'], 100.5)

    def test_scan_dwell_begins_after_slow_gimbal_settles(self):
        for duration in (1., 3.):
            with self.subTest(movement_seconds=duration):
                self.start()
                # Even a detected target during motion cannot be selected.
                for now in (100.3, 100.+duration):
                    moving = self.evidence(now, 'inspect')
                    moving['mission']['gimbal_ready'] = False
                    self.assertEqual(self.round.tick(now, **moving), [])
                    self.assertEqual(self.round.scan_index, 0)
                ready_at = 100.+duration+.1
                state = self.evidence(ready_at, 'inspect')
                self.assertEqual(self.round.tick(ready_at, **state), [])
                self.assertEqual(self.round.scan_ready_since, ready_at)
                # A fresh delivery of a frame captured before readiness is stale
                # for this pose even though it passes the general .5 s lease.
                state = self.evidence(ready_at+.1, 'inspect')
                state['targets']['capture_stamp'] = ready_at-.01
                self.assertEqual(self.round.tick(ready_at+.1, **state), [])
                self.assertEqual(self.round.scan_index, 0)
                state = self.evidence(ready_at+.2, 'inspect')
                selected = self.round.tick(ready_at+.2, **state)
                self.assertEqual(dict(selected)['select']['target_id'], 7)

    def test_next_pose_waits_for_ack_guard_and_full_ready_dwell(self):
        self.start()
        for now in (100.3, 101.31):
            state = self.evidence(now, 'inspect'); state['targets']['targets'] = []
            self.round.tick(now, **state)
        self.assertEqual(self.round.scan_index, 1)
        self.assertIsNone(self.round.scan_ready_since)
        # Stale ready=True immediately after publishing a new pan offset must
        # not select the old view or consume any observation dwell.
        self.assertEqual(self.round.tick(101.4, **self.evidence(101.4, 'inspect')), [])
        self.assertIsNone(self.round.scan_ready_since)
        for now in (101.6, 103.):
            state = self.evidence(now, 'inspect'); state['mission']['gimbal_ready'] = False
            self.round.tick(now, **state)
        for now in (103.1, 103.9):
            state = self.evidence(now, 'inspect'); state['targets']['targets'] = []
            self.round.tick(now, **state)
            self.assertEqual(self.round.scan_index, 1)
        state = self.evidence(104.11, 'inspect'); state['targets']['targets'] = []
        self.round.tick(104.11, **state)
        self.assertEqual(self.round.scan_index, 2)

    def test_unconfirmed_burst_is_not_reported_as_completed_fire(self):
        self.start(); self.aim()
        self.round.tick(100.6, **self.evidence(100.6))
        self.round.tick(102.7, **self.evidence(102.7))
        self.assertEqual(self.round.reason, 'burst_not_confirmed_or_too_long')
        self.assertFalse(self.round.snapshot(102.7)['extinguished_verified'])

    def test_unknown_pump_after_high_cannot_complete_round(self):
        self.start(); self.aim()
        self.round.tick(100.6, **self.evidence(100.6))
        high = self.evidence(100.8); high['actuator']['pump_on'] = True
        self.round.tick(100.8, **high)
        unknown = self.evidence(101.1); unknown['actuator']['pump_on'] = None
        self.assertEqual(self.round.tick(101.1, **unknown), [])
        self.assertEqual(self.round.phase, 'BURST')
        self.assertFalse(self.round.snapshot(101.1)['burst_completed'])
        unknown['mission']['stamp'] = unknown['selection']['capture_stamp'] = 102.7
        unknown['actuator']['stamp'] = 102.7
        self.round.tick(102.7, **unknown)
        self.assertEqual(self.round.reason, 'burst_not_confirmed_or_too_long')
        self.assertIsNone(self.round.snapshot(102.7)['confirmed_pump_off_at'])

    def test_fpga_requires_fresh_output_evidence_for_both_edges(self):
        self.start(); self.aim()
        self.round.tick(100.6, **self.evidence(100.6))
        def feedback(now, on, fresh):
            value = self.evidence(now)
            value['actuator'].update(pump_backend='fpga_uart', pump_on=on,
                                     pump_feedback_fresh=fresh)
            return value
        self.round.tick(100.7, **feedback(100.7, True, False))
        self.assertFalse(self.round.saw_pump)
        self.round.tick(100.8, **feedback(100.8, True, True))
        self.assertTrue(self.round.saw_pump)
        self.round.tick(100.9, **feedback(100.9, False, False))
        self.assertEqual(self.round.phase, 'BURST')
        self.round.tick(101., **feedback(101., False, True))
        self.assertEqual(self.round.phase, 'REVIEW')
        state = self.round.snapshot(101.)
        self.assertTrue(state['burst_completed'])
        self.assertEqual(state['confirmed_pump_off_at'], 101.)
        self.assertFalse(state['extinguished_verified'])

    def test_patrol_round_preserves_supervisor_mode_after_success(self):
        self.round = AutoFireRound()
        self.round.request(dict(action='start', command_id='run1', issued_at=100.,
                                ttl_s=2., return_to_standby=False), 100.)
        self.aim()
        self.round.tick(100.6, **self.evidence(100.6))
        high = self.evidence(100.8); high['actuator']['pump_on'] = True
        self.round.tick(100.8, **high)
        self.round.tick(101.1, **self.evidence(101.1))
        self.round.tick(102.2, **self.evidence(102.2, 'inspect'))
        finished = self.round.tick(102.8, **self.evidence(102.8, 'inspect'))
        self.assertNotIn('mission', dict(finished))
        self.assertNotIn('待机', dict(finished)['speech'])
        state = self.round.snapshot(102.8)
        self.assertTrue(state['burst_completed'])
        self.assertFalse(state['active'])
        self.assertEqual(state['reason'], 'one_target_round_finished')

    def test_return_mode_is_strict_boolean_and_resets_per_round(self):
        for value in (0, 1, None, 'false'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                AutoFireRound().request(dict(action='start', command_id='run1',
                    issued_at=100., ttl_s=2., return_to_standby=value), 100.)
        controller = AutoFireRound()
        controller.request(dict(action='start', command_id='one', issued_at=100.,
                                ttl_s=2., return_to_standby=False), 100.)
        controller.request(dict(action='cancel'), 101.)
        controller.request(dict(action='start', command_id='two', issued_at=102., ttl_s=2.), 102.)
        self.assertTrue(controller.return_to_standby)
        self.assertFalse(controller.snapshot(102.)['burst_completed'])


class AutoArmFenceTests(unittest.TestCase):
    def start(self, fence, cid='run1', now=100.):
        fence.request(dict(action='start', command_id=cid, issued_at=now, ttl_s=2), now)

    def arm(self, run_id='run1', now=100.1):
        return dict(action='arm_pump', auto_run_id=run_id, issued_at=now, ttl_s=.5)

    def test_cancel_rejects_delayed_old_arm_even_with_new_command_id(self):
        fence = AutoArmFence(); self.start(fence)
        self.assertTrue(fence.check_arm(self.arm(), 100.1))
        fence.request(dict(action='cancel', issued_at=-100), 100.2)
        self.assertFalse(fence.check_arm(self.arm(now=100.3), 100.3))
        self.assertFalse(fence.authorized('run1', 100.3))

    def test_cancel_rejects_queued_start_and_old_start_replay(self):
        fence = AutoArmFence(); self.start(fence)
        fence.manual_disarm(100.2)
        for cid in ('run1', 'delayed-different-id'):
            with self.assertRaises(ValueError):
                fence.request(dict(action='start', command_id=cid, issued_at=100.1, ttl_s=2), 100.3)
        self.start(fence, 'run2', 100.4)
        self.assertTrue(fence.check_arm(self.arm('run2',100.5),100.5))
        self.assertFalse(fence.check_arm(self.arm('run1',100.5),100.5))

    def test_startup_disarm_can_arrive_before_or_after_start(self):
        for before in (True, False):
            fence = AutoArmFence()
            if before: fence.manual_disarm(100., preserve_run_id='run1')
            self.start(fence)
            if not before: fence.manual_disarm(100.1, preserve_run_id='run1')
            self.assertTrue(fence.check_arm(self.arm(now=100.2), 100.2))
        fence.manual_disarm(100.3)
        fence.manual_disarm(100.4, preserve_run_id='run1')
        self.assertFalse(fence.check_arm(self.arm(now=100.4), 100.4))

    def test_completed_run_retirement_does_not_cancel_a_new_run(self):
        fence = AutoArmFence(); self.start(fence)
        fence.retire('run1')
        self.start(fence, 'run2', 100.5)
        fence.retire('run1')
        self.assertTrue(fence.authorized('run2', 100.6))
        self.assertFalse(fence.authorized('run2', 191))

    def test_one_burst_budget_survives_missing_supervisor_updates(self):
        guard = PumpGuard(dict(max_bursts=3, burst_s=.01, stable_lock_s=.01, cooldown_s=.5))
        guard.arm(0., 3., max_bursts=1)
        rising, was_on = 0, False
        for i in range(301):
            state = guard.update(i/100., mission_ok=True, command_ok=True, aim_ok=True)
            rising += int(state['pump_on'] and not was_on); was_on = state['pump_on']
        self.assertEqual(rising, 1)
        self.assertEqual(state['state'], 'EXHAUSTED')
        guard.disarm()
        manual = guard.arm(4., 3.)
        self.assertEqual(manual['session_max_bursts'], 3)


if __name__ == '__main__':
    unittest.main()
