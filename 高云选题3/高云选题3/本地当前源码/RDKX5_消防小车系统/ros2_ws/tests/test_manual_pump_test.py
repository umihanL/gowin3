"""Finite manual test controller: observed output transitions, no I/O."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.manual_pump_test import ManualPumpTest


class ManualPumpTests(unittest.TestCase):
    def setUp(self):
        self.test = ManualPumpTest(True)
        self.context = dict(mission_fresh=True, vehicle_stopped=True, idle_mode=True,
            extension_idle=True, estop=False, fault='', other_owner=False)
        self.feedback = dict(pump_feedback_fresh=True, pump_on=False, pump_ready=True,
            pump_feedback_stamp=100., pump_feedback_command_id='startup-off')

    def request(self, **kw):
        v=dict(action='start',command_id='click-1',issued_at=100.,ttl_s=2.,
            duration_ms=400,operator_confirmed=True);v.update(kw)
        return self.test.request(v,wall_now=100.,now=10.,feedback=self.feedback,context=self.context)

    def tick(self, wall=100.1, **kw):
        self.feedback.update(kw)
        return self.test.update(now=wall-90.,wall_now=wall,feedback=self.feedback,context=self.context)

    def start(self):
        self.assertEqual(self.request(),'off')
        self.test.submitted('prepare-wire',100.,enabled=False)
        self.assertEqual(self.tick(100.01,pump_feedback_stamp=100.01,
            pump_feedback_command_id='prepare-wire'),'on')
        self.test.submitted('on-wire',100.01,enabled=True)

    def test_stop_latch_requires_new_matched_off_before_one_on(self):
        self.feedback.update(pump_ready=False,pump_reason_code=5,pump_lockout=True,
            pump_scope=True,pump_link_ok=True,pump_wheels_still=True)
        self.assertEqual(self.request(),'off')
        self.test.submitted('prepare-wire',100.,enabled=False)
        self.assertIsNone(self.tick(100.01,pump_feedback_stamp=99.9))
        self.assertEqual(self.test.phase,'preparing_off')
        self.assertIsNone(self.tick(100.1,pump_feedback_stamp=100.1,
            pump_feedback_command_id='prepare-wire'))
        self.assertEqual(self.tick(100.2,pump_ready=True,pump_lockout=False,
            pump_reason_code=0,pump_feedback_stamp=100.2),'on')
        self.test.submitted('on-wire',100.2,enabled=True)
        self.assertIsNone(self.tick(100.3,pump_on=True,pump_ready=False,
            pump_feedback_stamp=100.3,pump_feedback_command_id='on-wire'))

    def test_preparation_expiry_or_external_stop_never_produces_on(self):
        for external_stop in (False,True):
            with self.subTest(external_stop=external_stop):
                self.setUp();self.assertEqual(self.request(),'off')
                self.test.submitted('prepare-wire',100.,enabled=False)
                self.assertEqual(self.tick(100.1 if external_stop else 100.81,
                    pump_feedback_stamp=100.1 if external_stop else 100.81,
                    pump_feedback_command_id=None if external_stop else 'prepare-wire'),'off')
                self.assertIsNone(self.test.preparation_deadline)
                self.assertNotEqual(self.test.phase,'waiting_on')

    def test_only_fresh_matched_high_then_matched_off_counts_as_complete(self):
        self.start()
        self.tick(pump_on=True,pump_ready=False,pump_feedback_stamp=100.1,pump_feedback_command_id='on-wire')
        self.assertTrue(self.test.saw_pump_on)
        self.assertEqual(self.tick(100.41),'off')
        self.test.submitted('off-wire',100.41,enabled=False)
        self.tick(100.42,pump_on=False,pump_feedback_stamp=100.40,pump_feedback_command_id='on-wire')
        self.assertTrue(self.test.blocking)
        self.tick(100.5,pump_on=False,pump_feedback_stamp=100.5,pump_feedback_command_id='off-wire')
        self.assertEqual(self.test.phase,'completed');self.assertFalse(self.test.blocking)
        self.assertTrue(self.test.confirmed_pump_off)
        self.assertFalse(self.test.readiness(10.5,self.feedback,self.context)[0])

    def test_old_low_does_not_release_pending_owner_and_no_high_is_not_success(self):
        self.start();self.tick()
        self.assertTrue(self.test.blocking)
        self.assertFalse(self.test.saw_pump_on)
        self.assertEqual(self.tick(100.41),'off');self.test.submitted('off-wire',100.41,enabled=False)
        self.tick(100.5,pump_feedback_command_id='off-wire',pump_feedback_stamp=100.5)
        self.assertEqual(self.test.phase,'failed');self.assertIn('high_not_observed',self.test.reason)

    def test_no_fresh_low_keeps_navigation_blocked_even_after_timeout(self):
        self.start();self.tick(100.41);self.test.submitted('off-wire',100.41,enabled=False)
        self.tick(101.3,pump_feedback_fresh=False)
        self.assertEqual(self.test.phase,'failed');self.assertTrue(self.test.blocking)
        self.tick(102.,pump_feedback_fresh=True,pump_on=False,pump_feedback_stamp=102.,pump_feedback_command_id='off-wire')
        self.assertFalse(self.test.blocking)
        self.assertEqual(self.test.phase,'failed')
        self.assertEqual(self.test.reason,'manual_pump_off_unconfirmed')

    def test_pending_readback_grace_never_authorizes_second_on(self):
        self.start();self.tick(pump_feedback_fresh=False,pump_readback_pending=True)
        self.assertEqual(self.test.phase,'waiting_on')
        with self.assertRaises(ValueError):self.request(command_id='click-2')
        self.assertEqual(self.test.request_id,'click-1')
        self.assertEqual(self.tick(100.31,pump_feedback_fresh=False,pump_readback_pending=False),'off')

    def test_feedback_from_other_command_cannot_prove_high(self):
        self.start();self.tick(pump_on=True,pump_feedback_stamp=100.1,pump_feedback_command_id='foreign-on')
        self.assertFalse(self.test.saw_pump_on)

    def test_stop_works_with_invalid_stamps_and_no_confirmation(self):
        self.start()
        event=self.test.request(dict(action='stop',command_id='stop-1'),wall_now=100.2,now=10.2,
            feedback={},context={})
        self.assertEqual(event,'off');self.assertEqual(self.test.request_id,'click-1')
        self.test.submitted('stop-wire',100.2,enabled=False)
        self.tick(100.3,pump_feedback_stamp=100.3,pump_feedback_command_id='stop-wire')
        self.assertEqual(self.test.phase,'cancelled');self.assertFalse(self.test.blocking)

    def test_disabled_stop_does_not_wait_for_simulated_off(self):
        self.test=ManualPumpTest(False)
        self.test.request(dict(action='stop'),wall_now=100.,now=10.,feedback={},context={})
        self.assertFalse(self.test.blocking);self.assertFalse(self.test.confirmed_pump_off)

    def test_disabled_manual_feature_preserves_short_automatic_pulse_configuration(self):
        test=ManualPumpTest(False,50)
        self.assertFalse(test.readiness(10.,self.feedback,self.context)[0])
        with self.assertRaises(ValueError):ManualPumpTest(True,50)

    def test_stop_watermark_rejects_delayed_start_after_off_readback(self):
        self.test.request(dict(action='stop'),wall_now=100.,now=10.,feedback={},context={})
        self.test.submitted('stop-wire',100.,enabled=False)
        self.tick(100.1,pump_feedback_stamp=100.1,pump_feedback_command_id='stop-wire')
        with self.assertRaisesRegex(ValueError,'stale_request'):
            self.test.request(dict(action='start',command_id='delayed',issued_at=99.9,ttl_s=2.,
                duration_ms=400,operator_confirmed=True),wall_now=101.2,now=11.2,
                feedback=self.feedback,context=self.context)

    def test_movement_fault_or_new_owner_requires_off(self):
        for field,value in [('vehicle_stopped',False),('mission_fresh',False),('extension_idle',False),
                ('idle_mode',False),('other_owner',True),('estop',True),('fault','error')]:
            with self.subTest(field=field):
                self.setUp();self.start();self.context[field]=value
                self.assertEqual(self.tick(),'off');self.assertTrue(self.test.blocking)

    def test_duration_bool_stale_duplicate_and_missing_confirmation_rejected(self):
        for kw in [dict(duration_ms=True),dict(duration_ms=99),dict(duration_ms=1001),
                dict(issued_at=97.),dict(operator_confirmed=False),dict(ttl_s=True)]:
            with self.subTest(kw=kw):
                self.setUp()
                with self.assertRaises(ValueError):self.request(**kw)
                self.assertFalse(self.test.blocking)
        self.setUp();self.start()
        with self.assertRaises(ValueError):self.request()

    def test_interleaved_rejected_request_retains_original_run_completion(self):
        self.start()
        with self.assertRaises(ValueError):self.request(command_id='click-2')
        self.tick(pump_on=True,pump_feedback_stamp=100.1,pump_feedback_command_id='on-wire')
        self.tick(100.41);self.test.submitted('off-wire',100.41,enabled=False)
        self.tick(100.5,pump_on=False,pump_feedback_stamp=100.5,pump_feedback_command_id='off-wire')
        status=self.test.status(10.5,self.feedback,self.context)
        self.assertEqual(status['request_id'],'click-1');self.assertEqual(status['phase'],'completed')
        self.assertEqual(status['last_request_id'],'click-2');self.assertEqual(status['last_result'],'rejected')


if __name__=='__main__':unittest.main()
