"""Continuous velocity ownership, measured feedback and loss-of-lease tests.

Uses fake transports only. No motors, ports, ROS nodes or network services.
"""
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src/rescue_vision'))
from rescue_vision.pd42_test import PD42Bench, validate_request
from rescue_vision.pd42_protocol import Command as PD
from rescue_vision.zdt_xs_protocol import Command as ZDT
from test_pd42_test import Clock, FakeTransport, FakeZDTTransport, evidence


class ContinuousTests(unittest.TestCase):
    def make_core(self, motor='motor1', mode='emm', scaled=False):
        clock=Clock(); pd=FakeTransport(); zdt=FakeZDTTransport()
        zdt.mode=mode;zdt.scaled=scaled
        core=PD42Bench(lambda *a,**kw: pd,clock,zdt_factory=lambda *a,**kw:zdt,motor_id=motor)
        self.sequence=0
        return core,clock,pd if motor=='motor1' else zdt

    def request(self, core, clock, action, **extra):
        self.sequence+=1
        request=dict(action=action,motor_id=core.motor_id,request_id=f'{core.motor_id}-{self.sequence}',
                     issued_at=clock(),ttl_s=3.)
        request.update(extra)
        core.handle(request,clock(),evidence(clock()))
        return request

    def ready(self, motor='motor1', mode='emm', scaled=False):
        core,clock,uart=self.make_core(motor,mode,scaled)
        self.request(core,clock,'connect');self.request(core,clock,'enable')
        self.assertTrue(core.snapshot(clock())['can_run'])
        return core,clock,uart

    def tick(self,core,clock): core.tick(clock(),evidence(clock()))

    def test_start_uses_one_true_velocity_request_and_ack_is_not_running(self):
        for motor,velocity_cmd in [('motor1',PD.VELOCITY),('motor2',ZDT.VELOCITY)]:
            with self.subTest(motor=motor):
                core,clock,uart=self.ready(motor)
                request=self.request(core,clock,'run_positive')
                s=core.snapshot(clock())
                self.assertTrue(s['busy']);self.assertTrue(s['continuous_pending'])
                self.assertFalse(s['continuous_running']);self.assertFalse(s['can_run'])
                self.assertEqual(s['run_id'],request['request_id']);self.assertEqual(s['continuous_direction'],1)
                self.assertEqual(uart.commands().count(velocity_cmd),1)
                self.tick(core,clock)
                self.assertTrue(core.snapshot(clock())['continuous_running'])
                self.assertFalse(core.snapshot(clock())['continuous_pending'])
                self.assertEqual(core.reason,'motor_test_continuous_running')

    def test_explicit_enable_still_required_and_bound_core_cannot_switch_motor(self):
        core,clock,uart=self.make_core('motor2')
        self.request(core,clock,'connect')
        self.request(core,clock,'run_positive')
        self.assertEqual(core.reason,'motor_test_explicit_enable_required')
        self.assertNotIn(ZDT.VELOCITY,uart.commands())
        self.request(core,clock,'connect',motor_id='motor1')
        self.assertEqual(core.reason,'motor_test_wrong_bound_motor')
        self.assertEqual(core.motor_id,'motor2')

    def test_zdt_velocity_firmware_layout_and_scale(self):
        for mode,scaled,expected in [('emm',False,'01 f6 01 00 05 0a 00 6b'),
                                     ('emm',True,'01 f6 01 00 32 0a 00 6b'),
                                     ('x',False,'01 f6 01 00 0a 00 32 00 6b')]:
            with self.subTest(mode=mode,scaled=scaled):
                core,clock,uart=self.ready('motor2',mode,scaled)
                self.request(core,clock,'run_negative')
                self.assertEqual(uart.writes[-1].hex(' '),expected)
                self.tick(core,clock)
                self.assertTrue(core.snapshot(clock())['continuous_running'])
                self.assertEqual(core.snapshot(clock())['speed_rpm'],-5)

    def test_keepalive_updates_deadline_without_uart_write_or_operator_receipt_change(self):
        core,clock,uart=self.ready()
        req=self.request(core,clock,'run_positive');self.tick(core,clock)
        clock.advance(2);self.tick(core,clock)
        count=len(uart.writes); prior=(core.last_request_id,core.last_action,core.last_result,core.reason)
        self.request(core,clock,'keepalive',run_id=req['request_id'])
        self.assertEqual(len(uart.writes),count)
        self.assertEqual((core.last_request_id,core.last_action,core.last_result,core.reason),prior)
        self.assertEqual(core.last_keepalive_result,'completed')
        self.assertEqual(core.snapshot(clock())['keepalive_remaining_s'],5.)
        self.assertEqual(core.run_id,req['request_id'])

    def test_wrong_session_keepalive_cannot_extend_or_replace_current_run(self):
        core,clock,uart=self.ready(); req=self.request(core,clock,'run_positive');self.tick(core,clock)
        deadline=core.run_deadline; count=len(uart.writes);clock.advance(.2)
        self.request(core,clock,'keepalive',run_id='previous-run')
        self.assertEqual(core.last_keepalive_reason,'motor_test_run_id_mismatch')
        self.assertEqual(core.run_deadline,deadline);self.assertEqual(core.run_id,req['request_id'])
        self.assertEqual(len(uart.writes),count)

    def test_watchdog_expiry_disables_without_resending_velocity(self):
        for motor,velocity_cmd in [('motor1',PD.VELOCITY),('motor2',ZDT.VELOCITY)]:
            core,clock,uart=self.ready(motor); self.request(core,clock,'run_positive');self.tick(core,clock)
            clock.advance(5.01);self.tick(core,clock)
            self.assertFalse(uart.enabled);self.assertIsNone(core.run_id)
            self.assertEqual(core.reason,'motor_test_continuous_lease_expired')
            self.assertFalse(core.snapshot(clock())['continuous_running'])
            self.assertEqual(uart.commands().count(velocity_cmd),1)

    def test_late_heartbeat_stops_and_cannot_resurrect_expired_session(self):
        core,clock,uart=self.ready();req=self.request(core,clock,'run_positive');self.tick(core,clock)
        clock.advance(5.01)
        self.request(core,clock,'keepalive',run_id=req['request_id'])
        self.assertFalse(uart.enabled);self.assertIsNone(core.run_id)
        self.assertEqual(core.reason,'motor_test_continuous_lease_expired')
        count=len(uart.writes)
        self.request(core,clock,'keepalive',run_id=req['request_id'])
        self.assertEqual(core.last_keepalive_reason,'motor_test_run_id_mismatch')
        self.assertEqual(len(uart.writes),count)

    def test_stale_feedback_heartbeat_cannot_keep_motor_running(self):
        core,clock,uart=self.ready();req=self.request(core,clock,'run_positive');self.tick(core,clock)
        clock.advance(.8)
        self.request(core,clock,'keepalive',run_id=req['request_id'])
        self.assertFalse(uart.enabled);self.assertTrue(core.fault)
        self.assertIn('continuous_feedback_lost',core.error)
        self.assertIsNone(core.run_id)

    def test_no_measured_start_speed_times_out_even_with_valid_ack(self):
        core,clock,uart=self.ready();self.request(core,clock,'run_positive');uart.speed=0
        clock.advance(3.1);self.tick(core,clock)
        self.assertFalse(uart.enabled);self.assertTrue(core.fault)
        self.assertIn('continuous_speed_not_confirmed',core.error)

    def test_wrong_direction_zero_after_start_and_overspeed_stop(self):
        for invalid_speed,reason in [(-5,'direction_mismatch'),(0,'speed_not_confirmed'),(50,'overspeed')]:
            core,clock,uart=self.ready();self.request(core,clock,'run_positive');self.tick(core,clock)
            uart.speed=invalid_speed;self.tick(core,clock)
            self.assertFalse(uart.enabled);self.assertTrue(core.fault)
            self.assertIn(reason,core.error);self.assertEqual(uart.commands().count(PD.VELOCITY),1)

    def test_stop_disconnect_and_interlock_loss_clear_run(self):
        for action in ('stop','disconnect','interlock'):
            core,clock,uart=self.ready();req=self.request(core,clock,'run_positive');self.tick(core,clock)
            if action=='interlock':
                unsafe=evidence(clock());unsafe['mission']['emergency_stop']=True
                core.tick(clock(),unsafe)
            else:self.request(core,clock,action)
            self.assertFalse(uart.enabled);self.assertIsNone(core.run_id)
            self.assertFalse(core.snapshot(clock())['continuous_running'])
            count=len(uart.writes);self.request(core,clock,'keepalive',run_id=req['request_id'])
            self.assertEqual(len(uart.writes),count)

    def test_stop_before_new_direction_or_step_is_required(self):
        core,clock,uart=self.ready();self.request(core,clock,'run_positive');self.tick(core,clock)
        count=len(uart.writes)
        for action in ('run_negative','step_positive','return_origin','enable'):
            self.request(core,clock,action)
            self.assertEqual(core.reason,'motor_test_continuous_active_stop_first')
        self.assertEqual(len(uart.writes),count)

    def test_continuous_run_has_no_angle_limit_but_return_requires_new_reference(self):
        core,clock,uart=self.ready();self.request(core,clock,'run_positive');self.tick(core,clock)
        uart.position+=51200;self.tick(core,clock)
        self.assertTrue(core.snapshot(clock())['continuous_running'])
        self.request(core,clock,'stop');self.request(core,clock,'enable')
        self.request(core,clock,'return_origin')
        self.assertEqual(core.reason,'motor_test_reference_range_reconnect_required')
        self.assertNotIn(PD.ABSOLUTE_POSITION,uart.commands())
        self.request(core,clock,'run_negative');self.assertEqual(core.last_result,'accepted')

    def test_lost_velocity_ack_never_retries_and_disables(self):
        for motor,cmd in [('motor1',PD.VELOCITY),('motor2',ZDT.VELOCITY)]:
            core,clock,uart=self.ready(motor);uart.fail=cmd
            self.request(core,clock,'run_positive')
            self.assertTrue(core.fault);self.assertFalse(uart.enabled);self.assertIsNone(core.run_id)
            self.assertEqual(uart.commands().count(cmd),1)
            count=len(uart.writes);self.tick(core,clock);self.assertEqual(len(uart.writes),count)

    def test_independent_core_watchdogs_do_not_stop_other_motor(self):
        a,ca,ua=self.ready('motor1');ra=self.request(a,ca,'run_positive');self.tick(a,ca)
        b,cb,ub=self.ready('motor2');rb=self.request(b,cb,'run_negative');self.tick(b,cb)
        ca.advance(4);cb.advance(4);self.tick(a,ca);self.tick(b,cb)
        self.request(b,cb,'keepalive',run_id=rb['request_id'])
        ca.advance(1.1);cb.advance(1.1);self.tick(a,ca);self.tick(b,cb)
        self.assertFalse(ua.enabled);self.assertTrue(ub.enabled)
        self.assertTrue(b.snapshot(cb())['continuous_running'])
        self.assertEqual(b.run_id,rb['request_id']);self.assertIsNone(a.run_id)

    def test_regular_heartbeats_allow_over_thirty_seconds_without_repeated_motion(self):
        core,clock,uart=self.ready();req=self.request(core,clock,'run_positive');self.tick(core,clock)
        for _ in range(12):
            clock.advance(3);self.tick(core,clock)
            self.request(core,clock,'keepalive',run_id=req['request_id'])
        self.assertTrue(core.snapshot(clock())['continuous_running'])
        self.assertTrue(uart.enabled);self.assertEqual(uart.commands().count(PD.VELOCITY),1)

    def test_heartbeats_never_extend_initial_measurement_deadline(self):
        core,clock,uart=self.ready();req=self.request(core,clock,'run_positive');uart.speed=0
        for _ in range(2):
            clock.advance(1);self.tick(core,clock)
            self.request(core,clock,'keepalive',run_id=req['request_id'])
        clock.advance(1.1);self.tick(core,clock)
        self.assertFalse(uart.enabled);self.assertTrue(core.fault)
        self.assertIn('speed_not_confirmed',core.error)

    def test_schema_only_keepalive_accepts_and_requires_run_id(self):
        base=dict(action='keepalive',request_id='hb',issued_at=100.,ttl_s=3.)
        for payload in [base,dict(base,run_id=''),dict(base,run_id=True),
                        dict(base,run_id='run',extra=1),dict(base,action='run_positive',run_id='run')]:
            with self.subTest(payload=payload),self.assertRaises(ValueError):validate_request(payload,100.)
        self.assertEqual(validate_request(dict(base,run_id='run'),100.)['run_id'],'run')


if __name__=='__main__':unittest.main(verbosity=2)
