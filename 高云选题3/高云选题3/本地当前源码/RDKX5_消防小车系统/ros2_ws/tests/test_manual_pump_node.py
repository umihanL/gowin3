"""Real node callbacks and FPGA transport, ROS/I/O replaced with local fixtures."""
import json
from copy import deepcopy
import types
import unittest
from unittest import mock
from test_gimbal_self_test_node import module, mixed_fixture
from rescue_vision.fpga_pump_io import FPGAPumpIO


class ManualPumpNodeTests(unittest.TestCase):
    def setUp(self):
        self.mod=module();self.io,self.pan,self.tilt,self.events=mixed_fixture()
        self.cfg=deepcopy(self.io.config)
        self.writes=[]
        self.make_node()

    def make_node(self, manual_enabled=True, auto_real=False):
        cfg=deepcopy(self.cfg);cfg['dry_run']=not auto_real;cfg['pump']['enabled']=auto_real
        self.mod.LocalActuator.parameters=dict(control_config=dict(enabled=auto_real,pump_enabled=auto_real),
            io_config=cfg,dry_run=not auto_real,pd42_reference_confirmed=auto_real,
            manual_pump_test_enabled=manual_enabled)
        def io_factory(value, fpga_pump):
            fpga_pump.emit=self.writes.append
            return FPGAPumpIO(self.io,fpga_pump,value['pump'])
        with mock.patch.object(self.mod,'create_actuator_io',side_effect=io_factory):self.node=self.mod.LocalActuator()
        self.node.manual_pump_transport.emit=self.writes.append
        self.node.now=100.;self.node.last_tick=9.9
        self.context(100.)
        self.feed(100.,False,'startup-off')

    def context(self, wall):
        self.node.now=wall
        self.node.mission=dict(stamp=wall,mode='inspect',vehicle_stopped=True,traffic_active=False,
            emergency_stop=False,operator_hold=False,auto_fire_active=False)
        self.node.mission_at=wall-90
        self.node.extension=dict(stamp=wall,real_uart=True,status_fresh=True,busy=False,vehicle_stopped=True,
            fpga_host=dict(fresh=True,selected=True,scope=True,link_ok=True))
        self.node.extension_at=wall-90

    def feed(self, wall, high, cid, *, fresh=True):
        doc=dict(stamp=wall,feedback_stamp=wall,feedback_fresh=fresh,real_uart=True,
            protocol_ready=True,pin='B13',active_high=True,pump_on=high,pump_ready=not high,
            feedback_command_id=cid)
        self.node.pump_state(types.SimpleNamespace(data=json.dumps(doc)))

    def request(self, action='start', **kw):
        doc=dict(action=action,command_id='start-1' if action=='start' else 'stop-1',
            issued_at=self.node.now,ttl_s=2.,duration_ms=400,operator_confirmed=True);doc.update(kw)
        with mock.patch.object(self.mod.time,'monotonic',return_value=self.node.now-90):
            self.node.pump_manual_request(types.SimpleNamespace(data=json.dumps(doc)))

    def tick(self, wall):
        self.context(wall)
        with mock.patch.object(self.mod.time,'monotonic',return_value=wall-90):self.node.tick()

    def start(self):
        self.request()
        self.assertFalse(self.writes[-1]['enabled'])
        self.assertEqual(self.node.pump_manual.phase,'preparing_off')
        off=self.writes[-1]['command_id']
        self.feed(100.01,False,off);self.tick(100.01)
        self.assertTrue(self.writes[-1]['enabled'])

    def test_uncalibrated_gimbal_can_manual_pulse_without_promoting_automatic_transport(self):
        self.assertTrue(self.node.fpga_pump.dry_run);self.assertFalse(self.node.enabled)
        self.assertFalse(self.node.manual_pump_transport.dry_run)
        self.start();self.assertEqual(sum(x['enabled'] for x in self.writes),1)
        on=self.writes[-1]['command_id'];self.feed(100.1,True,on);self.tick(100.1)
        self.assertTrue(self.node.pump_manual.saw_pump_on)
        self.tick(100.41);off=self.writes[-1]['command_id'];self.assertFalse(self.writes[-1]['enabled'])
        self.feed(100.5,False,off);self.tick(100.5)
        self.assertEqual(self.node.pump_manual.phase,'completed')
        self.assertFalse(self.node.pump_manual.blocking)
        self.assertEqual(len(self.events),0)  # No enable, disable or movement of either axis.
        self.assertFalse(self.node.output[-1]['enabled']);self.assertTrue(self.node.output[-1]['dry_run'])

    def test_manual_reservation_ignores_gimbal_and_rejects_automatic_arm_without_extra_off(self):
        self.request();count=len(self.writes)
        self.node.gimbal(types.SimpleNamespace())
        self.node.request(types.SimpleNamespace(data='{"action":"arm_pump"}'))
        self.node.auto_request(types.SimpleNamespace(data='{"action":"start"}'))
        self.assertEqual(len(self.writes),count)
        self.assertTrue(self.node.pump_manual.blocking);self.assertFalse(self.node.guard.armed)
        self.assertEqual(len(self.events),0)

    def test_disarm_uses_one_real_off_owner_and_can_complete_with_automatic_transport_real(self):
        self.make_node(auto_real=True);self.start()
        on=self.writes[-1]['command_id'];self.feed(100.1,True,on);self.tick(100.1)
        count=len(self.writes)
        with mock.patch.object(self.mod.time,'monotonic',return_value=10.1):
            self.node.request(types.SimpleNamespace(data='{"action":"disarm"}'))
        emitted=self.writes[count:]
        self.assertEqual(len(emitted),1);self.assertFalse(emitted[0]['enabled'])
        self.assertEqual(self.node.pump_manual.off_id,emitted[0]['command_id'])
        self.feed(100.2,False,emitted[0]['command_id']);self.tick(100.2)
        self.assertFalse(self.node.pump_manual.blocking)

    def test_global_stop_has_only_one_real_off_generation_with_wrapped_real_backend(self):
        self.make_node(auto_real=True);self.request();count=len(self.writes)
        with mock.patch.object(self.mod.time,'monotonic',return_value=10.1):self.node.stop_outputs('emergency_stop',force_pump=True)
        self.assertEqual(len(self.writes[count:]),1)
        off=self.writes[-1]['command_id'];self.assertEqual(self.node.pump_manual.off_id,off)
        self.feed(100.2,False,off);self.tick(100.2)
        self.assertFalse(self.node.pump_manual.blocking)

    def test_disabled_manual_stop_does_not_lock_axes_waiting_for_simulated_readback(self):
        self.make_node(manual_enabled=False);self.request('stop')
        self.assertFalse(self.node.pump_manual.blocking)
        self.assertFalse(self.node.pump_manual.confirmed_pump_off)
        self.assertFalse(self.writes)

    def test_stop_revokes_automatic_arm_even_without_existing_manual_run(self):
        self.make_node(auto_real=True)
        self.node.guard.armed=True
        self.node.armed_target=7;self.node.armed_auto_run='previous-auto-run'
        self.request('stop')
        self.assertFalse(self.node.guard.armed)
        self.assertIsNone(self.node.armed_target);self.assertIsNone(self.node.armed_auto_run)
        self.assertTrue(self.node.pump_manual.blocking)
        self.assertEqual(len(self.writes),1);self.assertFalse(self.writes[0]['enabled'])
        self.feed(100.1,False,self.writes[-1]['command_id']);self.tick(100.1)
        self.assertFalse(self.node.pump_manual.blocking)
        self.assertFalse(self.node.guard.armed)

    def test_pending_manual_owner_blocks_navigation_and_motor_bench_despite_old_low(self):
        from rescue_vision.pd42_test import bench_guard
        import test_extension_node as ext
        ext.ExtensionAdapterTests.setUpClass()
        node=ext.ExtensionAdapterTests.module.ExtensionTask.__new__(ext.ExtensionAdapterTests.module.ExtensionTask)
        node.actuator=dict(stamp=100.,pump_on=False,guard=dict(armed=False),manual_pump_active=True)
        self.assertEqual(node.actuator_guard(100.),'manual_pump_must_finish_before_motion')
        evidence=dict(mission=dict(stamp=100.,mode='inspect'),fpga_link=dict(stamp=100.,dry_run=True),
            actuators=dict(node.actuator,dry_run=True,enabled=False))
        self.assertEqual(bench_guard(evidence,100.),'motor_test_production_actuator_active')

    def test_live_or_expired_unretired_gimbal_and_bench_busy_reject_start(self):
        for name in ('pwm','bench','lease'):
            with self.subTest(name=name):
                self.make_node()
                if name=='pwm':self.node.pwm_active=True
                elif name=='bench':self.node.motor_bench={'motors':{'motor2':{'busy':True}}}
                else:self.node.requested=(0,0);self.node.command_deadline=11.
                self.request();self.assertEqual(self.node.pump_manual.last_result,'rejected')
                self.assertFalse(self.node.pump_manual.blocking)


if __name__=='__main__':unittest.main()
