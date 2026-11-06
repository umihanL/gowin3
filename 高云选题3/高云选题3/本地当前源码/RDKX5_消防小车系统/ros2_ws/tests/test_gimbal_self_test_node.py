"""Run actual LocalActuator callbacks with a mixed motor fixture and ROS type stubs."""
import importlib.util
import json
import pathlib
import sys
import types
import unittest
from copy import deepcopy
from unittest import mock

ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rescue_vision'))
from test_mixed_actuator_io import mixed_fixture
from test_zdt_fd8_control import fixture as fd8_fixture


class FakeNode:
    now=10.
    def __init__(self,*args):pass
    def create_publisher(self,*args):return []
    def create_subscription(self,*args):pass
    def create_timer(self,*args):pass
    def get_logger(self):return types.SimpleNamespace(info=lambda *a:None,warning=lambda *a:None)
    def get_clock(self):return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=int(self.now*1e9)))


def module():
    ns=types.SimpleNamespace
    common=ns(age=lambda node,stamp:0.,param=lambda node,name,default:node.parameters.get(name,default),
              read_json=deepcopy,publish_json=lambda pub,data:pub.append(data),spin=lambda *a:None)
    mods={'rclpy':ns(),'rclpy.node':ns(Node=FakeNode),'std_msgs':ns(),
          'std_msgs.msg':ns(Bool=object,String=object),'rescue_interfaces':ns(),
          'rescue_interfaces.msg':ns(GimbalCommand=object),'rescue_vision.node_common':common}
    spec=importlib.util.spec_from_file_location('rescue_vision._self_test_node_contract',
        ROOT/'src/rescue_vision/rescue_vision/local_actuator_node.py')
    m=importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules,mods):spec.loader.exec_module(m)
    return m


class SelfTestNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.mod=module()

    def setUp(self):
        self.io,self.pan,self.tilt,self.events=mixed_fixture()
        cfg=deepcopy(self.io.config);cfg['dry_run']=False
        self.mod.LocalActuator.parameters=dict(control_config=dict(enabled=True,pump_enabled=False),
            io_config=cfg,dry_run=False,pd42_reference_confirmed=True)
        with mock.patch.object(self.mod,'create_actuator_io',return_value=self.io):
            self.node=self.mod.LocalActuator()
        self.node.now=10.
        self.node.last_tick=9.9
        self.node.mission=dict(mode='inspect',vehicle_stopped=True,traffic_active=False,
                               emergency_stop=False,operator_hold=True)
        self.node.mission_at=10.

    def run_tick(self,now):
        self.node.now=now;self.node.mission_at=now
        # The mixed template now routes the pump through FPGA B13. A measured
        # OFF fixture is required; missing feedback must not count as pump-off.
        if self.node.fpga_pump is not None:
            self.node.pump_state(types.SimpleNamespace(data=json.dumps(dict(
                stamp=now,feedback_stamp=now,feedback_fresh=True,real_uart=True,
                protocol_ready=True,pin='B13',active_high=True,pump_on=False,pump_ready=True))))
        with mock.patch.object(self.mod.time,'monotonic',return_value=now):
            self.io._cycle();self.node.tick();self.io._cycle()

    def request(self,action='start',rid='test'):
        self.node.self_test_request(types.SimpleNamespace(data=json.dumps(dict(action=action,
            request_id=rid,issued_at=self.node.now,ttl_s=2.,axes='both',amplitude_deg=1.,speed_deg_s=6.))))

    def test_real_callback_claims_owner_and_ignores_fire_command_until_stopped(self):
        self.run_tick(10.)
        with mock.patch.object(self.mod.time,'monotonic',return_value=10.):self.request()
        self.run_tick(10.1)
        self.assertEqual(self.node.command_owner,'self_test')
        self.assertEqual(self.node.requested,(91.,91.))
        with mock.patch.object(self.mod.time,'monotonic',return_value=10.1):
            self.node.gimbal(types.SimpleNamespace())  # Must return before reading any foreign command.
        self.assertEqual(self.node.requested,(91.,91.))
        with mock.patch.object(self.mod.time,'monotonic',return_value=10.1):self.request('stop','stop-test')
        self.run_tick(10.2)
        self.assertFalse(self.pan.enabled or self.tilt.enabled)
        status=self.node.self_test_output[-1]
        self.assertEqual(status['last_request_id'],'stop-test')
        self.assertEqual(status['phase'],'cancelled')
        self.assertFalse(status['hardware_verified'])
        self.assertFalse(self.node.pump_on)

    def test_mission_emergency_cancels_self_test_even_without_separate_estop_topic(self):
        self.run_tick(10.)
        with mock.patch.object(self.mod.time,'monotonic',return_value=10.):self.request()
        self.run_tick(10.1)
        self.node.mission['emergency_stop']=True
        self.run_tick(10.2)
        self.assertEqual(self.node.self_test.phase,'failed')
        self.assertFalse(self.pan.enabled or self.tilt.enabled)

    def test_no_measured_vehicle_stop_returns_correlated_rejection(self):
        self.run_tick(10.)
        self.node.mission['vehicle_stopped']=False
        with mock.patch.object(self.mod.time,'monotonic',return_value=10.):self.request()
        self.run_tick(10.11)
        out=self.node.self_test_output[-1]
        self.assertEqual(out['last_result'],'rejected')
        self.assertEqual(out['last_request_id'],'test')
        self.assertFalse(out['can_start'])
        self.assertEqual(out['ready_reason'],'measured_vehicle_stop_required')

    def test_manual_move_owns_outputs_and_disarm_cannot_resume_it(self):
        self.run_tick(10.)
        message=types.SimpleNamespace(data=json.dumps(dict(action='start',request_id='manual-1',
            issued_at=10.,ttl_s=2.,pan_deg=92.,tilt_deg=90.)))
        with mock.patch.object(self.mod.time,'monotonic',return_value=10.):
            self.node.manual_request(message)
        self.run_tick(10.1)
        self.assertEqual(self.node.command_owner,'manual')
        self.assertTrue(self.node.manual.active)
        self.node.gimbal(types.SimpleNamespace())
        self.assertEqual(self.node.requested,(92.,90.))
        self.node.stop_outputs('operator_disarm')
        self.run_tick(10.2)
        self.assertFalse(self.node.manual.active)
        self.assertFalse(self.pan.enabled or self.tilt.enabled)
        self.assertEqual(self.node.manual_output[-1]['phase'],'cancelled')

    def test_manual_stop_also_cancels_self_test_and_clears_every_owner_field(self):
        self.run_tick(10.)
        with mock.patch.object(self.mod.time,'monotonic',return_value=10.):self.request()
        self.run_tick(10.1)
        self.assertTrue(self.node.self_test.active)
        with mock.patch.object(self.mod.time,'monotonic',return_value=10.1):
            self.node.manual_request(types.SimpleNamespace(data='{"action":"stop"}'))
        self.assertFalse(self.node.self_test.active or self.node.manual.active)
        self.assertIsNone(self.node.requested)
        self.assertIsNone(self.node.command_owner)
        self.assertEqual(self.node.command_deadline,0)
        self.run_tick(10.2)
        self.run_tick(10.3)
        self.assertFalse(self.pan.enabled or self.tilt.enabled)
        self.assertFalse(self.node.self_test.active)

    def test_self_test_stop_also_cancels_manual(self):
        self.run_tick(10.)
        doc=dict(action='start',request_id='manual-2',issued_at=10.,ttl_s=2.,pan_deg=92.,tilt_deg=90.)
        with mock.patch.object(self.mod.time,'monotonic',return_value=10.):
            self.node.manual_request(types.SimpleNamespace(data=json.dumps(doc)))
        self.run_tick(10.1)
        with mock.patch.object(self.mod.time,'monotonic',return_value=10.1):self.request('stop')
        self.run_tick(10.2)
        self.assertFalse(self.node.manual.active or self.node.self_test.active)
        self.assertIsNone(self.node.command_owner)
        self.assertFalse(self.pan.enabled or self.tilt.enabled)

    def test_fixed_camera_rejects_even_a_late_old_scan_lease(self):
        self.node.mission.update(operator_hold=False, fixed_camera_observation=True)
        self.assertFalse(self.node.owner_allowed(10., 'mission_home'))
        self.node.mission['mode'] = 'fire'
        self.assertFalse(self.node.owner_allowed(10., 'mission_home'))
        self.assertTrue(self.node.owner_allowed(10., 'fire'))
        self.node.mission['fixed_camera_observation'] = False
        self.assertTrue(self.node.owner_allowed(10., 'mission_home'))


class FD8NodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.mod=module()

    def setUp(self):
        self.io,self.pan,self.tilt,self.events=fd8_fixture()
        self.io.servo['tilt']['motion_enabled']=False
        cfg=deepcopy(self.io.config);cfg['dry_run']=False
        self.mod.LocalActuator.parameters=dict(control_config=dict(enabled=True,pump_enabled=False,max_speed_deg_s=6.),
            io_config=cfg,dry_run=False,pd42_reference_confirmed=True)
        with mock.patch.object(self.mod,'create_actuator_io',return_value=self.io):
            self.node=self.mod.LocalActuator()
        self.node.now=10.;self.node.last_tick=9.9
        self.node.mission=dict(mode='inspect',vehicle_stopped=True,traffic_active=False,
                               emergency_stop=False,operator_hold=True)
        self.node.mission_at=10.

    run_tick=SelfTestNodeTests.run_tick

    def test_shutdown_pump_publish_failure_still_closes_uart_owner_and_releases_axes(self):
        from rescue_vision.fpga_pump_io import FPGAPumpIO
        self.io._active = True
        self.io._thread = types.SimpleNamespace(join=lambda timeout:self.io._run(),is_alive=lambda:False)
        for motor in (self.pan,self.tilt): motor.close = mock.Mock()
        owner = mock.Mock(); self.io._owner = owner
        self.node.io = FPGAPumpIO(self.io,self.node.fpga_pump,self.io.config['pump'])
        self.node.manual_pump_transport.close = mock.Mock(side_effect=RuntimeError('publisher context invalid'))
        logger = types.SimpleNamespace(error=mock.Mock())
        with mock.patch.object(self.node,'get_logger',return_value=logger), \
                mock.patch.object(FakeNode,'destroy_node',return_value='destroyed',create=True):
            self.assertEqual(self.node.destroy_node(),'destroyed')
        self.assertTrue(self.io.closed)
        self.assertFalse(self.pan.enabled or self.tilt.enabled or self.io._active)
        self.pan.close.assert_called_once();self.tilt.close.assert_called_once()
        owner.close.assert_called_once()
        self.assertIn('manual pump: publisher context invalid',logger.error.call_args.args[0])

    def manual_request(self,angle,rid):
        message=types.SimpleNamespace(data=json.dumps(dict(action='start',request_id=rid,
            issued_at=self.node.now,ttl_s=2.,pan_deg=angle)))
        with mock.patch.object(self.mod.time,'monotonic',return_value=self.node.now):
            self.node.manual_request(message)

    def test_outside_limit_rejects_without_latched_fault_and_valid_move_finishes(self):
        self.run_tick(10.);self.run_tick(10.05)
        self.manual_request(91,'bad-boundary')
        self.assertEqual(self.node.manual.last_result,'rejected')
        self.assertFalse(self.node.fault)
        self.manual_request(1.,'good-angle')
        self.assertTrue(self.node.manual.active)
        for i in range(1,20):
            if self.io._fd8.pending:self.pan.arrive()
            self.run_tick(round(10.05+i*.05,6))
            if self.node.manual.phase=='completed':break
        self.assertEqual(self.node.manual.phase,'completed')
        self.assertFalse(self.node.fault)
        self.assertIsNone(self.node.command_owner)
        self.assertFalse(self.node.pump_on)

    def test_node_publishes_settled_for_same_quantized_fire_goal_without_arming_pump(self):
        self.node.mission.update(mode='fire',operator_hold=False)
        self.run_tick(10.);self.run_tick(10.05)
        for i in range(1,30):
            now=round(10.05+i*.05,6)
            self.node.now=now;self.node.mission_at=now
            if self.io._fd8.pending:self.pan.arrive()
            with mock.patch.object(self.mod.time,'monotonic',return_value=now):
                self.node.gimbal(types.SimpleNamespace(owner='fire',header=types.SimpleNamespace(stamp=0),
                    lease_ms=300,pan_deg=1.,tilt_deg=0.,speed_deg_s=6.))
            self.run_tick(now)
        self.assertTrue(any(row['gimbal_acknowledged'] for row in self.node.output))
        self.assertTrue(any(row['gimbal_settled_measured'] for row in self.node.output))
        self.assertFalse(self.node.pump_on)
        self.assertFalse(self.node.guard.armed)

    def test_fire_status_receives_same_reachable_lattice_as_manual_control(self):
        self.run_tick(10.)
        manual = self.node.manual_output[-1]
        automatic = self.node.output[-1]
        self.assertAlmostEqual(automatic['axis_step_deg'][0], .1125)
        self.assertEqual(automatic['motion_axes'], ['pan'])
        for field in ('axis_reachable_limits_deg', 'axis_step_deg', 'motion_axes'):
            self.assertEqual(automatic[field], manual[field])
        self.assertFalse(self.node.pump_on)


if __name__=='__main__':unittest.main(verbosity=2)
