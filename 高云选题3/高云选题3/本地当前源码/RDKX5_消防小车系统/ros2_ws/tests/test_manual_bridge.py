"""Exercise actual bridge callbacks with captured UART bytes; no ROS/device access."""
import importlib.util
import json
import pathlib
import sys
import types
import unittest
from unittest.mock import patch, Mock

ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rescue_vision'))
from rescue_vision import protocol as wire
from rescue_vision.manual_motion import parameters


class ManualBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        stubs={name:types.ModuleType(name) for name in ('rclpy','rclpy.node','std_msgs','std_msgs.msg',
            'rescue_interfaces','rescue_interfaces.msg','rescue_vision.node_common')}
        stubs['rclpy.node'].Node=object
        stubs['std_msgs.msg'].String=object
        for name in ('FpgaEvent','GimbalCommand','MotionPermission','ManeuverCommand'):
            setattr(stubs['rescue_interfaces.msg'],name,object)
        for name in ('param','age','publish_json','spin'):
            setattr(stubs['rescue_vision.node_common'],name,lambda *args,**kwargs:None)
        spec=importlib.util.spec_from_file_location('rescue_vision._manual_bridge_test',
            ROOT/'src/rescue_vision/rescue_vision/fpga_bridge_node.py')
        cls.module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,stubs):spec.loader.exec_module(cls.module)

    def setUp(self):
        self.now=10.;self.sent=[]
        self.node=self.module.FpgaBridge.__new__(self.module.FpgaBridge)
        n=self.node
        n.manual_motion_enabled=True;n.dry=False;n.manual=None;n.manual_seen=set()
        n.navigation_identity=None;n.navigation_latest_status=None;n.link_error=None
        n.manual_safety=dict(mission=dict(stamp=10.,vehicle_stopped=True,motion_allowed=False),
            actuator=dict(stamp=10.,pump_on=False,guard=dict(armed=False)),
            telemetry=dict(stamp=10.,vehicle_stopped=True,estop=False),navigation={})
        n.gate=wire.CommandGate();n.gate.observe_status(10.)
        n.get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=int(self.now*1e9)))
        n.get_logger=lambda:Mock()
        def send(kind,payload,metadata=None):
            self.sent.append((kind,payload,metadata));return len(self.sent)
        n.send=send
        p=patch.object(self.module.time,'monotonic',side_effect=lambda:self.now)
        p.start();self.addCleanup(p.stop)
    def command(self,**changes):
        data=dict(parameters('reverse'),action='manual_move',session_id=42,goal_id=1,deadline=14.,stamp=self.now)
        data.update(changes)
        self.node.navigation_command(types.SimpleNamespace(data=json.dumps(data)))
    def status(self,**changes):
        s=dict(stamp=self.now,state='accepted',obstacle_clear=True,estop=False)
        s.update(changes);self.node.manual['status']=s
    def refresh(self):
        for key in ('mission','actuator','telemetry'):self.node.manual_safety[key]['stamp']=self.now
    def test_disabled_or_dry_run_never_writes_manual_packet(self):
        self.node.manual_motion_enabled=False;self.command();self.assertEqual(self.sent,[])
        self.node.manual_motion_enabled=True;self.node.dry=True;self.command();self.assertEqual(self.sent,[])
    def test_retry_never_restarts_or_extends_deadline(self):
        self.command();self.assertEqual(self.sent[0][0],wire.MANUAL_MOVE)
        deadline=self.node.manual['deadline'];self.now=10.1;self.command(deadline=100.)
        self.assertEqual(self.node.manual['deadline'],deadline)
        self.status();self.refresh();self.assertEqual(self.node.manual_permission_guard(self.now),'')
        self.command(action='pause');before=len(self.sent);self.command()
        self.assertEqual(len(self.sent),before)
        self.command(action='resume');self.assertEqual(len(self.sent),before)
    def test_changed_parameters_or_previous_unfinished_goal_rejected(self):
        self.command();before=len(self.sent)
        self.command(**parameters('forward'));self.assertEqual(len(self.sent),before)
        self.node.manual=None;self.node.manual_seen=set();self.node.navigation_identity=(42,8,3)
        self.command();self.assertEqual(len(self.sent),before)
    def test_legacy_maneuver_cannot_compete_with_active_or_paused_manual_goal(self):
        self.command();self.status()
        self.node.maneuver_enabled=True
        before=len(self.sent)
        for cancelled in (False,True):
            self.node.manual['cancelled']=cancelled
            # Rejection must precede any message processing or UART write, even
            # if all ordinary permission and telemetry gates would allow GO.
            with patch.object(self.module,'age',side_effect=AssertionError('legacy entry reached')):
                self.node.maneuver(object())
            self.assertEqual(len(self.sent),before)
        # Only terminal+stopped feedback releases this particular ownership gate.
        self.node.manual['terminal_stopped']=True
        self.node.estop=False
        self.node.gate.observe_status(self.now)
        self.node.gate.set_permission(True,1,300,self.now,0.)
        self.node.telemetry_time=self.now
        self.node.segments={};self.node.segment_records={};self.node.segment_counter=1
        msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=0.),lease_ms=200,
            command_id='legacy',route_revision=1,edge_id='edge',turn=0,
            stop_at_end=True,speed_limit_m_s=.1,distance_m=.2)
        with patch.object(self.module,'age',return_value=0.):self.node.maneuver(msg)
        self.assertEqual(self.sent[-1][0],wire.MANEUVER)
    def test_deadline_stops_even_if_ros_permission_remains_go(self):
        self.command();self.now=14.;self.refresh();self.status()
        self.assertEqual(self.node.manual_permission_guard(self.now),'manual_deadline_expired')
        self.assertTrue(self.node.manual['cancelled']);self.assertEqual(self.sent[-1][0],wire.NAV_CONTROL)
        self.now=14.1;self.refresh();self.status()
        self.assertTrue(self.node.manual_permission_guard(self.now))
    def test_clearance_expiry_pump_and_traffic_each_latch_stop(self):
        for fault in ('obstacle','stale','pump','traffic'):
            self.setUp()
            self.command();self.status()
            if fault=='obstacle':self.status(obstacle_clear=False)
            elif fault=='stale':self.status(stamp=9.6)
            elif fault=='pump':self.node.manual_safety['actuator']['pump_on']=True
            else:self.node.manual_safety['mission']['traffic_active']=True
            self.assertTrue(self.node.manual_permission_guard(10.),fault)
            self.assertTrue(self.node.manual['cancelled'],fault)


if __name__=='__main__':unittest.main(verbosity=2)
