"""Revised FPGA-owned task flow with real ROS messages, no hardware receivers.

Run only with RESCUE_RUN_ROS_TEST=1 ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1.
Each callback is synchronous and all output publishers are replaced by captures.
"""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('RESCUE_RUN_ROS_TEST') == '1', 'Explicit isolated ROS test only')
class FpgaTaskMissionBoardTests(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '83')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        self.tmp = tempfile.TemporaryDirectory(prefix='rescue-fpga-task-test-')
        self.addCleanup(self.tmp.cleanup)
        for name in ('_HardwarePWM', '_PumpGPIO', '_ExclusiveOwner'):
            guard = patch('rescue_vision.local_actuator_io.'+name,
                          side_effect=AssertionError('Hardware access forbidden: '+name))
            guard.start(); self.addCleanup(guard.stop)
        self.now = 100.
        timer = patch('rescue_vision.mission_node.time.monotonic', side_effect=lambda: self.now)
        timer.start(); self.addCleanup(timer.stop)
        mount = json.loads((ROOT/'config/mount.template.json').read_text(encoding='utf-8-sig'))
        mount.update(mounting_verified=True, road_pose_verified=True, road_pan_deg=90., road_tilt_deg=120.,
                     pan_home_deg=90., tilt_home_deg=90., speed_deg_s=180., extra_settle_s=.1)
        traffic = json.loads((ROOT/'config/traffic.json').read_text(encoding='utf-8-sig'))
        traffic['controller'].update(stop_policy='delay', stop_delay_s=2.)
        mount_path = Path(self.tmp.name)/'mount.json'
        traffic_path = Path(self.tmp.name)/'traffic.json'
        mount_path.write_text(json.dumps(mount), encoding='utf-8')
        traffic_path.write_text(json.dumps(traffic), encoding='utf-8')
        import rclpy
        from rescue_vision.mission_node import Mission
        rclpy.init(args=['--ros-args', '-p', 'task_profiles_enabled:=true', '-p', 'gimbal_transport:=x5',
                        '-p', 'navigation_owner:=fpga', '-p', 'initial_mode:=standby',
                        '-p', 'mount_config:='+str(mount_path), '-p', 'traffic_config:='+str(traffic_path)])
        self.node = Mission()
        self.commands, self.permissions, self.states, self.traffic = [], [], [], []
        self.maneuvers, self.auto, self.speech = [], [], []
        class Capture:
            def __init__(self, output, is_json=False): self.output, self.is_json = output, is_json
            def publish(self, msg): self.output.append(json.loads(msg.data) if self.is_json else msg)
        for name, output, is_json in (
                ('gimbal_pub', self.commands, False), ('permission_pub', self.permissions, False),
                ('state_pub', self.states, True), ('traffic_pub', self.traffic, True),
                ('maneuver_pub', self.maneuvers, False), ('auto_pub', self.auto, True),
                ('speech_pub', self.speech, False)):
            setattr(self.node, name, Capture(output, is_json))
        self.seq = 0

    def tearDown(self):
        import rclpy
        self.node.destroy_node()
        rclpy.shutdown()

    def message(self, value):
        from std_msgs.msg import String
        return String(data=json.dumps(value))

    def ros_now(self):
        return self.node.get_clock().now().nanoseconds/1e9

    def ack(self):
        self.node.link_state(self.message(dict(stamp=self.ros_now(), status_fresh=True,
            gimbal_settled_estimate=True, gimbal_acknowledged=True, target_deg=self.node.target)))

    def fpga_status(self, stopped):
        from rescue_interfaces.msg import FpgaEvent
        value = FpgaEvent()
        value.header.stamp = self.node.get_clock().now().to_msg()
        value.event_type = FpgaEvent.STATUS
        value.vehicle_stopped = stopped
        self.node.event(value)

    def proposal(self, **changes):
        value = dict(stamp=self.ros_now(), owner='fpga', command_id='go-C', session_id=51, goal_id=1,
            building_id='C', building_number='03', valid=True, ready_to_move=True, arrived=False,
            status_fresh=True, vehicle_stopped=False, estop=False, arrival_policy='manual')
        value.update(changes)
        self.node.navigation_proposal(self.message(value))

    def traffic_ready(self, navigate=False):
        if navigate:
            self.node.request(self.message({'mode':'navigate'}));self.proposal()
        self.node.tick()
        self.assertEqual(self.commands, [], 'No removed road-pose command')

    def explicit_traffic(self):
        self.traffic_ready()
        self.node.request(self.message(dict(mode='traffic',request_id='manual-light-1',
            issued_at=self.ros_now(),ttl_s=2.)))
        self.assertIsNotNone(self.node.traffic.event_id)
        self.assertFalse(self.permissions[-1].allow_motion)

    def test_standby_waits_without_moving_camera_or_chassis(self):
        self.traffic_ready()
        self.assertEqual(self.node.mode, 'standby')
        self.assertEqual(self.states[-1]['camera_pose'], 'unconfigured')
        self.assertFalse(self.permissions[-1].allow_motion)
        self.assertFalse(self.states[-1]['yolo_requested'])
        self.assertEqual(self.maneuvers, [])

    def test_inspect_startup_enables_vision_without_inventing_hardware_feedback(self):
        from unittest.mock import Mock
        from rescue_vision import mission_node
        original = mission_node.param
        with patch.object(mission_node, 'param', side_effect=lambda node, name, default:
                          'inspect' if name == 'initial_mode' else original(node, name, default)):
            node = mission_node.Mission()
        try:
            node.state_pub = Mock()
            node.permission_pub = Mock()
            node.gimbal_pub = Mock()
            node.maneuver_pub = Mock()
            node.traffic_pub = Mock()
            node.tick()
            state = json.loads(node.state_pub.publish.call_args.args[0].data)
            self.assertTrue(state['yolo_requested'])
            self.assertFalse(state['vehicle_stopped'])
            self.assertFalse(state['gimbal_ready'])
            self.assertFalse(state['motion_allowed'])
            node.gimbal_pub.publish.assert_not_called()
            node.maneuver_pub.publish.assert_not_called()
        finally:
            node.destroy_node()

    def test_explicit_traffic_scans_after_two_seconds_without_inventing_stop_feedback(self):
        self.explicit_traffic()
        start = self.node.traffic._start
        count = len(self.commands)
        self.now = start+1.999
        self.ack(); self.node.tick()
        self.assertEqual(self.node.traffic.state, 'WAIT_STOP')
        self.assertEqual(len(self.commands), count)
        self.now = start+2.
        self.ack(); self.node.tick()
        self.assertEqual(self.commands[-1].owner, 'traffic')
        self.assertEqual(self.node.traffic.state, 'SCANNING')
        self.assertTrue(self.traffic[-1]['stop_wait_complete'])
        self.assertEqual(self.traffic[-1]['stop_confirmation_basis'], 'elapsed_time_only')
        self.assertFalse(self.states[-1]['vehicle_stopped'], 'Elapsed time must not create pump stop evidence')
        self.assertFalse(self.permissions[-1].allow_motion)

    def test_actual_stop_feedback_does_not_skip_requested_two_second_delay(self):
        self.traffic_ready()
        self.fpga_status(True)
        self.node.begin_crosswalk(123, self.now)
        count = len(self.commands)
        self.now += .1
        self.fpga_status(True); self.ack(); self.node.tick()
        self.assertEqual(self.node.traffic.state, 'WAIT_STOP')
        self.assertEqual(len(self.commands), count)

    def test_fpga_navigation_has_permission_without_x5_maneuvers(self):
        self.traffic_ready(navigate=True)
        self.assertTrue(self.permissions[-1].allow_motion)
        self.assertEqual(self.maneuvers, [])
        self.assertFalse(self.states[-1]['vehicle_stopped'])

    def test_wrong_owner_or_stale_navigation_cannot_drive_or_prove_stop(self):
        self.traffic_ready(navigate=True)
        self.proposal(owner='x5', vehicle_stopped=True)
        self.node.tick()
        self.assertFalse(self.permissions[-1].allow_motion)
        self.assertFalse(self.node.stopped(self.now))
        self.proposal(stamp=self.ros_now()-2., vehicle_stopped=True)
        self.node.tick()
        self.assertFalse(self.permissions[-1].allow_motion)
        self.assertFalse(self.node.stopped(self.now))

    def test_auto_arrival_waits_actual_stop_and_only_starts_once(self):
        self.node.request(self.message({'mode':'navigate', 'arrival_policy':'auto'}))
        self.proposal(arrived=True, ready_to_move=False, arrival_policy='auto')
        self.node.tick()
        self.assertEqual(self.node.mode, 'navigate')
        self.assertEqual(self.auto, [])
        self.proposal(arrived=True, ready_to_move=False, arrival_policy='auto', vehicle_stopped=True)
        self.node.tick()
        self.assertEqual(self.node.mode, 'inspect')
        self.assertEqual(len(self.auto), 1)
        self.assertEqual(self.auto[0]['action'], 'start')
        for _ in range(3): self.node.tick()
        self.assertEqual(len(self.auto), 1)
        self.assertFalse(self.permissions[-1].allow_motion)

    def test_manual_arrival_inspects_but_does_not_arm_auto(self):
        self.node.request(self.message({'mode':'navigate', 'arrival_policy':'manual'}))
        self.proposal(arrived=True, ready_to_move=False, vehicle_stopped=True, arrival_policy='manual')
        self.node.tick()
        self.assertEqual(self.node.mode, 'inspect')
        self.assertEqual(self.auto, [])
        self.assertTrue(self.states[-1]['yolo_requested'])
        self.assertFalse(self.permissions[-1].allow_motion)

    def test_enroute_switch_from_auto_to_manual_overrides_goal_policy(self):
        self.node.request(self.message({'mode':'navigate', 'arrival_policy':'auto'}))
        self.proposal(arrival_policy='auto')
        self.node.tick()
        self.node.request(self.message({'arrival_policy':'manual'}))
        self.proposal(arrived=True, ready_to_move=False, vehicle_stopped=True, arrival_policy='auto')
        self.node.tick()
        self.assertEqual(self.node.mode, 'inspect')
        self.assertEqual(self.node.arrival_policy, 'manual')
        self.assertEqual(self.auto, [])
        self.node.tick()
        self.assertEqual(self.auto, [])

    def test_enroute_switch_from_manual_to_auto_overrides_goal_policy(self):
        self.node.request(self.message({'mode':'navigate', 'arrival_policy':'manual'}))
        self.proposal(arrival_policy='manual')
        self.node.tick()
        self.node.request(self.message({'arrival_policy':'auto'}))
        self.proposal(arrived=True, ready_to_move=False, vehicle_stopped=True, arrival_policy='manual')
        self.node.tick()
        self.assertEqual(self.node.mode, 'inspect')
        self.assertEqual(self.node.arrival_policy, 'auto')
        self.assertEqual(len(self.auto), 1)
        self.assertEqual(self.auto[0]['action'], 'start')
        self.node.tick()
        self.assertEqual(len(self.auto), 1)

    def test_finish_standby_stops_observation_and_clears_fire_request(self):
        self.node.request(self.message({'mode':'inspect'}))
        self.fpga_status(True)
        self.node.tick()
        self.assertTrue(self.states[-1]['yolo_requested'])
        count=len(self.commands)
        self.node.request(self.message({'mode':'standby'}))
        self.node.tick()
        self.assertEqual(len(self.commands),count)
        self.assertEqual(self.node.mode, 'standby')
        self.assertFalse(self.states[-1]['yolo_requested'])
        self.assertFalse(self.permissions[-1].allow_motion)

    def test_paused_navigation_proves_actual_stop_without_motion_permission(self):
        self.node.request(self.message({'mode':'inspect'}))
        self.proposal(valid=False, ready_to_move=False, arrived=False, status_fresh=True,
                      vehicle_stopped=True, reason='fpga_paused')
        self.node.tick()
        self.assertTrue(self.node.stopped(self.now))
        self.assertTrue(self.states[-1]['vehicle_stopped'])
        self.assertEqual(self.commands[-1].owner, 'mission_home')
        self.assertEqual((self.commands[-1].pan_deg, self.commands[-1].tilt_deg), (90.,90.))
        self.assertFalse(self.permissions[-1].allow_motion)

    def test_fresh_estop_holds_motion_and_suppresses_gimbal_commands(self):
        self.traffic_ready(navigate=True)
        self.assertTrue(self.permissions[-1].allow_motion)
        count = len(self.commands)
        self.proposal(valid=False, ready_to_move=False, vehicle_stopped=True, estop=True)
        self.node.tick()
        self.assertFalse(self.permissions[-1].allow_motion)
        self.assertTrue(self.states[-1]['emergency_stop'])
        self.assertEqual(len(self.commands), count)
        self.node.request(self.message({'mode':'inspect'}))
        self.node.tick()
        self.assertTrue(self.states[-1]['emergency_stop'])
        self.assertFalse(self.permissions[-1].allow_motion)
        self.assertEqual(len(self.commands), count, 'Inspection must not home the gimbal during estop')


    def test_cancelled_auto_cannot_restore_fire_mode_after_standby(self):
        now=self.ros_now()
        self.node.auto_request(self.message(dict(action='start',command_id='testauto',issued_at=now,ttl_s=2.)))
        self.node.request(self.message(dict(mode='inspect',auto_run_id='testauto')))
        self.assertEqual(self.node.mode,'inspect')
        self.node.auto_request(self.message(dict(action='cancel')))
        self.node.request(self.message(dict(mode='standby')))
        self.node.request(self.message(dict(mode='fire',auto_run_id='testauto')))
        self.assertEqual(self.node.mode,'standby')
        # A delayed automatic completion cannot override a new manual task either.
        self.node.request(self.message(dict(mode='inspect')))
        self.node.request(self.message(dict(mode='standby',auto_run_id='testauto')))
        self.assertEqual(self.node.mode,'inspect')


if __name__ == '__main__':
    unittest.main(verbosity=2)
