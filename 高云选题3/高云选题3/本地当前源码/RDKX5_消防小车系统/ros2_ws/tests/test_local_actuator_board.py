"""Isolated ROS contract test: dry-run only, with all hardware constructors blocked.

Run only with RESCUE_RUN_ROS_TEST=1 ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1.
Synthetic mission/FPGA/aim messages must never be published in live domain 0.
No camera, YOLO model, real UART, PWM, or GPIO is opened.
"""
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'rescue_vision'))


@unittest.skipUnless(os.environ.get('RESCUE_RUN_ROS_TEST') == '1', 'explicit isolated ROS test only')
class LocalActuatorBoardTests(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '83', 'Never inject synthetic control into live ROS')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1', 'Keep test ROS traffic on this host')
        self.temporary = tempfile.TemporaryDirectory(prefix='rescue-actuator-test-')
        self.addCleanup(self.temporary.cleanup)
        self.nodes = []
        self.patches = []
        self.executor = None
        self.rclpy = None
        self.closed = False
        # These guards are additional to explicit dry_run=true. Even an adapter
        # regression must fail here, rather than touching a hardware endpoint.
        for name in ('_HardwarePWM', '_PumpGPIO', '_ExclusiveOwner'):
            guard = patch('rescue_vision.local_actuator_io.' + name,
                          side_effect=AssertionError('Hardware access forbidden in isolated test: ' + name))
            guard.start()
            self.patches.append(guard)

    def tearDown(self):
        try:
            if self.executor is not None:
                for node in self.nodes:
                    self.executor.remove_node(node)
            for node in reversed(self.nodes):
                node.destroy_node()
            if self.executor is not None:
                self.executor.shutdown()
            if self.rclpy is not None and self.rclpy.ok():
                self.rclpy.shutdown()
        finally:
            for guard in reversed(self.patches):
                guard.stop()

    def write_config(self, name, value):
        path = Path(self.temporary.name) / name
        path.write_text(json.dumps(value), encoding='utf-8')
        return str(path)

    def start(self, *, burst_s=.4, mission_node=False, fpga_node=False):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String
        from rescue_interfaces.msg import GimbalCommand, FpgaEvent
        from rescue_vision.local_actuator_node import LocalActuator
        self.rclpy, self.String = rclpy, String
        self.GimbalCommand, self.FpgaEvent = GimbalCommand, FpgaEvent
        io = json.loads((ROOT / 'config/actuator_io.template.json').read_text(encoding='utf-8-sig'))
        io['dry_run'] = True
        io['pump'].update(enabled=True, gpio_chip=None, line_offset=None, active_low=None)
        control = dict(enabled=True, pump_enabled=True, external_pump_timeout_verified=False,
                       max_speed_deg_s=90., settle_s=.2,
                       pump_guard=dict(burst_s=burst_s, cooldown_s=1., max_bursts=1, stable_lock_s=.35))
        mount = json.loads((ROOT / 'config/mount.template.json').read_text(encoding='utf-8-sig'))
        mount.update(mounting_verified=True, pan_home_deg=90., tilt_home_deg=90.,
                     pan_limits_deg=[89., 91.], tilt_limits_deg=[89., 91.],
                     speed_deg_s=90., extra_settle_s=.2)
        args = ['--ros-args']
        values = dict(control_config=self.write_config('control.json', control),
                      io_config=self.write_config('io.json', io),
                      mount_config=self.write_config('mount.json', mount),
                      traffic_config=str(ROOT / 'config/traffic.json'),
                      dry_run='true', gimbal_transport='x5', gimbal_enabled='false')
        for key, value in values.items():
            args.extend(['-p', f'{key}:={value}'])
        rclpy.init(args=args)
        self.executor = SingleThreadedExecutor()
        self.actuator = LocalActuator()
        self.nodes.append(self.actuator)
        self.probe = Node('local_actuator_contract_probe')
        self.nodes.append(self.probe)
        self.mission = None
        self.fpga = None
        if mission_node:
            from rescue_vision.mission_node import Mission
            self.mission = Mission()
            self.nodes.append(self.mission)
        if fpga_node:
            from rescue_vision.fpga_bridge_node import FpgaBridge
            self.fpga = FpgaBridge()
            self.nodes.append(self.fpga)
        for node in self.nodes:
            self.executor.add_node(node)
        self.assertTrue(self.actuator.dry_run)
        self.assertTrue(self.actuator.io.dry_run)
        if self.fpga:
            self.assertTrue(self.fpga.dry)
            self.assertIsNone(self.fpga.serial)
        self.states, self.mission_states, self.transmissions = [], [], []
        self.probe.create_subscription(String, '/actuators/state',
            lambda msg: self.states.append((time.monotonic(), json.loads(msg.data))), 10)
        self.probe.create_subscription(String, '/mission/state',
            lambda msg: self.mission_states.append(json.loads(msg.data)), 10)
        self.probe.create_subscription(String, '/fpga/tx',
            lambda msg: self.transmissions.append(json.loads(msg.data)), 20)
        self.aim_pub = self.probe.create_publisher(String, '/fire/proposal', 10)
        self.request_pub = self.probe.create_publisher(String, '/actuators/request', 10)
        self.mode_pub = self.probe.create_publisher(String, '/mission/request', 10)
        self.status_pub = self.probe.create_publisher(FpgaEvent, '/fpga/events', 20)
        self.mission_pub = (None if mission_node else self.probe.create_publisher(String, '/mission/state', 10))
        self.gimbal_pub = (None if mission_node else self.probe.create_publisher(GimbalCommand, '/control/gimbal', 10))
        self.send_mission = self.send_aim = self.send_command = True
        self.command_owner = 'fire'
        self.aim_override = {}

    def feed(self):
        now = self.probe.get_clock().now()
        stamp = now.nanoseconds / 1e9
        if self.mission is not None:
            event = self.FpgaEvent()
            event.header.stamp = now.to_msg()
            event.event_type = self.FpgaEvent.STATUS
            event.vehicle_stopped = True
            self.status_pub.publish(event)
            self.mode_pub.publish(self.String(data=json.dumps(dict(mode='fire'))))
        elif self.send_mission:
            self.mission_pub.publish(self.String(data=json.dumps(dict(stamp=stamp, mode='fire',
                traffic_active=False, vehicle_stopped=True, gimbal_ready=True,
                operator_hold=False, mounting_verified=True, gimbal_target_deg=[90., 90.]))))
        if self.gimbal_pub is not None and self.send_command:
            command = self.GimbalCommand()
            command.header.stamp = now.to_msg()
            command.owner = self.command_owner
            command.pan_deg = command.tilt_deg = 90.
            command.speed_deg_s = 90.
            command.lease_ms = 300
            self.gimbal_pub.publish(command)
        if self.send_aim:
            proposal = dict(valid=True, capture_stamp=stamp, sequence=int(time.monotonic()*1000),
                coordinate_frame='undistorted_left_optical', image_locked=True,
                landing_compensated=True, landing_calibration_id='isolated-fixture-landing',
                pump_candidate=True, target_range_m=.7, servo_reference_target_deg=[90., 90.],
                aim=dict(coordinate_frame='undistorted_left_optical', image_size_wh=[1280, 1088],
                         camera_calibration_id='isolated-fixture-camera', image_locked=True),
                servo_plan=dict(command=None))
            proposal.update(self.aim_override)
            self.aim_pub.publish(self.String(data=json.dumps(proposal)))

    def run_until(self, condition, *, timeout=5., message='condition did not become true'):
        deadline, next_feed = time.monotonic()+timeout, 0.
        while time.monotonic() < deadline:
            if time.monotonic() >= next_feed:
                self.feed()
                next_feed = time.monotonic()+.04
            self.executor.spin_once(timeout_sec=.01)
            if condition():
                return
        self.fail(message + '; actuator=' + str(self.actuator.guard.snapshot()) +
                  '; fault=' + str(self.actuator.fault) + '; request=' + self.actuator.last_request)

    def run_for(self, seconds):
        end = time.monotonic()+seconds
        self.run_until(lambda: time.monotonic() >= end, timeout=seconds+.5)

    def ready_and_arm(self):
        self.run_until(lambda: all(self.actuator.gates(time.monotonic())),
                       message='fresh stopped/settled/calibrated gates not established')
        self.request_pub.publish(self.String(data=json.dumps(dict(action='arm_pump', lease_s=10.))))
        self.run_until(lambda: self.actuator.pump_on,
                       message='explicit arm and stable fresh lock did not produce dry-run ON')

    def test_mission_gimbal_feedback_explicit_short_pump_and_no_fpga_gimbal(self):
        self.start(burst_s=.4, mission_node=True, fpga_node=True)
        self.run_until(lambda: any(m.get('gimbal_ready') for m in self.mission_states),
                       message='Mission did not receive estimated local gimbal readiness')
        self.assertTrue(any(x.get('gimbal_settled_estimate') for _, x in self.states))
        self.assertTrue(all(x.get('actual_angles_deg') is None for _, x in self.states))
        self.ready_and_arm()
        on_at = time.monotonic()
        self.run_until(lambda: not self.actuator.pump_on and self.actuator.guard.state == 'EXHAUSTED')
        self.assertLess(time.monotonic()-on_at, .8, 'requested short burst did not turn OFF promptly')
        self.assertEqual(self.actuator.guard.burst_count, 1)
        self.run_for(.5)
        self.assertFalse(self.actuator.pump_on, 'printed flame must not restart exhausted burst budget')
        from rescue_vision import protocol
        self.assertTrue(self.transmissions, 'UART dry-run telemetry observation was not exercised')
        self.assertFalse(any(packet['kind'] == protocol.GIMBAL for packet in self.transmissions),
                         'X5 and FPGA must not both execute gimbal commands')
        recorded = list(self.actuator.io.commands)
        self.assertEqual(sum(c.get('operation') == 'pump' and c.get('enabled') is True for c in recorded), 1)
        self.assertTrue(any(c.get('operation') == 'pump' and c.get('enabled') is False for c in recorded))
        self.assertTrue(all(not s.get('hardware_written') for _, s in self.states))

    def test_missing_mission_aim_or_command_cancels_arm_and_needs_manual_rearm(self):
        self.start(burst_s=1.)
        for source in ('mission', 'aim', 'command'):
            with self.subTest(missing=source):
                self.send_mission = self.send_aim = self.send_command = True
                self.actuator.guard.disarm('test_new_manual_session')
                self.ready_and_arm()
                on_at = time.monotonic()
                setattr(self, 'send_' + source, False)
                self.run_until(lambda: not self.actuator.pump_on and not self.actuator.guard.armed,
                               timeout=1.5, message='missing ' + source + ' failed to cancel pump arm')
                self.assertLess(time.monotonic()-on_at, .8)
                setattr(self, 'send_' + source, True)
                self.run_for(.65)
                self.assertFalse(self.actuator.pump_on, 'source recovery must not automatically rearm')
                self.assertFalse(self.actuator.guard.armed)

    def test_unknown_owner_and_wrong_coordinate_lock_never_activate_outputs(self):
        self.start()
        self.command_owner = 'untrusted_owner'
        self.run_for(.7)
        self.assertFalse(self.actuator.pwm_active)
        self.assertFalse(self.actuator.pump_on)
        self.command_owner = 'fire'
        self.run_until(lambda: self.actuator.pwm_active and time.monotonic() >= self.actuator.settled_after)
        self.aim_override = dict(coordinate_frame='raw_left_camera_optical')
        self.run_for(.2)
        self.request_pub.publish(self.String(data=json.dumps(dict(action='arm_pump', lease_s=10.))))
        self.run_for(.5)
        self.assertFalse(self.actuator.guard.armed)
        self.assertFalse(self.actuator.pump_on)


if __name__ == '__main__':
    unittest.main()
