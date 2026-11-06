"""Real LocalActuator gate + PD42 adapter feedback, without ROS graph/hardware.

Only ROS base/message imports are shimmed. The production gates() method and
PD42 feedback/transaction logic run unchanged against the motor fixture.
This validates callback integration, not DDS discovery or physical actuation.
"""
import importlib.util
import pathlib
import sys
import time
import types
import unittest
from unittest import mock

WORKSPACE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE/'src/rescue_vision'))
import test_pd42_actuator_io as fixtures


def load_local_actuator():
    modules = {name: types.ModuleType(name) for name in (
        'rclpy', 'rclpy.node', 'std_msgs', 'std_msgs.msg',
        'rescue_interfaces', 'rescue_interfaces.msg', 'rescue_vision.node_common')}
    modules['rclpy.node'].Node = object
    modules['std_msgs.msg'].Bool = type('Bool', (), {})
    modules['std_msgs.msg'].String = type('String', (), {})
    modules['rescue_interfaces.msg'].GimbalCommand = type('GimbalCommand', (), {})

    def unused(*args, **kwargs):
        raise AssertionError('ROS initialization or publishing forbidden in offline callback test')
    for name in ('age', 'param', 'publish_json', 'read_json', 'spin'):
        setattr(modules['rescue_vision.node_common'], name, unused)
    spec = importlib.util.spec_from_file_location(
        'rescue_vision._pd42_contract_node',
        WORKSPACE/'src/rescue_vision/rescue_vision/local_actuator_node.py')
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module.LocalActuator


class PD42NodeContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node_class = load_local_actuator()

    def setUp(self):
        self.io, self.pan, self.tilt, self.events = fixtures.AdapterTests().fixture()
        self.node = object.__new__(self.node_class)
        now = time.monotonic()
        node = self.node
        node.io = self.io
        node.fpga_pump = None  # This fixture exercises local PD42 feedback, not the FPGA pump bridge.
        node.measured_backend = True
        node.enabled = True
        node.mission_at = now
        node.mission = dict(mode='fire', vehicle_stopped=True, traffic_active=False,
                            operator_hold=False, emergency_stop=False)
        node.requested = node.commanded = (90., 90.)
        node.command_owner = 'fire'
        node.pwm_active = True
        node.command_deadline = now+.5
        node.settled_after = now-1  # Old PWM time estimate would already be ready.
        node.require_selection = False
        node.armed_auto_run = None
        node.aim_at = now
        node.get_clock = lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(nanoseconds=1000_000_000_000))
        node.aim = dict(
            valid=True, capture_stamp=1000., coordinate_frame='undistorted_left_optical',
            image_locked=True, pump_candidate=True, landing_compensated=True,
            landing_calibration_id='offline-landing', servo_reference_target_deg=[90., 90.],
            aim=dict(coordinate_frame='undistorted_left_optical', image_size_wh=[1280, 1088],
                     camera_calibration_id='offline-camera'))
        self.io.set_gimbal(90., 90., lease_s=.5)

    def gates(self):
        return self.node.gates(time.monotonic())

    def test_accepted_command_and_pwm_elapsed_time_are_insufficient(self):
        self.io._cycle()  # reads then accepts F2; no post-command reading yet
        self.assertTrue(self.io.feedback()['command_accepted'])
        self.assertFalse(self.io.feedback()['settled_measured'])
        self.assertEqual(self.gates(), (True, False, True))
        # Demonstrate that this scenario specifically exercises PD42's measured
        # branch: the same fields would pass the old time-only PWM path.
        self.node.measured_backend = False
        self.assertEqual(self.gates(), (True, True, True))

    def test_post_ack_encoder_feedback_enables_gate_but_stale_feedback_revokes(self):
        self.io._cycle()
        time.sleep(.02)
        self.io._cycle()
        self.assertEqual(self.gates(), (True, True, True))
        self.io._received = time.monotonic()-2
        self.assertEqual(self.gates(), (True, False, True))

    def test_correct_measurement_for_previous_target_cannot_authorize_new_target(self):
        self.io._cycle()
        time.sleep(.02)
        self.io._cycle()
        self.assertTrue(self.io.feedback()['settled_measured'])
        self.node.requested = self.node.commanded = (96., 90.)
        self.node.aim['servo_reference_target_deg'] = [96., 90.]
        self.assertEqual(self.gates(), (True, False, True))

    def fixed_proposal(self):
        self.io.config['servo']['reference_id'] = 'OFFLINE-reference'
        self.io.config['pump'].update(setting_id='OFFLINE-pressure',
            fixed_camera_landing_calibration_id='OFFLINE-impact',
            geometry_calibration_id='OFFLINE-geometry', camera_nozzle_mount_id='OFFLINE-rig')
        self.node.aim.update(aim_mode='fixed_camera_3d', camera_mount='fixed_to_vehicle',
            image_locked=False, geometry_locked=True, water_impact_calibration_verified=True,
            corrected_target_deg=[90., 90.],
            servo_plan=dict(actual_angles_deg=[90., 90.]),
            water_impact_angle_tolerance_deg=.1, landing_calibration_id='OFFLINE-impact',
            geometry_calibration_id='OFFLINE-geometry', actuator_reference_id='OFFLINE-reference',
            pump_setting_id='OFFLINE-pressure', camera_nozzle_mount_id='OFFLINE-rig',
            stereo_calibration_id='OFFLINE-stereo')
        for key in ('geometry_locked', 'water_impact_angle_tolerance_deg', 'landing_calibration_id',
                    'geometry_calibration_id', 'actuator_reference_id', 'pump_setting_id',
                    'camera_nozzle_mount_id', 'stereo_calibration_id', 'corrected_target_deg'):
            self.node.aim['aim'][key] = self.node.aim[key]
        self.node.aim['aim']['camera_calibration_id'] = 'OFFLINE-stereo'
        self.node.aim['aim']['image_locked'] = False
        self.io._cycle()
        time.sleep(.02)
        self.io._cycle()

    def test_fixed_camera_uses_measured_impact_lock_without_faking_image_lock(self):
        self.fixed_proposal()
        self.assertEqual(self.gates(), (True, True, True))
        self.assertFalse(self.node.aim['image_locked'])
        self.assertFalse(any(event[0] == 'pump' for event in self.events))

    def test_fixed_camera_profile_reference_or_verified_impact_mismatch_blocks_water(self):
        self.fixed_proposal()
        for key in ('pump_setting_id', 'landing_calibration_id', 'geometry_calibration_id',
                    'camera_nozzle_mount_id', 'actuator_reference_id', 'stereo_calibration_id',
                    'water_impact_calibration_verified', 'geometry_locked'):
            with self.subTest(key=key):
                old = self.node.aim[key]
                self.node.aim[key] = False if type(old) is bool else 'wrong-identity'
                self.assertFalse(self.gates()[2])
                self.node.aim[key] = old

    def test_current_angle_drift_cannot_reuse_a_previous_precise_water_lock(self):
        self.fixed_proposal()
        feedback = self.io.feedback()
        feedback['actual_angles_deg'] = [90.2, 90.]
        # General gimbal tolerance is 0.3 degrees; impact profile requires 0.1.
        with mock.patch.object(self.io, 'feedback', return_value=feedback):
            self.assertFalse(self.gates()[2])
        feedback['actual_angles_deg'] = [90.05, 90.]
        # Even inside 0.1 degrees, this different posture has not yet been
        # checked against the geometric ray tolerance by a fresh proposal.
        with mock.patch.object(self.io, 'feedback', return_value=feedback):
            self.assertFalse(self.gates()[2])
        self.node.aim['water_impact_angle_tolerance_deg'] = float('nan')
        self.assertFalse(self.gates()[2])


if __name__ == '__main__':
    unittest.main(verbosity=2)
