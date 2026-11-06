"""Direction confirmation never opens hardware or invents physical calibration."""
import copy
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
spec = importlib.util.spec_from_file_location('capture_mixed_reference', ROOT/'tools/capture_mixed_reference.py')
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)
from rescue_vision.pd42_actuator_io import angle_to_counts, counts_to_angle
from rescue_vision.gimbal_self_test import preflight_target


class ReadOnlyReply:
    def __init__(self,value=None,data=b''):
        self.value,self.data=value,data
    def require_success(self):return self


class CaptureBaselineTests(unittest.TestCase):
    def baseline(self,*,position=-3,error=19,realtime=16,speed=0,positions=None,pd42_error=0):
        pan=mock.Mock();tilt=mock.Mock();owner=mock.Mock()
        values=iter(positions) if positions is not None else None
        def pan_read(packet):
            command=capture.zdt.Command(packet[1])
            mapping={capture.zdt.Command.READ_VERSION:{'firmware_raw':206,'hardware_raw':0x2314},
                     capture.zdt.Command.READ_OPTIONS:{'raw_bytes_hex':'04 26'},
                     capture.zdt.Command.READ_TARGET_POSITION:16,
                     capture.zdt.Command.READ_REALTIME_TARGET_POSITION:realtime,
                     capture.zdt.Command.READ_POSITION_ERROR:error,
                     capture.zdt.Command.READ_SPEED:speed,
                     capture.zdt.Command.READ_STATE:{'raw':3}}
            if command==capture.zdt.Command.READ_CONFIG:return ReadOnlyReply(data=capture.vendor.CONFIG)
            if command==capture.zdt.Command.READ_POSITION:return ReadOnlyReply(next(values) if values else position)
            self.assertIn(command,mapping,'capture must not issue motor writes')
            return ReadOnlyReply(mapping[command])
        def tilt_read(packet):
            if packet==capture.pd42.read_request(1,capture.pd42.Command.READ_SYSTEM):
                return ReadOnlyReply({'speed_rpm':0,'stalled':False,'target_counts':pd42_error,'position_counts':0})
            self.assertEqual(packet,capture.pd42.read_request(1,capture.pd42.Command.READ_STATUS),'capture must only read')
            return ReadOnlyReply(1)
        pan.exchange.side_effect=pan_read;tilt.exchange.side_effect=tilt_read
        with mock.patch.object(capture,'_ExclusiveOwner',return_value=owner),mock.patch.object(capture,'ZDTSerialTransport',return_value=pan),mock.patch.object(capture,'PD42SerialTransport',return_value=tilt),mock.patch.object(capture.time,'sleep'):
            try:return capture.read_baseline()
            finally:
                pan.close.assert_called_once();tilt.close.assert_called_once();owner.close.assert_called_once()

    def test_stationary_hardware_residual_is_accepted_with_recorded_shared_tolerance(self):
        result=self.baseline()
        self.assertEqual(len(result['measurements']),3)
        self.assertEqual(result['fd8_stationary_tolerance_deg'],capture.vendor.STATIONARY_TOLERANCE_DEG)
        self.assertAlmostEqual(result['measurements'][0]['pan']['position_error_deg'],19*360/65536)
        self.assertEqual(result['measurements'][0]['pan']['position_counts'],-3)

    def test_27_count_boundary_passes_but_28_count_actual_or_following_error_fails(self):
        self.baseline(position=-11,error=27)
        for values in ({'position':-12,'error':27},{'position':-11,'error':28}):
            with self.subTest(values=values),self.assertRaisesRegex(ValueError,'ZDT must be stationary'):
                self.baseline(**values)

    def test_targets_motion_drift_and_pd42_limits_are_not_relaxed(self):
        for values,reason in [({'realtime':14},'ZDT must be stationary'),
                              ({'speed':1},'ZDT must be stationary'),
                              ({'positions':[-3,-3,3,3,3,3]},'pan moved during reference capture'),
                              ({'pd42_error':15},'PD42 must be stationary')]:
            with self.subTest(values=values),self.assertRaisesRegex(ValueError,reason):self.baseline(**values)


class DirectionConfirmationTests(unittest.TestCase):
    def setUp(self):
        axis = dict(min_angle_deg=0.,max_angle_deg=60.,reference_angle_deg=0.,
                    reference_counts=12345,direction=1,gear_ratio=1.,motion_enabled=False,
                    protocol='pd42')
        self.servo = {'pan':dict(axis,min_angle_deg=-90.,max_angle_deg=90.,motion_enabled=True),
                      'tilt':copy.deepcopy(axis),'position_tolerance_deg':.5}
        self.evidence = {'camera_to_nozzle_calibrated':False,'water_landing_calibrated':False}

    def test_omitted_option_locks_tilt_and_never_promotes_direction_or_other_calibration(self):
        self.servo['tilt'].update(motion_enabled=True,direction=-1)
        capture.apply_tilt_direction(self.servo,self.evidence)
        self.assertFalse(self.servo['tilt']['motion_enabled'])
        self.assertFalse(self.evidence['tilt_physical_direction_verified'])
        self.assertEqual(self.evidence['tilt_direction_confirmation']['source'],'not_confirmed')
        with self.assertRaisesRegex(ValueError,'tilt_direction_not_verified'):
            preflight_target([0.,1.8],self.servo)

    def test_observed_sign_controls_real_pd42_target_and_encoder_angle_conversion(self):
        for observed,sign in [('up',1),('down',-1)]:
            with self.subTest(observed=observed):
                capture.apply_tilt_direction(self.servo,self.evidence,observed)
                axis=self.servo['tilt'];target=angle_to_counts(1.8,axis)
                self.assertEqual(target-axis['reference_counts'],sign*256)
                self.assertAlmostEqual(counts_to_angle(target,axis),1.8)
                self.assertTrue(axis['motion_enabled'])
                self.assertEqual(preflight_target([0.,1.8],self.servo),[0.,1.8])
                self.assertEqual(self.evidence['tilt_direction_confirmation']['source'],'operator_observed_positive_motor_motion')
                self.assertEqual(self.evidence['tilt_direction_confirmation']['positive_motor_direction'],observed)
                self.assertFalse(self.evidence['camera_to_nozzle_calibrated'])
                self.assertFalse(self.evidence['water_landing_calibrated'])
                with self.assertRaises(ValueError):angle_to_counts(-1.,axis)
                with self.assertRaises(ValueError):angle_to_counts(61.,axis)

    def test_invalid_option_cannot_mutate_configuration(self):
        before=copy.deepcopy((self.servo,self.evidence))
        with self.assertRaises(ValueError):capture.apply_tilt_direction(self.servo,self.evidence,'unknown')
        self.assertEqual((self.servo,self.evidence),before)

    def test_cli_rejects_invalid_direction_before_any_serial_read(self):
        with mock.patch.object(sys,'argv',['capture','--operator-zero-confirmed','--supported-load','--tilt-positive-direction','left']), mock.patch.object(capture,'read_baseline') as read:
            with self.assertRaises(SystemExit):capture.main()
            read.assert_not_called()


if __name__=='__main__':unittest.main()
