"""Real fire callbacks with synthetic depth, explicit selection and encoders."""
from collections import OrderedDict
from types import SimpleNamespace as NS
import json
import unittest
from unittest.mock import patch

import test_fire_aim_geometry as support
from test_fixed_camera_aim import document, encoder, target_for
from rescue_vision.fixed_camera_aim import FixedCameraAim
from rescue_vision.fire_targets import SelectionHistory
from rescue_vision.aim_geometry import EncoderPoseHistory
from rescue_vision.legacy_visual_servo import ServoConfig
from rescue_vision.fixed_camera_landing import FixedCameraLanding
from test_fixed_camera_landing import landing_document, profile_encoder


class FixedCameraCallbacks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.module=support.isolated_node_class()

    def setUp(self):
        support.FireAimCallbackTests.setUp(self)
        self.node.fixed_aim=FixedCameraAim(document())
        self.node.require_selection=True
        self.node.selection_history=SelectionHistory()
        self.node.pending_detections=OrderedDict()
        self.node.encoder_history=EncoderPoseHistory()
        self.node.latest_actuator_state=None
        self.node.servo_cfg=ServoConfig(enabled=True,mounting_verified=True)
        self.node.stereo=NS(calibrated=True,calibration_id='SYNTHETIC_STEREO')

    def capture(self,number,*,identity=('session',1,1),selected=True,actual=(90,90)):
        now=100.+number*.1
        self.node.test_now=now;self.node.mission_received=number*.1
        msg=support.FireAimCallbackTests.frame(self,number)
        msg.stereo_calibration_id='SYNTHETIC_STEREO'
        target=target_for(self.node.fixed_aim)
        item=msg.detections[0]
        item.depth_valid=True;item.depth_reason=target['depth_reason'];item.depth_m=target['depth_m']
        item.position_m=NS(x=target['position_m'][0],y=target['position_m'][1],z=target['position_m'][2])
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        choice=dict(valid=True,registry_session=identity[0],selection_generation=identity[1],target_id=identity[2],
            capture_stamp_ns=stamp,sequence=number,coordinate_frame=msg.header.frame_id,
            image_size_wh=[msg.width,msg.height],physical_eye='left',bbox_xyxy=list(item.bbox_xyxy))
        self.node.actuator_state(NS(data=json.dumps(encoder(now,actual))))
        if selected:self.node.selection_history.observe(choice,number*.1)
        return msg,choice

    def receive(self,msg,number):
        with patch.object(self.module.time,'monotonic',return_value=number*.1):self.node.receive(msg)
        return self.node.publisher[-1]

    def test_selected_stereo_geometry_proposes_commands_with_no_pixel_lock(self):
        for i in range(3):
            msg,_=self.capture(i);result=self.receive(msg,i)
        self.assertEqual(result['aim_mode'],'fixed_camera_3d')
        self.assertIsNotNone(result['servo_plan']['command'])
        self.assertEqual(result['target_id'],1)
        self.assertFalse(result['image_locked']);self.assertFalse(result['pump_candidate'])

    def test_angle_lock_uses_actual_encoders_even_with_off_center_box(self):
        for i in range(7):
            msg,_=self.capture(i,actual=(100,100));result=self.receive(msg,i)
        self.assertTrue(result['geometry_locked'])
        self.assertEqual(result['aim']['state'],'GEOMETRY_LOCKED')
        self.assertFalse(result['image_locked']);self.assertFalse(result['physical_hit_verified'])

    def test_explicit_selection_arriving_after_depth_replays_exact_capture_only(self):
        msg,choice=self.capture(0,selected=False)
        result=self.receive(msg,0)
        self.assertFalse(result['valid']);self.assertIsNone(result['servo_plan']['command'])
        with patch.object(self.module.time,'monotonic',return_value=0.):
            self.node.selection(NS(data=json.dumps(choice)))
        self.assertTrue(self.node.publisher[-1]['valid'])
        self.assertEqual(self.node.publisher[-1]['target_id'],1)

    def test_repeated_depth_first_callback_order_does_not_reset_every_step(self):
        for i in range(3):
            msg,choice=self.capture(i,selected=False)
            self.receive(msg,i)
            with patch.object(self.module.time,'monotonic',return_value=i*.1):
                self.node.selection(NS(data=json.dumps(choice)))
        self.assertTrue(self.node.publisher[-1]['valid'])
        self.assertIsNotNone(self.node.publisher[-1]['servo_plan']['command'])
        self.assertAlmostEqual(self.node.publisher[-1]['servo_plan']['command']['pan_target_deg'],90.6)

    def test_unmatched_source_calibration_and_missing_depth_never_center_pixels(self):
        for fault in ('calibration','depth','reference'):
            self.setUp()
            msg,_=self.capture(0)
            if fault=='calibration':msg.stereo_calibration_id='old'
            elif fault=='depth':msg.detections[0].depth_valid=False
            else:self.node.latest_actuator_state['reference_id']='old'
            result=self.receive(msg,0)
            self.assertFalse(result['valid']);self.assertIsNone(result['servo_plan']['command'])
            self.assertFalse(result['geometry_locked']);self.assertFalse(result['image_locked'])

    def test_unfilled_fixed_template_stays_fixed_and_never_constructs_legacy_tracker(self):
        msg,_=self.capture(0)
        self.node.fixed_aim=FixedCameraAim(dict(mode='fixed_camera_3d',calibrated=False))
        result=self.receive(msg,0)
        self.assertEqual(result['aim_mode'],'fixed_camera_3d')
        self.assertFalse(result['calibration_ready']);self.assertIsNone(self.node.tracker)
        self.assertEqual(result['reason'],'fixed_camera_uncalibrated')

    def test_preview_camera_info_does_not_reset_fixed_stereo_identity(self):
        msg,_=self.capture(0);self.receive(msg,0)
        identity=self.node.fixed_aim.identity
        info=support.camera_info();info.header.frame_id='rectified_left_optical'
        self.node.info(info)
        self.assertEqual(self.node.fixed_aim.identity,identity)
        msg,_=self.capture(1);result=self.receive(msg,1)
        self.assertTrue(result['valid']);self.assertIsNotNone(result['servo_plan']['command'])

    def test_watchdog_and_selection_clear_remove_geometric_lock_and_commands(self):
        for i in range(7):
            msg,_=self.capture(i,actual=(100,100));self.receive(msg,i)
        with patch.object(self.module.time,'monotonic',return_value=1.2):self.node.watchdog()
        self.assertFalse(self.node.publisher[-1]['geometry_locked'])
        self.assertIsNone(self.node.publisher[-1]['servo_plan']['command'])
        with patch.object(self.module.time,'monotonic',return_value=1.2):
            self.node.selection(NS(data=json.dumps(dict(valid=False,registry_session='session',selection_generation=2))))
        self.assertIsNone(self.node.fixed_aim.identity)

    def test_measured_water_grid_proposal_binds_profile_and_never_fakes_image_lock(self):
        self.node.fixed_landing=FixedCameraLanding(landing_document())
        for i in range(7):
            msg,_=self.capture(i,actual=(100,101))
            self.node.latest_actuator_state=profile_encoder(100+i*.1)
            result=self.receive(msg,i)
        self.assertTrue(result['geometry_locked']);self.assertTrue(result['pump_candidate'])
        self.assertFalse(result['image_locked']);self.assertFalse(result['pump_allowed'])
        self.assertEqual(result['landing_calibration_id'],'SYNTHETIC_LANDING')
        self.assertEqual(result['aim']['pump_setting_id'],'SYNTHETIC_PUMP')
        self.assertEqual(result['aim']['image_size_wh'],[1280,1088])
        self.assertEqual(result['aim']['coordinate_frame'],'undistorted_left_optical')
        np = __import__('numpy')
        np.testing.assert_allclose(result['servo_reference_target_deg'],[100,101],atol=.001)
        msg,_=self.capture(7,actual=(100,101))
        self.node.latest_actuator_state=profile_encoder(100.7)
        self.node.latest_actuator_state['pump_setting_id']='CHANGED'
        result=self.receive(msg,7)
        self.assertFalse(result['valid']);self.assertFalse(result['pump_candidate'])
        self.assertIsNone(result['servo_plan']['command'])

    def test_post_solve_mounting_gate_clears_water_candidate(self):
        self.node.fixed_landing=FixedCameraLanding(landing_document())
        for i in range(7):
            msg,_=self.capture(i,actual=(100,101))
            self.node.latest_actuator_state=profile_encoder(100+i*.1)
            result=self.receive(msg,i)
        self.assertTrue(result['pump_candidate'])
        msg,_=self.capture(7,actual=(100,101))
        self.node.latest_actuator_state=profile_encoder(100.7)
        self.node.servo_cfg.mounting_verified=False
        result=self.receive(msg,7)
        self.assertFalse(result['valid']);self.assertFalse(result['pump_candidate'])

    def test_actual_node_proposal_is_accepted_by_actuator_fixed_water_gate(self):
        from copy import deepcopy
        from rescue_vision.fixed_camera_pump_gate import fixed_camera_water_lock
        self.node.fixed_landing=FixedCameraLanding(landing_document())
        for i in range(7):
            msg,_=self.capture(i,actual=(100,101))
            self.node.latest_actuator_state=profile_encoder(100+i*.1)
            result=self.receive(msg,i)
        config=dict(servo=dict(reference_id='SYNTHETIC_AXES'),pump=dict(
            setting_id='SYNTHETIC_PUMP',fixed_camera_landing_calibration_id='SYNTHETIC_LANDING',
            geometry_calibration_id='SYNTHETIC_FIXED',camera_nozzle_mount_id='SYNTHETIC_MOUNT'))
        feedback=profile_encoder(100.6)
        requested=result['servo_reference_target_deg']
        self.assertTrue(result['pump_candidate'])
        self.assertTrue(fixed_camera_water_lock(result,result['aim'],config,feedback,requested))
        changed=deepcopy(config);changed['pump']['setting_id']='CHANGED'
        self.assertFalse(fixed_camera_water_lock(result,result['aim'],changed,feedback,requested))
        feedback['actual_angles_deg'][0]+=.25
        self.assertFalse(fixed_camera_water_lock(result,result['aim'],config,feedback,requested))
        msg,_=self.capture(7,actual=(100,101))
        self.node.latest_actuator_state=profile_encoder(100.7)
        self.node.latest_actuator_state['pump_setting_id']='CHANGED'
        blocked=self.receive(msg,7)
        self.assertFalse(fixed_camera_water_lock(blocked,blocked['aim'],config,profile_encoder(),requested))


if __name__=='__main__':unittest.main(verbosity=2)
