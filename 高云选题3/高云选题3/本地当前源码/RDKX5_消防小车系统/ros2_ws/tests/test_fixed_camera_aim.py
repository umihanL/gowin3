"""Synthetic offset-gimbal geometry and admission checks; no ROS/physical I/O."""
from copy import deepcopy
from pathlib import Path
import json
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.fixed_camera_aim import FixedCameraAim


def document():
    doc = json.loads((Path(__file__).resolve().parents[1]/'config/fixed_camera_aim.template.json').read_text('utf-8'))
    doc.update(calibrated=True, calibration_id='SYNTHETIC_FIXED', stereo_calibration_id='SYNTHETIC_STEREO',
               actuator_reference_id='SYNTHETIC_AXES', axis_reference_verified=True, image_size_wh=[1280,1088],
               R_camera_to_gimbal=[[0,0,1],[-1,0,0],[0,-1,0]], t_camera_origin_in_gimbal_m=[0,0,0],
               tilt_origin_in_pan_m=[.02,0,.03], nozzle_origin_in_tilt_m=[.04,.01,.01],
               nozzle_direction_in_tilt=[1.,0.,0.], valid_range_m=[.2,3.],
               axes=dict(pan=dict(zero_angle_deg=90.,positive_direction=1,limits_deg=[30.,150.]),
                         tilt=dict(zero_angle_deg=90.,positive_direction=1,limits_deg=[40.,140.])))
    return doc


def encoder(now=100., angles=(90.,90.)):
    return dict(stamp=now, feedback_age_s=0., actual_angles_deg=list(angles),
                dry_run=False, enabled=True, feedback_fresh=True, status_fresh=True,
                error='', estop=False, reference_confirmed_this_start=True,
                reference_id='SYNTHETIC_AXES', axis_limits_deg=[[30.,150.],[40.,140.]],
                motor_rpm=[0,0],motor_enabled=[True,True])


def target_for(model, joints=(100.,100.), distance=1.):
    origin, direction = model.forward(joints)
    p = origin+distance*direction
    xyz = model.R.T @ (p-model.t)
    return dict(class_id=0, confidence=.9, depth_valid=True, depth_reason='verified_stereo_wallpaper_surface',
                position_m=xyz.tolist(), depth_m=float(xyz[2]))


def proposal(model, number=0, *, joints=(100.,100.), measured=(90.,90.), **changes):
    now = 100.+number*.1
    arguments = dict(target=target_for(model,joints),capture_s=now,sequence=number,now_s=now,
        frame_id='undistorted_left_optical',image_size=(1280,1088),rectified=False,
        stereo_calibration_id='SYNTHETIC_STEREO',active_stereo_calibration_id='SYNTHETIC_STEREO',
        identity=('session',1,1),encoder=encoder(now,measured))
    arguments.update(changes)
    return model.propose(**arguments)


class FixedCameraGeometryTests(unittest.TestCase):
    def test_camera_to_base_rotation_translation_and_pan_tilt_signs(self):
        doc=document();doc['tilt_origin_in_pan_m']=doc['nozzle_origin_in_tilt_m']=[0.,0.,0.]
        doc['t_camera_origin_in_gimbal_m']=[0.,.05,.02]
        model=FixedCameraAim(doc)
        target=dict(class_id=0,confidence=.9,depth_valid=True,depth_reason='verified_stereo_wallpaper_surface',
                    position_m=[.2,-.1,1.],depth_m=1.)
        result=proposal(model,target=target)
        np.testing.assert_allclose(result['target_position_gimbal_m'],[1.,-.15,.12],atol=1e-12)
        expected=[90+np.degrees(np.arctan2(-.15,1.)),90+np.degrees(np.arctan2(.12,np.hypot(1.,.15)))]
        np.testing.assert_allclose(result['geometry_target_deg'],expected,atol=.001)

    def test_offset_axes_inverse_returns_forward_ray_for_multiple_positions(self):
        model=FixedCameraAim(document())
        for pan in (45.,90.,140.):
            for tilt in (50.,90.,130.):
                with self.subTest(pan=pan,tilt=tilt):
                    origin,direction=model.forward([pan,tilt])
                    point=origin+1.3*direction
                    result=model.inverse(point,[90,90])
                    np.testing.assert_allclose(result['geometry_target_deg'],[pan,tilt],atol=.002)
                    np.testing.assert_allclose(np.array(result['nozzle_origin_gimbal_m'])+
                        result['nozzle_target_distance_m']*np.array(result['nozzle_direction_gimbal']),point,atol=1e-6)

    def test_nozzle_and_tilt_offsets_change_solution_not_silently_assumed_concentric(self):
        doc=document(); model=FixedCameraAim(doc)
        point=np.array([.5,.1,.15])
        correct=model.inverse(point,[90,90])['geometry_target_deg']
        doc['tilt_origin_in_pan_m']=doc['nozzle_origin_in_tilt_m']=[0,0,0]
        incorrect=FixedCameraAim(doc).inverse(point,[90,90])['geometry_target_deg']
        self.assertGreater(np.linalg.norm(np.array(correct)-incorrect),2.)

    def test_axis_reference_and_direction_are_used(self):
        doc=document();doc['axes']['pan']['positive_direction']=-1
        model=FixedCameraAim(doc)
        origin,direction=model.forward([70.,105.])
        result=model.inverse(origin+direction,[90,90])
        np.testing.assert_allclose(result['geometry_target_deg'],[70,105],atol=.002)

    def test_bad_extrinsics_units_or_unmeasured_offsets_cannot_claim_calibrated(self):
        cases=[('R_camera_to_gimbal',np.diag([1,1,-1]).tolist()),('translation_unit','mm'),
               ('R_camera_to_gimbal',(np.eye(3)*2).tolist()),('tilt_origin_in_pan_m',None),
               ('nozzle_direction_in_tilt',[2,0,0]),('actuator_reference_id',None),
               ('axis_model','concentric_assumed')]
        for key,value in cases:
            with self.subTest(key=key):
                doc=document();doc[key]=value
                with self.assertRaises(ValueError):FixedCameraAim(doc)

    def test_behind_target_and_outside_limits_have_no_clamped_solution(self):
        model=FixedCameraAim(document())
        for point in ([-1,0,0],[.1,3,0],[.1,0,3]):
            with self.subTest(point=point),self.assertRaisesRegex(ValueError,'unreachable'):
                model.inverse(point,[90,90])

    def test_template_has_no_geometry_or_pixel_fallback(self):
        doc=document();doc['calibrated']=False
        model=FixedCameraAim(doc)
        result=model.propose(target={},capture_s=100,sequence=1,frame_id='',image_size=[],rectified=False,
            stereo_calibration_id='',active_stereo_calibration_id='',identity=None,encoder={},now_s=100)
        self.assertFalse(result['valid']);self.assertFalse(result['calibration_ready'])
        self.assertIsNone(result['servo_plan']['command'])
        self.assertEqual(result['reason'],'fixed_camera_uncalibrated')


class FixedCameraAdmissionTests(unittest.TestCase):
    def test_sub_degree_angle_alone_cannot_claim_millimetre_ray_lock(self):
        model=FixedCameraAim(document())
        for i in range(7):
            result=proposal(model,i,joints=(100,100),measured=(99.75,100))
        self.assertTrue(result['valid'])
        self.assertLess(max(map(abs,result['servo_plan']['geometric_angle_error_deg'])),.3)
        self.assertGreater(result['actual_ray_error_m'],model.line_tolerance_m)
        self.assertFalse(result['geometry_locked'])

    def test_fd8_lattice_cannot_silently_relax_angle_or_ray_accuracy(self):
        model=FixedCameraAim(document())
        state=encoder(100,(90,100))
        state.update(axis_step_deg=[.9,None],axis_reachable_limits_deg=[[30.6,149.4],[40.,140.]])
        # Pan grid includes 90, 90.9, ...; 100 lies 0.1 from 99.9, while
        # 100.35 lies halfway between 99.9 and 100.8 (0.45 degree error).
        result=proposal(model,joints=(100.35,100),encoder=state)
        self.assertEqual(result['reason'],'fixed_camera_actuator_resolution_insufficient')
        self.assertFalse(result['valid']);self.assertFalse(result['geometry_locked'])
        self.assertAlmostEqual(result['attainable_angle_error_deg'],.45,places=3)
        self.assertIsNone(result['servo_plan']['command'])
        self.assertFalse(result['pump_candidate'])

    def test_reachable_fd8_target_can_lock_without_changing_precision(self):
        model=FixedCameraAim(document())
        for i in range(7):
            state=encoder(100+i*.1,(99.9,100))
            state.update(axis_step_deg=[.9,None],axis_reachable_limits_deg=[[30.6,149.4],[40.,140.]])
            result=proposal(model,i,joints=(99.9,100),encoder=state)
        self.assertTrue(result['actuator_resolution_checked'])
        self.assertTrue(result['attainable_precision_met'])
        self.assertTrue(result['geometry_locked'])
        self.assertLess(result['actual_ray_error_m'],1e-6)
        self.assertFalse(result['pump_candidate'])

    def test_feedback_scan_and_delivery_age_are_both_counted(self):
        state=encoder(99.9,(100,100));state['feedback_age_s']=.125
        result=proposal(FixedCameraAim(document()),encoder=state)
        self.assertEqual(result['reason'],'fixed_camera_encoder_feedback_required')
        self.assertFalse(result['geometry_locked'])

    def test_invalid_resolution_metadata_cannot_authorize_commands(self):
        for step,bounds in [(0,[[30.,150.],[40.,140.]]),
                            (.9,[[30.,150.],[40.,140.]]),
                            (.9,[[29.7,150.3],[40.,140.]])]:
            state=encoder();state.update(axis_step_deg=[step,None],axis_reachable_limits_deg=bounds)
            result=proposal(FixedCameraAim(document()),encoder=state)
            self.assertFalse(result['valid']);self.assertIsNone(result['servo_plan']['command'])

    def test_real_depth_and_exact_calibration_are_required(self):
        cases=[dict(stereo_calibration_id='old'),dict(active_stereo_calibration_id='old'),
               dict(frame_id='undistorted_right_optical'),dict(rectified=True),dict(image_size=(640,480)),
               dict(capture_s=99.4),dict(identity=None)]
        for change in cases:
            with self.subTest(change=change):
                result=proposal(FixedCameraAim(document()),**change)
                self.assertFalse(result['valid']);self.assertIsNone(result['servo_plan']['command'])
        for change in [dict(depth_valid=False),dict(depth_reason='verified_fixed_observation_assumption'),
                       dict(depth_m=4.),dict(position_m=[0,0,-1]),dict(confidence=.1)]:
            model=FixedCameraAim(document());target=target_for(model);target.update(change)
            result=proposal(model,target=target)
            self.assertFalse(result['valid']);self.assertFalse(result['pump_candidate'])

    def test_actual_fresh_calibrated_encoder_required(self):
        for change in [dict(dry_run=True),dict(reference_confirmed_this_start=False),dict(reference_id='old'),
                       dict(feedback_age_s=.3),dict(stamp=99.),dict(motor_rpm=[0,1],feedback_fresh=False),
                       dict(actual_angles_deg=[160,90]),dict(axis_limits_deg=[[0,180],[0,180]])]:
            with self.subTest(change=change):
                state=encoder();state.update(change)
                result=proposal(FixedCameraAim(document()),encoder=state)
                self.assertFalse(result['valid']);self.assertIsNone(result['servo_plan']['command'])

    def test_steps_use_actual_position_not_accumulation_of_unexecuted_commands(self):
        model=FixedCameraAim(document())
        first=proposal(model)
        self.assertIsNone(first['servo_plan']['command'])
        second=proposal(model,1);third=proposal(model,2)
        self.assertEqual(second['servo_plan']['command'],third['servo_plan']['command'])
        self.assertAlmostEqual(second['servo_plan']['command']['pan_target_deg'],90.6)
        self.assertFalse(third['geometry_locked'])

    def test_geometric_lock_does_not_require_image_center_or_authorize_water(self):
        model=FixedCameraAim(document())
        for i in range(6):result=proposal(model,i,measured=(100.,100.))
        self.assertTrue(result['geometry_locked'])
        self.assertNotEqual(result['target_position_camera_m'][:2],[0.,0.])
        self.assertFalse(result['image_locked'])
        self.assertFalse(result['pump_candidate']);self.assertFalse(result['physical_hit_verified'])

    def test_new_selection_depth_loss_and_duplicate_capture_reset_lock(self):
        for fault in ('selection','depth','duplicate'):
            with self.subTest(fault=fault):
                model=FixedCameraAim(document())
                for i in range(6):result=proposal(model,i,measured=(100,100))
                self.assertTrue(result['geometry_locked'])
                if fault=='selection':result=proposal(model,6,measured=(100,100),identity=('session',2,2))
                elif fault=='depth':
                    target=target_for(model);target['depth_valid']=False
                    result=proposal(model,6,measured=(100,100),target=target)
                else:result=proposal(model,5,measured=(100,100))
                self.assertFalse(result['geometry_locked'])
                self.assertIsNone(result['servo_plan']['command'])


if __name__=='__main__':unittest.main(verbosity=2)
