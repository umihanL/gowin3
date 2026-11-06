"""Synthetic calibration contracts only: no actual water samples or outputs."""
from copy import deepcopy
from itertools import product
import unittest

import numpy as np

from test_fixed_camera_aim import document, encoder, proposal, target_for
from rescue_vision.fixed_camera_aim import FixedCameraAim
from rescue_vision.fixed_camera_landing import FixedCameraLanding


def correction(pan,tilt,distance):
    return [.02*(pan-100)+.03*(distance-1),1+.02*(tilt-100)+.1*(distance-1)]


def landing_document():
    rows=[]
    for i,(pan,tilt,distance) in enumerate(product((95.,105.),(95.,105.),(.7,1.3))):
        rows.append(dict(nominal_pan_deg=pan,nominal_tilt_deg=tilt,nozzle_distance_m=distance,
            angle_correction_deg=correction(pan,tilt,distance),measurement_id=f'SYNTHETIC_CORNER_{i}',
            measured=True,measured_impact_error_m=[.001,0,0],measured_encoder_error_deg=[.01,.01]))
    return dict(mode='fixed_camera_measured_angular_grid',calibrated=True,
        calibration_id='SYNTHETIC_LANDING',measurement_session_id='SYNTHETIC_NO_PHYSICAL_WATER',
        geometry_calibration_id='SYNTHETIC_FIXED',stereo_calibration_id='SYNTHETIC_STEREO',
        actuator_reference_id='SYNTHETIC_AXES',pump_setting_id='SYNTHETIC_PUMP',
        camera_nozzle_mount_id='SYNTHETIC_MOUNT',angle_convention='actuator_joint_delta_deg',
        distance_reference='nominal_nozzle_forward_ray_m',measurement_unit='m',
        acceptance_radius_m=.005,water_impact_angle_tolerance_deg=.2,measurements=rows,
        independent_validation=[dict(nominal_pan_deg=100.,nominal_tilt_deg=100.,nozzle_distance_m=1.,
            measurement_id='SYNTHETIC_HELD_OUT',measured=True,measured_impact_error_m=[.002,0,0],
            measured_encoder_error_deg=[.01,.01],commanded_angles_deg=[100.,101.])])


def profile_encoder(now=100.,angles=(100.,101.)):
    state=encoder(now,angles)
    state.update(geometry_calibration_id='SYNTHETIC_FIXED',pump_setting_id='SYNTHETIC_PUMP',
                 camera_nozzle_mount_id='SYNTHETIC_MOUNT',fixed_camera_landing_calibration_id='SYNTHETIC_LANDING')
    return state


class MeasuredAngularGridTests(unittest.TestCase):
    def test_trilinear_correction_and_independent_validation(self):
        grid=FixedCameraLanding(landing_document())
        result=grid.correction([97,103],.9,document(),profile_encoder())
        np.testing.assert_allclose(result['angle_correction_deg'],[-.063,1.05],atol=1e-12)
        np.testing.assert_allclose(result['corrected_target_deg'],[96.937,104.05],atol=1e-12)
        self.assertTrue(result['water_impact_calibration_verified'])
        self.assertFalse(result['physical_hit_verified'])

    def test_complete_grid_and_independent_interior_water_validation_required(self):
        for fault in ('missing_corner','duplicate','no_validation','reused_sample','boundary_validation',
                      'bad_impact','unmeasured','wrong_validation_angle','bad_encoder'):
            with self.subTest(fault=fault):
                doc=landing_document()
                if fault=='missing_corner':doc['measurements'].pop()
                elif fault=='duplicate':doc['measurements'][-1]=deepcopy(doc['measurements'][0])
                elif fault=='no_validation':doc['independent_validation']=[]
                elif fault=='reused_sample':doc['independent_validation'][0]['measurement_id']=doc['measurements'][0]['measurement_id']
                elif fault=='boundary_validation':doc['independent_validation'][0]['nominal_pan_deg']=95
                elif fault=='bad_impact':doc['independent_validation'][0]['measured_impact_error_m']=[.01,0,0]
                elif fault=='unmeasured':doc['measurements'][0]['measured']=False
                elif fault=='wrong_validation_angle':doc['independent_validation'][0]['commanded_angles_deg']=[100,100]
                else:doc['independent_validation'][0]['measured_encoder_error_deg']=[.3,0]
                with self.assertRaises(ValueError):FixedCameraLanding(doc)

    def test_no_extrapolation_in_any_dimension(self):
        grid=FixedCameraLanding(landing_document())
        for point in ([94.99,100,1],[105.01,100,1],[100,94.99,1],[100,105.01,1],[100,100,.699],[100,100,1.301]):
            with self.subTest(point=point),self.assertRaisesRegex(ValueError,'outside_measured_grid'):
                grid.correction(point[:2],point[2],document(),profile_encoder())

    def test_all_installation_pump_reference_and_calibration_bindings_are_required(self):
        grid=FixedCameraLanding(landing_document())
        for key in ('reference_id','geometry_calibration_id','pump_setting_id',
                    'camera_nozzle_mount_id','fixed_camera_landing_calibration_id'):
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'active_profile_mismatch'):
                state=profile_encoder();state[key]='CHANGED'
                grid.correction([100,100],1,document(),state)
        for key in ('calibration_id','stereo_calibration_id','actuator_reference_id'):
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'geometry_reference_mismatch'):
                doc=document();doc[key]='CHANGED'
                grid.correction([100,100],1,doc,profile_encoder())

    def test_unfilled_template_stays_disabled(self):
        import json
        from pathlib import Path
        path=Path(__file__).resolve().parents[1]/'config/fixed_camera_landing.template.json'
        self.assertFalse(FixedCameraLanding(json.loads(path.read_text('utf-8'))).ready)


class FixedWaterProposalTests(unittest.TestCase):
    def test_water_offset_locks_outgoing_calibrated_ray_without_fake_pixel_lock(self):
        model=FixedCameraAim(document());grid=FixedCameraLanding(landing_document())
        for i in range(7):
            result=proposal(model,i,encoder=profile_encoder(100+i*.1),landing=grid)
        self.assertTrue(result['valid']);self.assertTrue(result['geometry_locked'])
        self.assertTrue(result['pump_candidate']);self.assertTrue(result['landing_compensated'])
        self.assertLess(result['actual_tracking_ray_error_m'],1e-6)
        self.assertGreater(result['actual_ray_error_m'],.01)
        self.assertFalse(result['image_locked']);self.assertFalse(result['physical_hit_verified'])
        self.assertFalse(result['pump_allowed'])
        np.testing.assert_allclose(result['corrected_target_deg'],[100,101],atol=.001)

    def test_same_straight_geometry_without_water_table_cannot_request_water(self):
        model=FixedCameraAim(document())
        for i in range(7):result=proposal(model,i,measured=(100,100))
        self.assertTrue(result['geometry_locked']);self.assertFalse(result['pump_candidate'])

    def test_profile_change_and_domain_exit_clear_existing_water_lock(self):
        for fault in ('pump','reference','target','depth'):
            model=FixedCameraAim(document());grid=FixedCameraLanding(landing_document())
            for i in range(7):result=proposal(model,i,encoder=profile_encoder(100+i*.1),landing=grid)
            self.assertTrue(result['pump_candidate'])
            state=profile_encoder(100.7);target=target_for(model)
            if fault=='pump':state['pump_setting_id']='other'
            elif fault=='reference':state['reference_id']='new_boot'
            elif fault=='target':target=target_for(model,joints=(120,100))
            else:target['depth_valid']=False
            result=proposal(model,7,encoder=state,target=target,landing=grid)
            self.assertFalse(result['valid']);self.assertFalse(result['pump_candidate'])
            self.assertFalse(result['geometry_locked']);self.assertIsNone(result['servo_plan']['command'])

    def test_compensated_angle_limit_is_not_clamped_into_success(self):
        doc=document();doc['axes']['tilt']['limits_deg']=[40,100.5]
        model=FixedCameraAim(doc);state=profile_encoder(100,(100,100))
        state['axis_limits_deg']=[[30,150],[40,100.5]]
        result=proposal(model,encoder=state,landing=FixedCameraLanding(landing_document()))
        self.assertEqual(result['reason'],'fixed_camera_compensated_target_outside_limits')
        self.assertIsNone(result['servo_plan']['command']);self.assertFalse(result['pump_candidate'])


if __name__=='__main__':unittest.main(verbosity=2)
