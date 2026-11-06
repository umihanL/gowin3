"""Validate the explicit fixed-camera water-aim policy and active profile.

This pure gate does not arm, send a pulse or assert that water hit a target.
It supplements the common fresh-target, stopped-vehicle and finite-pulse gates.
"""
import math


def fixed_camera_water_lock(proposal, detail, config, feedback, requested):
    if (proposal.get('aim_mode') != 'fixed_camera_3d' or
            proposal.get('camera_mount') != 'fixed_to_vehicle' or
            proposal.get('geometry_locked') is not True or
            detail.get('geometry_locked') is not True):
        return False
    frame = proposal.get('coordinate_frame')
    if (frame not in ('undistorted_left_optical', 'rectified_left_optical', 'left_camera_optical') or
            detail.get('coordinate_frame') != frame or detail.get('image_size_wh') != [1280, 1088]):
        return False
    pump = config.get('pump', {})
    servo = config.get('servo', {})
    policy = pump.get('water_aim_policy', 'measured_impact')
    if (policy not in ('measured_impact', 'straight_ray') or
            proposal.get('water_aim_policy', 'measured_impact') != policy or
            detail.get('water_aim_policy', 'measured_impact') != policy):
        return False
    identities = {
        'geometry_calibration_id': pump.get('geometry_calibration_id'),
        'camera_nozzle_mount_id': pump.get('camera_nozzle_mount_id'),
        'pump_setting_id': pump.get('setting_id'),
        'actuator_reference_id': servo.get('reference_id'),
    }
    if policy == 'measured_impact':
        if proposal.get('water_impact_calibration_verified') is not True:
            return False
        identities['landing_calibration_id'] = pump.get('fixed_camera_landing_calibration_id')
    if any(not isinstance(expected, str) or not expected.strip() or len(expected) > 128 or
           proposal.get(key) != expected or detail.get(key) != expected
           for key, expected in identities.items()):
        return False
    stereo = proposal.get('stereo_calibration_id')
    if (not isinstance(stereo, str) or not stereo or
            detail.get('camera_calibration_id') != stereo or detail.get('stereo_calibration_id') != stereo):
        return False
    if (feedback.get('feedback_fresh') is not True or
            feedback.get('reference_confirmed_this_start') is not True or
            feedback.get('motor_enabled') != [True, True] or feedback.get('motor_rpm') != [0, 0]):
        return False
    actual = feedback.get('actual_angles_deg')
    plan = proposal.get('servo_plan')
    qualified = plan.get('actual_angles_deg') if isinstance(plan, dict) else None
    target_key = 'corrected_target_deg'
    tolerance_key = 'water_impact_angle_tolerance_deg'
    if policy == 'straight_ray':
        # Straight aiming is an explicit policy, never an inferred fallback
        # from a missing water table. It must preserve its unmeasured impact.
        if any(value.get('landing_compensated') is not False or
               value.get('water_impact_calibration_verified') is not False or
               value.get('physical_hit_verified') is not False or
               value.get('landing_calibration_id') is not None or
               value.get('geometric_alignment_only') is not True or
               value.get('ray_tracking_reference') != 'straight_target_ray'
               for value in (proposal, detail)):
            return False
        ray_error = proposal.get('actual_ray_error_m')
        ray_limit = proposal.get('geometry_line_tolerance_m')
        along = proposal.get('actual_forward_ray_m')
        forward_min = proposal.get('geometry_min_forward_ray_m')
        if (any(type(v) not in (int, float) or not math.isfinite(v)
                for v in (ray_error, ray_limit, along, forward_min)) or
                not 0 <= ray_error <= ray_limit <= .002 or ray_limit <= 0 or
                not .01 <= forward_min <= .5 or along < forward_min or
                any(detail.get(key) != proposal.get(key) for key in
                    ('actual_ray_error_m', 'geometry_line_tolerance_m',
                     'actual_forward_ray_m', 'geometry_min_forward_ray_m'))):
            return False
        attainable = proposal.get('attainable_target_deg')
        if (not isinstance(attainable, (list, tuple)) or len(attainable) != 2 or
                detail.get('attainable_target_deg') != attainable or
                not isinstance(requested, (list, tuple)) or len(requested) != 2 or
                any(type(v) not in (int, float) or not math.isfinite(v)
                    for v in (*attainable, *requested)) or
                any(abs(a-b) > 1e-6 for a, b in zip(attainable, requested))):
            return False
        target_key = 'aim_target_deg'
        tolerance_key = 'geometry_angle_tolerance_deg'
    corrected = proposal.get(target_key)
    tolerance = proposal.get(tolerance_key)
    if (type(tolerance) not in (int, float) or not math.isfinite(tolerance) or
            not 0 < tolerance <= .3 or detail.get(tolerance_key) != tolerance or
            not isinstance(actual, (list, tuple)) or len(actual) != 2 or
            not isinstance(qualified, (list, tuple)) or len(qualified) != 2 or
            not isinstance(corrected, (list, tuple)) or len(corrected) != 2 or
            detail.get(target_key) != corrected or
            not isinstance(requested, (list, tuple)) or len(requested) != 2):
        return False
    # The geometric solver checked the 2 mm ray at this measured posture.
    # A newer, different encoder posture needs a new geometric proposal, even
    # if it remains inside the coarser generic motor angle tolerance.
    return all(type(a) in (int, float) and math.isfinite(a) and
               type(b) in (int, float) and math.isfinite(b) and
               type(c) in (int, float) and math.isfinite(c) and
               type(q) in (int, float) and math.isfinite(q) and abs(a-q) <= 1e-6 and
               abs(a-b) <= tolerance and abs(a-c) <= tolerance
               for a, b, c, q in zip(actual, requested, corrected, qualified))
