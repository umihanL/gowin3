"""Admission for an explicitly selected, stationary, single-burst fire round."""
import math

from .route_fire_patrol import fresh, off_disarmed


IDENTITY_KEYS = ('target_id', 'registry_session', 'selection_generation')


def selected_fire_round_reason(now, *, mission=None, selection=None, proposal=None,
                               actuator=None, extension=None, extension_enabled=False,
                               request=None):
    mission, selection, proposal, actuator, extension, request = (
        mission or {}, selection or {}, proposal or {}, actuator or {}, extension or {}, request or {})
    if (not fresh(mission, now, .5) or mission.get('mode') not in ('inspect', 'fire') or
            mission.get('vehicle_stopped') is not True or mission.get('traffic_active') or
            mission.get('operator_hold') or mission.get('emergency_stop') or mission.get('auto_fire_active')):
        return 'one_click_fire_requires_stopped_idle_observation'
    if extension_enabled:
        if (not fresh(extension, now, .6) or extension.get('real_uart') is not True or
                extension.get('status_fresh') is not True or extension.get('vehicle_stopped') is not True):
            return 'real_extension_uart_required'
        if extension.get('busy') is not False or extension.get('task') not in ('vision', 'dialogue', 'semantic'):
            return 'one_click_fire_requires_idle_extension'
    if (not fresh(selection, now, .5, 'capture_stamp') or selection.get('valid') is not True or
            type(selection.get('target_id')) is not int or selection['target_id'] < 1 or
            type(selection.get('selection_generation')) is not int or selection['selection_generation'] < 1 or
            not isinstance(selection.get('registry_session'), str) or not 1 <= len(selection['registry_session']) <= 160):
        return 'fresh_explicit_fire_selection_required'
    expected_keys = ['expected_' + key for key in IDENTITY_KEYS]
    if any(key in request for key in expected_keys):
        if (not all(key in request for key in expected_keys) or
                type(request.get('expected_target_id')) is not int or
                type(request.get('expected_selection_generation')) is not int or
                any(request.get('expected_' + key) != selection[key] for key in IDENTITY_KEYS)):
            return 'selected_fire_changed_before_submit'
    if not fresh(actuator, now, .5):
        return 'real_actuator_feedback_required'
    if actuator.get('dry_run') is not False or actuator.get('enabled') is not True:
        return 'real_actuator_not_enabled'
    if actuator.get('error') or actuator.get('estop') or actuator.get('fault'):
        return 'actuator_fault_or_estop'
    angles = actuator.get('actual_angles_deg')
    if (actuator.get('status_fresh') is not True or actuator.get('feedback_fresh') is not True or
            actuator.get('reference_confirmed_this_start') is not True or
            not isinstance(angles, (list, tuple)) or len(angles) != 2 or
            any(type(angle) not in (int, float) or not math.isfinite(angle) for angle in angles)):
        return 'region_real_gimbal_reference_required'
    if actuator.get('pump_enabled') is not True or actuator.get('pump_ready') is not True:
        return 'real_pump_not_enabled'
    if not off_disarmed(actuator, now):
        return 'pump_off_fresh_actuator_feedback_required'
    guard = actuator.get('guard') or {}
    if guard.get('state') in ('COOLDOWN', 'EXHAUSTED', 'FAULT') or guard.get('requires_disarm'):
        return 'pump_cooldown_or_fault'
    if mission.get('mounting_verified') is not True or mission.get('gimbal_transport') != 'x5':
        return 'fixed_camera_mounting_unverified'
    if not fresh(proposal, now, .4, 'capture_stamp') or proposal.get('valid') is not True:
        return str(proposal.get('reason') or 'one_click_fire_geometry_required')
    if any(proposal.get(key) != selection[key] for key in IDENTITY_KEYS):
        return 'selected_target_does_not_match_lock'
    # Geometry may be valid while the axes are still aligning: requiring an
    # already locked gimbal here would make the one-click action circular.
    if (proposal.get('aim_mode') != 'fixed_camera_3d' or proposal.get('calibration_ready') is not True):
        return 'fixed_camera_uncalibrated'
    policy = proposal.get('water_aim_policy', 'measured_impact')
    if policy != actuator.get('water_aim_policy', 'measured_impact'):
        return 'fixed_camera_landing_active_profile_mismatch'
    if policy == 'straight_ray':
        # Admission precedes movement, so do not require geometry_locked or
        # an already aligned ray. Preserve the same explicit unmeasured-policy
        # and active-profile identities used by fixed_camera_water_lock; that
        # final gate still checks the measured posture before any pulse.
        detail = proposal.get('aim') or {}
        if any(value.get('landing_compensated') is not False or
               value.get('water_impact_calibration_verified') is not False or
               value.get('physical_hit_verified') is not False or
               value.get('landing_calibration_id') is not None or
               value.get('geometric_alignment_only') is not True or
               value.get('ray_tracking_reference') != 'straight_target_ray' or
               value.get('water_aim_policy') != 'straight_ray'
               for value in (proposal, detail)):
            return 'fixed_camera_straight_ray_policy_mismatch'
        identities = {
            'geometry_calibration_id': actuator.get('geometry_calibration_id'),
            'camera_nozzle_mount_id': actuator.get('camera_nozzle_mount_id'),
            'pump_setting_id': actuator.get('pump_setting_id'),
            'actuator_reference_id': actuator.get('reference_id'),
        }
        if any(not isinstance(expected, str) or not expected.strip() or len(expected) > 128 or
               proposal.get(key) != expected or detail.get(key) != expected
               for key, expected in identities.items()):
            return 'fixed_camera_landing_active_profile_mismatch'
        stereo = proposal.get('stereo_calibration_id')
        if (not isinstance(stereo, str) or not stereo or
                detail.get('camera_calibration_id') != stereo or detail.get('stereo_calibration_id') != stereo):
            return 'fixed_camera_stereo_calibration_mismatch'
    elif policy == 'measured_impact':
        if (proposal.get('landing_compensated') is not True or
                proposal.get('water_impact_calibration_verified') is not True or
                not proposal.get('landing_calibration_id')):
            return 'fixed_camera_water_impact_unmeasured'
    else:
        return 'fixed_camera_water_aim_policy_invalid'
    return ''
