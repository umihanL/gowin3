"""Explicit, bounded straight movements. No LLM, ROS or hardware side effects."""
import math


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def parameters(direction, distance_m=.20):
    if direction not in ('forward', 'reverse'):
        raise ValueError('manual_direction_invalid')
    if not finite(distance_m) or not .05 <= distance_m <= .50:
        raise ValueError('manual_distance_requires_5_to_50_cm')
    return dict(direction=direction, distance_m=float(distance_m), speed_m_s=.10,
                max_duration_s=round(distance_m/.10+2., 3))


def validate(request):
    expected = parameters(request.get('direction'), request.get('distance_m'))
    for key in ('speed_m_s', 'max_duration_s'):
        if not finite(request.get(key)) or abs(request[key]-expected[key]) > 1e-8:
            raise ValueError('manual_limits_mismatch')
    return expected


def fresh(value, now, ttl=.5):
    stamp = value.get('stamp') if isinstance(value, dict) else None
    return finite(stamp) and finite(now) and 0 <= now-stamp <= ttl


def navigation_busy(navigation):
    """A paused goal is still owned; explicitly cancel it before changing mode."""
    if not navigation or navigation.get('mode') != 'execute' or not navigation.get('goal_id'):
        return False
    status = navigation.get('fpga_status') or {}
    return not (status.get('state') in ('arrived', 'cancelled', 'failed')
                and status.get('vehicle_stopped') is True and not navigation.get('pending_action'))


def start_guard(now, *, mission, actuator, telemetry, link, navigation):
    if not fresh(link, now) or link.get('dry_run') is not False or not link.get('status_fresh') or link.get('error'):
        return 'real_fpga_feedback_required'
    if link.get('manual_motion_enabled') is not True:
        return 'manual_motion_not_enabled'
    if (not fresh(mission, now) or mission.get('vehicle_stopped') is not True
            or mission.get('motion_allowed') is not False):
        return 'manual_requires_confirmed_stop'
    if mission.get('traffic_active') or mission.get('emergency_stop') or mission.get('auto_fire_active'):
        return 'manual_blocked_by_traffic_estop_or_auto'
    if not fresh(telemetry, now, .3) or telemetry.get('vehicle_stopped') is not True or telemetry.get('estop') is not False:
        return 'manual_requires_fresh_stopped_telemetry'
    if not fresh(navigation, now) or navigation_busy(navigation):
        return 'cancel_navigation_and_wait_stopped_before_manual'
    return runtime_guard(now, mission=mission, actuator=actuator, telemetry=telemetry)


def runtime_guard(now, *, mission, actuator, telemetry):
    if not fresh(mission, now) or mission.get('emergency_stop') or mission.get('auto_fire_active') or mission.get('traffic_active'):
        return 'manual_mission_unavailable_or_unsafe'
    if not fresh(telemetry, now, .3) or telemetry.get('estop') is not False:
        return 'manual_telemetry_unavailable_or_estop'
    if not fresh(actuator, now) or actuator.get('estop') or actuator.get('error'):
        return 'manual_actuator_status_unavailable'
    guard = actuator.get('guard') or {}
    if actuator.get('pump_on') is not False or guard.get('armed') is not False:
        return 'stop_and_disarm_pump_before_manual'
    return ''
