"""Yield optional observations only while a fresh stationary dialogue is running.

This gate grants no control authority. Unknown, stale or active control state
always restores observation work; neither images nor evidence are restamped.
"""
import math


def _fresh(value, now, limit):
    return (type(value) in (int, float) and math.isfinite(value)
            and math.isfinite(now) and 0 <= now-value <= limit)


def yield_optional_observation(mission, mission_received, dialogue,
                               dialogue_received, now_ros, now_mono):
    """Only non-actuating inspect mode can borrow CPU for the local LLM.

    Dialogue status is a 1 Hz heartbeat. A missing publisher restores work in
    at most two seconds. Fire aiming, armed spray, automatic fire searches,
    traffic and all driving modes never take this optional-work path.
    """
    return bool(isinstance(mission, dict) and isinstance(dialogue, dict)
        and _fresh(mission.get('stamp'), now_ros, .5)
        and _fresh(mission_received, now_mono, .5)
        and mission.get('mode') == 'inspect'
        and mission.get('motion_allowed') is False
        and mission.get('traffic_active') is False
        and mission.get('auto_fire_active') is False
        and mission.get('pump_armed') is False
        and _fresh(dialogue.get('stamp'), now_ros, 2.)
        and _fresh(dialogue_received, now_mono, 2.)
        and dialogue.get('busy') is True
        and dialogue.get('state') == 'thinking')
