"""Physical-camera rays and bounded servo updates under variable inference rate."""
import math
import numpy as np
from collections import deque
from copy import deepcopy
from .legacy_aiming import CameraCalibration
from .legacy_visual_servo import VisualServoPlanner


class RectifiedCameraCalibration(CameraCalibration):
    def __init__(self,document,rectification):
        super().__init__(document);self.rotation=np.asarray(rectification,dtype=float)
    def angle_error(self,point,reference,size):
        if tuple(size)!=self.size:raise ValueError('Calibration dimensions differ')
        rays=np.linalg.inv(self.K)@np.array([[*point,1.],[*reference,1.]]).T
        rays=self.rotation.T@rays
        yaw=np.arctan2(rays[0],rays[2]);pitch=np.arctan2(-rays[1],np.hypot(rays[0],rays[2]))
        return dict(yaw_right_positive=float(np.degrees(yaw[0]-yaw[1])),pitch_up_positive=float(np.degrees(pitch[0]-pitch[1])))


class VariableRateServo(VisualServoPlanner):
    def __init__(self,config=None,max_update_gap_s=.8):
        super().__init__(config);self.max_update_gap_s=max_update_gap_s
    def update(self,aim,now_s):
        if self.last_time is not None:
            gap=now_s-self.last_time
            if .25<gap<=self.max_update_gap_s:
                # Cap integration time: slower inference must not cause a large step.
                self.last_time=now_s-.2
        return super().update(aim,now_s)


class EncoderPoseHistory:
    """Read-only history of physical encoder samples, in the ROS clock domain."""
    def __init__(self, max_age_s=.2):
        self.samples = deque(maxlen=96)
        self.max_age_s = max_age_s

    def observe(self, state, now_s):
        stamp, delay = state.get('stamp'), state.get('feedback_age_s')
        angles = state.get('actual_angles_deg')
        if (state.get('dry_run') is not False or state.get('enabled') is not True or
                state.get('feedback_fresh') is not True or state.get('error') or state.get('estop') or
                type(stamp) not in (int, float) or type(delay) not in (int, float) or
                not math.isfinite(stamp) or not math.isfinite(delay) or
                not 0 <= now_s-stamp <= self.max_age_s or not 0 <= delay <= self.max_age_s):
            self.samples.clear()
            return False
        if not isinstance(angles, (list, tuple)) or len(angles) != 2 or not all(
                type(a) in (int, float) and math.isfinite(a) for a in angles):
            self.samples.clear()
            return False
        captured = stamp-delay
        if self.samples and captured < self.samples[-1][0]:
            self.samples.clear()
            return False
        if not self.samples or captured > self.samples[-1][0]:
            self.samples.append((captured, np.asarray(angles, dtype=float)))
        return True

    def pair(self, capture_s, now_s):
        if not self.samples or not 0 <= now_s-self.samples[-1][0] <= self.max_age_s:
            raise ValueError('fresh_measured_gimbal_feedback_required')
        prior = [sample for sample in self.samples if sample[0] <= capture_s]
        if not prior or not 0 <= capture_s-prior[-1][0] <= self.max_age_s:
            raise ValueError('capture_time_encoder_pose_unavailable')
        return prior[-1], self.samples[-1]


def measured_servo_update(planner, aim, now_s, capture_s, history):
    """Compensate image latency without accumulating requested motor targets.

For the small calibrated pan/tilt corrections used here, the world target is
the exposure-time joint angle plus signed camera angular error. The new step
is proportional to its difference from the latest measured angle. This is
position feedback, not integration of prior unexecuted commands.
    """
    old, current = history.pair(capture_s, now_s)
    old_stamp, exposure = old
    stamp, measured = current
    adjusted = deepcopy(aim)
    c = planner.cfg
    if not (c.pan_limits_deg[0] <= measured[0] <= c.pan_limits_deg[1] and
            c.tilt_limits_deg[0] <= measured[1] <= c.tilt_limits_deg[1]):
        raise ValueError('measured_gimbal_angles_outside_limits')
    error = aim.get('angle_error_deg')
    if error is not None:
        camera_error = np.array([error['yaw_right_positive'], error['pitch_up_positive']], dtype=float)
        signs = np.array([c.pan_sign_for_right, c.tilt_sign_for_up], dtype=float)
        remaining = (exposure+signs*camera_error-measured)*signs
        if not np.isfinite(remaining).all():
            raise ValueError('invalid_measured_servo_error')
        adjusted['angle_error_deg'] = dict(yaw_right_positive=float(remaining[0]),
                                          pitch_up_positive=float(remaining[1]))
        # An older centred image cannot hold a joint that has since moved.
        if np.max(np.abs(measured-exposure)) > .15:
            adjusted['image_locked'] = False
    planner.commanded = measured.copy()
    result = planner.update(adjusted, now_s)
    result.update(actual_angles_deg=measured.tolist(), exposure_angles_deg=exposure.tolist(),
                  encoder_stamp=stamp, exposure_encoder_stamp=old_stamp,
                  image_lock_consistent=adjusted.get('image_locked') is True,
                  reference_source='measured_encoder_latency_compensated')
    return result
