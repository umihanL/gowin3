"""Proportional visual servo command planner, with no PWM/serial side effects.

Stored angles are last commanded targets, NEVER measured joint positions.
Axis directions, motion limits and sensitivity must be calibrated on hardware.
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class ServoConfig:
    enabled: bool = False
    mounting_verified: bool = False
    pan_home_deg: float = 90.
    tilt_home_deg: float = 90.
    pan_limits_deg: tuple = (30.,150.)
    tilt_limits_deg: tuple = (40.,140.)
    pan_sign_for_right: int = 1
    tilt_sign_for_up: int = 1
    kp: float = .35
    max_speed_deg_s: float = 20.
    max_step_deg: float = 1.5
    # Optional empirically measured degrees per pixel when camera K is absent.
    pan_deg_per_px: float | None = None
    tilt_deg_per_px: float | None = None


class VisualServoPlanner:
    def __init__(self, config=None):
        self.cfg=config or ServoConfig();c=self.cfg
        for home,bounds in [(c.pan_home_deg,c.pan_limits_deg),(c.tilt_home_deg,c.tilt_limits_deg)]:
            if len(bounds)!=2 or not bounds[0]<=home<=bounds[1] or bounds[0]>=bounds[1]:raise ValueError('Invalid home/angle limits')
        if c.pan_sign_for_right not in (-1,1) or c.tilt_sign_for_up not in (-1,1) or not 0<c.kp<=1 or c.max_speed_deg_s<=0 or c.max_step_deg<=0:
            raise ValueError('Invalid servo control parameters')
        self.commanded=np.array([c.pan_home_deg,c.tilt_home_deg],dtype=float)
        self.last_time=None

    def update(self, aim, now_s):
        result=dict(command=None,last_commanded_angles_deg=self.commanded.tolist(),actual_angles_deg=None,reason='Disabled until mounting calibration',hardware_written=False)
        dt=0 if self.last_time is None else now_s-self.last_time
        self.last_time=now_s
        if not self.cfg.enabled or not self.cfg.mounting_verified:return result
        if not 0<dt<=.25:
            result['reason']='First update, out-of-order time, or long gap; hold';return result
        if aim['state'] not in ('ALIGNING','IMAGE_LOCKED') or aim['pixel_error'] is None:
            result['reason']='No confirmed fresh target; hold';return result
        if aim['image_locked']:
            result['reason']='Image locked; hold current command';return result
        angles=aim.get('angle_error_deg')
        if angles is not None:
            error=np.array([angles['yaw_right_positive'],angles['pitch_up_positive']],dtype=float)
        elif self.cfg.pan_deg_per_px and self.cfg.tilt_deg_per_px:
            error=np.array([aim['pixel_error']['x_right']*self.cfg.pan_deg_per_px,-aim['pixel_error']['y_down']*self.cfg.tilt_deg_per_px])
        else:
            result['reason']='Need camera intrinsics or measured pixel-to-angle response';return result
        if not np.isfinite(error).all():raise ValueError('Nonfinite aiming error')
        delta=error*self.cfg.kp*[self.cfg.pan_sign_for_right,self.cfg.tilt_sign_for_up]
        bound=min(self.cfg.max_step_deg,self.cfg.max_speed_deg_s*dt)
        delta=np.clip(delta,-bound,bound)
        self.commanded=np.clip(self.commanded+delta,[self.cfg.pan_limits_deg[0],self.cfg.tilt_limits_deg[0]],[self.cfg.pan_limits_deg[1],self.cfg.tilt_limits_deg[1]])
        result.update(command=dict(pan_target_deg=float(self.commanded[0]),tilt_target_deg=float(self.commanded[1])),last_commanded_angles_deg=self.commanded.tolist(),reason='Bounded proposed target angles; hardware adapter required')
        return result
