"""Image -> production tracking/geometry/control -> mixed protocol -> encoder plant.

The raster detector extracts a known coloured test marker. It exercises control
and latency, not YOLO accuracy, optics calibration or physical water impact.
"""
import math
import pathlib
import sys
import unittest
from unittest import mock
import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision.aim_geometry import EncoderPoseHistory, VariableRateServo, measured_servo_update
from rescue_vision.legacy_visual_servo import ServoConfig
from rescue_vision.legacy_aiming import AimConfig, CameraCalibration, FireAimTracker
from rescue_vision.pd42_protocol import Command
from rescue_vision import zdt_xs_protocol as zdt
from test_mixed_actuator_io import mixed_fixture


def state(now, angles, age=0.):
    return dict(stamp=now, feedback_age_s=age, dry_run=False, enabled=True,
                feedback_fresh=True, error='', estop=False, actual_angles_deg=list(angles))


def render_marker(error_deg, size=(1280, 1088)):
    image = np.full((size[1], size[0], 3), (35, 39, 45), dtype=np.uint8)
    yaw, pitch = np.radians(error_deg)
    x = math.tan(yaw)
    y = -math.tan(pitch)*math.sqrt(1+x*x)
    centre = (round(640+800*x), round(544+800*y))
    offsets = np.array([[-20,30],[-25,0],[-14,-12],[-8,0],[5,-30],[15,-12],[25,8],[18,30]])
    cv2.fillPoly(image, [offsets+centre], (0, 150, 255))
    mask = cv2.inRange(image, (0, 140, 250), (10, 160, 255))
    x, y, w, h = cv2.boundingRect(mask)
    detection = dict(class_id=0, confidence=.99, bbox_xyxy=[x, y, x+w-1, y+h-1])
    cv2.drawMarker(image, (640, 544), (230,230,230), cv2.MARKER_CROSS, 28, 1)
    return image, detection


def run_simulation(direction=(1, 1), latency_frames=2, dynamic=False):
    io, pan, tilt, events = mixed_fixture()
    cfg = ServoConfig(enabled=True, mounting_verified=True, max_speed_deg_s=6., max_step_deg=1.,
                      pan_sign_for_right=direction[0], tilt_sign_for_up=direction[1])
    planner, history = VariableRateServo(cfg), EncoderPoseHistory(.2)
    calibration = CameraCalibration(dict(camera_calibrated=True, calibration_id='synthetic-pinhole', eye='left',
        image_size_wh=[1280, 1088], camera_matrix=[[800.,0,640],[0,800.,544],[0,0,1]],
        distortion_coefficients=[0.]*5))
    tracker = FireAimTracker(AimConfig(max_frame_age_s=.25), calibration)
    actual = np.array([90., 90.]); commanded = actual.copy()
    trail, frames, records = [], {}, []
    dt = .05
    for frame in range(241):
        now = 10+frame*dt
        actual += np.clip(commanded-actual, -6*dt, 6*dt)
        pan.counts = round((actual[0]-90)*65536/360)
        tilt.counts = round((actual[1]-90)*51200/360)
        pan.rpm = 0 if abs(commanded[0]-actual[0]) < .04 else int(math.copysign(1, commanded[0]-actual[0]))
        tilt.rpm = 0 if abs(commanded[1]-actual[1]) < .04 else int(math.copysign(1, commanded[1]-actual[1]))
        pan.arrived = pan.rpm == 0
        tilt.status = 0 if tilt.rpm == 0 else 2
        with mock.patch('rescue_vision.pd42_actuator_io.time.monotonic', return_value=now):
            io._cycle()
            feedback = io.feedback()
            measured = feedback['actual_angles_deg']
            history.observe(state(now, measured), now)
            # Constant world target first, then a bounded smooth moving target.
            desired = np.array([98., 94.])
            if dynamic and frame > 100:
                desired += [1.2*math.sin((frame-100)*dt*.8), .8*math.sin((frame-100)*dt*.65)]
            trail.append((now, np.asarray(measured), desired))
            idx = max(0, frame-latency_frames)
            capture, exposure, old_target = trail[idx]
            error = (old_target-exposure)*np.asarray(direction)
            image, detection = render_marker(error)
            aim = tracker.update([detection], (1280,1088), frame, capture, now)
            plan = measured_servo_update(planner, aim, now, capture, history)
            if plan['command'] is not None:
                p = plan['command']
                io.set_gimbal(p['pan_target_deg'], p['tilt_target_deg'], speed_deg_s=6., lease_s=.3)
            elif io._goal is not None:
                # Keep the accepted hold target alive when image lock is stable.
                io.set_gimbal(*io._goal['target'], speed_deg_s=6., lease_s=.3)
            before = len(events)
            io._cycle()
            for name, cmd, packet in events[before:]:
                if name == 'tilt' and cmd == Command.ABSOLUTE_POSITION:
                    counts = int.from_bytes(packet[7:11], 'big', signed=True)
                    commanded[1] = 90+counts*360/51200
                elif name == 'pan' and cmd == zdt.Command.POSITION_EMM:
                    microsteps = int.from_bytes(packet[6:10], 'big')*(-1 if packet[2] else 1)
                    commanded[0] = 90+microsteps*1.8/16
            records.append(dict(t=now-10, target_deg=desired.tolist(), actual_angles_deg=list(measured),
                                commanded_deg=commanded.tolist(), error_deg=(desired-measured).tolist(),
                                image_locked=bool(aim['image_locked']), capture_age_s=now-capture,
                                marker_pixel=aim.get('target_pixel')))
            if frame in (0, 20, 50, 100, 160, 240):
                cv2.putText(image, f't={now-10:.2f}s  pan={measured[0]:.2f} tilt={measured[1]:.2f}',
                            (25,45), cv2.FONT_HERSHEY_SIMPLEX, .8, (240,240,240), 2)
                frames[frame] = image
    return records, frames, events


class MeasuredServoTests(unittest.TestCase):
    def test_no_integral_windup_if_motor_does_not_reach_command(self):
        p = VariableRateServo(ServoConfig(enabled=True, mounting_verified=True))
        h = EncoderPoseHistory()
        aim = dict(state='ALIGNING', image_locked=False, pixel_error=dict(x_right=100,y_down=50),
                   angle_error_deg=dict(yaw_right_positive=10.,pitch_up_positive=5.))
        targets=[]
        for i in range(100):
            now=10+i*.05
            h.observe(state(now,[90,90]),now)
            result=measured_servo_update(p,aim,now,now,h)
            if result['command']:targets.append(result['command']['pan_target_deg'])
        self.assertTrue(targets)
        self.assertLessEqual(max(targets),91.+1e-9)  # Floating roundoff only; never accumulated commands.

    def test_latency_compensation_uses_exposure_pose_and_current_encoder(self):
        p=VariableRateServo(ServoConfig(enabled=True,mounting_verified=True,max_step_deg=5,max_speed_deg_s=90))
        h=EncoderPoseHistory()
        h.observe(state(10.,[90,90]),10.)
        h.observe(state(10.1,[95,90]),10.1)
        p.last_time=10.
        aim=dict(state='ALIGNING',image_locked=False,pixel_error=dict(x_right=100,y_down=0),
                 angle_error_deg=dict(yaw_right_positive=10.,pitch_up_positive=0.))
        out=measured_servo_update(p,aim,10.1,10.,h)
        self.assertAlmostEqual(out['command']['pan_target_deg'],96.75)

    def test_stale_dry_run_or_future_feedback_cannot_move(self):
        h=EncoderPoseHistory()
        self.assertFalse(h.observe(dict(state(10,[90,90]),dry_run=True),10))
        with self.assertRaises(ValueError):h.pair(10,10)
        h.observe(state(10,[90,90]),10)
        with self.assertRaises(ValueError):h.pair(10,10.3)
        with self.assertRaises(ValueError):h.pair(9.99,10)

    def test_raster_to_mixed_motor_loop_converges_without_overshoot(self):
        for signs in ((1,1),(-1,1),(1,-1),(-1,-1)):
            with self.subTest(signs=signs):
                records, _, events=run_simulation(signs)
                final=np.array(records[-1]['error_deg'])
                self.assertLess(np.max(np.abs(final)),.45)
                actual=np.array([r['actual_angles_deg'] for r in records])
                self.assertLessEqual(np.max(actual[:,0]),98.4)
                self.assertLessEqual(np.max(actual[:,1]),94.4)
                self.assertTrue(any(r['image_locked'] for r in records[-30:]))
                self.assertTrue(any(e[0]=='pan' and e[1]==zdt.Command.POSITION_EMM for e in events))
                self.assertTrue(any(e[0]=='tilt' and e[1]==Command.ABSOLUTE_POSITION for e in events))

    def test_slow_moving_target_with_100ms_image_delay_is_tracked(self):
        records, _, _=run_simulation(dynamic=True)
        residual=np.array([r['error_deg'] for r in records[130:]])
        self.assertLess(np.sqrt(np.mean(residual**2)),.5)


if __name__=='__main__':unittest.main(verbosity=2)
