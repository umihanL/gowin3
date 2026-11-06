"""Fresh, geometry-aware fire proposals; the mission node owns hardware actions."""
import json
import math
import time
from collections import OrderedDict

import numpy as np
from .node_common import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from rescue_interfaces.msg import DetectionArray

from .node_common import IMAGE_QOS, param, read_json, seconds, age, publish_json, spin
from .legacy_aiming import FireAimTracker, AimConfig
from .legacy_visual_servo import ServoConfig
from .stereo import StereoDepth
from .aim_geometry import VariableRateServo, EncoderPoseHistory, measured_servo_update
from .nozzle_geometry import GeometryNozzleReference
from .fire_targets import SelectionHistory, selected_detection
from .fixed_camera_aim import FixedCameraAim
from .fixed_camera_landing import FixedCameraLanding
from .fire_aim_geometry import (corrected_camera, CORRECTED_FRAME, select_camera, load_landing, matched_detection, target_range,
                                landing_reference, mission_fresh, pump_candidate)


class FireAim(Node):
    def __init__(self):
        super().__init__('fire_aim')
        self.mission = {}
        self.mission_received = self.last_detection_received = None
        self.camera_info = None
        self.tracker = self.coordinate_mode = None
        file = param(self, 'stereo_calibration', '')
        self.stereo = StereoDepth(read_json(file) if file else {'calibrated': False})
        servo = param(self, 'servo_config', '')
        self.servo_cfg = ServoConfig(**read_json(servo)) if servo else ServoConfig()
        landing = param(self, 'landing_calibration', '')
        self.landing = load_landing(read_json(landing)) if landing else None
        nozzle = param(self, 'nozzle_calibration', '')
        self.nozzle = GeometryNozzleReference(read_json(nozzle) if nozzle else {'nozzle_calibrated': False})
        self.reference_signature = None
        self.planner = VariableRateServo(self.servo_cfg)
        self.encoder_history = EncoderPoseHistory()
        self.require_measured_feedback = bool(param(self, 'require_measured_feedback', True))
        self.max_age = float(param(self, 'max_frame_age_s', .5))
        self.mission_timeout = float(param(self, 'mission_timeout_s', .5))
        self.fixed_range = float(param(self, 'fixed_observation_range_m', 0.))
        self.fixed_verified = param(self, 'fixed_observation_verified', False)
        self.require_selection = param(self, 'require_selection', False)
        fixed_file = param(self, 'fixed_camera_aim_config', '')
        # Supplying this explicitly selects fixed-camera geometry. An unfilled
        # file stays blocked; it MUST NOT fall back to moving-camera centering.
        self.fixed_aim = FixedCameraAim(read_json(fixed_file)) if fixed_file else None
        fixed_landing_file = param(self, 'fixed_camera_landing_config', '')
        self.fixed_landing = FixedCameraLanding(read_json(fixed_landing_file)) if fixed_landing_file else None
        if self.fixed_landing is not None and self.fixed_landing.ready and self.fixed_aim is None:
            raise ValueError('Fixed-camera water calibration requires explicit fixed-camera geometry')
        if self.fixed_aim is not None:
            self.require_selection = True
        self.latest_actuator_state = None
        self.selection_history = SelectionHistory(max_age_s=self.max_age)
        self.pending_detections = OrderedDict()
        self.target_identity = None
        self.last_output_signature = None
        self.last_output_time = None
        if (not math.isfinite(self.max_age) or not 0 < self.max_age <= .5
                or not math.isfinite(self.mission_timeout) or not 0 < self.mission_timeout <= .5
                or not math.isfinite(self.fixed_range) or self.fixed_range < 0):
            raise ValueError('Invalid frame/mission timeout or fixed observation range')
        self.publisher = self.create_publisher(String, '/fire/proposal', 10)
        self.create_subscription(String, '/mission/state', self.mode, 10)
        self.create_subscription(String, '/actuators/state', self.actuator_state, 10)
        self.create_subscription(String, '/fire/selection', self.selection, 32)
        self.create_subscription(DetectionArray, param(self, 'detections_topic', '/perception/detections'), self.receive, IMAGE_QOS)
        info_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CameraInfo, '/vision/camera_info', self.info, info_qos)
        self.create_timer(.1, self.watchdog)

    def reset(self):
        self.tracker = self.coordinate_mode = None
        self.reference_signature = None
        self.target_identity = None
        self.planner = VariableRateServo(self.servo_cfg)
        if getattr(self, 'fixed_aim', None) is not None:
            self.fixed_aim.reset()

    def invalidate(self, reason, *, reset=True):
        fixed = getattr(self, 'fixed_aim', None)
        if reset and fixed is not None:
            fixed.reset()
        if reset and any(getattr(self, name, None) is not None for name in
                         ('tracker', 'coordinate_mode', 'reference_signature', 'target_identity')):
            self.reset()
        now = time.monotonic()
        # Use one primary gate across mode/selection/watchdog callbacks. Their
        # different secondary symptoms must not generate an idle-message storm.
        routine = reason in ('mission_not_fire_or_gimbal_not_ready', 'mission_stale_or_not_ready',
            'detection_timeout', 'selected_target_changed_or_cleared', 'explicit_fresh_selection_required',
            'selected_target_not_matched_to_this_capture', 'unavailable_or_stale_detection')
        if routine:
            if self.mission_received is None or not 0 <= now-self.mission_received <= self.mission_timeout:
                reason = 'mission_stale_or_unavailable'
            elif self.mission.get('mode') != 'fire':
                reason = 'mission_not_fire'
            elif self.mission.get('traffic_active', False) is not False:
                reason = 'traffic_active'
            elif self.mission.get('gimbal_ready') is not True:
                reason = 'gimbal_not_ready'
            elif (getattr(self, 'require_selection', False)
                  and self.selection_history.identity is None):
                reason = 'explicit_fresh_selection_required'
        extra = fixed.summary() if fixed is not None else dict(aim_mode='moving_camera_image_servo')
        if fixed is not None and not fixed.ready:
            reason = 'fixed_camera_uncalibrated'
        self.output(dict(extra, valid=False, image_locked=False, pump_candidate=False,
            pump_allowed=False, range_source='unavailable', landing_calibration_id=None,
            target_id=None, selection_generation=None, registry_session=None,
            target_range_m=None, physical_hit_verified=False, servo_reference_target_deg=None,
            servo_plan=dict(command=None, hardware_written=False, reason=reason), reason=reason))

    def output(self, document):
        now = time.monotonic()
        history = getattr(self, 'selection_history', None)
        signature = (document.get('valid'), document.get('reason'), self.mission.get('mode'),
            self.mission.get('traffic_active'), history.identity if history is not None else None)
        previous = getattr(self, 'last_output_signature', None)
        when = getattr(self, 'last_output_time', None)
        if document.get('valid') is not True and signature == previous and when is not None and 0 <= now-when < .2-1e-9:
            return
        self.last_output_signature, self.last_output_time = signature, now
        publish_json(self.publisher, document)

    def mode(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('Mission state must be an object')
        except (ValueError, TypeError):
            self.mission, self.mission_received = {}, None
            self.invalidate('invalid_mission_state')
            return
        self.mission, self.mission_received = value, time.monotonic()
        if not mission_fresh(self.mission, self.mission_received, self.mission_received, self.mission_timeout):
            self.invalidate('mission_not_fire_or_gimbal_not_ready')

    def info(self, msg):
        self.camera_info = msg
        if getattr(self, 'fixed_aim', None) is not None:
            # Fixed-camera XYZ is checked against the explicit source frame,
            # image size and stereo calibration ID. The preview CameraInfo
            # belongs to the separate 2D correction path, not this transform.
            return
        try:
            camera = corrected_camera(msg, CORRECTED_FRAME, (msg.width, msg.height))
            signature = (msg.header.frame_id, msg.width, msg.height, camera.doc.get('calibration_id'))
            if self.coordinate_mode is not None and self.coordinate_mode != signature:
                self.invalidate('camera_geometry_changed')
        except (ValueError, TypeError, AttributeError) as exc:
            self.invalidate('camera_geometry_rejected: ' + str(exc))

    def actuator_state(self, msg):
        try:
            state = json.loads(msg.data)
            if not isinstance(state, dict):
                raise ValueError('Invalid actuator state')
            self.latest_actuator_state = state
            self.encoder_history.observe(state, self.get_clock().now().nanoseconds/1e9)
        except (ValueError, TypeError):
            self.latest_actuator_state = None
            self.encoder_history.samples.clear()

    def watchdog(self):
        now = time.monotonic()
        if not mission_fresh(self.mission, self.mission_received, now, self.mission_timeout):
            self.invalidate('mission_stale_or_not_ready')
        elif self.last_detection_received is None or now - self.last_detection_received > self.max_age:
            self.invalidate('detection_timeout')

    def selection(self, msg):
        if not getattr(self, 'require_selection', False):
            return
        try:
            document = json.loads(msg.data)
        except (ValueError, TypeError):
            document = {}
        if self.selection_history.observe(document, time.monotonic()):
            self.invalidate('selected_target_changed_or_cleared')
        if self.selection_history.identity is None:
            self.invalidate('explicit_fresh_selection_required')
            return
        key = (document.get('capture_stamp_ns'), document.get('sequence'))
        pending = self.pending_detections.pop(key, None)
        if pending is not None:
            self.receive(pending)

    def receive(self, msg):
        if not mission_fresh(self.mission, self.mission_received, time.monotonic(), self.mission_timeout):
            self.invalidate('mission_stale_or_not_ready')
            return
        if not msg.detector_available or not 0 <= age(self, msg.header.stamp) <= self.max_age:
            self.invalidate('unavailable_or_stale_detection')
            return
        if getattr(self, 'fixed_aim', None) is not None:
            self.receive_fixed(msg)
            return
        try:
            calibration = select_camera(msg.header.frame_id, (msg.width, msg.height),
                                         msg.rectified, self.camera_info, self.stereo)
        except (ValueError, TypeError, AttributeError) as exc:
            self.invalidate('camera_geometry_rejected: ' + str(exc))
            return
        signature = (msg.header.frame_id, msg.width, msg.height, calibration.doc.get('calibration_id'))
        selection, explicit_target = None, None
        if getattr(self, 'require_selection', False):
            stamp_ns = int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)
            selection = self.selection_history.lookup(stamp_ns=stamp_ns, sequence=int(msg.sequence),
                frame_id=msg.header.frame_id, size=(msg.width, msg.height), now=time.monotonic())
            explicit_target = selected_detection(msg.detections, selection)
            if explicit_target is None:
                # Selection and depth may arrive in either ROS callback order.
                self.pending_detections[(stamp_ns, int(msg.sequence))] = msg
                while len(self.pending_detections) > 32:
                    self.pending_detections.popitem(last=False)
                self.invalidate('selected_target_not_matched_to_this_capture', reset=False)
                return
            identity = (selection['registry_session'], selection['selection_generation'], selection['target_id'])
            if getattr(self, 'target_identity', None) != identity:
                self.reset()
                self.target_identity = identity
        if self.tracker is None or self.coordinate_mode != signature:
            self.tracker = FireAimTracker(AimConfig(max_frame_age_s=self.max_age), calibration)
            self.coordinate_mode = signature
            self.planner = VariableRateServo(self.servo_cfg)
        finite_detections = [d for d in msg.detections if math.isfinite(d.confidence) and 0 <= d.confidence <= 1
                             and np.isfinite(d.bbox_xyxy).all()]
        if explicit_target is not None:
            finite_detections = [explicit_target]
        items = [dict(class_id=d.class_id, confidence=d.confidence, bbox_xyxy=list(d.bbox_xyxy))
                 for d in finite_detections]
        eligible = [d for d in finite_detections if d.class_id == 0 and d.confidence >= .4]
        selected = explicit_target if explicit_target is not None else matched_detection(eligible, self.tracker.box)
        distance, range_source = target_range(selected, fixed_range_m=self.fixed_range, fixed_verified=self.fixed_verified)
        reference, landing_id, compensated, landing_reason = None, None, False, 'no_valid_matching_landing_calibration'
        compensation_kind, geometry_compensation = 'image_alignment_only', None
        nozzle = getattr(self, 'nozzle', None)
        try:
            if (nozzle is not None and nozzle.ready and self.landing is not None
                    and self.landing.doc.get('camera_nozzle_mount_id') != nozzle.mount_id):
                raise ValueError('Measured water table and nozzle mount IDs differ')
            reference, landing_id = landing_reference(self.landing, calibration, msg.header.frame_id,
                                                      (msg.width, msg.height), distance)
            compensated, landing_reason = True, 'measured_in_range_landing_pixel'
            compensation_kind = 'measured_water_landing'
        except (ValueError, TypeError, KeyError) as exc:
            landing_reason = str(exc)
        geometry_reason = 'measured_water_landing_takes_priority' if compensated else 'no_measured_nozzle_geometry'
        if not compensated and nozzle is not None and nozzle.ready:
            try:
                xyz = ([selected.position_m.x, selected.position_m.y, selected.position_m.z]
                       if selected is not None and range_source == 'matched_stereo_euclidean' else None)
                geometry_compensation = nozzle.reference(calibration, (msg.width, msg.height), distance,
                    frame_id=msg.header.frame_id, target_xyz_m=xyz)
                reference = geometry_compensation['aim_pixel']
                compensation_kind = 'rigid_nozzle_geometry'
                geometry_reason = 'parallax_and_boresight_only_water_drop_unmeasured'
            except (ValueError, TypeError, KeyError) as exc:
                geometry_reason = str(exc)
        signature_reference = (compensation_kind, landing_id,
                               geometry_compensation['calibration_id'] if geometry_compensation else None,
                               range_source)
        if getattr(self, 'reference_signature', None) != signature_reference:
            # A previous centre-only lock cannot authorize a newly calibrated jet.
            self.tracker.stable_since = None
            self.tracker.hits.clear()
            self.tracker.points.clear()
            self.reference_signature = signature_reference
        now = self.get_clock().now().nanoseconds / 1e9
        aim = self.tracker.update(items, (msg.width, msg.height), msg.sequence,
                                  seconds(msg.header.stamp), now, reference)
        self.last_detection_received = time.monotonic()
        matched = matched_detection(eligible, aim.get('bbox_xyxy')) if aim.get('bbox_xyxy') else None
        if matched is not selected or matched is None:
            compensated, landing_id = False, None
            geometry_compensation = None
            compensation_kind = 'target_association_unverified'
            geometry_reason = 'reference_not_verified_for_current_tracked_target'
            aim['image_locked'] = False
            aim['angle_error_deg'] = None
            aim['pixel_error'] = None
            aim['state'] = 'ACQUIRING'
            self.tracker.stable_since = None
            distance, range_source = target_range(matched, fixed_range_m=self.fixed_range, fixed_verified=self.fixed_verified)
            landing_reason = 'landing_reference_not_verified_for_current_tracked_target'
        if matched is not None and range_source == 'matched_stereo_euclidean':
            aim['position_xyz_m'] = [matched.position_m.x, matched.position_m.y, matched.position_m.z]
            aim['position_frame'] = msg.header.frame_id
        aim.update(coordinate_space=msg.header.frame_id, coordinate_frame=msg.header.frame_id,
                   physical_eye='left', image_size_wh=[msg.width, msg.height],
                   camera_calibration_id=calibration.doc.get('calibration_id'), physical_hit_verified=False)
        target_metadata = dict(target_id=selection['target_id'] if selection else None,
            selection_generation=selection['selection_generation'] if selection else None,
            registry_session=selection['registry_session'] if selection else None)
        aim.update(target_metadata)
        accepted = self.mission.get('gimbal_target_deg')
        servo_reference = None
        if accepted is not None:
            try:
                command = np.asarray(accepted, dtype=float)
                if (command.shape != (2,) or not np.isfinite(command).all()
                        or not self.servo_cfg.pan_limits_deg[0] <= command[0] <= self.servo_cfg.pan_limits_deg[1]
                        or not self.servo_cfg.tilt_limits_deg[0] <= command[1] <= self.servo_cfg.tilt_limits_deg[1]):
                    raise ValueError('Invalid last-commanded gimbal target')
                self.planner.commanded = command
                servo_reference = command.tolist()
            except (ValueError, TypeError):
                self.invalidate('invalid_commanded_gimbal_target')
                return
        if getattr(self, 'require_measured_feedback', False) and self.servo_cfg.enabled:
            try:
                proposal = measured_servo_update(self.planner, aim, now, seconds(msg.header.stamp),
                                                self.encoder_history)
                if proposal.get('image_lock_consistent') is not True:
                    aim['image_locked'] = False
            except (ValueError, TypeError) as exc:
                self.invalidate('measured_servo_rejected: ' + str(exc))
                return
        else:
            proposal = self.planner.update(aim, now)
        fresh = mission_fresh(self.mission, self.mission_received, time.monotonic(), self.mission_timeout)
        fresh = fresh and 0 <= age(self, msg.header.stamp) <= self.max_age
        valid = bool(fresh and aim.get('state') not in ('STALE', 'INVALID_TIME'))
        locked = bool(valid and aim.get('image_locked'))
        aim['image_locked'] = locked
        if not valid:
            proposal['command'] = None
            proposal['reason'] = 'Detection or mission expired before proposal publication'
        candidate = pump_candidate(aim, valid=valid, compensated=compensated,
                                   range_m=distance, calibration_id=landing_id)
        self.output(dict(valid=valid, capture_stamp=seconds(msg.header.stamp), sequence=msg.sequence,
            aim_mode='moving_camera_image_servo', camera_mount='moves_with_gimbal', geometry_locked=False,
            **target_metadata,
            coordinate_frame=msg.header.frame_id, image_locked=locked, aim=aim, servo_plan=proposal,
            landing_compensated=compensated, landing_reason=landing_reason, target_range_m=distance,
            compensation_kind=compensation_kind, geometry_compensation=geometry_compensation,
            geometry_reason=geometry_reason,
            range_source=range_source, landing_calibration_id=landing_id, pump_candidate=candidate,
            pump_allowed=False, physical_hit_verified=False,
            servo_reference_target_deg=servo_reference,
            range_is_assumption=range_source == 'verified_fixed_observation_assumption'))

    def receive_fixed(self, msg):
        """Selected target, verified stereo XYZ and current motor encoders only."""
        fixed = self.fixed_aim
        if not fixed.ready:
            self.invalidate('fixed_camera_uncalibrated')
            return
        stamp_ns = int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)
        selection = self.selection_history.lookup(stamp_ns=stamp_ns, sequence=int(msg.sequence),
            frame_id=msg.header.frame_id, size=(msg.width, msg.height), now=time.monotonic())
        selected = selected_detection(msg.detections, selection)
        if selected is None:
            self.pending_detections[(stamp_ns, int(msg.sequence))] = msg
            while len(self.pending_detections) > 32:
                self.pending_detections.popitem(last=False)
            # Depth and selection can arrive in either callback order. Waiting
            # for this exposure must not reset the same target's time/lock state.
            # Real selection changes/clears still reset through selection().
            self.invalidate('fixed_camera_selection_required', reset=False)
            return
        metadata = {key:selection[key] for key in ('target_id', 'selection_generation', 'registry_session')}
        identity = (metadata['registry_session'], metadata['selection_generation'], metadata['target_id'])
        self.target_identity = identity
        now = self.get_clock().now().nanoseconds/1e9
        target = dict(class_id=selected.class_id, confidence=float(selected.confidence),
            depth_valid=selected.depth_valid, depth_reason=selected.depth_reason, depth_m=float(selected.depth_m),
            position_m=[selected.position_m.x, selected.position_m.y, selected.position_m.z])
        result = fixed.propose(target=target, capture_s=seconds(msg.header.stamp), sequence=int(msg.sequence),
            frame_id=msg.header.frame_id, image_size=(msg.width, msg.height), rectified=msg.rectified,
            stereo_calibration_id=getattr(msg, 'stereo_calibration_id', ''),
            active_stereo_calibration_id=getattr(self.stereo, 'calibration_id', None) if self.stereo.calibrated else None,
            identity=identity, encoder=getattr(self, 'latest_actuator_state', None), now_s=now,
            landing=getattr(self, 'fixed_landing', None))
        self.last_detection_received = time.monotonic()
        result.update(metadata, capture_stamp=seconds(msg.header.stamp), sequence=msg.sequence,
                      coordinate_frame=msg.header.frame_id, image_size_wh=[msg.width,msg.height],
                      landing_reason=('fixed_camera_measured_angular_grid' if result['landing_compensated'] else
                                      'fixed_camera_water_impact_unmeasured'),
                      compensation_kind=('fixed_camera_measured_angular_grid' if result['landing_compensated'] else
                                         'fixed_camera_nozzle_geometry'),
                      servo_reference_target_deg=(result.get('attainable_target_deg') if result['valid'] else None))
        if result['water_aim_policy'] == 'straight_ray':
            result.update(landing_reason='straight_ray_selected_no_water_impact_compensation',
                          compensation_kind='straight_nozzle_ray', geometric_alignment_only=True)
        result.setdefault('landing_calibration_id',None)
        if result['valid'] and (self.servo_cfg.enabled is not True or self.servo_cfg.mounting_verified is not True):
            fixed.reset()
            result.update(valid=False, geometry_locked=False, reason='fixed_camera_mounting_unverified')
            result['servo_plan'].update(command=None, reason=result['reason'])
        fresh = (mission_fresh(self.mission, self.mission_received, time.monotonic(), self.mission_timeout)
                 and 0 <= age(self, msg.header.stamp) <= self.max_age)
        if result['valid']:
            encoder_stamp = result['servo_plan'].get('encoder_stamp')
            encoder_age = self.get_clock().now().nanoseconds/1e9-encoder_stamp
            if not 0 <= encoder_age <= fixed.max_encoder_age_s:
                fixed.reset()
                result.update(valid=False, geometry_locked=False, reason='fixed_camera_encoder_feedback_required')
                result['servo_plan'].update(command=None, reason=result['reason'])
        if not fresh:
            fixed.reset()
            result.update(valid=False, geometry_locked=False, reason='fixed_camera_depth_stale')
            result['servo_plan'].update(command=None, reason=result['reason'])
        policy_ready = (result['water_aim_policy'] == 'straight_ray' and
                        result['landing_compensated'] is False and
                        result.get('water_impact_calibration_verified') is False and
                        result.get('ray_tracking_reference') == 'straight_target_ray')
        if result['water_aim_policy'] == 'measured_impact':
            policy_ready = (result['landing_compensated'] and
                            result.get('water_impact_calibration_verified') is True and
                            bool(result.get('landing_calibration_id')))
        result['pump_candidate'] = bool(result['valid'] and result['geometry_locked'] and policy_ready)
        result['aim'] = dict(state='GEOMETRY_LOCKED' if result['geometry_locked'] else 'GEOMETRY_ALIGNING',
            geometry_locked=result['geometry_locked'], image_locked=False, physical_hit_verified=False,
            coordinate_frame=msg.header.frame_id, image_size_wh=[msg.width,msg.height], physical_eye='left',
            position_xyz_m=result.get('target_position_camera_m'), position_frame=msg.header.frame_id,
            bbox_xyxy=list(selected.bbox_xyxy), camera_calibration_id=getattr(msg, 'stereo_calibration_id', ''),
            geometry_target_deg=result.get('geometry_target_deg'), **metadata)
        for key in ('actuator_reference_id','geometry_calibration_id','stereo_calibration_id',
                    'landing_calibration_id','pump_setting_id','camera_nozzle_mount_id',
                    'water_impact_calibration_verified','water_impact_angle_tolerance_deg','corrected_target_deg',
                    'water_aim_policy','landing_compensated','geometric_alignment_only',
                    'geometry_angle_tolerance_deg','geometry_line_tolerance_m','geometry_min_forward_ray_m',
                    'actual_ray_error_m','actual_forward_ray_m','ray_tracking_reference',
                    'attainable_target_deg','aim_target_deg'):
            result['aim'][key] = result.get(key)
        self.output(result)


def main():
    spin(FireAim)
