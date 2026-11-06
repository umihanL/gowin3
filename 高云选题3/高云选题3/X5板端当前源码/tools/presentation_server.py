#!/usr/bin/env python3
"""Live robot console: bounded ROS observation and same-origin task requests.

Only the ROS executor publishes parsed requests. The existing router and
actuator gates remain the owners; disconnected sources never get fabricated data.
The JPEG packet is the existing 8086 packet, passed through without re-encoding.
"""
import argparse
from collections import OrderedDict, deque
import copy
import http.client
import ipaddress
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import queue
import re
import socket
import struct
import subprocess
import sys
import threading
import time
from urllib.parse import parse_qs, quote, urlsplit
import uuid

def resource_root():
    """Use PyInstaller's asset directory independently of executable/CWD location."""
    if getattr(sys, 'frozen', False):
        bundle = getattr(sys, '_MEIPASS', None)
        if not bundle:
            raise RuntimeError('Frozen presentation bundle has no resource directory')
        return Path(bundle).resolve()
    return Path(__file__).resolve().parents[1]


WORKSPACE = resource_root()
if not getattr(sys, 'frozen', False):
    sys.path.insert(0, str(WORKSPACE / 'src' / 'rescue_vision'))
from rescue_vision.task_interaction import parse_task_text, point_request_guard, validate_point_request
from rescue_vision.extension_control import start_guard as extension_start_guard
from rescue_vision.extension_control import zebra_switch_guard
from rescue_vision.road_visualization import RoadVisualizationCache
from rescue_vision.fpga_map import load_catalog as load_map_catalog, live_map_snapshot
from rescue_vision.region_search import load_catalog as load_region_catalog
from rescue_vision.selected_fire_round import selected_fire_round_reason
from rescue_vision.pd42_test import ACTIONS as MOTOR_TEST_ACTIONS, bench_guard as motor_bench_guard

# key: (actual topic name, receiving/source freshness limit in seconds)
TOPICS = {
    'mission': ('/mission/state', 1.),
    'extension': ('/extension/state', 1.5),
    'traffic_observation': ('/traffic/observation', 1.), 'traffic': ('/traffic/state', 1.),
    'traffic_preview': ('/traffic/preview', 1.5),
    'navigation': ('/navigation/state', 1.5), 'fpga_link': ('/fpga/link', 1.5),
    'fpga_tx': ('/fpga/tx', 5.), 'motion_permission': ('/control/motion_permission', .5),
    'gimbal_command': ('/control/gimbal', .5),
    'fpga_telemetry': ('/fpga/telemetry', 1.), 'fpga_navigation': ('/fpga/navigation_status', 1.),
    'actuators': ('/actuators/state', 1.), 'fire_targets': ('/fire/targets', 1.),
    'fire_selection': ('/fire/selection', 1.), 'fire_selection_ack': ('/fire/selection_ack', 10.),
    'fire_proposal': ('/fire/proposal', 1.), 'fire_auto': ('/fire/auto/state', 1.),
    'stereo': ('/stereo/depth_status', 3.),
    'stereo_observation': ('/stereo/depth_observation', 3.),
    'scene': ('/scene/report', 2.),
    'ocr': ('/scene/ocr', 3.), 'scene_answer': ('/scene/answer', 90.),
    'dialogue_status': ('/scene/dialogue_status', 3.),
    'voice': ('/voice/status', 12.), 'transcript': ('/voice/transcript', 30.),
    'audio_test': ('/audio/test/status', 4.),
    'motor_test': ('/motor_test/state', 1.),
    'gimbal_self_test': ('/gimbal/self_test/state', 1.),
    'gimbal_manual': ('/gimbal/manual/state', 1.),
    'pump_manual': ('/pump/manual/state', 1.),
    'fpga_map': ('/fpga/map_state', 1.5),
    'speech': ('/speech/say', 30.), 'interaction': ('/interaction/result', 30.),
    'perception_diagnostics': ('/diagnostics/perception', 5.),
    'camera_diagnostics': ('/diagnostics/camera', 5.),
    'road_diagnostics': ('/diagnostics/road', 5.),
    'ocr_diagnostics': ('/diagnostics/building_ocr', 5.),
    'detections': ('/perception/detections', 2.),
}
EVENT_KEYS = {'transcript', 'speech', 'scene_answer', 'interaction', 'fire_selection_ack', 'voice'}
STEREO_LAST_VALID_KEY = 'stereo_last_valid_observation'  # Display cache, not a ROS subscription/control input.
VOICE_EVENT_FIELDS = {'state','recognition_phase','reason','error','command_id','session_id',
    'request_id','text','recognized_text','microphone_enabled','listening','capture_active',
    'listening_mode','capture_mode','audio_test_active','continuous_paused','tts_enabled','last_recognition',
    'tts_state','tts_error','tts_audio_device','tts_device_ready','tts_last_result',
    'stop_requested','stop_requested_at','capture_stop_confirmed',
    'stop_request_seq','stop_request_accepted','stop_request_reason','voice_enabled','voice_control'}
MAX_TOPIC_BYTES = 65536
MAX_FRAME_BYTES = 8 * 1024 * 1024
MAX_BODY_BYTES = 2048
ID_RE = re.compile(r'[A-Za-z0-9_:.-]{1,100}\Z')
MOTOR_TEST_IDS = ('motor1', 'motor2')


class ApiError(Exception):
    def __init__(self, code, reason):
        self.code, self.reason = code, reason
        super().__init__(reason)


def compact(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode('utf-8')


def source_stamp(data):
    for key in ('capture_stamp', 'capture_ros_s', 'stamp'):
        value = data.get(key)
        if isinstance(value, dict) and 'sec' in value and 'nanosec' in value:
            try:
                value = int(value['sec']) + int(value['nanosec']) / 1e9
            except (ValueError, TypeError):
                continue
        if type(value) in (int, float) and math.isfinite(value):
            return value
    return None


class PresentationState:
    def __init__(self, buildings=None, monotonic=time.monotonic, wall=time.time):
        self.buildings = buildings or {'buildings': []}
        self.monotonic, self.wall = monotonic, wall
        self.started = monotonic()
        self.lock = threading.RLock()
        self.latest = {}
        self.latest_json = {}
        self.last_valid_stereo_observation = None
        self.last_valid_stereo_json = None
        self.events = deque(maxlen=100)
        self.commands = OrderedDict()  # bounded lifetime dedup ledger, no silent eviction/retry
        self.execution_results = OrderedDict()  # also retains outcomes for car microphone commands
        self.pending = deque(maxlen=15)  # eight tasks plus seven independently reserved owner stops
        self.motor_heartbeats = {}  # coalesced, at most one renewal per motor; no command-log spam
        self.event_id = 0
        self.ros = {'available': False, 'domain_id': None, 'router_ready': False,
                    'listen_ready': False, 'audio_test_ready': False, 'voice_control_ready': False,
                    'motor_test_ready': False, 'gimbal_self_test_ready': False, 'gimbal_manual_ready': False,
                    'pump_manual_ready': False, 'error': 'not_connected'}
        self.preview = {'enabled': False, 'upstream_available': False,
                        'note': 'not_checked', 'updated_at': None}
        self.road_visualization = RoadVisualizationCache(monotonic=monotonic, wall=wall)
        self.map_catalog = load_map_catalog(WORKSPACE/'config/fpga_map_catalog.json')
        self.semantic_regions = load_region_catalog(WORKSPACE/'config/semantic_regions.json')
        traversal = json.loads((WORKSPACE/'config/region_traversability.json').read_text(encoding='utf-8'))
        self.start_heading_samples = traversal['start_heading_samples']
        self.start_heading_samples_sha256 = traversal['start_heading_samples_sha256']

    def event(self, kind, data, topic=''):
        self.event_id += 1
        self.events.append({'id': self.event_id, 'at': self.wall(), 'kind': kind,
                            'topic': topic, 'data': copy.deepcopy(data), 'synthetic': False})

    def update(self, key, data, *, take_ownership=False):
        if key not in TOPICS or not isinstance(data, dict):
            return False
        if data.get('synthetic') is True:
            return False
        try:
            encoded = compact(data)
            if len(encoded) > MAX_TOPIC_BYTES:
                return False
        except (ValueError, TypeError, OverflowError):
            return False
        with self.lock:
            old = self.latest.get(key)
            # ROS callbacks hand over newly decoded/built objects and never
            # touch them again. All other callers retain copy isolation.
            # Stored topic trees are replaced, never edited in place; the
            # matching validated bytes can therefore be reused by HTTP.
            self.latest[key] = (data if take_ownership else copy.deepcopy(data), self.monotonic(), self.wall())
            self.latest_json[key] = encoded
            if key == 'stereo_observation':
                self.remember_stereo_observation(self.latest[key])
            if key in EVENT_KEYS:
                # Voice heartbeats remain latest state; timeline retains actual transitions.
                old_sig = None if not old else {k: v for k, v in old[0].items() if k in VOICE_EVENT_FIELDS}
                sig = {k: v for k, v in data.items() if k in VOICE_EVENT_FIELDS}
                if key != 'voice' or sig != old_sig:
                    self.event(key, data, TOPICS[key][0])
            if key in ('interaction', 'scene_answer'):
                record = next((v for v in self.commands.values()
                               if v.get('command_id') == data.get('command_id')), None)
                if record:
                    record['receipt' if key == 'interaction' else 'answer'] = copy.deepcopy(data)
                    if key == 'interaction':
                        record['status'] = ('observed' if record.get('pump_manual_status') else
                                            'execution' if record.get('execution') else 'receipt')
                        record.setdefault('receipt_received_at', self.wall())
            elif key == 'extension':
                self.update_extension_feedback(data)
            elif key == 'audio_test':
                self.update_audio_test_feedback(data)
            elif key == 'motor_test':
                self.update_motor_test_feedback(data)
            elif key == 'gimbal_self_test':
                self.update_gimbal_self_test_feedback(data)
            elif key == 'gimbal_manual':
                self.update_gimbal_self_test_feedback(data, 'gimbal_manual')
            elif key == 'pump_manual':
                self.update_pump_manual_feedback(data)
            elif key == 'voice':
                self.update_voice_control_feedback(data)
        return True

    def update_voice_control_feedback(self, data):
        """Only a matching, current voice-node receipt confirms the switch."""
        feedback = data.get('voice_control')
        if not isinstance(feedback, dict):
            return
        identifier = feedback.get('request_id')
        if not isinstance(identifier, str):
            return
        record = self.commands.get(identifier)
        if (not record or record.get('kind') != 'voice_control'
                or 'dispatch_started_at' not in record
                or type(feedback.get('enabled')) is not bool
                or feedback['enabled'] is not record.get('voice_control')
                or feedback.get('status') not in ('pending', 'applied', 'rejected')):
            return
        stamp = source_stamp(data)
        if (not self.envelope('voice', copy_data=False)['fresh'] or stamp is None
                or stamp < record['dispatch_started_at'] - .05):
            return
        status = feedback['status']
        if record['status'] in ('applied', 'rejected'):
            return
        if status == 'applied' and data.get('voice_enabled') is not feedback['enabled']:
            return
        record['voice_control_status'] = copy.deepcopy(feedback)
        record['status'] = status
        record['reason'] = feedback.get('reason') or ('voice_control_' + status)
        record['feedback_received_at'] = self.wall()

    def remember_stereo_observation(self, value):
        """Keep one useful display sample with its ORIGINAL source/receipt times."""
        data, received, wall = value
        previous = self.last_valid_stereo_observation
        geometry = ('calibration_id', 'coordinate_frame', 'image_size_wh')
        if (data.get('observation_only') is not True or data.get('control_eligible') is not False
                or (previous is not None and any(data.get(k) != previous[0].get(k) for k in geometry))):
            self.last_valid_stereo_observation = None
            self.last_valid_stereo_json = None
            previous = None
        if data.get('observation_only') is not True or data.get('control_eligible') is not False:
            return
        stamp = source_stamp(data)
        if stamp is None or not -.05 <= wall - stamp <= 3.:
            return
        def valid_estimate(item):
            distance = item.get('range_m') if isinstance(item, dict) else None
            return (isinstance(item, dict) and item.get('estimate_valid') is True
                    and type(distance) in (int, float) and math.isfinite(distance) and distance > 0)
        probe = data.get('scene_probe')
        targets = data.get('targets')
        useful = (isinstance(probe, dict) and probe.get('kind') == 'calibration_probe' and valid_estimate(probe))
        useful = useful or (isinstance(targets, list) and any(
            isinstance(item, dict) and item.get('class_id') == 0 and valid_estimate(item) for item in targets))
        previous_stamp = source_stamp(previous[0]) if previous is not None else None
        if useful and (previous_stamp is None or stamp > previous_stamp):
            # A duplicate/replayed frame must not renew the display lifetime.
            self.last_valid_stereo_observation = value
            self.last_valid_stereo_json = self.latest_json.get('stereo_observation')

    def update_extension_feedback(self, data):
        """Keep exact-command executor outcomes separate from router acceptance.

        last_result is retained in fresh heartbeats; its original timestamp may
        legitimately be old. Never manufacture a new request or retry here.
        """
        if not self.envelope('extension', copy_data=False)['fresh']:
            return
        value = data.get('last_result')
        if not isinstance(value, dict) or value.get('synthetic') is True:
            return
        command_id, stamp = value.get('command_id'), source_stamp({'stamp': value.get('stamp')})
        if (not isinstance(command_id, str) or not ID_RE.fullmatch(command_id)
                or type(value.get('accepted')) is not bool or stamp is None
                or stamp > self.wall() + .05):
            return
        record = next((v for v in self.commands.values()
                       if v.get('command_id') == command_id and v.get('kind') == 'command'), None)
        if record and stamp < record['queued_at'] - .05:
            return
        result = {key: copy.deepcopy(value[key]) for key in
                  ('command_id', 'accepted', 'completed', 'reason', 'stamp') if key in value}
        prior = self.execution_results.get(command_id) or (record or {}).get('execution')
        if prior:
            if stamp < prior['stamp'] or result == prior:
                return
            if prior.get('accepted') is False or prior.get('completed') is True:
                return  # an old pending result cannot revive a terminal command
        self.execution_results[command_id] = result
        self.execution_results.move_to_end(command_id)
        while len(self.execution_results) > 100:
            self.execution_results.popitem(last=False)
        if record:
            record.update(status='execution', execution=copy.deepcopy(result))
            record.pop('reason', None)  # clear a superseded no-receipt timeout
        self.event('extension_result', result, TOPICS['extension'][0])

    def update_gimbal_self_test_feedback(self, data, kind='gimbal_self_test'):
        if not self.envelope(kind, copy_data=False)['fresh']:
            return
        records={v['command_id']:v for v in self.commands.values() if v['kind']==kind}
        stamp = data.get('last_request_stamp', data.get('stamp'))
        rid = data.get('last_request_id') or data.get('request_id')
        record=records.get(rid)
        if (record and type(stamp) in (int,float) and 0<=self.wall()-stamp<=1.
                and stamp>=record['queued_at']-.05 and record['status'] not in ('expired','rejected')):
            result=data.get('last_result')
            if result=='rejected':
                record.update(status='rejected',reason=data.get('last_request_reason') or data.get('reason'))
            elif result in ('accepted','completed','error','cancelled'):
                record.update(status='observed', reason=data.get('last_request_reason') or data.get('reason'))
                record[kind+'_status']=copy.deepcopy(data)
        # Session progress is separate from the latest request receipt. A later
        # rejected Start must not fail the original run; a 5s timeout must still
        # update the original run even though its accepted receipt is now old.
        run=records.get(data.get('request_id'))
        current=data.get('stamp')
        if (run and type(current) in (int,float) and 0<=self.wall()-current<=1.
                and current>=run['queued_at']-.05 and run['status'] not in ('expired','rejected')
                and data.get('phase') in ('running','completed','cancelled','failed')):
            run.update(status='observed',reason=data.get('reason'))
            run[kind+'_status']=copy.deepcopy(data)

    def update_pump_manual_feedback(self, data):
        """Keep node receipts and actual run progress correlated; never retry a pulse."""
        if (not self.envelope('pump_manual', copy_data=False)['fresh']
                or source_stamp({'stamp': data.get('stamp')}) is None):
            return
        records = {v['command_id']: v for v in self.commands.values()
                   if v['kind'] == 'pump_manual' or v.get('action') == 'manual_pump_test'}
        terminal = ('completed', 'cancelled', 'failed', 'rejected')

        def observe(record, stamp, result, reason):
            if (record is None or stamp is None or stamp > self.wall() + .05
                    or stamp < record['queued_at'] - .05 or record['status'] in ('expired', 'rejected')):
                return
            prior = record.get('pump_manual_result')
            if prior in terminal or prior == 'accepted' and result == 'rejected':
                return
            record.update(status='rejected' if result == 'rejected' else 'observed',
                          reason=reason, pump_manual_result=result,
                          pump_manual_status=copy.deepcopy(data))

        request = records.get(data.get('last_request_id'))
        result = data.get('last_result')
        if (request and result in ('accepted', *terminal)
                and data.get('last_request_action') == ('start' if request.get('action') == 'manual_pump_test' else request.get('action'))):
            # request_stamp retains the original issued_at even when a later
            # fresh heartbeat confirms LOW. It need only identify this request;
            # envelope freshness above belongs to the actual node observation.
            observe(request, source_stamp({'stamp': data.get('last_request_stamp', data.get('request_stamp'))}),
                    result, data.get('last_request_reason'))
        # A later rejected Start or a Stop receipt must not overwrite the active
        # Start's execution result. Current run identity survives either receipt.
        run = records.get(data.get('request_id'))
        phase = data.get('phase')
        running = ('waiting_on', 'spraying', 'waiting_off')
        if run and run.get('action') in ('start', 'manual_pump_test') and phase in (*running, *terminal):
            observe(run, source_stamp({'stamp': data.get('stamp')}),
                    'accepted' if phase in running else phase, data.get('reason'))

    def update_motor_test_feedback(self, data):
        """Only a fresh, correlated motor-node result changes execution progress."""
        if not self.envelope('motor_test', copy_data=False)['fresh']:
            return
        records = {value['command_id']: value for value in self.commands.values()
                   if value['kind'] == 'motor_test'}
        if 'recent_requests' in data:
            # Callback rejection can occur while a UART worker still owns the
            # latest state. The bounded node ledger preserves both requests.
            items = data['recent_requests']
            if not isinstance(items, list):
                return
            feedback = [dict(stamp=item.get('stamp'), last_request_id=item.get('request_id'),
                last_action=item.get('action'), last_result=item.get('result'),
                motor_id=item.get('motor_id', 'motor1'),
                reason=item.get('reason'), error=item.get('error'))
                for item in items[-32:] if isinstance(item, dict)]
        else:
            feedback = [dict(data, motor_id=data.get('last_motor_id', data.get('motor_id', 'motor1')))]
        for item in feedback:
            stamp = source_stamp({'stamp': item.get('stamp')})
            if stamp is None or not -.05 <= self.wall() - stamp <= TOPICS['motor_test'][1]:
                continue  # a fresh heartbeat cannot refresh an old result event
            record = records.get(item.get('last_request_id'))
            if (record is None or record.get('motor_test') != item.get('last_action')
                    or record.get('motor_id', 'motor1') != item.get('motor_id')
                    or record['status'] in ('expired', 'rejected')
                    or stamp < record['queued_at'] - .05):
                continue
            result = item.get('last_result')
            if result not in ('accepted', 'completed', 'rejected', 'error'):
                continue
            prior = record.get('motor_test_status', {})
            if prior.get('last_result') in ('completed', 'rejected', 'error'):
                continue  # late duplicate feedback cannot overwrite the terminal result
            record['motor_test_status'] = copy.deepcopy(item)
            record['status'] = 'rejected' if result in ('rejected', 'error') else 'observed'
            record['reason'] = item.get('error') or item.get('reason') or 'motor_test_' + result
            if prior.get('last_result') != result:
                self.event('motor_test', item, TOPICS['motor_test'][0])

    def update_audio_test_feedback(self, data):
        """Correlate session progress separately from each start/stop request.

        A rejected second request may carry the still-running session's image
        of its status. Its command_id remains that session's original start ID;
        only request_id names the rejected request. Never fail the active start
        just because another operator clicked Start while its microphone is busy.
        """
        request_id = data.get('request_id')
        session_id = data.get('session_id') or data.get('command_id')
        records = [v for v in self.commands.values() if v['kind'] == 'audio_test']
        rejected = data.get('request_rejected') is True or data.get('request_accepted') is False
        if rejected:
            record = next((v for v in records if v['command_id'] == request_id), None)
            if record is None:
                return
            if record.get('audio_test_status') and record['status'] == 'observed':
                # Duplicate/late rejection cannot erase previously confirmed
                # acceptance or a completed diagnostic result for the same ID.
                record['audio_test_rejection'] = copy.deepcopy(data)
            else:
                record.update(status='rejected', reason=data.get('request_reason') or
                              data.get('reason') or 'audio_test_request_rejected',
                              audio_test_status=copy.deepcopy(data))
            return
        if not isinstance(session_id, str):
            return
        for record in records:
            if record['status'] in ('rejected', 'expired'):
                continue
            direct_start = record.get('audio_test') is True and record['command_id'] == session_id
            direct_request = record['command_id'] == request_id
            known_session = record.get('audio_test_session_id') == session_id
            if direct_start or direct_request or known_session:
                record['audio_test_session_id'] = session_id
                record['audio_test_status'] = copy.deepcopy(data)
                record['status'] = 'observed'  # actual node feedback, never an HTTP-generated result

    def envelope(self, key, *, copy_data=True):
        # Freshness-only checks and snapshot_bytes' locked encoder can borrow
        # immutable topic data. Ordinary snapshots/callers keep independent copies.
        cached = key == STEREO_LAST_VALID_KEY
        topic, ttl = TOPICS['stereo_observation'] if cached else TOPICS[key]
        value = self.last_valid_stereo_observation if cached else self.latest.get(key)
        if value is None:
            return dict(topic=topic, data=None, received_at=None, age_s=None, source_age_s=None,
                        fresh=False, stale_after_s=ttl, synthetic=False, display_cache=cached)
        data, received, wall = value
        age = self.monotonic() - received
        stamp = source_stamp({'stamp': data.get('stamp')}) if key == 'audio_test' else source_stamp(data)
        source_age = self.wall() - stamp if stamp is not None else None
        fresh = 0 <= age <= ttl and (source_age is None or -.05 <= source_age <= ttl)
        return dict(topic=topic, data=copy.deepcopy(data) if copy_data else data, received_at=wall,
                    age_s=round(age, 3), source_age_s=None if source_age is None else round(source_age, 3),
                    fresh=fresh, stale_after_s=ttl, synthetic=False, display_cache=cached)

    def ready(self, kind='command', stop=False, reference=False, recovery=False, fixed_nozzle=False):
        if not self.ros['available']:
            return False, 'ros_unavailable'
        if kind == 'voice_control':
            # Re-enabling must work with no microphone and no FPGA task/page.
            return ((True, '') if self.ros.get('voice_control_ready') else
                    (False, 'voice_control_subscriber_unavailable'))
        if kind in ('gimbal_self_test','gimbal_manual','pump_manual'):
            if not self.ros.get(kind+'_ready'):
                return False, kind+'_subscriber_unavailable'
            if stop:return True, ''
            # Recovery establishes fresh feedback; requiring that feedback here
            # would make a disconnected actuator impossible to reconnect.
            if kind == 'gimbal_manual' and recovery:return True, ''
            if kind == 'gimbal_manual' and reference:
                return self.gimbal_reference_ready()
            value=self.envelope(kind)
            if not value['fresh']:return False, kind+'_status_stale'
            data=value['data'] or {}
            if kind == 'pump_manual' and source_stamp({'stamp': data.get('stamp')}) is None:
                return False, 'pump_manual_status_stale'
            if kind == 'pump_manual' and fixed_nozzle is True and data.get('can_start') is not True:
                return ((True,'') if data.get('fixed_nozzle_readiness') is True else
                    (False,data.get('fixed_nozzle_ready_reason') or 'fixed_nozzle_motion_retirement_unverified'))
            return (True,'') if data.get('can_start') is True else (False,data.get('ready_reason') or kind+'_not_ready')
        if kind == 'motor_test':
            if not self.ros['motor_test_ready']:
                return False, 'motor_test_subscriber_unavailable'
            if stop:
                return True, ''
            evidence = {key: self.envelope(key) for key in ('mission', 'actuators', 'fpga_link')}
            if any(not envelope['fresh'] for envelope in evidence.values()):
                return False, 'motor_test_system_status_stale'
            reason = motor_bench_guard({key: value['data'] or {} for key, value in evidence.items()}, self.wall())
            return not reason, reason
        if kind in ('listen', 'audio_test'):
            voice = self.envelope('voice')
            subscriber_key = 'listen_ready' if kind == 'listen' else 'audio_test_ready'
            if not self.ros[subscriber_key]:
                return False, ('voice_subscriber_unavailable' if kind == 'listen'
                               else 'audio_test_subscriber_unavailable')
            if stop:
                return True, ''
            if voice['fresh'] and (voice['data'] or {}).get('voice_enabled', True) is False:
                return False, 'voice_disabled'
            if not voice['fresh'] or not (voice['data'] or {}).get('microphone_enabled'):
                return False, 'microphone_disabled_or_voice_status_stale'
            return True, ''
        if not self.ros['router_ready']:
            return False, 'task_router_unavailable'
        if not stop and not self.envelope('mission', copy_data=False)['fresh']:
            return False, 'mission_state_stale'
        return True, ''

    def execution_ready(self, parsed):
        """Reject unavailable real outputs, both before queueing and before publishing."""
        action = parsed.get('action')
        if action == 'manual_pump_test':
            return self.ready('pump_manual')
        if action == 'extinguish_selected':
            keys = ('mission', 'fire_selection', 'fire_proposal', 'actuators')
            evidence = {key: self.envelope(key) for key in keys}
            if any(not value['fresh'] for value in evidence.values()):
                return False, 'selected_fire_evidence_stale'
            extension = self.envelope('extension')
            failure = selected_fire_round_reason(self.wall(),
                mission=evidence['mission']['data'], selection=evidence['fire_selection']['data'],
                proposal=evidence['fire_proposal']['data'], actuator=evidence['actuators']['data'],
                extension=extension['data'], extension_enabled=extension['received_at'] is not None,
                request=parsed)
            return not failure, failure
        if action == 'search_region':
            if type(parsed.get('execute_fire')) is not bool:
                return False, 'region_execute_fire_must_be_boolean'
            envelope = self.envelope('extension')
            data = envelope['data'] or {}
            ready_key, reason_key = ('region_ready', 'region_reason') if parsed['execute_fire'] else (
                'region_observe_ready', 'region_observe_reason')
            if not envelope['fresh']:
                return False, 'extension_status_unavailable'
            if data.get(ready_key) is not True:
                return False, data.get(reason_key) or 'region_search_not_ready'
        if action in ('navigate_point', 'confirm_point_start'):
            envelope = self.envelope('extension')
            if not envelope['fresh']:
                return False, 'extension_status_unavailable'
            declares_start = action == 'navigate_point' and parsed.get('start_declared') is True
            reason = point_request_guard(envelope['data'] or {}, self.wall(),
                                         action == 'confirm_point_start' or declares_start)
            if reason:
                return False, reason
        if action == 'set_zebra_detection':
            envelope = self.envelope('extension')
            if not envelope['fresh']:
                return False, 'extension_status_unavailable'
            reason = zebra_switch_guard(envelope['data'] or {}, self.wall())
            if reason:
                return False, reason
        if action in ('manual_move', 'route_turn', 'seek_fire', 'set_reference', 'traffic_drive'):
            envelope = self.envelope('extension')
            evidence = envelope['data'] or {}
            if not envelope['fresh'] or evidence.get('status_fresh') is not True:
                return False, 'extension_status_unavailable'
            capability = {'manual_move': 'distance', 'route_turn': 'route_turn', 'seek_fire': 'velocity', 'set_reference': 'route_turn', 'traffic_drive': 'traffic'}[action]
            reason = extension_start_guard(evidence, self.wall(), capability)
            if reason:
                return False, reason
            if action == 'set_reference' and evidence.get('vehicle_stopped') is not True:
                return False, 'measured_stop_required'
            if action == 'route_turn' and (evidence.get('reference') or {}).get('confirmed') is not True:
                return False, 'confirm_physical_start_before_junction_route'
            if action == 'traffic_drive' and (evidence.get('task') != 'traffic' or evidence.get('fpga_task_mode') != 3 or evidence.get('mode_transition')):
                return False, 'select_traffic_task_and_wait_ready_before_drive'
            if action == 'traffic_drive' and (evidence.get('zebra_detection_enabled') is not True or evidence.get('zebra_transition')):
                return False, 'enable_fpga_zebra_detection_before_traffic_drive'
        if action == 'navigate':
            link = self.envelope('fpga_link')
            data = link['data'] or {}
            if not link['fresh'] or data.get('status_fresh') is not True:
                return False, 'real_fpga_feedback_required'
            if data.get('dry_run') is not False or data.get('error'):
                return False, 'real_uart_not_enabled'
        needs_pump = action in ('start_spray', 'auto_fire', 'extinguish_selected') or (action == 'search_region' and parsed.get('execute_fire') is True) or (action == 'seek_fire' and parsed.get('after_fire') == 'auto') or (
            action in ('navigate', 'set_arrival_policy') and parsed.get('arrival_policy') == 'auto')
        if needs_pump:
            envelope = self.envelope('actuators')
            data = envelope['data'] or {}
            if not envelope['fresh']:
                return False, 'real_actuator_feedback_required'
            if data.get('dry_run') is not False or data.get('enabled') is not True:
                return False, 'real_actuator_not_enabled'
            if data.get('pump_enabled') is not True:
                return False, 'real_pump_not_enabled'
            if data.get('estop') or data.get('error'):
                return False, 'actuator_fault_or_estop'
        return True, ''

    def gimbal_reference_ready(self):
        state=self.envelope('gimbal_manual'); actuator=self.envelope('actuators')
        a=actuator['data'] or {}; status=state['data'] or {}
        if not state['fresh']:return False,'gimbal_manual_status_stale'
        if (not actuator['fresh'] or a.get('dry_run') is not False or
                a.get('raw_feedback_fresh') is not True):
            return False,'gimbal_reference_raw_feedback_required'
        reference_recovery = (a.get('reference_capture_supported') is True and
                              a.get('reference_recovery_permitted') is True and
                              bool(a.get('error')) and
                              a.get('reference_recovery_error') == a.get('error'))
        if a.get('estop') or (a.get('error') and not reference_recovery):
            return False,'actuator_fault_or_estop'
        counts=a.get('motor_counts'); speeds=a.get('motor_rpm')
        if (not isinstance(counts,list) or len(counts)!=2 or any(type(v) is not int for v in counts)
                or not isinstance(speeds,list) or len(speeds)!=2
                or any(type(v) not in (int,float) or not math.isfinite(v) or v!=0 for v in speeds)):
            return False,'gimbal_reference_stationary_readback_required'
        if status.get('reference_capture_ready') is not True:
            return False,status.get('reference_capture_reason') or 'gimbal_reference_not_ready'
        return True,''

    def gimbal_manual_ready(self, parsed):
        if parsed.get('action') in ('stop', 'recover'):return True,''
        if parsed.get('action') == 'confirm_reference':return self.gimbal_reference_ready()
        state=self.envelope('gimbal_manual'); actuator=self.envelope('actuators')
        a=actuator['data'] or {}; status=state['data'] or {}
        if (not actuator['fresh'] or a.get('dry_run') is not False or a.get('status_fresh') is not True
                or a.get('feedback_fresh') is not True):
            return False,'gimbal_manual_actuator_feedback_required'
        limits=a.get('axis_limits_deg');axes=status.get('motion_axes',[])
        if not isinstance(axes,list):return False,'gimbal_manual_motion_axes_unavailable'
        if not isinstance(limits,list) or len(limits)!=2:return False,'gimbal_manual_axis_limits_unavailable'
        for index,axis in enumerate(('pan','tilt')):
            key=axis+'_deg'
            if key not in parsed:continue
            if axis not in axes:return False,'gimbal_manual_'+axis+'_not_verified'
            bounds=limits[index]
            if (not isinstance(bounds,list) or len(bounds)!=2 or
                    any(type(v) not in (int,float) or not math.isfinite(v) for v in bounds) or bounds[0]>=bounds[1]):
                return False,'gimbal_manual_axis_limits_unavailable'
            if not bounds[0]<=parsed[key]<=bounds[1]:return False,'gimbal_manual_'+axis+'_outside_limits'
        return True,''

    def enqueue(self, text=None, request_id=None, listen=None, audio_test=None, motor_test=None, motor_id=None, point_command=None, gimbal_test=None, gimbal_manual=None, pump_manual=None, cancel_start_request_id=None, target_expectation=None, voice_control=None):
        if not isinstance(request_id, str) or not ID_RE.fullmatch(request_id):
            raise ApiError(400, 'valid_request_id_required')
        if sum(value is not None for value in (text, listen, audio_test, motor_test, point_command, gimbal_test, gimbal_manual, pump_manual, voice_control)) != 1:
            raise ApiError(400, 'exactly_one_request_kind_required')
        if voice_control is not None:
            if type(voice_control) is not bool:
                raise ApiError(400, 'enabled_must_be_boolean')
            parsed, kind = {'enabled': voice_control}, 'voice_control'
        elif pump_manual is not None:
            if not isinstance(pump_manual, dict) or pump_manual.get('action') not in ('start', 'stop'):
                raise ApiError(400, 'pump_manual_start_or_stop_required')
            allowed = {'action', 'duration_ms', 'operator_confirmed','fixed_nozzle_confirmed'} if pump_manual['action'] == 'start' else {'action'}
            if set(pump_manual) - allowed:
                raise ApiError(400, 'unsupported_pump_manual_fields')
            parsed = dict(pump_manual)
            if parsed['action'] == 'start':
                parsed.setdefault('duration_ms', 400)
                if type(parsed['duration_ms']) is not int or not 100 <= parsed['duration_ms'] <= 1000:
                    raise ApiError(400, 'pump_manual_duration_ms_must_be_integer_100_to_1000')
                if parsed.get('operator_confirmed') is not True:
                    raise ApiError(400, 'pump_manual_operator_confirmation_required')
                if 'fixed_nozzle_confirmed' in parsed and type(parsed['fixed_nozzle_confirmed']) is not bool:
                    raise ApiError(400,'manual_pump_fixed_nozzle_confirmation_must_be_boolean')
            pump_manual = dict(parsed)
            kind = 'pump_manual'
            text = '手动定点单次试喷' if parsed['action'] == 'start' else '停止手动试喷'
        elif gimbal_manual is not None:
            if not isinstance(gimbal_manual,dict) or gimbal_manual.get('action') not in ('start','stop','confirm_reference','recover'):
                raise ApiError(400,'gimbal_manual_action_required')
            allowed=({'action','pan_deg','tilt_deg'} if gimbal_manual['action']=='start' else
                     {'action','operator_zero_confirmed','supported_load','stationary_confirmed'}
                     if gimbal_manual['action']=='confirm_reference' else {'action'})
            if set(gimbal_manual)-allowed:raise ApiError(400,'unsupported_gimbal_manual_fields')
            parsed=dict(gimbal_manual)
            if parsed['action']=='start':
                if 'pan_deg' not in parsed:raise ApiError(400,'gimbal_manual_pan_required')
                for key in ('pan_deg','tilt_deg'):
                    if key in parsed and (type(parsed[key]) not in (int,float) or not math.isfinite(parsed[key])):
                        raise ApiError(400,'gimbal_manual_finite_angles_required')
            if parsed['action']=='confirm_reference' and any(parsed.get(key) is not True for key in
                    ('operator_zero_confirmed','supported_load','stationary_confirmed')):
                raise ApiError(400,'gimbal_reference_operator_confirmation_required')
            kind='gimbal_manual'
            text=({'start':'云台到指定角度','stop':'停止云台角度运动',
                   'confirm_reference':'确认云台当前位置为机械零位', 'recover':'重置连接并恢复双轴回读'})[parsed['action']]
        elif gimbal_test is not None:
            if not isinstance(gimbal_test,dict) or gimbal_test.get('action') not in ('start','stop'):
                raise ApiError(400,'gimbal_test_start_or_stop_required')
            allowed=({'action','axes','amplitude_deg','speed_deg_s'} if gimbal_test['action']=='start' else {'action'})
            if set(gimbal_test)-allowed:raise ApiError(400,'unsupported_gimbal_test_fields')
            parsed=dict(gimbal_test)
            if parsed['action']=='start':
                parsed.setdefault('axes','both');parsed.setdefault('amplitude_deg',1.);parsed.setdefault('speed_deg_s',6.)
                if parsed['axes'] not in ('pan','tilt','both'):raise ApiError(400,'invalid_gimbal_test_axes')
                for key,maximum in (('amplitude_deg',1.),('speed_deg_s',6.)):
                    if type(parsed[key]) not in (int,float) or not math.isfinite(parsed[key]) or not 0<parsed[key]<=maximum:
                        raise ApiError(400,'invalid_gimbal_test_'+key)
            kind='gimbal_self_test'
            text='开始两轴小角度自检' if parsed['action']=='start' else '停止两轴自检'
        elif point_command is not None:
            if not isinstance(point_command, dict):
                raise ApiError(400, 'point_command_must_be_object')
            action = point_command.get('action')
            allowed = ({'action', 'point_id', 'confirmed', 'start_heading_deg'} if action == 'confirm_point_start'
                       else {'action', 'target_point', 'start_point', 'start_heading_deg'} if action == 'navigate_point' else set())
            if not allowed or set(point_command) - allowed:
                raise ApiError(400, 'unsupported_point_command_fields')
            parsed = dict(point_command, source='web', manual_point_command=True)
            try:
                validate_point_request(parsed)
            except (ValueError, TypeError) as exc:
                raise ApiError(400, str(exc)) from exc
            text = ('确认实际起点为{}号点'.format(parsed['point_id']) if action == 'confirm_point_start'
                    else ('从{}号点导航到{}号点'.format(parsed['start_point'], parsed['target_point'])
                          if parsed.get('start_point') is not None else '导航到{}号点'.format(parsed['target_point'])))
            kind = 'command'
        elif motor_test is not None:
            if not isinstance(motor_test, str) or motor_test not in MOTOR_TEST_ACTIONS or motor_test == 'keepalive':
                raise ApiError(400, 'unsupported_motor_test_action')
            if motor_id is None:
                motor_id = 'motor1'
            if not isinstance(motor_id, str) or motor_id not in MOTOR_TEST_IDS:
                raise ApiError(400, 'unsupported_motor_id')
            parsed, kind = {'action': motor_test}, 'motor_test'
        elif listen is None and audio_test is None:
            if not isinstance(text, str) or not text.strip() or len(text) > 200:
                raise ApiError(400, 'text_must_be_1_to_200_characters')
            try:
                parsed = parse_task_text(text, self.buildings)
                if parsed.get('action') == 'navigate':
                    # A bare console destination never inherits automatic spraying.
                    parsed.setdefault('arrival_policy', 'manual')
            except (ValueError, TypeError, KeyError) as exc:
                raise ApiError(400, str(exc)) from exc
            kind = 'command'
        else:
            enabled = listen if listen is not None else audio_test
            if type(enabled) is not bool:
                raise ApiError(400, 'enabled_must_be_boolean')
            parsed, kind = {'enabled': enabled}, 'listen' if listen is not None else 'audio_test'
        if target_expectation is not None:
            if (kind != 'command' or parsed.get('action') != 'extinguish_selected' or
                    not isinstance(target_expectation, dict) or
                    set(target_expectation) != {'target_id', 'registry_session', 'selection_generation'}):
                raise ApiError(400, 'target_expectation_only_for_selected_fire_round')
            if (type(target_expectation['target_id']) is not int or target_expectation['target_id'] <= 0 or
                    type(target_expectation['selection_generation']) is not int or target_expectation['selection_generation'] <= 0 or
                    not isinstance(target_expectation['registry_session'], str) or
                    not 1 <= len(target_expectation['registry_session']) <= 160):
                raise ApiError(400, 'invalid_selected_fire_identity')
            parsed.update({'expected_'+name: value for name, value in target_expectation.items()})
        if kind != 'motor_test' and motor_id is not None:
            raise ApiError(400, 'motor_id_only_for_motor_test')
        if cancel_start_request_id is not None:
            if (kind != 'listen' or listen is not False
                    or not isinstance(cancel_start_request_id, str)
                    or not ID_RE.fullmatch(cancel_start_request_id)
                    or cancel_start_request_id == request_id):
                raise ApiError(400, 'cancel_start_request_id_only_for_listen_stop')
        with self.lock:
            if request_id in self.commands:
                old = self.commands[request_id]
                if (old['text'] != text or old.get('listen') != listen
                        or old.get('audio_test') != audio_test or old.get('motor_test') != motor_test
                        or old.get('motor_id') != motor_id
                        or old.get('point_command') != point_command
                        or old.get('gimbal_test') != gimbal_test
                        or old.get('gimbal_manual') != gimbal_manual
                        or old.get('pump_manual') != pump_manual
                        or old.get('voice_control') != voice_control
                        or old.get('target_expectation') != target_expectation
                        or old.get('cancel_start_request_id') != cancel_start_request_id
                        or old['kind'] != kind):
                    raise ApiError(409, 'request_id_reused_with_different_payload')
                return copy.deepcopy(old)
            # A Stop may arrive over HTTP before its in-flight Start. Retain the
            # cancelled ID in the stop ledger so that Start cannot open the mic later.
            if listen is True and any(v.get('cancel_start_request_id') == request_id
                                      for v in self.commands.values()):
                raise ApiError(409, 'superseded_by_listen_stop')
            emergency = kind == 'command' and parsed.get('action') in ('stop', 'stop_spray', 'finish_task')
            listen_stop = kind == 'listen' and listen is False
            test_stop = kind == 'audio_test' and audio_test is False
            motor_stop = kind == 'motor_test' and motor_test in ('stop', 'disconnect')
            gimbal_stop = kind in ('gimbal_self_test','gimbal_manual') and parsed['action']=='stop'
            gimbal_recover = kind == 'gimbal_manual' and parsed['action'] == 'recover'
            pump_stop = kind == 'pump_manual' and parsed['action'] == 'stop'
            if emergency or motor_stop:
                # Revoke at receipt, before ROS dispatch. An in-flight browser
                # heartbeat may not renew the run which this Stop is ending.
                affected = MOTOR_TEST_IDS if emergency else (motor_id,)
                for owner in affected:
                    self.motor_heartbeats.pop(owner, None)
                    for run in self.commands.values():
                        if (run.get('kind') == 'motor_test' and run.get('motor_id') == owner
                                and run.get('motor_test') in ('run_positive', 'run_negative')):
                            run['keepalive_revoked'] = True
            if len(self.commands) >= 1024 and not (emergency or listen_stop or test_stop or motor_stop or gimbal_stop or gimbal_recover or pump_stop):
                raise ApiError(429, 'command_session_limit_restart_required')
            # Stop requests preempt unsent motion/pump requests; no delayed arm after stop.
            if emergency:
                for pending_id, _, _ in self.pending:
                    self.commands[pending_id].update(status='rejected', reason='superseded_by_stop')
                self.pending.clear()
                if len(self.commands) >= 1152:
                    # Repeating a historical stop is harmless; never evict movement/start IDs.
                    expired_stop = next((k for k, v in self.commands.items()
                                         if v.get('action') in ('stop', 'stop_spray', 'finish_task')), None)
                    if expired_stop is not None:
                        self.commands.pop(expired_stop)
            elif listen_stop or test_stop or motor_stop or gimbal_stop or gimbal_recover or pump_stop:
                # Stop only this owner; preserve unrelated tasks and the other
                # owners' separately reserved stop slots.
                keep = []
                for pending_id, pending_data, queued_at in self.pending:
                    pending_record = self.commands[pending_id]
                    superseded = (gimbal_recover and pending_record['kind'] in ('gimbal_manual', 'gimbal_self_test')
                                  and pending_data.get('action') != 'stop')
                    if superseded or (not gimbal_recover and pending_record['kind'] == kind
                            and (not motor_stop or pending_record.get('motor_id', 'motor1') == motor_id)):
                        self.commands[pending_id].update(status='rejected', reason=(
                            'superseded_by_gimbal_recovery' if gimbal_recover else 'superseded_by_' + kind + '_stop'))
                    else:
                        keep.append((pending_id, pending_data, queued_at))
                self.pending.clear()
                self.pending.extend(keep)
                if len(self.commands) >= 1152:
                    expired_stop = next((k for k, v in self.commands.items()
                                         if v['status'] != 'queued' and ((v['kind'] == 'listen' and v.get('listen') is False)
                                         or (v['kind'] == 'audio_test' and v.get('audio_test') is False)
                                         or (v['kind'] == 'motor_test' and v.get('motor_test') in ('stop', 'disconnect'))
                                         or (v['kind']=='gimbal_self_test' and v.get('gimbal_test',{}).get('action')=='stop')
                                         or (v['kind']=='gimbal_manual' and v.get('gimbal_manual',{}).get('action') in ('stop','recover'))
                                         or (v['kind']=='pump_manual' and v.get('pump_manual',{}).get('action')=='stop'))), None)
                    if expired_stop is not None:
                        self.commands.pop(expired_stop)
            if len(self.pending) >= (self.pending.maxlen if listen_stop or test_stop or motor_stop or gimbal_stop or gimbal_recover or pump_stop else 8):
                raise ApiError(429, 'command_queue_full')
            stop = emergency or listen is False or test_stop or motor_stop or gimbal_stop or pump_stop
            ready, reason = self.ready(kind, stop, reference=kind=='gimbal_manual' and parsed.get('action')=='confirm_reference',
                                       recovery=gimbal_recover,
                                       fixed_nozzle=kind=='pump_manual' and parsed.get('fixed_nozzle_confirmed') is True)
            # Stopping listening is allowed with a discovered subscriber even if its mic is disabled.
            if listen is False and self.ros['available'] and self.ros['listen_ready']:
                ready = True
            if not ready:
                raise ApiError(409, reason)
            ready, reason = self.execution_ready(parsed)
            if not ready:
                raise ApiError(409, reason)
            if kind=='gimbal_manual':
                ready,reason=self.gimbal_manual_ready(parsed)
                if not ready:raise ApiError(409,reason)
            record = dict(request_id=request_id, command_id='presentation-' + uuid.uuid4().hex,
                          text=text, action=parsed.get('action', kind), kind=kind, listen=listen,
                          audio_test=audio_test, motor_test=motor_test, motor_id=motor_id,
                          point_command=copy.deepcopy(point_command),
                          gimbal_test=copy.deepcopy(gimbal_test),
                          gimbal_manual=copy.deepcopy(gimbal_manual),
                          pump_manual=copy.deepcopy(pump_manual),
                          voice_control=voice_control,
                          target_expectation=copy.deepcopy(target_expectation),
                          cancel_start_request_id=cancel_start_request_id,
                          status='queued', queued_at=self.wall(), synthetic=False)
            self.commands[request_id] = record
            pending_item = (request_id, parsed, self.monotonic())
            if listen_stop or test_stop or motor_stop or gimbal_stop or pump_stop:
                self.pending.appendleft(pending_item)
            else:
                self.pending.append(pending_item)
            self.event('command_queued', record)
            return copy.deepcopy(record)

    def drain(self, publish, ros_now):
        """Called only by the live ROS executor timer; never fabricates a receipt."""
        for _ in range(8):
            with self.lock:
                if not self.pending:
                    break
                request_id, parsed, queued = self.pending.popleft()
                record = self.commands[request_id]
                queue_ttl = 2. if record['kind'] == 'pump_manual' or parsed.get('action') == 'manual_pump_test' else 3.
                pump_stop = record['kind'] == 'pump_manual' and parsed.get('action') == 'stop'
                if not pump_stop and not 0 <= self.monotonic() - queued <= queue_ttl:
                    record.update(status='expired', reason='queue_ttl_exceeded')
                    continue
                stop = (parsed.get('action') in ('stop', 'stop_spray', 'finish_task')
                        or record.get('listen') is False or record.get('audio_test') is False
                        or record['kind'] == 'motor_test' and record.get('motor_test') == 'disconnect')
                ready, reason = self.ready(record['kind'], stop,
                    reference=record['kind']=='gimbal_manual' and parsed.get('action')=='confirm_reference',
                    recovery=record['kind']=='gimbal_manual' and parsed.get('action')=='recover',
                    fixed_nozzle=record['kind']=='pump_manual' and parsed.get('fixed_nozzle_confirmed') is True)
                if record.get('listen') is False and self.ros['available'] and self.ros['listen_ready']:
                    ready = True
                if not ready:
                    record.update(status='rejected', reason=reason)
                    continue
                ready, reason = self.execution_ready(parsed)
                if not ready:
                    record.update(status='rejected', reason=reason)
                    continue
                if record['kind']=='gimbal_manual':
                    ready,reason=self.gimbal_manual_ready(parsed)
                    if not ready:
                        record.update(status='rejected',reason=reason)
                        continue
                value = dict(parsed, command_id=record['command_id'], source='web',
                             issued_at=ros_now(), ttl_s=2. if parsed.get('action') == 'manual_pump_test' else 3.)
                if record['kind'] == 'motor_test':
                    value = dict(action=record['motor_test'], request_id=record['command_id'],
                                 issued_at=value['issued_at'], ttl_s=3., motor_id=record['motor_id'])
                elif record['kind'] in ('gimbal_self_test','gimbal_manual'):
                    value=dict(parsed,request_id=record['command_id'],issued_at=value['issued_at'],ttl_s=3.)
                elif record['kind'] == 'pump_manual':
                    value = dict(parsed, command_id=record['command_id'], issued_at=value['issued_at'], ttl_s=2.)
                elif record['kind'] == 'voice_control':
                    # Voice owns this request ID. Do not substitute the task
                    # router's generated command ID or attach actuator fields.
                    value = dict(enabled=record['voice_control'], request_id=record['request_id'])
                # Remove from pending before publish. An exception never causes an automatic retry.
                try:
                    record['dispatch_started_at'] = self.wall()
                    publish(record['kind'], value)
                    observed_status = (record['status'] if (record['kind'] == 'audio_test'
                                       and record.get('audio_test_status') or record['kind'] == 'motor_test'
                                       and record.get('motor_test_status') or record['kind']=='gimbal_self_test'
                                       and record.get('gimbal_self_test_status') or record['kind']=='gimbal_manual'
                                       and record.get('gimbal_manual_status') or record['kind'] == 'pump_manual'
                                       and record.get('pump_manual_status') or record['kind'] == 'voice_control'
                                       and record.get('voice_control_status')) else
                                       'execution' if record.get('execution') else
                                       'receipt' if record.get('receipt') else 'submitted')
                    record.update(status=observed_status, submitted_at=self.wall())
                    if record['kind'] == 'listen':
                        record['reason'] = 'listen_dispatched_observe_voice_status_for_outcome'
                    elif record['kind'] == 'audio_test' and record['status'] != 'rejected':
                        record['reason'] = 'audio_test_dispatched_observe_test_status_for_outcome'
                    elif record['kind'] == 'motor_test' and not record.get('motor_test_status'):
                        record['reason'] = 'motor_test_dispatched_waiting_feedback'
                    elif record['kind']=='gimbal_self_test' and not record.get('gimbal_self_test_status'):
                        record['reason']='gimbal_test_dispatched_waiting_encoder_feedback'
                    elif record['kind']=='gimbal_manual' and not record.get('gimbal_manual_status'):
                        record['reason']=('gimbal_reference_dispatched_waiting_reference_feedback' if
                            parsed.get('action')=='confirm_reference' else 'gimbal_recovery_dispatched_waiting_feedback' if
                            parsed.get('action')=='recover' else 'gimbal_manual_dispatched_waiting_encoder_feedback')
                    elif record['kind'] == 'pump_manual' and not record.get('pump_manual_status'):
                        record['reason'] = 'pump_manual_dispatched_waiting_hardware_feedback'
                    elif record['kind'] == 'voice_control' and not record.get('voice_control_status'):
                        record['reason'] = 'voice_control_dispatched_waiting_applied_feedback'
                    self.event('command_submitted', record)
                except Exception as exc:
                    record.update(status='rejected', reason='publish_failed:' + type(exc).__name__)
        self.drain_motor_heartbeats(publish, ros_now)

    def _motor_heartbeat_guard(self, motor_id, run_id):
        ready, reason = self.ready('motor_test')
        if not ready:
            raise ApiError(409, reason)
        run = next((v for v in self.commands.values() if v['command_id'] == run_id), None)
        if (run is None or run.get('kind') != 'motor_test' or run.get('motor_id') != motor_id
                or run.get('motor_test') not in ('run_positive', 'run_negative')
                or run.get('keepalive_revoked') or run['status'] in ('queued', 'expired', 'rejected')):
            raise ApiError(409, 'motor_heartbeat_unknown_or_ended_run')
        envelope = self.envelope('motor_test')
        motors = (envelope['data'] or {}).get('motors')
        motor = motors.get(motor_id) if isinstance(motors, dict) else None
        if not isinstance(motor, dict):
            raise ApiError(409, 'motor_heartbeat_live_run_required')
        stamp = source_stamp({'stamp': motor.get('stamp')})
        if (not envelope['fresh'] or motor.get('synthetic') is True or stamp is None
                or not -.05 <= self.wall() - stamp <= 1.0
                or not motor.get('feedback_fresh') or not motor.get('connected')
                or motor.get('run_id') != run_id
                or not (motor.get('continuous_running') or motor.get('continuous_pending'))):
            raise ApiError(409, 'motor_heartbeat_live_run_required')

    def motor_heartbeat(self, motor_id, run_id):
        if not isinstance(motor_id, str) or motor_id not in MOTOR_TEST_IDS:
            raise ApiError(400, 'unsupported_motor_id')
        if not isinstance(run_id, str) or not ID_RE.fullmatch(run_id):
            raise ApiError(400, 'valid_run_id_required')
        with self.lock:
            self._motor_heartbeat_guard(motor_id, run_id)
            self.motor_heartbeats[motor_id] = (run_id, self.monotonic())
        return dict(motor_id=motor_id, run_id=run_id, queued=True, synthetic=False)

    def drain_motor_heartbeats(self, publish, ros_now):
        with self.lock:
            pending, self.motor_heartbeats = self.motor_heartbeats, {}
            for motor_id, (run_id, queued) in pending.items():
                age = self.monotonic() - queued
                if not 0 <= age < 1.:
                    continue
                try:
                    self._motor_heartbeat_guard(motor_id, run_id)
                    publish('motor_test', dict(action='keepalive', motor_id=motor_id, run_id=run_id,
                        request_id='motor-heartbeat-' + uuid.uuid4().hex,
                        issued_at=ros_now(), ttl_s=max(.01, 1. - age)))
                except Exception:
                    # Never retry: the board's short run lease expires instead.
                    continue

    def snapshot(self, *, copy_data=True):
        with self.lock:
            for record in self.commands.values():
                if (record['kind'] == 'voice_control' and record['status'] in ('submitted', 'pending')
                        and self.wall() - record['submitted_at'] > 30):
                    record.update(status='receipt_timeout', reason='voice_control_confirmation_timeout_no_automatic_retry')
                elif record['kind'] in ('command', 'motor_test','gimbal_self_test','gimbal_manual','pump_manual') and record['status'] == 'submitted' and self.wall() - record['submitted_at'] > 5:
                    record.update(status='receipt_timeout', reason='no_receipt_no_automatic_retry')
                elif (record.get('action') in ('navigate_point', 'search_region') and record['status'] == 'receipt'
                      and record.get('receipt', {}).get('accepted') is True and not record.get('execution')
                      and self.wall() - record.get('receipt_received_at', self.wall()) > 5):
                    record.update(status='execution_timeout', reason='navigation_execution_receipt_missing')
            ready, reason = self.ready()
            listen_ready, _ = self.ready('listen')
            audio_test_ready, _ = self.ready('audio_test')
            voice_control_ready, voice_control_reason = self.ready('voice_control')
            voice = self.envelope('voice', copy_data=False)
            # Legacy nodes have no master switch and retain their old enabled
            # behavior. Readiness still requires the dedicated subscriber.
            voice_enabled = (voice['data'] or {}).get('voice_enabled', True)
            if type(voice_enabled) is not bool:
                voice_enabled = True
            motor_test_ready, motor_test_reason = self.ready('motor_test')
            preview = copy.deepcopy(self.preview)
            preview['fresh'] = (preview.get('updated_at') is not None and
                                0 <= self.wall() - preview['updated_at'] <= 3.)
            if not preview['fresh']:
                preview['upstream_available'] = False
            return dict(schema_version=1, mode='live', synthetic=False,
                        server_time=self.wall(), uptime_s=round(self.monotonic() - self.started, 2),
                        ros=copy.deepcopy(self.ros),
                        control=dict(requires_token=False, access_mode="direct", command_ready=ready, listen_ready=listen_ready,
                                     audio_test_ready=audio_test_ready,
                                     voice_control_ready=voice_control_ready, voice_control_reason=voice_control_reason,
                                     voice_enabled=voice_enabled, voice_state_fresh=voice['fresh'],
                                     motor_test_ready=motor_test_ready, motor_test_reason=motor_test_reason,
                                     read_only_reason=reason, accepted_is_execution=False),
                        preview=preview, topics={key: self.envelope(key, copy_data=copy_data) for key in (*TOPICS, STEREO_LAST_VALID_KEY)},
                        events=list(copy.deepcopy(self.events)) if copy_data else list(self.events),
                        commands=(copy.deepcopy(list(self.commands.values())[-40:]) if copy_data
                                  else list(self.commands.values())[-40:]),
                        buildings=[dict(id=str(b['building_id']), number=str(b['number']),
                                        name=b.get('name', '')) for b in self.buildings['buildings']])

    def snapshot_bytes(self):
        """Encode a current response without copying/re-encoding topic trees.

        Only immutable validated topic payload bytes are reused. Freshness,
        source/receipt ages, control gates and command timeouts are recomputed
        on every request. Mutable events/commands are encoded under the same
        lock; no borrowed objects escape to the HTTP writer.
        """
        with self.lock:
            value = self.snapshot(copy_data=False)
            topics = []
            for key, envelope in value['topics'].items():
                data = (self.last_valid_stereo_json if key == STEREO_LAST_VALID_KEY
                        else self.latest_json.get(key))
                if data is None:
                    data = compact(envelope['data'])
                metadata = {name: item for name, item in envelope.items() if name != 'data'}
                body = compact(metadata)[:-1] + b',"data":' + data + b'}'
                topics.append(compact(key) + b':' + body)
            topic_bytes = b'{' + b','.join(topics) + b'}'
            return b'{' + b','.join(compact(key) + b':' +
                (topic_bytes if key == 'topics' else compact(item))
                for key, item in value.items()) + b'}'

    def fpga_map(self, include_headings=False):
        with self.lock:
            values={key:self.envelope(key) for key in ('extension','navigation','fpga_map')}
            result = live_map_snapshot(self.map_catalog,
                values['extension']['data'] if values['extension']['fresh'] else {},
                values['navigation']['data'] if values['navigation']['fresh'] else {},self.wall(),
                values['fpga_map']['data'] if values['fpga_map']['fresh'] else {})
            result['semantic_regions'] = copy.deepcopy(self.semantic_regions)
            result['start_heading_samples_sha256'] = self.start_heading_samples_sha256
            if include_headings:
                result['start_heading_samples'] = copy.deepcopy(self.start_heading_samples)
            return result


class PreviewProxy:
    """Fixed loopback endpoint; no URL input, redirects, image decode or recompression."""
    def __init__(self, port=8086):
        self.port = port
        self.slots = threading.BoundedSemaphore(2)
        self.connections = queue.LifoQueue(maxsize=2)
        for _ in range(2):
            self.connections.put(http.client.HTTPConnection('127.0.0.1', self.port, timeout=2.))

    def request(self, path, payload=None, limit=MAX_FRAME_BYTES):
        if not self.slots.acquire(blocking=False):
            raise ApiError(503, 'preview_busy')
        connection = self.connections.get_nowait()
        reusable = False
        try:
            body = compact(payload) if payload is not None else None
            headers = {'Content-Type': 'application/json'} if body else {}
            connection.request('POST' if body else 'GET', path, body=body, headers=headers)
            response = connection.getresponse()
            length = response.getheader('Content-Length')
            if response.status != 200 or (length is not None and int(length) > limit):
                raise ApiError(502, 'preview_invalid_response')
            data = response.read(limit + 1)
            if len(data) > limit:
                raise ApiError(502, 'preview_response_too_large')
            reusable = True
            return data
        except (OSError, ValueError, http.client.HTTPException) as exc:
            raise ApiError(503, 'preview_unavailable') from exc
        finally:
            if not reusable:
                connection.close()
            self.connections.put_nowait(connection)
            self.slots.release()

    def close(self):
        while not self.connections.empty():
            self.connections.get_nowait().close()

    def metadata(self):
        data = json.loads(self.request('/api/state?images=0', limit=MAX_TOPIC_BYTES))
        if not isinstance(data, dict) or data.get('synthetic') is True or data.get('mode') in ('demo', 'recorded_demo'):
            raise ApiError(502, 'real_preview_required')
        return data

    def stream(self, enabled):
        return json.loads(self.request('/api/stream', {'enabled': enabled}, MAX_TOPIC_BYTES))

    def frame(self, after):
        packet = self.request('/api/frame?after=' + quote(after, safe=''))
        if len(packet) < 4:
            raise ApiError(502, 'preview_packet_truncated')
        size = struct.unpack('>I', packet[:4])[0]
        if size > MAX_TOPIC_BYTES or 4 + size > len(packet):
            raise ApiError(502, 'preview_metadata_invalid')
        try:
            metadata = json.loads(packet[4:4 + size])
            if not isinstance(metadata, dict) or metadata.get('transport_version') != 1:
                raise ValueError()
            if (metadata.get('synthetic') is True or metadata.get('mode') in ('demo', 'recorded_demo')
                    or isinstance(metadata.get('frame'), dict) and metadata['frame'].get('synthetic') is True):
                raise ApiError(502, 'real_preview_required')
        except (ValueError, TypeError):
            raise ApiError(502, 'preview_metadata_invalid')
        return packet


class BoundedServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    request_queue_size = 16

    def __init__(self, *args, **kwargs):
        self.connection_slots = threading.BoundedSemaphore(12)
        super().__init__(*args, **kwargs)

    def get_request(self):
        request, address = super().get_request()
        request.settimeout(4.)
        return request, address

    def process_request(self, request, address):
        if not self.connection_slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, address)
        except Exception:
            self.connection_slots.release()
            raise

    def process_request_thread(self, request, address):
        try:
            super().process_request_thread(request, address)
        finally:
            self.connection_slots.release()


def handler_class(state, web_dir, proxy, allowed_hosts, allow_socket_address=False):
    web_dir = Path(web_dir)
    assets = {'/': ('presentation.html', 'text/html; charset=utf-8'),
              '/presentation.html': ('presentation.html', 'text/html; charset=utf-8'),
              '/presentation.css': ('presentation.css', 'text/css; charset=utf-8'),
              '/presentation.js': ('presentation.js', 'application/javascript; charset=utf-8'),
              '/fire_commissioning.html': ('fire_commissioning.html', 'text/html; charset=utf-8'),
              '/fire_commissioning.js': ('fire_commissioning.js', 'application/javascript; charset=utf-8'),
              '/fire_commissioning.css': ('fire_commissioning.css', 'text/css; charset=utf-8'),
              '/assets/fpga_map.svg':('assets/fpga_map.svg','image/svg+xml')}
    allowed_hosts = {str(value).lower() for value in allowed_hosts}

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'
        disable_nagle_algorithm = True

        def log_message(self, *args):
            pass  # Do not log authorization headers or teacher command contents.

        def host(self):
            values = self.headers.get_all('Host', [])
            if len(values) != 1:
                raise ApiError(400, 'one_host_header_required')
            host = values[0].lower()
            socket_host = None
            if allow_socket_address and host not in allowed_hosts:
                # A new interface address may appear after this service starts.
                # Trust only this accepted socket's actual local destination,
                # never Host resolution, client input, DNS or a subnet wildcard.
                try:
                    address, port = self.connection.getsockname()[:2]
                    literal = ipaddress.IPv4Address(address)
                    if not literal.is_unspecified:
                        socket_host = str(literal) + ':' + str(port)
                except (OSError, ValueError, TypeError):
                    pass
            if host not in allowed_hosts and host != socket_host:
                raise ApiError(403, 'host_not_allowed')
            return host

        def validate_request_origin(self):
            host = self.host()
            if self.headers.get('Origin') != 'http://' + host:
                raise ApiError(403, 'same_origin_required')
            if self.headers.get('Sec-Fetch-Site', 'same-origin') not in ('same-origin', 'none'):
                raise ApiError(403, 'cross_site_forbidden')

        def send(self, code, payload, content_type='application/json; charset=utf-8', head=False):
            body = payload if isinstance(payload, bytes) else compact(payload)
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('X-Frame-Options', 'DENY')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' blob: data:; connect-src 'self'; object-src 'none'; frame-ancestors 'none'; base-uri 'none'")
            self.end_headers()
            if not head:
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError, socket.timeout):
                    pass

        def fail(self, exc):
            self.close_connection = True
            self.send(exc.code, {'error': exc.reason, 'synthetic': False})

        def do_HEAD(self):
            self.get(True)

        def do_GET(self):
            self.get(False)

        def get(self, head):
            try:
                self.host()
                if len(self.path) > 1024:
                    raise ApiError(414, 'url_too_long')
                parsed = urlsplit(self.path)
                if parsed.scheme or parsed.netloc:
                    raise ApiError(400, 'origin_form_required')
                if parsed.path in assets:
                    name, mime = assets[parsed.path]
                    self.send(200, (web_dir / name).read_bytes(), mime, head)
                elif parsed.path == '/api/state':
                    self.send(200, state.snapshot_bytes(), head=head)
                elif parsed.path == '/api/fpga-map':
                    self.send(200,state.fpga_map(include_headings=parse_qs(parsed.query).get('include_headings')==['1']),head=head)
                elif parsed.path in ('/api/frame', '/api/road-frame'):
                    if self.headers.get('Sec-Fetch-Site') == 'cross-site':
                        raise ApiError(403, 'cross_site_forbidden')
                    # HEAD never renews the upstream preview lease.
                    if head:
                        self.send(200, b'', 'application/octet-stream', head=True)
                        return
                    query = parse_qs(parsed.query)
                    if set(query) - {'after'} or any(len(v) != 1 for v in query.values()):
                        raise ApiError(400, 'invalid_frame_query')
                    after = query.get('after', [''])[0]
                    if len(after) > 256:
                        raise ApiError(400, 'frame_token_too_long')
                    if parsed.path == '/api/road-frame':
                        with state.lock:
                            enabled = state.preview.get('enabled') is True
                        packet = state.road_visualization.packet(enabled=enabled)
                    else:
                        packet = proxy.frame(after)
                    self.send(200, packet, 'application/octet-stream')
                elif parsed.path == '/favicon.ico':
                    self.send(204, b'', head=head)
                else:
                    raise ApiError(404, 'not_found')
            except ApiError as exc:
                self.fail(exc)
            except (OSError, ValueError, TypeError):
                self.fail(ApiError(503, 'resource_unavailable'))

        def do_POST(self):
            try:
                self.validate_request_origin()
                if self.headers.get('Transfer-Encoding') or len(self.headers.get_all('Content-Length', [])) != 1:
                    raise ApiError(400, 'single_content_length_required')
                if self.headers.get('Content-Type', '').split(';')[0].strip() != 'application/json':
                    raise ApiError(415, 'application_json_required')
                try:
                    length = int(self.headers['Content-Length'])
                except (ValueError, TypeError):
                    raise ApiError(400, 'invalid_content_length')
                if not 0 < length <= MAX_BODY_BYTES:
                    raise ApiError(413, 'request_body_limit')
                payload = json.loads(self.rfile.read(length))
                if not isinstance(payload, dict):
                    raise ApiError(400, 'json_object_required')
                if self.path == '/api/command':
                    if set(payload) not in ({'text', 'request_id'}, {'text', 'request_id', 'target_expectation'}):
                        raise ApiError(400, 'only_text_and_request_id_allowed')
                    if 'target_expectation' in payload and not isinstance(payload['target_expectation'], dict):
                        raise ApiError(400, 'invalid_target_expectation')
                    result = state.enqueue(**payload)
                    self.send(202, result)
                elif self.path == '/api/point-navigation':
                    if 'request_id' not in payload:
                        raise ApiError(400, 'valid_request_id_required')
                    self.send(202, state.enqueue(request_id=payload['request_id'],
                        point_command={k: v for k, v in payload.items() if k != 'request_id'}))
                elif self.path == '/api/gimbal-self-test':
                    if 'request_id' not in payload:raise ApiError(400,'valid_request_id_required')
                    self.send(202,state.enqueue(request_id=payload['request_id'],
                        gimbal_test={k:v for k,v in payload.items() if k!='request_id'}))
                elif self.path == '/api/gimbal-manual':
                    if 'request_id' not in payload:raise ApiError(400,'valid_request_id_required')
                    self.send(202,state.enqueue(request_id=payload['request_id'],
                        gimbal_manual={k:v for k,v in payload.items() if k!='request_id'}))
                elif self.path == '/api/pump-manual':
                    if 'request_id' not in payload:
                        raise ApiError(400, 'valid_request_id_required')
                    self.send(202, state.enqueue(request_id=payload['request_id'],
                        pump_manual={k: v for k, v in payload.items() if k != 'request_id'}))
                elif self.path == '/api/listen':
                    if set(payload) not in ({'enabled', 'request_id'}, {'enabled', 'request_id', 'cancel_start_request_id'}):
                        raise ApiError(400, 'enabled_and_request_id_required')
                    if type(payload['enabled']) is not bool:
                        raise ApiError(400, 'enabled_must_be_boolean')
                    self.send(202, state.enqueue(request_id=payload['request_id'], listen=payload['enabled'],
                                                cancel_start_request_id=payload.get('cancel_start_request_id')))
                elif self.path == '/api/voice-control':
                    if set(payload) != {'enabled', 'request_id'}:
                        raise ApiError(400, 'enabled_and_request_id_required')
                    if type(payload['enabled']) is not bool:
                        raise ApiError(400, 'enabled_must_be_boolean')
                    self.send(202, state.enqueue(request_id=payload['request_id'], voice_control=payload['enabled']))
                elif self.path == '/api/audio-test':
                    if set(payload) != {'enabled', 'request_id'}:
                        raise ApiError(400, 'enabled_and_request_id_required')
                    if type(payload['enabled']) is not bool:
                        raise ApiError(400, 'enabled_must_be_boolean')
                    self.send(202, state.enqueue(request_id=payload['request_id'], audio_test=payload['enabled']))
                elif self.path == '/api/motor-test':
                    if set(payload) not in ({'action', 'request_id'}, {'action', 'request_id', 'motor_id'}):
                        raise ApiError(400, 'action_and_request_id_required')
                    motor_id = payload.get('motor_id', 'motor1')
                    if not isinstance(motor_id, str) or motor_id not in MOTOR_TEST_IDS:
                        raise ApiError(400, 'unsupported_motor_id')
                    self.send(202, state.enqueue(request_id=payload['request_id'],
                                               motor_test=payload['action'], motor_id=motor_id))
                elif self.path == '/api/motor-heartbeat':
                    if set(payload) != {'motor_id', 'run_id'}:
                        raise ApiError(400, 'motor_id_and_run_id_required')
                    self.send(202, state.motor_heartbeat(**payload))
                elif self.path == '/api/stream':
                    if set(payload) != {'enabled'} or type(payload['enabled']) is not bool:
                        raise ApiError(400, 'enabled_boolean_required')
                    result = proxy.stream(payload['enabled'])
                    with state.lock:
                        state.preview.update(enabled=bool(result.get('stream', {}).get('enabled')),
                            upstream_available=True, updated_at=state.wall(), note=result.get('state'))
                    self.send(200, result)
                elif self.path == '/api/session':
                    if payload:
                        raise ApiError(400, 'empty_object_required')
                    self.send(200, {'authorized': True, 'synthetic': False})
                else:
                    raise ApiError(404, 'not_found')
            except ApiError as exc:
                self.fail(exc)
            except (ValueError, TypeError, UnicodeError, socket.timeout):
                self.fail(ApiError(400, 'invalid_json_or_timeout'))
    return Handler


def spin_until_stopped(node, stop_event, executor, is_ok):
    """Keep subscriptions registered while waiting for work or shutdown.

    The rclpy.spin_once(node) convenience function adds and removes the node
    on every call. Both operations wake the executor's guard condition, which
    can keep an otherwise idle console busy rebuilding its entire wait set.
    """
    executor.add_node(node)
    try:
        while not stop_event.is_set() and is_ok():
            # PresentationNode's permanent 50 ms tick already wakes the
            # executor for command dispatch and shutdown. A finite spin timeout
            # makes Humble allocate/destroy another ROS Timer for each wait set.
            executor.spin_once(timeout_sec=None)
    finally:
        executor.remove_node(node)
        executor.shutdown()


def ros_loop(state, stop_event):
    """No function outside this executor publishes on the live ROS graph."""
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos_event import SubscriptionEventCallbacks, PublisherEventCallbacks
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from std_msgs.msg import String, Bool
        from rescue_interfaces.msg import DetectionArray, MotionPermission, GimbalCommand
        rclpy.init()

        class PresentationNode(Node):
            def __init__(self):
                # This observer has no remotely configured parameters. Avoid
                # rebuilding six unused parameter services in every wait set.
                super().__init__('rescue_presentation', start_parameter_services=False)
                self.command = self.create_publisher(String, '/interaction/command', 10)
                self.listen = self.create_publisher(Bool, '/audio/listen', 10)
                self.audio_test = self.create_publisher(String, '/audio/test/request', 10)
                self.voice_control = self.create_publisher(String, '/voice/control', 10)
                self.motor_test = self.create_publisher(String, '/motor_test/request', 10)
                self.gimbal_self_test = self.create_publisher(String, '/gimbal/self_test/request', 10)
                self.gimbal_manual = self.create_publisher(String, '/gimbal/manual/request', 10)
                self.pump_manual = self.create_publisher(String, '/pump/manual/request', 10)
                qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
                # These are dashboard observers. Per-subscription default QoS warning
                # waitables were consuming ~half the executor's samples. Actual
                # data freshness still drives visible availability and control gates.
                observer_events = SubscriptionEventCallbacks(use_default_callbacks=False)
                self.road_visualization_subscription = None
                self.road_visualization_qos = qos
                self.observer_events = observer_events
                for key, (topic, _) in TOPICS.items():
                    if key == 'detections':
                        self.create_subscription(DetectionArray, topic, self.detections, qos, event_callbacks=observer_events)
                    elif key == 'motion_permission':
                        self.create_subscription(MotionPermission, topic, lambda msg: state.update('motion_permission',
                            dict(stamp=msg.header.stamp.sec + msg.header.stamp.nanosec/1e9,
                                 event_id=int(msg.event_id), allow_motion=bool(msg.allow_motion),
                                 lease_ms=int(msg.lease_ms), reason=msg.reason), take_ownership=True), qos, event_callbacks=observer_events)
                    elif key == 'gimbal_command':
                        self.create_subscription(GimbalCommand, topic, lambda msg: state.update('gimbal_command',
                            dict(stamp=msg.header.stamp.sec + msg.header.stamp.nanosec/1e9,
                                 owner=msg.owner, pan_deg=float(msg.pan_deg), tilt_deg=float(msg.tilt_deg),
                                 speed_deg_s=float(msg.speed_deg_s), lease_ms=int(msg.lease_ms)), take_ownership=True), qos, event_callbacks=observer_events)
                    else:
                        self.create_subscription(String, topic, lambda msg, k=key: self.receive(k, msg),
                                                 10 if key in EVENT_KEYS else qos, event_callbacks=observer_events)
                self.create_timer(.05, self.tick)

            def create_publisher(self, *args, **kwargs):
                # Includes Node's /parameter_events publisher. Only the default
                # incompatible-QoS logging waitable is omitted; message QoS,
                # dispatch, subscriber counts and execution receipts stay intact.
                kwargs.setdefault('event_callbacks', PublisherEventCallbacks(use_default_callbacks=False))
                return super().create_publisher(*args, **kwargs)

            def receive(self, key, msg):
                if len(msg.data) > MAX_TOPIC_BYTES:
                    return
                try:
                    if key == 'speech':
                        data = {'text': msg.data, 'playback_verified': False}
                        if msg.data.lstrip().startswith('{'):
                            request = json.loads(msg.data)
                            if isinstance(request, dict):
                                data.update(request=request, text=request.get('text', '请求播报当前场景'))
                    else:
                        data = json.loads(msg.data)
                    state.update(key, data, take_ownership=True)
                except (ValueError, TypeError):
                    pass

            def detections(self, msg):
                values = [dict(class_name=d.class_name, class_id=int(d.class_id), confidence=float(d.confidence),
                    bbox_xyxy=list(d.bbox_xyxy), depth_valid=bool(d.depth_valid),
                    depth_m=float(d.depth_m) if math.isfinite(float(d.depth_m)) else None,
                    depth_reason=d.depth_reason) for d in msg.detections[:128]]
                state.update('detections', dict(sequence=int(msg.sequence), width=int(msg.width), height=int(msg.height),
                    capture_stamp=msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                    coordinate_frame=msg.header.frame_id, detector_available=bool(msg.detector_available),
                    rectified=bool(msg.rectified), status=msg.status, detections=values), take_ownership=True)

            def publish_request(self, kind, value):
                if kind == 'listen':
                    self.listen.publish(Bool(data=value['enabled']))
                elif kind == 'voice_control':
                    self.voice_control.publish(String(data=compact(value).decode('utf-8')))
                elif kind == 'audio_test':
                    self.audio_test.publish(String(data=compact(value).decode('utf-8')))
                elif kind == 'motor_test':
                    self.motor_test.publish(String(data=compact(value).decode('utf-8')))
                elif kind=='gimbal_self_test':
                    self.gimbal_self_test.publish(String(data=compact(value).decode('utf-8')))
                elif kind=='gimbal_manual':
                    self.gimbal_manual.publish(String(data=compact(value).decode('utf-8')))
                elif kind == 'pump_manual':
                    self.pump_manual.publish(String(data=compact(value).decode('utf-8')))
                else:
                    self.command.publish(String(data=compact(value).decode('utf-8')))

            def tick(self):
                with state.lock:
                    state.ros.update(available=True, domain_id=int(os.getenv('ROS_DOMAIN_ID', '0')),
                        router_ready=self.command.get_subscription_count() > 0,
                        listen_ready=self.listen.get_subscription_count() > 0,
                        audio_test_ready=self.audio_test.get_subscription_count() > 0,
                        voice_control_ready=self.voice_control.get_subscription_count() > 0,
                        motor_test_ready=self.motor_test.get_subscription_count() > 0,
                        gimbal_self_test_ready=self.gimbal_self_test.get_subscription_count()>0,
                        gimbal_manual_ready=self.gimbal_manual.get_subscription_count()>0,
                        pump_manual_ready=self.pump_manual.get_subscription_count()>0, error=None)
                    image_enabled = state.preview.get('enabled') is True
                wanted = state.road_visualization.wanted(enabled=image_enabled)
                if wanted and self.road_visualization_subscription is None:
                    self.road_visualization_subscription = self.create_subscription(
                        String, '/road/visualization',
                        lambda msg: state.road_visualization.update(msg.data),
                        self.road_visualization_qos, event_callbacks=self.observer_events)
                elif not wanted and self.road_visualization_subscription is not None:
                    self.destroy_subscription(self.road_visualization_subscription)
                    self.road_visualization_subscription = None
                state.drain(self.publish_request, lambda: self.get_clock().now().nanoseconds / 1e9)

        node = PresentationNode()
        try:
            spin_until_stopped(node, stop_event,
                               SingleThreadedExecutor(context=node.context), rclpy.ok)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except Exception as exc:
        with state.lock:
            state.ros.update(available=False, router_ready=False, listen_ready=False, audio_test_ready=False, motor_test_ready=False,
                             gimbal_self_test_ready=False, gimbal_manual_ready=False, pump_manual_ready=False, voice_control_ready=False,
                             error=type(exc).__name__ + ': ' + str(exc)[:200])
    finally:
        with state.lock:
            state.ros.update(available=False, router_ready=False, listen_ready=False, audio_test_ready=False, motor_test_ready=False,gimbal_self_test_ready=False,gimbal_manual_ready=False,pump_manual_ready=False,voice_control_ready=False)


def background(state, proxy, stop_event):
    while not stop_event.is_set():
        try:
            result = proxy.metadata()  # images=0; does not touch the streaming lease
            with state.lock:
                state.preview.update(enabled=bool(result.get('stream', {}).get('enabled')),
                    upstream_available=True, updated_at=state.wall(), note=result.get('state'))
        except (ApiError, ValueError, TypeError):
            with state.lock:
                state.preview.update(upstream_available=False, updated_at=state.wall(), note='preview_unavailable')
        stop_event.wait(1.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--allow-host', action='append', default=[], help='Exact additional Host header including :port for LAN use')
    parser.add_argument('--allow-local-addresses', action='store_true', help='Allow local literal IPv4 hosts, including the accepted socket address after network changes')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('port must be 1..65535')
    if getattr(sys, 'frozen', False):
        parser.error('Run this ROS service on X5; the desktop app connects directly to X5')
    os.environ.setdefault('ROS_DOMAIN_ID', '0')
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '0')
    if os.environ['ROS_DOMAIN_ID'] != '0':
        parser.error('Live camera/task system requires ROS_DOMAIN_ID=0')
    buildings = json.loads((WORKSPACE / 'config/buildings.json').read_text(encoding='utf-8-sig'))
    state = PresentationState(buildings=buildings)
    proxy = PreviewProxy()
    hosts = {'localhost:' + str(args.port), '127.0.0.1:' + str(args.port), args.bind + ':' + str(args.port), *args.allow_host}
    if args.allow_local_addresses:
        try:
            interfaces = json.loads(subprocess.run(['ip', '-j', '-4', 'addr'], check=True,
                capture_output=True, timeout=2., text=True).stdout)
            for interface in interfaces:
                for address in interface.get('addr_info', []):
                    literal = str(ipaddress.IPv4Address(address['local']))
                    hosts.add(literal + ':' + str(args.port))
        except (OSError, ValueError, KeyError, subprocess.SubprocessError):
            print('Startup address discovery unavailable; accepted socket local IPv4 remains allowed', flush=True)
    server = BoundedServer((args.bind, args.port), handler_class(
        state, WORKSPACE / 'web', proxy, hosts, allow_socket_address=args.allow_local_addresses))
    stop_event = threading.Event()
    workers = [threading.Thread(target=background, args=(state, proxy, stop_event), daemon=True)]
    workers.append(threading.Thread(target=ros_loop, args=(state, stop_event), daemon=True))
    for worker in workers:
        worker.start()
    print('LIVE robot console - all commands go through the existing task router', flush=True)
    print('Open http://127.0.0.1:%d/ ; direct control, no unlock/token step' % args.port, flush=True)
    try:
        server.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        server.server_close()
        for worker in workers:
            worker.join(timeout=3.)
        if proxy is not None:
            proxy.close()


if __name__ == '__main__':
    main()
