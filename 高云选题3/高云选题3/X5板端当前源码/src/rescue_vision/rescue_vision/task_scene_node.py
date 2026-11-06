"""Fresh YOLO scene reports, speech and bounded read-only task dialogue."""
import json
import math
import re
import time
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor

from .node_common import Node
from std_msgs.msg import String
from rescue_interfaces.msg import DetectionArray
from .node_common import IMAGE_QOS, param, read_json, seconds, publish_json, spin
from .task_interaction import fresh
from .task_scene import build_scene, render_dialogue, fallback_decision, LocalIntentModel, SprayHistory
from .extension_control import dialogue_allowed


class TaskScene(Node):
    def __init__(self):
        super().__init__('task_scene')
        path = param(self, 'edge_dialogue_config', '')
        self.model = LocalIntentModel(read_json(path) if path else {'enabled': False})
        self.evidence = {}
        self.extension_enabled=bool(param(self,'extension_enabled',False))
        self.ocr_pose_context = None
        self.ocr_valid_after = 0.
        self.report = {}
        self.spray_history = SprayHistory()
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.pending = None
        self.seen = OrderedDict()
        self.report_pub = self.create_publisher(String, '/scene/report', 10)
        self.answer_pub = self.create_publisher(String, '/scene/answer', 10)
        self.dialogue_status_pub = self.create_publisher(String, '/scene/dialogue_status', 10)
        self.dialogue_status_value = None
        self.dialogue_status_published = 0.
        self.speech_pub = self.create_publisher(String, '/speech/say', 10)
        self.create_subscription(DetectionArray, '/perception/detections', self.detect, IMAGE_QOS)
        self.create_subscription(String, '/scene/question', self.question, 10)
        for key, topic in (('ocr', '/scene/ocr'), ('mission', '/mission/state'),
                           ('actuator', '/actuators/state'), ('targets', '/fire/targets'),
                           ('voice', '/voice/status'), ('fpga_link', '/fpga/link'),
                           ('navigation', '/navigation/state'), ('traffic', '/traffic/state'),
                           ('extension', '/extension/state')):
            self.create_subscription(String, topic, lambda msg, k=key: self.update(k, msg), 10)
        self.create_timer(.25, self.tick)
        self.dialogue_status('idle')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def dialogue_enabled(self):
        value=self.evidence.get('extension',({},None))[0]
        return not self.extension_enabled or dialogue_allowed(value,self.now())

    def update(self, key, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError('expected_object')
            data['_received_stamp'] = self.now()
            if key == 'mission':
                context = json.dumps([data.get(k) for k in ('mode', 'traffic_active', 'motion_allowed',
                    'gimbal_ready', 'gimbal_target_deg', 'observation_pose_id', 'task_id')], sort_keys=True)
                if context != self.ocr_pose_context:
                    self.ocr_pose_context = context
                    self.ocr_valid_after = self.now()
                    self.evidence.pop('ocr', None)
            if key == 'ocr' and data.get('valid') is True:
                capture = data.get('capture_stamp')
                if type(capture) not in (int, float) or not math.isfinite(capture) or capture < self.ocr_valid_after:
                    self.evidence.pop('ocr', None)
                    return
            self.evidence[key] = (data, time.monotonic())
            if key == 'actuator':
                self.spray_history.update(data, self.now())
        except (ValueError, TypeError):
            self.evidence.pop(key, None)

    def detect(self, msg):
        self.evidence['detection'] = (dict(capture_stamp=seconds(msg.header.stamp), sequence=msg.sequence,
            coordinate_frame=msg.header.frame_id, detector_available=bool(msg.detector_available),
            detections=[dict(class_name=d.class_name, confidence=float(d.confidence)) for d in msg.detections]), time.monotonic())

    def scene(self):
        # OCR has its own original-capture 3 s deadline. Slow inference must not
        # lose an otherwise valid sign merely because YOLO refreshes more often.
        state = {k: data for k, (data, received) in self.evidence.items()
                 if 0 <= time.monotonic()-received <= (12. if k == 'voice' else 3. if k == 'ocr' else 1.5)}
        system = {k: state.get(k, {}) for k in ('voice', 'fpga_link', 'navigation', 'traffic', 'mission', 'actuator', 'extension')}
        for key in ('voice', 'fpga_link', 'navigation', 'traffic', 'extension'):
            state.pop(key, None)
        report = build_scene(state.pop('detection', {}), self.now(), last_spray=self.spray_history.snapshot(), **state)
        report['system_status'] = system
        return report

    def dialogue_status(self, state, **details):
        self.dialogue_status_value = dict(stamp=self.now(), state=state, busy=self.pending is not None,
            dialogue_enabled=self.dialogue_enabled(),
            model=self.model.model, model_enabled=self.model.enabled,
            answer_timeout_s=self.model.answer_timeout, command_execution_allowed=False, **details)
        self.dialogue_status_published = time.monotonic()
        publish_json(self.dialogue_status_pub, self.dialogue_status_value)

    def question(self, msg):
        now = self.now()
        data, cid = None, None
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError('question_must_be_object')
            cid = data.get('command_id')
            if not self.dialogue_enabled():
                raise ValueError('select_observation_task_for_voice_and_dialogue')
            if not isinstance(cid, str) or not re.fullmatch(r'[A-Za-z0-9_:.-]{1,100}', cid):
                raise ValueError('invalid_command_id')
            ttl = data.get('ttl_s', 3.)
            if (isinstance(ttl, bool) or not isinstance(ttl, (int, float)) or not math.isfinite(ttl)
                    or not 0 < ttl <= 3. or not fresh(data, now, ttl)):
                raise ValueError('question_expired')
            text = data.get('text')
            if not isinstance(text, str) or not text.strip() or len(text) > 200:
                raise ValueError('invalid_question')
            if cid in self.seen:
                raise ValueError('duplicate_question')
            use_worker = self.model.requires_worker(text)
            if use_worker and self.pending is not None:
                raise ValueError('dialogue_busy')
            self.seen[cid] = now
            while len(self.seen) > 512:
                self.seen.popitem(last=False)
            if not use_worker:
                self.answer(data, fallback_decision(text))
                return
            snapshot = self.scene()
            snapshot['_question_stamp'] = now
            self.pending = dict(future=self.worker.submit(self.model.respond, text, snapshot),
                question=data, started=time.monotonic(), deadline=time.monotonic()+self.model.answer_timeout,
                expired=False)
            self.dialogue_status('thinking', command_id=cid, question=text, elapsed_s=0.)
        except (ValueError, TypeError, KeyError) as exc:
            publish_json(self.answer_pub, dict(stamp=now, accepted=False, reason=str(exc), command_id=cid))
            if self.pending is None:
                self.dialogue_status('error', command_id=cid, reason=str(exc), result_stamp=now)

    def answer(self, question, decision):
        # Live facts remain completion-time evidence. Historical analysis has a
        # separately labelled request snapshot and never becomes control evidence.
        now = self.now()
        if not self.dialogue_enabled():
            publish_json(self.answer_pub,dict(stamp=now,accepted=False,command_id=question.get('command_id'),
                reason='dialogue_task_changed_result_discarded'))
            return
        self.report = self.scene()
        text = render_dialogue(question['text'], decision, self.report, now)
        scope = decision.get('response_scope') or ('domain_reference' if decision['backend'] == 'domain_reference' else
            'general_knowledge' if decision['intent'] == 'knowledge' and text.startswith('通用知识') else 'current_facts')
        snapshot = decision.get('analysis_snapshot') if scope == 'historical_snapshot_analysis' else None
        snapshot_stamp = snapshot.get('capture_stamp') if isinstance(snapshot, dict) else None
        snapshot_age = (round(now-snapshot_stamp, 3) if type(snapshot_stamp) in (int, float)
            and math.isfinite(snapshot_stamp) and now >= snapshot_stamp else None)
        publish_json(self.answer_pub, dict(stamp=now, accepted=True, command_id=question.get('command_id'),
            text=text, intent=decision['intent'], backend=decision['backend'], model_error=decision.get('model_error'),
            facts=self.report, command_execution_allowed=False,
            model=decision.get('model') or self.model.model,
            general_chat_available=decision.get('general_chat_available') is True,
            response_scope=scope, analysis_snapshot=snapshot, snapshot_age_s=snapshot_age,
            answer_facts_source='request_snapshot' if snapshot is not None else 'completion_time'))
        # Fast factual answers may arrive while a different model question is
        # running. Keep its thinking indicator instead of claiming it completed.
        if self.pending is None or self.pending['question'].get('command_id') == question.get('command_id'):
            self.dialogue_status('error' if decision.get('model_error') else 'complete',
                command_id=question.get('command_id'), backend=decision['backend'], response_scope=scope,
                reason=decision.get('model_error'), result_stamp=now)
        # The historical-age prefix and final "no control" qualifier must not be
        # silently cut off when a richer analytical reply is spoken.
        self.speech_pub.publish(String(data=text[:900]))

    def tick(self):
        self.report = self.scene()
        publish_json(self.report_pub, self.report)
        if self.dialogue_status_value and time.monotonic()-self.dialogue_status_published >= 1.:
            value = dict(self.dialogue_status_value, stamp=self.now(), busy=self.pending is not None,
                         dialogue_enabled=self.dialogue_enabled())
            if self.pending is not None:
                value['elapsed_s'] = round(time.monotonic()-self.pending['started'], 1)
            self.dialogue_status_published = time.monotonic()
            publish_json(self.dialogue_status_pub, value)
        if self.pending is None:
            return
        pending = self.pending
        future, question = pending['future'], pending['question']
        if not self.dialogue_enabled() and not pending['expired']:
            pending['expired']=True
            future.cancel()
            publish_json(self.answer_pub,dict(stamp=self.now(),accepted=False,command_id=question.get('command_id'),
                reason='dialogue_task_changed_result_discarded'))
            self.dialogue_status('disabled',reason='select_observation_task_for_voice_and_dialogue')
        if time.monotonic() > pending['deadline'] and not pending['expired']:
            pending['expired'] = True
            self.answer(question, fallback_decision(question['text'], 'answer_deadline_exceeded'))
            # A running Python future cannot be canceled. Keep its slot occupied until
            # bounded HTTP returns, so repeated timeout requests never accumulate work.
            future.cancel()
        if not future.done():
            return
        self.pending = None
        if pending['expired']:
            return
        try:
            self.answer(question, future.result())
        except Exception as exc:
            self.answer(question, fallback_decision(question['text'], type(exc).__name__))

    def destroy_node(self):
        self.worker.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main():
    spin(TaskScene)
