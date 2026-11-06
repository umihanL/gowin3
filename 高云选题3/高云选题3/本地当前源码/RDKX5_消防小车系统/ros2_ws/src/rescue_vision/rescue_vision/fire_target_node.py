"""Publish stable flame IDs and explicitly selected per-frame observations."""
import json
import time

from .node_common import Node
from std_msgs.msg import String
from rescue_interfaces.msg import DetectionArray

from .node_common import IMAGE_QOS, param, age, publish_json, spin
from .fire_targets import FireTargetRegistry


class FireTargets(Node):
    def __init__(self):
        super().__init__('fire_targets')
        self.registry = FireTargetRegistry()
        self.firefighter_registry = FireTargetRegistry(target_class='firefighter')
        self.mission_received = None
        self.mission = {}
        self.last_emitted = {}
        self.targets_pub = self.create_publisher(String, '/fire/targets', 10)
        self.firefighter_targets_pub = self.create_publisher(String, '/firefighter/targets', 10)
        self.selection_pub = self.create_publisher(String, '/fire/selection', 32)
        self.ack_pub = self.create_publisher(String, '/fire/selection_ack', 10)
        self.create_subscription(DetectionArray, param(self, 'detections_topic', '/perception/detections'),
                                 self.receive, IMAGE_QOS)
        self.create_subscription(String, '/mission/state', self.mode, 10)
        self.create_subscription(String, '/fire/select', self.select, 10)
        self.create_timer(.1, self.tick)

    def mission_ok(self, now):
        return (self.mission_received is not None and 0 <= now-self.mission_received <= .5
                and (self.mission.get('mode') in ('inspect', 'fire') or
                    self.mission.get('extension_enabled') is True and self.mission.get('extension_task') in ('vision','dialogue','semantic'))
                and self.mission.get('traffic_active', False) is False)

    def mode(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('Mission must be an object')
        except (ValueError, TypeError):
            value = {}
        self.mission, self.mission_received = value, time.monotonic()
        self.tick()

    def emit(self, now):
        targets, selected = self.registry.snapshot(now)
        firefighter_targets, _ = self.firefighter_registry.snapshot(now)
        for topic, publisher, document in (('targets', self.targets_pub, targets),
                                            ('selection', self.selection_pub, selected),
                                            ('firefighter_targets', self.firefighter_targets_pub, firefighter_targets)):
            # Idle snapshots may otherwise alternate expire()/mission reasons
            # for every camera callback. Report the primary permission gate.
            if not self.mission_ok(now):
                document['reason'] = ('mission_timeout' if self.mission_received is None
                    or not 0 <= now-self.mission_received <= .5 else
                    'traffic_active' if self.mission.get('traffic_active') else 'mission_not_inspect_or_fire')
            signature = (document['valid'], document.get('reason'), self.mission.get('mode'),
                self.mission.get('traffic_active'), document['registry_session'],
                document['selection_generation'], document.get('target_id'),
                document['coordinate_frame'], tuple(document['image_size_wh']))
            if document['valid']:
                signature += (document['capture_stamp_ns'], document['sequence'])
            previous = self.last_emitted.get(topic)
            if previous is not None and previous[0] == signature:
                # A valid exposure is sent once. Invalid snapshots get only a
                # 5 Hz heartbeat, regardless of upstream callback frequency.
                if document['valid'] or 0 <= now-previous[1] < .2-1e-9:
                    continue
            self.last_emitted[topic] = (signature, now)
            publish_json(publisher, document)

    def tick(self):
        now = time.monotonic()
        self.registry.set_enabled(self.mission_ok(now))
        self.firefighter_registry.set_enabled(self.mission_ok(now))
        self.emit(now)

    def receive(self, msg):
        now = time.monotonic()
        self.registry.set_enabled(self.mission_ok(now))
        self.firefighter_registry.set_enabled(self.mission_ok(now))
        if not 0 <= age(self, msg.header.stamp) <= self.registry.max_age_s:
            self.registry.clear('detection_capture_expired')
            self.firefighter_registry.clear('detection_capture_expired')
            self.emit(now)
            return
        items = [dict(class_id=item.class_id, confidence=float(item.confidence),
            bbox_xyxy=[float(value) for value in item.bbox_xyxy]) for item in msg.detections]
        for registry in (self.registry, self.firefighter_registry):
            registry.update(items,
                stamp_ns=int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec),
                sequence=int(msg.sequence), frame_id=msg.header.frame_id,
                size=(int(msg.width), int(msg.height)), now=now, available=msg.detector_available)
        self.emit(now)

    def select(self, msg):
        now = time.monotonic()
        self.registry.set_enabled(self.mission_ok(now))
        try:
            command = json.loads(msg.data)
        except (ValueError, TypeError):
            command = {}
        accepted, reason = self.registry.select(command, now=now,
            ros_now=self.get_clock().now().nanoseconds/1e9)
        publish_json(self.ack_pub, dict(accepted=accepted, reason=reason,
            command_id=command.get('command_id') if isinstance(command, dict) else None,
            target_id=self.registry.selected, selection_generation=self.registry.generation,
            registry_session=self.registry.session))
        self.emit(now)


def main():
    spin(FireTargets)
