"""Fresh task permission for YOLO; camera/preview operation is independent."""
import math
import threading


class VisionTaskGate:
    def __init__(self, enabled=False):
        self.enabled = enabled
        self.lock = threading.Lock()
        self.signature = None
        self.received = None
        self.transition_stamp = None

    def update(self, mission, now_ros, now_mono):
        stamp = mission.get('stamp')
        valid = isinstance(stamp, (int, float)) and math.isfinite(stamp) and 0 <= now_ros-stamp <= .5
        mode = ('extension_yolo' if mission.get('extension_enabled') is True and
                mission.get('extension_task') in ('vision','dialogue','semantic') else mission.get('mode'))
        if mission.get('extension_enabled') is True and mission.get('extension_task') not in ('vision','dialogue','semantic'):
            mode='extension_no_yolo'
        signature = (mode, mission.get('traffic_active')) if valid else None
        with self.lock:
            if signature != self.signature:
                self.transition_stamp = stamp if valid else None
            self.signature = signature
            self.received = now_mono if valid else None

    def allowed(self, capture_ros, now_mono):
        if not self.enabled:
            return True
        with self.lock:
            return bool(self.signature is not None and self.signature[0] in ('inspect','fire','extension_yolo')
                        and self.signature[1] is False and self.received is not None
                        and 0 <= now_mono-self.received <= .5 and self.transition_stamp is not None
                        and capture_ros >= self.transition_stamp)
