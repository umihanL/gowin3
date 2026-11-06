"""A bounded handoff: busy consumers receive the newest value, never a backlog."""
import threading


class LatestSlot:
    def __init__(self):
        self.condition = threading.Condition()
        self.value = None
        self.version = 0
        self.closed = False

    def put(self, value):
        with self.condition:
            if self.closed:
                return False
            self.value = value
            self.version += 1
            self.condition.notify_all()
            return True

    def take_after(self, version, timeout=.2):
        with self.condition:
            self.condition.wait_for(lambda: self.closed or self.version > version, timeout)
            if self.closed or self.version <= version:
                return None
            return self.version, self.value

    def close(self):
        with self.condition:
            self.closed = True
            self.value = None
            self.condition.notify_all()
