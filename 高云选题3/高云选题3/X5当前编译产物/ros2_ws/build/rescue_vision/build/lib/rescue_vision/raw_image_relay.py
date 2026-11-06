"""Demand pacing for an unchanged serialized camera image, without a queue."""
import math


class RawImageRelay:
    def __init__(self, source_topic, output_topic, max_age_s=3.0):
        self.source_topic, self.output_topic = source_topic, output_topic
        self.max_age_s = max_age_s
        self.rate = 0.0
        self.expires = 0.0
        self.next_due = None
        self.last_stamp = None

    def update(self, report, now_ros, now_monotonic):
        """A fresh road observer reports its existing analysis frequency."""
        try:
            stamp, rate = float(report['stamp']), float(report['process_fps_limit'])
            valid = (all(math.isfinite(x) for x in (stamp, rate, now_ros, now_monotonic))
                and 0 <= now_ros-stamp <= self.max_age_s and 1 <= rate <= 15
                and report.get('source_topic') == self.source_topic
                and report.get('raw_input_topic') == self.output_topic
                and report.get('process_on_receive') is True
                and report.get('raw_subscription_active') is True)
        except (KeyError, TypeError, ValueError, AttributeError):
            valid = False
        if not valid:
            self.rate, self.expires, self.next_due = 0.0, 0.0, None
            return
        if self.last_stamp is not None and stamp <= self.last_stamp and now_monotonic <= self.expires:
            return
        if rate != self.rate or now_monotonic > self.expires:
            self.next_due = None
        self.last_stamp = stamp
        self.rate = rate
        # A report delayed in DDS has already consumed part of its lease.
        self.expires = now_monotonic + self.max_age_s - (now_ros-stamp)

    def due(self, now, subscribers):
        if not subscribers or not self.rate or now > self.expires:
            self.next_due = None
            return False
        if self.next_due is not None and now < self.next_due:
            return False
        period = 1.0/self.rate
        # Keep the cadence across 30-Hz camera ticks; never catch up a backlog.
        self.next_due = (now+period if self.next_due is None or now-self.next_due >= period
                         else self.next_due+period)
        return True
