"""Two independent unloaded-motor bench workers with bounded keepalive queues.

motor1 owns /dev/ttyS1 (PD42, PIN8 TX/PIN10 RX); motor2 owns /dev/ttyS3
(ZDT_XS, PIN3 TX/PIN5 RX). No UART opens at startup. A blocked transaction on
one motor cannot delay the other motor's Stop. Each worker exclusively owns its
core/transport and verifies its motor identity before processing any request.
"""
from collections import deque
from copy import deepcopy
import json
import threading
import time

from .node_common import Node
from std_msgs.msg import String
from .node_common import publish_json, spin
from .pd42_test import MOTOR_PROFILES, PD42Bench, bench_guard, validate_request


class MotorWorker:
    """One fixed motor, one action/stop slot and a separate latest heartbeat.

    No callback or other motor holds a lock while serial I/O is in progress.
    """
    def __init__(self, motor_id, core, now, get_evidence, on_error=lambda message: None):
        if motor_id not in MOTOR_PROFILES:
            raise ValueError('invalid_motor_test_motor_id')
        self.motor_id, self.core = motor_id, core
        self.now, self.get_evidence, self.on_error = now, get_evidence, on_error
        self.lock = threading.Lock()
        self.wake, self.stopping = threading.Event(), threading.Event()
        self.pending = self.keepalive = self.executing = None
        self.view = self.core.snapshot(self.now())
        self.receipts = deque(maxlen=24)
        self.notice = self.last_core_receipt = self.last_keepalive = None
        self.worker_error = ''
        self.core.guard_probe = self.current_guard
        self.thread = threading.Thread(target=self.run, name='bench-uart-'+motor_id, daemon=True)

    def start(self):
        self.thread.start()

    def current_guard(self):
        with self.lock:
            stopping = self.stopping.is_set() or self.pending is not None and self.pending['action'] in ('stop', 'disconnect')
        if stopping:
            return 'motor_test_stop_pending'
        return bench_guard(self.get_evidence(), self.now())

    def _receipt(self, request, result, reason):
        item = dict(request_id=request.get('request_id'), action=request.get('action'),
                    motor_id=self.motor_id, result=result, reason=reason, stamp=self.now())
        if request.get('action') == 'keepalive':
            self.last_keepalive = item
            return
        self.receipts.append(item)
        self.notice = dict(item, expires=time.monotonic()+1.0)

    def reject(self, request, reason):
        with self.lock:
            self._receipt(request, 'rejected', reason)

    def submit(self, request):
        """Nonblocking queue mutation; never opens or writes a serial device."""
        with self.lock:
            if request.get('motor_id') != self.motor_id:
                self._receipt(request, 'rejected', 'motor_test_wrong_worker')
                return False
            if self.stopping.is_set() or not self.thread.is_alive():
                self._receipt(request, 'rejected', 'motor_test_shutting_down')
                return False
            action = request['action']
            if action in ('stop', 'disconnect'):
                if self.pending:
                    self._receipt(self.pending, 'rejected', 'motor_test_superseded_by_stop')
                self.pending, self.keepalive = request, None
                self._receipt(request, 'accepted', 'motor_test_queued_stop')
            elif action == 'keepalive':
                if any(item and item['action'] in ('stop', 'disconnect') for item in (self.pending, self.executing)):
                    self._receipt(request, 'rejected', 'motor_test_stop_pending')
                    return False
                self.keepalive = request
            elif self.pending is not None or self.executing is not None:
                self._receipt(request, 'rejected', 'motor_test_worker_busy')
                return False
            else:
                self.pending = request
                self._receipt(request, 'accepted', 'motor_test_queued')
        self.wake.set()
        return True

    def _next(self):
        with self.lock:
            if self.pending is not None:
                request, self.pending = self.pending, None
            elif self.keepalive is not None:
                request, self.keepalive = self.keepalive, None
            else:
                request = None
            self.executing = request
            return request

    def _save(self, snapshot, request=None):
        with self.lock:
            current = (snapshot.get('last_request_id'), snapshot.get('last_motor_id', self.motor_id),
                       snapshot.get('last_action'), snapshot.get('last_result'), snapshot.get('reason'))
            if self.last_core_receipt != current and current[0] is not None:
                item = dict(request_id=current[0], motor_id=self.motor_id, action=current[2],
                            result=current[3], reason=current[4], stamp=self.now())
                if current[2] == 'keepalive':
                    self.last_keepalive = item
                else:
                    self.receipts.append(item)
                self.last_core_receipt = current
            if request is not None and self.notice and request['request_id'] == self.notice['request_id']:
                self.notice = None
            self.view, self.executing = snapshot, None

    def run(self):
        poll_after = 0.
        try:
            while not self.stopping.is_set():
                self.wake.wait(.05)
                self.wake.clear()
                request = self._next()
                if request is not None:
                    # Recheck TTL after queue delay; the core binds run_id.
                    self.core.handle(request, self.now(), self.get_evidence())
                if time.monotonic() >= poll_after:
                    self.core.tick(self.now(), self.get_evidence())
                    poll_after = time.monotonic()+.15
                self._save(self.core.snapshot(self.now()), request)
                with self.lock:
                    waiting = self.pending is not None or self.keepalive is not None
                if waiting:
                    self.wake.set()
        except Exception as exc:
            self.worker_error = str(exc)
            self.on_error(self.motor_id+' worker stopped: '+self.worker_error)
        finally:
            try:
                self.core.close()
            except Exception as exc:
                self.worker_error = 'motor_disable_unconfirmed: '+str(exc)
                self.on_error(self.motor_id+' '+self.worker_error)
            self._save(self.core.snapshot(self.now()))

    def snapshot(self):
        with self.lock:
            value = deepcopy(self.view)
            value['motor_id'] = self.motor_id
            value['busy'] = value.get('busy', False) or self.executing is not None or self.pending is not None
            value['recent_requests'] = list(self.receipts)
            if self.notice and time.monotonic() < self.notice['expires']:
                value.update(last_request_id=self.notice['request_id'], last_action=self.notice['action'],
                             last_motor_id=self.motor_id, last_result=self.notice['result'], reason=self.notice['reason'])
            value['last_keepalive'] = deepcopy(self.last_keepalive)
        now = self.now()
        age = now-value.get('stamp', now)
        alive = self.thread.is_alive() and not self.stopping.is_set()
        if not alive or age < -.05 or age > .7:
            value.update(feedback_fresh=False, can_enable=False, can_move=False, can_run=False,
                         position_counts=None, position_deg=None, speed_rpm=None,
                         enabled=None, run_state=None, voltage_v=None)
        value.update(stamp=now, worker_snapshot_age_s=age, worker_alive=alive,
                     worker_error=self.worker_error)
        return value

    def request_close(self):
        self.stopping.set()
        with self.lock:
            if self.pending:
                self._receipt(self.pending, 'rejected', 'motor_test_shutting_down')
            self.pending = self.keepalive = None
        self.wake.set()

    def join(self, timeout=2.0):
        self.thread.join(timeout)
        if self.thread.is_alive():
            self.worker_error = 'motor_disable_unconfirmed_worker_still_running'
            self.on_error(self.motor_id+' '+self.worker_error)
            return False
        return True


class PD42Test(Node):
    def __init__(self):
        super().__init__('pd42_test')
        self.lock = threading.Lock()
        self.evidence = {}
        self.invalid_requests = deque(maxlen=8)
        self.workers = {motor: MotorWorker(motor, PD42Bench(motor_id=motor), self.now, self.get_evidence,
            self.get_logger().error) for motor in MOTOR_PROFILES}
        self.publisher = self.create_publisher(String, '/motor_test/state', 10)
        self.create_subscription(String, '/motor_test/request', self.request, 10)
        for key, topic in (('mission', '/mission/state'), ('actuators', '/actuators/state'),
                           ('fpga_link', '/fpga/link')):
            self.create_subscription(String, topic, lambda msg, k=key: self.receive(k, msg), 10)
        for worker in self.workers.values():
            worker.start()
        self.create_timer(.2, self.publish_state)
        self.get_logger().info('Two independent bench workers: motor1 PD42 PIN8/10 ttyS1; motor2 ZDT_XS PIN3/5 ttyS3; UARTs closed until Connect')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def get_evidence(self):
        with self.lock:
            return deepcopy(self.evidence)

    def receive(self, key, msg):
        try:
            if len(msg.data) > 32000:
                raise ValueError('oversize')
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('not_object')
        except (ValueError, TypeError):
            value = {}
        with self.lock:
            self.evidence[key] = value
        for worker in self.workers.values():
            worker.wake.set()

    def request(self, msg):
        request = {}
        try:
            if len(msg.data) > 2048:
                raise ValueError('motor_test_request_too_large')
            request = json.loads(msg.data)
            request = validate_request(request, self.now())
        except (ValueError, TypeError) as exc:
            if not isinstance(request, dict):
                request = {}
            worker = self.workers.get(request.get('motor_id')) if isinstance(request.get('motor_id'), str) else None
            if worker:
                worker.reject(request, str(exc))
            else:
                with self.lock:
                    self.invalid_requests.append(dict(request_id=request.get('request_id'),
                        action=request.get('action'), motor_id=request.get('motor_id'),
                        result='rejected', reason=str(exc), stamp=self.now()))
            return
        self.workers[request['motor_id']].submit(request)

    def state(self):
        motors = {motor: worker.snapshot() for motor, worker in self.workers.items()}
        with self.lock:
            receipts = list(self.invalid_requests)
        for snapshot in motors.values():
            receipts.extend(snapshot['recent_requests'])
        receipts.sort(key=lambda item: item['stamp'])
        # Compatibility fields describe motor1 only; use motors[motor_id].
        value = dict(motors['motor1'])
        value.update(stamp=self.now(), motors=motors, recent_requests=receipts[-32:],
                     independent_workers=True, motor_count=2,
                     workers_healthy=all(s['worker_alive'] for s in motors.values()),
                     synthetic=False, bench_only=True)
        return value

    def publish_state(self):
        publish_json(self.publisher, self.state())

    def destroy_node(self):
        # Signal both before waiting for either: no cross-motor stop blocking.
        for worker in self.workers.values():
            worker.request_close()
        for worker in self.workers.values():
            worker.join(2.0)
        try:
            publish_json(self.publisher, self.state())
        except Exception:
            # ExternalShutdownException can mean the ROS context is already
            # invalid. Both workers have already been stopped; a final status
            # publication must not prevent normal ROS node/resource teardown.
            pass
        return super().destroy_node()


def main():
    spin(PD42Test)
