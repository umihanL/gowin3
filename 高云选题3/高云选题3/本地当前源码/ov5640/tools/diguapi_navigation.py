"""Thread-safe navigation tool adapter for a DiGuaPi AI application.

Pass an opened pyserial-compatible port. Keep this object alive while driving.
All serial access must go through this adapter while its worker is running.
"""
import queue
import threading
import time
from host_serial_client import Client


class VoiceNavigator:
    def __init__(self, port):
        self.client = Client(port)
        self.events = queue.Queue()
        self._lock = threading.Lock()
        self._stop_worker = threading.Event()
        self._cancel_plan = threading.Event()
        self._worker = None
        self._worker_error = None
        self._active = False
        self._selected = None
        self._task = None

    def __enter__(self):
        if self._worker is not None:
            raise RuntimeError('adapter already started')
        self._worker = threading.Thread(target=self._maintain_link, daemon=True)
        self._worker.start()
        return self

    def __exit__(self, *exc):
        try:
            if self._active:
                self.stop()
        finally:
            self._stop_worker.set()
            if self._worker is not None:
                self._worker.join(timeout=2)

    def _collect_events(self, incoming=()):
        for event in [*self.client.events, *incoming]:
            self.events.put(event)
            if event['command'] in (0xf0, 0xf1, 0xf2) and event['data'][0] == self._task:
                self._active = False
        self.client.events.clear()

    def _call(self, method, *args):
        if self._worker is None:
            raise RuntimeError('use VoiceNavigator inside a with block')
        if self._worker_error is not None:
            raise RuntimeError('serial heartbeat worker failed') from self._worker_error
        with self._lock:
            result = method(*args)
            self._collect_events()
            return result

    def _maintain_link(self):
        next_heartbeat = time.monotonic()
        try:
            while not self._stop_worker.wait(0.02):
                with self._lock:
                    if time.monotonic() >= next_heartbeat:
                        self.client.request(1)
                        next_heartbeat = time.monotonic() + 0.5
                        self._collect_events()
                    else:
                        self._collect_events(self.client.poll())
        except Exception as error:
            self._worker_error = error
            self.events.put(dict(command=None, error=str(error)))
            # FPGA's existing 2 s timeout stops the vehicle if the link is lost.

    def select_point(self, point, task=1):
        self._cancel_plan.clear()
        result = self._call(self.client.select_point, point, task)
        self._selected = point
        return result

    def wait_ready(self, timeout=60.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._cancel_plan.is_set():
                raise RuntimeError('navigation planning cancelled')
            status = self._call(self.client.point_status)
            if status['point'] != self._selected:
                raise RuntimeError('destination changed')
            if status['state'] == 2:
                return status
            if status['state'] != 1:
                raise RuntimeError(f'route unavailable: {status}')
            self._cancel_plan.wait(0.1)
        raise TimeoutError('FPGA planning timed out')

    def start(self, task=1):
        if self._cancel_plan.is_set():
            raise RuntimeError('navigation cancelled; select a destination again')
        # Mark active within the same lock so a completion cannot race START.
        with self._lock:
            if self._cancel_plan.is_set():
                raise RuntimeError('navigation cancelled; select a destination again')
            if self._worker_error is not None:
                raise RuntimeError('serial heartbeat worker failed') from self._worker_error
            # Retire buffered notifications from the preceding session, even
            # when an AI application reuses its default task number.
            self._collect_events(self.client.poll())
            while True:
                try:
                    self.events.get_nowait()
                except queue.Empty:
                    break
            result = self.client.start_navigation(task)
            self._task = task
            self._active = True
            self._collect_events()
            return result

    def navigate_to(self, point, task=1, timeout=60.0):
        self.select_point(point, task)
        self.wait_ready(timeout)
        return self.start(task)

    def stop(self):
        self._cancel_plan.set()
        result = self._call(self.client.request, 5)
        self._active = False
        return result

    def resume(self):
        with self._lock:
            if self._worker_error is not None:
                raise RuntimeError('serial heartbeat worker failed') from self._worker_error
            result = self.client.request(7)
            self._active = True
            self._collect_events()
            return result

    def wait_until_done(self, timeout=180.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                event = self.events.get(timeout=min(0.1, max(0.001, deadline-time.monotonic())))
            except queue.Empty:
                continue
            if event['command'] is None or (event['command'] in (0xf0, 0xf1, 0xf2) and event['data'][0] == self._task):
                return event
        raise TimeoutError('navigation has not completed')
