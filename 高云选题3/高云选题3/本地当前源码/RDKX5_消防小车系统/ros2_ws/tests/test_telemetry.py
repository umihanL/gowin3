import http.client
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision.telemetry import DashboardServer, TelemetryStore, runtime_token, validate_command


def command(**changes):
    data = dict(command_id='test-command-001', source='web', action='navigate',
                building_id='C', text='去3号楼观察', issued_at=time.time(), ttl_s=10)
    data.update(changes)
    return data


class HTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.store = TelemetryStore(['C'])
        cls.token = 'test-only-'+'x'*32
        cls.server = DashboardServer(cls.store, ROOT/'web', cls.token, port=0)
        cls.server.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.close()

    def request(self, method, path, data=None, headers=None, token=True):
        h = {'Content-Type': 'application/json'}
        if token:
            h['X-Rescue-Token'] = self.token
        h.update(headers or {})
        conn = http.client.HTTPConnection('127.0.0.1', self.server.port, timeout=3)
        conn.request(method, path, json.dumps(data).encode() if data is not None else None, h)
        response = conn.getresponse()
        result = response.status, response.read(), dict(response.getheaders())
        conn.close()
        return result

    def test_auth_required_for_snapshot_and_frame(self):
        for path in ('/api/state', '/api/frame'):
            self.assertEqual(self.request('GET', path, token=False)[0], 401)
        self.assertEqual(self.request('GET', '/api/state', headers={'X-Rescue-Token': 'wrong'})[0], 401)

    def test_token_query_is_rejected(self):
        self.assertEqual(self.request('GET', '/api/state?token=secret')[0], 400)

    def test_origin_and_host_rejected(self):
        self.assertEqual(self.request('POST', '/api/command', command(), {'Origin': 'http://evil.example'})[0], 403)
        self.assertEqual(self.request('GET', '/api/state', headers={'Host': 'evil.example'})[0], 403)
        self.assertEqual(self.request('POST', '/api/command', command(), {'Origin': 'null'})[0], 403)

    def test_accepted_once_via_queue(self):
        origin = 'http://127.0.0.1:'+str(self.server.port)
        data = command(command_id='accepted-once-123')
        first = self.request('POST', '/api/command', data, {'Origin': origin})
        self.assertEqual(first[0], 202)
        self.assertEqual(json.loads(first[1])['status'], 'queued_not_executed')
        again = self.request('POST', '/api/command', data, {'Origin': origin})
        self.assertEqual(again[0], 200)
        self.assertTrue(json.loads(again[1])['duplicate'])
        self.assertEqual([c['command_id'] for c in self.store.drain()], ['accepted-once-123'])

    def test_ttl_cap_and_nonfinite(self):
        for values in ({'ttl_s': 11}, {'ttl_s': 40}, {'ttl_s': True},
                       {'issued_at': float('nan')}, {'issued_at': float('inf')}, {'issued_at': True}):
            self.assertEqual(self.request('POST', '/api/command', command(**values))[0], 400)

    def test_client_clock_does_not_control_ros_deadline(self):
        list(self.store.drain())
        for index, client_stamp in enumerate((0., time.time()-3600, time.time()+86400, 1e30)):
            before = time.time()
            response = self.request('POST', '/api/command', command(
                command_id=f'clock-skew-command-{index}', issued_at=client_stamp, ttl_s=10))
            self.assertEqual(response[0], 202)
            commands = list(self.store.drain())
            self.assertEqual(len(commands), 1)
            issued = commands[0]['issued_at']
            self.assertGreaterEqual(issued, before)
            self.assertLessEqual(issued, time.time())
            self.assertEqual(commands[0]['ttl_s'], 10)
            with self.assertRaisesRegex(ValueError, 'expired or future'):
                validate_command(commands[0], issued+10.01, {'C'})

    def test_schema_and_whitelist(self):
        for values in ({'action': 'shell'}, {'source': 'fpga'}, {'building_id': '../../etc'},
                       {'extra': 'rm -rf'}, {'text': 'a'*81}, {'text': 'go\nnow'},
                       {'command_id': '<script>'}):
            self.assertEqual(self.request('POST', '/api/command', command(**values))[0], 400)

    def test_static_routes_and_no_token_leak(self):
        status, data, headers = self.request('GET', '/', token=False)
        self.assertEqual(status, 200)
        self.assertNotIn(self.token.encode(), data)
        self.assertEqual(headers['X-Frame-Options'], 'DENY')
        for path in ('/../config/telemetry.json', '/config/telemetry.json', '/dashboard.token'):
            self.assertEqual(self.request('GET', path)[0], 404)

    def test_snapshot_unknown_not_connected(self):
        status, data, _ = self.request('GET', '/api/state')
        self.assertEqual(status, 200)
        self.assertNotIn('fpga', json.loads(data)['states'])

    def test_oversized_payload(self):
        self.assertEqual(self.request('POST', '/api/command', {'text': 'x'*4096})[0], 400)


class StoreTests(unittest.TestCase):
    def test_queue_expiry_before_executor(self):
        store = TelemetryStore(['C'])
        now = time.time()
        store.enqueue(command(issued_at=now, ttl_s=1))
        with patch('rescue_vision.telemetry.time.time', return_value=now+2):
            self.assertEqual(list(store.drain()), [])

    def test_queue_is_bounded(self):
        store = TelemetryStore(['C'])
        for i in range(32):
            store.enqueue(command(command_id=f'command-{i:04d}'))
        with self.assertRaisesRegex(ValueError, 'queue full'):
            store.enqueue(command(command_id='overflow-command'))

    def test_snapshot_copies_and_stales(self):
        store = TelemetryStore()
        now = time.monotonic()
        data = {'valid': True, 'visible_person_count': 2}
        store.update('scene', data)
        data['visible_person_count'] = 99
        with patch('rescue_vision.telemetry.time.monotonic', return_value=now+5):
            snap = store.snapshot()
        self.assertTrue(snap['states']['scene']['stale'])
        self.assertEqual(snap['states']['scene']['data']['visible_person_count'], 2)

    def test_stale_preview_hidden(self):
        store = TelemetryStore()
        now = time.monotonic()
        store.set_frame(b'\xff\xd8test-jpeg')
        self.assertIsNotNone(store.frame())
        with patch('rescue_vision.telemetry.time.monotonic', return_value=now+5):
            self.assertIsNone(store.frame())

    def test_token_created_and_reused(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'token'
            first = runtime_token(path)
            self.assertGreaterEqual(len(first), 32)
            self.assertEqual(first, runtime_token(path))
            other = runtime_token(Path(tmp)/'other')
            self.assertNotEqual(first, other)


if __name__ == '__main__':
    unittest.main(verbosity=2)
