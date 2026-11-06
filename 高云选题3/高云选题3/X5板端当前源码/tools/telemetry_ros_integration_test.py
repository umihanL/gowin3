"""Actual HTTP -> ROS 2 -> navigation preview -> HTTP integration, no hardware.

Run in the isolated Linux ROS environment after sourcing its built workspace.
"""
from pathlib import Path
import json
import os
import secrets
import signal
import socket
import subprocess
import tempfile
import time
import urllib.error
import urllib.request
import uuid

os.environ['ROS_DOMAIN_ID'] = '179'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]


def main():
    checks = []
    result = {'passed': False, 'real_hardware': False, 'bpu_executed': False,
              'ros_domain_id': 179, 'nodes': ['telemetry', 'interaction', 'navigation'],
              'transport': 'actual localhost HTTP and ROS 2 messages', 'checks': checks}
    processes, streams = [], []
    temporary = tempfile.TemporaryDirectory(prefix='rescue_http_ros_')
    tmp = Path(temporary.name)
    logs = ROOT/'logs/telemetry_ros_integration'
    logs.mkdir(parents=True, exist_ok=True)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    token = secrets.token_urlsafe(32)
    token_path = tmp/'token'
    token_path.write_text(token, encoding='ascii')
    token_path.chmod(0o600)
    config = dict(bind='127.0.0.1', port=port, allowed_hosts=['127.0.0.1'],
                  token_file=str(token_path), preview_hz=1, preview_max_width=640)
    (tmp/'telemetry.json').write_text(json.dumps(config), encoding='utf-8')
    base = f'http://127.0.0.1:{port}'
    rclpy.init()
    node = Node('http_ros_integration_harness')
    commands, answers, missions = [], [], []
    node.create_subscription(String, '/interaction/command', lambda m: commands.append(json.loads(m.data)), 20)
    node.create_subscription(String, '/interaction/result', lambda m: answers.append(json.loads(m.data)), 20)
    node.create_subscription(String, '/mission/request', lambda m: missions.append(json.loads(m.data)), 20)

    def check(name, condition):
        checks.append({'name': name, 'passed': bool(condition)})
        if not condition:
            raise AssertionError(name)
        print('PASS '+name, flush=True)

    def start(exe, params):
        args = ['ros2', 'run', 'rescue_vision', exe, '--ros-args']
        for key, value in params.items():
            args += ['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}']
        stream = (logs/(exe+'.log')).open('w')
        streams.append(stream)
        processes.append(subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))

    def http(path, body=None, authenticate=True):
        headers = {'Content-Type': 'application/json', 'Origin': base}
        if authenticate:
            headers['X-Rescue-Token'] = token
        request = urllib.request.Request(base+path,
            data=None if body is None else json.dumps(body).encode('utf-8'), headers=headers)
        try:
            with urllib.request.urlopen(request, timeout=.8) as response:
                payload = response.read()
                return response.status, json.loads(payload) if payload else None
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read())

    def pump(duration=.1):
        until = time.monotonic()+duration
        while time.monotonic() < until:
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('A ROS node stopped unexpectedly; inspect integration logs')
            rclpy.spin_once(node, timeout_sec=.02)

    def wait(predicate, timeout=8):
        until = time.monotonic()+timeout
        while time.monotonic() < until:
            pump(.05)
            try:
                value = predicate()
                if value:
                    return value
            except (OSError, TimeoutError):
                pass
        raise TimeoutError('Condition not reached within '+str(timeout)+' seconds')

    def snapshot():
        status, body = http('/api/state')
        if status != 200:
            return {}
        return body.get('states', {})

    def command(action='navigate', text='去3号楼观察', issued_at=None):
        return {'command_id': str(uuid.uuid4()), 'source': 'web', 'action': action,
                'text': text, 'issued_at': time.time() if issued_at is None else issued_at, 'ttl_s': 10}

    try:
        start('telemetry', {'telemetry_config': tmp/'telemetry.json', 'buildings_config': ROOT/'config/buildings.json'})
        start('interaction', {'buildings_config': ROOT/'config/buildings.json', 'navigation_execute_enabled': False})
        start('navigation', {'navigation_map': ROOT/'config/navigation_map.json', 'navigation_config': ROOT/'config/navigation.json'})
        wait(lambda: {'telemetry', 'interaction', 'navigation'} <= set(node.get_node_names())
             and node.count_subscribers('/interaction/command') >= 2 and bool(snapshot()), 15)
        check('three_nodes_and_http_ready', True)
        check('http_requires_token', http('/api/state', authenticate=False)[0] == 401)

        navigate = command(issued_at=time.time()-3600)
        sent_at = time.time()
        status, response = http('/api/command', navigate)
        check('http_accepts_clock_skewed_request', status == 202 and response['accepted'])
        wait(lambda: any(a.get('command_id') == navigate['command_id'] and a.get('accepted') for a in answers))
        observed = next(c for c in commands if c['command_id'] == navigate['command_id'])
        check('server_timestamp_reaches_ros', sent_at <= observed['issued_at'] <= time.time()
              and observed['ttl_s'] == 10)

        state = wait(lambda: (s if (s := snapshot()).get('navigation', {}).get('data', {}).get('route')
                     and s.get('interaction', {}).get('data', {}).get('command_id') == navigate['command_id'] else None))
        route_state = state['navigation']['data']
        route = route_state['route']
        check('navigation_route_returns_to_http', route_state['command_id'] == navigate['command_id']
              and route['building_name'] == '春晖楼' and len(route['segments']) > 0)
        check('preview_has_no_motion_permission', route_state['mode'] == 'preview'
              and not route_state['proposal']['ready_to_move'] and not route_state['proposal']['valid'])
        check('interaction_reply_returns_to_http', state.get('interaction', {}).get('data', {}).get('command_id') == navigate['command_id'])
        result['route_summary'] = {'building_id': route['building_id'], 'building_name': route['building_name'],
                                   'segment_count': len(route['segments']), 'mode': route_state['mode']}

        before = len(commands)
        status, response = http('/api/command', navigate)
        pump(.4)
        check('duplicate_http_command_not_republished', status == 200 and response.get('duplicate') and len(commands) == before)

        stop = command('stop', '停车', time.time()+86400)
        before_stop = time.time()
        check('future_client_stamp_does_not_block_stop', http('/api/command', stop)[0] == 202)
        wait(lambda: any(a.get('command_id') == stop['command_id'] and a.get('accepted') for a in answers))
        state = wait(lambda: (s if (s := snapshot()).get('navigation', {}).get('data', {}).get('paused') else None))
        stop_ros = next(c for c in commands if c['command_id'] == stop['command_id'])
        check('stop_enters_task_arbiter_and_pauses_route', any(m.get('mode') == 'idle' for m in missions)
              and state['navigation']['data']['paused'])
        check('future_client_stamp_cannot_extend_server_ttl', before_stop <= stop_ros['issued_at'] <= time.time()
              and stop_ros['issued_at']+stop_ros['ttl_s'] <= time.time()+10)

        invalid = command(text='去不存在的楼观察')
        check('unresolved_text_enters_interaction_for_feedback', http('/api/command', invalid)[0] == 202)
        state = wait(lambda: (s if (s := snapshot()).get('interaction', {}).get('data', {}).get('accepted') is False
                     and '楼栋' in str(s['interaction']['data'].get('reason')) else None))
        check('semantic_rejection_is_visible_via_http', '楼栋' in state['interaction']['data']['reason'])
        check('semantic_rejection_does_not_resume_navigation', state['navigation']['data']['paused'])
        invalid_ttl = command('resume', '继续')
        invalid_ttl['ttl_s'] = 11
        check('http_rejects_ttl_over_ten_seconds', http('/api/command', invalid_ttl)[0] == 400)
        check('no_camera_fpga_or_speech_hardware_nodes_started', not {'stereo_camera', 'fpga_bridge', 'voice'} & set(node.get_node_names()))
        result['passed'] = True
    except Exception as exc:
        result['error'] = repr(exc)
        print('FAIL '+repr(exc), flush=True)
    finally:
        for p in processes:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
        for stream in streams:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()
        temporary.cleanup()
        result['check_count'] = len(checks)
        (ROOT/'reports/telemetry_ros_integration.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
