"""Replay a recorded synthetic-image report. No ROS, microphone or hardware I/O."""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision.telemetry import DashboardServer, TelemetryStore, runtime_token


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', type=int, default=8089)
    ap.add_argument('--token-file', default='~/.local/state/rescue_robot/dashboard_demo.token')
    args = ap.parse_args()
    recorded = json.loads((ROOT/'reports/scene_demo.json').read_text(encoding='utf-8-sig'))
    buildings = json.loads((ROOT/'config/buildings.json').read_text(encoding='utf-8-sig'))
    store = TelemetryStore([b['building_id'] for b in buildings['buildings']], demo=True)
    server = DashboardServer(store, ROOT/'web', runtime_token(args.token_file), port=args.port)
    report = recorded['frames'][-1]['report']
    jpg = None
    try:
        import cv2
        import numpy as np
        source = ROOT/'reports/recorded_scene_image.png'
        if not source.is_file():source = Path(recorded['source']['file'])
        if source.is_file():
            image = cv2.imdecode(np.fromfile(source, np.uint8), cv2.IMREAD_COLOR)
            image = cv2.resize(image, (640, 480))
            ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 65])
            jpg = encoded.tobytes() if ok else None
    except ImportError:
        pass
    server.start()
    print(f'Recorded demonstration only: http://127.0.0.1:{server.port}/', flush=True)
    print('Read local token file: '+str(Path(args.token_file).expanduser()), flush=True)
    try:
        while True:
            store.update('scene', report)
            store.update('fpga', {'connected': False, 'status_fresh': False, 'reason': 'demo_no_hardware'})
            store.update('mission', {'mode': 'inspect', 'status': 'recorded_demo_no_actuation'})
            store.update('navigation', {'status': '场景回放 / 导航未运行', 'can_execute': False})
            store.update('traffic', {'state': '演示 / 无灯珠输入'})
            if jpg:
                store.set_frame(jpg)
            for cmd in store.drain():
                store.update('voice', {'status': 'demo_request_recorded', 'text': '演示已收到：'+cmd.get('text', cmd['action'])+'。未连接硬件，不会执行。', 'command_id': cmd['command_id']})
            time.sleep(.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.close()


if __name__ == '__main__':
    main()
