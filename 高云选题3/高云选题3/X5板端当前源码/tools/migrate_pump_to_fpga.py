"""Preview/apply only the local pump configuration migration; never drive I/O."""
import argparse
from copy import deepcopy
from datetime import datetime
import json
import os
from pathlib import Path


def migrated(document):
    if not isinstance(document, dict) or not isinstance(document.get('servo'), dict):
        raise ValueError('Expected an actuator I/O configuration with a servo object')
    result = deepcopy(document)
    current = result.get('pump', {})
    if not isinstance(current, dict):
        raise ValueError('Expected a pump configuration object')
    if (current.get('backend') == 'fpga_uart' and current.get('pin') == 'B13'
            and current.get('active_high') is True):
        from sys import path
        path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/rescue_vision'))
        from rescue_vision.fpga_pump_io import validate_pump
        validate_pump(current)
        return result
    result['pump'] = dict(backend='fpga_uart', pin='B13', active_high=True,
                          max_duration_ms=1000, feedback_timeout_s=.6, enabled=False)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--apply', action='store_true')
    args=parser.parse_args()
    target=args.config.resolve(strict=True)
    original=target.read_bytes()
    data=json.loads(original.decode('utf-8-sig'))
    result=migrated(data)
    changed=result != data
    backup=None
    if args.apply and changed:
        backup=target.with_name(target.name+'.before-fpga-pump-'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        with backup.open('xb') as handle:handle.write(original)
        temporary=target.with_name(target.name+'.pump-migration.tmp')
        with temporary.open('x',encoding='utf-8') as handle:
            json.dump(result,handle,ensure_ascii=False,indent=2);handle.write('\n')
        temporary.chmod(target.stat().st_mode)
        os.replace(temporary,target)
    print(json.dumps(dict(config=str(target),changed=changed,applied=args.apply and changed,
        pump=result['pump'],backup=str(backup) if backup else None,hardware_access=False,
        servo_unchanged=result['servo']==data['servo']),ensure_ascii=False))


if __name__=='__main__':main()
