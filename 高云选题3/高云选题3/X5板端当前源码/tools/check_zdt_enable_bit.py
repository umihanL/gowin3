#!/usr/bin/env python3
"""Unloaded M2 enable-state experiment. No position, velocity or settings writes."""
import argparse
import datetime
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision import zdt_xs_protocol as wire
from rescue_vision.zdt_xs_transport import ZDTSerialTransport


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--unloaded',action='store_true')
    parser.add_argument('--report',type=Path,required=True)
    args=parser.parse_args()
    if args.execute and not args.unloaded:
        parser.error('--unloaded is required: this experiment releases torque')
    report=dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        motion_commands_sent=False,configuration_modified=False,transitions=[],samples=[],error=None)
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(report),encoding='utf-8')
    link=None;changed=False;origin=None
    def read(command):
        r=link.exchange(wire.read_request(1,command))
        return r
    def sample(label):
        options=read(wire.Command.READ_OPTIONS)
        state=read(wire.Command.READ_STATE).value
        pos=wire.position_degrees(read(wire.Command.READ_POSITION).value,'emm')
        speed=wire.speed_rpm(read(wire.Command.READ_SPEED).value,'emm')
        error=wire.error_degrees(read(wire.Command.READ_POSITION_ERROR).value,'emm')
        value=dict(label=label,options_hex=options.data.hex(' '),state=state,position_deg=pos,speed_rpm=speed,error_deg=error)
        report['samples'].append(value)
        if state.get('stalled') or state.get('stall_protection') or state.get('left_limit') or state.get('right_limit'):
            raise RuntimeError('driver_fault_or_limit')
        if speed!=0 or abs(error)>.2 or (origin is not None and abs(pos-origin)>1.7):
            raise RuntimeError('unexpected_position_or_speed')
        return value
    try:
        if not args.execute:
            report['note']='Offline: no port opened; execute requires explicit unloaded confirmation.'
        else:
            link=ZDTSerialTransport('/dev/ttyS3',baudrate=115200,timeout_s=.2)
            version=read(wire.Command.READ_VERSION).value
            config=read(wire.Command.READ_CONFIG).value
            report.update(version=version,config=config)
            if (version.get('firmware_raw'),version.get('hardware_raw'),config.get('mode'))!=(206,0x2314,'emm'):
                raise RuntimeError('unexpected_device')
            first=sample('before')
            if first['state'].get('enabled') is not True:
                raise RuntimeError('expected_already_enabled_motor')
            origin=first['position_deg']
            for enabled in (False,True):
                changed=True
                frame=wire.enable_request(1,enabled)
                entry=dict(enabled=enabled,request_hex=frame.hex(' '));report['transitions'].append(entry)
                ack=link.exchange(frame);entry['ack']=ack.ok
                time.sleep(.2)
                observed=sample('enabled' if enabled else 'disabled')
                if observed['state'].get('enabled') is not enabled:
                    raise RuntimeError('enable_readback_mismatch')
            report['experiment_completed']=True
            report['note']='Correlation experiment only; it does not define unknown option bits or authorize motion.'
    except (Exception,KeyboardInterrupt) as exc:
        report['error']=type(exc).__name__+': '+str(exc)
        if changed and link is not None:
            try:
                link.exchange(wire.enable_request(1,False))
                report['fault_disabled']=True
            except Exception as clean:report['cleanup_error']=str(clean)
    finally:
        if link is not None:link.close()
        args.report.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2))
    return 1 if report['error'] else 0


if __name__=='__main__':raise SystemExit(main())
