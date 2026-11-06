#!/usr/bin/env python3
"""Read-only numbered navigation RULE check; historical filename retained.

No Qwen request, ROS publisher, microphone, serial port or hardware command.
  python3 tools/check_semantic_navigation.py
  python3 tools/check_semantic_navigation.py --text '你现在在8号点位，车头朝左，请导航前往至4号点位。'
Exit codes: 0 = explicit points and heading parsed; 1 = context/clarification;
2 = invalid command. This check never confirms a physical vehicle position.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
import time

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE/'src'/'rescue_vision'))
from rescue_vision.task_interaction import parse_task_text


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--text',default='你现在在8号点位，车头朝左，请导航前往至4号点位。')
    parser.add_argument('--config',help=argparse.SUPPRESS)  # Former CLI only; never read a model config.
    args=parser.parse_args(argv)
    started=time.monotonic()
    report=dict(read_only=True,hardware_commands_sent=False,model_called=False,
        backend='explicit_navigation_rules',navigation_parser='rules',original_text=args.text,
        status='error',structured=None,physical_position_verified=False)
    try:
        command=parse_task_text(args.text)
        report['structured']={k:v for k,v in command.items() if k!='text'}
        if command['action']!='navigate_point':
            report.update(status='clarification',reason='not_an_explicit_numbered_navigation_command')
            code=1
        elif command.get('start_point') is None or command.get('start_heading_deg') is None:
            report.update(status='clarification',reason='confirmed_vehicle_point_and_heading_context_required')
            code=1
        else:
            report.update(status='ready',reason='rules_parsed_no_hardware_command')
            code=0
    except (TypeError,ValueError) as exc:
        report['reason']=str(exc)
        code=2
    report['elapsed_s']=round(time.monotonic()-started,6)
    print(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
    return code


if __name__=='__main__':
    raise SystemExit(main())
