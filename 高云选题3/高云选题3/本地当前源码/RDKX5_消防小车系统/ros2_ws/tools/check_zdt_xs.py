#!/usr/bin/env python3
"""Explicitly read one ZDT_XS UART; never enable, move, stop or change settings."""
import argparse
import datetime
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision import zdt_xs_protocol as zdt
from rescue_vision.zdt_xs_transport import ZDTSerialTransport


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--read-only', action='store_true', required=True)
    p.add_argument('--port', required=True)
    p.add_argument('--address', type=int, default=1)
    p.add_argument('--baudrate', type=int, default=115200)
    p.add_argument('--report', type=Path)
    args = p.parse_args(argv)
    report = dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  protocol='zdt_xs', port=args.port, address=args.address, baudrate=args.baudrate,
                  read_only=True, motion_commands_sent=False, enable_commands_sent=False,
                  configuration_modified=False, readings={}, error=None)
    try:
        with ZDTSerialTransport(args.port, baudrate=args.baudrate, timeout_s=.3) as link:
            for cmd in (zdt.Command.READ_VERSION, zdt.Command.READ_CONFIG, zdt.Command.READ_OPTIONS,
                        zdt.Command.READ_VOLTAGE, zdt.Command.READ_POSITION,
                        zdt.Command.READ_TARGET_POSITION, zdt.Command.READ_REALTIME_TARGET_POSITION,
                        zdt.Command.READ_SPEED, zdt.Command.READ_POSITION_ERROR, zdt.Command.READ_STATE):
                request = zdt.read_request(args.address, cmd)
                response = link.exchange(request)
                report['readings'][cmd.name] = dict(request_hex=request.hex(' '),
                    response_data_hex=response.data.hex(' '), decoded=response.value)
        config = report['readings']['READ_CONFIG']['decoded']
        version = report['readings']['READ_VERSION']['decoded']
        options = zdt.normalize_options(version, config, report['readings']['READ_OPTIONS']['decoded'])
        # Keep the untouched base decoder output and raw bytes in readings.
        report['normalized_options'] = options
        mode = config['mode']
        report['firmware_mode'] = mode
        report['position_deg'] = zdt.position_degrees(report['readings']['READ_POSITION']['decoded'], mode)
        report['target_position_deg'] = zdt.position_degrees(
            report['readings']['READ_TARGET_POSITION']['decoded'], mode)
        report['realtime_target_position_deg'] = zdt.position_degrees(
            report['readings']['READ_REALTIME_TARGET_POSITION']['decoded'], mode)
        report['position_error_deg'] = zdt.error_degrees(
            report['readings']['READ_POSITION_ERROR']['decoded'], mode)
        report['speed_rpm'] = zdt.speed_rpm(report['readings']['READ_SPEED']['decoded'], mode)
        try:
            report['motion_parameters'] = zdt.verified_motion_parameters(config, options)
            report['motion_configuration_valid'] = True
        except zdt.ProtocolError as exc:
            report['motion_configuration_valid'] = False
            report['motion_configuration_reason'] = str(exc)
        report['note'] = 'Communication readback only; movement, mechanical zero, limits and fire aiming remain unverified.'
    except Exception as exc:
        report['error'] = str(exc)
    text = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(text + '\n', encoding='utf-8')
    print(text)
    return 1 if report['error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
