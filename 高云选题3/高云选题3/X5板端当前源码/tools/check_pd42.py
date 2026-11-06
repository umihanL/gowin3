#!/usr/bin/env python3
"""PD42 configuration check and opt-in read-only native serial diagnostic.

With no flags, validate the supplied dry-run template without opening a port.
--read-only --port opens one explicitly selected PD42 port and reads feedback.
Never enables, disables, homes, clears faults, calibrates, saves or moves a motor.
Do not point this diagnostic at the FPGA's separate UART.
"""
import argparse
import datetime
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision.pd42_actuator_io import validate
from rescue_vision.pd42_protocol import Command, read_request
from rescue_vision.pd42_transport import PD42SerialTransport


READS = (Command.READ_VERSION, Command.READ_VOLTAGE, Command.READ_POSITION,
         Command.READ_POSITION_ERROR, Command.READ_SPEED, Command.READ_STATUS,
         Command.READ_STALL, Command.READ_ENABLED, Command.READ_ARRIVED)


def read_feedback(port, address, baudrate):
    readings = {}
    with PD42SerialTransport(port, baudrate=baudrate, timeout_s=.2) as connection:
        for command in READS:
            response = connection.exchange(read_request(address, command))
            readings[command.name] = dict(command='0x%02X' % command,
                decoded=response.value, raw_data_hex=response.data.hex(' '),
                response_success=response.ok)
    return readings


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/actuator_io.pd42.template.json')
    parser.add_argument('--read-only', action='store_true', help='Explicitly read one connected PD42; otherwise no port opens')
    parser.add_argument('--port', help='PD42 UART only; must not be owned by the live actuator node or FPGA bridge')
    parser.add_argument('--address', type=int, default=1)
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args(argv)
    if args.read_only != bool(args.port):
        parser.error('--read-only and --port are required together')
    if not 1 <= args.address <= 247:
        parser.error('Use the configured unicast address 1–247, never broadcast')
    if args.baudrate not in (9600, 19200, 38400, 57600, 115200, 230400, 460800):
        parser.error('Unsupported baud rate; match the actual driver settings')
    report = dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        config=str(args.config), hardware_access_requested=args.read_only,
        motion_commands_sent=False, enable_commands_sent=False, configuration_modified=False,
        position_reference_verified=False, gimbal_arrival_verified=False)
    error = None
    try:
        config = validate(json.loads(args.config.read_text(encoding='utf-8-sig')))
        report['configuration_valid'] = True
        report['configured_backend'] = config['servo']['backend']
        report['configured_dry_run'] = config['dry_run']
        if args.read_only:
            report['port'] = args.port
            report['address'] = args.address
            report['readings'] = read_feedback(args.port, args.address, args.baudrate)
            report['note'] = ('Reads succeeded, not a movement or installation-reference test. '
                              'Check READ_ENABLED raw byte against the driver display due to conflicting manufacturer tables.')
        else:
            report['note'] = 'Offline configuration validation only; no UART opened and no device commands sent.'
    except Exception as exc:
        error = str(exc)
    report['error'] = error
    encoded = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(encoded + '\n', encoding='utf-8')
    print(encoded)
    return 1 if error else 0


if __name__ == '__main__':
    raise SystemExit(main())
