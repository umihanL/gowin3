"""Read fixed-10-byte ov5640 capabilities/status only; never start or move."""
import argparse
import datetime
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/rescue_vision'))
from rescue_vision.ov5640_protocol import encode, Parser, capabilities, status


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='/dev/ttyS2')
    p.add_argument('--report', type=Path, required=True)
    args = p.parse_args()
    if args.port in ('/dev/ttyS0', '/dev/ttyS1', '/dev/ttyS3', '/dev/ttyS5'):
        p.error('Reserved system/motor port')
    result = dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  port=args.port, motion_commands_sent=False, replies=[], error=None)
    try:
        import serial
        parser = Parser()
        with serial.Serial(args.port, 115200, timeout=.05, write_timeout=.2, exclusive=True) as uart:
            uart.reset_input_buffer()
            for seq, cmd in enumerate((0x15, 0x14, 0x06, 0x09), start=70):
                frame = encode(seq, cmd)
                uart.write(frame)
                deadline = time.monotonic() + 1.
                found = False
                while time.monotonic() < deadline:
                    for reply in parser.feed(uart.read(128)):
                        row = dict(sequence=reply.sequence, command=reply.command, data_hex=reply.data.hex(' '))
                        if (reply.sequence, reply.command) == (seq, cmd | 0x80):
                            found = True
                            if cmd == 0x15:
                                row['capabilities'] = capabilities(reply.data)
                            elif cmd == 0x14:
                                row['status'] = status(reply.data)
                        result['replies'].append(row)
                    if found:
                        break
                if not found:
                    raise RuntimeError('no_matching_reply_for_' + hex(cmd))
        result['protocol_verified'] = True
    except Exception as exc:
        result.update(protocol_verified=False, error=str(exc))
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['protocol_verified'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
