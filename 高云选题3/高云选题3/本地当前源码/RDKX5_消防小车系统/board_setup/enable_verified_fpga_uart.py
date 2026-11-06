"""Enable the already verified PIN15/22 FPGA link, without motion requests."""
import datetime
import json
from pathlib import Path
import shutil
import sys

ws = Path('/root/rescue/ros2_ws')
report_dir = Path('/root/rescue/deployment_20261031/task_runtime')
probe = json.loads((report_dir / 'fpga_uart_probe.json').read_text())
age = (datetime.datetime.now(datetime.timezone.utc) -
       datetime.datetime.fromisoformat(probe['created_utc'])).total_seconds()
if (sys.platform != 'linux' or probe.get('protocol_verified') is not True or
        probe.get('port') != '/dev/ttyS2' or not 0 <= age <= 180):
    raise RuntimeError('A fresh verified UART2 probe is required')
status = Path('/proc/device-tree/soc/a55_apb0/serial@34080000/status').read_bytes().rstrip(b'\0')
if status != b'okay':
    raise RuntimeError('UART2 not enabled in the running device tree')
p = ws / 'config/extension_uart.json'
backup = report_dir / 'extension_uart.before.json'
if not backup.exists():
    shutil.copy2(p, backup)
cfg = json.loads(p.read_text())
cfg.update(enabled=True, port='/dev/ttyS2', baud=115200)
cfg['note'] = ('Verified UART2: physical PIN15 TX -> FPGA E13 RX, PIN22 RX <- E14 TX; '
               'GND common. Startup does not start navigation. Explicit operator commands required.')
p.write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(dict(fpga_uart_enabled=True, port=cfg['port'],
                     movement_requested=False, motor_configuration_changed=False)))
