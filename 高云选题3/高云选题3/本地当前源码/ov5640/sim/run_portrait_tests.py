"""Verify image processing, orientation, frame ownership, scanout and touch."""
from pathlib import Path
import argparse
import subprocess
from threading import Thread

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--prim', default='C:/Gowin/Gowin_V1.9.12.03_x64/IDE/simlib/gw5a/prim_sim.v')
parser.add_argument('--full', action='store_true', help='also verify three complete 640x480 DVP/DDR scanouts with the vendor FIFO models')
args = parser.parse_args()
tests = {
    'display': ['src/portrait/portrait_display.v'],
    'ddr': ['src/portrait/portrait_ddr_adapter.v',
            'src/rd_data_fifo/rd_data_fifo.vo', 'src/wr_data_fifo/wr_data_fifo.vo', args.prim],
    'capture': ['src/DVP_Capture/DVP_Capture.v', 'src/portrait/portrait_display.v',
                'src/portrait/portrait_ddr_adapter.v', 'src/rd_data_fifo/rd_data_fifo.vo',
                'src/wr_data_fifo/wr_data_fifo.vo', args.prim],
    'touch': ['src/portrait/gt911_touch.v', 'src/camera_init/i2c_control/i2c_control.v',
              'src/camera_init/i2c_control/i2c_bit_shift.v'],
}
if args.full:
    tests['capture_full'] = tests['capture']
with (root / 'sim/portrait_test_results.txt').open('w', encoding='utf-8') as log:
    for name, files in tests.items():
        top = 'video_capture_tb' if name.startswith('capture') else f'portrait_{name}_tb'
        output = f'sim/portrait_{name}.vvp'
        for cmd in [
            ['iverilog', '-g2012', *(['-DFULL_SIZE'] if name.endswith('_full') else []),
             '-s', top, '-o', output, f'sim/{top}.v', *files],
            ['vvp', output],
        ]:
            with subprocess.Popen(cmd, cwd=root, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, text=True) as process:
                def forward_output():
                    for line in process.stdout:
                        print(line, end='', flush=True)
                        log.write(line)
                        log.flush()

                reader = Thread(target=forward_output)
                reader.start()
                try:
                    result = process.wait(timeout=2400 if args.full else 120)
                except subprocess.TimeoutExpired:
                    process.kill()
                    reader.join()
                    message = f'FAIL: timed out running {cmd}\n'
                    print(message, end='', flush=True)
                    log.write(message)
                    raise SystemExit(1)
                reader.join()
            if result:
                raise SystemExit(result)
