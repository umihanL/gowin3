"""Run display regressions and the regenerated Gowin FIFO simulation models."""
from pathlib import Path
import argparse
import subprocess

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--prim', default='C:/Gowin/Gowin_V1.9.12.03_x64/IDE/simlib/gw5a/prim_sim.v')
args = parser.parse_args()
tests = {
    'display': ['src/reset_sync.v', 'src/vga/disp_driver.v'],
    'source': ['src/vga/tft_video_source.v'],
    'fifo': ['src/rd_data_fifo/rd_data_fifo.vo', 'src/wr_data_fifo/wr_data_fifo.vo', args.prim],
    'ddr_handoff': ['src/ddr_pll/ddr_pll.v', 'src/pll_init.v'],
}
with (root / 'sim/fix_test_results.txt').open('w', encoding='utf-8') as log:
    for name, files in tests.items():
        top = f'fix_{name}_tb'
        output = f'sim/fix_{name}.vvp'
        for cmd in [
            ['iverilog', '-g2012', *(['-DSIM'] if name == 'ddr_handoff' else []),
             '-s', top, '-I', 'src/vga', '-o', output, f'sim/{top}.v', *files],
            ['vvp', output],
        ]:
            result = subprocess.run(cmd, cwd=root, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True)
            print(result.stdout, end='')
            log.write(result.stdout)
            log.flush()
            if result.returncode:
                raise SystemExit(result.returncode)
