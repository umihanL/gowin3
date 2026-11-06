"""Navigation RTL regression; --snapshot also renders pixels from the real RTL."""
from pathlib import Path
import argparse
import subprocess
from PIL import Image

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--snapshot', action='store_true')
args = parser.parse_args()
navigation = [str(p.relative_to(root)) for p in (root/'src/navigation').glob('*.v')]
tests = {'road_planner': ['src/navigation/nav_road_planner.v'],
         'map_rom': ['src/navigation/nav_map_rom.v'],
         'road_ui': navigation+['src/imu/imu_math.v'],
         'video_timing': ['src/vga/disp_driver.v']}
with (root/'sim/navigation_test_results.txt').open('w', encoding='utf-8') as log:
    for name, files in tests.items():
        top = f'{name}_tb' if name.startswith('road_') else f'nav_{name}_tb'
        extension = 'sv' if name.startswith('road_') else 'v'
        output = f'sim/nav_{name}.vvp'
        for cmd in [['iverilog', '-g2012', '-s', top, '-I', 'src/navigation', '-I', 'src/vga',
                     '-o', output, f'sim/{top}.{extension}', *files],
                    ['vvp', output, *(['+snapshot'] if name=='road_ui' and args.snapshot else [])]]:
            result = subprocess.run(cmd, cwd=root, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, timeout=1200)
            print(result.stdout, end='', flush=True)
            log.write(result.stdout)
            log.flush()
            if result.returncode:
                raise SystemExit(result.returncode)
if args.snapshot:
    for name in ('road_map',):
        Image.open(root/f'sim/menu_{name}.ppm').save(root/f'docs/menu_{name}.png')
