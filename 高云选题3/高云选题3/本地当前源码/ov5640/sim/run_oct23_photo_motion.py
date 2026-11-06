"""Execute and draw the actual RTL route for the October 23 screenshot."""
from pathlib import Path
import csv
import json
import math
import subprocess
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
rows = [tuple(map(int, row)) for row in csv.reader(
    (ROOT/'sim/road_global_routes.csv').read_text().splitlines()) if row and int(row[0]) == 10]
assert rows and rows[0][3] == 0 and rows[-1][3] == 3
dependencies = [ROOT/'src/navigation/nav_road_planner.v',
                *list((ROOT/'src/navigation').glob('nav_centerline_*.vh'))]
assert (ROOT/'sim/road_global_routes.csv').stat().st_mtime >= max(p.stat().st_mtime for p in dependencies)
points = [(r[1], r[2], int(i == 0 or i == len(rows)-1 or r[3] != rows[i+1][3]))
          for i, r in enumerate(rows)]
route = ROOT/'sim/oct23_photo_route.csv'
route.write_text(''.join(f'{x},{y},{lock}\n' for x,y,lock in points), encoding='ascii')
assert sum(p[2] for p in points) == 5, 'start, three ordered waypoints, goal'
sources = [p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').rglob('*.v')]
commands = [
    ['iverilog', '-g2012', '-s', 'road_motion_tb', '-Proad_motion_tb.TEST_SEGMENTS=4',
     '-o', 'sim/oct23_photo_motion.vvp', 'sim/road_motion_tb.sv', *sources, 'src/imu/imu_math.v'],
    ['vvp', 'sim/oct23_photo_motion.vvp', '+ROUTE=sim/oct23_photo_route.csv',
     '+CSV=sim/oct23_photo_motion.csv']]
with (ROOT/'sim/oct23_photo_motion_results.txt').open('w', encoding='utf-8') as log:
    for cmd in commands:
        result = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=600)
        output = result.stdout+result.stderr
        print(output, end='', flush=True)
        log.write(output); log.flush()
        assert result.returncode == 0, f'photo motion test failed: {result.returncode}'

# Use the exact ROM road mask, so this picture cannot hide off-road samples.
raw = [int(v,16) for v in (ROOT/'src/navigation/map_raw_1cm.mem').read_text().split()]
scale, margin = 3, 35
base = Image.new('RGB', (352,320))
base.putdata([(250,250,250) if v == 0 else (52,55,59) for v in raw])
canvas = Image.new('RGB', (352*scale+2*margin,320*scale+90), 'white')
canvas.paste(base.resize((352*scale,320*scale),Image.Resampling.NEAREST),(margin,50))
draw = ImageDraw.Draw(canvas)
def pixel(x,y): return margin+x*scale,50+y*scale
planned = [pixel(x/256,y/256) for x,y,_ in points]
draw.line(planned, fill=(10,100,240), width=6)
actual = []
for row in csv.reader((ROOT/'sim/oct23_photo_motion.csv').read_text().splitlines()):
    actual.append(pixel(float(row[0])/10,320-float(row[1])/10))
if actual: draw.line(actual,fill=(240,110,0),width=2)
for label,(x,y) in zip(('S','1','2','3','G'),((327,241),(243,293),(105,255),(77,199),(25,177))):
    u,v = pixel(x,319-y)
    draw.ellipse((u-7,v-7,u+7,v+7),fill=(220,30,45))
    draw.text((u+9,v-14),label,fill=(0,0,0),stroke_width=2,stroke_fill='white')
length = sum(math.dist(a[:2],b[:2])/256 for a,b in zip(points,points[1:]))
draw.text((margin,12),f'Oct 23 screenshot | ROM cost 406 cm | sampled geometry {length:.1f} cm',fill='black')
draw.text((margin,canvas.height-28),'Blue: planned nose path   Orange: simulated nose   White: road',fill='black')
canvas.save(ROOT/'docs/oct23_photo_route.png')
print(f'PASS exact screenshot motion and overlay: {len(points)} samples; {length:.1f} cm sampled geometry')
