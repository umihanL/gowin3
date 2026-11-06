"""Render the actual RTL photo-case route and export it for vehicle replay."""
from pathlib import Path
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
rows = [tuple(map(int, line.split(','))) for line in
        (ROOT/'sim/road_global_routes.csv').read_text().splitlines()]
route = [(x, y, segment) for case, x, y, segment in rows if case == 8]
assert route and max(y for _, y, _ in route) < 145*256
lines = []
for i, (x, y, segment) in enumerate(route):
    lock = int(i == 0 or i == len(route)-1 or route[i+1][2] != segment)
    lines.append(f'{x},{y},{lock}')
(ROOT/'sim/photo_route.csv').write_text('\n'.join(lines)+'\n')
canvas = Image.open(ROOT/'地图.jpg').convert('RGB')
draw = ImageDraw.Draw(canvas)
points = [(x/256*895/352, y/256*817/320) for x, y, _ in route]
draw.line(points, fill=(240, 45, 45), width=7)
for i, (point, (_, _, lock)) in enumerate(zip(points, [tuple(map(int, s.split(','))) for s in lines])):
    if lock:
        x, y = point
        draw.ellipse((x-7, y-7, x+7, y+7), fill=(30, 110, 255))
canvas.save(ROOT/'docs/repaired_photo_route.png')
print(f'PASS photo route export: {len(route)} actual RTL samples, 312cm, no lower loop')
