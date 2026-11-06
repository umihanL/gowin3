"""Render the actual synthesized OSD RTL, with known telemetry test values."""
from pathlib import Path
import subprocess
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
subprocess.run(['iverilog','-g2012','-s','vehicle_overlay_tb','-I','src/navigation',
                '-o','sim/vehicle_overlay.vvp','sim/vehicle_overlay_tb.sv',
                'src/navigation/nav_renderer.v'], cwd=ROOT, check=True)
result = subprocess.run(['vvp','sim/vehicle_overlay.vvp'], cwd=ROOT,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                        text=True, timeout=300)
print(result.stdout, end='')
(ROOT/'sim/vehicle_overlay_results.txt').write_text(result.stdout, encoding='utf-8')
result.check_returncode()
for page in ('detect','navigation'):
    Image.open(ROOT/f'sim/menu_{page}.ppm').save(ROOT/f'docs/menu_{page}.png')
