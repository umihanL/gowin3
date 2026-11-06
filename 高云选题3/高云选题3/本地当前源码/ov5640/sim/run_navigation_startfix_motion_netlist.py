"""Check the stopped steering preload on synthesized motion/odometry cells."""
from pathlib import Path
import re
ROOT=Path(__file__).resolve().parents[1]
text=(ROOT/'sim/run_navigation_visual_netlist.py').read_text()
text=text[:text.index("probe('fusion',")].replace('navigation_visual_','navigation_startfix_')
exec(compile(text,str(ROOT/'sim/run_navigation_visual_netlist.py'),'exec'))
source=(ROOT/'src/vehicle/vehicle_motion.v').read_text()
ports=source[source.index(')(\n')+3:source.index('\n);')]
ports=re.sub(r'\breg\b','wire',ports)
# Duplicate only the public interface; bind the actual RTL with a shortened
# simulation clock divider. The real design source is never edited.
groups=re.findall(r'\b(?:input|output)\b(.*?)(?=\b(?:input|output)\b|$)',ports,re.S)
names=[]
for group in groups:
 group=re.sub(r'\[[^]]+\]','',group)
 group=re.sub(r'\b(?:reg|wire|signed)\b','',group)
 names+=re.findall(r'\b[a-zA-Z_]\w*\b',group)
wrapper='module vehicle_motion_preload_probe(\n'+ports+'\n);\n'
wrapper+='vehicle_motion #(.CLK_HZ(20000),.START_DELAY_MS(100)) motion('+','.join('.'+n+'('+n+')' for n in names)+');\nendmodule\n'
sources=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').glob('*.v')]+['src/imu/imu_math.v']
probe('motion','vehicle_motion_preload_probe',sources,
 [('navigation_start_preload_tb',[],[('vehicle_motion #(.CLK_HZ(20000),.START_DELAY_MS(100)) dut(',
                                    'vehicle_motion_preload_probe dut(')])],wrapper)
(ROOT/'sim/navigation_startfix_motion_netlist_runs.json').write_text(json.dumps(results,indent=2)+'\n')
