"""Existing corner/video/motion/UART regressions with separate release logs."""
from pathlib import Path
import subprocess,sys
ROOT=Path(__file__).resolve().parents[1]
groups=[('corners','run_navigation_visual_corners.py',[]),
 ('targeted','run_navigation_visual_targeted.py',[]),
 ('extras','run_navigation_visual_targeted.py',['--extras']),
 ('host','run_navigation_visual_host.py',[])]
for label,name,args in groups:
 if '--resume' in sys.argv and label=='corners' and (ROOT/'sim/navigation_turnfix_compat_corners_corner_runs.json').exists():continue
 text=(ROOT/'sim'/name).read_text().replace('navigation_visual_',f'navigation_turnfix_compat_{label}_')
 target=ROOT/f'sim/_navigation_turnfix_compat_{label}.py';target.write_text(text)
 result=subprocess.run([sys.executable,str(target),*args],cwd=ROOT)
 if result.returncode:raise SystemExit(result.returncode)
