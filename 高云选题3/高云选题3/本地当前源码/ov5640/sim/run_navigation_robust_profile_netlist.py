"""Verify synthesized route tangents, corner exits and tiny graph joins."""
from pathlib import Path
import re,subprocess
ROOT=Path(__file__).resolve().parents[1];PROBE=ROOT/'impl/navigation_robust_profile_probe';PROBE.mkdir(exist_ok=True)
project=re.sub(r'<FileList>.*?</FileList>','<FileList><File path="../../src/vehicle/vehicle_route_profile.v" type="file.verilog" enable="1" /><File path="../../src/imu/imu_math.v" type="file.verilog" enable="1" /></FileList>',(ROOT/'ov5640.gprj').read_text(),flags=re.S)
(PROBE/'navigation_robust_profile_probe.gprj').write_text(project)
(PROBE/'build.tcl').write_text('cd [file dirname [file normalize [info script]]]\nopen_project navigation_robust_profile_probe.gprj\n'+f'set_option -include_path {{{ROOT.as_posix()}}}\nset_option -top_module vehicle_route_profile\nset_option -output_base_name navigation_robust_profile_probe\nrun syn\nexit 0\n')
commands=[['C:/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe',str(PROBE/'build.tcl')]]
for top in ('navigation_profile_corner_tb','navigation_profile_graph_tb'):
 bench=(ROOT/f'sim/{top}.sv').read_text().replace(f'module {top};',f"module {top};\n GSR GSR(.GSRI(1'b1));")
 (PROBE/f'{top}.sv').write_text(bench)
 exe=f'sim/navigation_robust_{top}_netlist.vvp'
 commands += [['iverilog','-g2012','-s',top,'-o',exe,str(PROBE/f'{top}.sv'),str(PROBE/'impl/gwsynthesis/navigation_robust_profile_probe.vg'),'C:/Gowin/Gowin_V1.9.12.03_x64/IDE/simlib/gw5a/prim_sim.v'],['vvp',exe]]
with (ROOT/'sim/navigation_robust_profile_netlist.log').open('w',encoding='utf-8') as log:
 for command in commands:
  p=subprocess.run(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=300)
  print(p.stdout,end='',flush=True);log.write(p.stdout);log.flush()
  if p.returncode:raise SystemExit(p.returncode)
print('PASS synthesized route profile',flush=True)
