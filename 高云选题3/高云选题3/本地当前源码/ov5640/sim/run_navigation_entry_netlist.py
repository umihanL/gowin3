"""Exercise the public map/vision handoff on synthesized Gowin primitives."""
from pathlib import Path
import re,subprocess
ROOT=Path(__file__).resolve().parents[1];PROBE=ROOT/'impl/navigation_entry_probe';PROBE.mkdir(exist_ok=True)
project=re.sub(r'<FileList>.*?</FileList>','<FileList><File path="../../src/vehicle/vehicle_vision_fusion.v" type="file.verilog" enable="1" /></FileList>',(ROOT/'ov5640.gprj').read_text(),flags=re.S)
(PROBE/'navigation_entry_probe.gprj').write_text(project)
(PROBE/'build.tcl').write_text('cd [file dirname [file normalize [info script]]]\nopen_project navigation_entry_probe.gprj\n'+f'set_option -include_path {{{ROOT.as_posix()}}}\nset_option -top_module vehicle_vision_fusion\nset_option -output_base_name navigation_entry_probe\nrun syn\nexit 0\n')
bench=(ROOT/'sim/navigation_handoff_tb.sv').read_text().replace('module navigation_handoff_tb;',"module navigation_handoff_tb;\n GSR GSR(.GSRI(1'b1));")
(PROBE/'navigation_handoff_tb.sv').write_text(bench)
commands=[['C:/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe',str(PROBE/'build.tcl')],['iverilog','-g2012','-s','navigation_handoff_tb','-o','sim/navigation_entry_handoff_netlist.vvp',str(PROBE/'navigation_handoff_tb.sv'),str(PROBE/'impl/gwsynthesis/navigation_entry_probe.vg'),'C:/Gowin/Gowin_V1.9.12.03_x64/IDE/simlib/gw5a/prim_sim.v'],['vvp','sim/navigation_entry_handoff_netlist.vvp']]
with (ROOT/'sim/navigation_entry_handoff_netlist.log').open('w',encoding='utf-8') as log:
 for command in commands:
  p=subprocess.run(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=300)
  print(p.stdout,end='',flush=True);log.write(p.stdout);log.flush()
  if p.returncode:raise SystemExit(p.returncode)
print('PASS synthesized map/vision handoff',flush=True)
