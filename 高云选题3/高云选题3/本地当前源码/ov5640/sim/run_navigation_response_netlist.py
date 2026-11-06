"""Run the new loss sequence against standalone synthesized Gowin hardware."""
from pathlib import Path
import re
import subprocess
ROOT=Path(__file__).resolve().parents[1]
PROBE=ROOT/'impl/navigation_response_probe'
PROBE.mkdir(exist_ok=True)
project=(ROOT/'ov5640.gprj').read_text()
project=re.sub(r'<FileList>.*?</FileList>',
    '<FileList><File path="../../src/vehicle/vehicle_vision_fusion.v" type="file.verilog" enable="1" /></FileList>',
    project,flags=re.S)
(PROBE/'navigation_response_probe.gprj').write_text(project)
(PROBE/'build.tcl').write_text(
    'cd [file dirname [file normalize [info script]]]\n'
    'open_project navigation_response_probe.gprj\n'
    f'set_option -include_path {{{ROOT.as_posix()}}}\n'
    'set_option -top_module vehicle_vision_fusion\n'
    'set_option -output_base_name navigation_response_probe\nrun syn\nexit 0\n')
bench=(ROOT/'sim/navigation_response_tb.sv').read_text().replace('module navigation_response_tb;',
    "module navigation_response_tb;\n GSR GSR(.GSRI(1'b1));")
(PROBE/'navigation_response_tb.sv').write_text(bench)
commands=[['C:/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe',str(PROBE/'build.tcl')],
    ['iverilog','-g2012','-s','navigation_response_tb','-o','sim/navigation_response_netlist_20261025.vvp',
     str(PROBE/'navigation_response_tb.sv'),str(PROBE/'impl/gwsynthesis/navigation_response_probe.vg'),
     'C:/Gowin/Gowin_V1.9.12.03_x64/IDE/simlib/gw5a/prim_sim.v'],
    ['vvp','sim/navigation_response_netlist_20261025.vvp']]
with (ROOT/'sim/navigation_response_netlist_20261025.log').open('w',encoding='utf-8') as log:
    for command in commands:
        result=subprocess.run(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                              text=True,timeout=300)
        print(result.stdout,end='',flush=True);log.write(result.stdout);log.flush()
        if result.returncode:raise SystemExit(result.returncode)
print('PASS navigation synthesized response',flush=True)
