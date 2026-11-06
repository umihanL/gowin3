"""Check the new synchronous upcoming-corner ROM after Gowin synthesis."""
from pathlib import Path
import re,subprocess
ROOT=Path(__file__).resolve().parents[1]
probe=ROOT/'impl/navigation_followup_profile_probe';probe.mkdir(exist_ok=True)
files=('src/vehicle/vehicle_route_profile.v','src/imu/imu_math.v')
project=re.sub(r'<FileList>.*?</FileList>',
    '<FileList>'+''.join(f'<File path="../../{n}" type="file.verilog" enable="1" />' for n in files)+'</FileList>',
    (ROOT/'ov5640.gprj').read_text(),flags=re.S)
(probe/'probe.gprj').write_text(project)
(probe/'build.tcl').write_text('cd [file dirname [file normalize [info script]]]\nopen_project probe.gprj\n'+
    f'set_option -include_path {{{ROOT.as_posix()}}}\nset_option -top_module vehicle_route_profile\n'+
    'set_option -output_base_name profile_probe\nrun syn\nexit 0\n')
bench=(ROOT/'sim/navigation_profile_corner_tb.sv').read_text().replace(
    'module navigation_profile_corner_tb;',"module navigation_profile_corner_tb;\n GSR GSR(.GSRI(1'b1));")
(probe/'bench.sv').write_text(bench)
commands=[['C:/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe',str(probe/'build.tcl')],
    ['iverilog','-g2012','-s','navigation_profile_corner_tb','-o','sim/navigation_followup_profile_netlist.vvp',
     str(probe/'bench.sv'),str(probe/'impl/gwsynthesis/profile_probe.vg'),
     'C:/Gowin/Gowin_V1.9.12.03_x64/IDE/simlib/gw5a/prim_sim.v'],
    ['vvp','sim/navigation_followup_profile_netlist.vvp']]
with (ROOT/'sim/navigation_followup_profile_netlist.log').open('w') as log:
    for command in commands:
        p=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=300)
        if p.returncode:raise SystemExit(p.returncode)
print('PASS synthesized upcoming entry distances, signed turns and exit headings',flush=True)
