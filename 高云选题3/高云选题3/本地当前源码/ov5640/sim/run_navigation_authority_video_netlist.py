"""Compare synthesized Gowin preview hardware with exact rational RTL."""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PROBE = ROOT/'impl/vision_probe'
PROBE.mkdir(exist_ok=True)
project = (ROOT/'ov5640.gprj').read_text(encoding='utf-8')
project = re.sub(r'<FileList>.*?</FileList>',
                 '<FileList><File path="../../src/portrait/vision_video.v" '
                 'type="file.verilog" enable="1" /><File path="../../src/portrait/vision_zebra.v" '
                 'type="file.verilog" enable="1" /></FileList>', project, flags=re.S)
(PROBE/'vision_probe.gprj').write_text(project, encoding='utf-8')
(PROBE/'build.tcl').write_text(
    'cd [file dirname [file normalize [info script]]]\n'
    'open_project vision_probe.gprj\n'
    f'set_option -include_path {{{ROOT.as_posix()}}}\n'
    'set_option -top_module vision_video\n'
    'set_option -output_base_name vision_probe\nrun syn\nexit 0\n')
reference = (ROOT/'src/portrait/vision_video.v').read_text()
reference = reference.replace('module vision_video #(', 'module vision_video_reference #(')
reference = reference.replace('vision_zebra zebra(', 'vision_zebra_reference zebra(')
reference = re.sub(r'wire \[6:0\] source_column=scale_lut\[view_u\],source_row=scale_lut\[view_v\];',
                   'wire [6:0] source_column=(view_u*13)/60,source_row=(view_v*13)/60;', reference)
zebra_reference = (ROOT/'src/portrait/vision_zebra.v').read_text().replace(
    'module vision_zebra(', 'module vision_zebra_reference(')
(PROBE/'vision_video_reference.v').write_text(reference+'\n'+zebra_reference)
commands = [
    ['C:/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe', 'impl/vision_probe/build.tcl'],
    ['iverilog', '-g2012', '-s', 'vision_netlist_tb', '-o', 'sim/vision_netlist_tb.vvp',
     'sim/vision_netlist_tb.sv', 'impl/vision_probe/vision_video_reference.v',
     'impl/vision_probe/impl/gwsynthesis/vision_probe.vg',
     'C:/Gowin/Gowin_V1.9.12.03_x64/IDE/simlib/gw5a/prim_sim.v'],
    ['vvp', 'sim/vision_netlist_tb.vvp'],
]
with (ROOT/'sim/navigation_authority_video_netlist.log').open('w') as log:
    for command in commands:
        with subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True) as process:
            for line in process.stdout:
                print(line, end='', flush=True)
                log.write(line)
                log.flush()
            if process.wait():
                raise SystemExit(process.returncode)
