"""Repeat the three physical fault scenarios on Gowin's synthesized cells."""
from pathlib import Path
import hashlib,json,subprocess

ROOT=Path(__file__).resolve().parents[1]
probe=ROOT/'impl/navigation_followup_probe'
netlist=probe/'impl/gwsynthesis/navigation_followup_probe.vg'
assert netlist.stat().st_mtime > (ROOT/'src/vehicle/vehicle_vision_fusion.v').stat().st_mtime
source=(ROOT/'sim/navigation_followup_tb.sv').read_text().replace(
    'module navigation_followup_tb;',
    "module navigation_followup_tb;\n GSR GSR(.GSRI(1'b1));")
bench=probe/'navigation_followup_tb.sv';bench.write_text(source)
results=[]
for case in range(3):
    output=f'sim/navigation_followup_gate_{case}.vvp'
    log=ROOT/f'sim/navigation_followup_gate_{case}.log'
    with log.open('w') as stream:
        for cmd in (['iverilog','-g2012','-s','navigation_followup_tb',
                     f'-Pnavigation_followup_tb.CASE={case}','-o',output,str(bench),
                     str(netlist),'C:/Gowin/Gowin_V1.9.12.03_x64/IDE/simlib/gw5a/prim_sim.v'],
                    ['vvp',output]):
            subprocess.run(cmd,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,check=True,timeout=300)
    results.append(dict(case=case,passed=True,log=log.relative_to(ROOT).as_posix(),
                        sha256=hashlib.sha256(log.read_bytes()).hexdigest()))
    print('PASS synthesized entry fault',case,flush=True)
(ROOT/'sim/navigation_followup_gate_runs.json').write_text(json.dumps(results,indent=2)+'\n')
