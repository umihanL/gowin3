"""Run the reported-fault regressions on the synthesized fusion circuit."""
from pathlib import Path
import hashlib, json, subprocess

ROOT = Path(__file__).resolve().parents[1]
probe = ROOT/'impl/navigation_entry_probe'
netlist = probe/'impl/gwsynthesis/navigation_entry_probe.vg'
assert netlist.stat().st_mtime > (ROOT/'src/vehicle/vehicle_vision_fusion.v').stat().st_mtime
bench = (ROOT/'sim/navigation_authority_tb.sv').read_text().replace(
    'module navigation_authority_tb;',
    "module navigation_authority_tb;\n GSR GSR(.GSRI(1'b1));")
(probe/'navigation_authority_tb.sv').write_text(bench)
results = []
for case in range(4):
    executable = f'sim/navigation_entry_gate_{case}.vvp'
    log = ROOT/f'sim/navigation_entry_gate_{case}.log'
    with log.open('w') as output:
        for command in (['iverilog', '-g2012', '-s', 'navigation_authority_tb',
                         f'-Pnavigation_authority_tb.CASE={case}', '-o', executable,
                         str(probe/'navigation_authority_tb.sv'), str(netlist),
                         'C:/Gowin/Gowin_V1.9.12.03_x64/IDE/simlib/gw5a/prim_sim.v'],
                        ['vvp', executable]):
            subprocess.run(command, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT,
                           check=True, timeout=300)
    results.append(dict(case=case, passed=True, log=log.relative_to(ROOT).as_posix(),
                        sha256=hashlib.sha256(log.read_bytes()).hexdigest()))
    print('PASS synthesized authority', case, flush=True)
(ROOT/'sim/navigation_entry_gate_runs.json').write_text(json.dumps(results, indent=2)+'\n')
