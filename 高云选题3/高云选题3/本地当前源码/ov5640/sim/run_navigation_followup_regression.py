"""Run isolated follow-up logs using the existing meaningful navigation suites."""
from pathlib import Path
import argparse
import subprocess

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('suite',choices=('corners','routes','targeted','netlist','profile_netlist','gate','fault_gate'))
args=parser.parse_args()
name={'gate':'gate_tests'}.get(args.suite,args.suite)
source=(ROOT/f'sim/run_navigation_entry_{name}.py').read_text()
source=source.replace('navigation_entry','navigation_followup')
if args.suite=='fault_gate':
    source=source.replace("navigation_followup_fault_tb.sv", "navigation_followup_tb.sv")
    source=source.replace("navigation_followup_fault_tb", "navigation_followup_tb")
    source=source.replace("navigation_followup_fault_gate", "navigation_followup_gate")
    source=source.replace("navigation_followup_probe", "navigation_followup_probe")
path=ROOT/f'sim/generated_followup_{args.suite}.py'
path.write_text(source)
raise SystemExit(subprocess.run(['python',str(path)],cwd=ROOT).returncode)
