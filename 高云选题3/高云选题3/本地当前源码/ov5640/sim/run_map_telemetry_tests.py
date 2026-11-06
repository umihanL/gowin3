"""Read-only map protocol and real path-RAM verification; no board commands."""
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'diagnostics/map_telemetry_20261031'


def invoke(args, name, timeout=600):
    cmd = ['wsl.exe', '--exec', *args] if os.name == 'nt' else args
    result = subprocess.run(cmd, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
    text = result.stdout.decode('utf-8', errors='replace').replace('\0', '')
    (OUT/(name+'.log')).write_text(text, encoding='utf-8')
    if result.returncode or 'FATAL:' in text:
        raise RuntimeError(text)
    return dict(name=name, passed=True, command=cmd, log=str((OUT/(name+'.log')).relative_to(ROOT)))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    files = [p for p in (ROOT/'src').rglob('*') if p.is_file()]
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    cases = []
    for top, sources in [
        ('map_telemetry_tb', ['src/vehicle/vehicle_host_link.v','src/vehicle/host_uart_rx.v','src/vehicle/motor/uart_byte_tx.v']),
        ('road_planner_tb', ['src/navigation/nav_road_planner.v']),
    ]:
        output=(OUT/(top+'.vvp')).relative_to(ROOT).as_posix()
        invoke(['iverilog','-g2012','-s',top,'-I','src/navigation','-o',output,f'sim/{top}.sv',*sources], top+'_compile')
        cases.append(invoke(['vvp',output],top))
        print('PASS',top,flush=True)
    # Reuse only the previous documented compilation command, with current
    # production source files. Encrypted vendor IP remain explicit blackboxes.
    prior=json.loads((ROOT/'diagnostics/map_uart_merge_20261031/rtl_merge_audit.json').read_text(encoding='utf-8'))
    commands=prior['top_elaboration']['attempts'] if 'attempts' in prior['top_elaboration'] else []
    if not commands:
        commands=next(v for v in prior['top_elaboration'].values() if isinstance(v,list) and v and isinstance(v[0],dict) and 'command' in v[0])
    item=next(x for x in commands if x['name']=='full_top_blackbox_compat')
    args=item['command'][2:]
    # The current top includes the later B13 pump interlock. The archived
    # command predates that module, but remains useful for vendor-IP stubs.
    if 'src/vehicle/vehicle_pump.v' not in args:
        args.append('src/vehicle/vehicle_pump.v')
    args[args.index('-o')+1]=(OUT/'full_top_blackbox_compat.vvp').relative_to(ROOT).as_posix()
    record=invoke(args,'full_top_blackbox_compat')
    record['boundary']=item['boundary'];record['compatibility']=item['compatibility']
    cases.append(record)
    unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    report=dict(cases=cases,source_hashes=hashes,source_unchanged=unchanged,
        board_tested=False,synthesis_or_timing_verified=False,new_bitstream_built=False,
        limitation='This runner does not invoke Gowin; encrypted IP excluded only for structural elaboration')
    (OUT/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    if not unchanged:raise RuntimeError('source changed during verification')
    print('PASS map UART, independent actual route RAM and structural top elaboration; no bitstream/hardware claim',flush=True)


if __name__=='__main__':main()
