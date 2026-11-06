"""Reproducible RTL acceptance. Windows uses installed WSL Icarus, no board I/O."""
from pathlib import Path
import argparse,hashlib,json,os,subprocess,time
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'diagnostics/extension_revision_20261029'
vehicle=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').rglob('*.v')]+['src/imu/imu_math.v']
navigation=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/navigation').glob('*.v')]
cases=[
 ('zebra_switch','zebra_switch_tb',['src/vehicle/vehicle_extension.v','src/portrait/vision_zebra.v','src/navigation/nav_control.v'],[]),
 ('detector','vision_zebra_tb',['src/portrait/vision_zebra.v'],[]),
 ('navigation_no_limit','navigation_no_zebra_limit_tb',vehicle+['src/portrait/vision_video.v','src/portrait/vision_zebra.v'],[]),
 ('controller','vehicle_extension_tb',['src/vehicle/vehicle_extension.v'],[]),
 ('extension_uart','extension_uart_tb',vehicle,[]),
 ('legacy_uart','vehicle_host_link_tb',vehicle,[]),
 ('legacy_integration','host_navigation_integration_tb',vehicle,[]),
 
 ('settings','vehicle_settings_tb',navigation+['src/imu/imu_math.v','src/vehicle/motor/ps2_test_control.v'],['+revision_snapshot']),
]
inputs={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (ROOT/'src').rglob('*') if p.is_file()}
records=[]
p=argparse.ArgumentParser();p.add_argument('--only',nargs='+');a=p.parse_args()
for name,top,sources,extra in cases:
    if a.only and name not in a.only:continue
    start=time.monotonic();binary=f'diagnostics/extension_revision_20261029/{name}.vvp'
    commands=[['iverilog','-g2012','-s',top,'-I','src/navigation','-I','src/vga','-o',binary,f'sim/{top}.sv',*sources],['vvp',binary,*extra]]
    outputs=[]
    for command in commands:
        actual=['wsl.exe','--exec',*command] if os.name=='nt' else command
        r=subprocess.run(actual,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=1200)
        text=r.stdout.decode('utf-8',errors='replace').replace('\0','');outputs.append(text)
        if r.returncode or 'FATAL:' in text:raise RuntimeError(f'{name}: {text}')
    combined='\n'.join(outputs);assert 'PASS ' in combined,name
    (OUT/f'{name}.log').write_text(combined,encoding='utf-8')
    records.append(dict(name=name,passed=True,seconds=round(time.monotonic()-start,3),test=f'sim/{top}.sv',test_sha256=hashlib.sha256((ROOT/f'sim/{top}.sv').read_bytes()).hexdigest()))
    print('PASS',name,records[-1]['seconds'],flush=True)
assert inputs=={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'src').rglob('*') if p.is_file()},'RTL changed during tests'
if a.only and (OUT/'validation.json').exists():
    previous=json.loads((OUT/'validation.json').read_text(encoding='utf-8'))
    updated={r['name'] for r in records}
    records=[r for r in previous['cases'] if r['name'] not in updated]+records
(OUT/'validation.json').write_text(json.dumps(dict(hardware_inputs=inputs,cases=records,physical_hardware_tested=False),indent=2)+'\n',encoding='utf-8')
