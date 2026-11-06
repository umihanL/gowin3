"""Reproducible RTL acceptance. Windows uses installed WSL Icarus, no board I/O."""
from pathlib import Path
import hashlib,json,os,subprocess,time
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'diagnostics/extension_tasks_20261029'
vehicle=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').rglob('*.v')]+['src/imu/imu_math.v']
navigation=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/navigation').glob('*.v')]
cases=[
 ('controller','vehicle_extension_tb',['src/vehicle/vehicle_extension.v'],[]),
 ('extension_uart','extension_uart_tb',vehicle,[]),
 ('legacy_uart','vehicle_host_link_tb',vehicle,[]),
 ('legacy_integration','host_navigation_integration_tb',vehicle,[]),
 ('route_upload','extension_route_upload_tb',['src/navigation/nav_control.v','src/navigation/nav_road_planner.v'],[]),
 ('settings','vehicle_settings_tb',navigation+['src/imu/imu_math.v','src/vehicle/motor/ps2_test_control.v'],['+snapshot']),
]
inputs={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (ROOT/'src').rglob('*') if p.is_file()}
records=[]
for name,top,sources,extra in cases:
    start=time.monotonic();binary=f'diagnostics/extension_tasks_20261029/{name}.vvp'
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
(OUT/'validation.json').write_text(json.dumps(dict(hardware_inputs=inputs,cases=records,physical_hardware_tested=False),indent=2)+'\n',encoding='utf-8')
