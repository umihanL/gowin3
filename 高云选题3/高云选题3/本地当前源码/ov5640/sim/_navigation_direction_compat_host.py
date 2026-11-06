"""Record protocol/integration and old/new navigation fault verification."""
from pathlib import Path
import subprocess,hashlib,json,time,sys
ROOT=Path(__file__).resolve().parents[1]
plant=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').rglob('*.v')]+['src/imu/imu_math.v']
baseline='impl/baseline_host_navigation_20261027/src/vehicle/vehicle_vision_fusion.v'
cases=[
 ('uart','vehicle_host_link_tb',['src/vehicle/vehicle_host_link.v','src/vehicle/host_uart_rx.v','src/vehicle/motor/uart_byte_tx.v'],False),
 ('integration','host_navigation_integration_tb',plant,False),
 ('next_entry_before','navigation_next_entry_tb',[baseline],True),
 ('next_entry_after','navigation_next_entry_tb',['src/vehicle/vehicle_vision_fusion.v'],False),
]
results=[]
for name,top,sources,expected in cases:
 output=f'sim/navigation_direction_compat_host_host_{name}.vvp';log=ROOT/f'sim/navigation_direction_compat_host_host_{name}.log';start=time.monotonic()
 with log.open('w',encoding='utf-8') as f:
  compile=subprocess.run(['iverilog','-g2012','-s',top,'-o',output,f'sim/{top}.sv',*sources],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
  assert compile.returncode==0,name
  run=subprocess.run(['vvp',output],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=120)
 text=log.read_text()
 if expected:assert run.returncode!=0 and 'NEXT_ENTRY_EARLY' in text,name
 else:assert run.returncode==0 and 'PASS ' in text and '$finish called' in text,name
 result=dict(name=name,top=top,passed=True,expected_failure=expected,exit_code=run.returncode,
  seconds=round(time.monotonic()-start,3),log=log.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(log.read_bytes()).hexdigest(),
  inputs={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in [f'sim/{top}.sv',*sources]})
 results.append(result);print(name,'EXPECTED_FAILURE' if expected else 'PASS',flush=True)
(ROOT/'sim/navigation_direction_compat_host_host_runs.json').write_text(json.dumps(results,indent=2)+'\n')
