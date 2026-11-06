"""Local HDL regressions only: never opens a serial port or flashes hardware."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import hashlib,json,subprocess,sys,time
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'diagnostics/semantic_navigation_20261101'
OUT.mkdir(parents=True,exist_ok=True)
SOURCES=[p.relative_to(ROOT).as_posix() for folder in ('navigation','vehicle')
         for p in (ROOT/'src'/folder).rglob('*.v')]+['src/imu/imu_math.v']
CASES=['semantic_navigation_tb','semantic_touch_preflight_tb','semantic_planner_tb',
       'travelled_raster_tb','semantic_renderer_tb','host_patrol_pause_tb']
def command(args,log):
    with log.open('wb') as f:
        result=subprocess.run(['wsl.exe','--exec',*args],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=2400)
    raw=log.read_bytes()
    return result.returncode,raw

def run(name):
    started=time.monotonic();binary='sim/'+name+'_semantic.vvp'
    code,text=command(['iverilog','-g2012','-s',name,'-I','src/navigation','-I','src/vga','-o',binary,'sim/'+name+'.sv',*SOURCES],OUT/(name+'_compile.log'))
    if code:return {'case':name,'passed':False,'stage':'compile'}
    code,text=command(['vvp','-i',binary],OUT/(name+'.log'))
    passed=code==0 and b'PASS ' in text and b'FATAL:' not in text
    print(('PASS ' if passed else 'FAIL ')+name,flush=True)
    return {'case':name,'passed':passed,'seconds':round(time.monotonic()-started,2)}
if __name__=='__main__':
    selected=sys.argv[1:] or CASES
    with ThreadPoolExecutor(max_workers=4) as pool:results=list(pool.map(run,selected))
    report={'passed':all(x['passed'] for x in results),'cases':results,'hardware_tested':False,'flashed':False,
      'sources':{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (ROOT/'src').rglob('*') if p.is_file()},
      'scope':'Real UART/declared heading/real road planner and motion controller; raster/renderer unit checks. No physical travel accuracy claim.'}
    (OUT/'latest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    raise SystemExit(0 if report['passed'] else 1)
