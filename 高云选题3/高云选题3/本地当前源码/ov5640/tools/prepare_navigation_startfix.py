"""Preserve the reported release and rebuild the latest photographed route."""
from pathlib import Path
import hashlib,json,shutil
from prepare_navigation_robust_routes import route,nodes
ROOT=Path(__file__).resolve().parents[1]
base=ROOT/'impl/baseline_navigation_startfix_20261027'
if not (base/'manifest.json').exists():
    files=[p for p in (ROOT/'src').rglob('*') if p.is_file()]
    files += [ROOT/n for n in ('ov5640.gprj','build.tcl','impl/pnr/ov5640.fs','impl/pnr/ov5640.pr')]
    manifest={}
    for p in files:
        if not p.exists():continue
        n=p.relative_to(ROOT);target=base/n;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(p,target);manifest[n.as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
    (base/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
points=[(327,171),(221,293),(25,241),(175,165),(275,127),(327,219)]
path,cost=route([(x,319-y) for x,y in points])
(ROOT/'sim/navigation_startfix_reported_route.csv').write_text(''.join(
    f"{nodes[i]['x']},{nodes[i]['y']},{int(k in (0,len(path)-1))}\n" for k,i in enumerate(path)))
print('Preserved release; reported route:',len(path),'samples,',cost,'cm',flush=True)
