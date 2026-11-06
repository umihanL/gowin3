"""Export directed graph routes through the junction boxed in the photo."""
from pathlib import Path
import json
from prepare_navigation_robust_routes import route,nodes
ROOT=Path(__file__).resolve().parents[1]
cases={
 'box_enter_north':[(105,245),(145,174)],
 'box_enter_south':[(105,140),(145,174)],
 'box_exit_north':[(145,174),(105,140)],
 'box_exit_south':[(145,174),(105,245)],
}
manifest={}
for name,points in cases.items():
 path,cost=route(points)
 filename=f'sim/host_navigation_{name}_route.csv'
 (ROOT/filename).write_text(''.join(f"{nodes[i]['x']},{nodes[i]['y']},{int(k in (0,len(path)-1))}\n" for k,i in enumerate(path)))
 manifest[name]=dict(points=points,samples=len(path),cost_cm=cost,file=filename)
(ROOT/'sim/host_navigation_box_routes.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(manifest,indent=2))
