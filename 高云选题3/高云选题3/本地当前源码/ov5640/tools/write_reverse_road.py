"""Generate reverse adjacency with exactly the forward graph's edge weights."""
from pathlib import Path
import json, math
ROOT=Path(__file__).resolve().parents[1]
def generate():
    data=json.loads((ROOT/'src/navigation/road_centerlines.json').read_text())
    nodes=data['graph'];bits=data['index_bits'];invalid=(1<<bits)-1
    reverse=[[] for _ in nodes]
    for i,p in enumerate(nodes):
        for j in p['next']:
            q=nodes[j]
            cost=max(1,round(math.hypot(q['x']-p['x'],q['y']-p['y'])/256))
            reverse[j].append((i,cost))
    assert max(map(len,reverse))<=3
    width=3*(bits+4)
    lines=['// Generated reverse adjacency; weights equal the forward ROM.']
    for i,edges in enumerate(reverse):
        value=sum(((edges[k][0] | edges[k][1]<<bits) if k<len(edges) else invalid)
                  <<(k*(bits+4)) for k in range(3))
        lines.append(f"reverse_graph[{i}]={width}'h{value:0{(width+3)//4}x};")
    (ROOT/'src/navigation/nav_centerline_reverse.vh').write_text('\n'.join(lines)+'\n')
    print(f'Reverse graph: {len(nodes)} x {width} bits')
if __name__=='__main__':generate()
