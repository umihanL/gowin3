"""Resample only existing S arcs; preserve all accepted road geometry.

Use this for an installed map whose junctions have already been accepted.
The full geometry generator has an independent, unresolved junction steering
preflight; this tool never regenerates or relaxes those junctions.
"""
from pathlib import Path
import copy
import json
import math
import re

ROOT = Path(__file__).resolve().parents[1]
NAV = ROOT / 'src/navigation'


def densify():
    original = json.loads((NAV / 'road_centerlines.json').read_text())
    data = copy.deepcopy(original)
    old = original['graph']
    targets = {i for i, p in enumerate(data['primitives']) if p.get('role') == 's_bend'}
    assert len(targets) == 2
    nodes, remap, external = [], {}, []
    cursor = 0
    while cursor < len(old):
        stop = cursor + 1
        while stop < len(old) and old[stop]['primitive'] == old[cursor]['primitive']:
            stop += 1
        if old[cursor]['primitive'] not in targets:
            for i in range(cursor, stop):
                remap[i] = len(nodes)
                node = copy.deepcopy(old[i]); node['next'] = []
                nodes.append(node); external.append((remap[i], old[i]['next']))
        else:
            # Each primitive has separate, contiguous forward/reverse chains.
            nold = (stop - cursor) // 2
            assert nold * 2 == stop - cursor
            p = data['primitives'][old[cursor]['primitive']]
            n = math.ceil(abs(p['radius'] * p['delta']) * 352 / 895 / 1.0)
            xy = [(round((p['centre'][0] + p['radius'] * math.cos(p['a0'] + p['delta'] * k / n)) * 352 / 895 * 256),
                   round((p['centre'][1] + p['radius'] * math.sin(p['a0'] + p['delta'] * k / n)) * 320 / 817 * 256))
                  for k in range(n + 1)]
            for direction in range(2):
                seq = xy if direction == 0 else xy[::-1]
                a = cursor + direction * nold; b = a + nold - 1
                assert seq[0] == (old[a]['x'], old[a]['y'])
                assert seq[-1] == (old[b]['x'], old[b]['y'])
                first = len(nodes); remap[a] = first; remap[b] = first + n
                for k, (x, y) in enumerate(seq):
                    nodes.append(dict(x=x, y=y, primitive=old[a]['primitive'],
                                      next=[first+k+1] if k < n else []))
                external.append((first+n, old[b]['next']))
        cursor = stop
    for i, edges in external:
        nodes[i]['next'] = [remap[j] for j in edges]
    assert len(nodes) < 2048, 'Keep the existing 11-bit planner memory capacity'
    assert data['vertices'] == original['vertices'] and data['primitives'] == original['primitives']
    # Untouched directed edges retain their exact endpoints and ordering.
    for i, p in enumerate(old):
        if p['primitive'] not in targets:
            q = nodes[remap[i]]
            assert (p['x'], p['y']) == (q['x'], q['y'])
            assert q['next'] == [remap[j] for j in p['next']]
    raw = [int(x, 16) for x in (NAV/'map_raw_1cm.mem').read_text().split()]
    for p in nodes:
        for j in p['next']:
            q = nodes[j]
            steps = max(1, math.ceil(math.hypot(q['x']-p['x'], q['y']-p['y']) / 64))
            for k in range(steps+1):
                x = round(p['x']+(q['x']-p['x'])*k/steps)//256
                y = round(p['y']+(q['y']-p['y'])*k/steps)//256
                assert 0 <= x < 352 and 0 <= y < 320 and raw[y*352+x] == 0
    bits = data['index_bits']; width = data['word_bits']; invalid = (1 << bits)-1
    lines = ['// Existing road graph; S arcs resampled at 1 cm by tools/densify_s_bend.py.']
    for i, p in enumerate(nodes):
        word = p['x'] | p['y'] << 17
        for k in range(3):
            if k < len(p['next']):
                j = p['next'][k]; q = nodes[j]
                cost = max(1, round(math.hypot(q['x']-p['x'], q['y']-p['y'])/256))
            else:
                j, cost = invalid, 0
            assert cost <= 15
            word |= (j | cost << bits) << (34+k*(bits+4))
        lines.append(f"graph[{i}]={width}'h{word:0{(width+3)//4}x};")
    data.update(nodes=len(nodes), graph=nodes, s_bend_spacing_cm=1.0)
    (NAV/'road_centerlines.json').write_text(json.dumps(data, indent=2)+'\n')
    (NAV/'nav_centerline_init.vh').write_text('\n'.join(lines)+'\n')
    config = (NAV/'nav_centerline_config.vh').read_text()
    config = re.sub(r'`define ROAD_NODES \d+', f'`define ROAD_NODES {len(nodes)}', config)
    (NAV/'nav_centerline_config.vh').write_text(config)
    from write_reverse_road import generate
    generate()
    print(f'PASS S-only resampling: {len(old)} -> {len(nodes)} nodes, unchanged geometry and non-S edges; every edge inside source road')


if __name__ == '__main__':
    densify()
