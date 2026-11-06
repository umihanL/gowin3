"""Bind the read-only inertial review to the current hardware and tests."""
from pathlib import Path
import datetime, hashlib, json

ROOT = Path(__file__).resolve().parents[1]
sha = lambda n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest()
release = json.loads((ROOT/'impl/pnr/ov5640_current_audit.json').read_text(encoding='utf-8'))
assert all(sha(n) == digest for n, digest in release['hardware_inputs'].items())
assert release['sha256'] == sha(release['bitstream']) == sha(release['release_bitstream'])
groups = {}
for stage in ('plan', 'motion', 'control'):
    groups[stage] = json.loads((ROOT/f'sim/inertial_audit_{stage}_runs.json').read_text())
    for row in groups[stage]:
        assert row['passed'] and sha(row['log']) == row['log_sha256']
        assert all(sha(n) == digest for n, digest in row['inputs'].items()), row['name']
        for field in ('route', 'trace'):
            if field in row: assert sha(row[field]) == row[field+'_sha256']

network_name = 'src/navigation/road_centerlines.json'
network = json.loads((ROOT/network_name).read_text())['graph']
for row in groups['plan']:
    points = [tuple(map(int, line.split(',')))[:2] for line in (ROOT/row['route']).read_text().splitlines()]
    assert len(points) == row['count']
    possible = {i for i, n in enumerate(network) if (n['x'], n['y']) == points[0]}
    for point in points[1:]:
        closure, pending = set(possible), list(possible)
        while pending:
            i = pending.pop()
            for j in network[i]['next']:
                if j not in closure and (network[j]['x'], network[j]['y']) == (network[i]['x'], network[i]['y']):
                    closure.add(j)
                    pending.append(j)
        possible = {j for i in closure for j in network[i]['next']
                    if (network[j]['x'], network[j]['y']) == point}
        assert possible, (row['name'], point)
for row in groups['motion']:
    m = row['metrics']
    assert m['status'] == 3 and m['index'] == m['count']-1 and m['target_checks'] >= m['count']

normal = 'sim/inertial_audit_outer_left_trace.csv'
hostile = 'sim/inertial_audit_outer_left_hostile_trace.csv'
assert (ROOT/normal).read_bytes() == (ROOT/hostile).read_bytes()
control = (ROOT/groups['control'][0]['log']).read_text(encoding='utf-8')
assert 'mapSteer=1 output=-3 left=50 right=1500' in control
assert 'mapSteer=-1 output=3 left=1500 right=50' in control
assert 'front=500 left=1500 right=1500 output=-28 phase=1' in control

note = 'docs/inertial_navigation_audit_20261028.md'
scripts = ['sim/run_inertial_audit.py', 'tools/finalize_inertial_audit_20261028.py']
result = dict(
    generated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    status='inspection_and_reproduction_completed', hardware_changed=False, flashed=False,
    bitstream=release['bitstream'], bitstream_sha256=release['sha256'],
    resources_unchanged=release['resources'], hardware_inputs=release['hardware_inputs'],
    tests=groups, graph_json_sha256=sha(network_name), report=note, report_sha256=sha(note),
    scripts={n: sha(n) for n in scripts},
    checks=dict(planner_display_matches_execution_ram=True, directed_graph_paths_valid=True,
                motion_target_order_and_coordinates_valid=True, hostile_vision_trace_identical=True,
                side_tof_reverses_map_command_reproduced=True, front_tof_overrides_route_reproduced=True),
    limitations=[
        'Four planner examples and five motion cases, not exhaustive map coverage.',
        'Independent ideal bicycle plant; not physical vehicle validation.',
        'The physical incident lacks recorded ranges, phase, pose and steering commands.',
        'Opposite steering during a sampled S bend can be legitimate closed-loop correction.',
    ])
path = ROOT/'docs/inertial_navigation_audit_20261028_validation.json'
path.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps({k: result[k] for k in ('status', 'hardware_changed', 'bitstream_sha256', 'checks')}, indent=2))
