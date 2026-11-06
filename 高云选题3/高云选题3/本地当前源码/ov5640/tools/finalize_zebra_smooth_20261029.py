"""Verify and attach saved test evidence to the frozen release audits."""
from pathlib import Path
import csv
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]
LABEL = 'zebra_smooth_20261029'


def sha(name):
    return hashlib.sha256((ROOT / name).read_bytes()).hexdigest()


def read(name):
    b = (ROOT / name).read_bytes()
    return b.decode('utf-16' if b.startswith((b'\xff\xfe', b'\xfe\xff')) else 'utf-8-sig')


frozen = json.loads(read(f'impl/{LABEL}_inputs.json'))['hardware_inputs']
assert all(sha(p) == h for p, h in frozen.items()), 'Frozen hardware input changed'
validation = json.loads(read(f'docs/{LABEL}_validation.json'))
assert sha(validation['bitstream']) == sha(validation['release_bitstream']) == validation['sha256']
assert validation['timing_violations'] == dict(Setup=0, Hold=0)

logs = [f'sim/zebra_smooth_{n}.log' for n in
        ('detector', 'exposure', 'camera', 'overlay', 'detector_netlist')]
logs += [f'sim/inertial_zebra_{n}.log' for n in ('settings', 'modes', 'avoidance')]
evidence = []
for name in logs:
    result = read(name)
    assert 'PASS ' in result and 'FATAL:' not in result, name
    evidence.append(dict(log=name, sha256=sha(name),
                         results=[line.strip() for line in result.splitlines() if 'PASS ' in line]))

plans = json.loads(read('sim/zebra_smooth_plan_runs.json'))
motions = json.loads(read('sim/zebra_smooth_motion_runs.json'))
assert len(plans) == 5 and len(motions) == 6
graph = json.loads(read('src/navigation/road_centerlines.json'))['graph']


def directed_path(name):
    points = [(int(r[0]), int(r[1])) for r in csv.reader(read(name).splitlines())]
    coords = [(n['x'], n['y']) for n in graph]
    current = {i for i, c in enumerate(coords) if c == points[0]}
    assert current, (name, 'first point missing')
    for point in points[1:]:
        # Junction primitives can share coordinates in consecutive graph nodes.
        pending = list(current)
        while pending:
            i = pending.pop()
            for j in graph[i]['next']:
                if coords[j] == coords[i] and j not in current:
                    current.add(j)
                    pending.append(j)
        current = {j for i in current for j in graph[i]['next'] if coords[j] == point}
        assert current, (name, 'non-contiguous directed edge', point)
    return len(points)


for result in plans + motions:
    assert result['passed'] and sha(result['log']) == result['log_sha256'], result['name']
    if 'route' in result:
        result['points_on_continuous_directed_graph'] = directed_path(result['route'])
        result['route_sha256'] = sha(result['route'])
    if 'trace' in result:
        assert result['metrics']['status'] == 3
        result['trace_sha256'] = sha(result['trace'])
        if result['name'] in ('outer_left', 'outer_right'):
            assert sha(result['trace']) == sha(f"sim/inertial_zebra_{result['name']}_trace.csv")

post = dict(status='passed', compared_cycles=105482,
            source_netlist_sha256=sha('impl/gwsynthesis/ov5640.vg'),
            extracted_netlist='sim/zebra_smooth_detector_synth.v',
            extracted_netlist_sha256=sha('sim/zebra_smooth_detector_synth.v'),
            wrapper='sim/vision_zebra_synth_wrap.sv',
            wrapper_sha256=sha('sim/vision_zebra_synth_wrap.sv'),
            testbench='sim/vision_zebra_synth_tb.sv',
            testbench_sha256=sha('sim/vision_zebra_synth_tb.sv'),
            log='sim/zebra_smooth_detector_netlist.log',
            log_sha256=sha('sim/zebra_smooth_detector_netlist.log'),
            scope='detector with constant centre=52 and binary samples; cross-boundary gates restored')
for name in (f'docs/{LABEL}_validation.json', f'impl/pnr/ov5640_{LABEL}_audit.json',
             'impl/pnr/ov5640_current_audit.json'):
    data = json.loads(read(name))
    assert data['release'] == LABEL and data['sha256'] == validation['sha256']
    data['post_synthesis_detector_simulation'] = post
    data['simulation_evidence'] = dict(logs=evidence, plans=plans, motions=motions)
    data['known_limitations'] = [
        'Not flashed and not tested on a physical vehicle.',
        'S transition improvement is simulated; full-route peak error is essentially unchanged.',
        'Full-map generator has an existing junction steering-limit precheck failure.',
        'Legacy nav_ui_tb.v references removed planner.state and is not a passing test.'
    ]
    (ROOT / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(f'PASS frozen inputs={len(frozen)}; logs={len(logs)}; directed plans={len(plans)}; '
      f'completed motion replays={len(motions)}; matching bitstream={validation["sha256"]}')
