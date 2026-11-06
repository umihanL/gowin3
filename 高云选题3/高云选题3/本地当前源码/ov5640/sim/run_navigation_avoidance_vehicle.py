"""Stream the long existing whole-vehicle regression; retain input hashes."""
from pathlib import Path
import hashlib, json, subprocess, time
ROOT = Path(__file__).resolve().parents[1]
sources = [p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').rglob('*.v')]
sources += ['src/imu/imu_math.v', 'sim/ps2_receiver_model.sv',
            'sim/vl53l0x_sensor_model.v', 'sim/vl53l1x_sensor_model.v', 'sim/vehicle_motion_tb.sv']
deps = sources+['src/vehicle/vehicle_config.vh']
sha = lambda n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest()
frozen = {n: sha(n) for n in deps}
log = 'sim/navigation_avoidance_vehicle_motion.log'
output = 'sim/navigation_avoidance_vehicle_motion.vvp'
# Keep the bench first as in the established runner. Add diagnostics only;
# all original stimulus, independent physics and assertions remain intact.
bench = 'sim/navigation_avoidance_vehicle_progress_tb.sv'
source = (ROOT/'sim/vehicle_motion_tb.sv').read_text()
source = source.replace('module vehicle_motion_tb;', '''module vehicle_motion_tb;
initial begin forever begin
    #5000000;
    $display("PROGRESS t=%0t ticks=%0d mode=%0d state=%0d status=%0d x=%f phase=%0d",
             $time,ticks,mode,dut.state,status,world_x,phase);
    $fflush();
end end''')
(ROOT/bench).write_text(source)
frozen[bench] = sha(bench)
start = time.monotonic()
with (ROOT/log).open('w', encoding='utf-8') as stream:
    for command in (
        ['iverilog', '-g2012', '-s', 'vehicle_motion_tb', '-o', output, bench, *sources[:-1]],
        ['vvp', '-i', output],
    ):
        p = subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, timeout=1800)
        assert p.returncode == 0, log
assert all(sha(n) == digest for n, digest in frozen.items())
content = (ROOT/log).read_text()
assert 'PASS navigation dynamic obstacle:' in content and '$finish called' in content and 'FATAL:' not in content
record = dict(passed=True, seconds=round(time.monotonic()-start, 3), log=log,
              log_sha256=sha(log), inputs=frozen)
(ROOT/'sim/navigation_avoidance_vehicle_run.json').write_text(json.dumps(record, indent=2)+'\n')
print('PASS whole-vehicle motion regression', record['seconds'], 'seconds', flush=True)
