"""Reproduce the reported authority faults through public module ports."""
from pathlib import Path
import argparse, hashlib, json, subprocess, time

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--extras', action='store_true')
args = parser.parse_args()
fusion = ['src/vehicle/vehicle_vision_fusion.v']
plant = [p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').rglob('*.v')]
plant += ['src/imu/imu_math.v']
if args.extras:
    cases = [('vehicle', 'vehicle_motion_tb', [], plant + ['sim/ps2_receiver_model.sv',
              'sim/vl53l0x_sensor_model.v', 'sim/vl53l1x_sensor_model.v']),
             ('profile', 'vehicle_profile_tb', [], plant),
             ('zebra', 'vision_zebra_tb', [], ['src/portrait/vision_zebra.v'])]
else:
    cases = [('authority_'+str(i), 'navigation_authority_tb',
              [f'-Pnavigation_authority_tb.CASE={i}'], fusion) for i in range(4)]
    cases += [('handoff', 'navigation_handoff_tb', [], fusion),
              ('video', 'vision_video_tb', [], ['src/portrait/vision_video.v',
               'src/portrait/vision_zebra.v', *fusion])]
results = []
for name, top, params, sources in cases:
    executable = f'sim/navigation_visual_{name}.vvp'
    log = ROOT/f'sim/navigation_visual_{name}.log'
    started = time.monotonic()
    with log.open('w', encoding='utf-8') as output:
        for command in (['iverilog', '-g2012', '-s', top, *params, '-o', executable,
                         f'sim/{top}.sv', *sources], ['vvp', '-i', executable]):
            result = subprocess.run(command, cwd=ROOT, stdout=output,
                                    stderr=subprocess.STDOUT, timeout=1800)
            if result.returncode:
                raise SystemExit(f'{name} failed; see {log}')
    results.append(dict(name=name, top=top, passed=True,
                        seconds=round(time.monotonic()-started, 2),
                        log=log.relative_to(ROOT).as_posix(),
                        sha256=hashlib.sha256(log.read_bytes()).hexdigest()))
    print(name, 'PASS', results[-1]['seconds'], flush=True)
manifest = 'extras' if args.extras else 'targeted'
(ROOT/f'sim/navigation_visual_{manifest}_runs.json').write_text(
    json.dumps(results, indent=2)+'\n')
