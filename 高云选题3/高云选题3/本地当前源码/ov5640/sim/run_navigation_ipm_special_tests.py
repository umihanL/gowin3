"""IPM response, asynchronous close-side alignment and visual bypass handback."""
from pathlib import Path
import hashlib, json, subprocess, time

ROOT = Path(__file__).resolve().parents[1]
vehicle = [p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').glob('*.v')]
plant = vehicle + ['src/imu/imu_math.v']
fusion = ['src/vehicle/vehicle_vision_fusion.v']
bypass = ['src/vehicle/vehicle_bypass.v', 'src/vehicle/vehicle_tracker.v',
          'src/vehicle/vehicle_sincos.v', 'src/imu/imu_math.v']
cases = [('vision_ipm_response_tb', 'first_frame', [], fusion),
         ('bypass_single_front_tb', 'single_front', [], bypass),
         ('bypass_edge_trend_tb', 'dual_front', [], bypass),
         ('navigation_handoff_tb', 'handoff_rtl', [], fusion),
         ('vehicle_close_bypass_tb', 'vision_rejoin', ['-Pvehicle_close_bypass_tb.ENABLE_VISION=1'], plant),
         ('vehicle_motion_tb', 'motion_regression', [], plant+['sim/ps2_receiver_model.sv', 'sim/vl53l0x_sensor_model.v', 'sim/vl53l1x_sensor_model.v']),
         ('vehicle_profile_tb', 'profile', [], plant)]
results = []
for top, variant, params, sources in cases:
    log = ROOT/f'sim/navigation_ipm_{variant}.log'
    exe = f'sim/navigation_ipm_{variant}.vvp'
    start = time.monotonic()
    with log.open('w', encoding='utf-8') as output:
        for command in (['iverilog', '-g2012', '-s', top, *params, '-o', exe,
                         f'sim/{top}.sv', *sources], ['vvp', exe]):
            result = subprocess.run(command, cwd=ROOT, stdout=output,
                                    stderr=subprocess.STDOUT, timeout=900)
            if result.returncode:
                raise SystemExit(f'{variant} failed; see {log}')
    text = log.read_text()
    assert 'PASS ' in text and '$finish called' in text, variant
    results.append(dict(name=variant, passed=True, seconds=round(time.monotonic()-start, 3),
                        log=log.relative_to(ROOT).as_posix(), sha256=hashlib.sha256(log.read_bytes()).hexdigest()))
    print('PASS', variant, flush=True)
(ROOT/'sim/navigation_ipm_special_runs.json').write_text(json.dumps(results, indent=2)+'\n')
