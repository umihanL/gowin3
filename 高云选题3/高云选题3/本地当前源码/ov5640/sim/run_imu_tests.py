"""Bit-level SPI/UART, independently calculated quaternion oracle and UI/reset tests."""
from pathlib import Path
import math
import random
import struct
import subprocess

root = Path(__file__).resolve().parents[1]

def quaternion(r, p, y):
    r, p, y = [math.radians(a)/2 for a in (r, p, y)]
    cr, cp, cy = math.cos(r), math.cos(p), math.cos(y)
    sr, sp, sy = math.sin(r), math.sin(p), math.sin(y)
    return (cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy)

def mul(a, b):
    w,x,y,z=a; v,i,j,k=b
    return (w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v)

def packed(q, expected):
    if q[0] < 0:
        q = tuple(-v for v in q)
    halves = [int.from_bytes(struct.pack('<e', v), 'little') for v in q[1:]]
    values = halves+[round(v*10) & 0xffff for v in expected]
    return ''.join(f'{v:04x}' for v in reversed(values))

rng = random.Random(5640)
angles = [(0,0,0),(30,0,0),(-45,0,0),(0,30,0),(0,-60,0),(0,0,90),
          (0,0,-90),(180,0,0),(0,0,180),(0,90,0),(0,-90,0)]
angles += [(rng.uniform(-170,170),rng.uniform(-65,65),rng.uniform(-170,170)) for _ in range(200-len(angles))]
vectors=[packed(quaternion(*a),a) for a in angles]
ref=quaternion(20,-25,35)
rel=[(0,0,0)]+[(rng.uniform(-80,80),rng.uniform(-50,50),rng.uniform(-80,80)) for _ in range(39)]
vectors += [packed(mul(ref,quaternion(*a)),a) for a in rel]
vectors += ['0'*24]*16
(root/'sim/imu_vectors.mem').write_text('\n'.join(vectors)+'\n',encoding='ascii')

# Independent ideal rotations around the calibrated axes. Pitch at +/-90
# has no unique roll/yaw, so the boundary test checks pitch there separately.
boundary_refs = [(0,0,0),(20,-25,35),(0,0,60),(0,0,120),
                 (45,30,60),(-35,40,-80),(-60,-45,30),(15,20,-120)]
roll_sweep = [0,80,85,88,89,89.5,89.9,90,90.1,91,95,120,170,180,
              -80,-89,-90,-91,-120,-180]
pitch_sweep = [0,45,80,85,88,89,89.5,89.9,90,90.1,91,95,120,180,
               -45,-80,-85,-88,-89,-89.5,-89.9,-90,-90.1,-91,-95,-120,-180]
boundary_vectors = []
for pose in boundary_refs:
    reference = quaternion(*pose)
    boundary_vectors.append(packed(reference,(0,0,0)))
    for a in roll_sweep:
        boundary_vectors.append(packed(mul(reference,quaternion(a,0,0)),(a,0,0)))
    for a in pitch_sweep:
        euler_pitch = 180-a if a>90 else (-180-a if a< -90 else a)
        boundary_vectors.append(packed(mul(reference,quaternion(0,a,0)),(0,euler_pitch,0)))
(root/'sim/imu_angle_limits_vectors.mem').write_text(
    '\n'.join(boundary_vectors)+'\n',encoding='ascii')

def axis_quaternion(axis, angle):
    half = math.radians(angle)/2
    return (math.cos(half), *(v*math.sin(half) for v in axis))

def sensor_quantized(q):
    if q[0] < 0:
        q = tuple(-v for v in q)
    xyz = [struct.unpack('<e',struct.pack('<e',v))[0] for v in q[1:]]
    return (math.sqrt(max(0,1-sum(v*v for v in xyz))),*xyz)

def rotation_vector(q):
    if q[0] < 0:
        q = tuple(-v for v in q)
    size = math.sqrt(sum(v*v for v in q[1:]))
    scale = math.degrees(2*math.atan2(size,q[0]))/size if size else 0
    return tuple(v*scale for v in q[1:])

# End-to-end binary16 sensor -> saved zero -> quaternion log. Use a floating
# reference based on the actual compressed input, without RTL Q14 arithmetic.
# This distinguishes sensor compression uncertainty from FPGA calculation error.
rv_references = [(0,0,0),(20,-25,35),(-35,40,-80),(0,0,120)]
sweep = list(range(0,151,3))+list(range(147,-1,-3))
sweep += list(range(-3,-151,-3))+list(range(-147,1,3))
rv_axes = [(1,0,0),(0,1,0),(0,0,1),
           (0,math.cos(math.radians(9)),math.sin(math.radians(9)))]
rv_vectors = []
rv_cases = len(rv_axes)*len(sweep)+12
for pose in rv_references:
    ref = quaternion(*pose)
    decoded_ref = sensor_quantized(ref)
    inverse = (decoded_ref[0], *(-v for v in decoded_ref[1:]))
    rv_vectors.append(packed(ref,(0,0,0)))
    for axis in rv_axes:
        for a in sweep:
            q = mul(ref,axis_quaternion(axis,a))
            expected = rotation_vector(mul(inverse,sensor_quantized(q)))
            rv_vectors.append(packed(q,expected))
    # Return to EXACT original input, then hold a genuine 10 degree offset.
    # The latter must remain visible: stationary does not mean original zero.
    for a in [0]*6+[10]*6:
        q = mul(ref,axis_quaternion((0,1,0),a))
        rv_vectors.append(packed(q,rotation_vector(mul(inverse,sensor_quantized(q)))))
(root/'sim/imu_rotation_vectors.mem').write_text('\n'.join(rv_vectors)+'\n',encoding='ascii')
tests = {
    'attitude':['src/imu/imu_attitude.v','src/imu/imu_math.v'],
    'angle_limits':['src/imu/imu_attitude.v','src/imu/imu_math.v'],
    'spi':['src/imu/lsm6dsv16x_spi.v','src/imu/spi_register.v'],
    'ui':['src/imu/attitude_overlay.v','src/imu/board_buttons.v'],
    'firewater':['src/imu/imu_firewater_tx.v','src/vehicle/motor/uart_byte_tx.v'],
    'motor_firewater':['src/imu/imu_firewater_tx.v','src/vehicle/motor/uart_byte_tx.v'],
    'rotation':['src/imu/imu_attitude.v','src/imu/imu_math.v','src/imu/imu_rotation_vector.v'],
}
with (root/'sim/imu_test_results.txt').open('w',encoding='utf-8') as log:
    for name,files in tests.items():
        top=f'imu_{name}_tb'
        for fault in (range(6) if name=='spi' else (range(4) if name=='firewater' else (range(2) if name in ('ui','motor_firewater') else [0]))):
            params=[f'-P{top}.BANK_FAULT={fault}'] if name=='spi' else []
            if name=='firewater':
                params += [f'-P{top}.EXTENDED_POSE={fault//2}']
                if fault%2==0:
                    params += [f'-P{top}.CLK_HZ=1000000',f'-P{top}.BAUD=100000']
            if name=='motor_firewater' and fault==0:
                params += [f'-P{top}.CLK_HZ=1000000',f'-P{top}.BAUD=100000']
            if name=='ui':
                params += [f'-P{top}.AXIS_VECTOR={fault}']
            if name=='angle_limits':
                params += [f'-P{top}.REFERENCE_COUNT={len(boundary_refs)}',
                           f'-P{top}.ROLL_CASES={len(roll_sweep)}',
                           f'-P{top}.PITCH_CASES={len(pitch_sweep)}']
            if name=='rotation':
                params += [f'-P{top}.REFERENCES={len(rv_references)}',f'-P{top}.CASES={rv_cases}']
            for cmd in [['iverilog','-g2012','-s',top,*params,'-o',f'sim/{top}.vvp',f'sim/{top}.v',*files],
                        ['vvp',f'sim/{top}.vvp']]:
                result=subprocess.run(cmd,cwd=root,capture_output=True,text=True,timeout=90)
                output=result.stdout+result.stderr
                print(output,end='',flush=True);log.write(output);log.flush()
                if result.returncode:raise SystemExit(result.returncode)
