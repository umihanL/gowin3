"""Audit actual RTL routes with a continuous rear-axle bicycle/body model.

This is a conservative diagnostic against the SOURCE DRAWING's 1cm occupancy
mask, not a real-car certificate: its lane widths differ from measured widths.
Never reset vehicle yaw at a primitive boundary or infer safety from a global
signed-distance envelope. A nonzero exit reports unresolved body clearance.
"""
from pathlib import Path
import json, math, re, collections
import numpy as np
from scipy.ndimage import distance_transform_edt

ROOT=Path(__file__).resolve().parents[1]
NAV=ROOT/'src/navigation'

def audit():
    graph=json.loads((NAV/'road_centerlines.json').read_text())
    cfg={k:int(v) for k,v in re.findall(r'^`define (VEHICLE_\w+) (-?\d+)$',
         (ROOT/'src/vehicle/vehicle_config.vh').read_text(),re.M)}
    raw=np.array([int(v,16) for v in (NAV/'map_raw_1cm.mem').read_text().split()]).reshape(320,352)
    free=raw==0
    signed=(distance_transform_edt(free)-distance_transform_edt(~free))*10
    file=ROOT/'sim/road_global_routes.csv'
    assert file.stat().st_mtime>=max(p.stat().st_mtime for p in NAV.glob('nav_centerline_*.vh')), 'RTL routes are stale; rerun global road tests'
    cases=collections.defaultdict(list)
    for line in file.read_text().splitlines():
        case,x,y,_=map(int,line.split(','));cases[case].append([x*10/256,3200-y*10/256])
    along,side=np.meshgrid(np.arange(-cfg['VEHICLE_BODY_LENGTH_MM'],1,5),
                          np.arange(-cfg['VEHICLE_BODY_WIDTH_MM']/2,cfg['VEHICLE_BODY_WIDTH_MM']/2+1,5))
    along=along.ravel();side=side.ravel();L=cfg['VEHICLE_NOSE_FROM_REAR_MM']
    reports=[]
    for case,points in sorted(cases.items()):
        route=[]
        for a,b in zip(points,points[1:]):
            a,b=np.array(a),np.array(b);distance=np.linalg.norm(b-a)
            if distance<1e-6:continue
            n=math.ceil(distance/2.5)
            route.extend(a+(b-a)*i/n for i in range(n))
        route=np.array([*route,points[-1]])
        heading=np.arctan2(np.diff(route[:,1]),np.diff(route[:,0]));yaw=float(heading[0])
        hit_frames=0;first=None;min_clear=1e9;peak=0
        for i in range(len(route)-1):
            h=np.array([math.cos(yaw),math.sin(yaw)]);left=np.array([-h[1],h[0]])
            body=route[i]+along[:,None]*h+side[:,None]*left
            x=np.floor(body[:,0]/10).astype(int);y=np.floor((3200-body[:,1])/10).astype(int)
            inside=(x>=0)&(x<352)&(y>=0)&(y<320)
            clear=np.full(len(body),-10000.)
            clear[inside]=signed[y[inside],x[inside]]
            hit=clear<0
            min_clear=min(min_clear,float(clear.min()))
            if hit.any():
                hit_frames+=1
                if first is None:
                    j=int(np.flatnonzero(hit)[0]);first=dict(nose_mm=route[i].tolist(),yaw_deg=math.degrees(yaw),
                        body_point_mm=body[j].tolist(),along_mm=float(along[j]),left_mm=float(side[j]))
            error=(heading[i]-yaw+math.pi)%(2*math.pi)-math.pi
            peak=max(peak,abs(math.degrees(math.atan(cfg['VEHICLE_WHEELBASE_MM']*math.tan(error)/L))))
            ds=float(np.linalg.norm(route[i+1]-route[i]));t=heading[i]
            def f(q):return math.sin(t-q)/L
            k1=f(yaw);k2=f(yaw+ds*k1/2);k3=f(yaw+ds*k2/2);k4=f(yaw+ds*k3)
            yaw+=ds*(k1+2*k2+2*k3+k4)/6
        reports.append(dict(case=case,poses=len(route),body_hits_source_mask=hit_frames,
                            minimum_source_mask_clearance_mm=min_clear,
                            ideal_polyline_peak_steer_deg=peak,first_source_mask_hit=first))
    result=dict(body_mm=[cfg['VEHICLE_BODY_LENGTH_MM'],cfg['VEHICLE_BODY_WIDTH_MM']],
                nose_from_rear_mm=L,route_step_mm=2.5,footprint_grid_mm=5,
                initial_yaw='first route tangent; continuous across every subsequent join',
                steering_formula='atan(wheelbase*tan(nose_tangent-body_yaw)/nose_offset)',
                occupancy_source='original drawing, 1cm cells; not metrically calibrated to measured lane widths',
                body_clearance_certified=False,source_mask_sweep_passed=not any(r['body_hits_source_mask'] for r in reports),
                cases=reports)
    (ROOT/'docs/road_body_sweep_oct24.json').write_text(json.dumps(result,indent=2)+'\n')
    for r in reports:print(f"case {r['case']}: body hits drawing in {r['body_hits_source_mask']}/{r['poses']} poses; source-mask minimum {r['minimum_source_mask_clearance_mm']:.0f} mm")
    print('BODY CLEARANCE NOT CERTIFIED: measured road widths and source drawing require metric registration.')
    return result

if __name__=='__main__':
    result=audit()
    raise SystemExit(0 if result['source_mask_sweep_passed'] else 2)
