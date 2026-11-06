"""Generate fixed-point inverse-perspective coefficients without bird-view RAM."""
from pathlib import Path
import json, math, re
ROOT=Path(__file__).resolve().parents[1]
cfg={k:int(v) for k,v in re.findall(r'^`define (VEHICLE_\w+) (-?\d+)$',(ROOT/'src/vehicle/vehicle_config.vh').read_text(),re.M)}
width,height=640,480
road=json.loads((ROOT/'src/navigation/road_dimensions.json').read_text())
measured=road[road['current_measurement']]
inner_width=measured['inner_width_mm'];outer_width=measured['outer_width_mm']
hfov=road['provisional']['horizontal_fov_deg']
focal=width/(2*math.tan(math.radians(hfov/2)))
H=cfg['VEHICLE_CAMERA_HEIGHT_MM'];nose=cfg['VEHICLE_CAMERA_FROM_NOSE_MM'];left=cfg['VEHICLE_CAMERA_LEFT_MM']
centre=cfg['VEHICLE_CAMERA_CENTRE_GROUND_MM']
# Lens behind nose: camera-to-centre-point distance is centre+nose.
pitch=math.atan2(H,centre+nose)
rows=[]
for i,row in enumerate(range(70,17,-2)):
    v=math.ceil((row+1)*height/78)-1
    q=(v-(height-1)/2)/focal
    den=math.sin(pitch)+q*math.cos(pitch)
    depth=H/den
    forward=H*(math.cos(pitch)-q*math.sin(pitch))/den-nose
    scale=depth/focal*width/208
    base_u=(102/2+1)*width/104-1
    bias=(base_u-(width-1)/2)*depth/focal-left
    # Keep the true projected width even when it exceeds the 104-column
    # thumbnail. Clamping it to the image width invents a clipped centre.
    rows.append(dict(index=i,row=row,native_v=v,forward_mm=round(forward),scale_q8=round(scale*256),bias_q8=round(bias*256),nominal_width=round(inner_width/(scale*2)),outer_width=round(outer_width/(scale*2)),usable=150<=forward<=800))
# Retain several observable rows for boundary evidence on the widest lane.
# The old 288 mm window cannot show both edges of a centred 373 mm lane.
first_visible=next(i for i,z in enumerate(rows) if z['outer_width']<=102)
boundary_max=rows[first_visible+4]['forward_mm']
header='// Generated metric IPM; lens intrinsics remain provisional.\n'
for name,row in [('NEAR',60),('MIDDLE',46),('FAR',32)]:
    z=next(z for z in rows if z['row']==row)
    ref=102-z['bias_q8']/z['scale_q8']
    for key,value in [('ROW',row),('COLUMN',round(ref/2)),('CENTRE_Q1',round(ref)),('NOMINAL_WIDTH',z['nominal_width'])]:
        header+=f'`define VISION_{name}_{key} {value}\n'
header+='`define VISION_ERROR_IS_METRIC 1\n`define VISION_IPM_MIN_FORWARD_MM 150\n`define VISION_IPM_MAX_FORWARD_MM 800\n`define VISION_IPM_LOOKAHEAD_MM 210\n`define VISION_IPM_HEADING_BASE_MM 110\n'
header+=f'`define VISION_BOUNDARY_MAX_FORWARD_MM {boundary_max}\n'
(ROOT/'src/portrait/vision_camera_config.vh').write_text(header,encoding='ascii')
lookup='// Generated ray/flat-ground intersection for each actual scan row.\n'
for name,bits,field,signed in [('ipm_scale_q8',12,'scale_q8',False),('ipm_bias_q8',12,'bias_q8',True),('ipm_forward_mm',10,'forward_mm',False),('ipm_nominal_width',8,'nominal_width',False),('ipm_outer_width',8,'outer_width',False)]:
    lookup+=f'function {"signed " if signed else ""}[{bits-1}:0] {name};\n    input [4:0] index;\n    begin case(index)\n'
    for z in rows:
        v=z[field];kind='s' if signed else '';lit=f"{bits}'{kind}d{abs(v)}";lit=('-' if v<0 else '')+lit
        lookup+=f"        5'd{z['index']}: {name}={lit};\n"
    lookup+=f"        default: {name}=0;\n    endcase end\nendfunction\n"
(ROOT/'src/portrait/vision_ipm_lut.vh').write_text(lookup,encoding='ascii')
report=dict(image_width=width,image_height=height,measured_mount=dict(height_mm=H,forward_from_nose_mm=-nose,left_from_centre_mm=left,centre_ground_from_nose_mm=centre,mechanical_pitch_deg=cfg['VEHICLE_CAMERA_PITCH_TENTHS']/10,effective_projection_pitch_deg=math.degrees(pitch)),provisional_intrinsics=dict(horizontal_fov_deg=hfov,fx=focal,fy=focal,cx=319.5,cy=239.5,yaw_deg=0,roll_deg=0,distortion=[0]*5),intrinsics_calibrated=False,measured_ground_calibration=dict(forward_rows_mm=[200,400,600],upper_left=[600,100],middle_three_forward_mm=400,lower_right=[200,-100],pixel_coordinates_supplied=False,photo_files_available=False,note='Ground positions recorded. Matching native pixels are unavailable in this workspace; these are not fitted correspondences.'),signal_units=dict(road_near='road centre at 210 mm ahead of nose, mm; image right positive',road_heading='ground centre slope times 110 mm; image right positive'),assumptions=['60 degree horizontal field of view and square pixels, retained from previous setup','zero yaw, roll and lens distortion','flat ground','black road on light floor'],control_method='per-scanline inverse ray/ground intersection; robust ground centres, metric slope and 210 mm lookahead extrapolation',control_uses_intrinsics=True,unobservable_region_below_mm=150,scanlines=rows)
report['physical_road_widths_mm']=dict(outer=outer_width,inner=inner_width,s_bend=measured['s_bend_width_mm'])
report['boundary_evidence_max_forward_mm']=boundary_max
(ROOT/'docs/camera_parameters.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(effective_pitch_deg=math.degrees(pitch),near=rows[0],far=rows[-1]),indent=2))
