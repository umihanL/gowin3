"""Build the editable, textured thin-wall rescue maze in Blender 4.5 LTS.

Run: blender --background --python build_rescue_maze.py
Optional arguments after --: --skip-render
All distances are metres. The layout is an engineering design, not an official
competition specification. GLB is a static map snapshot; Blender contains the
barrier and traffic-light demonstration animations.
"""
from __future__ import annotations
import json
import math
import sys
from pathlib import Path
import bpy
from mathutils import Vector, Quaternion

ROOT = Path(__file__).resolve().parent
CFG = json.loads((ROOT / "rescue_maze_config.json").read_text(encoding="utf-8"))
TEX = ROOT / "assets" / "textures"
OUT = ROOT / "renders"
OUT.mkdir(exist_ok=True)
COLS = {}
COLLIDERS = []


def coll(name):
    if name not in COLS:
        c = bpy.data.collections.new(name)
        bpy.context.scene.collection.children.link(c)
        COLS[name] = c
    return COLS[name]


def put(obj, name, group):
    obj.name = name
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    coll(group).objects.link(obj)
    return obj


def mat(name, color, rough=.75, metallic=0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    m.diffuse_color = (*color, 1)
    p = m.node_tree.nodes.get("Principled BSDF")
    p.inputs["Base Color"].default_value = (*color, 1)
    p.inputs["Roughness"].default_value = rough
    p.inputs["Metallic"].default_value = metallic
    return m


def image_mat(name, path, emission=.08):
    if not path.exists():
        raise FileNotFoundError(path)
    m = mat(name, (.8, .8, .8))
    nt = m.node_tree
    p = nt.nodes.get("Principled BSDF")
    tex = nt.nodes.new("ShaderNodeTexImage")
    tex.image = bpy.data.images.load(str(path), check_existing=True)
    tex.image.colorspace_settings.name = "sRGB"
    tex.image.pack()
    tex.image.filepath = bpy.path.relpath(str(path), start=str(ROOT))
    tex.extension = "REPEAT"
    nt.links.new(tex.outputs["Color"], p.inputs["Base Color"])
    if emission:
        nt.links.new(tex.outputs["Color"], p.inputs["Emission Color"])
        p.inputs["Emission Strength"].default_value = emission
    return m


def box(name, loc, dims, material, group, bevel=0, collision=False):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    o = put(bpy.context.object, name, group)
    o.dimensions = dims
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    o.data.materials.append(material)
    if bevel:
        mod = o.modifiers.new("Edge_rounding", "BEVEL")
        mod.width = bevel
        mod.segments = 2
        o.modifiers.new("Weighted_normals", "WEIGHTED_NORMAL")
    o["collision"] = collision
    if collision:
        COLLIDERS.append({"id": name, "shape": "box", "center": list(loc), "size": list(dims)})
    return o


def cyl(name, loc, radius, depth, material, group, vertices=32, rotation=None, collision=False):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius, depth=depth, location=loc)
    o = put(bpy.context.object, name, group)
    if rotation:
        o.rotation_euler = rotation
    o.data.materials.append(material)
    o["collision"] = collision
    if collision:
        COLLIDERS.append({"id": name, "shape": "cylinder", "center": list(loc), "radius": radius, "height": depth})
    return o


def quad(name, vertices, material, group, uv=None):
    mesh = bpy.data.meshes.new(name + "_mesh")
    mesh.from_pydata(vertices, [], [(0, 1, 2, 3)])
    mesh.materials.append(material)
    mesh.uv_layers.new(name="Print_UV")
    uvs = uv or [(0, 0), (1, 0), (1, 1), (0, 1)]
    for i, loop in enumerate(mesh.uv_layers.active.data):
        loop.uv = uvs[i]
    o = bpy.data.objects.new(name, mesh)
    coll(group).objects.link(o)
    o["collision"] = False
    return o


def floor_quad(name, bounds, z, material, group, uv=None):
    x0, x1, y0, y1 = bounds
    return quad(name, [(x0,y0,z),(x1,y0,z),(x1,y1,z),(x0,y1,z)], material, group, uv)


def text(name, content, pos, size, material, group, rotation=(0,0,0)):
    curve = bpy.data.curves.new(name, "FONT")
    curve.body = content
    curve.size = size
    curve.align_x = "CENTER"
    curve.align_y = "CENTER"
    curve.extrude = .0005
    o = bpy.data.objects.new(name, curve)
    coll(group).objects.link(o)
    o.location = pos
    o.rotation_euler = rotation
    curve.materials.append(material)
    return o


def wall(w):
    a, b = Vector((*w["a"],0)), Vector((*w["b"],0))
    d = b-a
    length = d.length
    u = d.normalized()
    normal = Vector((u.y, -u.x, 0))
    h = w.get("height", CFG["wall_height"])
    thickness = CFG["wall_thickness"]
    o = box(w["id"] + "_thin_panel", (a+b)/2+Vector((0,0,h/2)),
            (length, thickness, h), M["wall"], "02_薄墙结构_THIN_WALLS", .005, True)
    o.rotation_euler[2] = math.atan2(d.y,d.x)
    # The collision inventory describes the wall by endpoints, avoiding rotated AABBs.
    COLLIDERS[-1] = {"id":o.name, "shape":"wall", "a":w["a"], "b":w["b"], "thickness":thickness, "height":h}
    o["print_height_m"] = h-.065
    o["art_source"] = f"facade_{w['art']}.png"
    count = max(1, math.ceil(length / (2*(h-.06))))
    for side in (-1,1):
        for i in range(count):
            p0 = a + d*(i/count) + u*.014 + normal*side*(thickness/2+.0015)
            p1 = a + d*((i+1)/count) - u*.014 + normal*side*(thickness/2+.0015)
            if side < 0:
                p0,p1 = p1,p0
            vertices = [(p0.x,p0.y,.028),(p1.x,p1.y,.028),(p1.x,p1.y,h-.032),(p0.x,p0.y,h-.032)]
            # Crop short panels to preserve people/window proportions instead of squeezing.
            span = min(1, (p1-p0).length/(2*(h-.06)))
            u0, u1 = (1-span)/2, (1+span)/2
            panel = quad(f"{w['id']}_print_{side}_{i+1:02}", vertices, M[w["art"]], "03_居民楼双面壁纸_FACADE_PRINTS", [(u0,0),(u1,0),(u1,1),(u0,1)])
            panel["texture_file"] = f"assets/textures/facade_{w['art']}.png"
            panel["printed_width_m"] = (p1-p0).length
            panel["printed_height_m"] = h-.060
    return o


def sign(name, xy, z, width, height, material, facing_deg, group):
    # Image normal +Y at angle 0; supports signs approached from every direction.
    yaw = math.radians(facing_deg)
    right = Vector((-math.cos(yaw),-math.sin(yaw),0))
    normal = Vector((-math.sin(yaw),math.cos(yaw),0))
    center = Vector((*xy,z))
    body = box(name+"_board",center,(width,.038,height),M["white"],group,.018)
    body.rotation_euler.z=yaw
    for side in [-1,1]:
        c = center+normal*.021*side
        r = right*side*width/2
        verts = [c-r-Vector((0,0,height/2)),c+r-Vector((0,0,height/2)),c+r+Vector((0,0,height/2)),c-r+Vector((0,0,height/2))]
        quad(name+f"_print_{side}",[tuple(v) for v in verts],material,group)
    return body


def make_gate():
    g=CFG["gate"]
    x,y,z=g["pivot"]
    group="05_可升降道闸_BARRIER_ANIMATED"
    box("Gate_motor_cabinet",(x,y-.09,.27),(.20,.22,.54),M["yellow"],group,.025,True)
    box("Gate_motor_dark_cap",(x,y-.09,.555),(.215,.235,.03),M["dark"],group,.006)
    cyl("Gate_axle",(x,y,z),.07,.28,M["metal"],group,rotation=(0,math.pi/2,0))
    pivot=bpy.data.objects.new("GATE_PIVOT_道闸转轴_播放时间轴",None)
    coll(group).objects.link(pivot)
    pivot.location=(x,y,z)
    pivot.empty_display_type="ARROWS"
    pivot.empty_display_size=.18
    pivot["operation"]="Rotate local X: 0 degrees = closed; 82 degrees = open."
    pivot["pivot_height_m"]=z
    arm=box("Gate_arm_yellow_black",(0,g["arm_length"]/2,0),(.075,g["arm_length"],g["arm_thickness"]),M["yellow"],group,.009)
    arm.parent=pivot
    arm["dynamic_obstacle"]=True
    arm["simulation_joint_needed"]="revolute hinge X; animation is not a motor controller"
    for side in [-1,1]:
        xx=side*.039
        vs=[(xx,.025,-.038),(xx,g["arm_length"]-.025,-.038),(xx,g["arm_length"]-.025,.038),(xx,.025,.038)]
        if side<0:
            vs=[vs[1],vs[0],vs[3],vs[2]]
        p=quad("Gate_striped_print_"+str(side),vs,M["stripe"],group)
        p.parent=pivot
    cap=box("Gate_arm_end_reflector",(0,g["arm_length"]-.035,0),(.082,.04,.082),M["red"],group,.005)
    cap.parent=pivot
    for frame,angle in g["keyframes"]:
        pivot.rotation_euler.x=math.radians(angle)
        pivot.keyframe_insert(data_path="rotation_euler",frame=frame)
    box("Gate_receiver",(x,6.62,.24),(.09,.085,.48),M["dark"],group,.01)
    return pivot


def make_lights():
    group="06_红绿灯_TRAFFIC_LIGHTS"
    for spec in CFG["traffic_lights"]:
        x,y=spec["position"]
        yaw=math.radians(spec["facing_deg"])
        # Front is along the given world heading, housing local -Y.
        forward=Vector((math.cos(yaw),math.sin(yaw),0))
        box(spec["id"]+"_foot",(x,y,.035),(.19,.19,.07),M["dark"],group,.012)
        cyl(spec["id"]+"_pole",(x,y,.54),.035,1.08,M["metal"],group,collision=True)
        body=box(spec["id"]+"_housing",(x,y,1.035),(.19,.16,.50),M["dark"],group,.022)
        body.rotation_euler.z=yaw+math.pi/2
        for k,color in enumerate(("red","amber","green")):
            z=1.185-k*.15
            p=Vector((x,y,z))+forward*.088
            # Cylinder's axis is Z, align it with the lens normal.
            o=cyl(spec["id"]+"_lens_"+color,p,.054,.018,M[color],group)
            o.rotation_euler=forward.to_track_quat("Z","Y").to_euler()
            lens=M[color].copy()
            lens.name=spec["id"]+"_animated_"+color
            o.data.materials[0]=lens
            node=lens.node_tree.nodes.get("Principled BSDF")
            rgb={"red":(1,.015,.005),"amber":(1,.48,.005),"green":(.015,.8,.11)}[color]
            for frame,state in [(1,"red"),(95,"red"),(96,"green"),(220,"green"),(221,"amber"),(250,"amber"),(251,"red"),(300,"red")]:
                on=color==state
                node.inputs["Base Color"].default_value=(*(rgb if on else tuple(v*.075 for v in rgb)),1)
                node.inputs["Emission Color"].default_value=(*rgb,1)
                node.inputs["Emission Strength"].default_value=3.0 if on else 0
                node.inputs["Base Color"].keyframe_insert(data_path="default_value",frame=frame)
                node.inputs["Emission Strength"].keyframe_insert(data_path="default_value",frame=frame)
            # Small top visor, oriented out from the housing.
            visor=box(spec["id"]+"_visor_"+color,(x+forward.x*.045,y+forward.y*.045,z+.066),(.16,.21,.018),M["dark"],group,.004)
            visor.rotation_euler.z=yaw+math.pi/2


def make_construction():
    c=CFG["construction"]
    x,y=c["center"]
    group="07_施工路障_DETOUR"
    box("ROAD_CLOSED_collision_body",(x,y,.37),(c["width"],c["thickness"],.27),M["yellow"],group,.013,True)
    for xx in [x-.90,x+.90]:
        box("Roadblock_support",(xx,y,.20),(.075,.14,.40),M["metal"],group,.005)
        box("Roadblock_foot",(xx,y,.025),(.28,.38,.05),M["dark"],group,.01)
        bulb=cyl("Roadblock_warning_beacon",(xx,y,.565),.049,.10,M["amber"],group)
    for side in [-1,1]:
        xx0,xx1=x-c["width"]/2+.015,x+c["width"]/2-.015
        verts=[(xx0,y+side*.082,.25),(xx1,y+side*.082,.25),(xx1,y+side*.082,.49),(xx0,y+side*.082,.49)]
        if side>0:
            verts=[verts[1],verts[0],verts[3],verts[2]]
        quad("Roadblock_hazard_stripes_"+str(side),verts,M["stripe"],group)
    for i,(cx,cy) in enumerate(c["cones"]):
        box(f"Cone_{i}_base",(cx,cy,.016),(.24,.24,.032),M["dark"],group,.012)
        bpy.ops.mesh.primitive_cone_add(vertices=32,radius1=.10,radius2=.024,depth=.30,location=(cx,cy,.18))
        o=put(bpy.context.object,f"Cone_{i}_orange",group)
        o.data.materials.append(M["orange"])
        COLLIDERS.append({"id":o.name,"shape":"cylinder","center":[cx,cy,.17],"radius":.12,"height":.34})
        bpy.ops.mesh.primitive_cone_add(vertices=32,radius1=.075,radius2=.058,depth=.068,location=(cx,cy,.178))
        put(bpy.context.object,f"Cone_{i}_white_band",group).data.materials.append(M["white"])
    sx,sy=c["sign"]
    box("Detour_sign_foot",(sx,sy,.02),(.32,.22,.04),M["dark"],group,.008)
    cyl("Detour_sign_post",(sx,sy,.29),.022,.58,M["metal"],group)
    sign("DETOUR_turn_right",(sx,sy),.79,.45,.45,M["construction"],180,group)
    COLLIDERS.append({"id":"Detour_sign","shape":"box","center":[sx,sy,.51],"size":[.45,.22,1.02]})


def camera(name, pos, target, ortho=None, lens=25):
    d=bpy.data.cameras.new(name)
    o=bpy.data.objects.new(name,d)
    coll("09_相机_CAMERAS").objects.link(o)
    o.location=pos
    o.rotation_euler=(Vector(target)-Vector(pos)).to_track_quat("-Z","Y").to_euler()
    d.clip_start=.025
    d.clip_end=250
    d.lens=lens
    if ortho:
        d.type="ORTHO"
        d.ortho_scale=ortho
    return o


def build():
    global M
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for c in list(bpy.data.collections):
        if c.name == "Collection":
            bpy.data.collections.remove(c)
    M={
        "wall":mat("Thin_board_ivory_edges",(.89,.87,.80)),
        "base":mat("Map_base_charcoal",(.065,.09,.11)),
        "white":mat("Warm_white",(.88,.9,.88)),
        "dark":mat("Graphite",(.026,.042,.054)),
        "yellow":mat("Safety_yellow",(.95,.65,.025)),
        "metal":mat("Brushed_metal",(.16,.2,.23),.4,.65),
        "red":mat("Signal_red",(.7,.01,.005),.3),
        "amber":mat("Signal_amber",(1,.42,.005),.3),
        "green":mat("Signal_green",(.005,.65,.08),.3),
        "orange":mat("Cone_orange",(.95,.12,.008)),
        "blue":mat("Parking_blue",(.02,.28,.64)),
        "studio":mat("Studio_background",(.48,.52,.53))
    }
    for key,file in [("calm","facade_calm.png"),("people","facade_people.png"),("fire","facade_fire.png"),("stripe","barrier_stripes.png"),("parking","parking_sign.png"),("construction","construction_sign.png")]:
        M[key]=image_mat("PRINT_"+key,TEX/file)
    M["road"]=image_mat("PRINT_12x8m_road_wallpaper",ROOT/"assets"/"road_wallpaper.png",.015)
    box("Map_base_12x8m",(6,4,-.085),(12.24,8.24,.17),M["base"],"01_地面_ROAD_WALLPAPER",.035)
    floor_quad("Road_print_12x8m",[0,12,0,8],.001,M["road"],"01_地面_ROAD_WALLPAPER")
    for w in CFG["walls"]:
        wall(w)
    # Small corner markings identify the blocks while facades remain flat prints.
    for label,x,y in [("A",3.4,2.6),("B",8.5,2.6),("C",8.5,5.4)]:
        text("Courtyard_label_"+label,"BLOCK "+label,(x,y,.009),.14,M["dark"],"04_停车区_PARKING")
    group="04_停车区_PARKING"
    cyl("Parking_P_sign_pole",(4.98,6.77,.42),.032,.84,M["metal"],group,collision=True)
    box("Parking_P_sign_foot",(4.98,6.77,.025),(.19,.19,.05),M["dark"],group,.008)
    sign("P_PARKING_专用停车区",(4.98,6.77),1.04,.39,.44,M["parking"],-90,group)
    gate=make_gate()
    make_lights()
    make_construction()
    # Metric vehicle footprint marker: hidden from render, editable camera rig.
    group="08_小车参考与语义_ROBOT_REFERENCE"
    rig=bpy.data.objects.new("ROBOT_CAMERA_RIG_移动车载视角",None)
    coll(group).objects.link(rig)
    rig.location=(6,3.88,0)
    rig.empty_display_type="CUBE"
    rig.empty_display_size=.24
    rig["robot_length_m"]=CFG["robot"]["length"]
    rig["robot_width_m"]=CFG["robot"]["width"]
    rig["camera_height_m"]=CFG["robot"]["camera_height"]
    rig["not_a_physics_robot"]=True
    # Cameras are parented with explicit local transform to avoid depsgraph timing.
    camera_z=CFG["robot"]["camera_height"]
    car=camera("CAM_02_车载火场_0.48m",(6,3.88,camera_z),(8.6,4.78,camera_z+.07),lens=22)
    car.parent=rig
    car.location=(0,0,camera_z)
    car["horizontal_fov_degrees"]=math.degrees(2*math.atan(36/(2*22)))
    for name,xy in CFG["semantic_points"].items():
        e=bpy.data.objects.new(name,None)
        coll(group).objects.link(e)
        e.location=(*xy,.025)
        e.empty_display_type="ARROWS"
        e.empty_display_size=.18
        e.hide_render=True
    overview=camera("CAM_01_全景",(17,-12.5,16),(6,4,.08),ortho=15.8)
    parking_cam=camera("CAM_03_停车道闸_0.48m",(6.30,5.90,camera_z),(3.0,5.95,camera_z+.02),lens=18)
    top=camera("CAM_04_正俯视",(6,4,18),(6,4,0),ortho=13.0)
    scene=bpy.context.scene
    scene.unit_settings.system="METRIC"
    scene.unit_settings.scale_length=1
    scene.render.engine="CYCLES"
    scene.cycles.samples=32
    scene.cycles.use_denoising=True
    scene.render.image_settings.file_format="PNG"
    scene.render.film_transparent=False
    scene.render.fps=30
    scene.frame_start=1
    scene.frame_end=300
    scene.view_settings.view_transform="AgX"
    scene.world.use_nodes=True
    bg=scene.world.node_tree.nodes.get("Background")
    bg.inputs["Color"].default_value=(.55,.63,.72,1)
    bg.inputs["Strength"].default_value=.65
    ld=bpy.data.lights.new("Sun_daylight","SUN")
    ld.energy=2.0
    ld.angle=math.radians(12)
    sun=bpy.data.objects.new("Sun_daylight",ld)
    coll("10_灯光与背景_LIGHTING").objects.link(sun)
    sun.rotation_euler=(math.radians(25),math.radians(-18),math.radians(-35))
    ad=bpy.data.lights.new("Large_softbox","AREA")
    ad.energy=1300
    ad.shape="DISK"
    ad.size=9
    ao=bpy.data.objects.new("Large_softbox",ad)
    coll("10_灯光与背景_LIGHTING").objects.link(ao)
    ao.location=(4,1,10)
    box("Studio_floor",(6,4,-.23),(200,200,.10),M["studio"],"10_灯光与背景_LIGHTING")
    for frame,name in [(1,"CLOSED / RED"),(96,"GREEN"),(110,"GATE OPEN"),(145,"OVERVIEW"),(205,"CLOSING SOON"),(221,"AMBER"),(251,"CLOSED / RED")]:
        scene.timeline_markers.new(name,frame=frame)
    scene["map_version"]="rescue_maze_textured_v1"
    scene["dimension_source"]=CFG["dimension_note"]
    scene["wall_model"]="Thin printed panels, 0.06 m thick; no solid house meshes."
    scene["simulation_status"]="Editable visual environment and demonstration animations. No robot dynamics/controller."
    scene["default_frame"]=145
    scene["old_exports_note"]="Previous community_fire_map.wbt/.sdf describe the old baseline, not this maze."
    readme=bpy.data.texts.new("使用说明_README")
    readme.write("社区消防救援薄墙街区\n\n12×8米工程假设，非赛方规定尺寸。\n默认145帧道闸打开、绿灯；第1帧道闸落下、红灯。空格播放1—300帧演示。\n选 GATE_PIVOT_道闸转轴_播放时间轴，可编辑局部X旋转关键帧。\nCAM_01 全景，CAM_02 车载火场，CAM_03 停车道闸，CAM_04 俯视。\n所有贴图已打包；按Z选材质预览可以看到壁纸。\n03集合是双面壁纸，02集合是6cm厚可碰撞薄墙实体。\n当前场景仅建模与视觉演示；没有小车动力学或控制器。\nGLB导出145帧静态材质和道闸姿态。旧SDF/WBT不是这版地图。\n外部PNG和CLAUDE_TEXTURE_HANDOFF.md放在工程目录中。\n")
    scene.frame_set(145)
    scene.camera=overview
    scene.render.resolution_x=1600
    scene.render.resolution_y=1100
    scene.render.resolution_percentage=100
    # Open directly into a textured, framed overview.
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type=="VIEW_3D":
                space=area.spaces.active
                space.shading.type="MATERIAL"
                space.overlay.show_extras=False
                space.clip_end=500
                space.region_3d.view_distance=16
                space.region_3d.view_location=(6,4,.1)
                space.region_3d.view_rotation=overview.rotation_euler.to_quaternion()
                space.region_3d.view_perspective="CAMERA"
    bpy.ops.object.select_all(action="DESELECT")
    bpy.context.view_layer.objects.active=None
    bpy.ops.file.pack_all()
    blend=ROOT/"community_fire_rescue_maze.blend"
    bpy.ops.wm.save_as_mainfile(filepath=str(blend))
    export_groups=[c for name,c in COLS.items() if name[:2] in ["01","02","03","04","05","06","07"]]
    for c in export_groups:
        for o in c.objects:
            o.select_set(True)
    bpy.context.view_layer.objects.active=gate
    bpy.ops.export_scene.gltf(filepath=str(ROOT/"community_fire_rescue_maze.glb"),export_format="GLB",use_selection=True,export_yup=True,export_extras=True,export_animations=False,export_current_frame=True,export_cameras=False,export_lights=False,export_apply=True)
    report={"blend":str(blend),"object_count":len(scene.objects),"mesh_count":sum(o.type=="MESH" for o in scene.objects),"packed_images":[im.name for im in bpy.data.images if im.type=="IMAGE" and im.packed_file],"wall_count":len(CFG["walls"]),"cameras":[o.name for o in scene.objects if o.type=="CAMERA"],"gate_frame_1_degrees":0,"gate_frame_145_degrees":82,"glb_static_frame":145,"colliders":COLLIDERS,"physical_simulation_verified":False}
    (ROOT/"rescue_maze_build_report.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    if "--skip-render" not in sys.argv:
        for cam,name,frame,res in [(overview,"01_overview.png",145,(1600,1100)),(car,"02_vehicle_fire_view.png",145,(1280,800)),(parking_cam,"03_parking_gate_closed.png",30,(1280,800)),(top,"04_top_plan.png",145,(1600,1100))]:
            scene.camera=cam
            scene.frame_set(frame)
            scene.render.resolution_x,scene.render.resolution_y=res
            scene.render.filepath=str(OUT/name)
            bpy.ops.render.render(write_still=True)
    print("RESCUE_MAZE_COMPLETE",blend)


if __name__=="__main__":
    build()
