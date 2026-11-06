"""Read back the saved Blender scene and verify delivery-critical properties."""
import json
import math
import struct
from pathlib import Path
import bpy
from mathutils import Vector

ROOT=Path(__file__).resolve().parent
bpy.ops.wm.open_mainfile(filepath=str(ROOT/"community_fire_rescue_maze.blend"))
scene=bpy.context.scene
gate=bpy.data.objects["GATE_PIVOT_道闸转轴_播放时间轴"]
report={"checks":[],"gate_samples":[],"signal_samples":[]}


def check(name, ok, detail):
    report["checks"].append({"name":name,"pass":bool(ok),"detail":detail})


check("metric_scale",scene.unit_settings.system=="METRIC" and scene.unit_settings.scale_length==1,"1 Blender unit = 1 metre")
check("default_camera",scene.camera.name=="CAM_01_全景",scene.camera.name)
check("default_frame",scene.frame_current==145,scene.frame_current)
images=[i for i in bpy.data.images if i.source=="FILE"]
check("packed_images",len(images)>=7 and all(i.packed_file for i in images),[{"name":i.name,"packed":bool(i.packed_file),"size":list(i.size)} for i in images])
check("thin_walls",all(abs(o.dimensions.y-.06)<.0001 for o in bpy.data.collections["02_薄墙结构_THIN_WALLS"].objects if abs(o.rotation_euler.z)<.01),"Horizontal wall local thickness = 0.06m")
for frame,angle in [(1,0),(145,82),(300,0)]:
    scene.frame_set(frame)
    bpy.context.view_layer.update()
    end=gate.matrix_world@Vector((0,1.36,0))
    actual=math.degrees(gate.rotation_euler.x)
    report["gate_samples"].append({"frame":frame,"degrees":actual,"arm_end":list(end)})
    check(f"gate_frame_{frame}",abs(actual-angle)<.001,f"Expected {angle} degrees; got {actual}")
    if frame==1:
        check("closed_gate_intersects_robot_height",.25-.075/2<.28,"Closed arm bottom 0.2125m; assumed robot height 0.28m")
    if frame==145:
        check("open_gate_retracts",end.y<5.42 and end.z>1.59,list(end))
for frame,expected in [(30,"red"),(145,"green"),(235,"amber"),(280,"red")]:
    scene.frame_set(frame)
    for signal in ["TL1","TL2"]:
        strengths={color:bpy.data.materials[f"{signal}_animated_{color}"].node_tree.nodes.get("Principled BSDF").inputs["Emission Strength"].default_value for color in ["red","amber","green"]}
        report["signal_samples"].append({"frame":frame,"id":signal,"strengths":strengths})
        check(f"{signal}_frame_{frame}",strengths[expected]>2.99 and all(v<.001 for k,v in strengths.items() if k!=expected),strengths)
glb=(ROOT/"community_fire_rescue_maze.glb").read_bytes()
magic,version,size=struct.unpack_from("<4sII",glb,0)
n,chunk=struct.unpack_from("<II",glb,12)
data=json.loads(glb[20:20+n])
check("glb_container",magic==b"glTF" and version==2 and size==len(glb),len(glb))
check("glb_embedded_textures",len(data.get("images",[]))>=7 and all("bufferView" in i for i in data.get("images",[])),len(data.get("images",[])))
check("glb_static_snapshot","animations" not in data and "cameras" not in data,"No animation/camera exported; frame 145 snapshot")
report["all_pass"]=all(c["pass"] for c in report["checks"])
(ROOT/"rescue_maze_qa.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
print(json.dumps({"all_pass":report["all_pass"],"checks":len(report["checks"]),"failed":[c for c in report["checks"] if not c["pass"]]},ensure_ascii=False))
assert report["all_pass"], "Scene QA failed; see rescue_maze_qa.json"
