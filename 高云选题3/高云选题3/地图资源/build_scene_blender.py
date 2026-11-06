"""Build geometry-only Blender/GLB assets and a vehicle-camera rig.

Run after generate_map_artifacts.py:
  blender -b --python build_scene_blender.py

The saved GLB contains only the map geometry. Blender's glTF exporter performs
the standard Z-up to glTF Y-up conversion. Cameras, lights and debug semantics
remain in the .blend and are excluded from the GLB.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parent
CFG = json.loads((ROOT / "scene-config.json").read_text(encoding="utf-8"))
MANIFEST = json.loads((ROOT / "geometry_manifest.json").read_text(encoding="utf-8"))
COLOR = {"ground": (.18,.20,.22), "road": (.10,.12,.14), "building": (.52,.56,.58), "obstacle": (.88,.46,.08), "water": (.04,.42,.62), "low": (.75,.15,.12), "door": (.12,.16,.18), "frame": (.85,.85,.85), "boundary": (.30,.32,.34), "wall": (.36,.42,.44), "traffic": (.78,.18,.12), "barrier": (.92,.66,.10)}


def collection(name):
    found = bpy.data.collections.get(name)
    if found: return found
    found = bpy.data.collections.new(name); bpy.context.scene.collection.children.link(found); return found


def link_only(obj, target):
    for col in list(obj.users_collection): col.objects.unlink(obj)
    target.objects.link(obj)


def material(name):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.diffuse_color = (*COLOR.get(name, (.65,.65,.65)), 1)
    m.use_nodes = True
    principled = m.node_tree.nodes.get("Principled BSDF")
    if principled:
        principled.inputs["Base Color"].default_value = m.diffuse_color
        principled.inputs["Roughness"].default_value = .78
    return m


def build_geometry(target, predicate=lambda item: True):
    for it in MANIFEST["items"]:
        if not it.get("render", True) or not predicate(it): continue
        if it["type"] == "box":
            bpy.ops.mesh.primitive_cube_add(location=it["center"])
            obj = bpy.context.object; obj.dimensions = it["size"]
            bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        elif it["type"] == "cylinder":
            bpy.ops.mesh.primitive_cylinder_add(vertices=48, radius=it["radius"], depth=it["height"], location=it["center"])
            obj = bpy.context.object
        else:
            raise ValueError(f"Unsupported manifest geometry: {it['type']}")
        obj.name = it["id"]; obj.data.materials.append(material(it["color"]))
        for key in ("id", "collision", "lidar_visible", "source_id", "semantic", "approximation"):
            if it.get(key) is not None: obj[key] = it[key]
        link_only(obj, target)


def create_camera(target, name, center, yaw, camera_cfg):
    data = bpy.data.cameras.new(name); obj = bpy.data.objects.new(name, data); target.objects.link(obj)
    obj.location = center
    direction = Vector((math.cos(yaw), math.sin(yaw), 0.0))
    obj.rotation_euler = direction.to_track_quat('-Z', 'Y').to_euler()
    data.type = "PERSP"; data.lens_unit = "FOV"
    # Blender's angle is horizontal FOV. Derive it from the configured
    # vertical FOV and image aspect when that is what the robot config gives.
    width, height = camera_cfg.get("camera_resolution", camera_cfg.get("resolution", [960, 540]))
    vertical = camera_cfg.get("camera_fov_vertical_deg", camera_cfg.get("vertical_fov_deg"))
    if vertical is not None:
        horizontal = 2 * math.degrees(math.atan(math.tan(math.radians(vertical) / 2) * width / height))
    else:
        horizontal = camera_cfg.get("fov_degrees", camera_cfg.get("horizontal_fov_deg", 80))
    data.angle = math.radians(horizontal)
    data.clip_start = .03; data.clip_end = 100
    obj["export_to_glb"] = False
    return obj


def build_camera_rig(target):
    start = next(p for p in CFG["semantic_points"] if p["id"] == "start")
    x, y, _, yaw = start["pose"]
    robot = CFG["robot"]; camera_cfg = robot.get("camera", robot)
    rig = bpy.data.objects.new("vehicle_camera_rig", None); target.objects.link(rig)
    rig.location = (x, y, 0); rig.rotation_euler[2] = yaw; rig["base_link_forward"] = "+X"
    rig["source"] = "scene-config.json robot camera fields"
    h = robot["camera_z"]; baseline = camera_cfg.get("stereo_baseline_m", camera_cfg.get("baseline", robot.get("stereo_baseline", .12)))
    forward = camera_cfg.get("forward_offset_m", robot["length"]/2)
    # Cameras are world-positioned initially; parent inverse preserves this pose.
    cx, cy = x+forward*math.cos(yaw), y+forward*math.sin(yaw)
    for name, side in (("camera_rgb", 0), ("camera_left", baseline/2), ("camera_right", -baseline/2)):
        c = create_camera(target, name, (cx-side*math.sin(yaw), cy+side*math.cos(yaw), h), yaw, camera_cfg)
        c.parent = rig; c.matrix_parent_inverse = rig.matrix_world.inverted()
        if name == "camera_rgb": bpy.context.scene.camera = c
    return camera_cfg


def build_debug(target):
    for p in CFG["semantic_points"]:
        obj = bpy.data.objects.new(p["id"], None); obj.empty_display_type = "ARROWS"; obj.empty_display_size = .15
        obj.location = p["pose"][:3]; obj.rotation_euler[2] = p["pose"][3]
        obj["semantic_type"] = p["type"]; obj["debug_only"] = True; target.objects.link(obj)
    target.hide_render = True


def build():
    bpy.ops.object.select_all(action="SELECT"); bpy.ops.object.delete(use_global=False)
    geometry = collection("map_geometry"); printable = collection("printable_thin_wall_layout"); rig = collection("camera_rig"); debug = collection("debug_semantics"); lighting = collection("lighting")
    build_geometry(geometry); build_geometry(printable, lambda item: item.get("printable", False)); camera_cfg = build_camera_rig(rig); build_debug(debug)
    scene = bpy.context.scene; scene["source_config"] = str((ROOT/"scene-config.json").resolve()); scene["coordinate_frame"] = "x=east,y=north,z=up"
    scene.unit_settings.system = "METRIC"; scene.unit_settings.scale_length = 1
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    resolution = camera_cfg.get("camera_resolution", [CFG["robot"].get("camera_width", 960), CFG["robot"].get("camera_height", 540)])
    scene.render.resolution_x, scene.render.resolution_y = resolution
    scene.render.resolution_percentage = 100
    scene.world.use_nodes = True; background = scene.world.node_tree.nodes.get("Background")
    background.inputs["Color"].default_value = (.08,.10,.12,1); background.inputs["Strength"].default_value = .65
    light_data = bpy.data.lights.new("sun", "SUN"); light_data.energy = 2.2
    light_obj = bpy.data.objects.new("sun", light_data); lighting.objects.link(light_obj); light_obj.rotation_euler = (math.radians(28),math.radians(-22),math.radians(-35))
    bpy.ops.wm.save_as_mainfile(filepath=str(ROOT / "community_fire_map.blend"))
    bpy.ops.object.select_all(action="DESELECT")
    for obj in geometry.objects: obj.select_set(True)
    bpy.context.view_layer.objects.active = next(iter(geometry.objects))
    bpy.ops.export_scene.gltf(filepath=str(ROOT / "community_fire_map.glb"), export_format="GLB", use_selection=True, export_yup=True, export_cameras=False, export_lights=False, export_extras=True)
    bpy.ops.object.select_all(action="DESELECT")
    for obj in printable.objects: obj.select_set(True)
    if printable.objects:
        bpy.context.view_layer.objects.active = next(iter(printable.objects))
        bpy.ops.export_scene.gltf(filepath=str(ROOT / "community_fire_printable_thin_wall.glb"), export_format="GLB", use_selection=True, export_yup=True, export_cameras=False, export_lights=False, export_extras=True)
    print("Saved community_fire_map.blend, community_fire_map.glb and community_fire_printable_thin_wall.glb")


if __name__ == "__main__": build()
