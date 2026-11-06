"""Run with Blender --background --python; inspect saved files, never render."""
from pathlib import Path
import hashlib
import json
import bpy

ROOT = Path(__file__).resolve().parents[1] / "source_data/firefighter_holdout_v1"


def read(path):
    return json.loads(Path(path).read_text(encoding="utf8"))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    plan = read(ROOT / "capture_plan.json")
    annotations = read(ROOT / "source/source_annotations/annotations.json")
    textures = {Path(t["source_name"]).stem: t for t in annotations["textures"]}
    checked, errors = [], []
    for scene_id, wallpaper in sorted({(p["scene_id"], p["wallpaper"]) for p in plan["pairs"]}):
        path = ROOT / "scene" / f"{scene_id}_{wallpaper}.blend"
        bpy.ops.wm.open_mainfile(filepath=str(path), load_ui=False)
        obj = bpy.data.objects["FrozenSourceFacade"]
        material = obj.data.materials[0]
        bsdf = next(n for n in material.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
        if not bsdf.inputs["Base Color"].is_linked:
            errors.append(path.name+": Base Color is unlinked")
            continue
        stack = [link.from_node for link in bsdf.inputs["Base Color"].links]
        visited, images = set(), []
        while stack:
            node = stack.pop()
            if node.name in visited:
                continue
            visited.add(node.name)
            if node.type == "TEX_IMAGE":
                image = node.image
                digest = hashlib.sha256(bytes(image.packed_file.data)).hexdigest() if image.packed_file else sha(bpy.path.abspath(image.filepath))
                images.append(dict(node=node.name, image=image.name, sha256=digest, packed=image.packed_file is not None))
            else:
                stack.extend(link.from_node for socket in node.inputs for link in socket.links)
        expected = sha(ROOT / "source/source_annotations" / textures[wallpaper]["local_texture"])
        passed = len(images) == 1 and images[0]["sha256"] == expected
        if not passed:
            errors.append(path.name+": Base Color image does not match planned wallpaper")
        if bsdf.inputs["Emission Strength"].default_value != 0:
            errors.append(path.name+": Unexpected emission strength")
        checked.append(dict(scene_id=scene_id, wallpaper=wallpaper, scene_file=str(path.relative_to(ROOT)).replace("\\", "/"),
                            blend_sha256=sha(path), images=images, expected_image_sha256=expected, passed=passed))
    report = dict(passed=not errors, method="Independent Blender CPU read of saved scene Base Color upstream image bytes; no rendering", 
                  capture_plan_sha256=sha(ROOT / "capture_plan.json"), manifest_sha256=sha(ROOT / "manifest.jsonl"),
                  checked=checked, errors=errors,
                  limitation="Saved scene inspection proves saved material binding; older frames without render-time binding logs require RGB visual review too.")
    (ROOT / "independent_material_audit.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf8")
    print("MATERIAL_AUDIT "+json.dumps(report, ensure_ascii=False), flush=True)
    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
