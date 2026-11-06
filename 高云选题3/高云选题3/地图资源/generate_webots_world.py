"""Export the manifest into a geometry-only Webots .wbt world.

The map configuration is Z-up (x=east, y=north, z=up); Webots is Y-up, so
centres and box dimensions are permuted at export time. No robot/controller is
included: import the user's measured mecanum robot as a separate node.
"""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
MANIFEST = json.loads((ROOT / "geometry_manifest.json").read_text(encoding="utf-8"))
COLORS = {
    "ground": (0.18, 0.20, 0.22), "road": (0.10, 0.12, 0.14),
    "building": (0.52, 0.56, 0.58), "obstacle": (0.88, 0.46, 0.08),
    "water": (0.04, 0.42, 0.62), "low": (0.75, 0.15, 0.12),
    "door": (0.12, 0.16, 0.18), "frame": (0.85, 0.85, 0.85),
    "boundary": (0.30, 0.32, 0.34), "wall": (0.36, 0.42, 0.44),
    "traffic": (0.78, 0.18, 0.12), "barrier": (0.92, 0.66, 0.10),
}


def vec(values) -> str:
    return " ".join(f"{float(v):.6g}" for v in values)


def item_node(item: dict) -> str:
    x, y, z = item["center"]
    r, g, b = COLORS.get(item["color"], (0.6, 0.6, 0.6))
    appearance = f"Appearance {{ material Material {{ diffuseColor {r:.4g} {g:.4g} {b:.4g} }} }}"
    if item["type"] == "box":
        sx, sy, sz = item["size"]
        geometry = f"Box {{ size {vec((sx, sz, sy))} }}"
        bounding = f"boundingObject Box {{ size {vec((sx, sz, sy))} }}" if item.get("collision") else ""
    else:
        geometry = f"Cylinder {{ radius {item['radius']:.6g} height {item['height']:.6g} }}"
        bounding = f"boundingObject Cylinder {{ radius {item['radius']:.6g} height {item['height']:.6g} }}" if item.get("collision") else ""
    return f'''Solid {{
  translation {x:.6g} {z:.6g} {y:.6g}
  name "{item['id']}"
  children [ Shape {{ appearance {appearance} geometry {geometry} }} ]
  {bounding}
}}'''


def main() -> None:
    nodes = "\n".join(item_node(item) for item in MANIFEST["items"] if item.get("render", True))
    world = f'''#VRML_SIM R2025a utf8
WorldInfo {{ basicTimeStep 16 coordinateSystem "ENU" }}
Viewpoint {{ position 13 9 -14 orientation 0.82 0.36 0.42 1.35 description "community fire map" }}
Background {{ skyColor [ 0.04 0.06 0.07 ] }}
DirectionalLight {{ direction -0.4 -1 -0.6 castShadows TRUE intensity 2 }}
Group {{ children [
{nodes}
] }}
'''
    (ROOT / "community_fire_map.wbt").write_text(world, encoding="utf-8")
    print(f"wrote community_fire_map.wbt ({len(MANIFEST['items'])} solids)")


if __name__ == "__main__":
    main()
