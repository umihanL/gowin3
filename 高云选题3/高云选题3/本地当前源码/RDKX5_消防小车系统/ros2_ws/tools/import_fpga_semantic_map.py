"""Export topology only; never execute or rewrite the FPGA geometry generator."""
import argparse
import ast
import hashlib
import json
from pathlib import Path


def export(fpga, output):
    source = fpga/'src/navigation/road_centerlines.json'
    raw = source.read_bytes()
    model = json.loads(raw)
    tree = ast.parse((fpga/'tools/generate_centerline.py').read_text(encoding='utf8'))
    edges = None
    for statement in tree.body:
        if isinstance(statement, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'lines' for t in statement.targets):
            edges = ast.literal_eval(statement.value)
            break
    if not edges:
        raise ValueError('Source has no literal straight-edge topology')
    vertices = {name: [xy[0]*352/895, xy[1]*320/817] for name, xy in model['vertices'].items()}
    if any(a not in vertices or b not in vertices for a, b in edges):
        raise ValueError('Topology references a missing vertex')
    result = dict(schema_version=1, source='ov5640/src/navigation/road_centerlines.json',
                  map_sha256=hashlib.sha256(raw).hexdigest(), width_cm=352, height_cm=320,
                  reference='nose_midpoint', coordinates='image cm: x right, y down',
                  payload_coordinates='2 cm raster cells; x=0..175, y=0..159',
                  vertices_image_cm=vertices, straight_edges=edges, junctions=model['junctions'],
                  turn_entry_cm=model['junction_turn_entry_mm']/10,
                  turn_exit_cm=model['junction_turn_exit_mm']/10,
                  physical_geometry_verified=False,
                  note='Semantic constraints only. The FPGA graph remains the path planner. Re-export after map changes.')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--fpga-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1]/'config/extension_map.json')
    args = parser.parse_args()
    result = export(args.fpga_root, args.output)
    print(json.dumps({'vertices':len(result['vertices_image_cm']), 'map_sha256':result['map_sha256'],
                      'output':str(args.output)}, ensure_ascii=True))
