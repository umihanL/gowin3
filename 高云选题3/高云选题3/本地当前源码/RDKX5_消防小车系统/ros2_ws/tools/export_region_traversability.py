"""Export the existing FPGA planner ROM; never regenerate map geometry."""
import argparse
import hashlib
import json
import re
from pathlib import Path


def export(fpga_root, catalog_path, output):
    source = fpga_root / 'src/navigation/map_plan_2cm.mem'
    data = source.read_bytes()
    cells = bytes(int(word, 16) for word in data.split())
    if len(cells) != 176 * 160 or any(value not in (0, 1) for value in cells):
        raise ValueError('invalid_fpga_planner_rom')
    catalog = json.loads(catalog_path.read_text(encoding='utf-8'))
    metadata = json.loads((fpga_root / 'src/navigation/map_metadata.json').read_text(encoding='utf-8'))
    graph_path = fpga_root / 'src/navigation/nav_centerline_init.vh'
    config = (fpga_root / 'src/navigation/nav_centerline_config.vh').read_text(encoding='utf-8')
    node_count = int(re.search(r'`define ROAD_NODES\s+(\d+)', config).group(1))
    graph = {int(index): int(word, 16) for index, word in
             re.findall(r"graph\[(\d+)\]\s*=\s*\d+'h([0-9a-fA-F]+)", graph_path.read_text(encoding='utf-8'))}
    if any(index not in graph for index in range(node_count)):
        raise ValueError('incomplete_fpga_execution_graph')
    graph_cells = sorted({((graph[index] & 0x1ffff) >> 9, ((graph[index] >> 17) & 0x1ffff) >> 9)
                          for index in range(node_count)})
    graph_cells = [list(cell) for cell in graph_cells if 0 <= cell[0] < 176 and 0 <= cell[1] < 160 and
                   cells[cell[1] * 176 + cell[0]] == 0]
    heading_path = fpga_root / 'src/navigation/nav_centerline_heading.vh'
    heading_text = heading_path.read_text(encoding='utf-8')
    headings = {int(index): int(word, 16) for index, word in
                re.findall(r"start_tangent\[(\d+)\]\s*=\s*9'h([0-9a-fA-F]+)", heading_text)}
    if any(index not in headings for index in range(node_count)):
        raise ValueError('incomplete_fpga_start_tangent_rom')
    declared_graph = re.search(r'Graph SHA256:\s*([0-9a-f]{64})', heading_text)
    if declared_graph and declared_graph.group(1) != hashlib.sha256(graph_path.read_bytes()).hexdigest():
        raise ValueError('start_tangent_rom_belongs_to_different_execution_graph')
    start_samples = []
    bits = int(re.search(r'`define ROAD_BITS\s+(\d+)', config).group(1))
    invalid = (1 << bits) - 1
    directed_nodes = []
    for node in range(node_count):
        heading = headings[node] - 512 if headings[node] & 256 else headings[node]
        word = graph[node]
        edges = []
        for slot in range(3):
            edge = word >> (34 + slot * (bits + 4))
            destination, cost = edge & invalid, (edge >> bits) & 15
            if destination != invalid:
                if destination >= node_count or cost <= 0:
                    raise ValueError('invalid_fpga_directed_edge')
                edges.append([destination, cost])
        directed_nodes.append([word & 0x1ffff, (word >> 17) & 0x1ffff, heading, edges])
        if heading == 255:
            continue
        if not -180 <= heading <= 180:
            raise ValueError('invalid_fpga_start_tangent')
        x_q8, y_q8 = graph[node] & 0x1ffff, (graph[node] >> 17) & 0x1ffff
        start_samples.append(dict(node=node, cell=[x_q8 >> 9, y_q8 >> 9],
                                  heading_deg=heading, xy_q8=[x_q8, y_q8]))
    runs = []
    for y in range(160):
        x = 0
        while x < 176:
            if cells[y * 176 + x]:
                x += 1
                continue
            start = x
            while x < 176 and not cells[y * 176 + x]:
                x += 1
            runs.append([y, start, x - start])
    result = dict(schema_version=1, source='ov5640/src/navigation/map_plan_2cm.mem',
                  width_cells=176, height_cells=160, cell_cm=2, free_value=0,
                  planner_mem_sha256=hashlib.sha256(data).hexdigest(),
                  cells_sha256=hashlib.sha256(cells).hexdigest(),
                  map_signature=catalog['map_signature'],
                  raw_map_sha256=catalog['source_hashes']['src/navigation/map_raw_1cm.mem'],
                  map_sha256=hashlib.sha256((fpga_root / 'src/navigation/road_centerlines.json').read_bytes()).hexdigest(),
                  planner_inflation_m=metadata['inflation_radius_cm'] / 100.,
                  reference=metadata['path_reference'], free_runs=runs,
                  graph_rom_sha256=hashlib.sha256(graph_path.read_bytes()).hexdigest(),
                  graph_cells_sha256=hashlib.sha256(bytes(value for cell in graph_cells for value in cell)).hexdigest(),
                  graph_cells=graph_cells,
                  heading_rom_sha256=hashlib.sha256(heading_path.read_bytes()).hexdigest(),
                  start_heading_samples_sha256=hashlib.sha256(json.dumps(start_samples, separators=(',', ':'),
                      ensure_ascii=True).encode('ascii')).hexdigest(),
                  start_heading_samples=start_samples,
                  directed_nodes=directed_nodes,
                  directed_nodes_sha256=hashlib.sha256(json.dumps(directed_nodes, separators=(',', ':')).encode('ascii')).hexdigest(),
                  directed_nodes_fields=['x_cm_q8', 'y_cm_q8', 'heading_deg', 'edges_destination_cost_cm'],
                  start_heading_sample_order='ascending ROM node; preserve this order for lowest-node ties',
                  start_heading_sample_coordinates='xy_q8 is image cm Q8 (x right/y down); heading_deg is world CCW (x right/y up)',
                  note='Connectivity precheck only. FPGA A* and its turn constraints remain authoritative.')
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--fpga-root', required=True, type=Path)
    parser.add_argument('--catalog', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = export(args.fpga_root, args.catalog, args.output)
    print(json.dumps(dict(free_cells=sum(run[2] for run in result['free_runs']),
                         cells_sha256=result['cells_sha256'], output=str(args.output))))
