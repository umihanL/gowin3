"""Check every occupied/free cell against the actual Gowin ROM INIT bits."""
from pathlib import Path
import json
import re

ROOT = Path(__file__).resolve().parents[1]


def audit():
    netlist = (ROOT/'impl/gwsynthesis/ov5640.vg').read_text()
    block = re.search(r'module nav_map_rom\s*\(.*?endmodule', netlist, re.S)[0]
    result = {}
    for array, filename in [('raw_map', 'map_raw_1cm.mem'), ('plan_map', 'map_plan_2cm.mem')]:
        expected = [int(v) for v in (ROOT/'src/navigation'/filename).read_text().split()]
        instances = re.findall(r'\bpROM\s+('+array+r'\w+)\s*\((.*?)\);', block, re.S)
        assert instances, f'{array} was optimized out of the synthesized netlist'
        coverage = {}
        for name, ports in instances:
            width = int(re.search(r'defparam '+name+r'\.BIT_WIDTH=(\d+);', block)[1])
            word = re.search(r'(?:display_plan_word|plan_word|raw_word)\[(\d+):(\d+)\]', ports)
            assert word, f'Unknown ROM output packing: {name}'
            high, low = map(int, word.groups())
            assert high-low+1 == width
            chunks = dict((int(i, 16), int(v, 16)) for i, v in re.findall(
                r'defparam '+name+r"\.INIT_RAM_([0-9A-F]+)=256'h([0-9A-F]+);", block))
            for index in range(len(expected)//16):
                for bit in range(width):
                    address = index*16+low+bit
                    physical = index*width+bit
                    actual = (chunks.get(physical//256, 0) >> (physical % 256)) & 1
                    assert actual == expected[address], f'{name}: cell {address} initialization mismatch'
                    coverage[address] = actual
        assert len(coverage) == len(expected), f'{array}: incomplete ROM coverage'
        result[array] = dict(cells=len(expected), free=expected.count(0), rom_instances=len(instances))
    heading = [int(v,16) for v in (ROOT/'src/navigation/map_heading_2cm.mem').read_text().split()]
    central = [int(v) for v in (ROOT/'src/navigation/map_central_2cm.mem').read_text().split()]
    values = [sum(((h>>(2*a))&1)<<a for a in range(4)) | (c<<4) for h,c in zip(heading,central)]
    instances = re.findall(r'\bpROM(?:X9)?\s+(heading_map\w+)\s*\((.*?)\);', block, re.S)
    # Heading/clearance costs are intentionally no longer connected. Only the
    # unused hints may disappear; raw and planning occupancy are required above.
    if not instances:
        planner=(ROOT/'src/navigation/astar_grid.v').read_text(encoding='utf-8')
        assert "wire [12:0] edge_cost=direction<4?13'd5:13'd7;" in planner
        result['heading_clearance']=dict(used=False,reason='body-heading and clearance costs removed')
        print('PASS synthesized map ROM INIT audit: '+json.dumps(result))
        return result
    checked = 0
    for name, ports in instances:
        width = int(re.search(r'defparam '+name+r'\.BIT_WIDTH=(\d+);', block)[1])
        chunks = re.findall(r'defparam '+name+r"\.INIT_RAM_([0-9A-F]+)=(256|288)'h([0-9A-F]+);", block)
        chunk_size = 288 if 'X9' in re.search(r'\bpROM(?:X9)?\s+'+name, block)[0] else 256
        data = {int(i,16):int(v,16) for i,_,v in chunks}
        outputs = []
        for high,low in re.findall(r'heading_word\[(\d+)(?::(\d+))?\]', ports):
            high,low = int(high),int(low or high)
            outputs += list(range(high,low-1,-1))
        assert 0 < len(outputs) <= width
        for index in range(1760):
            for bit, logical in enumerate(reversed(outputs)):
                cell = index*16+logical//5
                expected_bit = (values[cell] >> (logical%5)) & 1
                physical = index*width+bit
                actual = (data.get(physical//chunk_size,0) >> (physical%chunk_size)) & 1
                assert actual == expected_bit, f'{name}: heading/clearance cell {cell} mismatch'
                checked += 1
    result['heading_clearance'] = dict(cells=len(values), rom_instances=len(instances), checked_physical_bits=checked)
    print('PASS synthesized map ROM INIT audit: '+json.dumps(result))
    return result


if __name__ == '__main__':
    audit()
