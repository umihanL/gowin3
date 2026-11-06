"""Compare all font pixels with the actual Gowin row-ROM initialization."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def audit():
    netlist = (ROOT/'impl/gwsynthesis/ov5640.vg').read_text()
    block = re.search(r'module nav_renderer\s*\(.*?endmodule', netlist, re.S)[0]
    rows = [int(v, 16) for v in (ROOT/'src/navigation/nav_font_rows.mem').read_text().split()]
    assert len(rows) == 4096
    instances = re.findall(r'\bpROM\s+(font_rom\w+)\s*\((.*?)\);', block, re.S)
    assert len(instances) == 4, 'Expected four font BSRAM blocks'
    coverage = set()
    for name, ports in instances:
        width = int(re.search(r'defparam '+name+r'\.BIT_WIDTH=(\d+);', block)[1])
        high, low = map(int, re.search(r'font_row\[(\d+):(\d+)\]', ports).groups())
        assert high-low+1 == width
        chunks = {int(i, 16): int(v, 16) for i, v in re.findall(
            r'defparam '+name+r"\.INIT_RAM_([0-9A-F]+)=256'h([0-9A-F]+);", block)}
        for row, expected in enumerate(rows):
            for bit in range(width):
                physical = row*width+bit
                actual = (chunks.get(physical//256, 0) >> (physical % 256)) & 1
                assert actual == ((expected >> (low+bit)) & 1), (name, row, bit)
                coverage.add(row*16+low+bit)
    assert len(coverage) == 65536
    result = dict(rows=4096, pixels=len(coverage), rom_instances=len(instances))
    print('PASS synthesized font ROM INIT audit:', result)
    return result


if __name__ == '__main__':
    audit()
