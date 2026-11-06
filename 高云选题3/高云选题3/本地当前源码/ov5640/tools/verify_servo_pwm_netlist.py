"""Independently audit the synthesized 0.1-degree PWM ROM contents."""
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parents[1]

def audit():
    source=(ROOT/'impl/gwsynthesis/ov5640.vg').read_text()
    block=re.search(r'module servo_pwm\s*\(.*?endmodule',source,re.S)[0]
    instances=re.findall(r'\bpROM\s+(pulse_rom\w+)\s*\((.*?)\);',block,re.S)
    assert instances,'No synthesized pulse ROM'
    coverage=set()
    for name,ports in instances:
        width=int(re.search(r'defparam '+name+r'\.BIT_WIDTH=(\d+);',block)[1])
        do=re.search(r'\.DO\((.*?)\)',ports,re.S)[1]
        high,low=map(int,re.search(r'pulse_us\[(\d+):(\d+)\]',do).groups())
        assert high>=low and high-low+1<=width,(name,width,do)
        chunks={int(i,16):int(v,16) for i,v in re.findall(
            r'defparam '+name+r"\.INIT_RAM_([0-9A-F]+)=256'h([0-9A-F]+);",block)}
        assert chunks,name
        for row in range(2048):
            expected=500+(2000*min(row,1800)+900)//1800
            for bit in range(high-low+1):
                physical=row*width+bit
                actual=(chunks.get(physical//256,0)>>(physical%256))&1
                assert actual==((expected>>(low+bit))&1),(name,row,bit)
                coverage.add((row,low+bit))
    assert len(coverage)==2048*12,len(coverage)
    result=dict(entries=2048,verified_bits=len(coverage),rom_instances=len(instances),
                min_pulse_us=500,max_pulse_us=2500,step_tenths=1)
    print('PASS synthesized servo PWM ROM INIT audit:',result)
    return result

if __name__=='__main__':
    audit()
