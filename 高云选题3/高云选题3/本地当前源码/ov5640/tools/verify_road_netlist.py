"""Audit every retained road graph/centreline bit in Gowin ROM INIT."""
from pathlib import Path
import re
ROOT=Path(__file__).resolve().parents[1]

def audit():
 netlist=(ROOT/'impl/gwsynthesis/ov5640.vg').read_text()
 result={}
 for module,array,include,output in [
   ('nav_road_planner','graph','nav_centerline_init.vh','gq'),
   ('nav_road_planner','reverse_graph','nav_centerline_reverse.vh','reverse_q'),
   ('navigation_ui','centre_map','nav_centerline_bitmap.vh','centre_word')]:
  block=re.search(r'module '+module+r'\s*\(.*?endmodule',netlist,re.S)[0]
  values=[int(w,16) for w in re.findall(r"\[\d+\]=\d+'h([0-9a-f]+);",(ROOT/'src/navigation'/include).read_text())]
  assert values,include
  instances=re.findall(r'\b(pROM(?:X9)?)\s+('+array+r'\w+)\s*\((.*?)\);',block,re.S)
  assert instances,(array,'removed ROM')
  coverage=set();checked=0
  for primitive,name,ports in instances:
   width=int(re.search(r'defparam '+name+r'\.BIT_WIDTH=(\d+);',block)[1]);chunk_size=288 if primitive.endswith('X9') else 256
   chunks={int(n,16):int(v,16) for n,v in re.findall(r'defparam '+name+r"\.INIT_RAM_([0-9A-F]+)=\d+'h([0-9A-F]+);",block)}
   outputs=[]
   for hi,lo in re.findall(output+r'\[(\d+)(?::(\d+))?\]',ports):
    hi,lo=int(hi),int(lo or hi);outputs+=list(range(hi,lo-1,-1))
   assert outputs and len(outputs)<=width,(name,ports)
   for i,value in enumerate(values):
    for physical_bit,logical_bit in enumerate(reversed(outputs)):
     physical=i*width+physical_bit;actual=(chunks.get(physical//chunk_size,0)>>(physical%chunk_size))&1
     assert actual==(value>>logical_bit)&1,(name,i,physical_bit,logical_bit)
     coverage.add(logical_bit);checked+=1
  varying=0
  for v in values:varying|=v^values[0]
  assert all(i in coverage for i in range(varying.bit_length()) if varying&(1<<i)),('lost varying ROM bit',array)
  result[array]=dict(words=len(values),rom_instances=len(instances),checked_bits=checked,logical_bits=sorted(coverage))
 print('PASS synthesized road ROM INIT:',result)
 return result

if __name__=='__main__':audit()
