"""AI adapter integration using a deterministic serial FPGA protocol model."""
from pathlib import Path
import sys,threading,time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from host_serial_client import encode,decode,Client
from diguapi_navigation import VoiceNavigator

class Port:
 def __init__(self,fail=False,delay=2):
  self.incoming=bytearray();self.commands=[];self.point=0;self.state=0;self.error=0
  self.query_count=0;self.fail=fail;self.delay=delay;self.run=False;self.task=0
 @property
 def in_waiting(self):return len(self.incoming)
 def write(self,raw):
  frame=decode(raw);op=frame['command'];a,b,c,d=frame['data'];self.commands.append((op,bytes([a,b,c,d])))
  result=0;data=bytes(4)
  if op==8:
   self.point=a;self.task=b;self.state=1;self.query_count=0
   data=bytes([0,a,1,b])
  elif op==9:
   self.query_count+=1
   if self.query_count>=self.delay:self.state=3 if self.fail else 2;self.error=1 if self.fail else 0
   data=bytes([0,self.point,self.state,self.error])
  elif op==3:
   if self.state!=2:result=3
   else:self.run=True;self.state=4;self.task=b
  elif op==5:self.run=False;self.state=3
  elif op==7:self.run=True
  self.incoming+=encode(frame['sequence'],op|128,bytes([result,0,0,0]) if op not in (8,9) else data)
 def flush(self):pass
 def read(self,n):
  result=bytes(self.incoming[:n]);del self.incoming[:n];return result
 def complete(self):self.incoming+=encode(0,0xf0,bytes([self.task,3,0,0]));self.run=False

p=Port();c=Client(p);c.navigate_to(9,42)
assert [op for op,_ in p.commands if op!=1]==[8,9,9,3]
assert p.run and p.task==42 and p.point==9
for point in (0,10,-1):
 try:c.select_point(point)
 except ValueError:pass
 else:raise AssertionError('bad point accepted')
try:Client(Port(fail=True)).navigate_to(2,4)
except RuntimeError:pass
else:raise AssertionError('failed planning started')

p=Port()
with VoiceNavigator(p) as n:
 n.events.put(dict(command=0xf0,data=bytes([7,3,0,0])))
 n.navigate_to(3,7);assert p.run and n.events.empty()
 time.sleep(0.6)
 assert len([c for c in p.commands if c[0]==1])>=2
 p.incoming+=encode(0,0xf0,bytes([99,3,0,0]));time.sleep(0.1)
 assert n._active,'old task completion cleared active navigation'
 p.complete();event=n.wait_until_done(2);assert event['command']==0xf0 and event['data'][0]==7
assert p.commands[-1][0]!=5,'completed task sent unnecessary stop'
p=Port()
with VoiceNavigator(p) as n:n.navigate_to(4,8)
assert not p.run and p.commands[-1][0]==5,'active context exit failed to stop'
p=Port(delay=100)
with VoiceNavigator(p) as n:
 n.select_point(2,3)
 outcome=[]
 def waiting():
  try:n.wait_ready(3)
  except RuntimeError as e:outcome.append(str(e))
 t=threading.Thread(target=waiting);t.start();time.sleep(0.15);n.stop();t.join(2)
 assert outcome and not t.is_alive()
 try:n.start(3)
 except RuntimeError:pass
 else:raise AssertionError('cancelled plan started')
 assert not any(op==3 for op,_ in p.commands)
print('PASS DiGuaPi adapter: plan/status/start order, invalid point, failed planning, persistent heartbeat, completion queue, stop on exit and concurrent cancellation')
