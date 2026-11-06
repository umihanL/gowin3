"""Protocol vectors, stream recovery and asynchronous events during ACK wait."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from host_serial_client import encode,decode,Parser,Client

vectors=[(1,2,bytes([100,0,0,0]),'a55a0101026400000066'),
         (2,3,bytes([0,1,0,0]),'a55a0102030001000001'),
         (3,5,bytes(4),'a55a0103050000000007')]
for seq,cmd,payload,expected in vectors:
 wire=encode(seq,cmd,payload)
 assert wire.hex()==expected
 assert decode(wire)==dict(sequence=seq,command=cmd,data=payload)
bad=bytearray(encode(9,2));bad[-1]^=128
stream=b'noise\xa5'+bytes(bad)+b'junk'+b''.join(encode(s,c,d) for s,c,d,_ in vectors)
for chunk in range(1,len(stream)+1):
 parser=Parser();frames=[]
 for i in range(0,len(stream),chunk):frames+=parser.feed(stream[i:i+chunk])
 assert frames==[dict(sequence=s,command=c,data=d) for s,c,d,_ in vectors]

class FakePort:
 def __init__(self,reject=False):self.incoming=bytearray();self.reject=reject
 @property
 def in_waiting(self):return len(self.incoming)
 def write(self,data):
  request=decode(data)
  self.incoming+=encode(0,0xf0,bytes([7,3,0,0]))
  self.incoming+=encode(request['sequence'],request['command']|0x80,bytes([2 if self.reject else 0,0,0,0]))
 def flush(self):pass
 def read(self,count):
  result=bytes(self.incoming[:count]);del self.incoming[:count];return result

client=Client(FakePort());ack=client.request(5)
assert ack['command']==0x85 and len(client.events)==1 and client.events[0]['command']==0xf0
try:Client(FakePort(True)).request(2)
except RuntimeError:pass
else:raise AssertionError('rejected command accepted')
print('PASS host Python client: fixed vectors, arbitrary chunks, corruption/resync, ACK and queued completion event')
