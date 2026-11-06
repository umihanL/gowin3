"""Small host protocol reference, no serial dependency for encoding/decoding."""
import argparse
import time

def encode(seq,command,data=b'\0\0\0\0'):
 if len(data)!=4:raise ValueError('exactly four payload bytes required')
 body=bytes([1,seq,command])+bytes(data)
 checksum=0
 for byte in body:checksum^=byte
 return b'\xa5\x5a'+body+bytes([checksum])

def decode(frame):
 if len(frame)!=10 or frame[:3]!=b'\xa5\x5a\x01':raise ValueError('invalid frame header or version')
 checksum=0
 for byte in frame[2:9]:checksum^=byte
 if checksum!=frame[9]:raise ValueError('checksum mismatch')
 return dict(sequence=frame[3],command=frame[4],data=bytes(frame[5:9]))

class Parser:
 def __init__(self):self.pending=bytearray()
 def feed(self,data):
  self.pending.extend(data);frames=[]
  while len(self.pending)>=2:
   at=self.pending.find(b'\xa5\x5a')
   if at<0:
    self.pending[:]=self.pending[-1:] if self.pending[-1:]==b'\xa5' else b'';break
   del self.pending[:at]
   if len(self.pending)<10:break
   try:frame=decode(self.pending[:10])
   except ValueError:del self.pending[0];continue
   frames.append(frame);del self.pending[:10]
  return frames

class Client:
 # CMD 08 selects a numbered goal and starts FPGA planning; CMD 03 starts motion.
 def select_point(self,point,task=1):
  if not 1<=point<=9:raise ValueError('point must be 1..9')
  if not 0<=task<=255:raise ValueError('task must be 0..255')
  return self.request(8,bytes([point,task,0,0]))
 def point_status(self):
  data=self.request(9)['data']
  return dict(point=data[1],state=data[2],error=data[3])
 def wait_point_ready(self,point,timeout=60.0):
  deadline=time.monotonic()+timeout
  while time.monotonic()<deadline:
   status=self.point_status()
   if status['point']!=point:raise RuntimeError('selected destination changed')
   if status['state']==2:return status
   if status['state']==3:raise RuntimeError(f"FPGA route planning failed: error={status['error']}")
   if status['state']!=1:raise RuntimeError('route invalidated or navigation already active')
   self.request(1)
   time.sleep(0.1)
  raise TimeoutError('route planning timed out')
 def start_navigation(self,task=1):
  if not 0<=task<=255:raise ValueError('task must be 0..255')
  return self.request(3,bytes([1,task,0,0]))
 def navigate_to(self,point,task=1,timeout=60.0):
  self.select_point(point,task)
  self.wait_point_ready(point,timeout)
  return self.start_navigation(task)
 def __init__(self,port):self.port=port;self.sequence=0;self.parser=Parser();self.events=[]
 def poll(self):
  return self.parser.feed(self.port.read(max(1,self.port.in_waiting)))
 def request(self,command,data=b'\0\0\0\0',timeout=1.0):
  self.sequence=(self.sequence+1)&255
  self.port.write(encode(self.sequence,command,data));self.port.flush()
  deadline=time.monotonic()+timeout
  while time.monotonic()<deadline:
   matched=None
   for frame in self.poll():
    print(format_frame(frame),flush=True)
    if frame['sequence']==self.sequence and frame['command']==command|0x80:
     if frame['data'][0]!=0:raise RuntimeError(f"command rejected: result={frame['data'][0]}")
     matched=frame
    elif frame['command']>=0xe0:self.events.append(frame)
   if matched:return matched
  raise TimeoutError('no matching reply; verify mode, page, wiring and serial settings')

def format_frame(frame):
 return f"seq={frame['sequence']:02X} cmd={frame['command']:02X} data={frame['data'].hex(' ').upper()}"

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--port',required=True);p.add_argument('--baud',type=int,default=115200)
 s=p.add_subparsers(dest='action',required=True)
 for name in ('heartbeat','status','point-status','stop'):s.add_parser(name)
 resume=s.add_parser('resume');resume.add_argument('--watch',action='store_true')
 speed=s.add_parser('speed');speed.add_argument('mm_s',type=int)
 for name in ('start-drive','start-nav'):
  sub=s.add_parser(name);sub.add_argument('--task',type=int,default=1);sub.add_argument('--watch',action='store_true')
 for name in ('select-point','navigate-to'):
  sub=s.add_parser(name);sub.add_argument('point',type=int);sub.add_argument('--task',type=int,default=1)
  if name=='navigate-to':sub.add_argument('--watch',action='store_true')
 args=p.parse_args()
 if hasattr(args,'point') and not 1<=args.point<=9:p.error('point must be 1..9')
 if args.action=='speed' and not 0<=args.mm_s<=1000:p.error('speed must be 0..1000 mm/s')
 if hasattr(args,'task') and not 0<=args.task<=255:p.error('task must be 0..255')
 try:import serial
 except ImportError:raise SystemExit('Install pyserial: python -m pip install pyserial')
 commands={'heartbeat':1,'status':6,'point-status':9,'stop':5,'resume':7,'speed':2,'start-drive':3,'start-nav':3}
 payload=b'\0\0\0\0'
 if args.action=='speed':payload=args.mm_s.to_bytes(2,'little')+b'\0\0'
 elif args.action.startswith('start-'):payload=bytes([args.action=='start-nav',args.task,0,0])
 with serial.Serial(args.port,args.baud,timeout=0.05,write_timeout=1) as port:
  client=Client(port)
  if args.action=='select-point':client.select_point(args.point,args.task)
  elif args.action=='navigate-to':client.navigate_to(args.point,args.task)
  else:client.request(commands[args.action],payload)
  if getattr(args,'watch',False):
   next_heartbeat=time.monotonic()+0.5
   try:
    while True:
     if any(frame['command'] in (0xf0,0xf1,0xf2) for frame in client.events):return
     client.events.clear()
     for frame in client.poll():
      print(format_frame(frame),flush=True)
      if frame['command'] in (0xf0,0xf1,0xf2):return
     if time.monotonic()>=next_heartbeat:
      client.request(1);next_heartbeat=time.monotonic()+0.5
   except KeyboardInterrupt:
    client.request(5)
if __name__=='__main__':main()
