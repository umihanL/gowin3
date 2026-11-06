"""Draw measured synthetic nose traces over the unmodified occupancy map."""
from pathlib import Path
import csv,hashlib,json
from PIL import Image,ImageDraw,ImageFont,PngImagePlugin
ROOT=Path(__file__).resolve().parents[1]
files=['src/navigation/map_raw_1cm.mem','sim/navigation_reported_route.csv']
raw=[int(x) for x in (ROOT/files[0]).read_text().split()]
road=Image.new('RGB',(352,320))
road.putdata([(238,238,238) if x else (42,45,48) for x in raw])
im=Image.new('RGB',(1056,1040),'white');im.paste(road.resize((1056,960)),(0,80));d=ImageDraw.Draw(im)
route=[(int(r[0])*3/256,int(r[1])*3/256+80) for r in csv.reader((ROOT/files[1]).open())]
d.line(route,fill=(20,190,240),width=5)
for offset,color in [(-60,(250,180,60)),(0,(255,80,80)),(60,(90,240,110))]:
 name=f'sim/navigation_visual_reported_long{offset}_trace.csv';files.append(name)
 rows=list(csv.reader((ROOT/name).open()))
 d.line([(float(r[1])*.3,(320-float(r[2])/10)*3+80) for r in rows],fill=color,width=2)
font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',17)
d.text((18,12),'Plan: cyan. Synthetic physical NOSE: start -60mm orange / 0 red / +60mm green.',fill='black',font=font)
d.text((18,42),'Limited-FOV camera + independent bicycle. This does not certify full-body clearance.',fill='black',font=font)
meta=PngImagePlugin.PngInfo()
meta.add_text('inputs_sha256',json.dumps({n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in files}))
im.save(ROOT/'docs/navigation_visual_reported_trace.png',pnginfo=meta)
print('PASS trace figure with embedded input hashes')
