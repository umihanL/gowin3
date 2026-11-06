"""Exercise the real browser quaternion renderer/parser with simulated serial input."""
from pathlib import Path
import hashlib
import importlib.util
import json
import math
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('analyzer', root/'tools/analyze_imu_pose.py')
analyzer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analyzer)


def frame(angle, sign=1):
    a = math.radians(angle)/2
    q = [sign*math.cos(a), 0, sign*math.sin(a), 0]
    return 'samples:0,0,0,0,0,0,'+','.join(f'{x:.8f}' for x in q)


lines = [frame(0)]*20+[frame(90), frame(150), frame(-90), frame(-150)]+[frame(0, -1)]*20
result = analyzer.analyze('\n'.join(lines))
assert result['endpoint_orientation_difference_deg'] < 0.00001
lines[-20:] = [frame(1.8, -1)]*20
result = analyzer.analyze('\n'.join(lines))
assert abs(result['endpoint_orientation_difference_deg']-1.8) < 0.0001
assert result['start_window_max_deviation_deg'] < 0.00001
print('PASS quaternion endpoint analysis: +/-90 and +/-150 round trip, q/-q equivalence, real 1.8-degree offset retained')

mock = '''<script>
Object.defineProperty(navigator,'serial',{value:{requestPort:async()=>({
open:async()=>{},close:async()=>{},readable:{getReader:()=>{let sent=false;return {
read:async()=>{if(!sent){sent=true;return {done:false,value:new TextEncoder().encode(
'samples:12.3,-45.6,78.9,20.0,-30.0,60.0,0.8013,0.2727,-0.1370,0.5145\\r\\n')}};return new Promise(()=>{});},
cancel:async()=>{},releaseLock:()=>{}}}}})}});
</script>'''
tests = '''<script>
(async()=>{try{
let cases=0;
function assert(ok,message){if(!ok)throw new Error(message);}
for(let angle=-540;angle<=540;angle+=3){
 for(const axis of [[1,0,0],[0,1,0],[0,0,1],[.2672612419,.5345224838,.8017837257]]){
  const rv=axis.map(v=>v*angle),q=PoseMath.fromVector(rv);
  const units=[[1,0,0],[0,1,0],[0,0,1]];
  const rotated=units.map(v=>PoseMath.rotate(v,q));
  for(let i=0;i<3;i++)for(let j=0;j<3;j++){
   const dot=rotated[i].reduce((sum,x,k)=>sum+x*rotated[j][k],0);
   assert(Math.abs(dot-(i===j?1:0))<1e-8,'Matrix orthogonality');
  }
  // Independent Rodrigues oracle; no Euler decomposition in renderer.
  const t=angle*Math.PI/180,c=Math.cos(t),s=Math.sin(t),[x,y,z]=axis;
  const oracle=[[c+x*x*(1-c),x*y*(1-c)-z*s,x*z*(1-c)+y*s],
   [y*x*(1-c)+z*s,c+y*y*(1-c),y*z*(1-c)-x*s],
   [z*x*(1-c)-y*s,z*y*(1-c)+x*s,c+z*z*(1-c)]];
  for(let col=0;col<3;col++)for(let row=0;row<3;row++)
   assert(Math.abs(rotated[col][row]-oracle[row][col])<1e-8,'Rodrigues disagreement');
  const negative=PoseMath.rotate([.2,.5,-.7],q.map(v=>-v));
  const positive=PoseMath.rotate([.2,.5,-.7],q);
  assert(negative.every((v,i)=>Math.abs(v-positive[i])<1e-9),'Quaternion sign seam');
  cases++;
 }
}
assert(PoseMath.parse('samples:0,90,0').q.every((v,i)=>Math.abs(v-[Math.SQRT1_2,0,Math.SQRT1_2,0][i])<1e-8),'Legacy vector stream');
for(const line of ['samples:NaN,0,0','samples:,0,0','samples:1,2,3,4','samples:0,0,0,0,0,0,0,0,0,0'])
 assert(PoseMath.parse(line)===null,'Malformed input accepted');
document.getElementById('connect').click();
await new Promise(resolve=>setTimeout(resolve,200));
assert(document.getElementById('rx').textContent.startsWith('12.3'),'Serial receive vector');
assert(document.getElementById('roll').textContent.startsWith('20.0'),'Serial receive Euler');
assert(!document.getElementById('save').disabled,'Recording unavailable');
assert(document.getElementById('scene').width>0,'Canvas did not render');
document.body.setAttribute('data-host-test','passed');
const pre=document.createElement('pre');pre.id='host-test-result';pre.textContent=
'PASS browser pose: '+cases+' arbitrary-axis rotations from -540 to +540, +/-90 and 180 crossings, independent Rodrigues oracle, q/-q, legacy vector/extended quaternion streams, malformed frames, simulated Web Serial/UI/recording';document.body.appendChild(pre);
}catch(e){document.body.setAttribute('data-host-test','failed');document.body.append('FAIL '+e.stack);}})();
</script>'''
original = (root/'docs/imu_pose_viewer.html').read_text(encoding='utf-8')
page = original.replace('<script>\n(() => {',mock+'\n<script>\n(() => {',1)
assert page != original
page = page.replace('</html>',tests+'\n</html>')
folder = Path(tempfile.mkdtemp(prefix='imu_pose_browser_'))
test_page = folder/'viewer_test.html'
test_page.write_text(page,encoding='utf-8')
browser = Path(r'C:\Program Files\Google\Chrome\Application\chrome.exe')
if not browser.exists():
    browser=Path(r'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe')
command = [str(browser),'--headless','--disable-gpu','--no-first-run','--no-default-browser-check',
           '--allow-file-access-from-files',f'--user-data-dir={folder / "profile"}',
           '--virtual-time-budget=1000','--window-size=1280,900','--hide-scrollbars',
           f'--screenshot={root / "docs/imu_pose_viewer_simulation.png"}',
           '--dump-dom',test_page.as_uri()]
run = subprocess.run(command,capture_output=True,timeout=40,creationflags=subprocess.CREATE_NO_WINDOW)
dom = run.stdout.decode('utf-8',errors='replace')
(folder/'dom.html').write_text(dom,encoding='utf-8')
assert run.returncode==0 and 'data-host-test="passed"' in dom, str(folder)+'\n'+run.stderr.decode('utf-8',errors='replace')[-3000:]+'\n'+dom[-1800:]
output=re.search(r'<pre id="host-test-result">(.*?)</pre>',dom,re.S)[1]
print(output)
log = ('PASS quaternion endpoint analysis: +/-90 and +/-150 round trip, q/-q equivalence, real 1.8-degree offset retained\n'+output+'\n'+
       json.dumps({'viewer_sha256':hashlib.sha256(original.encode('utf-8')).hexdigest(),
                   'source':'simulated browser/serial input; not hardware measurements'})+'\n')
(root/'sim/imu_host_test_results.txt').write_text(log,encoding='utf-8')
