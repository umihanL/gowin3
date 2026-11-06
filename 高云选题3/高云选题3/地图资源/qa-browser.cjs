const { chromium } = require('C:\\Users\\18522\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\node\\node_modules\\playwright');
const fs = require('fs');
(async()=>{
  const browser=await chromium.launch({headless:true,executablePath:'C:/Program Files/Google/Chrome/Application/chrome.exe',args:['--no-sandbox','--disable-gpu','--disable-dev-shm-usage']});
  const results=[];
  for(const spec of [{name:'desktop',w:1440,h:900},{name:'mobile',w:390,h:844}]){
    const page=await browser.newPage({viewport:{width:spec.w,height:spec.h},deviceScaleFactor:1});
    const errors=[];page.on('pageerror',e=>errors.push(String(e)));page.on('console',m=>{if(m.type()==='error')errors.push(m.text())});
    await page.goto('http://127.0.0.1:8765/scene-preview.html',{waitUntil:'domcontentloaded',timeout:10000});await page.waitForTimeout(900);
    const canvas=page.locator('canvas');
    const pixel=await page.evaluate(()=>{const c=document.querySelector('canvas');const x=c.getContext('webgl2')||c.getContext('webgl');const p=new Uint8Array(4);x.readPixels(Math.floor(c.width/2),Math.floor(c.height/2),1,1,x.RGBA,x.UNSIGNED_BYTE,p);return {w:c.width,h:c.height,p:[...p]}});
    const initial=await page.locator('#error').textContent();
    await page.locator('#topBtn').click({force:true});await page.waitForTimeout(150);const topLabel=await page.locator('#modeLabel').textContent();
    await page.evaluate(()=>document.querySelector('#robotBtn').click());await page.waitForTimeout(150);const robotLabel=await page.locator('#modeLabel').textContent();
    if(await page.locator('#routeSelect').isVisible()){await page.locator('#routeSelect').selectOption({index:2});}else{await page.evaluate(()=>{const s=document.querySelector('#routeSelect');s.selectedIndex=2;s.dispatchEvent(new Event('change',{bubbles:true}))});}if(await page.locator('#playBtn').isVisible()){await page.locator('#playBtn').click({force:true});}else{await page.evaluate(()=>document.querySelector('#playBtn').click())}await page.waitForTimeout(350);const pos=await page.locator('#posReadout').textContent();
    await page.screenshot({path:`D:/工程仿真/FPGA赛道_小车仿真/map_assets/preview-${spec.name}.png`,fullPage:false});
    results.push({name:spec.name,canvas:await canvas.count(),pixel,initial,topLabel,robotLabel,pos,errors});await page.close();
  }
  fs.writeFileSync('D:/工程仿真/FPGA赛道_小车仿真/map_assets/qa-browser.json',JSON.stringify(results,null,2));console.log(JSON.stringify(results,null,2));await browser.close();
})().catch(e=>{console.error(e);process.exit(1)});
