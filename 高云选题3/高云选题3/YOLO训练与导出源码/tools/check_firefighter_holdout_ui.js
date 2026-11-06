// Offline review QA. Edge is explicitly denied GPU use while training runs.
const fs = require('fs');
const path = require('path');
const { pathToFileURL } = require('url');
const { chromium } = require('playwright');

(async () => {
  const report = path.resolve(process.argv[2]);
  const output = path.dirname(report);
  const browser = await chromium.launch({ channel: 'msedge', headless: true, args: ['--disable-gpu'] });
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 }, deviceScaleFactor: 1 });
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  await page.goto(pathToFileURL(report).href);
  await page.waitForFunction(() => [...document.querySelectorAll('#gallery img')].every(im => im.complete && im.naturalWidth === 640));
  const pairCount = await page.locator('#counter').textContent();
  const initialTitle = await page.locator('#pairTitle').textContent();
  await page.screenshot({ path: path.join(output, 'review_desktop.png'), fullPage: true });
  await page.locator('#next').click();
  if (await page.locator('#pairTitle').textContent() === initialTitle) throw new Error('Pair pagination did not advance');
  await page.locator('#model').selectOption('old_v7_best');
  await page.waitForFunction(() => [...document.querySelectorAll('#gallery img')].every(im => im.complete && im.naturalWidth === 640));
  const oldPaths = await page.locator('#gallery img').evaluateAll(ims => ims.map(im => im.getAttribute('src')));
  if (!oldPaths.every(p => p.startsWith('old_v7_best/'))) throw new Error('Model switch did not change overlays');
  await page.locator('#display').selectOption('original');
  const originalPaths = await page.locator('#gallery img').evaluateAll(ims => ims.map(im => im.getAttribute('src')));
  if (!originalPaths.every(p => p.startsWith('originals/'))) throw new Error('Original view did not change images');
  await page.locator('#light').selectOption('dim');
  if (!await page.locator('#gallery').textContent().then(t => t.includes('暗光'))) throw new Error('Dim filter failed');
  await page.locator('#light').selectOption('');
  await page.locator('#result').selectOption('negative');
  if (!await page.locator('#gallery').textContent().then(t => t.includes('无消防员负例'))) throw new Error('Negative filter failed');
  await page.locator('#result').selectOption('failed');
  const failedPairs = await page.locator('#counter').textContent();
  await page.locator('#result').selectOption('');
  await page.locator('#display').selectOption('overlay');
  await page.locator('#model').selectOption('current_v8_best');
  // Verify every original and both-model overlay, independent of currently visible pair.
  const samples = JSON.parse(fs.readFileSync(path.join(output, 'samples.json'), 'utf8'));
  const paths = samples.flatMap(p => p.eyes.flatMap(e => [e.original, ...Object.values(e.models).map(m => m.overlay)]));
  const missing = paths.filter(p => !fs.existsSync(path.join(output, p)));
  if (missing.length) throw new Error('Missing preview files: ' + missing.join(', '));
  const loaded = await page.evaluate(async paths => {
    const out=[];
    for (const src of paths) {
      out.push(await new Promise(resolve => {const im=new Image();im.onload=()=>resolve(im.naturalWidth===640&&im.naturalHeight===480);im.onerror=()=>resolve(false);im.src=src;}));
    }
    return out.filter(Boolean).length;
  }, paths);
  if (loaded !== paths.length) throw new Error('Some images failed browser decoding');
  const desktopOverflow = await page.evaluate(() => document.documentElement.scrollWidth > innerWidth);
  await page.locator('.gallery').screenshot({ path: path.join(output, 'review_pair.png') });
  await page.setViewportSize({ width: 390, height: 844 });
  const mobileOverflow = await page.evaluate(() => document.documentElement.scrollWidth > innerWidth);
  await page.screenshot({ path: path.join(output, 'review_mobile.png'), fullPage: true });
  const result = { passed: errors.length === 0 && !desktopOverflow && !mobileOverflow,
    browser: 'Edge headless --disable-gpu', initial_pair_counter: pairCount, old_model_failed_pair_counter: failedPairs,
    all_preview_images_loaded: loaded, expected_preview_images: paths.length,
    model_switch: true, original_switch: true, dim_filter: true, negative_filter: true, pagination: true,
    desktop_overflow: desktopOverflow, mobile_overflow: mobileOverflow, page_errors: errors };
  fs.writeFileSync(path.join(output, 'review_ui_validation.json'), JSON.stringify(result, null, 2));
  await browser.close();
  console.log(JSON.stringify(result));
  if (!result.passed) process.exitCode = 1;
})().catch(error => { console.error(error); process.exitCode = 1; });
