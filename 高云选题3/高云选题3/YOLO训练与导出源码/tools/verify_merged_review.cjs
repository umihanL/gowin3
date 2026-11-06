/* Optional offline UI QA with Playwright + locally installed Edge. No network. */
const { chromium } = require('playwright');
const path = require('path');
const fs = require('fs');
const { pathToFileURL } = require('url');

(async () => {
  const arg = process.argv.indexOf('--dataset');
  const dataset = path.resolve(arg >= 0 ? process.argv[arg + 1] : path.join(__dirname, '..', 'dataset'));
  const reportDir = path.join(dataset, 'review_qa');
  fs.mkdirSync(reportDir, { recursive: true });
  const browser = await chromium.launch({ channel: 'msedge', headless: true, args: ['--disable-gpu'] });
  const page = await browser.newPage({ viewport: { width: 1440, height: 1080 }, deviceScaleFactor: 1 });
  const errors = [];
  page.on('pageerror', error => errors.push(String(error)));
  page.on('console', message => { if (message.type() === 'error') errors.push(message.text()); });
  await page.route(/^https?:/, route => { errors.push('Unexpected network request: ' + route.request().url()); return route.abort(); });
  const assert = (condition, message) => { if (!condition) throw new Error(message); };
  const results = [];
  try {
    await page.goto(pathToFileURL(path.join(dataset, 'review.html')).href);
    await page.waitForFunction(() => document.getElementById('status').textContent.startsWith('读取成功'));
    const summary = await page.locator('#review-data').textContent().then(JSON.parse);
    assert(summary.classes.join(',') === 'fire,firefighter,person', 'Wrong class schema');
    assert(summary.sources.includes('v5') && summary.sources.includes('v6'), 'Missing merged source');
    assert((await page.locator('.notice').textContent()).includes('不是模型预测结果'), 'Missing automatic-label disclosure');
    for (const source of summary.sources) {
      for (const split of ['train', 'val', 'test']) {
        await page.selectOption('#source', source);
        await page.selectOption('#split', split);
        await page.selectOption('#variant', 'original');
        if (!summary.samples.some(s => s.source === source && s.split === split)) {
          assert((await page.locator('#counter').textContent()).includes('没有符合'), 'Empty source/split should show empty state');
          assert(await page.locator('#next').isDisabled(), 'Empty source/split next button should be disabled');
          results.push({ source, split, empty: true });
          continue;
        }
        await page.waitForFunction(() => document.getElementById('status').textContent.startsWith('读取成功'));
        const title = await page.locator('#sample-title').textContent();
        assert(title.startsWith(source + '__'), 'Source filter failed');
        assert((await page.locator('#version-tag').textContent()).includes(split), 'Split filter failed');
        const before = title;
        await page.click('#other-eye');
        await page.waitForFunction(previous => document.getElementById('sample-title').textContent !== previous && document.getElementById('status').textContent.startsWith('读取成功'), before);
        results.push({ source, split, counterpart_switched: true, counter: await page.locator('#counter').textContent() });
      }
    }
    await page.click('#reset');
    await page.selectOption('#source', 'v6');
    await page.selectOption('#split', 'train');
    await page.selectOption('#variant', 'augmented');
    await page.selectOption('#eye', 'R');
    await page.waitForFunction(() => document.getElementById('status').textContent.startsWith('读取成功'));
    assert((await page.locator('#sample-title').textContent()).includes('_R_a01'), 'Augmentation/eye filter failed');
    for (const cls of ['0', '1', '2']) {
      await page.selectOption('#class', cls);
      await page.waitForFunction(() => document.getElementById('status').textContent.startsWith('读取成功'));
      assert((await page.locator('#objects').textContent()).includes(summary.classes[Number(cls)]), 'Class filter failed');
    }
    await page.selectOption('#split', 'val');
    assert((await page.locator('#counter').textContent()).includes('没有符合'), 'Held-out augmentation should be empty');
    assert(await page.locator('#next').isDisabled(), 'Empty selection next button is not disabled');
    await page.click('#reset');
    await page.selectOption('#class', 'negative');
    await page.waitForFunction(() => document.getElementById('status').textContent.startsWith('读取成功'));
    assert((await page.locator('#objects').textContent()).includes('空标签'), 'Negative filter failed');
    await page.click('#reset');
    await page.selectOption('#source', summary.sources.includes('v8') ? 'v8' : summary.sources.includes('v7') ? 'v7' : 'v6');
    await page.selectOption('#class', summary.sources.includes('v7') ? '1' : '0');
    await page.waitForFunction(() => document.getElementById('status').textContent.startsWith('读取成功'));
    await page.uncheck('#tiny');
    await page.uncheck('#boxes');
    await page.check('#boxes');
    await page.waitForFunction(() => document.getElementById('status').textContent.startsWith('读取成功'));
    await page.screenshot({ path: path.join(reportDir, 'desktop.png'), fullPage: true });
    assert(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth), 'Desktop horizontal overflow');
    await page.setViewportSize({ width: 390, height: 844 });
    await page.screenshot({ path: path.join(reportDir, 'mobile.png'), fullPage: true });
    assert(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth), 'Mobile horizontal overflow');
    assert(errors.length === 0, 'Browser errors: ' + errors.join(' | '));
    const report = { passed: true, offline_file_url: true, browser_gpu_disabled: true, images: summary.samples.length, sources: summary.sources,
      classes: summary.classes, tested: ['source/split', 'left/right counterpart', 'augmentation/eye', 'class', 'negative',
        'held-out augmentation empty state', 'box/tiny display', 'desktop/mobile overflow'], selections: results, browser_errors: errors,
      screenshots: ['review_qa/desktop.png', 'review_qa/mobile.png'], note: 'UI displays automatic annotation truth; no trained predictions.' };
    fs.writeFileSync(path.join(dataset, 'review_ui_validation.json'), JSON.stringify(report, null, 2));
    process.stdout.write(JSON.stringify({ passed: true, images: summary.samples.length, cases: results.length, report: path.join(dataset, 'review_ui_validation.json') }) + '\n');
  } finally {
    await browser.close();
  }
})().catch(error => { process.stderr.write(String(error.stack || error) + '\n'); process.exitCode = 1; });
