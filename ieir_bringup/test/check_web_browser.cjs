// Run only against --preview. Requires Playwright and Chromium for developer QA.
const { chromium } = require('playwright');
const assert = require('node:assert/strict');

(async () => {
  const browser = await chromium.launch({ headless: true, args: ['--no-sandbox', '--enable-unsafe-swiftshader'] });
  const page = await browser.newPage({ viewport: { width: 1440, height: 1050 } });
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('response', response => { if (response.status() >= 400) errors.push(`${response.status()} ${response.url()}`); });
  await page.goto(process.env.CONSOLE_URL || 'http://127.0.0.1:8765');
  assert.match(await page.title(), /^IEIR/);
  assert.match(await page.locator('.brand').innerText(), /^IEIR/);
  await page.waitForFunction(() => window.consoleMeshCount > 10, { timeout: 60000 });
  await page.locator('#environment').waitFor({ state: 'visible' });
  assert.match(await page.locator('#environment').innerText(), /预览/);
  await page.waitForFunction(() => document.querySelector('#joint-count').textContent.includes('14'));
  assert.equal(await page.locator('#send-joints').isDisabled(), true);
  const pixels = await page.evaluate(() => {
    window.consoleScene.renderer.render(window.consoleScene.scene, window.consoleScene.camera);
    const canvas = document.querySelector('#viewport canvas');
    const sample = document.createElement('canvas'); sample.width = 100; sample.height = 100;
    const ctx = sample.getContext('2d'); ctx.drawImage(canvas, 0, 0, 100, 100);
    const data = ctx.getImageData(0, 0, 100, 100).data;
    const colors = new Set(); let dark = 0;
    for (let i = 0; i < data.length; i += 4) { colors.add(`${data[i]},${data[i+1]},${data[i+2]}`); if (data[i] < 180) dark++; }
    return { colors: colors.size, dark, meshes: window.consoleMeshCount, frames: window.consoleScene.frames,
      triangles: window.consoleScene.renderer.info.render.triangles };
  });
  assert(pixels.colors > 150 && pixels.dark > 50 && pixels.frames > 1, JSON.stringify(pixels));
  assert(pixels.triangles > 500000, 'Default display must retain the complete original STL surfaces');
  const before = await page.locator('#viewport canvas').screenshot();
  const box = await page.locator('#viewport canvas').boundingBox();
  await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
  await page.mouse.down(); await page.mouse.move(box.x + box.width / 2 + 110, box.y + box.height / 2, { steps: 8 }); await page.mouse.up();
  await page.waitForTimeout(400);
  assert.notDeepEqual(before, await page.locator('#viewport canvas').screenshot(), 'Orbit control must change image');
  await page.locator('#fit-view').click();
  await page.screenshot({ path: '/tmp/ieir-web-desktop.png', fullPage: true });

  await page.locator('#unlock').click();
  await page.locator('#confirm-dialog button[value=confirm]').click();
  await page.locator('[data-mode=joint]').click();
  await page.locator('#confirm-dialog button[value=confirm]').click();
  await page.waitForFunction(() => document.querySelector('#mode-status').textContent === '关节位置');
  await page.locator('[data-view=joints]').click();
  await page.locator('#capture-joints').click();
  const initial = await page.evaluate(async () => (await (await fetch('/api/state')).json()).joints.left_joint_0.position);
  await page.locator('#joint-number-0').fill('8');
  await page.locator('[data-scene=target]').click();
  await page.locator('#send-joints').click();
  await page.locator('#confirm-dialog button[value=cancel]').click();
  const cancelled = await page.evaluate(async () => (await (await fetch('/api/state')).json()).joints.left_joint_0.position);
  assert(Math.abs(cancelled - initial) < 0.001);
  await page.locator('#send-joints').click();
  await page.locator('#confirm-dialog button[value=confirm]').click();
  await page.waitForFunction(async () => (await (await fetch('/api/state')).json()).joints.left_joint_0.position > .1);
  await page.locator('[data-scene=live]').click();
  const streaming = await page.evaluate(async () => {
    let packets = 0;
    const stream = new EventSource('/api/pose-stream');
    stream.onmessage = () => { packets++; };
    const before = window.consoleScene.frames;
    await new Promise(resolve => setTimeout(resolve, 2000));
    stream.close();
    return { packets, frames: window.consoleScene.frames - before };
  });
  assert(streaming.packets >= 25, `Pose stream is too slow: ${JSON.stringify(streaming)}`);
  await page.locator('[data-arm=right]').first().click();
  assert.equal(await page.locator('#joint-number-0').getAttribute('aria-label'), 'right 关节 1 角度');
  for (const view of ['cartesian', 'teleop', 'calibration', 'recording', 'config', 'overview']) {
    await page.locator(`[data-view=${view}]`).click();
    assert.equal(await page.locator(`#view-${view}`).isVisible(), true);
  }
  await page.locator('[data-zero=left]').click();
  assert.match(await page.locator('#confirm-message').innerText(), /模型 0°/);
  await page.locator('#confirm-dialog button[value=cancel]').click();
  assert(await page.evaluate(async () => (await (await fetch('/api/state')).json()).joints.left_joint_0.position > .1));
  await page.locator('[data-zero=dual]').click();
  await page.locator('#confirm-dialog button[value=confirm]').click();
  await page.waitForFunction(async () => (await (await fetch('/api/state')).json()).joints.left_joint_0.position < .005);
  await page.setViewportSize({ width: 390, height: 844 });
  await page.waitForTimeout(400);
  const overflow = await page.evaluate(() => document.documentElement.scrollWidth > innerWidth);
  assert.equal(overflow, false, 'No mobile horizontal overflow');
  await page.locator('#fit-view').click();
  await page.screenshot({ path: '/tmp/ieir-web-mobile.png', fullPage: true });
  assert(await page.evaluate(() => {
    window.consoleScene.renderer.render(window.consoleScene.scene, window.consoleScene.camera);
    const source = document.querySelector('#viewport canvas');
    const canvas = document.createElement('canvas'); canvas.width = canvas.height = 100;
    const context = canvas.getContext('2d'); context.drawImage(source, 0, 0, 100, 100);
    const data = context.getImageData(0, 0, 100, 100).data;
    let dark = 0; for (let i = 0; i < data.length; i += 4) if (data[i] < 180) dark++;
    return dark > 50;
  }), 'Mobile canvas must show robot pixels');
  assert.equal(errors.length, 0, errors.join('\n'));
  console.log('PASS', JSON.stringify({ ...pixels, streaming }), 'all views, zero confirmation/cancel, preview target, orbit, desktop/mobile');
  await browser.close();
})().catch(error => { console.error(error); process.exit(1); });
