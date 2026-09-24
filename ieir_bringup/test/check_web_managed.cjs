// Browser-only lifecycle mock on an explicitly isolated preview. No ROS actions.
const { chromium } = require('playwright');
const assert = require('node:assert/strict');

(async () => {
  const browser = await chromium.launch({ headless: true, args: ['--no-sandbox', '--enable-unsafe-swiftshader'] });
  const page = await browser.newPage({ viewport: { width: 1440, height: 1050 } });
  const jobs = {}, requests = [], errors = [];
  let calibration = {};
  page.on('pageerror', e => errors.push(e.message));
  await page.route('**/api/bootstrap', async route => {
    const response = await route.fetch(); const data = await response.json();
    assert.equal(data.preview, true, 'Never run this test on hardware');
    await route.fulfill({ json: { ...data, manage_processes: true } });
  });
  await page.route('**/api/state', async route => {
    const response = await route.fetch(); const data = await response.json();
    await route.fulfill({ json: { ...data, processes: jobs, calibration } });
  });
  await page.route('**/api/action', async route => {
    const body = route.request().postDataJSON();
    if (!['process_start', 'process_stop', 'calibration_key', 'calibration_angle'].includes(body.action)) return route.continue();
    requests.push(body);
    if (body.action === 'process_start') {
      jobs[body.kind] = { pid: 123, label: `${body.kind} / UI mock`, running: true, stopping: false, exit_code: null };
      if (body.kind === 'calibration') calibration = { '/calibration_preview_mock': {
        age: 0, text: 'LIMIT REFERENCE REVIEW | right_joint_6 | -90 deg',
        positions: { right_joint_6: -Math.PI / 2 },
        metadata: { stage: body.stage, joint: 'right_joint_6', angle_deg: -90, backend: 'web' },
      } };
    } else if (body.action === 'process_stop') {
      jobs[body.kind].running = false; jobs[body.kind].exit_code = 0;
      if (body.kind === 'calibration') calibration = {};
    }
    await route.fulfill({ json: { message: 'Browser test acknowledgment; no ROS process' } });
  });
  try {
    await page.goto(process.env.CONSOLE_URL || 'http://127.0.0.1:18768');
    await page.waitForFunction(() => window.consoleMeshCount > 10, { timeout: 60000 });
    await page.locator('#environment').waitFor({ state: 'visible' });
    assert(await page.locator('#start-control').isDisabled());
    await page.locator('#unlock').click();
    await page.locator('#confirm-dialog button[value=confirm]').click();
    await page.locator('#start-control').click();
    assert.match(await page.locator('#confirm-message').innerText(), /自动使能/);
    await page.locator('#confirm-dialog button[value=cancel]').click();
    assert.equal(requests.length, 0);
    await page.locator('#start-control').click();
    await page.locator('#confirm-dialog button[value=confirm]').click();
    await page.waitForFunction(() => document.querySelector('[data-stop-kind=control]').disabled === false);
    await page.screenshot({ path: '/tmp/ieir-web-managed-desktop.png', fullPage: true });
    await page.locator('[data-view=calibration]').click();
    assert(await page.locator('#start-calibration').isDisabled(), 'Controller/calibration UI exclusivity');
    await page.locator('[data-view=overview]').click();
    await page.locator('[data-stop-kind=control]').click();
    await page.locator('#confirm-dialog button[value=confirm]').click();
    await page.locator('[data-view=calibration]').click();
    await page.locator('#view-calibration [data-arm=right]').click();
    await page.locator('#cal-stage').selectOption('limits');
    await page.locator('#start-calibration').click();
    await page.locator('#confirm-dialog button[value=confirm]').click();
    await page.locator('#cal-angle-row').waitFor({ state: 'visible' });
    assert.equal(await page.locator('#cal-angle').inputValue(), '-90.00');
    await page.locator('#cal-angle').fill('-89.5');
    await page.locator('#set-cal-angle').click();
    await page.locator('#confirm-dialog button[value=confirm]').click();
    await page.locator('[data-key=enter]').click();
    await page.locator('#confirm-dialog button[value=confirm]').click();
    await page.screenshot({ path: '/tmp/ieir-web-managed-calibration.png', fullPage: true });
    await page.setViewportSize({ width: 390, height: 844 });
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false);
    await page.screenshot({ path: '/tmp/ieir-web-managed-mobile.png', fullPage: true });
    await page.locator('[data-stop-kind=calibration]').click();
    await page.locator('#confirm-dialog button[value=confirm]').click();
    await page.locator('#cal-stage').selectOption('manual');
    await page.locator('#cal-joint').selectOption('4');
    await page.locator('#start-calibration').click();
    await page.locator('#confirm-dialog button[value=confirm]').click();
    assert(requests.some(r => r.kind === 'calibration' && r.stage === 'manual' && r.joint === 4 && r.side === 'right'));
    assert(requests.some(r => r.action === 'calibration_angle' && r.angle_deg === -89.5));
    assert(requests.some(r => r.action === 'calibration_key' && r.key === 'enter'));
    assert.equal(errors.length, 0, errors.join('\n'));
    console.log('PASS managed UI: confirmation/cancel, process exclusivity, start/stop, reference angle, manual joint selection, desktop/mobile');
  } finally { await browser.close(); }
})().catch(e => { console.error(e); process.exit(1); });
