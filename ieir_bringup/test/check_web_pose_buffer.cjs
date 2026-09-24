const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

(async () => {
  const source = fs.readFileSync(path.join(__dirname, '../web/pose-buffer.js'), 'utf8');
  const { PoseBuffer } = await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
  const buffer = new PoseBuffer(65);
  assert.equal(buffer.sample(0), null);
  buffer.push({ q: 0 }, 0);
  buffer.push({ q: 1 }, 100);
  assert.equal(buffer.sample(115).q, 0.5);
  assert.equal(buffer.sample(1000).q, 1, 'No extrapolation on packet loss');
  buffer.push({}, 200);
  assert.equal(buffer.sample(300), null, 'Stale feedback stops interpolation');
  buffer.push({ q: 0 }, 400);
  buffer.push({ q: -1 }, 440);
  const samples = Array.from({ length: 20 }, (_, i) => buffer.sample(450 + i * 10).q);
  assert(samples.every(q => q >= -1 && q <= 0));
  assert(new Set(samples).size > 3, 'Intermediate display frames, not only network samples');
  buffer.clear();
  assert.equal(buffer.sample(1000), null);
  console.log('PASS: display interpolation, no overshoot/extrapolation, stale packets, scene reset');
})().catch(error => { console.error(error); process.exit(1); });
