// Display-only interpolation. Never extrapolate motion beyond received feedback.
export class PoseBuffer {
  constructor(delayMs = 65) {
    this.delayMs = delayMs;
    this.samples = [];
  }
  clear() { this.samples = []; }
  push(positions, now) {
    if (!Object.keys(positions).length) { this.clear(); return; }
    this.samples.push({ positions, time: now });
    if (this.samples.length > 12) this.samples.shift();
  }
  sample(now) {
    if (!this.samples.length) return null;
    const time = now - this.delayMs;
    while (this.samples.length > 2 && this.samples[1].time <= time) this.samples.shift();
    const [a, b] = this.samples;
    if (!b || time <= a.time) return a.positions;
    const alpha = Math.min(1, (time - a.time) / Math.max(0.001, b.time - a.time));
    return Object.fromEntries(Object.entries(b.positions).map(([name, q]) =>
      [name, name in a.positions ? a.positions[name] + alpha * (q - a.positions[name]) : q]));
  }
}
