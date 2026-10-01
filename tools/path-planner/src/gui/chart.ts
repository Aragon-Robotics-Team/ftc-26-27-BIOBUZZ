/** Speed along the selected chain: the prediction coloured by what limits it, plus any matched logged runs. */
import type { Limit } from '../core/model.ts';
import type { Derived } from './fieldView.ts';
import type { RunComparison } from '../core/logs.ts';

const canvas = document.getElementById('chart') as HTMLCanvasElement;
const legend = document.getElementById('chartLegend') as HTMLSpanElement;
const ctx = canvas.getContext('2d')!;
const css = (name: string) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

export const LIMIT_COLORS: Record<Limit, string> = {
  direction: '#2563eb',
  turn: '#7c3aed',
  curvature: '#0891b2',
  accel: '#16a34a',
  decel: '#d97706',
  stop: '#6b7280',
};
export const LIMIT_NAMES: Record<Limit, string> = {
  direction: 'top speed',
  turn: 'turning',
  curvature: 'curve',
  accel: 'speeding up',
  decel: 'braking',
  stop: 'stop',
};

legend.innerHTML = (Object.keys(LIMIT_COLORS) as Limit[])
  .filter((k) => k !== 'stop')
  .map((k) => `<span><i class="sw" style="border-color:${LIMIT_COLORS[k]}"></i>${LIMIT_NAMES[k]}</span>`)
  .join('');

export function drawChart(d: Derived | null, runs: RunComparison[], cursorTime: number): void {
  const r = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(100, r.width);
  const h = Math.max(80, r.height);
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  ctx.font = `11px ${css('--font')}`;
  if (!d) {
    return;
  }
  const L = { l: 34, r: 12, t: 10, b: 22 };
  const len = d.samples[d.samples.length - 1].s;
  let vmax = 10;
  for (const v of d.profile.v) vmax = Math.max(vmax, v);
  for (const c of runs) for (const v of c.run.measuredSpeed) vmax = Math.max(vmax, v);
  vmax = Math.ceil(vmax / 10) * 10;
  const X = (s: number) => L.l + (s / len) * (w - L.l - L.r);
  const Y = (v: number) => h - L.b - (v / vmax) * (h - L.t - L.b);
  // grid
  ctx.strokeStyle = css('--line');
  ctx.fillStyle = css('--dim');
  ctx.lineWidth = 1;
  for (let v = 0; v <= vmax; v += vmax > 60 ? 20 : 10) {
    ctx.beginPath();
    ctx.moveTo(L.l, Y(v));
    ctx.lineTo(w - L.r, Y(v));
    ctx.stroke();
    ctx.fillText(String(v), 6, Y(v) + 4);
  }
  const step = len > 120 ? 24 : 12;
  for (let s = 0; s <= len; s += step) ctx.fillText(String(s), X(s) - 6, h - 6);
  // prediction, one colour per limit
  ctx.lineWidth = 2.5;
  for (let i = 1; i < d.samples.length; i++) {
    ctx.beginPath();
    ctx.moveTo(X(d.samples[i - 1].s), Y(d.profile.v[i - 1]));
    ctx.lineTo(X(d.samples[i].s), Y(d.profile.v[i]));
    ctx.strokeStyle = LIMIT_COLORS[d.profile.limit[i]];
    ctx.stroke();
  }
  // logged runs
  ctx.save();
  ctx.setLineDash([2, 3]);
  ctx.lineWidth = 1.5;
  ctx.strokeStyle = css('--text');
  for (const c of runs) {
    ctx.beginPath();
    c.run.samples.forEach((smp, i) => {
      const x = X((smp.s / c.run.samples[c.run.samples.length - 1].s) * len);
      const y = Y(c.run.measuredSpeed[i]);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }
  ctx.restore();
  // playback cursor
  const time = d.profile.time;
  let i = 0;
  while (i < time.length - 1 && time[i + 1] < cursorTime) i++;
  const x = X(d.samples[i].s);
  ctx.strokeStyle = css('--dim');
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(x, L.t);
  ctx.lineTo(x, h - L.b);
  ctx.stroke();
}
