/**
 * The field: draws the field, the paths and the robot, and is where paths are edited.
 * Drag a point to move it (it snaps onto other paths' ends), drag the arrow to turn it, double-click to add a point,
 * Delete removes the selected point, click another path to switch to it.
 */
import { sampleChain, type PlannedSegment, type Sample } from '../core/chain.ts';
import { CENTERLINE_X, FIELD_MARKINGS, fieldObstacles, WALL_MAX, WALL_MIN } from '../core/field.ts';
import { convexHull, deg, offsetConvex, pointInConvex, pointSegmentDistance, rad, wrap, type Polygon, type Pose, type Vec } from '../core/geom.ts';
import { speedProfile, type Profile } from '../core/model.ts';
import type { Plan } from '../core/optimize.ts';
import { marginsFor, type Chain, type ChainPoint } from '../core/project.ts';
import { localFootprint, placeFootprint } from '../core/robot.ts';
import { changed, planStatus, pointIssues, selectedChain, state } from './state.ts';

const canvas = document.getElementById('field') as HTMLCanvasElement;
const wrapEl = document.getElementById('fieldWrap') as HTMLDivElement;
const hud = document.getElementById('hud') as HTMLDivElement;
const ctx = canvas.getContext('2d')!;

let size = 400;
let dpr = 1;
const PAD = 8;
const scale = () => (size - 2 * PAD) / 144;
const toPx = (p: Vec): [number, number] => [PAD + p.x * scale(), size - PAD - p.y * scale()];
const toField = (px: number, py: number): Vec => ({ x: (px - PAD) / scale(), y: (size - PAD - py) / scale() });
const css = (name: string) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

export interface Derived {
  samples: Sample[];
  profile: Profile;
}
let cache = new WeakMap<Plan, Derived>();

export function clearDerived(): void {
  cache = new WeakMap();
}

/** Samples and the predicted speed of a plan (cached). */
export function derive(plan: Plan, chain: Chain): Derived {
  let d = cache.get(plan);
  if (!d) {
    const samples = sampleChain(plan.segments, 0.5).samples;
    d = { samples, profile: speedProfile(samples, state.settings, { endStopped: chain.endStopped }) };
    cache.set(plan, d);
  }
  return d;
}

export function poseAtTime(d: Derived, t: number): Pose {
  const time = d.profile.time;
  let lo = 0;
  let hi = d.samples.length - 1;
  if (t <= 0) return { ...d.samples[0].p, h: d.samples[0].h };
  if (t >= time[hi]) return { ...d.samples[hi].p, h: d.samples[hi].h };
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (time[mid] <= t) lo = mid;
    else hi = mid;
  }
  const f = (t - time[lo]) / Math.max(1e-9, time[hi] - time[lo]);
  const a = d.samples[lo];
  const b = d.samples[hi];
  return { x: a.p.x + (b.p.x - a.p.x) * f, y: a.p.y + (b.p.y - a.p.y) * f, h: a.h + (b.h - a.h) * f };
}

export function resize(): void {
  const r = wrapEl.getBoundingClientRect();
  size = Math.max(200, Math.floor(Math.min(r.width, r.height)));
  dpr = window.devicePixelRatio || 1;
  canvas.style.width = `${size}px`;
  canvas.style.height = `${size}px`;
  canvas.width = Math.round(size * dpr);
  canvas.height = Math.round(size * dpr);
  draw();
}

function poly(points: Vec[], close = true): void {
  ctx.beginPath();
  points.forEach((p, i) => {
    const [x, y] = toPx(p);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  if (close) ctx.closePath();
}

function curves(segments: PlannedSegment[]): void {
  ctx.beginPath();
  segments.forEach((s, i) => {
    const [a, b, c, d] = s.curve.map(toPx);
    if (i === 0) ctx.moveTo(a[0], a[1]);
    else ctx.lineTo(a[0], a[1]);
    ctx.bezierCurveTo(b[0], b[1], c[0], c[1], d[0], d[1]);
  });
}

function robot(pose: Pose, stroke: string, fill: string | null, width: number, arrow = true): void {
  for (const part of placeFootprint(localFootprint(state.settings.robot), pose)) {
    poly(part);
    if (fill) {
      ctx.fillStyle = fill;
      ctx.fill();
    }
    ctx.strokeStyle = stroke;
    ctx.lineWidth = width;
    ctx.stroke();
  }
  if (arrow) headingArrow(pose, stroke, width);
}

function arrowTip(pose: Pose): Vec {
  const r = state.settings.robot.length / 2 + 4;
  return { x: pose.x + Math.cos(pose.h) * r, y: pose.y + Math.sin(pose.h) * r };
}

function headingArrow(pose: Pose, color: string, width: number): void {
  const [cx, cy] = toPx(pose);
  const [tx, ty] = toPx(arrowTip(pose));
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(tx, ty);
  ctx.stroke();
  const a = Math.atan2(ty - cy, tx - cx);
  ctx.beginPath();
  ctx.moveTo(tx + 2 * Math.cos(a), ty + 2 * Math.sin(a));
  ctx.lineTo(tx - 7 * Math.cos(a - 0.5), ty - 7 * Math.sin(a - 0.5));
  ctx.lineTo(tx - 7 * Math.cos(a + 0.5), ty - 7 * Math.sin(a + 0.5));
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.fill();
}

function badge(text: string, at: Vec, fill: string, selected: boolean): void {
  const [x, y] = toPx(at);
  const r = selected ? 10 : 8.5;
  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fillStyle = fill;
  ctx.fill();
  ctx.lineWidth = selected ? 2.5 : 1.5;
  ctx.strokeStyle = css('--view');
  ctx.stroke();
  ctx.fillStyle = '#fff';
  ctx.font = `700 ${selected ? 11 : 10}px ${css('--font')}`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, x, y + 0.5);
  ctx.textAlign = 'start';
  ctx.textBaseline = 'alphabetic';
}

function label(text: string, at: Vec, color: string, dx = 12, dy = -10): void {
  const [x, y] = toPx(at);
  ctx.font = `500 11px ${css('--font')}`;
  ctx.lineWidth = 3;
  ctx.strokeStyle = css('--view');
  ctx.strokeText(text, x + dx, y + dy);
  ctx.fillStyle = color;
  ctx.fillText(text, x + dx, y + dy);
}

const pointPose = (p: ChainPoint): Pose => ({ x: p.x, y: p.y, h: rad(p.heading) });

export function draw(): void {
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, size, size);
  const chain = selectedChain();
  const margins = chain ? marginsFor(state.settings, chain) : state.settings.margins;
  const field = [
    { x: WALL_MIN, y: WALL_MIN },
    { x: WALL_MAX, y: WALL_MIN },
    { x: WALL_MAX, y: WALL_MAX },
    { x: WALL_MIN, y: WALL_MAX },
  ];
  poly(field);
  ctx.fillStyle = css('--tile');
  ctx.fill();
  ctx.strokeStyle = css('--tile-line');
  ctx.lineWidth = 1;
  const tile = (WALL_MAX - WALL_MIN) / 6;
  for (let i = 1; i < 6; i++) {
    const v = WALL_MIN + i * tile;
    poly([{ x: v, y: WALL_MIN }, { x: v, y: WALL_MAX }], false);
    ctx.stroke();
    poly([{ x: WALL_MIN, y: v }, { x: WALL_MAX, y: v }], false);
    ctx.stroke();
  }
  // the far half is off limits in AUTO, unless this path may cross
  if (!chain?.allowCrossing) {
    poly([{ x: CENTERLINE_X, y: WALL_MIN }, { x: WALL_MAX, y: WALL_MIN }, { x: WALL_MAX, y: WALL_MAX }, { x: CENTERLINE_X, y: WALL_MAX }]);
    ctx.fillStyle = css('--off-limits');
    ctx.fill();
  }
  for (const m of FIELD_MARKINGS) {
    poly(m.polygon);
    const red = m.name.startsWith('Red');
    ctx.fillStyle = css(red ? '--red-fill' : '--blue-fill');
    ctx.fill();
    ctx.strokeStyle = css(red ? '--red' : '--blue');
    ctx.lineWidth = 1;
    ctx.stroke();
  }
  ctx.save();
  ctx.setLineDash([6, 5]);
  ctx.strokeStyle = css('--wall');
  ctx.lineWidth = 1.5;
  poly([{ x: CENTERLINE_X, y: WALL_MIN }, { x: CENTERLINE_X, y: WALL_MAX }], false);
  ctx.stroke();
  ctx.restore();
  poly(field);
  ctx.strokeStyle = css('--wall');
  ctx.lineWidth = 3;
  ctx.stroke();
  for (const o of fieldObstacles(state.settings.robot.height + margins.headroom)) {
    poly(convexHull(offsetConvex(o.polygon, margins.obstacle)));
    ctx.fillStyle = css('--margin-fill');
    ctx.fill();
    poly(o.polygon);
    ctx.fillStyle = css('--obstacle');
    ctx.fill();
  }

  for (const c of state.chains) {
    if (c === chain) continue;
    const plan = state.plans.get(c.name);
    ctx.strokeStyle = css('--path-other');
    ctx.lineWidth = 2;
    if (plan) curves(plan.segments);
    else poly(c.points, false);
    ctx.stroke();
  }
  if (chain) drawChain(chain);
}

function drawChain(chain: Chain): void {
  const plan = state.plans.get(chain.name);
  const status = planStatus(chain);
  const progress = state.running?.chain === chain.name ? state.running.progress : null;
  const sel = state.selection;

  chain.keepOut.forEach((k, i) => {
    poly(convexHull(k.points as Polygon));
    ctx.fillStyle = css('--keepout-fill');
    ctx.fill();
    ctx.strokeStyle = css('--keepout');
    ctx.lineWidth = 1.5;
    ctx.stroke();
    k.points.forEach((v, j) => {
      const [x, y] = toPx(v);
      const on = sel?.kind === 'keepout' && sel.index === i && sel.vertex === j;
      ctx.fillStyle = on ? css('--keepout') : css('--view');
      ctx.strokeStyle = css('--keepout');
      ctx.beginPath();
      ctx.rect(x - 4, y - 4, 8, 8);
      ctx.fill();
      ctx.stroke();
    });
  });

  // areas
  chain.points.forEach((p, i) => {
    if (p.kind !== 'region') return;
    ctx.save();
    ctx.setLineDash([5, 4]);
    poly([
      { x: p.x - p.halfWidth, y: p.y - p.halfHeight },
      { x: p.x + p.halfWidth, y: p.y - p.halfHeight },
      { x: p.x + p.halfWidth, y: p.y + p.halfHeight },
      { x: p.x - p.halfWidth, y: p.y + p.halfHeight },
    ]);
    ctx.fillStyle = css('--region-fill');
    ctx.fill();
    ctx.strokeStyle = css('--region');
    ctx.lineWidth = sel?.kind === 'point' && sel.index === i ? 2.5 : 1.5;
    ctx.stroke();
    ctx.restore();
    // resize handle
    const [cx, cy] = toPx({ x: p.x + p.halfWidth, y: p.y + p.halfHeight });
    ctx.fillStyle = css('--region');
    ctx.fillRect(cx - 3.5, cy - 3.5, 7, 7);
  });

  const segs = progress?.segments ?? plan?.segments;
  if (segs) {
    curves(segs);
    ctx.save();
    if (progress || status === 'stale') ctx.setLineDash([7, 5]);
    ctx.strokeStyle = status === 'infeasible' && !progress ? css('--bad') : css('--path');
    ctx.lineWidth = 3;
    ctx.stroke();
    ctx.restore();
  } else {
    ctx.save();
    ctx.setLineDash([3, 5]);
    poly(chain.points, false);
    ctx.strokeStyle = css('--path');
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.restore();
  }

  if (plan && !progress) {
    const d = derive(plan, chain);
    // where the plan breaks a margin
    for (const c of plan.check.clearances) {
      if (c.gap >= c.margin - 1e-3) continue;
      const at = d.samples.reduce((best, s) => (Math.abs(s.s - c.at) < Math.abs(best.s - c.at) ? s : best), d.samples[0]);
      robot({ x: at.p.x, y: at.p.y, h: at.h }, css('--bad'), css('--bad-fill'), 2, false);
    }
    for (const m of plan.markers) {
      const s = d.samples.find((x) => x.seg === m.seg && x.t >= m.t) ?? d.samples[d.samples.length - 1];
      const [x, y] = toPx(s.p);
      ctx.beginPath();
      ctx.moveTo(x, y - 6);
      ctx.lineTo(x + 6, y);
      ctx.lineTo(x, y + 6);
      ctx.lineTo(x - 6, y);
      ctx.closePath();
      ctx.fillStyle = css('--marker');
      ctx.fill();
      label(m.name, s.p, css('--marker'), 9, 4);
    }
    if (state.playTime > 0 || state.playing) robot(poseAtTime(d, state.playTime), css('--robot'), css('--robot-fill'), 2);
  }


  // points; ones that can't keep the gaps where they are drawn in red
  const forced = new Set(pointIssues(chain).map((f) => f.point));
  chain.points.forEach((p, i) => {
    const on = sel?.kind === 'point' && sel.index === i;
    const bad = forced.has(i);
    if (p.kind === 'fixed') {
      robot(pointPose(p), css(bad ? '--bad' : on ? '--robot' : '--robot-dim'), bad ? css('--bad-fill') : null, on || bad ? 1.8 : 1, false);
      headingArrow(pointPose(p), css(bad ? '--bad' : on ? '--accent' : '--robot-dim'), on ? 2.5 : 1.5);
    }
    badge(String(i + 1), p, css(bad ? '--bad' : p.kind === 'region' ? '--region' : '--accent'), on);
  });
}

// ---- Editing ----

type Drag =
  | { kind: 'point'; index: number; mode: 'move' | 'turn' | 'size'; dx: number; dy: number }
  | { kind: 'keepout'; index: number; vertex: number };
let drag: Drag | null = null;

const nearPx = (a: Vec, b: Vec, px: number) => Math.hypot(a.x - b.x, a.y - b.y) * scale() <= px;

function hit(p: Vec, chain: Chain): Drag | null {
  for (let i = 0; i < chain.keepOut.length; i++) {
    const k = chain.keepOut[i];
    for (let j = 0; j < k.points.length; j++) if (nearPx(p, k.points[j], 7)) return { kind: 'keepout', index: i, vertex: j };
  }
  // the selected point first, so its arrow wins over neighbours
  const order = chain.points.map((_, i) => i);
  const sel = state.selection;
  if (sel?.kind === 'point') order.sort((a, b) => (a === sel.index ? -1 : b === sel.index ? 1 : 0));
  for (const i of order) {
    const pt = chain.points[i];
    if (pt.kind === 'fixed') {
      if (nearPx(p, arrowTip(pointPose(pt)), 9)) return { kind: 'point', index: i, mode: 'turn', dx: 0, dy: 0 };
      if (nearPx(p, pt, 11)) return { kind: 'point', index: i, mode: 'move', dx: p.x - pt.x, dy: p.y - pt.y };
    } else {
      if (nearPx(p, { x: pt.x + pt.halfWidth, y: pt.y + pt.halfHeight }, 8)) return { kind: 'point', index: i, mode: 'size', dx: 0, dy: 0 };
      if (Math.abs(p.x - pt.x) <= pt.halfWidth + 1 && Math.abs(p.y - pt.y) <= pt.halfHeight + 1) return { kind: 'point', index: i, mode: 'move', dx: p.x - pt.x, dy: p.y - pt.y };
    }
  }
  for (const pt of chain.points) {
    if (pt.kind !== 'fixed') continue;
    const parts = placeFootprint(localFootprint(state.settings.robot), pointPose(pt));
    const i = chain.points.indexOf(pt);
    if (parts.some((part) => pointInConvex(p, part))) return { kind: 'point', index: i, mode: 'move', dx: p.x - pt.x, dy: p.y - pt.y };
  }
  return null;
}

/** Another path whose drawn curve passes near p. */
function otherPathAt(p: Vec): Chain | null {
  for (const c of state.chains) {
    if (c.name === state.selected) continue;
    const plan = state.plans.get(c.name);
    const pts = plan ? sampleChain(plan.segments, 2).samples.map((s) => s.p) : c.points;
    for (let i = 1; i < pts.length; i++) if (pointSegmentDistance(p, pts[i - 1], pts[i]) * scale() < 6) return c;
  }
  return null;
}

const snap = (v: number, step: number) => Math.round(v / step) * step;

/** Ends of other paths, which points snap onto so paths join up. */
function snapTargets(): ChainPoint[] {
  return state.chains.filter((c) => c.name !== state.selected).flatMap((c) => [c.points[0], c.points[c.points.length - 1]]);
}

function pos(e: MouseEvent): Vec {
  const r = canvas.getBoundingClientRect();
  return toField(e.clientX - r.left, e.clientY - r.top);
}

function apply(d: Drag, p: Vec, fine: boolean): void {
  const chain = selectedChain();
  if (!chain) return;
  const step = fine ? 0.1 : 0.5;
  if (d.kind === 'keepout') {
    const v = chain.keepOut[d.index].points[d.vertex];
    v.x = snap(p.x, step);
    v.y = snap(p.y, step);
  } else {
    const pt = chain.points[d.index];
    if (d.mode === 'turn') {
      let h = snap(deg(Math.atan2(p.y - pt.y, p.x - pt.x)), fine ? 1 : 5);
      if (h <= -180) h += 360;
      pt.heading = h;
    } else if (d.mode === 'size' && pt.kind === 'region') {
      pt.halfWidth = Math.max(0, snap(p.x - pt.x, step));
      pt.halfHeight = Math.max(0, snap(p.y - pt.y, step));
    } else {
      pt.x = snap(p.x - d.dx, step);
      pt.y = snap(p.y - d.dy, step);
      if (pt.kind === 'fixed') {
        const target = snapTargets().find((t) => nearPx(pt, t, 10));
        if (target) {
          pt.x = target.x;
          pt.y = target.y;
          pt.heading = target.heading;
        }
      }
    }
  }
  changed('work');
}

canvas.addEventListener('pointerdown', (e) => {
  const p = pos(e);
  const chain = selectedChain();
  const h = chain && !state.running ? hit(p, chain) : null;
  if (h) {
    drag = h;
    canvas.setPointerCapture(e.pointerId);
    canvas.classList.add('grabbing');
    state.selection = h.kind === 'point' ? { kind: 'point', index: h.index } : { kind: 'keepout', index: h.index, vertex: h.vertex };
    changed('selection');
    return;
  }
  const other = otherPathAt(p);
  if (other) {
    state.selected = other.name;
    state.selection = null;
    state.playTime = 0;
    changed('selected');
    return;
  }
  if (state.selection) {
    state.selection = null;
    changed('selection');
  }
});

canvas.addEventListener('pointermove', (e) => {
  const p = pos(e);
  hud.textContent = `${p.x.toFixed(1)}, ${p.y.toFixed(1)}`;
  if (drag) return apply(drag, p, e.shiftKey);
  const chain = selectedChain();
  const h = chain ? hit(p, chain) : null;
  canvas.style.cursor = h
    ? state.running
      ? 'not-allowed'
      : h.kind === 'point' && h.mode === 'turn'
        ? 'alias'
        : 'grab'
    : otherPathAt(p)
      ? 'pointer'
      : 'crosshair';
});

const endDrag = () => {
  drag = null;
  canvas.classList.remove('grabbing');
};
canvas.addEventListener('pointerup', endDrag);
canvas.addEventListener('pointercancel', endDrag);
canvas.addEventListener('pointerleave', () => (hud.textContent = ''));

/** Adds a point at p to the leg it's closest to. */
export function addPointAt(p: Vec, kind: ChainPoint['kind'] = 'fixed'): void {
  const chain = selectedChain();
  if (!chain) return;
  let best = 0;
  let bestD = Infinity;
  for (let i = 0; i < chain.points.length - 1; i++) {
    const d = pointSegmentDistance(p, chain.points[i], chain.points[i + 1]);
    if (d < bestD) {
      bestD = d;
      best = i;
    }
  }
  const a = chain.points[best];
  const b = chain.points[best + 1];
  const heading = Math.round(deg(Math.atan2(b.y - a.y, b.x - a.x)));
  const pt: ChainPoint =
    kind === 'fixed'
      ? { kind: 'fixed', x: snap(p.x, 0.5), y: snap(p.y, 0.5), heading, headingTol: 180 }
      : { kind: 'region', x: snap(p.x, 0.5), y: snap(p.y, 0.5), halfWidth: 6, halfHeight: 6, heading, headingTol: 180 };
  chain.points.splice(best + 1, 0, pt);
  chain.legs.splice(best + 1, 0, { rules: [...(chain.legs[best]?.rules ?? [])] });
  for (const m of chain.markers) if (m.leg > best) m.leg++;
  state.selection = { kind: 'point', index: best + 1 };
  changed('work');
}

canvas.addEventListener('dblclick', (e) => {
  if (!state.running) addPointAt(pos(e));
});

export function deletePoint(index: number): void {
  const chain = selectedChain();
  if (!chain || index <= 0 || index >= chain.points.length - 1) return;
  chain.points.splice(index, 1);
  chain.legs.splice(index, 1);
  chain.markers = chain.markers.filter((m) => m.leg < chain.legs.length);
  state.selection = null;
  changed('work');
}

canvas.addEventListener('keydown', (e) => {
  const sel = state.selection;
  const chain = selectedChain();
  if (!chain || sel?.kind !== 'point' || state.running) return;
  if (e.key === 'Delete' || e.key === 'Backspace') {
    e.preventDefault();
    return deletePoint(sel.index);
  }
  const pt = chain.points[sel.index];
  const step = e.shiftKey ? 0.1 : 0.5;
  const moves: Record<string, () => void> = {
    ArrowLeft: () => (pt.x -= step),
    ArrowRight: () => (pt.x += step),
    ArrowUp: () => (pt.y += step),
    ArrowDown: () => (pt.y -= step),
    '[': () => (pt.heading = Math.round(deg(wrap(rad(pt.heading + (e.shiftKey ? 1 : 5)))))),
    ']': () => (pt.heading = Math.round(deg(wrap(rad(pt.heading - (e.shiftKey ? 1 : 5)))))),
  };
  const fn = moves[e.key];
  if (!fn) return;
  e.preventDefault();
  fn();
  pt.x = Math.round(pt.x * 10) / 10;
  pt.y = Math.round(pt.y * 10) / 10;
  changed('work');
});

new ResizeObserver(() => resize()).observe(wrapEl);
