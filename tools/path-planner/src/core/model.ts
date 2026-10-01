/**
 * Predicted speed along a path, driven the way Pedro's Foresight follower drives it, by the drivetrain in drivetrain.ts.
 *
 * Pedro gives its corrections first call on the wheels (the centripetal push that keeps it on a curve, and the heading
 * control), pushes along the path with whatever is left, and brakes only for the end of the path. It never slows down
 * ahead of a curve: if a curve needs more sideways push than the wheels have left, the robot drifts wide until it has
 * coasted slow enough, and Pedro then pulls it back. The model does the same and estimates how far wide it drifts, so
 * the optimizer can avoid paths Pedro can't hold.
 *
 * A path that turns back on itself has to stop there; that stop is taken halfway between the two samples around it.
 * (Pedro won't make that stop mid-path; the panel says to split the path there.)
 */
import type { Sample } from './chain.ts';
import { drivetrain } from './drivetrain.ts';
import { wrap } from './geom.ts';
import type { ModelConfig, RobotConfig } from './project.ts';

export type Limit = 'direction' | 'turn' | 'curvature' | 'accel' | 'grip' | 'decel' | 'stop' | 'shot';

export interface Profile {
  /** Speed at each sample, in/s. */
  v: Float64Array;
  /** Time at each sample, s. */
  time: Float64Array;
  /** What holds the speed down at each sample. */
  limit: Limit[];
  /** Total time, including the stop overhead if the path ends stopped. */
  total: number;
  /** How far wide of the path the robot has drifted at each sample, in. */
  drift: Float64Array;
  /** The widest drift and the arc length where it happens. */
  worstDrift: number;
  driftAt: number;
  /** How far the heading has fallen behind the plan at each sample (it can't turn that fast there), rad. */
  lag: Float64Array;
  worstLag: number;
  lagAt: number;
  /** Places where the path turns back on itself (more than 90° between samples). Pedro won't stop there mid-path. */
  reversals: number;
  /** Per speed cap: arc length where Pedro's capped stretch has to begin so it has coasted down by the cap's `from`. */
  capStarts: number[];
}

/**
 * Drive no faster than `vmax` from `from` to `to` (arc length): Pedro's maxPathSpeed on that stretch of the path. Pedro
 * doesn't slow down ahead of it, and on a capped stretch it cuts drive power and coasts while it's too fast, so the
 * capped stretch starts early enough to coast down in time (`capStarts`); with `fixedStart`, it starts at `from`.
 */
export interface SpeedCap {
  from: number;
  to: number;
  vmax: number;
  fixedStart?: boolean;
}

export interface DriveSettings {
  model: ModelConfig;
  robot: RobotConfig;
}

/**
 * Where w1·|c1·y + d1| + w2·|c2·y + d2| + k ≤ budget holds for y ≥ 0: writes [lo, hi] into `out` and returns true,
 * or returns false if nowhere. The left side is convex and piecewise linear in y with at most two corners, so the set
 * is one interval. (Written without allocating: it runs a few times per sample, for every path the optimizer tries.)
 */
function within(c1: number, d1: number, w1: number, c2: number, d2: number, w2: number, k: number, budget: number, out: Float64Array): boolean {
  const f = (y: number) => w1 * Math.abs(c1 * y + d1) + w2 * Math.abs(c2 * y + d2) + k;
  // Corners at y > 0, sorted, then the lowest point (at 0 or a corner).
  let a = Math.abs(c1) > 1e-12 ? -d1 / c1 : -1;
  let b = Math.abs(c2) > 1e-12 ? -d2 / c2 : -1;
  if (a > b) {
    const t = a;
    a = b;
    b = t;
  }
  let best = 0;
  let fBest = f(0);
  if (a > 0 && f(a) < fBest) {
    best = a;
    fBest = f(a);
  }
  if (b > 0 && f(b) < fBest) {
    best = b;
    fBest = f(b);
  }
  if (fBest > budget + 1e-12) return false;
  // Right end: from the lowest point, f only rises; find where it reaches the budget.
  let x = best;
  let hi = 1e9;
  for (let q = 0; q < 3; q++) {
    const c = q === 0 ? a : q === 1 ? b : Infinity;
    if (c <= x) continue;
    const fc = c === Infinity ? Infinity : f(c);
    if (fc > budget) {
      const y = c === Infinity ? x + 1 : (x + c) / 2;
      const m = w1 * c1 * Math.sign(c1 * y + d1) + w2 * c2 * Math.sign(c2 * y + d2);
      hi = m > 1e-12 ? x + (budget - f(x)) / m : 1e9;
      break;
    }
    x = c;
  }
  // Left end, down to 0.
  x = best;
  let lo = 0;
  for (let q = 0; q < 3; q++) {
    const c = q === 0 ? b : q === 1 ? a : 0;
    if (c >= x || c < 0) continue;
    if (f(c) > budget) {
      const y = (x + c) / 2;
      const m = w1 * c1 * Math.sign(c1 * y + d1) + w2 * c2 * Math.sign(c2 * y + d2);
      lo = m < -1e-12 ? x + (budget - f(x)) / m : x;
      break;
    }
    x = c;
  }
  out[0] = Math.max(0, lo);
  out[1] = hi;
  return true;
}

const voltsRange = new Float64Array(2);
const gripRange = new Float64Array(2);

/** Pedro's translational correction pulls a drift back in roughly this long, s. */
const RECOVERY = 0.3;

export function speedProfile(samples: Sample[], settings: DriveSettings, opts: { endStopped: boolean; voltage?: number; caps?: SpeedCap[] }): Profile {
  const { model } = settings;
  const dt = drivetrain(model, settings.robot);
  const k = (opts.voltage ?? 12.5) / model.nominalVoltage; // the battery's voltage budget, relative to the motor specs
  const n = samples.length;
  const v = new Float64Array(n);
  const drift = new Float64Array(n);
  const limit: Limit[] = new Array(n).fill('accel');

  // Geometry per sample in the robot's frame: direction of travel (A), curvature, heading rate and its change.
  const cosA = new Float64Array(n);
  const sinA = new Float64Array(n);
  const kappa = new Float64Array(n); // signed, at least as sharp as the bend to a neighbour
  const hRate = new Float64Array(n); // dh/ds, rad/in
  const hCurve = new Float64Array(n); // d²h/ds², rad/in², over ±3 in (the heading is drawn as straight pieces)
  const coast = new Float64Array(n); // friction deceleration in this direction, in/s²
  const reversesAfter = new Uint8Array(n);
  for (let i = 0; i + 1 < n; i++) if (Math.abs(wrap(samples[i + 1].travel - samples[i].travel)) > Math.PI / 2) reversesAfter[i] = 1;
  for (let i = 0; i < n; i++) {
    const a = samples[i].travel - samples[i].h;
    cosA[i] = Math.cos(a);
    sinA[i] = Math.sin(a);
    coast[i] = 1 / (Math.abs(cosA[i]) / dt.coastForward + Math.abs(sinA[i]) / dt.coastStrafe);
    let kap = samples[i].kappa;
    for (const j of [i - 1, i + 1]) {
      if (j < 0 || j >= n || reversesAfter[Math.min(i, j)]) continue;
      const step = Math.abs(samples[j].s - samples[i].s);
      if (step < 1e-9) continue;
      const bend = wrap(samples[j].travel - samples[i].travel) * (j > i ? 1 : -1);
      if (Math.abs(bend) / step > Math.abs(kap)) kap = bend / step;
    }
    kappa[i] = kap;
    const a0 = samples[Math.max(0, i - 1)];
    const a1 = samples[Math.min(n - 1, i + 1)];
    hRate[i] = a1.s - a0.s > 1e-9 ? (a1.h - a0.h) / (a1.s - a0.s) : 0;
  }
  for (let i = 0, lo = 0, hi = 0; i < n; i++) {
    while (samples[i].s - samples[lo].s > 3) lo++;
    while (hi < n - 1 && samples[hi + 1].s - samples[i].s <= 3) hi++;
    const span = samples[hi].s - samples[lo].s;
    hCurve[i] = span > 1e-6 ? (hRate[hi] - hRate[lo]) / span : 0;
  }

  /**
   * What Pedro can do at sample i, moving at speed u, while delivering a share `share` of the corrections (centripetal
   * push towards the inside of the curve, and the heading's angular acceleration).
   *
   * Per wheel, voltage = speed term + force term, so with x the acceleration along the path:
   *   X = u·cos A/vF + (x·cos A − an·sin A)/aF,   Y = u·sin A/vS + (x·sin A + an·cos A)/aS,   Ω = ω/ωMax + α/αMax
   * and the wheels need |X| + |Y| + |Ω| ≤ battery. Grip needs |ax| + |ay| + |α|·spinShare ≤ grip·g.
   * Pedro's drive power along the path is never negative mid-path, so the most it slows is with drive power at zero,
   * where the motors' back-EMF brakes it; `xMin` is that acceleration. Returns the largest x it can reach, or
   * NaN if the corrections don't fit even at xMin.
   */
  const reach = (i: number, u: number, share: number, budget: number, back = false) => {
    const c = cosA[i];
    const sn = sinA[i];
    const an = u * u * kappa[i] * share;
    const alpha = u * u * hCurve[i] * share;
    const omegaTerm = Math.abs((share * u * hRate[i]) / dt.omegaMax + alpha / dt.alphaMax);
    // X and Y are linear in x: X = x·xc + xd, Y = x·yc + yd.
    const xc = c / dt.aForward;
    const xd = (u * c) / dt.vForward - (an * sn) / dt.aForward;
    const yc = sn / dt.aStrafe;
    const yd = (u * sn) / dt.vStrafe + (an * c) / dt.aStrafe;
    // Drive power along the path is the command's projection on the travel direction; zero at x = xMin.
    const proj = c * xc + sn * yc;
    const xMin = -(c * xd + sn * yd) / proj;
    const sign = back ? -1 : 1;
    // Search y ≥ 0 from the start of the allowed range: forwards x = xMin + y, or (braking for the end) x = −y.
    const from = back ? 0 : xMin;
    if (!within(sign * xc, xc * from + xd, 1, sign * yc, yc * from + yd, 1, omegaTerm, budget, voltsRange)) return NaN;
    if (!within(sign * c, c * from - an * sn, 1, sign * sn, sn * from + an * c, 1, Math.abs(alpha) * dt.spinShare, dt.traction, gripRange)) return NaN;
    const top = Math.min(voltsRange[1], gripRange[1]);
    if (Math.max(voltsRange[0], gripRange[0]) > top) return NaN;
    return from + sign * top;
  };

  // Speed caps: the cap inside, and before it the speed it could still coast down from in time (per sample, the
  // lowest over the caps ahead, and which cap that is).
  const caps = opts.caps ?? [];
  const envelope = new Float64Array(n).fill(Infinity);
  const envCap = new Int16Array(n).fill(-1);
  const inside = new Int16Array(n).fill(-1);
  const capStarts = caps.map((c) => c.from);
  caps.forEach((c, ci) => {
    let env = c.vmax;
    for (let i = n - 1; i >= 0; i--) {
      const si = samples[i].s;
      if (si > c.to) continue;
      if (si >= c.from) {
        inside[i] = ci;
      } else {
        if (c.fixedStart) break;
        env = Math.sqrt(env * env + 2 * coast[i] * (samples[i + 1].s - si));
      }
      const e = si >= c.from ? c.vmax : env;
      if (e < envelope[i]) {
        envelope[i] = e;
        envCap[i] = ci;
      }
    }
  });
  const capActive = new Int16Array(n).fill(-1);

  // Forward: from rest, full drive power with what the corrections leave.
  let lateralErr = 0; // in/s, drifting outwards
  let wide = 0; // in
  let behind = 0; // rad, heading behind the plan
  const lag = new Float64Array(n);
  for (let i = 0; i + 1 < n; i++) {
    const ds = samples[i + 1].s - samples[i].s;
    const u = v[i];
    let a = reach(i, u, 1, k);
    let shortfall = 0;
    let turnShortfall = 0;
    limit[i + 1] = 'accel';
    if (Number.isNaN(a)) {
      // The corrections don't fit even with drive power at zero: Pedro gives them all it has (drive at zero, so the
      // motors brake it). The centripetal push it can't deliver becomes drift outwards, and the turning it can't keep
      // up with leaves the heading behind.
      let lo = 0;
      let hi = 1;
      for (let it = 0; it < 8; it++) {
        const mid = (lo + hi) / 2;
        if (Number.isNaN(reach(i, u, mid, k))) hi = mid;
        else lo = mid;
      }
      a = reach(i, u, lo, k);
      if (Number.isNaN(a)) a = -coast[i];
      shortfall = Math.abs(u * u * kappa[i]) * (1 - lo);
      turnShortfall = Math.abs(u * hRate[i]) * (1 - lo);
      limit[i + 1] = 'grip';
    }
    if (limit[i + 1] === 'grip') a -= coast[i]; // friction on top of the motors' braking while it coasts
    // On a capped stretch and too fast: Pedro cuts drive power and coasts.
    const capped = capActive[i] >= 0 ? capActive[i] : -1;
    if (capped >= 0 && u > caps[capped].vmax + 1e-9) a = Math.min(a, -coast[i]);
    const next0 = Math.sqrt(Math.max(0.25, u * u + 2 * a * ds)); // never quite stops on its own
    const step = (2 * ds) / Math.max(u + next0, 1e-3);
    if (limit[i + 1] === 'grip') {
      lateralErr += shortfall * step;
      wide += lateralErr * step;
      behind += turnShortfall * step;
    } else {
      lateralErr = 0;
      wide *= Math.exp(-step / RECOVERY);
      behind *= Math.exp(-step / RECOVERY);
      const pushing = a > 0.02 * dt.aForward;
      const turning = Math.abs(u * hRate[i]) / dt.omegaMax > 0.4;
      limit[i + 1] = pushing ? 'accel' : turning ? 'turn' : Math.abs(u * u * kappa[i]) > 0.2 * dt.traction ? 'curvature' : 'direction';
    }
    drift[i + 1] = wide;
    lag[i + 1] = behind;
    let next = next0;
    if (reversesAfter[i]) {
      // Turning back between these samples: stop halfway and set off again from rest.
      const launch = Math.max(0, reach(i + 1, 0, 1, k) || 0);
      next = Math.min(next, Math.sqrt(launch * ds)); // v² = 2·a·(ds / 2)
      limit[i + 1] = 'curvature';
    }
    // Speed caps: stay on a capped stretch until its end; start one where the speed has to begin coasting down for
    // it, or on reaching it.
    let on = capped >= 0 && samples[i + 1].s <= caps[capped].to ? capped : -1;
    if (on < 0 && envCap[i + 1] >= 0 && (next > envelope[i + 1] + 1e-9 || inside[i + 1] === envCap[i + 1])) {
      on = envCap[i + 1];
      if (next > envelope[i + 1] + 1e-9) capStarts[on] = Math.min(capStarts[on], samples[i].s);
    }
    if (on >= 0) {
      capActive[i + 1] = on;
      next = Math.min(next, Math.max(caps[on].vmax, envelope[i + 1]));
      if (inside[i + 1] === on || u > caps[on].vmax + 1e-9) limit[i + 1] = 'shot';
    }
    v[i + 1] = next;
  }

  // Backward: brake for the end of the path and for any reversal. Pedro's end braking can reverse the motors, which
  // pushes harder than driving (the back-EMF adds to the battery); friction helps; corrections still come first.
  const brake = (i: number, u: number) => {
    const x = reach(i, u, 1, k, true);
    return (Number.isNaN(x) ? 0 : Math.max(0, -x)) + coast[i];
  };
  if (opts.endStopped) {
    v[n - 1] = 0;
    limit[n - 1] = 'stop';
  }
  for (let i = n - 2; i >= 0; i--) {
    const ds = samples[i + 1].s - samples[i].s;
    const r = reversesAfter[i] ? Math.sqrt(brake(i, v[i]) * ds) : Math.sqrt(v[i + 1] * v[i + 1] + 2 * brake(i, v[i + 1]) * ds);
    if (r < v[i]) {
      v[i] = r;
      limit[i] = 'decel';
    }
  }

  const time = new Float64Array(n);
  for (let i = 1; i < n; i++) {
    const ds = samples[i].s - samples[i - 1].s;
    let step: number;
    if (ds <= 0) step = 0;
    else if (reversesAfter[i - 1]) step = ds / Math.max(v[i - 1], 1e-3) + ds / Math.max(v[i], 1e-3); // stop in the middle
    else step = (2 * ds) / Math.max(v[i] + v[i - 1], 1e-3);
    time[i] = time[i - 1] + step;
  }
  let worstDrift = 0;
  let driftAt = 0;
  for (let i = 0; i < n; i++) {
    if (drift[i] > worstDrift) {
      worstDrift = drift[i];
      driftAt = samples[i].s;
    }
  }
  let worstLag = 0;
  let lagAt = 0;
  for (let i = 0; i < n; i++) {
    if (lag[i] > worstLag) {
      worstLag = lag[i];
      lagAt = samples[i].s;
    }
  }
  const total = time[n - 1] + (opts.endStopped ? model.stopOverhead : 0);
  let reversals = 0;
  for (let i = 0; i + 1 < n; i++) reversals += reversesAfter[i];
  return { v, time, limit, total, drift, worstDrift, driftAt, lag, worstLag, lagAt, reversals, capStarts };
}
