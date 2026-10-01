/** Cubic Bezier curves with an arc-length table, matching what Pedro's BezierCurve follows. */
import { type Vec } from './geom.ts';

export type Cubic = [Vec, Vec, Vec, Vec];

export function point(c: Cubic, t: number): Vec {
  const u = 1 - t;
  const a = u * u * u;
  const b = 3 * u * u * t;
  const d = 3 * u * t * t;
  const e = t * t * t;
  return {
    x: a * c[0].x + b * c[1].x + d * c[2].x + e * c[3].x,
    y: a * c[0].y + b * c[1].y + d * c[2].y + e * c[3].y,
  };
}

export function derivative(c: Cubic, t: number): Vec {
  const u = 1 - t;
  const a = 3 * u * u;
  const b = 6 * u * t;
  const d = 3 * t * t;
  return {
    x: a * (c[1].x - c[0].x) + b * (c[2].x - c[1].x) + d * (c[3].x - c[2].x),
    y: a * (c[1].y - c[0].y) + b * (c[2].y - c[1].y) + d * (c[3].y - c[2].y),
  };
}

export function secondDerivative(c: Cubic, t: number): Vec {
  const u = 1 - t;
  return {
    x: 6 * u * (c[2].x - 2 * c[1].x + c[0].x) + 6 * t * (c[3].x - 2 * c[2].x + c[1].x),
    y: 6 * u * (c[2].y - 2 * c[1].y + c[0].y) + 6 * t * (c[3].y - 2 * c[2].y + c[1].y),
  };
}

/** Signed curvature, 1/in (positive turns left). */
export function curvature(c: Cubic, t: number): number {
  const d = derivative(c, t);
  const dd = secondDerivative(c, t);
  const s = Math.sqrt(d.x * d.x + d.y * d.y);
  if (s < 1e-9) return 0;
  return (d.x * dd.y - d.y * dd.x) / (s * s * s);
}

/** Cumulative arc length at evenly spaced parameters, for converting between t and completion (arc-length fraction). */
export class ArcTable {
  readonly length: number;
  private readonly cumulative: Float64Array;
  private readonly n: number;

  readonly curve: Cubic;

  constructor(curve: Cubic, n = 256) {
    this.curve = curve;
    this.n = n;
    this.cumulative = new Float64Array(n + 1);
    let prev = point(curve, 0);
    for (let i = 1; i <= n; i++) {
      const p = point(curve, i / n);
      const dx = p.x - prev.x;
      const dy = p.y - prev.y;
      this.cumulative[i] = this.cumulative[i - 1] + Math.sqrt(dx * dx + dy * dy);
      prev = p;
    }
    this.length = this.cumulative[n];
  }

  /** Arc-length fraction reached at parameter t (Pedro's Curve.pathCompletion). */
  completion(t: number): number {
    if (this.length <= 0) return t;
    const x = Math.min(Math.max(t, 0), 1) * this.n;
    const i = Math.min(Math.floor(x), this.n - 1);
    const s = this.cumulative[i] + (this.cumulative[i + 1] - this.cumulative[i]) * (x - i);
    return s / this.length;
  }

  /** Parameter at an arc-length fraction (Pedro's Curve.parameter). */
  parameter(completion: number): number {
    if (this.length <= 0) return completion;
    const s = Math.min(Math.max(completion, 0), 1) * this.length;
    let lo = 0;
    let hi = this.n;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (this.cumulative[mid] <= s) lo = mid;
      else hi = mid;
    }
    const span = this.cumulative[hi] - this.cumulative[lo];
    const f = span > 0 ? (s - this.cumulative[lo]) / span : 0;
    return (lo + f) / this.n;
  }
}

const tables = new WeakMap<Cubic, ArcTable>();

/** The arc-length table for a curve, built once per curve object. */
export function tableFor(curve: Cubic): ArcTable {
  let t = tables.get(curve);
  if (!t) {
    t = new ArcTable(curve, 128);
    tables.set(curve, t);
  }
  return t;
}
