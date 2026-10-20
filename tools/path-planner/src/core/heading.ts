/**
 * Headings, written the only way Pedro 3.0.1 follows them faithfully: per Bezier segment, an
 * `Interpolator.piecewise()` of `Interpolator.linear(H[k], H[k+1])` pieces with breakpoints at arc-length fractions F.
 *
 * Pedro evaluates such a heading at curve parameter t like this (from the 3.0.1 bytecode):
 *   c = completion(t);  the piece is the one whose breakpoint is the first F ≥ c
 *   local = (t − parameter(F[k])) / (parameter(F[k+1]) − parameter(F[k]))      (parametric, not arc length)
 *   heading = H[k] + shortestError(H[k], H[k+1]) · completion(local)          (completion of the WHOLE curve)
 * so inside each piece the sweep is warped by the curve's speed profile near its start, while the breakpoints
 * themselves land exactly. `pedroHeading` reproduces that, so the planner predicts what the robot will do.
 */
import { ArcTable } from './bezier.ts';
import { wrap } from './geom.ts';

export interface HeadingPieces {
  /** Breakpoints, arc-length fractions: F[0] = 0 < F[1] < … < F[n] = 1. */
  F: number[];
  /** Headings at the breakpoints, radians, unwrapped (consecutive differences stay below π). */
  H: number[];
}

/** Heading at curve parameter t, exactly as Pedro computes it. */
export function pedroHeading(table: ArcTable, pieces: HeadingPieces, t: number): number {
  const c = table.completion(t);
  const { F, H } = pieces;
  // ceilingEntry over the breakpoints F[1..n]: the first F[k + 1] ≥ c
  let k = 0;
  while (k < F.length - 2 && F[k + 1] < c) k++;
  const tA = k === 0 ? 0 : table.parameter(F[k]);
  const tB = table.parameter(F[k + 1]);
  const local = tB > tA ? (t - tA) / (tB - tA) : 0;
  const err = wrap(H[k + 1] - H[k]);
  return H[k] + err * table.completion(Math.min(Math.max(local, 0), 1));
}

/**
 * Smooth heading through knots (s = arc length along the whole chain, h = unwrapped heading), with Catmull–Rom
 * tangents clamped so it never overshoots between knots (monotone cubic Hermite).
 */
export function smoothHeading(knotS: number[], knotH: number[]): (s: number) => number {
  const n = knotS.length;
  const m = new Array<number>(n).fill(0);
  const d: number[] = [];
  for (let i = 0; i < n - 1; i++) {
    const ds = knotS[i + 1] - knotS[i];
    d.push(ds > 1e-9 ? (knotH[i + 1] - knotH[i]) / ds : 0);
  }
  for (let i = 1; i < n - 1; i++) m[i] = d[i - 1] * d[i] <= 0 ? 0 : (d[i - 1] + d[i]) / 2;
  if (n >= 2) {
    m[0] = d[0];
    m[n - 1] = d[n - 2];
  }
  // Fritsch–Carlson limiter
  for (let i = 0; i < n - 1; i++) {
    if (Math.abs(d[i]) < 1e-12) {
      m[i] = 0;
      m[i + 1] = 0;
      continue;
    }
    const a = m[i] / d[i];
    const b = m[i + 1] / d[i];
    const r = a * a + b * b;
    if (r > 9) {
      const tau = 3 / Math.sqrt(r);
      m[i] = tau * a * d[i];
      m[i + 1] = tau * b * d[i];
    }
  }
  return (s: number) => {
    if (n === 1) return knotH[0];
    if (s <= knotS[0]) return knotH[0];
    if (s >= knotS[n - 1]) return knotH[n - 1];
    let i = 0;
    while (i < n - 2 && knotS[i + 1] < s) i++;
    const h = knotS[i + 1] - knotS[i];
    if (h <= 1e-12) return knotH[i + 1];
    const u = (s - knotS[i]) / h;
    const u2 = u * u;
    const u3 = u2 * u;
    return (
      (2 * u3 - 3 * u2 + 1) * knotH[i] +
      (u3 - 2 * u2 + u) * h * m[i] +
      (-2 * u3 + 3 * u2) * knotH[i + 1] +
      (u3 - u2) * h * m[i + 1]
    );
  };
}

/**
 * Sample a smooth heading into linear pieces for one segment that spans [s0, s0 + length] of the chain.
 * Pieces are about `spacing` inches long, split further wherever the heading would change by more than `maxStep`.
 */
export function toPieces(heading: (s: number) => number, s0: number, length: number, spacing = 4, maxStep = 0.5): HeadingPieces {
  const n = Math.max(2, Math.ceil(length / spacing));
  const F: number[] = [0];
  const H: number[] = [heading(s0)];
  for (let i = 1; i <= n; i++) {
    const f = i / n;
    const h = heading(s0 + f * length);
    const prevF = F[F.length - 1];
    const prevH = H[H.length - 1];
    const steps = Math.ceil(Math.abs(h - prevH) / maxStep);
    for (let j = 1; j < steps; j++) {
      const fj = prevF + ((f - prevF) * j) / steps;
      F.push(fj);
      H.push(heading(s0 + fj * length));
    }
    F.push(f);
    H.push(h);
  }
  F[F.length - 1] = 1;
  return { F, H };
}
