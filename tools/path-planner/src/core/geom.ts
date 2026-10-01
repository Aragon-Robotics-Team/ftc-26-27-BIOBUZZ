/** 2D geometry in the Pedro field frame: inches, radians, counter-clockwise. */

export interface Vec {
  x: number;
  y: number;
}

/** A pose. Headings are radians inside the planner; project.json stores degrees. */
export interface Pose extends Vec {
  h: number;
}

/** Convex polygon, vertices counter-clockwise. */
export type Polygon = Vec[];

export const vec = (x: number, y: number): Vec => ({ x, y });
export const add = (a: Vec, b: Vec): Vec => ({ x: a.x + b.x, y: a.y + b.y });
export const sub = (a: Vec, b: Vec): Vec => ({ x: a.x - b.x, y: a.y - b.y });
export const scale = (a: Vec, k: number): Vec => ({ x: a.x * k, y: a.y * k });
export const dot = (a: Vec, b: Vec): number => a.x * b.x + a.y * b.y;
export const cross = (a: Vec, b: Vec): number => a.x * b.y - a.y * b.x;
export const len = (a: Vec): number => Math.sqrt(a.x * a.x + a.y * a.y);
export const dist = (a: Vec, b: Vec): number => {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  return Math.sqrt(dx * dx + dy * dy);
};
export const lerp = (a: Vec, b: Vec, t: number): Vec => ({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t });
export const fromAngle = (a: number, r = 1): Vec => ({ x: Math.cos(a) * r, y: Math.sin(a) * r });
export const angleOf = (a: Vec): number => Math.atan2(a.y, a.x);
export const rotate = (a: Vec, ang: number): Vec => {
  const c = Math.cos(ang);
  const s = Math.sin(ang);
  return { x: a.x * c - a.y * s, y: a.x * s + a.y * c };
};

export const deg = (rad: number): number => (rad * 180) / Math.PI;
export const rad = (degrees: number): number => (degrees * Math.PI) / 180;

/** Angle wrapped to (-π, π]. */
export function wrap(a: number): number {
  let r = a % (2 * Math.PI);
  if (r <= -Math.PI) r += 2 * Math.PI;
  if (r > Math.PI) r -= 2 * Math.PI;
  return r;
}

/** Axis-aligned rectangle as a CCW polygon. */
export function rect(x0: number, y0: number, x1: number, y1: number): Polygon {
  const [ax, bx] = x0 < x1 ? [x0, x1] : [x1, x0];
  const [ay, by] = y0 < y1 ? [y0, y1] : [y1, y0];
  return [vec(ax, ay), vec(bx, ay), vec(bx, by), vec(ax, by)];
}

/** Convex hull (Andrew's monotone chain), CCW, no collinear points. */
export function convexHull(points: Vec[]): Polygon {
  const pts = [...points].sort((a, b) => a.x - b.x || a.y - b.y);
  if (pts.length <= 2) return pts;
  const turn = (o: Vec, a: Vec, b: Vec) => cross(sub(a, o), sub(b, o));
  const lower: Vec[] = [];
  for (const p of pts) {
    while (lower.length >= 2 && turn(lower[lower.length - 2], lower[lower.length - 1], p) <= 1e-12) lower.pop();
    lower.push(p);
  }
  const upper: Vec[] = [];
  for (let i = pts.length - 1; i >= 0; i--) {
    const p = pts[i];
    while (upper.length >= 2 && turn(upper[upper.length - 2], upper[upper.length - 1], p) <= 1e-12) upper.pop();
    upper.push(p);
  }
  upper.pop();
  lower.pop();
  return lower.concat(upper);
}

export function translate(poly: Polygon, by: Vec): Polygon {
  return poly.map((p) => add(p, by));
}

export function centroid(poly: Polygon): Vec {
  let x = 0;
  let y = 0;
  for (const p of poly) {
    x += p.x;
    y += p.y;
  }
  return vec(x / poly.length, y / poly.length);
}

export function bounds(poly: Polygon): { x0: number; y0: number; x1: number; y1: number } {
  let x0 = Infinity;
  let y0 = Infinity;
  let x1 = -Infinity;
  let y1 = -Infinity;
  for (const p of poly) {
    x0 = Math.min(x0, p.x);
    y0 = Math.min(y0, p.y);
    x1 = Math.max(x1, p.x);
    y1 = Math.max(y1, p.y);
  }
  return { x0, y0, x1, y1 };
}

/** Distance from point p to segment ab. */
export function pointSegmentDistance(p: Vec, a: Vec, b: Vec): number {
  const ab = sub(b, a);
  const l2 = dot(ab, ab);
  const t = l2 === 0 ? 0 : Math.max(0, Math.min(1, dot(sub(p, a), ab) / l2));
  return dist(p, lerp(a, b, t));
}

export function pointInConvex(p: Vec, poly: Polygon): boolean {
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i];
    const b = poly[(i + 1) % poly.length];
    if (cross(sub(b, a), sub(p, a)) < 0) return false;
  }
  return true;
}

/**
 * Signed distance between two convex polygons: the gap when apart, minus the smallest overlap (along a separating-axis
 * candidate) when they intersect.
 */
export function convexDistance(a: Polygon, b: Polygon): number {
  // Smallest overlap over every edge normal of both polygons (SAT). A positive "overlap" on every axis means they
  // intersect; a negative one on some axis means they are apart, but that axis gap understates the true gap at corners.
  let minOverlap = Infinity;
  let separated = false;
  for (const poly of [a, b]) {
    for (let i = 0; i < poly.length; i++) {
      const p = poly[i];
      const q = poly[(i + 1) % poly.length];
      const e = sub(q, p);
      const l = len(e);
      if (l < 1e-12) continue;
      const n = vec(e.y / l, -e.x / l);
      let aMin = Infinity;
      let aMax = -Infinity;
      let bMin = Infinity;
      let bMax = -Infinity;
      for (const v of a) {
        const d = dot(v, n);
        if (d < aMin) aMin = d;
        if (d > aMax) aMax = d;
      }
      for (const v of b) {
        const d = dot(v, n);
        if (d < bMin) bMin = d;
        if (d > bMax) bMax = d;
      }
      const overlap = Math.min(aMax, bMax) - Math.max(aMin, bMin);
      if (overlap < 0) separated = true;
      if (overlap < minOverlap) minOverlap = overlap;
    }
  }
  if (!separated) return -minOverlap;
  // Apart: the true gap is the smallest vertex-to-edge distance.
  let best = Infinity;
  for (const [p, q] of [
    [a, b],
    [b, a],
  ]) {
    for (const v of p) {
      for (let i = 0; i < q.length; i++) {
        const d = pointSegmentDistance(v, q[i], q[(i + 1) % q.length]);
        if (d < best) best = d;
      }
    }
  }
  return best;
}

/** True if segment ab crosses the interior of convex polygon poly. */
export function segmentHitsConvex(a: Vec, b: Vec, poly: Polygon): boolean {
  // Cyrus–Beck clip against the polygon's half-planes.
  let t0 = 0;
  let t1 = 1;
  const d = sub(b, a);
  for (let i = 0; i < poly.length; i++) {
    const p = poly[i];
    const q = poly[(i + 1) % poly.length];
    const e = sub(q, p);
    const n = vec(e.y, -e.x); // outward normal for CCW polygons
    const num = dot(n, sub(a, p));
    const den = dot(n, d);
    if (Math.abs(den) < 1e-12) {
      if (num >= -1e-9) return false; // parallel and outside or on the edge
      continue;
    }
    const t = -num / den;
    if (den < 0) t0 = Math.max(t0, t);
    else t1 = Math.min(t1, t);
    if (t0 >= t1 - 1e-9) return false;
  }
  return true;
}

/** Push every vertex of a convex polygon outward so each edge moves out by r (sharp corners). */
export function offsetConvex(poly: Polygon, r: number): Polygon {
  const n = poly.length;
  const out: Vec[] = [];
  for (let i = 0; i < n; i++) {
    const prev = poly[(i - 1 + n) % n];
    const cur = poly[i];
    const next = poly[(i + 1) % n];
    const e1 = sub(cur, prev);
    const e2 = sub(next, cur);
    const n1 = scale(vec(e1.y, -e1.x), 1 / len(e1));
    const n2 = scale(vec(e2.y, -e2.x), 1 / len(e2));
    const bis = add(n1, n2);
    const k = r / Math.max(0.2, 1 + dot(n1, n2)); // |bis|² / 2 = 1 + cos; offset = r·bis / (1 + cos)
    out.push(add(cur, scale(bis, k)));
  }
  return out;
}

/** Deterministic PRNG (mulberry32) so optimizer runs repeat exactly. */
export function rng(seed: number): () => number {
  let s = seed >>> 0;
  return () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export function gaussian(random: () => number): () => number {
  let spare: number | null = null;
  return () => {
    if (spare !== null) {
      const s = spare;
      spare = null;
      return s;
    }
    let u = 0;
    let v = 0;
    while (u === 0) u = random();
    v = random();
    const m = Math.sqrt(-2 * Math.log(u));
    spare = m * Math.sin(2 * Math.PI * v);
    return m * Math.cos(2 * Math.PI * v);
  };
}
