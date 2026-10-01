/**
 * Starting routes for the optimizer: shortest polylines through a visibility graph of the obstacle corners, one per
 * distinct way around the obstacles. The obstacles are grown by half the robot's narrow side plus the margin, so a
 * route exists wherever the robot could fit at its best orientation; the optimizer then checks the real rectangle.
 */
import { CENTERLINE_X, WALL_MAX, WALL_MIN } from './field.ts';
import { convexHull, dist, lerp, offsetConvex, pointInConvex, segmentHitsConvex, type Polygon, type Vec } from './geom.ts';

export interface RouteWorld {
  obstacles: Polygon[];
  /** Area the robot centre may use. */
  x0: number;
  y0: number;
  x1: number;
  y1: number;
}

/** `centerlineMargin` null: the robot may cross into the other half. */
export function routeWorld(obstacles: Polygon[], robotHalfNarrow: number, margin: number, centerlineMargin: number | null): RouteWorld {
  const r = robotHalfNarrow + margin;
  return {
    obstacles: obstacles.map((o) => convexHull(offsetConvex(convexHull(o), r))),
    x0: WALL_MIN + r,
    y0: WALL_MIN + r,
    x1: centerlineMargin === null ? WALL_MAX - r : Math.min(WALL_MAX - r, CENTERLINE_X - centerlineMargin - robotHalfNarrow),
    y1: WALL_MAX - r,
  };
}

function inBox(w: RouteWorld, p: Vec): boolean {
  return p.x >= w.x0 - 1e-9 && p.x <= w.x1 + 1e-9 && p.y >= w.y0 - 1e-9 && p.y <= w.y1 + 1e-9;
}

/** Obstacles containing an endpoint are ignored for edges touching it (start and goal may sit near walls or flowers). */
function visible(w: RouteWorld, a: Vec, b: Vec, ignore: Set<number>): boolean {
  for (let i = 0; i < w.obstacles.length; i++) {
    if (ignore.has(i)) continue;
    if (segmentHitsConvex(a, b, w.obstacles[i])) return false;
  }
  return true;
}

interface Graph {
  nodes: Vec[];
  adj: { to: number; w: number }[][];
}

function buildGraph(w: RouteWorld, start: Vec, goal: Vec): Graph {
  const nodes: Vec[] = [start, goal];
  for (const o of w.obstacles) {
    for (const v of o) {
      if (!inBox(w, v)) continue;
      if (w.obstacles.some((other) => other !== o && pointInConvex(v, other))) continue;
      nodes.push(v);
    }
  }
  const containing = (p: Vec) => new Set(w.obstacles.map((o, i) => (pointInConvex(p, o) ? i : -1)).filter((i) => i >= 0));
  const ignoreStart = containing(start);
  const ignoreGoal = containing(goal);
  const adj: { to: number; w: number }[][] = nodes.map(() => []);
  for (let i = 0; i < nodes.length; i++) {
    for (let j = i + 1; j < nodes.length; j++) {
      const ignore = new Set<number>();
      if (i === 0 || j === 0) for (const k of ignoreStart) ignore.add(k);
      if (i === 1 || j === 1) for (const k of ignoreGoal) ignore.add(k);
      if (visible(w, nodes[i], nodes[j], ignore)) {
        const d = dist(nodes[i], nodes[j]);
        adj[i].push({ to: j, w: d });
        adj[j].push({ to: i, w: d });
      }
    }
  }
  return { nodes, adj };
}

function dijkstra(g: Graph, from: number, to: number, banned: Set<string>, bannedNodes: Set<number>): number[] | null {
  const n = g.nodes.length;
  const d = new Float64Array(n).fill(Infinity);
  const prev = new Int32Array(n).fill(-1);
  const done = new Uint8Array(n);
  d[from] = 0;
  for (;;) {
    let u = -1;
    for (let i = 0; i < n; i++) if (!done[i] && d[i] < Infinity && (u < 0 || d[i] < d[u])) u = i;
    if (u < 0) return null;
    if (u === to) break;
    done[u] = 1;
    for (const e of g.adj[u]) {
      if (bannedNodes.has(e.to) || banned.has(`${u}-${e.to}`)) continue;
      if (d[u] + e.w < d[e.to]) {
        d[e.to] = d[u] + e.w;
        prev[e.to] = u;
      }
    }
  }
  const path: number[] = [];
  for (let v = to; v >= 0; v = prev[v]) path.unshift(v);
  return path;
}

const pathLength = (g: Graph, p: number[]) => p.slice(1).reduce((s, v, i) => s + dist(g.nodes[p[i]], g.nodes[v]), 0);

/** Yen's k shortest loop-free paths. */
function kShortest(g: Graph, k: number): number[][] {
  const first = dijkstra(g, 0, 1, new Set(), new Set());
  if (!first) return [];
  const A: number[][] = [first];
  const B: number[][] = [];
  for (let i = 1; i < k; i++) {
    const last = A[i - 1];
    for (let j = 0; j < last.length - 1; j++) {
      const root = last.slice(0, j + 1);
      const banned = new Set<string>();
      for (const p of A) {
        if (p.length > j && root.every((v, idx) => p[idx] === v)) banned.add(`${p[j]}-${p[j + 1]}`);
      }
      const bannedNodes = new Set(root.slice(0, -1));
      const spur = dijkstra(g, root[root.length - 1], 1, banned, bannedNodes);
      if (!spur) continue;
      const candidate = root.slice(0, -1).concat(spur);
      if (!B.some((p) => p.join() === candidate.join()) && !A.some((p) => p.join() === candidate.join())) B.push(candidate);
    }
    if (!B.length) break;
    B.sort((a, b) => pathLength(g, a) - pathLength(g, b));
    A.push(B.shift()!);
  }
  return A;
}

function resample(points: Vec[], n: number): Vec[] {
  const lengths = [0];
  for (let i = 1; i < points.length; i++) lengths.push(lengths[i - 1] + dist(points[i - 1], points[i]));
  const total = lengths[lengths.length - 1];
  const out: Vec[] = [];
  let j = 0;
  for (let i = 0; i <= n; i++) {
    const s = (total * i) / n;
    while (j < points.length - 2 && lengths[j + 1] < s) j++;
    const span = lengths[j + 1] - lengths[j];
    out.push(lerp(points[j], points[j + 1], span > 0 ? (s - lengths[j]) / span : 0));
  }
  return out;
}

/** Two routes count as the same way around if every point of one is within `tol` of the other (sampled Hausdorff). */
function similar(a: Vec[], b: Vec[], tol: number): boolean {
  const ra = resample(a, 40);
  const rb = resample(b, 40);
  const near = (p: Vec, q: Vec[]) => q.some((x) => dist(p, x) <= tol);
  return ra.every((p) => near(p, rb)) && rb.every((p) => near(p, ra));
}

/**
 * Up to `max` distinct routes from start to goal, shortest first. Each is the list of corner points between them
 * (empty for a straight line). If no route exists the straight line is returned so the optimizer can still report why.
 */
export function seedRoutes(w: RouteWorld, start: Vec, goal: Vec, max = 3): Vec[][] {
  const g = buildGraph(w, start, goal);
  const paths = kShortest(g, 8);
  if (!paths.length) return [[]];
  const routes: Vec[][] = [];
  for (const p of paths) {
    const pts = p.map((i) => g.nodes[i]);
    if (routes.some((r) => similar([start, ...r, goal], pts, 10))) continue;
    routes.push(pts.slice(1, -1));
    if (routes.length >= max) break;
  }
  return routes;
}
