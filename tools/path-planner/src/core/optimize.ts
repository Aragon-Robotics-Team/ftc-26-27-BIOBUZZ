/**
 * Turns a chain spec into the fastest path the model allows: seeds from the visibility graph, then CMA-ES over
 * joint positions (within regions), joint tangent directions, Bezier handle lengths and heading knots.
 */
import { ArcTable, tableFor, type Cubic } from './bezier.ts';
import { sampleChain, type PlannedSegment } from './chain.ts';
import { chainObstacles, checkChain, type Check } from './constraints.ts';
import { cmaes } from './cmaes.ts';
import { CENTERLINE_X, RED_HIVE_TARGET, WALL_MAX, WALL_MIN } from './field.ts';
import { add, angleOf, convexDistance, dist, fromAngle, rad, sub, wrap, type Vec } from './geom.ts';
import { smoothHeading, toPieces } from './heading.ts';
import { speedProfile, type Limit } from './model.ts';
import { marginsFor, turretRange, type Chain, type Margins, type Settings } from './project.ts';
import { localFootprint, placeFootprint } from './robot.ts';
import { routeWorld, seedRoutes } from './routes.ts';
import { fnv1a, geometryHash, roundSegments } from './rows.ts';

type JointPos = { kind: 'fixed'; p: Vec } | { kind: 'region'; c: Vec; hw: number; hh: number } | { kind: 'via'; p0: Vec };
type JointHeading = { kind: 'fixed'; h: number; tol: number } | { kind: 'free' };

interface Joint {
  pos: JointPos;
  heading: JointHeading;
  /** Leg of the segment that leaves this joint. */
  leg: number;
}

export interface PlannedMarker {
  name: string;
  /** Segment index in the compound path (Pedro's pathIndex). */
  seg: number;
  /** Curve parameter within that segment (Pedro's parametricCompletion). */
  t: number;
}

export interface Plan {
  chain: string;
  /** Hash of the chain spec + everything it depends on, to tell when it needs re-planning. */
  specHash: string;
  /** Hash of the planned geometry. Logged with every run, so a log can be matched to the exact path it drove. */
  geometryHash: string;
  feasible: boolean;
  /** Predicted time, s. */
  time: number;
  length: number;
  segments: PlannedSegment[];
  markers: PlannedMarker[];
  check: Pick<Check, 'clearances' | 'rules' | 'worst'>;
  /** Share of the path each limit holds the speed down, for the report. */
  limits: Partial<Record<Limit, number>>;
  seeds: number;
  evals: number;
  modelSource: string;
}

export interface Progress {
  /** Which starting route is being searched (0-based), of how many. */
  seed: number;
  seeds: number;
  /** Which run on this route (0-based). The first `runs` are independent runs; any after are retries for a route that
   *  still breaks a limit, up to `attempts` in all. */
  attempt: number;
  runs: number;
  attempts: number;
  /** Path evaluations so far, over all routes. */
  evals: number;
  /** The current run's best path: its time and whether it keeps every limit. */
  bestTime: number;
  feasible: boolean;
  /** The fastest path that keeps every limit, over all finished runs, or null if none yet. */
  bestSoFar: number | null;
  /** Rough share of the work done, 0–1, for a progress bar (runs often stop early, so it can jump ahead). */
  fraction: number;
  segments: PlannedSegment[];
}

export interface OptimizeOptions {
  seed?: number;
  maxEvalsPerSeed?: number;
  maxSeeds?: number;
  /** Independent CMA-ES runs per route. */
  runsPerRoute?: number;
  /** Extra runs, each with a bigger population, for a route whose runs all break a limit. */
  restarts?: number;
  onProgress?: (p: Progress) => boolean | void;
}

const sigmoid = (u: number) => 1 / (1 + Math.exp(-u));
const HANDLE_U0 = Math.log((1 / 3 - 0.05) / 0.9 / (1 - (1 / 3 - 0.05) / 0.9)); // u = 0 ↔ handle = chord / 3
const VIA_SCALE = 8; // inches per unit
/** Sampling step when scoring paths, inches. The search and the final report use the same one. */
const SEARCH_DS = 0.5;
const ANGLE_SCALE = 0.7; // radians per unit

/** Positions and headings for one route through the chain's points. */
function buildJoints(chain: Chain, vias: Vec[][]): Joint[] {
  const joints: Joint[] = [];
  chain.points.forEach((pt, i) => {
    const legAfter = Math.min(i, chain.legs.length - 1);
    const heading: JointHeading = { kind: 'fixed', h: rad(pt.heading), tol: rad(pt.headingTol) };
    if (pt.kind === 'fixed') joints.push({ pos: { kind: 'fixed', p: { x: pt.x, y: pt.y } }, heading, leg: legAfter });
    else joints.push({ pos: { kind: 'region', c: { x: pt.x, y: pt.y }, hw: pt.halfWidth, hh: pt.halfHeight }, heading, leg: legAfter });
    if (i < chain.points.length - 1) for (const v of vias[i] ?? []) joints.push({ pos: { kind: 'via', p0: v }, heading: { kind: 'free' }, leg: i });
  });
  return joints;
}

function seedPos(j: Joint): Vec {
  return j.pos.kind === 'fixed' ? j.pos.p : j.pos.kind === 'region' ? j.pos.c : j.pos.p0;
}

/** Layout of the variable vector. */
interface Layout {
  n: number;
  posIdx: number[]; // index of x (y follows), or -1
  psiIdx: number[];
  headIdx: number[]; // -1 when fixed exactly
  handleIdx: number[]; // per segment, a then b
  knotIdx: number[]; // per segment, first interior knot
  /** Per segment, where its interior heading knots sit, as fractions of its length. */
  knotF: number[][];
  psi0: number[];
  /** Unwrapped reference heading for every knot, in chain order: joint 0, interior…, joint 1, … */
  baseH: number[];
}

/** About one heading knot every 10 in, at least two per segment, so turns can happen where they're needed. */
const KNOT_SPACING = 10;

/** Rough cost (seconds, plus heavy penalties where there's no room) of moving between two poses in a straight line. */
type Steer = (a: Vec, ha: number, b: Vec, hb: number) => number;

/** The candidate per step with the least total cost of consecutive moves. */
function viterbi(candidates: number[][], cost: (i: number, a: number, b: number) => number): number[] {
  let acc = candidates[0].map(() => 0);
  const back: number[][] = [];
  for (let i = 1; i < candidates.length; i++) {
    const next: number[] = [];
    const from: number[] = [];
    for (const b of candidates[i]) {
      let best = Infinity;
      let arg = 0;
      candidates[i - 1].forEach((a, k) => {
        const c = acc[k] + cost(i - 1, a, b);
        if (c < best) {
          best = c;
          arg = k;
        }
      });
      next.push(best);
      from.push(arg);
    }
    acc = next;
    back.push(from);
  }
  let k = acc.indexOf(Math.min(...acc));
  const out = [candidates[candidates.length - 1][k]];
  for (let i = back.length - 1; i >= 0; i--) {
    k = back[i][k];
    out.unshift(candidates[i][k]);
  }
  return out;
}

function layout(joints: Joint[], chain: Chain, steer: Steer, turretMid: number): Layout {
  let n = 0;
  const posIdx: number[] = [];
  const psiIdx: number[] = [];
  const headIdx: number[] = [];
  for (const j of joints) {
    posIdx.push(j.pos.kind === 'fixed' ? -1 : n);
    if (j.pos.kind !== 'fixed') n += 2;
    psiIdx.push(n++);
    if (j.heading.kind === 'free' || j.heading.tol > 1e-9) headIdx.push(n++);
    else headIdx.push(-1);
  }
  const pts = joints.map(seedPos);
  const handleIdx: number[] = [];
  const knotIdx: number[] = [];
  const knotF: number[][] = [];
  for (let s = 0; s < joints.length - 1; s++) {
    handleIdx.push(n);
    n += 2;
    const m = Math.max(2, Math.round(dist(pts[s], pts[s + 1]) / KNOT_SPACING));
    knotF.push(Array.from({ length: m }, (_, q) => (q + 1) / (m + 1)));
    knotIdx.push(n);
    n += m;
  }
  // Seed tangent directions: along the polyline through the joints.
  const psi0 = pts.map((_, i) => {
    const a = pts[Math.max(0, i - 1)];
    const b = pts[Math.min(pts.length - 1, i + 1)];
    const d = sub(b, a);
    return Math.hypot(d.x, d.y) > 1e-9 ? angleOf(d) : 0;
  });
  // Reference headings: fixed where given; front-first legs face along the travel; turret legs face the hive;
  // anything else in between is interpolated.
  const ruleOf = (leg: number) => chain.legs[leg]?.rules ?? [];
  const preferred = (leg: number, at: Vec, travel: number): number | null => {
    const rules = ruleOf(leg);
    if (rules.some((r) => r.type === 'front-first')) return travel;
    const fixed = rules.find((r) => r.type === 'fixed');
    if (fixed && fixed.type === 'fixed') return rad(fixed.heading);
    // Face so the hive sits in the middle of the turret's range.
    if (rules.some((r) => r.type === 'turret-reach')) return angleOf(sub(RED_HIVE_TARGET, at)) - turretMid;
    return null;
  };
  const raw: (number | null)[] = [];
  const where: Vec[] = [];
  const travelAt: number[] = [];
  for (let i = 0; i < joints.length; i++) {
    const j = joints[i];
    raw.push(j.heading.kind === 'fixed' ? j.heading.h : preferred(j.leg, pts[i], psi0[i]));
    where.push(pts[i]);
    travelAt.push(psi0[i]);
    if (i < joints.length - 1) {
      const travel = angleOf(sub(pts[i + 1], pts[i]));
      for (const f of knotF[i]) {
        const at = { x: pts[i].x + (pts[i + 1].x - pts[i].x) * f, y: pts[i].y + (pts[i + 1].y - pts[i].y) * f };
        raw.push(preferred(j.leg, at, travel));
        where.push(at);
        travelAt.push(travel);
      }
    }
  }
  // Unwrap the known values, then fill the gaps linearly.
  const baseH: number[] = new Array(raw.length);
  let last: number | null = null;
  for (let i = 0; i < raw.length; i++) {
    const r = raw[i];
    if (r === null) continue;
    baseH[i] = last === null ? r : last + wrap(r - last);
    last = baseH[i];
  }
  let prevKnown = -1;
  for (let i = 0; i <= raw.length; i++) {
    if (i < raw.length && raw[i] === null) continue;
    for (let k = prevKnown + 1; k < i; k++) {
      const a = prevKnown >= 0 ? baseH[prevKnown] : i < raw.length ? baseH[i] : 0;
      const b = i < raw.length ? baseH[i] : a;
      const f = prevKnown >= 0 && i < raw.length ? (k - prevKnown) / (i - prevKnown) : 0;
      baseH[k] = a + (b - a) * f;
    }
    prevKnown = i;
  }
  // Starting headings for the free knots, chosen together (Viterbi over a few candidates each): the sequence with the
  // least estimated time, where any pose without room costs heavily. This is what finds "strafe through the gap".
  const candidates = raw.map((r, i) =>
    r !== null ? [baseH[i]] : [0, 1, 2, 3, 4, 5, 6, 7].map((q) => baseH[i] + (q * Math.PI) / 4).concat([1, 3].map((q) => travelAt[i] + (q * Math.PI) / 2)),
  );
  const pick = viterbi(candidates, (i, a, b) => steer(where[i], a, where[i + 1], b));
  for (let i = 0; i < raw.length; i++) if (raw[i] === null) baseH[i] = pick[i];
  for (let i = 1; i < baseH.length; i++) baseH[i] = baseH[i - 1] + wrap(baseH[i] - baseH[i - 1]);
  return { n, posIdx, psiIdx, headIdx, handleIdx, knotIdx, knotF, psi0, baseH };
}

interface Decoded {
  segments: PlannedSegment[];
}

function decode(x: Float64Array, joints: Joint[], L: Layout): Decoded {
  const pos = joints.map((j, i) => {
    const k = L.posIdx[i];
    if (j.pos.kind === 'fixed') return j.pos.p;
    if (j.pos.kind === 'region') return { x: j.pos.c.x + j.pos.hw * Math.tanh(x[k]), y: j.pos.c.y + j.pos.hh * Math.tanh(x[k + 1]) };
    return { x: j.pos.p0.x + VIA_SCALE * x[k], y: j.pos.p0.y + VIA_SCALE * x[k + 1] };
  });
  const psi = joints.map((_, i) => L.psi0[i] + ANGLE_SCALE * x[L.psiIdx[i]]);
  const curves: Cubic[] = [];
  for (let s = 0; s < joints.length - 1; s++) {
    const a = pos[s];
    const b = pos[s + 1];
    const chord = Math.max(dist(a, b), 1e-3);
    const ha = chord * (0.05 + 0.9 * sigmoid(x[L.handleIdx[s]] + HANDLE_U0));
    const hb = chord * (0.05 + 0.9 * sigmoid(x[L.handleIdx[s] + 1] + HANDLE_U0));
    curves.push([a, add(a, fromAngle(psi[s], ha)), sub(b, fromAngle(psi[s + 1], hb)), b]);
  }
  // Heading knots along the chain's arc length.
  const tables = curves.map(tableFor);
  const knotS: number[] = [];
  const knotH: number[] = [];
  let s0 = 0;
  let ki = 0;
  for (let i = 0; i < joints.length; i++) {
    const j = joints[i];
    const base = L.baseH[ki];
    let h: number;
    if (j.heading.kind === 'fixed') h = j.heading.tol > 1e-9 ? base + j.heading.tol * Math.tanh(x[L.headIdx[i]]) : base;
    else h = base + ANGLE_SCALE * x[L.headIdx[i]];
    knotS.push(s0);
    knotH.push(h);
    ki++;
    if (i < joints.length - 1) {
      const len = tables[i].length;
      L.knotF[i].forEach((f, q) => {
        knotS.push(s0 + f * len);
        knotH.push(L.baseH[ki] + ANGLE_SCALE * x[L.knotIdx[i] + q]);
        ki++;
      });
      s0 += len;
    }
  }
  const heading = smoothHeading(knotS, knotH);
  let start = 0;
  const segments = curves.map((curve, i) => {
    const len = tables[i].length;
    const seg: PlannedSegment = { curve, heading: toPieces(heading, start, len), leg: joints[i].leg };
    start += len;
    return seg;
  });
  return { segments };
}

export interface Evaluation {
  time: number;
  check: Check;
  cost: number;
  length: number;
  limits: Partial<Record<Limit, number>>;
}

export function evaluate(segments: PlannedSegment[], project: Settings, chain: Chain, margins: Margins, ds = 0.5): Evaluation {
  const sampled = sampleChain(segments, ds);
  const profile = speedProfile(sampled.samples, project.model, { endStopped: chain.endStopped });
  const check = checkChain(sampled.samples, project, chain, margins, chainObstacles(project, chain, margins));
  const cost = profile.total + 2 * check.penalty + 20 * check.worst + (check.worst > 1e-3 ? 3 : 0);
  const limits: Partial<Record<Limit, number>> = {};
  const n = sampled.samples.length;
  for (let i = 0; i < n; i++) limits[profile.limit[i]] = (limits[profile.limit[i]] ?? 0) + 1 / n;
  return { time: profile.total, check, cost, length: sampled.length, limits };
}

/** Where each marker lands: segment index and curve parameter. */
export function placeMarkers(chain: Chain, segments: PlannedSegment[]): PlannedMarker[] {
  const tables = segments.map((s) => new ArcTable(s.curve, 128));
  return chain.markers.map((m) => {
    const legSegs = segments.map((s, i) => ({ s, i })).filter((x) => x.s.leg === m.leg);
    const total = legSegs.reduce((acc, x) => acc + tables[x.i].length, 0);
    let target = m.at * total;
    for (const { i } of legSegs) {
      const len = tables[i].length;
      if (target <= len + 1e-9 || i === legSegs[legSegs.length - 1].i) {
        return { name: m.name, seg: i, t: tables[i].parameter(len > 0 ? Math.min(target / len, 1) : 0) };
      }
      target -= len;
    }
    return { name: m.name, seg: 0, t: 0 };
  });
}

/** Everything a chain's result depends on, hashed: the chain, robot, margins and model. */
export function chainSpecHash(settings: Settings, chain: Chain): string {
  return fnv1a(JSON.stringify([chain, settings.robot, marginsFor(settings, chain), settings.model, PLANNER_VERSION]));
}

/** Bump when the optimizer's output changes for the same input, so saved plans get redone. */
export const PLANNER_VERSION = 2;

export function optimizeChain(project: Settings, chain: Chain, opts: OptimizeOptions = {}): Plan {
  const margins = marginsFor(project, chain);
  const obstacles = chainObstacles(project, chain, margins);
  const halfNarrow = Math.min(project.robot.width, project.robot.length) / 2;
  const world = routeWorld(obstacles.map((o) => o.polygon), halfNarrow, margins.obstacle, chain.allowCrossing ? null : margins.centerline);
  const centre = (i: number): Vec => ({ x: chain.points[i].x, y: chain.points[i].y });
  // Routes per leg, then combinations of them, shortest first.
  const perLeg = chain.legs.map((_, i) => seedRoutes(world, centre(i), centre(i + 1), 2));
  let combos: Vec[][][] = [[]];
  for (const routes of perLeg) combos = combos.flatMap((c) => routes.map((r) => [...c, r]));
  const polyLength = (combo: Vec[][]) => {
    let total = 0;
    combo.forEach((vias, i) => {
      const pts = [centre(i), ...vias, centre(i + 1)];
      for (let k = 1; k < pts.length; k++) total += dist(pts[k - 1], pts[k]);
    });
    return total;
  };
  combos.sort((a, b) => polyLength(a) - polyLength(b));
  combos = combos.slice(0, opts.maxSeeds ?? 4);

  // Room at a pose (smallest gap minus its margin), and the rough cost of a straight move, for starting headings.
  const parts = localFootprint(project.robot);
  const room = (p: Vec, h: number) => {
    const placed = placeFootprint(parts, { x: p.x, y: p.y, h });
    let r = Infinity;
    for (const poly of placed) {
      for (const v of poly) {
        r = Math.min(r, v.x - WALL_MIN - margins.obstacle, v.y - WALL_MIN - margins.obstacle);
        r = Math.min(r, WALL_MAX - v.x - margins.obstacle, WALL_MAX - v.y - margins.obstacle);
        if (!chain.allowCrossing) r = Math.min(r, CENTERLINE_X - v.x - margins.centerline);
      }
      for (const o of obstacles) r = Math.min(r, convexDistance(poly, o.polygon) - margins.obstacle);
    }
    return r;
  };
  const m = project.model;
  const steer: Steer = (a, ha, b, hb) => {
    const d = dist(a, b);
    const travel = angleOf(sub(b, a));
    const turn = wrap(hb - ha);
    let cost = Math.abs(turn) / m.omegaMax;
    let squeeze = 0;
    for (let q = 0; q <= 4; q++) {
      const f = q / 4;
      const h = ha + turn * f;
      const alpha = travel - h;
      if (q < 4) cost += (d / 4) * (Math.abs(Math.cos(alpha)) / m.vForward + Math.abs(Math.sin(alpha)) / m.vStrafe);
      squeeze += Math.max(0, -room({ x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f }, h));
    }
    return cost + 100 * squeeze;
  };

  // The optimum sits right on a limit, and penalties let it lean a hair past. Searching with slightly wider margins
  // lands it just inside the real ones.
  const searchMargins: Margins = { ...margins, obstacle: margins.obstacle + 0.1, centerline: margins.centerline + 0.1 };

  let best: { segments: PlannedSegment[]; ev: Evaluation } | null = null;
  let evals = 0;
  let stop = false;
  let lastReport = 0;
  let bestSoFar: number | null = null;
  // Every route gets a few independent runs (CMA-ES can settle on a local optimum), and a route that still breaks a
  // limit gets more, each with a bigger population (IPOP-style).
  const runs = opts.runsPerRoute ?? 2;
  const extra = opts.restarts ?? 2;
  const fastest = Math.max(project.model.vForward, project.model.vStrafe);
  const maxEvals = opts.maxEvalsPerSeed ?? 12000;
  combos.forEach((combo, seedIdx) => {
    if (stop) return;
    // Skip a route that can't beat the best path found, even driven flat out along its corners.
    if (bestSoFar !== null && (0.9 * polyLength(combo)) / fastest >= bestSoFar) return;
    const joints = buildJoints(chain, combo);
    const turret = turretRange(project.robot);
    const L = layout(joints, chain, steer, rad((turret.min + turret.max) / 2));
    const f = (x: Float64Array) => evaluate(decode(x, joints, L).segments, project, chain, searchMargins, SEARCH_DS).cost;
    const baseLambda = 4 + Math.floor(3 * Math.log(Math.max(L.n, 1)));
    let routeFits = false;
    for (let attempt = 0; attempt < runs + extra && !stop; attempt++) {
      if (attempt >= runs && routeFits) break;
      const result = cmaes(f, new Float64Array(L.n), {
        sigma: 0.4,
        seed: (opts.seed ?? 1) + seedIdx * 7919 + attempt * 104729,
        maxEvals,
        lambda: attempt >= runs ? 2 ** (attempt - runs + 1) * baseLambda : undefined,
        onGeneration: (g) => {
          if (!opts.onProgress) return;
          const now = Date.now();
          if (now - lastReport < 150) return;
          lastReport = now;
          const segs = decode(g.x, joints, L).segments;
          const ev = evaluate(segs, project, chain, margins, SEARCH_DS);
          const keepGoing = opts.onProgress({
            seed: seedIdx,
            seeds: combos.length,
            attempt,
            runs,
            attempts: runs + extra,
            evals: evals + g.evals,
            bestTime: ev.time,
            feasible: ev.check.worst <= 1e-3,
            bestSoFar,
            fraction: Math.min(1, (seedIdx + (Math.min(attempt, runs - 1) + Math.min(1, g.evals / maxEvals)) / runs) / combos.length),
            segments: segs,
          });
          if (keepGoing === false) {
            stop = true;
            return false;
          }
        },
      });
      evals += result.evals;
      const segments = decode(result.x, joints, L).segments;
      const ev = evaluate(segments, project, chain, margins, SEARCH_DS);
      if (!best || ev.cost < best.ev.cost) best = { segments, ev };
      if (ev.check.worst <= 1e-3) {
        routeFits = true;
        if (bestSoFar === null || ev.time < bestSoFar) bestSoFar = ev.time;
      }
    }
  });
  const found = best as { segments: PlannedSegment[]; ev: Evaluation } | null;
  if (!found) throw new Error(`${chain.name}: nothing to optimize`);
  // Report on the path exactly as the robot will get it: rounded the way the Java writes it.
  const segments = roundSegments(found.segments);
  const chosen = { segments, ev: evaluate(segments, project, chain, margins, SEARCH_DS) };
  return {
    chain: chain.name,
    specHash: chainSpecHash(project, chain),
    geometryHash: geometryHash(chosen.segments),
    feasible: chosen.ev.check.worst <= 1e-3,
    time: chosen.ev.time,
    length: chosen.ev.length,
    segments: chosen.segments,
    markers: placeMarkers(chain, chosen.segments),
    check: { clearances: chosen.ev.check.clearances, rules: chosen.ev.check.rules, worst: chosen.ev.check.worst },
    limits: chosen.ev.limits,
    seeds: combos.length,
    evals,
    modelSource: project.model.source,
  };
}
