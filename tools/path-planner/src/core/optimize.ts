/**
 * Turns a chain spec into the fastest path the model allows: seeds from the visibility graph, then CMA-ES over
 * joint positions (within regions), joint tangent directions, Bezier handle lengths and heading knots.
 */
import { ArcTable, tableFor, type Cubic } from './bezier.ts';
import { sampleChain, splitSegment, type PlannedSegment, type Sample } from './chain.ts';
import { chainObstacles, checkChain, type Check } from './constraints.ts';
import { cmaes } from './cmaes.ts';
import { CENTERLINE_X, RED_HIVE_TARGET, WALL_MAX, WALL_MIN } from './field.ts';
import { add, angleOf, convexDistance, dist, fromAngle, rad, sub, wrap, type Vec } from './geom.ts';
import { smoothHeading, toPieces } from './heading.ts';
import { drivetrain } from './drivetrain.ts';
import { speedProfile, type Limit, type SpeedCap } from './model.ts';
import { moveWindows, pieceStarts, simulateShots, type PieceRun, type ShotResult } from './shooting.ts';
import { cellTarget, type Cell } from './shot.ts';
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
  /** The robot stops here (to shoot), so the path may leave in a different direction than it came in. */
  stop?: boolean;
  /** The hive cell it shoots at from here, if it does. */
  cell?: Cell;
}

export interface PlannedMarker {
  name: string;
  /** Which of the Pedro paths it's on (0 unless the path stops to shoot). */
  piece: number;
  /** Segment index in that compound path (Pedro's pathIndex). */
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
  check: Pick<Check, 'clearances' | 'rules' | 'worst'> & { drift?: Drift; lag?: Drift };
  /** Every shot along the path, and the first segment of each Pedro path it's driven as. */
  shots?: ShotResult[];
  pieces?: number[];
  /** Share of the path each limit holds the speed down, for the report. */
  limits: Partial<Record<Limit, number>>;
  seeds: number;
  evals: number;
  modelSource: string;
  /** How many differently seeded searches this plan is the best of (Optimize again on an unchanged path adds one). */
  searches?: number;
}

export interface Progress {
  /** Screening every route briefly, or refining the most promising ones. */
  phase: 'screen' | 'refine';
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
  /** Independent CMA-ES runs per route that makes the final round. */
  runsPerRoute?: number;
  /** Evaluations for each route's screening run. */
  screenEvals?: number;
  /** How many routes get full runs after screening. */
  finalRoutes?: number;
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
    const stop = pt.shoot?.mode === 'stop' && i > 0 && i < chain.points.length - 1;
    const cell = pt.shoot?.cell;
    if (pt.kind === 'fixed') joints.push({ pos: { kind: 'fixed', p: { x: pt.x, y: pt.y } }, heading, leg: legAfter, stop, cell });
    else joints.push({ pos: { kind: 'region', c: { x: pt.x, y: pt.y }, hw: pt.halfWidth, hh: pt.halfHeight }, heading, leg: legAfter, stop, cell });
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
  psiIdx: number[]; // direction leaving the joint
  psiInIdx: number[]; // direction arriving (the same one unless the robot stops there)
  headIdx: number[]; // -1 when fixed exactly
  handleIdx: number[]; // per segment, a then b
  knotIdx: number[]; // per segment, first interior knot
  /** Per segment, where its interior heading knots sit, as fractions of its length. */
  knotF: number[][];
  psi0: number[];
  psiIn0: number[];
  /** Unwrapped reference heading for every knot, in chain order: joint 0, interior…, joint 1, … */
  baseH: number[];
}

/** About one heading knot every 14 in, at least two per segment, so turns can happen where they're needed. */
const KNOT_SPACING = 14;

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
  const psiInIdx: number[] = [];
  const headIdx: number[] = [];
  for (const j of joints) {
    posIdx.push(j.pos.kind === 'fixed' ? -1 : n);
    if (j.pos.kind !== 'fixed') n += 2;
    psiIdx.push(n++);
    psiInIdx.push(j.stop ? n++ : psiIdx[psiIdx.length - 1]);
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
  // At a stop, arriving and leaving are seeded separately: straight in from the previous joint, straight out to the next.
  const psiIn0 = psi0.slice();
  joints.forEach((j, i) => {
    if (!j.stop || i === 0 || i === pts.length - 1) return;
    psiIn0[i] = angleOf(sub(pts[i], pts[i - 1]));
    psi0[i] = angleOf(sub(pts[i + 1], pts[i]));
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
    // A shooting point with heading to spare starts out facing so the hive cell sits mid-turret.
    let h0 = j.heading.kind === 'fixed' ? j.heading.h : preferred(j.leg, pts[i], psi0[i]);
    if (j.cell && j.heading.kind === 'fixed' && j.heading.tol > 1e-9) {
      const c = cellTarget(j.cell);
      const want = angleOf(sub(c, pts[i])) - turretMid;
      h0 = j.heading.h + Math.max(-j.heading.tol, Math.min(j.heading.tol, wrap(want - j.heading.h)));
    }
    raw.push(h0);
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
  return { n, posIdx, psiIdx, psiInIdx, headIdx, handleIdx, knotIdx, knotF, psi0, psiIn0, baseH };
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
  const psiIn = joints.map((_, i) => L.psiIn0[i] + ANGLE_SCALE * x[L.psiInIdx[i]]);
  const curves: Cubic[] = [];
  for (let s = 0; s < joints.length - 1; s++) {
    const a = pos[s];
    const b = pos[s + 1];
    const chord = Math.max(dist(a, b), 1e-3);
    const ha = chord * (0.05 + 0.9 * sigmoid(x[L.handleIdx[s]] + HANDLE_U0));
    const hb = chord * (0.05 + 0.9 * sigmoid(x[L.handleIdx[s] + 1] + HANDLE_U0));
    curves.push([a, add(a, fromAngle(psi[s], ha)), sub(b, fromAngle(psiIn[s + 1], hb)), b]);
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
  /** How far wide of the path Pedro would drift where a curve asks more than the wheels have. */
  drift: Drift;
  /** How far Pedro's heading would fall behind where the plan turns faster than the robot can. */
  lag: Drift;
  cost: number;
  length: number;
  limits: Partial<Record<Limit, number>>;
  /** Places where it turns back on itself mid-path, which Pedro can't drive (it brakes only at the end). */
  reversals: number;
  /** Keeps every gap, heading rule and the wheels' limits, never turns back on itself, and every shot scores. */
  fits: boolean;
  shots: ShotResult[];
  /** First segment of each Pedro path it's driven as. */
  pieces: number[];
  /** Capped stretches for shooting on the move: piece, arc length along it, cap. */
  windows: CapWindow[];
  runs: PieceRun[];
}

export interface CapWindow {
  piece: number;
  from: number;
  to: number;
  vmax: number;
  point: number;
  /** Where the shot's point is along the piece. */
  s: number;
}

export interface Drift {
  /** The worst of it: inches off the path (drift) or radians behind (heading lag). */
  worst: number;
  /** Arc length where it happens, in. */
  at: number;
}

/** Drifting this far wide is within what Pedro corrects and the model's own precision, in. */
export const DRIFT_LIMIT = 1;
/** And the heading falling this far behind, rad (5°). */
export const LAG_LIMIT = (5 * Math.PI) / 180;
/** A radian of heading lag counts like this many inches of drift. */
const LAG_WEIGHT = 10;

/**
 * Scores a path the way the robot would drive it: each Pedro path it splits into (at stops to shoot) driven on its own,
 * speed caps where it shoots on the move, every shot along the way. `stride`: check the footprint at every n-th
 * sample (the search uses 2, with a little extra margin).
 */
export function evaluate(segments: PlannedSegment[], project: Settings, chain: Chain, margins: Margins, ds = 0.5, stride = 1, shotSlack = 0): Evaluation {
  const starts = pieceStarts(chain, segments);
  const sampledPieces = starts.map((a, p) => sampleChain(segments.slice(a, starts[p + 1] ?? segments.length), ds));
  const lengths = sampledPieces.flatMap((sp) => sp.tables.map((t) => t.length));
  const windows = moveWindows(chain, project, segments, lengths, starts);
  const runs: PieceRun[] = [];
  const capStarts: number[] = windows.map((w) => w.cap.from);
  let all: Sample[] = sampledPieces[0].samples;
  if (sampledPieces.length > 1) {
    all = [];
    let off = 0;
    for (const sp of sampledPieces) {
      for (const smp of sp.samples) all.push({ ...smp, s: smp.s + off });
      off += sp.length;
    }
  }
  sampledPieces.forEach((sp, p) => {
    // Capped segments (a finished plan) are capped from where they start, as Pedro does; otherwise the caps come from
    // the shots and start where the robot has to begin coasting down.
    const fixed: SpeedCap[] = [];
    let s0 = 0;
    for (let j = starts[p]; j < (starts[p + 1] ?? segments.length); j++) {
      const len = sp.tables[j - starts[p]].length;
      const cap = segments[j].maxSpeed;
      if (cap !== undefined) {
        const prev = fixed[fixed.length - 1];
        if (prev && Math.abs(prev.to - s0) < 1e-6 && prev.vmax === cap) prev.to = s0 + len;
        else fixed.push({ from: s0, to: s0 + len, vmax: cap, fixedStart: true });
      }
      s0 += len;
    }
    const mine = windows.map((w, i) => ({ w, i })).filter((x) => x.w.piece === p);
    const caps = fixed.length ? fixed : mine.map((x) => x.w.cap);
    const profile = speedProfile(sp.samples, project, { endStopped: p === starts.length - 1 ? chain.endStopped : true, caps });
    if (!fixed.length) mine.forEach((x, q) => (capStarts[x.i] = profile.capStarts[q]));
    runs.push({ samples: sp.samples, profile });
  });
  const check = checkChain(all, project, chain, margins, chainObstacles(project, chain, margins), stride);
  // Drifting wide counts like breaking a gap: a path Pedro can't hold isn't the path that was checked for clearance.
  let driftArea = 0;
  let worstDrift = 0;
  let driftAt = 0;
  let worstLag = 0;
  let lagAt = 0;
  let reversals = 0;
  let driveTime = 0;
  let off = 0;
  const limits: Partial<Record<Limit, number>> = {};
  const total = all.length;
  sampledPieces.forEach((sp, p) => {
    const { profile } = runs[p];
    const smp = sp.samples;
    for (let i = 1; i < smp.length; i++) {
      const step = smp[i].s - smp[i - 1].s;
      driftArea += Math.max(0, profile.drift[i] - DRIFT_LIMIT) * step + LAG_WEIGHT * Math.max(0, profile.lag[i] - LAG_LIMIT) * step;
    }
    if (profile.worstDrift > worstDrift) {
      worstDrift = profile.worstDrift;
      driftAt = profile.driftAt + off;
    }
    if (profile.worstLag > worstLag) {
      worstLag = profile.worstLag;
      lagAt = profile.lagAt + off;
    }
    reversals += profile.reversals;
    driveTime += profile.total;
    for (let i = 0; i < smp.length; i++) limits[profile.limit[i]] = (limits[profile.limit[i]] ?? 0) + 1 / total;
    off += sp.length;
  });
  const { shots, extra } = simulateShots(chain, project, runs, starts, segments, windows, shotSlack);
  let shotDeficit = 0;
  let worstShot = 0;
  for (const sh of shots) {
    shotDeficit += sh.deficit;
    worstShot = Math.max(worstShot, sh.deficit);
  }
  const driftExcess = Math.max(0, worstDrift - DRIFT_LIMIT) + LAG_WEIGHT * Math.max(0, worstLag - LAG_LIMIT);
  const broken = check.worst > 1e-3 || driftExcess > 0 || reversals > 0 || worstShot > 1e-6;
  const time = driveTime + extra;
  const cost = time + 2 * check.penalty + 20 * check.worst + 2 * driftArea + 20 * driftExcess + 5 * reversals + 2 * shotDeficit + 20 * worstShot + (broken ? 3 : 0);
  return {
    time,
    check,
    drift: { worst: worstDrift, at: driftAt },
    lag: { worst: worstLag, at: lagAt },
    reversals,
    cost,
    length: off,
    limits,
    fits: !broken,
    shots,
    pieces: starts,
    windows: windows.map((w, i) => ({ piece: w.piece, from: capStarts[i], to: w.cap.to, vmax: w.cap.vmax, point: w.point, s: w.s })),
    runs,
  };
}

/** Which Pedro path and segment arc length `s` along piece `piece` falls in, as Pedro's (segment, parameter). */
function locate(segments: PlannedSegment[], starts: number[], piece: number, s: number): { seg: number; t: number } {
  const end = starts[piece + 1] ?? segments.length;
  let left = s;
  for (let j = starts[piece]; j < end; j++) {
    const table = tableFor(segments[j].curve);
    if (left <= table.length + 1e-9 || j === end - 1) return { seg: j - starts[piece], t: table.parameter(Math.min(1, Math.max(0, left / Math.max(table.length, 1e-9)))) };
    left -= table.length;
  }
  return { seg: 0, t: 0 };
}

/** The marker name a shot on the move fires at, e.g. SHOT_3 for point 3. */
export const shotMarkerName = (point: number) => `SHOT_${point + 1}`;

/**
 * Where each marker lands: which Pedro path, segment index and curve parameter. Shots on the move add a marker where
 * the gate should open.
 */
export function placeMarkers(chain: Chain, segments: PlannedSegment[], ev?: Pick<Evaluation, 'pieces' | 'shots'>): PlannedMarker[] {
  const starts = ev?.pieces ?? pieceStarts(chain, segments);
  const tables = segments.map((s) => new ArcTable(s.curve, 128));
  const pieceOf = (seg: number) => {
    let p = 0;
    while (p + 1 < starts.length && starts[p + 1] <= seg) p++;
    return p;
  };
  const out = chain.markers.map((m) => {
    const legSegs = segments.map((s, i) => ({ s, i })).filter((x) => x.s.leg === m.leg);
    const total = legSegs.reduce((acc, x) => acc + tables[x.i].length, 0);
    let target = m.at * total;
    for (const { i } of legSegs) {
      const len = tables[i].length;
      if (target <= len + 1e-9 || i === legSegs[legSegs.length - 1].i) {
        const piece = pieceOf(i);
        return { name: m.name, piece, seg: i - starts[piece], t: tables[i].parameter(len > 0 ? Math.min(target / len, 1) : 0) };
      }
      target -= len;
    }
    return { name: m.name, piece: 0, seg: 0, t: 0 };
  });
  for (const sh of ev?.shots ?? []) {
    if (!sh.openAt) continue;
    out.push({ name: shotMarkerName(sh.point), piece: sh.openAt.piece, ...locate(segments, starts, sh.openAt.piece, sh.openAt.s) });
  }
  return out;
}

/**
 * Splits the segments where each shot's capped stretch starts and ends, and caps the ones in between, so the Java
 * gives Pedro exactly that stretch. Positions within 3 % of a segment's end snap to it.
 */
function applyCaps(segments: PlannedSegment[], ev: Evaluation): PlannedSegment[] {
  if (!ev.windows.length) return segments;
  const out = segments.map((s) => ({ ...s }));
  const starts = ev.pieces;
  // Global arc length positions to cut at, last first so earlier indices stay put.
  const pieceOffset = (p: number) => {
    let off = 0;
    for (let j = 0; j < starts[p]; j++) off += tableFor(segments[j].curve).length;
    return off;
  };
  const cuts = ev.windows.flatMap((w) => [pieceOffset(w.piece) + w.from, pieceOffset(w.piece) + w.to]).sort((a, b) => b - a);
  for (const cut of cuts) {
    let s0 = 0;
    for (let j = 0; j < out.length; j++) {
      const len = tableFor(out[j].curve).length;
      if (cut < s0 + len - 1e-6) {
        const c = (cut - s0) / len;
        if (c > 0.03 && c < 0.97) out.splice(j, 1, ...splitSegment(out[j], c));
        break;
      }
      s0 += len;
    }
  }
  // Cap every segment whose middle is inside a stretch.
  const spans = ev.windows.map((w) => ({ a: pieceOffset(w.piece) + w.from, b: pieceOffset(w.piece) + w.to, v: w.vmax }));
  let s0 = 0;
  return out.map((seg) => {
    const len = tableFor(seg.curve).length;
    const mid = s0 + len / 2;
    s0 += len;
    const span = spans.find((x) => mid > x.a && mid < x.b);
    const { maxSpeed: _old, ...rest } = seg;
    void _old;
    return span ? { ...rest, maxSpeed: span.v } : rest;
  });
}

/** Everything a chain's result depends on, hashed: the chain, robot, margins and model. */
export function chainSpecHash(settings: Settings, chain: Chain): string {
  const shoots = chain.points.some((p) => p.shoot);
  return fnv1a(JSON.stringify([chain, settings.robot, marginsFor(settings, chain), settings.model, shoots ? settings.shooter : null, PLANNER_VERSION]));
}

/** Bump when the optimizer's output changes for the same input, so saved plans get redone. */
export const PLANNER_VERSION = 5;

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
  const m = drivetrain(project.model, project.robot);
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
  const searchMargins: Margins = { ...margins, obstacle: margins.obstacle + 0.2, centerline: margins.centerline + 0.2 };

  let best: { segments: PlannedSegment[]; ev: Evaluation } | null = null;
  let evals = 0;
  let stop = false;
  let lastReport = 0;
  let bestSoFar: number | null = null;
  // Every route gets a few independent runs (CMA-ES can settle on a local optimum), and a route that still breaks a
  // limit gets more, each with a bigger population (IPOP-style).
  const runs = opts.runsPerRoute ?? 2;
  const extra = opts.restarts ?? 2;
  const fastest = Math.max(m.vForward, m.vStrafe);
  const maxEvals = opts.maxEvalsPerSeed ?? 12000;
  const screenEvals = Math.min(maxEvals, opts.screenEvals ?? 2500);
  // Every route gets a short screening run; the most promising few get full runs (successive halving), so the work
  // goes where the fastest path is likely to be.
  interface Route {
    index: number;
    joints: Joint[];
    L: Layout;
    f: (x: Float64Array) => number;
    x: Float64Array;
    cost: number;
  }
  const turret = turretRange(project.robot);
  const turretMid = rad((turret.min + turret.max) / 2);
  const run = (route: Route, x0: Float64Array, attempt: number, budget: number, lambda: number | undefined, progress: (done: number) => number, phase: Progress['phase']) => {
    const result = cmaes(route.f, x0, {
      sigma: 0.4,
      seed: (opts.seed ?? 1) + route.index * 7919 + attempt * 104729,
      maxEvals: budget,
      lambda,
      tolFun: 2e-3,
      onGeneration: (g) => {
        if (!opts.onProgress) return;
        const now = Date.now();
        if (now - lastReport < 150) return;
        lastReport = now;
        const segs = decode(g.x, route.joints, route.L).segments;
        const ev = evaluate(segs, project, chain, margins, SEARCH_DS);
        const keepGoing = opts.onProgress({
          phase,
          seed: route.index,
          seeds: combos.length,
          attempt,
          runs,
          attempts: runs + extra,
          evals: evals + g.evals,
          bestTime: ev.time,
          feasible: ev.fits,
          bestSoFar,
          fraction: Math.min(1, progress(g.evals / budget)),
          segments: segs,
        });
        if (keepGoing === false) {
          stop = true;
          return false;
        }
      },
    });
    evals += result.evals;
    const segments = decode(result.x, route.joints, route.L).segments;
    const ev = evaluate(segments, project, chain, margins, SEARCH_DS);
    if (!best || ev.cost < best.ev.cost) best = { segments, ev };
    if (ev.fits && (bestSoFar === null || ev.time < bestSoFar)) bestSoFar = ev.time;
    if (ev.cost < route.cost) {
      route.cost = ev.cost;
      route.x = result.x;
    }
    return ev;
  };

  const screened: Route[] = [];
  combos.forEach((combo, index) => {
    if (stop) return;
    // Skip a route that can't beat the best path found, even driven flat out along its corners.
    if (bestSoFar !== null && (0.9 * polyLength(combo)) / fastest >= bestSoFar) return;
    const joints = buildJoints(chain, combo);
    const L = layout(joints, chain, steer, turretMid);
    const route: Route = {
      index,
      joints,
      L,
      f: (x) => evaluate(decode(x, joints, L).segments, project, chain, searchMargins, SEARCH_DS, 2, 1).cost,
      x: new Float64Array(L.n),
      cost: Infinity,
    };
    run(route, new Float64Array(L.n), 0, screenEvals, undefined, (d) => (0.3 * (index + d)) / combos.length, 'screen');
    screened.push(route);
  });

  const finalists = [...screened].sort((a, b) => a.cost - b.cost).slice(0, opts.finalRoutes ?? 2);
  finalists.forEach((route, rank) => {
    const baseLambda = 4 + Math.floor(3 * Math.log(Math.max(route.L.n, 1)));
    let fits = false;
    for (let attempt = 1; attempt <= runs + extra && !stop; attempt++) {
      if (attempt > runs && fits) break;
      // The first full run carries on from the screening result; the others start afresh, with a bigger population
      // for retries.
      const x0 = attempt === 1 ? route.x : new Float64Array(route.L.n);
      const lambda = attempt > runs ? 2 ** (attempt - runs) * baseLambda : undefined;
      const ev = run(route, x0, attempt, maxEvals, lambda, (d) => 0.3 + (0.7 * (rank + (Math.min(attempt, runs) - 1 + d) / runs)) / finalists.length, 'refine');
      if (ev.fits) fits = true;
    }
  });
  const found = best as { segments: PlannedSegment[]; ev: Evaluation } | null;
  if (!found) throw new Error(`${chain.name}: nothing to optimize`);
  // Report on the path exactly as the robot will get it: rounded the way the Java writes it, with its capped stretches
  // split out as their own segments.
  let segments = roundSegments(found.segments);
  let ev = evaluate(segments, project, chain, margins, SEARCH_DS);
  if (ev.windows.length) {
    segments = roundSegments(applyCaps(segments, ev));
    ev = evaluate(segments, project, chain, margins, SEARCH_DS);
  }
  return planOf(project, chain, segments, ev, combos.length, evals);
}

/** The plan for these segments and their evaluation. */
export function planOf(project: Settings, chain: Chain, segments: PlannedSegment[], ev: Evaluation, seeds: number, evals: number): Plan {
  return {
    chain: chain.name,
    specHash: chainSpecHash(project, chain),
    geometryHash: geometryHash(segments),
    feasible: ev.fits,
    time: ev.time,
    length: ev.length,
    segments,
    markers: placeMarkers(chain, segments, ev),
    check: { clearances: ev.check.clearances, rules: ev.check.rules, worst: ev.check.worst, drift: ev.drift, lag: ev.lag },
    limits: ev.limits,
    shots: ev.shots,
    pieces: ev.pieces,
    seeds,
    evals,
    modelSource: project.model.source,
  };
}
