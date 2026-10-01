/**
 * Shots along a planned path. A path with "stop and shoot" points is driven as several Pedro paths (Pedro only stops at
 * the end of one), with a volley between them; "shoot on the move" points fire while passing, on a stretch where
 * Pedro's speed is capped. The flywheel is followed along the whole timeline, chasing the speed the next shot needs as
 * fast as its motor allows, and every ball is checked against what that shot can take.
 */
import type { PlannedSegment, Sample } from './chain.ts';
import { type Vec } from './geom.ts';
import type { Profile, SpeedCap } from './model.ts';
import { turretRange, type Chain, type Settings, type ShootSpec } from './project.ts';
import { cachedZone, Flywheel, READY_FRACTION, shotMargin, ShotSolver, zoneFlywheel, zoneMargin, type Shot } from './shot.ts';

/** Where the planned path splits into separate Pedro paths: the first segment of each, from 0. */
export function pieceStarts(chain: Chain, segments: PlannedSegment[]): number[] {
  const starts = [0];
  chain.points.forEach((pt, k) => {
    if (k === 0 || k === chain.points.length - 1 || pt.shoot?.mode !== 'stop') return;
    const i = segments.findIndex((s) => s.leg === k);
    if (i > 0) starts.push(i);
  });
  return starts;
}

/** Arc length along its piece where point k sits (the start of its leg's first segment), given segment lengths. */
function pointPosition(segments: PlannedSegment[], lengths: number[], starts: number[], k: number): { piece: number; s: number } {
  const i = segments.findIndex((s) => s.leg === k);
  let piece = 0;
  while (piece + 1 < starts.length && starts[piece + 1] <= i) piece++;
  let s = 0;
  for (let j = starts[piece]; j < i; j++) s += lengths[j];
  return { piece, s };
}

export interface MoveWindow {
  point: number;
  piece: number;
  /** Arc length of the point along its piece. */
  s: number;
  cap: SpeedCap;
}

/**
 * The capped stretch for each shot on the move: centred on its point, long enough that at the capped speed the robot
 * is inside it for the whole volley.
 */
export function moveWindows(chain: Chain, settings: Settings, segments: PlannedSegment[], lengths: number[], starts: number[]): MoveWindow[] {
  const out: MoveWindow[] = [];
  const pieceLength = (p: number) => {
    let t = 0;
    for (let j = starts[p]; j < (starts[p + 1] ?? segments.length); j++) t += lengths[j];
    return t;
  };
  chain.points.forEach((pt, k) => {
    if (pt.shoot?.mode !== 'move' || k === 0 || k === chain.points.length - 1) return;
    const at = pointPosition(segments, lengths, starts, k);
    const half = (pt.shoot.maxSpeed * settings.shooter.volley) / 2;
    out.push({ point: k, piece: at.piece, s: at.s, cap: { from: Math.max(0, at.s - half), to: Math.min(pieceLength(at.piece), at.s + half), vmax: pt.shoot.maxSpeed } });
  });
  return out;
}

export interface BallShot {
  t: number;
  p: Vec;
  /** Shot margin (degrees of aim / percent of speed to spare, the smaller), NaN if there's no shot. */
  margin: number;
  /** How far the flywheel is off the speed this ball needs, percent. */
  lag: number;
  /** Turret angle from the robot's front, degrees. */
  turret: number;
}

export interface ShotResult {
  point: number;
  mode: ShootSpec['mode'];
  /** When the gate opens (s from the start), and for a stop, how long it waited for the launcher first. */
  open: number;
  /** On the move: which Pedro path, and how far along it, the gate opens (where the routine fires). */
  openAt?: { piece: number; s: number };
  wait: number;
  balls: BallShot[];
  /** How badly it misses what a good shot needs (0 = fine): degrees, percent and inches, for the optimizer. */
  deficit: number;
  problem: string | null;
}

export interface PieceRun {
  samples: Sample[];
  profile: Profile;
}

/** Pose and velocity at time t along a piece (t from the piece's start). */
function stateAt(run: PieceRun, t: number): { p: Vec; h: number; vx: number; vy: number; omega: number } {
  const { samples, profile } = run;
  const n = samples.length;
  let i = 1;
  while (i < n - 1 && profile.time[i] < t) i++;
  const t0 = profile.time[i - 1];
  const t1 = profile.time[i];
  const f = t1 > t0 ? Math.min(1, Math.max(0, (t - t0) / (t1 - t0))) : 1;
  const a = samples[i - 1];
  const b = samples[i];
  const v = profile.v[i - 1] + (profile.v[i] - profile.v[i - 1]) * f;
  const travel = a.travel + (b.travel - a.travel) * f;
  const ds = b.s - a.s;
  const omega = ds > 1e-9 ? ((b.h - a.h) / ds) * v : 0;
  return { p: { x: a.p.x + (b.p.x - a.p.x) * f, y: a.p.y + (b.p.y - a.p.y) * f }, h: a.h + (b.h - a.h) * f, vx: v * Math.cos(travel), vy: v * Math.sin(travel), omega };
}

/** Arc length at time t along a piece. */
function distanceAt(run: PieceRun, t: number): number {
  const { samples, profile } = run;
  let i = 1;
  while (i < samples.length - 1 && profile.time[i] < t) i++;
  const t0 = profile.time[i - 1];
  const t1 = profile.time[i];
  const f = t1 > t0 ? Math.min(1, Math.max(0, (t - t0) / (t1 - t0))) : 1;
  return samples[i - 1].s + (samples[i].s - samples[i - 1].s) * f;
}

/** Time at arc length s along a piece. */
function timeAt(run: PieceRun, s: number): number {
  const { samples, profile } = run;
  let i = 1;
  while (i < samples.length - 1 && samples[i].s < s) i++;
  const a = samples[i - 1];
  const b = samples[i];
  const f = b.s > a.s ? Math.min(1, Math.max(0, (s - a.s) / (b.s - a.s))) : 1;
  return profile.time[i - 1] + (profile.time[i] - profile.time[i - 1]) * f;
}

/**
 * `slack` (degrees and percent) asks for that much more room than needed: the search uses a little, so the optimum,
 * which sits right on a limit, lands inside it.
 *
 * Runs the shots along the pieces: returns each shot, and the time the stops add (waiting for the launcher plus the
 * volley). `windows` are the shots on the move; a piece ends in a stop shot when the next piece starts at one (or, for
 * the last piece, when the end point shoots), and the first point may shoot before setting off.
 */
export function simulateShots(chain: Chain, settings: Settings, runs: PieceRun[], starts: number[], segments: PlannedSegment[], windows: MoveWindow[], slack = 0): { shots: ShotResult[]; extra: number } {
  const cfg = settings.shooter;
  const shooting = chain.points.map((p, k) => ({ k, spec: p.shoot })).filter((x): x is { k: number; spec: ShootSpec } => !!x.spec);
  if (!shooting.length) return { shots: [], extra: 0 };
  const fw = new Flywheel(cfg);
  const solver = new ShotSolver(cfg);
  const turret = turretRange(settings.robot);
  const tpe = cfg.ticksPerExitSpeed;
  const spacing = cfg.volley / Math.max(1, cfg.balls);

  // Which point ends each piece (a stop between pieces, or the chain's end).
  const endPoint = runs.map((_, p) => (p + 1 < starts.length ? segments[starts[p + 1]].leg : chain.points.length - 1));
  let next = 0; // index into `shooting` of the next shot to come
  const cellNow = () => shooting[Math.min(next, shooting.length - 1)].spec.cell;
  const shots: ShotResult[] = [];
  let extra = 0;
  let clock = 0;
  const first = runs[0].samples[0];
  let fly = zoneFlywheel(cachedZone(cfg, cellNow()), first.p); // it's been tracking before the path starts

  /** One ball: how good it is, and the flywheel after it leaves. */
  const ball = (t: number, p: Vec, shot: Shot, target: number, balls: BallShot[]) => {
    const lagTicks = Math.abs(fly - target);
    const margin = shotMargin(shot, cfg, lagTicks);
    balls.push({ t, p, margin, lag: target > 0 ? (lagTicks / target) * 100 : 0, turret: (shot.turretAngle * 180) / Math.PI });
    if (shot.feasible) fly = fw.afterBall(fly, shot.exitSpeed);
  };

  /** Scores a shot's balls; works out the deficit and what to say about it. */
  const judge = (point: number, mode: ShootSpec['mode'], open: number, wait: number, balls: BallShot[], extraDeficit: number, extraProblem: string | null): ShotResult => {
    const spec = chain.points[point].shoot as ShootSpec;
    const zone = cachedZone(cfg, spec.cell);
    let deficit = extraDeficit;
    let problem = extraProblem;
    let worst = 0;
    for (const b of balls) {
      let d = 0;
      let why: string | null = null;
      if (Number.isNaN(b.margin)) {
        d = 5 + Math.max(0, -zoneMargin(zone, b.p));
        why = 'no flywheel speed gets a ball in from there';
      } else if (b.margin < slack) {
        d = slack - b.margin;
        why = b.lag > 0.5 ? `the flywheel is ${b.lag.toFixed(1)}% off the speed the shot needs` : `the shot has too little room for the robot's aim and speed errors`;
      }
      const out = Math.max(0, b.turret - (turret.max - slack), turret.min + slack - b.turret);
      if (out > 0) {
        d += out / 2;
        why ??= `the turret would have to turn to ${Math.round(b.turret)}°`;
      }
      deficit += d;
      if (d > worst) {
        worst = d;
        problem = why;
      }
    }
    return { point, mode, open, wait, balls, deficit, problem };
  };

  /** A volley standing still at pose (p, h), starting now. Returns the shot, and steps the clock. */
  const standingVolley = (point: number, p: Vec, h: number) => {
    const spec = chain.points[point].shoot as ShootSpec;
    solver.reset();
    const shot = solver.solve(p.x, p.y, h, 0, 0, 0, spec.cell);
    const target = shot.flywheel;
    // robot.shoot(): wait until ready (up to 1.5 s), then open the gate for the volley.
    const ready = READY_FRACTION * shot.speedTolerance * tpe;
    let wait = 0;
    while (shot.feasible && Math.abs(fly - target) > ready && wait < 1.5) {
      fly = fw.step(fly, target, 0.01);
      wait += 0.01;
    }
    const open = clock + wait;
    const balls: BallShot[] = [];
    let t = 0;
    for (let b = 0; b < cfg.balls; b++) {
      const at = (b + 0.5) * spacing;
      fly = fw.step(fly, target, at - t);
      t = at;
      ball(open + at, p, shot, target, balls);
    }
    fly = fw.step(fly, target, cfg.volley - t);
    shots.push(judge(point, 'stop', open, wait, balls, 0, null));
    clock = open + cfg.volley;
    extra += wait + cfg.volley;
  };

  if (chain.points[0].shoot?.mode === 'stop') {
    standingVolley(0, first.p, first.h);
    next++;
  }

  runs.forEach((run, pi) => {
    const { samples, profile } = run;
    const n = samples.length;
    // Moments on this piece: shots on the move, with exact targets every 0.1 s around them.
    const here = windows.filter((w) => w.piece === pi).map((w) => {
      const tp = timeAt(run, w.s);
      return { w, open: tp - cfg.volley / 2, balls: Array.from({ length: cfg.balls }, (_, b) => tp - cfg.volley / 2 + (b + 0.5) * spacing) };
    });
    const ticks: { t: number; kind: 'sample' | 'exact' | 'open' | 'ball'; m?: number; b?: number }[] = [];
    for (let i = 1; i < n; i++) ticks.push({ t: profile.time[i], kind: 'sample' });
    here.forEach((h, m) => {
      for (let t = h.open - 0.3; t < h.open + cfg.volley; t += 0.1) ticks.push({ t, kind: 'exact' });
      ticks.push({ t: h.open, kind: 'open', m });
      h.balls.forEach((t, b) => ticks.push({ t, kind: 'ball', m, b }));
    });
    ticks.sort((a, b) => a.t - b.t);
    const exactFrom = here.map((h) => [h.open - 0.3, h.open + cfg.volley]);
    const end = profile.time[n - 1];
    let last = 0;
    let target = fly;
    const pending = here.map(() => ({ balls: [] as BallShot[], extraDeficit: 0, extraProblem: null as string | null }));
    for (const tick of ticks) {
      const t = Math.min(Math.max(tick.t, 0), end);
      const st = stateAt(run, t);
      // Around a shot on the move the target is solved exactly (with the robot's velocity) every 0.1 s and at the
      // gate and every ball, and held in between; elsewhere it's the standing shot's speed from the zone grid.
      const inExact = exactFrom.some(([a, b]) => t >= a - 1e-9 && t <= b + 1e-9);
      let shot: Shot | null = null;
      if (tick.kind !== 'sample') {
        shot = solver.solve(st.p.x, st.p.y, st.h, st.vx, st.vy, st.omega, cellNow());
        target = shot.feasible ? shot.flywheel : zoneFlywheel(cachedZone(cfg, cellNow()), st.p);
      } else if (!inExact) {
        target = zoneFlywheel(cachedZone(cfg, cellNow()), st.p);
      }
      fly = fw.step(fly, target, Math.max(0, t - last));
      last = t;
      if (tick.kind === 'open' && shot && tick.m !== undefined) {
        // The gate opens when the launcher says it's ready; it must be by now, or the volley starts late.
        const allowed = READY_FRACTION * shot.speedTolerance * tpe;
        const late = shot.feasible ? Math.max(0, Math.abs(fly - target) - allowed) : 0;
        if (late > 0) {
          pending[tick.m].extraDeficit += (late / target) * 100;
          pending[tick.m].extraProblem = 'the flywheel isn\'t up to speed when the volley should start';
        }
      }
      if (tick.kind === 'ball' && shot && tick.m !== undefined) {
        ball(clock + t, st.p, shot, target, pending[tick.m].balls);
        const pt = chain.points[here[tick.m].w.point];
        if (pt.kind === 'region') {
          const out = Math.hypot(Math.max(0, Math.abs(st.p.x - pt.x) - pt.halfWidth), Math.max(0, Math.abs(st.p.y - pt.y) - pt.halfHeight));
          if (out > 0) {
            pending[tick.m].extraDeficit += out;
            pending[tick.m].extraProblem ??= 'part of the volley happens outside the area';
          }
        }
        if (tick.b === cfg.balls - 1) next++;
      }
    }
    here.forEach((h, m) => shots.push({ ...judge(h.w.point, 'move', clock + h.open, 0, pending[m].balls, pending[m].extraDeficit, pending[m].extraProblem), openAt: { piece: pi, s: distanceAt(run, Math.max(0, h.open)) } }));
    // Settling at the end of the piece, still tracking.
    const endState = samples[n - 1];
    const settle = profile.total - end;
    if (settle > 0) fly = fw.step(fly, zoneFlywheel(cachedZone(cfg, cellNow()), endState.p), settle);
    clock += profile.total;
    const k = endPoint[pi];
    if (chain.points[k].shoot?.mode === 'stop') {
      standingVolley(k, endState.p, endState.h);
      next++;
    }
  });
  shots.sort((a, b) => a.open - b.open);
  return { shots, extra };
}
