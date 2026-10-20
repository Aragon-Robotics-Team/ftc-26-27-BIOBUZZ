/** Checks shared by the GUI and tests. */
import type { Sample } from './chain.ts';
import { chainObstacles, checkChain, checkStartPose, ruleExcess, type Clearance, type StartProblem } from './constraints.ts';
import { RED_HIVE_TARGET } from './field.ts';
import { angleOf, rad, sub, type Vec } from './geom.ts';
import { marginsFor, turretRange, type Chain, type ChainPoint, type Settings } from './project.ts';
import { shotMargin, ShotSolver } from './shot.ts';

/** G304 problems with a path's first point, if it is a match start. */
export function startProblems(settings: Settings, chain: Chain): StartProblem[] {
  if (!chain.matchStart) return [];
  const p = chain.points[0];
  return checkStartPose(settings, chain.name, { x: p.x, y: p.y, h: rad(p.heading) });
}

export interface PointIssue {
  /** Index of the point the problem is at. */
  point: number;
  message: string;
}

/** How a point is called in the panel and messages. */
export function pointLabel(chain: Chain, i: number): string {
  if (i === 0) return 'Start';
  if (i === chain.points.length - 1) return 'End';
  return `${chain.points[i].kind === 'region' ? 'Area' : 'Point'} ${i + 1}`;
}

/** Headings a point allows, sampled: its heading ± its tolerance (every heading if the tolerance is 180°). */
function headingsOf(p: ChainPoint, step: number): number[] {
  const tol = Math.min(p.headingTol, 180);
  if (tol <= 0) return [rad(p.heading)];
  const n = Math.max(1, Math.ceil(tol / step));
  const out: number[] = [];
  for (let k = -n; k <= n; k++) out.push(rad(p.heading + (tol * k) / n));
  return out;
}

/** Positions a point allows, sampled: the point itself, or a grid over an area. */
function positionsOf(p: ChainPoint): Vec[] {
  if (p.kind === 'fixed') return [{ x: p.x, y: p.y }];
  const out: Vec[] = [];
  const n = 4;
  for (let i = -n; i <= n; i++) for (let j = -n; j <= n; j++) out.push({ x: p.x + (p.halfWidth * i) / n, y: p.y + (p.halfHeight * j) / n });
  return out;
}

const sampleAt = (p: Vec, h: number): Sample => ({ seg: 0, leg: -1, t: 0, s: 0, p, travel: h, kappa: 0, h });

function describe(c: Clearance): string {
  const what = c.name === 'Walls' ? 'a wall' : c.name === 'Centerline' ? 'the centerline' : `the ${c.name.charAt(0).toLowerCase()}${c.name.slice(1)}`;
  return c.gap < 0 ? `hits ${what}` : `is ${c.gap.toFixed(1)} in from ${what} (needs ${c.margin})`;
}

const deg = (v: number) => Math.round(v * 10) / 10;

/** A point's allowed heading, as people set it. */
function facing(p: ChainPoint): string {
  return p.headingTol > 0 ? `${deg(p.heading)}° ±${deg(p.headingTol)}°` : `${deg(p.heading)}°`;
}

/**
 * Problems the optimizer can't fix because a point forces them: wherever and however the point lets the robot sit,
 * it is too close to something, or the point's heading clashes with a leg's facing rule. Empty when every point
 * leaves at least one way to keep the rules.
 */
export function precheck(settings: Settings, chain: Chain): PointIssue[] {
  const issues: PointIssue[] = [];
  const margins = marginsFor(settings, chain);
  const obstacles = chainObstacles(settings, chain, margins);
  const noRules: Chain = { ...chain, legs: [] };
  chain.points.forEach((p, i) => {
    const label = pointLabel(chain, i);
    const headings = headingsOf(p, p.kind === 'region' ? 10 : 2.5);
    const positions = positionsOf(p);
    // Clearance: is there any pose here that keeps every gap? If not, report the pose that comes closest.
    let best: { worst: number; bad: Clearance[] } | null = null;
    search: for (const pos of positions) {
      for (const h of headings) {
        const check = checkChain([sampleAt(pos, h)], settings, noRules, margins, obstacles);
        if (check.worst <= 1e-3) {
          best = null;
          break search;
        }
        if (!best || check.worst < best.worst) best = { worst: check.worst, bad: check.clearances.filter((c) => c.gap < c.margin - 1e-3) };
      }
    }
    if (best) {
      const where = p.kind === 'region' ? ' anywhere in it' : headings.length > 1 ? ' at any allowed heading' : '';
      issues.push({ point: i, message: `${label}${where} ${best.bad.slice(0, 2).map(describe).join(' and ')}.` });
    }
    // Heading rules of the legs on either side of the point, at the point itself.
    for (const legIndex of [i - 1, i]) {
      const rule = chain.legs[legIndex]?.rules[0];
      if (!rule || rule.type === 'free' || rule.type === 'front-first') continue; // the way it drives in isn't fixed
      const turret = turretRange(settings.robot);
      const ok = positions.some((pos) => headings.some((h) => ruleExcess(rule, sampleAt(pos, h), turret) <= 1e-6));
      if (ok) continue;
      // A leg's facing rule also holds at both its ends, so it has to agree with the heading set at each end.
      const leg = `leg ${legIndex + 1}→${legIndex + 2}`;
      const faces = `${label} faces ${facing(p)}`;
      if (rule.type === 'fixed') {
        issues.push({ point: i, message: `${faces}, but ${leg} must face ${deg(rule.heading)}° ±${deg(rule.tol)}°.` });
      } else if (rule.type === 'turret-reach') {
        // Headings that put the hive inside the turret's range (less the margin), from this point.
        const bearing = (angleOf(sub(RED_HIVE_TARGET, p)) * 180) / Math.PI;
        const norm = (a: number) => Math.round((((a + 180) % 360) + 360) % 360 - 180);
        const from = norm(bearing - (turret.max - rule.margin));
        const to = norm(bearing - (turret.min + rule.margin));
        issues.push({ point: i, message: `${faces}, but on ${leg} the turret must reach the hive, which from here needs facing between ${from}° and ${to}°.` });
      }
    }
  });
  chain.points.forEach((p, i) => {
    const problem = shotProblem(settings, p);
    if (problem) issues.push({ point: i, message: `${pointLabel(chain, i)} ${problem}` });
  });
  for (const s of startProblems(settings, chain)) issues.push({ point: 0, message: `Start ${s.problem}.` });
  return issues;
}

/**
 * Why a shooting point can't work, if it can't: no standing shot from anywhere it allows scores with the robot's
 * accuracy, or none of its headings lets the turret face the hive. (Shots on the move are checked standing here; the
 * optimizer checks them moving.)
 */
export function shotProblem(settings: Settings, p: ChainPoint): string | null {
  if (!p.shoot) return null;
  const cfg = settings.shooter;
  const solver = new ShotSolver(cfg);
  const turret = turretRange(settings.robot);
  const headings = headingsOf(p, 5);
  let bestMargin = -Infinity;
  let anyShot = false;
  let turretOk = false;
  let bearing = 0;
  for (const pos of positionsOf(p)) {
    const shot = solver.solve(pos.x, pos.y, 0, 0, 0, 0, p.shoot.cell);
    if (!shot.feasible) continue;
    anyShot = true;
    const m = shotMargin(shot, cfg);
    bestMargin = Math.max(bestMargin, m);
    if (m < 0) continue;
    bearing = (shot.turretAngle * 180) / Math.PI; // relative to heading 0, so the field bearing of the aim
    if (headings.some((h) => {
      const rel = ((((bearing - (h * 180) / Math.PI) % 360) + 540) % 360) - 180;
      return rel >= turret.min && rel <= turret.max;
    })) {
      turretOk = true;
      break;
    }
  }
  const where = p.kind === 'region' ? 'from anywhere in it' : 'from here';
  if (!anyShot) return `can't shoot ${where}: no flywheel speed gets a ball into the ${p.shoot.cell} cell.`;
  if (bestMargin < 0) return `can't shoot ${where} reliably: the shot has less room than the robot's aim (±${cfg.aimAccuracy}°) and flywheel (±${cfg.speedAccuracy}%) errors.`;
  if (!turretOk) return `can't shoot ${where} at its heading (${facing(p)}): the turret can't turn to face the ${p.shoot.cell} cell.`;
  return null;
}
