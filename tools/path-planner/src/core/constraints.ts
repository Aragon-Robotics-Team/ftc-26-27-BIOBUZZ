/** Clearance and heading-rule checks along a sampled chain, plus the G304 start-pose check. */
import type { Sample } from './chain.ts';
import { CENTERLINE_X, fieldObstacles, flowers, RED_HIVE_TARGET, RED_LOADING_ZONE, WALL_MAX, WALL_MIN, type Obstacle } from './field.ts';
import { angleOf, bounds, convexDistance, convexHull, rad, sub, wrap, type Polygon, type Pose } from './geom.ts';
import { turretRange, type Chain, type HeadingRule, type Margins, type Settings } from './project.ts';
import { localFootprint, placeFootprint } from './robot.ts';

export interface Clearance {
  /** Obstacle name, "Walls" or "Centerline". */
  name: string;
  /** Smallest gap along the chain, inches (negative = overlap). */
  gap: number;
  /** Margin it must keep. */
  margin: number;
  /** Arc length where the smallest gap happens. */
  at: number;
}

export interface RuleViolation {
  leg: number;
  rule: HeadingRule['type'];
  /** Worst excess beyond the rule's tolerance, radians. */
  excess: number;
  at: number;
}

export interface Check {
  clearances: Clearance[];
  rules: RuleViolation[];
  /** Sum of all violations (inches, and radians × 10), weighted by distance: 0 when every limit holds. */
  penalty: number;
  /** Largest single violation in the same units. */
  worst: number;
}

/** Obstacles that apply to a chain: the field at robot height + headroom, plus the chain's own keep-out zones. */
export function chainObstacles(project: Settings, chain: Chain, margins: Margins): Obstacle[] {
  const top = project.robot.height + margins.headroom;
  return [
    ...fieldObstacles(top),
    // Keep-outs are used as their convex outline.
    ...chain.keepOut.map((k) => ({ name: k.name || 'Keep-out', polygon: convexHull(k.points as Polygon) })),
  ];
}

const RULE_WEIGHT = 10; // a radian of heading error counts like 10 in of collision

export function checkChain(
  samples: Sample[],
  project: Settings,
  chain: Chain,
  margins: Margins,
  obstacles: Obstacle[],
  stride = 1,
): Check {
  // A match start touches the wall (G304), so on such a path the wall margin grows from the start's own gap as the
  // robot drives away, reaching the full margin after that many inches.
  const p0 = chain.points[0];
  const startWallGap = chain.matchStart && p0 ? wallGap(project, { x: p0.x, y: p0.y, h: rad(p0.heading) }) : null;
  const parts = localFootprint(project.robot);
  const turret = turretRange(project.robot);
  const obstacleBounds = obstacles.map((o) => bounds(o.polygon));
  // The centerline (G402) only applies if the path may not cross into the other half.
  const centerline = !chain.allowCrossing;
  const first = centerline ? 2 : 1;
  const clear: Clearance[] = [
    { name: 'Walls', gap: Infinity, margin: margins.obstacle, at: 0 },
    ...(centerline ? [{ name: 'Centerline', gap: Infinity, margin: margins.centerline, at: 0 }] : []),
    ...obstacles.map((o) => ({ name: o.name, gap: Infinity, margin: margins.obstacle, at: 0 })),
  ];
  const rules = new Map<string, RuleViolation>();
  let penalty = 0;
  let worst = 0;
  const step = (i: number) => (i + stride < samples.length ? samples[i + stride].s - samples[i].s : 0.5);
  const note = (c: Clearance, gap: number, s: number, weight: number) => {
    if (gap < c.gap) {
      c.gap = gap;
      c.at = s;
    }
    const v = c.margin - gap;
    if (v > 0) {
      penalty += v * weight;
      if (v > worst) worst = v;
    }
  };
  const nParts = parts.length;
  for (let i = 0; i < samples.length; i += stride) {
    const smp = samples[i];
    const w = Math.max(step(i), 0.25);
    const c = Math.cos(smp.h);
    const sn = Math.sin(smp.h);
    // The footprint at this sample, and its bounding box, without allocating per vertex more than needed.
    const placed: Polygon[] = new Array(nParts);
    let x0 = Infinity;
    let y0 = Infinity;
    let x1 = -Infinity;
    let y1 = -Infinity;
    for (let k = 0; k < nParts; k++) {
      const local = parts[k];
      const out: Polygon = new Array(local.length);
      for (let v = 0; v < local.length; v++) {
        const px = local[v].x * c - local[v].y * sn + smp.p.x;
        const py = local[v].x * sn + local[v].y * c + smp.p.y;
        out[v] = { x: px, y: py };
        if (px < x0) x0 = px;
        if (px > x1) x1 = px;
        if (py < y0) y0 = py;
        if (py > y1) y1 = py;
      }
      placed[k] = out;
    }
    const toWall = Math.min(x0 - WALL_MIN, y0 - WALL_MIN, WALL_MAX - x1, WALL_MAX - y1);
    // Reported as if the full margin applied, so "gap ≥ margin" still means fine.
    const wallAllowance = startWallGap === null ? 0 : Math.max(0, margins.obstacle - Math.max(0, startWallGap) - smp.s);
    note(clear[0], toWall + wallAllowance, smp.s, w);
    if (centerline) note(clear[1], CENTERLINE_X - x1, smp.s, w);
    for (let j = 0; j < obstacles.length; j++) {
      const ob = obstacleBounds[j];
      const cl = clear[j + first];
      // Not near the margin: the bounding-box gap is a lower bound on the real gap, and close enough to report.
      const bx = Math.max(ob.x0 - x1, x0 - ob.x1, 0);
      const by = Math.max(ob.y0 - y1, y0 - ob.y1, 0);
      const boxGap = Math.sqrt(bx * bx + by * by);
      if (boxGap > cl.margin + 0.5) {
        if (boxGap < cl.gap) {
          cl.gap = boxGap;
          cl.at = smp.s;
        }
        continue;
      }
      let gap = Infinity;
      for (const poly of placed) gap = Math.min(gap, convexDistance(poly, obstacles[j].polygon));
      note(cl, gap, smp.s, w);
    }
    const leg = chain.legs[smp.leg];
    for (const rule of leg?.rules ?? []) {
      const excess = ruleExcess(rule, smp, turret);
      if (excess > 0) {
        const key = `${smp.leg}:${rule.type}`;
        const prev = rules.get(key);
        if (!prev || excess > prev.excess) rules.set(key, { leg: smp.leg, rule: rule.type, excess, at: smp.s });
        penalty += excess * RULE_WEIGHT * w;
        worst = Math.max(worst, excess * RULE_WEIGHT);
      }
    }
  }
  return { clearances: clear, rules: [...rules.values()], penalty, worst };
}

/** How far a sample breaks a heading rule, radians (0 if it holds). */
export function ruleExcess(rule: HeadingRule, smp: Sample, turret: { min: number; max: number } = { min: -90, max: 90 }): number {
  switch (rule.type) {
    case 'free':
      return 0;
    case 'fixed':
      return Math.max(0, Math.abs(wrap(smp.h - rad(rule.heading))) - rad(rule.tol));
    case 'front-first':
      return Math.max(0, Math.abs(wrap(smp.h - smp.travel)) - rad(rule.tol));
    case 'turret-reach': {
      // The hive's direction relative to the robot's front must be inside the turret's range, less the margin.
      if (turret.max - turret.min >= 360) return 0;
      const lo = rad(turret.min + rule.margin);
      const hi = rad(turret.max - rule.margin);
      if (hi < lo) return Math.PI;
      const rel = wrap(angleOf(sub(RED_HIVE_TARGET, smp.p)) - smp.h);
      let excess = Infinity;
      for (const r of [rel - 2 * Math.PI, rel, rel + 2 * Math.PI]) excess = Math.min(excess, r < lo ? lo - r : r > hi ? r - hi : 0);
      return excess;
    }
  }
}

export interface StartProblem {
  pose: string;
  problem: string;
}

/**
 * G304: a robot starts fully on its own side, touching a field wall, not touching a flower, and not in its loading
 * zone. "Touching" allows 0.5 in, since people place robots by hand.
 */
export function checkStartPose(project: Settings, name: string, pose: Pose): StartProblem[] {
  const out: StartProblem[] = [];
  const placed = placeFootprint(localFootprint(project.robot), pose);
  let x0 = Infinity;
  let y0 = Infinity;
  let x1 = -Infinity;
  let y1 = -Infinity;
  for (const poly of placed) {
    const b = bounds(poly);
    x0 = Math.min(x0, b.x0);
    y0 = Math.min(y0, b.y0);
    x1 = Math.max(x1, b.x1);
    y1 = Math.max(y1, b.y1);
  }
  const touch = 0.5;
  if (x1 > CENTERLINE_X) out.push({ pose: name, problem: `reaches x = ${x1.toFixed(1)}, past the centerline (G304)` });
  if (x0 < WALL_MIN - 0.05 || y0 < WALL_MIN - 0.05 || x1 > WALL_MAX + 0.05 || y1 > WALL_MAX + 0.05) {
    out.push({ pose: name, problem: 'goes through a field wall' });
  }
  const wallGap = Math.min(x0 - WALL_MIN, y0 - WALL_MIN, WALL_MAX - x1, WALL_MAX - y1);
  if (wallGap > touch) out.push({ pose: name, problem: `isn't touching a wall (closest is ${wallGap.toFixed(2)} in away; G304)` });
  for (const f of flowers()) {
    let gap = Infinity;
    for (const poly of placed) gap = Math.min(gap, convexDistance(poly, f.polygon));
    if (gap <= 0) out.push({ pose: name, problem: `touches the ${f.name.toLowerCase()} (G304)` });
  }
  let zone = Infinity;
  for (const poly of placed) zone = Math.min(zone, convexDistance(poly, RED_LOADING_ZONE));
  if (zone < 0) out.push({ pose: name, problem: 'is inside the loading zone (G304)' });
  return out;
}

/** Gap between the robot at a pose and the nearest wall (negative if it's through one). */
export function wallGap(project: Settings, pose: Pose): number {
  let gap = Infinity;
  for (const poly of placeFootprint(localFootprint(project.robot), pose)) {
    const b = bounds(poly);
    gap = Math.min(gap, b.x0 - WALL_MIN, b.y0 - WALL_MIN, WALL_MAX - b.x1, WALL_MAX - b.y1);
  }
  return gap;
}
