/**
 * What a path is made from. Everything is red side, inches, degrees; the planner converts headings to radians.
 * A path (Chain) travels with its Settings inside the exported Java, so pasting the code back restores both.
 */

export type Side = 'front' | 'back' | 'left' | 'right';

/** A box sticking out of one side of the frame: `from`/`to` measured along that side from its centre, `depth` outward. */
export interface Protrusion {
  side: Side;
  from: number;
  to: number;
  depth: number;
}

export interface RobotConfig {
  /** Side to side, inches. */
  width: number;
  /** Front to back, inches. */
  length: number;
  /** Highest point, including the turret. */
  height: number;
  protrusions: Protrusion[];
  /** How far the turret turns, degrees from the robot's front (counter-clockwise positive). Default −90 to 90. */
  turretMin?: number;
  turretMax?: number;
}

/** The turret's range in degrees, with the defaults for settings saved before it was configurable. */
export function turretRange(robot: RobotConfig): { min: number; max: number } {
  return { min: robot.turretMin ?? -90, max: robot.turretMax ?? 90 };
}

export interface Margins {
  /** Gap to obstacles and walls. */
  obstacle: number;
  /** Gap to the field centerline. */
  centerline: number;
  /** Gap above the robot under the hive frame. */
  headroom: number;
}

export interface ModelConfig {
  /** Where the numbers came from, e.g. "placeholder", "Foresight Tuner 2026-10-04", "fit from 12 runs". */
  source: string;
  /** Top speeds, in/s. */
  vForward: number;
  vStrafe: number;
  /** Fastest turn, rad/s, while not translating. */
  omegaMax: number;
  /** Accelerations, in/s². */
  aForward: number;
  aStrafe: number;
  dForward: number;
  dStrafe: number;
  /** Largest sideways (centripetal) acceleration, in/s². */
  aLateral: number;
  /** Added once at the end of a path that stops: settling on the end pose, s. */
  stopOverhead: number;
  /** Battery voltage the speeds were measured at. */
  nominalVoltage: number;
}

/** The robot, margins and speed model: shared by every path. */
export interface Settings {
  robot: RobotConfig;
  margins: Margins;
  model: ModelConfig;
}

/** A point the robot centre passes exactly, with its heading. */
export interface FixedPoint {
  kind: 'fixed';
  x: number;
  y: number;
  heading: number;
  /** Allowed heading error here, degrees (0 = exact). */
  headingTol: number;
}

/** An area the robot centre passes through somewhere; the optimizer picks where. */
export interface RegionPoint {
  kind: 'region';
  x: number;
  y: number;
  halfWidth: number;
  halfHeight: number;
  heading: number;
  headingTol: number;
}

export type ChainPoint = FixedPoint | RegionPoint;

export type HeadingRule =
  | { type: 'free' }
  | { type: 'fixed'; heading: number; tol: number }
  | { type: 'front-first'; tol: number }
  | { type: 'turret-reach'; margin: number };

export interface Leg {
  rules: HeadingRule[];
}

export interface MarkerSpec {
  name: string;
  /** Which leg (0 = from the first point to the second). */
  leg: number;
  /** How far along that leg, as a fraction of its length. */
  at: number;
}

/** One path: driven as a single follow(). */
export interface Chain {
  name: string;
  points: ChainPoint[];
  /** legs[i] goes from points[i] to points[i + 1]. */
  legs: Leg[];
  /** Whether the robot stops at the last point, or keeps moving into whatever follows. */
  endStopped: boolean;
  /** The first point is where the robot is placed for the match; checked against G304. */
  matchStart: boolean;
  /** May cross the centerline into the other half (not allowed in AUTO, G402). */
  allowCrossing?: boolean;
  markers: MarkerSpec[];
  /** Extra areas to avoid on this path only (red side). */
  keepOut: { name: string; points: { x: number; y: number }[] }[];
  /** Per-path margin overrides. */
  margins?: Partial<Margins>;
}

export const PLACEHOLDER_MODEL: ModelConfig = {
  source: 'placeholder (not measured)',
  vForward: 60,
  vStrafe: 48,
  omegaMax: 6,
  aForward: 70,
  aStrafe: 55,
  dForward: 60,
  dStrafe: 45,
  aLateral: 80,
  stopOverhead: 0.25,
  nominalVoltage: 12.5,
};

export function defaultSettings(): Settings {
  return {
    robot: { width: 16, length: 14, height: 14, protrusions: [], turretMin: -90, turretMax: 90 },
    margins: { obstacle: 2, centerline: 1, headroom: 2 },
    model: { ...PLACEHOLDER_MODEL },
  };
}

export class ProjectError extends Error {
  readonly problems: string[];
  constructor(problems: string[]) {
    super(problems.join('\n'));
    this.problems = problems;
  }
}

const isNum = (v: unknown): v is number => typeof v === 'number' && Number.isFinite(v);

/** Throws with every problem found, so they can be shown at once. */
export function validateSettings(s: Settings): void {
  const problems: string[] = [];
  const num = (v: unknown, what: string, min = -Infinity) => {
    if (!isNum(v)) problems.push(`${what} must be a number`);
    else if (v < min) problems.push(`${what} must be at least ${min}`);
  };
  num(s.robot?.width, 'robot width', 1);
  num(s.robot?.length, 'robot length', 1);
  num(s.robot?.height, 'robot height', 1);
  for (const k of ['obstacle', 'centerline', 'headroom'] as const) num(s.margins?.[k], `${k} margin`, 0);
  if (s.robot?.turretMin !== undefined) num(s.robot.turretMin, 'turret range start');
  if (s.robot?.turretMax !== undefined) num(s.robot.turretMax, 'turret range end');
  if (s.robot && turretRange(s.robot).max <= turretRange(s.robot).min) problems.push('the turret range must end after it starts');
  for (const k of ['vForward', 'vStrafe', 'omegaMax', 'aForward', 'aStrafe', 'dForward', 'dStrafe', 'aLateral', 'nominalVoltage'] as const) {
    num(s.model?.[k], k, 1e-6);
  }
  num(s.model?.stopOverhead, 'stopOverhead', 0);
  if (problems.length) throw new ProjectError(problems);
}

export const JAVA_NAME = /^[a-z][A-Za-z0-9]*$/;
export const MARKER_NAME = /^[A-Z][A-Z0-9_]*$/;

export function validateChain(c: Chain): void {
  const problems: string[] = [];
  if (!JAVA_NAME.test(c.name ?? '')) problems.push(`"${c.name}" isn't a Java method name (camelCase, letters and digits)`);
  if (!Array.isArray(c.points) || c.points.length < 2) problems.push(`${c.name}: needs a start and an end`);
  else if (c.points[0].kind !== 'fixed' || c.points[c.points.length - 1].kind !== 'fixed') {
    problems.push(`${c.name}: the start and end must be points, not areas`);
  }
  if (!Array.isArray(c.legs) || c.legs.length !== Math.max(0, (c.points?.length ?? 0) - 1)) problems.push(`${c.name}: needs one leg between each pair of points`);
  for (const p of c.points ?? []) {
    if (![p.x, p.y, p.heading, p.headingTol].every(isNum)) problems.push(`${c.name}: a point has a missing number`);
    if (p.kind === 'region' && !(p.halfWidth >= 0 && p.halfHeight >= 0)) problems.push(`${c.name}: an area has a negative size`);
  }
  if (c.allowCrossing !== undefined && typeof c.allowCrossing !== 'boolean') problems.push(`${c.name}: allowCrossing must be true or false`);
  const names = new Set<string>();
  for (const m of c.markers ?? []) {
    if (!MARKER_NAME.test(m.name)) problems.push(`${c.name}: marker "${m.name}" must be UPPER_SNAKE_CASE`);
    if (names.has(m.name)) problems.push(`${c.name}: two markers are called ${m.name}`);
    names.add(m.name);
    if (!(m.leg >= 0 && m.leg < (c.legs?.length ?? 0))) problems.push(`${c.name}: marker ${m.name} is on a leg that doesn't exist`);
    if (!(m.at >= 0 && m.at <= 1)) problems.push(`${c.name}: marker ${m.name} must be between 0 and 100 % along its leg`);
  }
  for (const k of c.keepOut ?? []) if (k.points.length < 3) problems.push(`${c.name}: avoid zone "${k.name}" needs at least 3 corners`);
  if (problems.length) throw new ProjectError(problems);
}

export function marginsFor(s: Settings, chain: Chain): Margins {
  return { ...s.margins, ...(chain.margins ?? {}) };
}
