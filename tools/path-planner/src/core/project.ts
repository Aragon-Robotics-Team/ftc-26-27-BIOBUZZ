import { DEFAULT_SHOOTER, type Cell, type ShooterConfig } from './shot.ts';

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

/**
 * The drivetrain, physically: four mecanum wheels, each driven by a DC motor whose force falls as it spins faster.
 * The planner derives top speeds, launch acceleration, turning and grip limits from these (see drivetrain.ts).
 */
export interface ModelConfig {
  /** Where the numbers came from, e.g. "sample", "Foresight Tuner 2026-10-04", "fit from 12 runs". */
  source: string;
  /** Motor output shaft at the spec voltage: free speed (RPM) and stall torque (N·m). */
  motorRpm: number;
  motorStallTorque: number;
  /** Extra reduction between motor and wheel (motor turns per wheel turn; 1 = direct). */
  reduction: number;
  /** Wheel diameter, inches. */
  wheelDiameter: number;
  /** Distance between the front and back axles, and between the left and right wheels, inches. */
  wheelbase: number;
  trackWidth: number;
  /** Robot mass, kg. */
  mass: number;
  /** How hard the wheels can push before slipping, as a fraction of the robot's weight (μ). */
  grip: number;
  /** Top speed forward as a fraction of the wheels' free speed (gearbox and roller losses). */
  driveEfficiency: number;
  /** Top speed sideways as a fraction of the top speed forward (mecanum rollers lose some). */
  strafeEfficiency: number;
  /** Slowing down with no power (friction), in/s²: the Foresight Tuner's natural decelerations. */
  coastForward: number;
  coastStrafe: number;
  /** Added once at the end of a path that stops: settling on the end pose, s. */
  stopOverhead: number;
  /** Voltage the motor specs are for. */
  nominalVoltage: number;
}

/** goBILDA 5203 Yellow Jacket motors at 12 V (goBILDA spec sheets). */
export const MOTORS: { name: string; rpm: number; stallTorque: number }[] = [
  { name: 'goBILDA 223 RPM', rpm: 223, stallTorque: 3.73 }, // 38.0 kg·cm
  { name: 'goBILDA 312 RPM', rpm: 312, stallTorque: 2.38 }, // 24.3 kg·cm
  { name: 'goBILDA 435 RPM', rpm: 435, stallTorque: 1.83 }, // 18.7 kg·cm
  { name: 'goBILDA 1150 RPM', rpm: 1150, stallTorque: 0.775 }, // 7.9 kg·cm
];

/**
 * Starting numbers until the robot is measured: the drive motors in pedro/Constants.java (goBILDA 312 RPM), 104 mm
 * mecanum wheels, and the sample robot from Pedro's quickstart for mass, strafe/forward speed ratio (65.4 / 81.3) and
 * coasting decelerations (34.6, 78.2 in/s²). Grip (mecanum rollers on tiles) and drive efficiency are typical values.
 */
export const SAMPLE_MODEL: ModelConfig = {
  source: 'sample (not measured)',
  motorRpm: 312,
  motorStallTorque: 2.38,
  reduction: 1,
  wheelDiameter: 4.094,
  wheelbase: 10,
  trackWidth: 13,
  mass: 10.66,
  grip: 0.5,
  driveEfficiency: 0.9,
  strafeEfficiency: 0.8,
  coastForward: 34.6,
  coastStrafe: 78.2,
  stopOverhead: 0.25,
  nominalVoltage: 12,
};

/** Settings saved before the drivetrain was modelled physically get the sample drivetrain; before shooting, the
 *  default shooter. */
export function migrateSettings(s: Settings): Settings {
  const m = s.model as unknown as Record<string, unknown> | undefined;
  let out = s;
  if (!(m && typeof m.motorRpm === 'number')) {
    out = { ...out, model: { ...SAMPLE_MODEL, stopOverhead: typeof m?.stopOverhead === 'number' ? m.stopOverhead : SAMPLE_MODEL.stopOverhead } };
  }
  if (!out.shooter) out = { ...out, shooter: { ...DEFAULT_SHOOTER } };
  return out;
}

/** The robot, margins, speed model and shooter: shared by every path. */
export interface Settings {
  robot: RobotConfig;
  margins: Margins;
  model: ModelConfig;
  shooter: ShooterConfig;
}

/**
 * Shooting at a point: `stop` comes to rest there (or somewhere in the area), waits for the launcher to be ready and
 * fires a volley, then drives on as a new Pedro path. `move` fires the volley while passing through, centred on the
 * point, no faster than `maxSpeed` in/s.
 */
export interface ShootSpec {
  mode: 'stop' | 'move';
  cell: Cell;
  maxSpeed: number;
}

/** A point the robot centre passes exactly, with its heading. */
export interface FixedPoint {
  kind: 'fixed';
  x: number;
  y: number;
  heading: number;
  /** Allowed heading error here, degrees (0 = exact). */
  headingTol: number;
  shoot?: ShootSpec;
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
  shoot?: ShootSpec;
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


export function defaultSettings(): Settings {
  return {
    robot: { width: 16, length: 14, height: 14, protrusions: [], turretMin: -90, turretMax: 90 },
    margins: { obstacle: 2, centerline: 1, headroom: 2 },
    model: { ...SAMPLE_MODEL },
    shooter: { ...DEFAULT_SHOOTER },
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
  for (const k of ['motorRpm', 'motorStallTorque', 'reduction', 'wheelDiameter', 'wheelbase', 'trackWidth', 'mass', 'grip', 'driveEfficiency', 'strafeEfficiency', 'coastForward', 'coastStrafe', 'nominalVoltage'] as const) {
    num(s.model?.[k], k, 1e-6);
  }
  num(s.model?.stopOverhead, 'stopOverhead', 0);
  for (const k of ['balls', 'volley', 'inertia', 'motorRpm', 'motorStallTorque', 'ticksPerExitSpeed', 'hoodAngle', 'exitHeight'] as const) {
    num(s.shooter?.[k], `shooter ${k}`, 1e-6);
  }
  for (const k of ['aimAccuracy', 'speedAccuracy', 'drag', 'lift', 'scoringMargin'] as const) num(s.shooter?.[k], `shooter ${k}`, 0);
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
  (c.points ?? []).forEach((p, i) => {
    if (!p.shoot) return;
    if (!['stop', 'move'].includes(p.shoot.mode) || !['left', 'right'].includes(p.shoot.cell)) problems.push(`${c.name}: a shot has an unknown mode or cell`);
    if (p.shoot.mode === 'move' && (i === 0 || i === c.points.length - 1)) problems.push(`${c.name}: shooting on the move needs a point in the middle of the path`);
    if (p.shoot.mode === 'move' && !(isNum(p.shoot.maxSpeed) && p.shoot.maxSpeed > 0)) problems.push(`${c.name}: a shot on the move needs a top speed`);
  });
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
