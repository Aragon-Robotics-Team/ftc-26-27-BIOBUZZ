/**
 * Shooting, as the robot does it: TeamCode's ShotSolver ported line for line (ball flight with drag and backspin lift,
 * launched with the robot's velocity; Newton on exit speed and turret heading, then the most reliable speed), plus the
 * flywheel, which can only change speed as fast as its motor and inertia allow and loses some speed to every ball.
 *
 * A pose is a good place to shoot from when the shot there can take the robot's errors: the turret's aim error inside
 * the shot's angle tolerance, the flywheel's speed error (its own accuracy plus how far it lags the speed the shot
 * needs) inside the shot's speed tolerance.
 */
import type { Vec } from './geom.ts';

/** Which cell of our hive is up (red side, from our driver station). Both alliances start with the right one up. */
export type Cell = 'left' | 'right';

export interface ShooterConfig {
  /** Balls in a volley, and how long the gate stays open for them (Gate.AUTO_SHOOT_MS), s. */
  balls: number;
  volley: number;
  /** How far off the robot's aim and flywheel speed can be: localization plus turret control, degrees either way;
   *  flywheel control plus calibration, percent either way. A shot counts only if it can take both. */
  aimAccuracy: number;
  speedAccuracy: number;
  /** Everything the flywheel motor spins, at the motor shaft (wheel, counter-rollers through their gearing), kg·cm². */
  inertia: number;
  /** The flywheel motor at its spec voltage: free speed (RPM) and stall torque (N·m). */
  motorRpm: number;
  motorStallTorque: number;
  /** RobotConstants.Launcher: flywheel encoder ticks/s per in/s of ball exit speed (calibrated on the robot). */
  ticksPerExitSpeed: number;
  /** Fixed hood angle above horizontal (degrees) and where the ball leaves (inches above the tiles). */
  hoodAngle: number;
  exitHeight: number;
  /** Air: drag and backspin lift coefficients. */
  drag: number;
  lift: number;
  /** Inches the ball's edge must clear each side of the opening by. */
  scoringMargin: number;
}

/**
 * The launcher as RobotConstants.Launcher has it, a goBILDA 6000 RPM motor (1.47 kg·cm stall) on a 96 mm goBILDA
 * Rhino wheel. Inertia: the 142 g wheel with most of its mass in the tread (≈ 0.65·m·r²) plus about 0.1 kg·cm² for
 * the hub, shaft and motor rotor, ≈ 2.2 kg·cm². Add the counter-rollers (times the square of their gear ratio) once
 * they're built. Volley 3 balls in 1 s is Gate.AUTO_SHOOT_MS; the accuracies are guesses until measured.
 */
export const DEFAULT_SHOOTER: ShooterConfig = {
  balls: 3,
  volley: 1,
  aimAccuracy: 1.5,
  speedAccuracy: 2,
  inertia: 2.2,
  motorRpm: 6000,
  motorStallTorque: 0.144,
  ticksPerExitSpeed: 8,
  hoodAngle: 67.608,
  exitHeight: 13,
  drag: 0.5,
  lift: 0.3,
  scoringMargin: 0.5,
};

// RobotConstants.Field and the parts of RobotConstants.Launcher that aren't tunables here.
const CELLS: Record<Cell, { x: number; y: number; heading: number }> = {
  left: { x: 59.25, y: 87.7, heading: Math.PI / 2 }, // rear cell
  right: { x: 59.25, y: 56.3, heading: -Math.PI / 2 }, // audience cell
};
const OPENING_CENTER_Z = 59.6;
const OPENING_WIDTH = 20;
const OPENING_HEIGHT = 14;
const OPENING_TILT = (30 * Math.PI) / 180;
const BALL_DIAMETER = 2.8;
const BALL_MASS = 0.0249; // kg
const AIR_DENSITY = 1.2;
const ENCODER_TICKS_PER_REV = 28;
/** The robot fires once flywheel and turret are within this share of what the shot can take (READY_WINDOW_FRACTION). */
export const READY_FRACTION = 0.5;

/** Where to aim for a cell: centre of its opening, heading pointing out of it. */
export function cellTarget(cell: Cell): { x: number; y: number; heading: number } {
  return CELLS[cell];
}

export interface Shot {
  feasible: boolean;
  /** Ball speed out of the launcher, in/s, and the flywheel speed for it, ticks/s. */
  exitSpeed: number;
  flywheel: number;
  /** Turret angle relative to the robot's front, rad (counter-clockwise positive), unclamped. */
  turretAngle: number;
  /** How far exit speed (in/s) and aim (rad) can be off either way and still score. */
  speedTolerance: number;
  angleTolerance: number;
  timeOfFlight: number;
  distance: number;
}

const GRAVITY = 386.09;
const METERS_PER_INCH = 0.0254;
const STEP = 0.01;
const MAX_FLIGHT = 2;
const SPEED_PROBE = 0.5;
const YAW_PROBE = 0.002;
const CONVERGED = 0.05;
const MAX_ITERATIONS = 8;

const normalize = (a: number) => {
  let r = a % (2 * Math.PI);
  if (r > Math.PI) r -= 2 * Math.PI;
  if (r <= -Math.PI) r += 2 * Math.PI;
  return r;
};
const clamp = (v: number, limit: number) => Math.max(-limit, Math.min(limit, v));

/** TeamCode's ShotSolver, minus its per-loop cache and aim lookahead (the planner knows the pose exactly). */
export class ShotSolver {
  private readonly hood: number;
  private readonly exitHeight: number;
  private readonly dragC: number;
  private readonly liftC: number;
  private readonly margin: number;
  private readonly ticksPerExit: number;
  private readonly maxExit: number;
  private readonly k: number;

  private pivotX = 0;
  private pivotY = 0;
  private ballVx = 0;
  private ballVy = 0;
  private centerX = 0;
  private centerY = 0;
  private normalX = 0;
  private normalY = 0;
  private normalZ = 0;
  private upX = 0;
  private upY = 0;
  private upZ = 0;
  private sideX = 0;
  private sideY = 0;
  private hitUp = 0;
  private hitSide = 0;
  private hitTime = 0;
  private spinX = 0;
  private spinY = 0;
  private ax = 0;
  private ay = 0;
  private az = 0;
  private lastSpeed = NaN;
  private lastYaw = 0;

  constructor(cfg: ShooterConfig) {
    this.hood = (cfg.hoodAngle * Math.PI) / 180;
    this.exitHeight = cfg.exitHeight;
    this.dragC = cfg.drag;
    this.liftC = cfg.lift;
    this.margin = cfg.scoringMargin;
    this.ticksPerExit = cfg.ticksPerExitSpeed;
    this.maxExit = flywheelMaxTicks(cfg) / cfg.ticksPerExitSpeed;
    const radius = (BALL_DIAMETER / 2) * METERS_PER_INCH;
    this.k = ((0.5 * AIR_DENSITY * Math.PI * radius * radius) / BALL_MASS) * METERS_PER_INCH;
  }

  /** Forget the warm start, e.g. before solving somewhere far from the last pose. */
  reset(): void {
    this.lastSpeed = NaN;
  }

  /** @param vx, vy field velocity of the robot (in/s), omega its turn rate (rad/s); h its heading (rad). */
  solve(x: number, y: number, h: number, vx: number, vy: number, omega: number, cell: Cell): Shot {
    const target = CELLS[cell];
    this.setUp(x, y, vx, vy, omega, target);
    const dx = this.centerX - this.pivotX;
    const dy = this.centerY - this.pivotY;
    const distance = Math.max(Math.hypot(dx, dy), 1);
    const directYaw = Math.atan2(dy, dx);

    let speed = Number.isNaN(this.lastSpeed) ? this.firstGuess(distance) : this.lastSpeed;
    let yaw = Number.isNaN(this.lastSpeed) ? directYaw : this.lastYaw;

    let converged = false;
    let upPerSpeed = 0;
    let sidePerYaw = 0;
    for (let i = 0; i < MAX_ITERATIONS && !converged; i++) {
      if (!this.fly(speed, yaw)) {
        speed *= 1.1;
        continue;
      }
      const up = this.hitUp;
      const side = this.hitSide;
      if (!this.fly(speed + SPEED_PROBE, yaw)) break;
      const upDv = (this.hitUp - up) / SPEED_PROBE;
      const sideDv = (this.hitSide - side) / SPEED_PROBE;
      if (!this.fly(speed, yaw + YAW_PROBE)) break;
      const upDyaw = (this.hitUp - up) / YAW_PROBE;
      const sideDyaw = (this.hitSide - side) / YAW_PROBE;
      upPerSpeed = upDv;
      sidePerYaw = sideDyaw;
      if (Math.abs(up) < CONVERGED && Math.abs(side) < CONVERGED) {
        converged = true;
        break;
      }
      const det = upDv * sideDyaw - upDyaw * sideDv;
      if (Math.abs(det) < 1e-9) break;
      const dSpeed = (sideDyaw * up - upDyaw * side) / det;
      const dYaw = (upDv * side - sideDv * up) / det;
      speed -= clamp(dSpeed, 50);
      yaw -= clamp(dYaw, 0.3);
      if (speed <= 0) break;
    }

    const none = (s: number, aim: number): Shot => {
      this.lastSpeed = NaN;
      const exit = Math.min(Number.isNaN(s) || s <= 0 ? this.firstGuess(distance) : s, this.maxExit);
      return { feasible: false, exitSpeed: exit, flywheel: exit * this.ticksPerExit, turretAngle: normalize(aim - h), speedTolerance: 0, angleTolerance: 0, timeOfFlight: 0, distance };
    };
    if (!converged || speed > this.maxExit || upPerSpeed <= 0) return none(speed, directYaw);

    const halfHeight = OPENING_HEIGHT / 2 - BALL_DIAMETER / 2 - this.margin;
    const halfWidth = OPENING_WIDTH / 2 - BALL_DIAMETER / 2 - this.margin;
    const low = this.speedForUp(-halfHeight, speed, yaw, upPerSpeed);
    const high = this.speedForUp(halfHeight, speed, yaw, upPerSpeed);
    const best = (low + high) / 2;
    const speedTolerance = (high - low) / 2;

    if (this.fly(best, yaw) && Math.abs(sidePerYaw) > 1e-6) yaw -= this.hitSide / sidePerYaw;
    if (!this.fly(best, yaw) || best > this.maxExit) return none(speed, yaw);

    this.lastSpeed = speed;
    this.lastYaw = yaw;
    return {
      feasible: true,
      exitSpeed: best,
      flywheel: best * this.ticksPerExit,
      turretAngle: normalize(yaw - h),
      speedTolerance,
      angleTolerance: Math.abs(sidePerYaw) > 1e-6 ? halfWidth / Math.abs(sidePerYaw) : 0,
      timeOfFlight: this.hitTime,
      distance,
    };
  }

  private setUp(x: number, y: number, vx: number, vy: number, omega: number, target: { x: number; y: number; heading: number }) {
    // TURRET_OFFSET_X/Y are 0, so the pivot is the robot centre and spinning adds no velocity to the ball.
    this.pivotX = x;
    this.pivotY = y;
    this.ballVx = vx;
    this.ballVy = vy;
    void omega;
    this.centerX = target.x;
    this.centerY = target.y;
    const outX = Math.cos(target.heading);
    const outY = Math.sin(target.heading);
    const cosTilt = Math.cos(OPENING_TILT);
    const sinTilt = Math.sin(OPENING_TILT);
    this.normalX = outX * cosTilt;
    this.normalY = outY * cosTilt;
    this.normalZ = sinTilt;
    this.upX = -outX * sinTilt;
    this.upY = -outY * sinTilt;
    this.upZ = cosTilt;
    this.sideX = -outY;
    this.sideY = outX;
  }

  private firstGuess(distance: number): number {
    const rise = OPENING_CENTER_Z - this.exitHeight;
    const cos = Math.cos(this.hood);
    const denominator = 2 * cos * cos * (distance * Math.tan(this.hood) - rise);
    if (denominator <= 0) return this.maxExit * 0.8;
    return Math.sqrt((GRAVITY * distance * distance) / denominator);
  }

  private speedForUp(up: number, speed: number, yaw: number, upPerSpeed: number): number {
    let estimate = speed + up / upPerSpeed;
    for (let i = 0; i < 3; i++) {
      if (!this.fly(estimate, yaw)) break;
      const error = this.hitUp - up;
      if (Math.abs(error) < CONVERGED) break;
      estimate -= error / upPerSpeed;
    }
    return estimate;
  }

  private fly(speed: number, yaw: number): boolean {
    const cosYaw = Math.cos(yaw);
    const sinYaw = Math.sin(yaw);
    const horizontal = speed * Math.cos(this.hood);
    let x = this.pivotX;
    let y = this.pivotY;
    let z = this.exitHeight;
    let vx = horizontal * cosYaw + this.ballVx;
    let vy = horizontal * sinYaw + this.ballVy;
    let vz = speed * Math.sin(this.hood);
    this.spinX = sinYaw;
    this.spinY = -cosYaw;
    let side = this.planeSide(x, y, z);
    for (let t = 0; t < MAX_FLIGHT; t += STEP) {
      this.accel(vx, vy, vz);
      const k1x = this.ax, k1y = this.ay, k1z = this.az;
      this.accel(vx + (k1x * STEP) / 2, vy + (k1y * STEP) / 2, vz + (k1z * STEP) / 2);
      const k2x = this.ax, k2y = this.ay, k2z = this.az;
      this.accel(vx + (k2x * STEP) / 2, vy + (k2y * STEP) / 2, vz + (k2z * STEP) / 2);
      const k3x = this.ax, k3y = this.ay, k3z = this.az;
      this.accel(vx + k3x * STEP, vy + k3y * STEP, vz + k3z * STEP);
      const k4x = this.ax, k4y = this.ay, k4z = this.az;
      const nx = x + STEP * (vx + (STEP / 6) * (k1x + k2x + k3x));
      const ny = y + STEP * (vy + (STEP / 6) * (k1y + k2y + k3y));
      const nz = z + STEP * (vz + (STEP / 6) * (k1z + k2z + k3z));
      const nvx = vx + (STEP / 6) * (k1x + 2 * k2x + 2 * k3x + k4x);
      const nvy = vy + (STEP / 6) * (k1y + 2 * k2y + 2 * k3y + k4y);
      const nvz = vz + (STEP / 6) * (k1z + 2 * k2z + 2 * k3z + k4z);
      const nextSide = this.planeSide(nx, ny, nz);
      if (side > 0 && nextSide <= 0) {
        const f = side / (side - nextSide);
        const hx = x + f * (nx - x) - this.centerX;
        const hy = y + f * (ny - y) - this.centerY;
        const hz = z + f * (nz - z) - OPENING_CENTER_Z;
        const inward = -((vx + f * (nvx - vx)) * this.normalX + (vy + f * (nvy - vy)) * this.normalY + (vz + f * (nvz - vz)) * this.normalZ);
        if (inward <= 0) return false;
        this.hitUp = hx * this.upX + hy * this.upY + hz * this.upZ;
        this.hitSide = hx * this.sideX + hy * this.sideY;
        this.hitTime = t + f * STEP;
        return true;
      }
      if (nz < 0) return false;
      x = nx;
      y = ny;
      z = nz;
      vx = nvx;
      vy = nvy;
      vz = nvz;
      side = nextSide;
    }
    return false;
  }

  private planeSide(x: number, y: number, z: number): number {
    return (x - this.centerX) * this.normalX + (y - this.centerY) * this.normalY + (z - OPENING_CENTER_Z) * this.normalZ;
  }

  private accel(vx: number, vy: number, vz: number): void {
    const speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
    const drag = this.k * this.dragC * speed;
    const lx = this.spinY * vz;
    const ly = -this.spinX * vz;
    const lz = this.spinX * vy - this.spinY * vx;
    const lNorm = Math.sqrt(lx * lx + ly * ly + lz * lz);
    const lift = lNorm > 1e-9 ? (this.k * this.liftC * speed * speed) / lNorm : 0;
    this.ax = -drag * vx + lift * lx;
    this.ay = -drag * vy + lift * ly;
    this.az = -drag * vz + lift * lz - GRAVITY;
  }
}

// ---- Flywheel ----

/** Flywheel top speed in encoder ticks/s (6000 RPM, 28 ticks per turn: 2800, FLYWHEEL_MAX_VELOCITY). */
export function flywheelMaxTicks(cfg: ShooterConfig): number {
  return (cfg.motorRpm / 60) * ENCODER_TICKS_PER_REV;
}

/** The flywheel's speed in ticks/s, stepped toward the speed it's asked for as fast as its motor can change it. */
export class Flywheel {
  private readonly radPerTick = (2 * Math.PI) / ENCODER_TICKS_PER_REV;
  private readonly inertia: number; // kg·m²
  private readonly stall: number;
  private readonly free: number; // rad/s
  private readonly ballEnergy: number; // J per (in/s)² of exit speed
  private readonly voltage: number;

  constructor(cfg: ShooterConfig, voltage = 12.5) {
    this.voltage = voltage;
    this.inertia = cfg.inertia * 1e-4;
    this.stall = cfg.motorStallTorque;
    this.free = (cfg.motorRpm / 60) * 2 * Math.PI;
    // Energy a ball takes: its kinetic energy, the backspin the hood gives it (a hollow ball rolling off the hood
    // spins at about v/r: 2/3 of ½mv² more), and about as much again lost squeezing it, so ≈ 2.5 × ½mv².
    this.ballEnergy = 2.5 * 0.5 * BALL_MASS * METERS_PER_INCH * METERS_PER_INCH;
  }

  /** Speed after `dt` seconds of chasing `target` (both ticks/s). The hub's velocity PID gives full power either way
   *  until it's close, so this is torque-limited: (k − ω/ωfree)·stall speeding up, (k + ω/ωfree)·stall slowing. */
  step(speed: number, target: number, dt: number): number {
    const k = this.voltage / 12;
    let w = speed * this.radPerTick;
    const goal = target * this.radPerTick;
    let left = dt;
    // A few sub-steps: the torque changes with speed.
    const n = Math.max(1, Math.ceil(dt / 0.02));
    for (let i = 0; i < n && left > 0; i++) {
      const h = left / (n - i);
      left -= h;
      if (Math.abs(goal - w) < 1e-9) break;
      const up = goal > w;
      const torque = this.stall * (up ? Math.max(0, k - w / this.free) : k + w / this.free);
      const change = (torque / this.inertia) * h;
      w = up ? Math.min(goal, w + change) : Math.max(goal, w - change);
    }
    return w / this.radPerTick;
  }

  /** Speed right after a ball leaves at `exitSpeed` (in/s), from `speed` (ticks/s). */
  afterBall(speed: number, exitSpeed: number): number {
    const w = speed * this.radPerTick;
    const e = 0.5 * this.inertia * w * w - this.ballEnergy * exitSpeed * exitSpeed;
    return Math.sqrt(Math.max(0, (2 * e) / this.inertia)) / this.radPerTick;
  }

  /** Seconds to get from `speed` to within `tolerance` of `target` (ticks/s). */
  timeTo(speed: number, target: number, tolerance: number): number {
    let s = speed;
    let t = 0;
    while (Math.abs(s - target) > tolerance && t < 5) {
      s = this.step(s, target, 0.01);
      t += 0.01;
    }
    return t;
  }
}

// ---- Where shots work ----

/**
 * How good a standing shot is from each point on the field, every `step` inches: the smaller of the aim margin
 * (angle tolerance − aim accuracy, degrees) and the speed margin (speed tolerance − speed accuracy, percent). ≥ 0 means
 * a shot from there scores despite the robot's errors; NaN means no flywheel speed gets the ball in.
 */
export interface ShotZone {
  cell: Cell;
  step: number;
  nx: number;
  ny: number;
  margin: Float32Array;
  /** Flywheel speed a standing shot from there needs, ticks/s. */
  flywheel: Float32Array;
}

export const ZONE_STEP = 2;

/** Shot margins (degrees / percent) for a solved shot, given how accurate the robot is. */
export function shotMargin(shot: Shot, cfg: ShooterConfig, flywheelLag = 0): number {
  if (!shot.feasible) return NaN;
  const aim = (shot.angleTolerance * 180) / Math.PI - cfg.aimAccuracy;
  const speed = (shot.speedTolerance / shot.exitSpeed) * 100 - cfg.speedAccuracy - (flywheelLag / shot.flywheel) * 100;
  return Math.min(aim, speed);
}

export function shotZone(cfg: ShooterConfig, cell: Cell, step = ZONE_STEP): ShotZone {
  const nx = Math.floor(144 / step) + 1;
  const ny = nx;
  const margin = new Float32Array(nx * ny);
  const flywheel = new Float32Array(nx * ny);
  const solver = new ShotSolver(cfg);
  for (let j = 0; j < ny; j++) {
    solver.reset();
    // Snake through the rows so each solve starts from its neighbour's answer.
    for (let q = 0; q < nx; q++) {
      const i = j % 2 ? nx - 1 - q : q;
      const shot = solver.solve(i * step, j * step, 0, 0, 0, 0, cell);
      margin[j * nx + i] = shotMargin(shot, cfg);
      flywheel[j * nx + i] = shot.flywheel;
    }
  }
  return { cell, step, nx, ny, margin, flywheel };
}

const zones = new Map<string, ShotZone>();

/** The zone for these shooter settings and cell, computed once. */
export function cachedZone(cfg: ShooterConfig, cell: Cell): ShotZone {
  const key = JSON.stringify([cfg, cell]);
  let z = zones.get(key);
  if (!z) {
    if (zones.size > 8) zones.clear();
    z = shotZone(cfg, cell);
    zones.set(key, z);
  }
  return z;
}

/** Bilinear lookup in one of a zone's grids. */
function lookup(zone: ShotZone, grid: Float32Array, p: Vec, nanAs: number): number {
  const fx = Math.max(0, Math.min(zone.nx - 1.001, p.x / zone.step));
  const fy = Math.max(0, Math.min(zone.ny - 1.001, p.y / zone.step));
  const i = Math.floor(fx);
  const j = Math.floor(fy);
  const u = fx - i;
  const w = fy - j;
  const at = (a: number, b: number) => {
    const m = grid[b * zone.nx + a];
    return Number.isNaN(m) ? nanAs : m;
  };
  return (1 - u) * (1 - w) * at(i, j) + u * (1 - w) * at(i + 1, j) + (1 - u) * w * at(i, j + 1) + u * w * at(i + 1, j + 1);
}

/** Flywheel speed a standing shot from p needs, ticks/s. */
export function zoneFlywheel(zone: ShotZone, p: Vec): number {
  return lookup(zone, zone.flywheel, p, 0);
}

/** Zone margin at a point, bilinear between grid points (no shot counts as −10). */
export function zoneMargin(zone: ShotZone, p: Vec): number {
  return lookup(zone, zone.margin, p, -10);
}
