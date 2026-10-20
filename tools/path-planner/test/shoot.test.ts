import { describe, expect, it } from 'vitest';
import { sampleChain } from '../src/core/chain.ts';
import { fromJava, toJava } from '../src/core/export.ts';
import { speedProfile } from '../src/core/model.ts';
import { optimizeChain } from '../src/core/optimize.ts';
import { precheck } from '../src/core/planner.ts';
import { defaultSettings, type Chain } from '../src/core/project.ts';
import { DEFAULT_SHOOTER, Flywheel, shotZone, ShotSolver } from '../src/core/shot.ts';

const settings = defaultSettings();
const base = { legs: [{ rules: [] }, { rules: [] }], endStopped: true, matchStart: false, markers: [], keepOut: [] };

/** From the launch spot, stopping somewhere in an area to shoot, then into the garden corner. */
const stopChain: Chain = {
  ...base,
  name: 'launchShot',
  points: [
    { kind: 'fixed', x: 56, y: 20.22, heading: 90, headingTol: 0 },
    { kind: 'region', x: 36, y: 30, halfWidth: 8, halfHeight: 8, heading: 90, headingTol: 90, shoot: { mode: 'stop', cell: 'right', maxSpeed: 20 } },
    { kind: 'fixed', x: 28.4, y: 12, heading: 180, headingTol: 10 },
  ],
};

/** Out of the garden to park, shooting on the way through an area. */
const moveChain: Chain = {
  ...base,
  name: 'gardenToPark',
  points: [
    { kind: 'fixed', x: 28.4, y: 12, heading: 180, headingTol: 10 },
    { kind: 'region', x: 36, y: 34, halfWidth: 10, halfHeight: 10, heading: 90, headingTol: 180, shoot: { mode: 'move', cell: 'right', maxSpeed: 20 } },
    { kind: 'fixed', x: 14, y: 100, heading: 90, headingTol: 0 },
  ],
};

describe('shot solver (TeamCode ShotSolver, ported)', () => {
  const solver = new ShotSolver(DEFAULT_SHOOTER);

  it('finds a shot into the right cell from the audience side, with room for error', () => {
    solver.reset();
    const shot = solver.solve(56, 20, 0, 0, 0, 0, 'right');
    expect(shot.feasible).toBe(true);
    expect(shot.exitSpeed).toBeGreaterThan(150);
    expect(shot.exitSpeed).toBeLessThan(350);
    expect(shot.speedTolerance / shot.exitSpeed).toBeGreaterThan(0.02);
    expect((shot.angleTolerance * 180) / Math.PI).toBeGreaterThan(2);
  });

  it('has no shot right under the hive (too steep for the hood) or from behind the opening', () => {
    solver.reset();
    expect(solver.solve(40, 40, 0, 0, 0, 0, 'right').feasible).toBe(false);
    solver.reset();
    expect(solver.solve(20, 72, 0, 0, 0, 0, 'right').feasible).toBe(false);
  });

  it('needs a faster ball when the robot is driving away from the target', () => {
    solver.reset();
    const still = solver.solve(40, 30, 0, 0, 0, 0, 'right');
    const away = solver.solve(40, 30, 0, 0, -20, 0, 'right'); // moving toward the audience wall, away from the cell
    expect(away.feasible).toBe(true);
    expect(away.exitSpeed).toBeGreaterThan(still.exitSpeed);
  });

  it('allows fewer places to shoot from when the robot is less accurate', () => {
    const count = (aim: number) => Array.from(shotZone({ ...DEFAULT_SHOOTER, aimAccuracy: aim }, 'right', 6).margin).filter((m) => m >= 0).length;
    expect(count(8)).toBeLessThan(count(1.5));
    expect(count(8)).toBeGreaterThan(0);
  });
});

describe('flywheel', () => {
  it('with more inertia, spins up slower and loses less speed per ball', () => {
    const light = new Flywheel({ ...DEFAULT_SHOOTER, inertia: 1 });
    const heavy = new Flywheel({ ...DEFAULT_SHOOTER, inertia: 5 });
    expect(heavy.timeTo(0, 2000, 20)).toBeGreaterThan(light.timeTo(0, 2000, 20));
    expect(2000 - heavy.afterBall(2000, 250)).toBeLessThan(2000 - light.afterBall(2000, 250));
  });

  it('slows down faster than it speeds up (the motor brakes and back-EMF helps)', () => {
    const fw = new Flywheel(DEFAULT_SHOOTER);
    expect(fw.timeTo(2200, 2000, 10)).toBeLessThan(fw.timeTo(2000, 2200, 10));
  });
});

describe('speed cap (Pedro maxPathSpeed)', () => {
  it('holds the cap through the stretch, coasting down to it beforehand', () => {
    const L = 120;
    const seg = { curve: [{ x: 0, y: 0 }, { x: L / 3, y: 0 }, { x: (2 * L) / 3, y: 0 }, { x: L, y: 0 }] as [never, never, never, never], heading: { F: [0, 1], H: [0, 0] }, leg: 0 };
    const { samples } = sampleChain([seg], 0.5);
    const p = speedProfile(samples, settings, { endStopped: true, caps: [{ from: 70, to: 85, vmax: 20 }] });
    samples.forEach((smp, i) => {
      if (smp.s >= 70 && smp.s <= 85) expect(p.v[i]).toBeLessThanOrEqual(20 + 1e-6);
    });
    expect(p.capStarts[0]).toBeLessThan(60); // coasting from top speed takes a while
    expect(p.total).toBeGreaterThan(speedProfile(samples, settings, { endStopped: true }).total);
  });
});

describe('shooting along a path', () => {
  it('stops to shoot as its own Pedro path, and the volley counts in the time', () => {
    const plan = optimizeChain(settings, stopChain, { maxEvalsPerSeed: 4000, maxSeeds: 2 });
    expect(plan.feasible).toBe(true);
    expect(plan.pieces).toEqual([0, expect.any(Number)]);
    const shot = plan.shots![0];
    expect(shot.mode).toBe('stop');
    expect(shot.balls).toHaveLength(DEFAULT_SHOOTER.balls);
    for (const b of shot.balls) expect(b.margin).toBeGreaterThanOrEqual(0);
    expect(plan.time).toBeGreaterThan(DEFAULT_SHOOTER.volley);
    const java = toJava(stopChain, settings, plan);
    expect(java).toContain('static Path launchShot(PoseFactory f)');
    expect(java).toContain('static Path launchShot2(PoseFactory f)');
    expect(java).toContain('follow(follower, launchShot(f)), robot.shoot(), follow(follower, launchShot2(f))');
    const back = fromJava(java)[0].plan;
    expect(back.time).toBeCloseTo(plan.time, 2);
    expect(back.pieces).toEqual(plan.pieces);
  }, 120_000);

  it('shoots on the move on a capped stretch, firing at a marker', () => {
    const plan = optimizeChain(settings, moveChain, { maxEvalsPerSeed: 4000, maxSeeds: 2 });
    expect(plan.feasible).toBe(true);
    const shot = plan.shots![0];
    expect(shot.mode).toBe('move');
    for (const b of shot.balls) expect(b.margin).toBeGreaterThanOrEqual(0);
    expect(plan.segments.some((s) => s.maxSpeed === 20)).toBe(true);
    expect(plan.markers.map((m) => m.name)).toContain('SHOT_2');
    const java = toJava(moveChain, settings, plan);
    expect(java).toContain('PlannedPath.SLOW, 20,');
    expect(java).toContain('passed(p, GARDEN_TO_PARK_SHOT_2), robot.shoot()');
    const back = fromJava(java)[0].plan;
    expect(back.segments.map((s) => s.maxSpeed)).toEqual(plan.segments.map((s) => s.maxSpeed));
    expect(back.time).toBeCloseTo(plan.time, 2);
  }, 120_000);

  it('warns before optimizing when a shooting point has no shot', () => {
    const hopeless: Chain = { ...stopChain, points: stopChain.points.map((p, i) => (i === 1 ? { ...p, x: 20, y: 72 } : p)) };
    expect(precheck(settings, hopeless).some((f) => f.point === 1 && /can't shoot/.test(f.message))).toBe(true);
    expect(precheck(settings, stopChain).filter((f) => f.point === 1)).toEqual([]);
  });
});
