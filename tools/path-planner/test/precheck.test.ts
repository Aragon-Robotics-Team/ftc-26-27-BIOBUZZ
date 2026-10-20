import { describe, expect, it } from 'vitest';
import { optimizeChain } from '../src/core/optimize.ts';
import { precheck } from '../src/core/planner.ts';
import { defaultSettings, type Chain, type ChainPoint } from '../src/core/project.ts';
import { startChain } from './fixtures/garden.ts';

const settings = defaultSettings();
const pt = (x: number, y: number, heading: number, headingTol = 0): ChainPoint => ({ kind: 'fixed', x, y, heading, headingTol });
const path = (points: ChainPoint[], rules: Chain['legs'] = points.slice(1).map(() => ({ rules: [] }))): Chain => ({
  name: 'p', points, legs: rules, endStopped: true, matchStart: false, markers: [], keepOut: [],
});

describe('checks before optimizing', () => {
  it('finds nothing wrong with points that have room', () => {
    expect(precheck(settings, path([pt(24, 30, 90), pt(30, 110, 90)]))).toEqual([]);
  });

  it('flags a point on top of a flower or the hive frame', () => {
    const issues = precheck(settings, path([pt(24, 30, 90), pt(12, 48, 0), pt(50, 72, 90), pt(30, 120, 90)]));
    expect(issues.map((i) => i.point)).toEqual([1, 2]);
    expect(issues[0].message).toMatch(/^Point 2 .*flower \(red wall\)/);
    expect(issues[1].message).toMatch(/^Point 3 hits the hive frame/);
  });

  it('only flags a turnable point when no allowed heading fits', () => {
    // 16 wide x 14 long: in the strafe-only gap beside the hive leg, facing ±y doesn't fit but facing ±x does
    expect(precheck(settings, path([pt(24, 30, 90), pt(63, 56, 90), pt(30, 120, 90)]))).toHaveLength(1);
    expect(precheck(settings, path([pt(24, 30, 90), pt(63, 56, 90, 180), pt(30, 120, 90)]))).toEqual([]);
  });

  it('flags an area with no room anywhere in it, but not one with some room', () => {
    const area = (x: number, hw: number): ChainPoint => ({ kind: 'region', x, y: 72, halfWidth: hw, halfHeight: 4, heading: 90, headingTol: 0 });
    expect(precheck(settings, path([pt(24, 30, 90), area(48.5, 1), pt(30, 120, 90)])).map((i) => i.point)).toEqual([1]);
    expect(precheck(settings, path([pt(24, 30, 90), area(48.5, 20), pt(30, 120, 90)]))).toEqual([]);
  });

  it('flags a heading that clashes with a leg rule', () => {
    const rules: Chain['legs'] = [{ rules: [{ type: 'fixed', heading: 0, tol: 10 }] }];
    const issues = precheck(settings, path([pt(24, 30, 90), pt(30, 110, 90)], rules));
    expect(issues.map((i) => i.point)).toEqual([0, 1]);
    expect(issues[0].message).toBe('Start faces 90°, but leg 1→2 must face 0° ±10°.');
  });

  it('lets a match start touch the wall, and the path still plans within limits', () => {
    const chain = startChain(56, 8.4, 90);
    expect(precheck(settings, chain)).toEqual([]);
    expect(optimizeChain(settings, chain, { maxEvalsPerSeed: 3000, maxSeeds: 1, runsPerRoute: 1 }).feasible).toBe(true);
  }, 60_000);

  it('uses the configured turret range for "turret aims at hive"', () => {
    // At (24, 30) facing 90° (up the field) the hive (59.25, 72) is about 40° to the right: inside ±90.
    const rules: Chain['legs'] = [{ rules: [{ type: 'turret-reach', margin: 10 }] }];
    const behind = path([pt(24, 30, -90), pt(30, 40, -90)], rules); // facing away: the hive is ~140° off the front
    expect(precheck(settings, behind).length).toBeGreaterThan(0);
    expect(precheck(settings, behind)[0].message).toMatch(/turret must reach the hive, which from here needs facing between/);
    const wide = { ...settings, robot: { ...settings.robot, turretMin: -180, turretMax: 180 } };
    expect(precheck(wide, behind)).toEqual([]);
    const lopsided = { ...settings, robot: { ...settings.robot, turretMin: -170, turretMax: 0 } };
    expect(precheck(lopsided, path([pt(24, 30, 90), pt(30, 40, 90)], rules))).toEqual([]); // hive ~40° right: in range
    expect(precheck(lopsided, path([pt(64, 110, 90), pt(64, 100, 90)], rules)).length).toBeGreaterThan(0); // hive to the left: not
  });

  it('keeps to our half unless the path may cross the centerline', () => {
    const across = path([pt(36, 30, 0), pt(100, 30, 0)]);
    expect(precheck(settings, across).map((i) => i.message)).toEqual([expect.stringMatching(/^End .*centerline/)]);
    const allowed = { ...across, allowCrossing: true };
    expect(precheck(settings, allowed)).toEqual([]);
    const plan = optimizeChain(settings, allowed, { maxEvalsPerSeed: 3000, maxSeeds: 1, runsPerRoute: 1 });
    expect(plan.feasible).toBe(true);
    expect(plan.check.clearances.some((c) => c.name === 'Centerline')).toBe(false);
  }, 60_000);
});
