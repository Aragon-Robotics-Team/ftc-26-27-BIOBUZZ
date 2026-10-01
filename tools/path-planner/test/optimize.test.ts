import { describe, expect, it } from 'vitest';
import { sampleChain } from '../src/core/chain.ts';
import { optimizeChain } from '../src/core/optimize.ts';
import { defaultSettings, type Chain } from '../src/core/project.ts';
import { garden } from './fixtures/garden.ts';

/** Straight through the gap between the red hive frame's legs and the centerline. */
const underHive: Chain = {
  name: 'underHive',
  points: [
    { kind: 'fixed', x: 62, y: 36, heading: 90, headingTol: 0 },
    { kind: 'fixed', x: 62, y: 118, heading: 90, headingTol: 0 },
  ],
  legs: [{ rules: [] }],
  endStopped: true,
  matchStart: false,
  markers: [],
  keepOut: [],
};

describe('optimizer', () => {
  it('plans the garden cycle within every limit, with the intake leading into the garden', () => {
    const { settings, chains } = garden();
    const plan = optimizeChain(settings, chains[0], { maxEvalsPerSeed: 4000, maxSeeds: 2 });
    expect(plan.feasible).toBe(true);
    expect(plan.check.rules).toEqual([]);
    for (const c of plan.check.clearances) expect(c.gap).toBeGreaterThanOrEqual(c.margin - 1e-3);
    expect(plan.time).toBeGreaterThan(1);
    expect(plan.time).toBeLessThan(6);
    // ends exactly on the poses it was given
    const last = plan.segments[plan.segments.length - 1];
    expect(last.curve[3]).toEqual({ x: 56, y: 20.22 });
    expect(plan.markers).toHaveLength(1);
  }, 60_000);

  it('finds that the gap under the hive only fits sideways, and strafes through it', () => {
    const plan = optimizeChain(defaultSettings(), underHive);
    expect(plan.feasible).toBe(true);
    const mid = sampleChain(plan.segments, 2).samples.filter((s) => s.p.y > 55 && s.p.y < 88);
    for (const s of mid) expect(Math.abs(Math.sin(s.h))).toBeLessThan(0.35); // facing ±x, not up the field
  }, 120_000);

  it('gives the same plan every time', () => {
    const { settings, chains } = garden();
    const a = optimizeChain(settings, chains[1], { maxEvalsPerSeed: 1500 });
    const b = optimizeChain(settings, chains[1], { maxEvalsPerSeed: 1500 });
    expect(a.segments).toEqual(b.segments);
  }, 60_000);
});
