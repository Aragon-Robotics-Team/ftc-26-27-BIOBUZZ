import { describe, expect, it } from 'vitest';
import { sampleChain } from '../src/core/chain.ts';
import { evaluate, optimizeChain } from '../src/core/optimize.ts';
import { marginsFor } from '../src/core/project.ts';
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
    expect(last.curve[3]).toEqual({ x: 12, y: 12 });
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

  it("doesn't count a path that turns back on itself as fitting: Pedro would overshoot", () => {
    const settings = defaultSettings();
    const chain: Chain = { ...underHive, name: 'outAndBack', points: [{ kind: 'fixed', x: 30, y: 30, heading: 0, headingTol: 0 }, { kind: 'fixed', x: 30, y: 60, heading: 0, headingTol: 0 }] };
    const out = { curve: [{ x: 30, y: 30 }, { x: 30, y: 45 }, { x: 30, y: 55 }, { x: 30, y: 70 }] as const, heading: { F: [0, 1], H: [0, 0] }, leg: 0 };
    const back = { curve: [{ x: 30, y: 70 }, { x: 30, y: 66 }, { x: 30, y: 63 }, { x: 30, y: 60 }] as const, heading: { F: [0, 1], H: [0, 0] }, leg: 0 };
    const ev = evaluate([out, back].map((s) => ({ ...s, curve: [...s.curve] })), settings, chain, marginsFor(settings, chain));
    expect(ev.reversals).toBe(1);
    expect(ev.fits).toBe(false);
  });
});
