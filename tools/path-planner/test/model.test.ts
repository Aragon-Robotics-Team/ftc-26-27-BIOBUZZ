import { describe, expect, it } from 'vitest';
import type { PlannedSegment } from '../src/core/chain.ts';
import { sampleChain } from '../src/core/chain.ts';
import { speedProfile } from '../src/core/model.ts';
import { PLACEHOLDER_MODEL, type ModelConfig } from '../src/core/project.ts';

const model: ModelConfig = { ...PLACEHOLDER_MODEL, vForward: 50, vStrafe: 25, aForward: 50, aStrafe: 50, dForward: 50, dStrafe: 50, stopOverhead: 0 };

/** A straight line along +x, driven at a constant heading. */
function line(length: number, headingDeg: number): PlannedSegment[] {
  const h = (headingDeg * Math.PI) / 180;
  return [
    {
      curve: [
        { x: 0, y: 0 },
        { x: length / 3, y: 0 },
        { x: (2 * length) / 3, y: 0 },
        { x: length, y: 0 },
      ],
      heading: { F: [0, 1], H: [h, h] },
      leg: 0,
    },
  ];
}

/** Accelerate to v, cruise, decelerate to rest: the textbook time. */
function trapezoid(d: number, v: number, a: number): number {
  const ramp = (v * v) / a; // distance to reach v and to stop again
  if (ramp >= d) return 2 * Math.sqrt(d / a);
  return 2 * (v / a) + (d - ramp) / v;
}

describe('speed model', () => {
  it('matches the textbook time for a straight forward move', () => {
    const { samples } = sampleChain(line(100, 0), 0.25);
    const p = speedProfile(samples, model, { endStopped: true });
    expect(p.total).toBeCloseTo(trapezoid(100, 50, 50), 1);
  });

  it('is slower strafing than driving forward', () => {
    const forward = speedProfile(sampleChain(line(100, 0), 0.5).samples, model, { endStopped: true }).total;
    const strafe = speedProfile(sampleChain(line(100, 90), 0.5).samples, model, { endStopped: true }).total;
    expect(strafe).toBeCloseTo(trapezoid(100, 25, 50), 1);
    expect(strafe).toBeGreaterThan(forward);
  });

  it('goes faster with more battery voltage', () => {
    const { samples } = sampleChain(line(100, 0), 0.5);
    const low = speedProfile(samples, model, { endStopped: true, voltage: 11 }).total;
    const high = speedProfile(samples, model, { endStopped: true, voltage: 13.5 }).total;
    expect(high).toBeLessThan(low);
  });

  it('doesn\'t slow down at the end of a chain that keeps moving', () => {
    const { samples } = sampleChain(line(100, 0), 0.5);
    const stopping = speedProfile(samples, model, { endStopped: true });
    const moving = speedProfile(samples, model, { endStopped: false });
    expect(moving.total).toBeLessThan(stopping.total);
    expect(moving.v[moving.v.length - 1]).toBeCloseTo(50);
  });

  it('adds the stop overhead to chains that stop', () => {
    const { samples } = sampleChain(line(50, 0), 0.5);
    const a = speedProfile(samples, model, { endStopped: true }).total;
    const b = speedProfile(samples, { ...model, stopOverhead: 0.3 }, { endStopped: true }).total;
    expect(b - a).toBeCloseTo(0.3);
  });

  it('has to stop to reverse, instead of turning around at full speed', () => {
    const h = 0;
    const out: PlannedSegment = { curve: [{ x: 0, y: 0 }, { x: 10, y: 0 }, { x: 20, y: 0 }, { x: 30, y: 0 }], heading: { F: [0, 1], H: [h, h] }, leg: 0 };
    const back: PlannedSegment = { curve: [{ x: 30, y: 0 }, { x: 20, y: 0 }, { x: 10, y: 0 }, { x: 0, y: 0 }], heading: { F: [0, 1], H: [h, h] }, leg: 0 };
    const { samples } = sampleChain([out, back], 0.25);
    const p = speedProfile(samples, model, { endStopped: true });
    const turn = samples.findIndex((smp) => smp.s >= 30 - 1e-9);
    // the stop is taken halfway to the next sample, so at the turn it is still braking: v² = 2·d·(step / 2)
    expect(p.v[turn]).toBeLessThanOrEqual(Math.sqrt(50 * 0.25) + 1e-9);
    // the same as two separate 30 in moves, each from rest to rest
    expect(p.total).toBeCloseTo(2 * trapezoid(30, 50, 50), 1);
  });

  it('slows for a tight hook even when it falls between samples', () => {
    // A U-turn of about 1 in radius, sampled every 0.5 in
    const hook: PlannedSegment = { curve: [{ x: 0, y: 0 }, { x: 40, y: 0 }, { x: 40, y: 2 }, { x: 0, y: 2 }], heading: { F: [0, 1], H: [0, 0] }, leg: 0 };
    const { samples } = sampleChain([hook], 0.5);
    const p = speedProfile(samples, model, { endStopped: false });
    let worst = 0;
    for (let i = 1; i < samples.length; i++) {
      const v = Math.min(p.v[i], p.v[i - 1]);
      const bend = Math.abs(Math.atan2(Math.sin(samples[i].travel - samples[i - 1].travel), Math.cos(samples[i].travel - samples[i - 1].travel)));
      if (bend > Math.PI / 2) continue; // turns back on itself: the model stops there instead
      worst = Math.max(worst, (v * v * bend) / (samples[i].s - samples[i - 1].s));
    }
    expect(worst).toBeLessThanOrEqual(model.aLateral * 1.05);
  });
});
