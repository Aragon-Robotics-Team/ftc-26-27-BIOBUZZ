import { describe, expect, it } from 'vitest';
import type { PlannedSegment } from '../src/core/chain.ts';
import { sampleChain } from '../src/core/chain.ts';
import { drivetrain } from '../src/core/drivetrain.ts';
import { speedProfile } from '../src/core/model.ts';
import { defaultSettings, SAMPLE_MODEL, type Settings } from '../src/core/project.ts';

const settings: Settings = { ...defaultSettings(), model: { ...SAMPLE_MODEL, stopOverhead: 0 } };
const deg = (d: number) => (d * Math.PI) / 180;

/** A straight line along +x, with the heading going from h0 to h1 (degrees). */
function line(length: number, h0: number, h1 = h0): PlannedSegment[] {
  return [
    {
      curve: [{ x: 0, y: 0 }, { x: length / 3, y: 0 }, { x: (2 * length) / 3, y: 0 }, { x: length, y: 0 }],
      heading: { F: [0, 1], H: [deg(h0), deg(h1)] },
      leg: 0,
    },
  ];
}

/** A straight run of `straight` inches, then a quarter turn of radius r. */
function hook(straight: number, r: number): PlannedSegment[] {
  const k = 0.5523 * r;
  return [
    { curve: [{ x: 0, y: 0 }, { x: straight / 3, y: 0 }, { x: (2 * straight) / 3, y: 0 }, { x: straight, y: 0 }], heading: { F: [0, 1], H: [0, 0] }, leg: 0 },
    { curve: [{ x: straight, y: 0 }, { x: straight + k, y: 0 }, { x: straight + r, y: r - k }, { x: straight + r, y: r }], heading: { F: [0, 1], H: [0, 0] }, leg: 0 },
  ];
}

const profile = (segs: PlannedSegment[], opts: { endStopped?: boolean; voltage?: number } = {}, s = settings) =>
  speedProfile(sampleChain(segs, 0.5).samples, s, { endStopped: opts.endStopped ?? true, voltage: opts.voltage ?? s.model.nominalVoltage });

describe('drivetrain', () => {
  it('gets its top speeds from the motor and wheel', () => {
    const d = drivetrain(settings.model, settings.robot);
    // goBILDA 312 RPM on 104 mm wheels: 312 / 60 · π · 4.094 ≈ 66.9 in/s free; 90 % of that forward, 80 % of forward sideways
    expect(d.vForward).toBeCloseTo(66.88 * 0.9, 1);
    expect(d.vStrafe).toBeCloseTo(d.vForward * 0.8, 6);
    expect(d.traction).toBeCloseTo(0.5 * 386.09, 1);
  });
});

describe('speed model', () => {
  it("speeds up to top speed and no further, at the motors' rated voltage", () => {
    const p = profile(line(120, 0), { endStopped: false });
    const top = drivetrain(settings.model, settings.robot).vForward;
    expect(Math.max(...p.v)).toBeLessThanOrEqual(top + 1e-6);
    expect(p.v[p.v.length - 1]).toBeGreaterThan(top * 0.98);
  });

  it('speeds up harder at low speed than near top speed (motor torque falls with speed)', () => {
    const p = profile(line(120, 0), { endStopped: false });
    const accelAt = (i: number) => (p.v[i + 1] ** 2 - p.v[i] ** 2) / (2 * 0.5);
    const slow = p.v.findIndex((v) => v > 10);
    const fast = p.v.findIndex((v) => v > 55);
    expect(accelAt(fast)).toBeLessThan(accelAt(slow) * 0.6);
  });

  it('is slower strafing than driving forward', () => {
    expect(profile(line(96, 90)).total).toBeGreaterThan(profile(line(96, 0)).total + 0.1);
  });

  it('goes faster with more battery voltage', () => {
    expect(profile(line(96, 0), { voltage: 13.5 }).total).toBeLessThan(profile(line(96, 0), { voltage: 11.5 }).total);
  });

  it('is slower when it has to turn on the way', () => {
    expect(profile(line(96, 0, 180)).total).toBeGreaterThan(profile(line(96, 0)).total + 0.3);
  });

  it("a heavier robot speeds up more slowly once grip isn't the limit", () => {
    const heavy = { ...settings, model: { ...settings.model, mass: 30, grip: 2 } };
    const light = { ...settings, model: { ...settings.model, mass: 8, grip: 2 } };
    expect(profile(line(96, 0), {}, heavy).total).toBeGreaterThan(profile(line(96, 0), {}, light).total);
  });

  it('drifts wide on a tight curve taken from full speed, as Pedro does, and not on a gentle one', () => {
    expect(profile(hook(60, 40)).worstDrift).toBeLessThan(0.5);
    expect(profile(hook(60, 10)).worstDrift).toBeGreaterThan(1);
    // from a standstill there's time to get round
    expect(profile(hook(4, 10)).worstDrift).toBeLessThan(0.5);
  });

  it('has to stop to reverse, instead of turning around at full speed', () => {
    const out: PlannedSegment = { curve: [{ x: 0, y: 0 }, { x: 10, y: 0 }, { x: 20, y: 0 }, { x: 30, y: 0 }], heading: { F: [0, 1], H: [0, 0] }, leg: 0 };
    const back: PlannedSegment = { curve: [{ x: 30, y: 0 }, { x: 20, y: 0 }, { x: 10, y: 0 }, { x: 0, y: 0 }], heading: { F: [0, 1], H: [0, 0] }, leg: 0 };
    const { samples } = sampleChain([out, back], 0.25);
    const p = speedProfile(samples, settings, { endStopped: true, voltage: 12 });
    const turn = samples.findIndex((smp) => smp.s >= 30 - 1e-9);
    expect(p.v[turn]).toBeLessThan(15);
    // about the same as two separate 30 in moves, each from rest to rest
    const single = profile(line(30, 0)).total;
    expect(p.total).toBeGreaterThan(2 * single - 0.1);
    expect(p.total).toBeLessThan(2 * single + 0.15);
  });

  it('adds the stop overhead to paths that stop', () => {
    const withSettle = { ...settings, model: { ...settings.model, stopOverhead: 0.3 } };
    expect(profile(line(50, 0), {}, withSettle).total - profile(line(50, 0)).total).toBeCloseTo(0.3);
  });
});
