import { describe, expect, it } from 'vitest';
import { cmaes } from '../src/core/cmaes.ts';

describe('CMA-ES', () => {
  it('finds the minimum of a shifted sphere', () => {
    const r = cmaes((x) => x.reduce((s, v, i) => s + (v - i) ** 2, 0), new Float64Array(6), { sigma: 1, seed: 1, maxEvals: 4000 });
    r.x.forEach((v, i) => expect(v).toBeCloseTo(i, 2));
  });

  it('follows the Rosenbrock valley', () => {
    const rosen = (x: Float64Array) => {
      let s = 0;
      for (let i = 0; i < x.length - 1; i++) s += 100 * (x[i + 1] - x[i] * x[i]) ** 2 + (1 - x[i]) ** 2;
      return s;
    };
    const r = cmaes(rosen, new Float64Array(4), { sigma: 0.5, seed: 3, maxEvals: 20000, tolFun: 1e-12, patience: 200 });
    expect(r.f).toBeLessThan(1e-6);
  });

  it('repeats exactly with the same seed', () => {
    const f = (x: Float64Array) => Math.abs(x[0] - 3) + (x[1] + 1) ** 2;
    const a = cmaes(f, new Float64Array(2), { sigma: 1, seed: 42, maxEvals: 500 });
    const b = cmaes(f, new Float64Array(2), { sigma: 1, seed: 42, maxEvals: 500 });
    expect(Array.from(a.x)).toEqual(Array.from(b.x));
  });
});
