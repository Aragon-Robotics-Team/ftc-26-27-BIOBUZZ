import { describe, expect, it } from 'vitest';
import { sampleChain } from '../src/core/chain.ts';
import { compareRun, drift, extractRuns, fitModel, parseLog, syntheticLog } from '../src/core/logs.ts';
import { optimizeChain, type Plan } from '../src/core/optimize.ts';
import { defaultSettings, SAMPLE_MODEL, type Settings } from '../src/core/project.ts';
import { garden } from './fixtures/garden.ts';

// The "real" robot: clearly slower than the sample numbers, especially strafing, and with less grip.
const sample: Settings = defaultSettings();
const truth: Settings = { ...sample, model: { ...SAMPLE_MODEL, driveEfficiency: 0.75, strafeEfficiency: 0.65, grip: 0.35, mass: 13 } };

let plans: Plan[] | null = null;
function getPlans(): Plan[] {
  if (!plans) {
    const { settings, chains } = garden();
    plans = chains.map((c) => optimizeChain(settings, c, { maxEvalsPerSeed: 1500, maxSeeds: 1 }));
  }
  return plans;
}

function logFor(plan: Plan, seed: number, voltage = 12.5) {
  const samples = sampleChain(plan.segments, 0.5).samples;
  return parseLog(`${plan.chain}-${seed}.csv`, syntheticLog(`${plan.chain}#${plan.geometryHash}`, samples, truth, { endStopped: true, noise: 0.05, seed, voltage, settle: 0.3 }));
}

describe('logs', () => {
  it('reads runs back out of a log', () => {
    const plan = getPlans()[0];
    const runs = extractRuns(logFor(plan, 1), getPlans());
    expect(runs).toHaveLength(1);
    expect(runs[0].chain).toBe(plan.chain);
    expect(runs[0].matched).toBe(true);
    expect(runs[0].endStopped).toBe(true);
    expect(runs[0].settle).toBeCloseTo(0.3, 1);
  }, 60_000);

  it('flips blue logs into the red frame', () => {
    const text = '# alliance=BLUE\nt,voltage,x,y,heading,mode,busy,path,segment,tparam\n0,12.5,100,130,0,IDLE,0,,-1,0\n';
    const log = parseLog('blue.csv', text);
    expect(log.rows[0].x).toBeCloseTo(44);
    expect(log.rows[0].y).toBeCloseTo(14);
    expect(Math.abs(log.rows[0].heading)).toBeCloseTo(Math.PI);
  });

  it('predicts a run driven at the model\'s own speeds', () => {
    const plan = getPlans()[0];
    const [run] = extractRuns(logFor(plan, 2), getPlans());
    expect(Math.abs(compareRun(run, truth).error)).toBeLessThan(0.02);
  }, 60_000);

  it('still gets close when the plan it drove is gone, by rebuilding the path from the poses', () => {
    const plan = getPlans()[0];
    const [run] = extractRuns(logFor(plan, 3), []);
    expect(run.matched).toBe(false);
    expect(Math.abs(compareRun(run, truth).error)).toBeLessThan(0.1);
  }, 60_000);

  it('flags the placeholder model as out of date, and fitting fixes it', () => {
    const runs = getPlans().flatMap((plan, i) => [1, 2].flatMap((k) => extractRuns(logFor(plan, 10 * i + k, 12 + k * 0.3), getPlans())));
    const before = drift(runs.map((r) => compareRun(r, sample)));
    expect(before.flagged).toBe(true);
    const fit = fitModel(runs, sample, { maxEvals: 2500 });
    expect(fit.after).toBeLessThan(fit.before);
    const after = drift(runs.map((r) => compareRun(r, { ...sample, model: fit.model })));
    expect(after.flagged).toBe(false);
    expect(fit.model.stopOverhead).toBeCloseTo(0.3, 1);
    expect(fit.model.source).toMatch(/fit from 4 runs/);
  }, 120_000);
});
