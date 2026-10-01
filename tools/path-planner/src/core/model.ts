/**
 * Predicted speed along a chain and the time it takes.
 *
 * Mecanum limits, with α the travel direction relative to the heading and ω the turn rate:
 *   |v·cos α|/vForward + |v·sin α|/vStrafe + |ω|/omegaMax ≤ 1      (the wheels share one speed budget)
 *   v²·|κ| ≤ aLateral                                               (sideways grip on curves)
 *   κ is the sharper of the curvature and the turn between neighbouring samples; turning back by more than 90°
 *   within one sample step means the robot has to stop there (taken as halfway between the two samples)
 *   speeding up / slowing down: direction-dependent the same way
 * Speeds and accelerations scale with battery voltage / nominal voltage.
 */
import type { Sample } from './chain.ts';
import { wrap } from './geom.ts';
import type { ModelConfig } from './project.ts';

export type Limit = 'direction' | 'turn' | 'curvature' | 'accel' | 'decel' | 'stop';

export interface Profile {
  /** Speed at each sample, in/s. */
  v: Float64Array;
  /** Time at each sample, s. */
  time: Float64Array;
  /** What holds the speed down at each sample. */
  limit: Limit[];
  /** Total time, including the stop overhead if the chain ends stopped. */
  total: number;
}

function mix(cosA: number, sinA: number, forward: number, strafe: number): number {
  return 1 / (Math.abs(cosA) / forward + Math.abs(sinA) / strafe);
}

export function speedProfile(samples: Sample[], model: ModelConfig, opts: { endStopped: boolean; voltage?: number }): Profile {
  const n = samples.length;
  const k = (opts.voltage ?? model.nominalVoltage) / model.nominalVoltage;
  const vF = model.vForward * k;
  const vS = model.vStrafe * k;
  const omega = model.omegaMax * k;
  const v = new Float64Array(n);
  const limit: Limit[] = new Array(n);
  const acc = new Float64Array(n);
  const dec = new Float64Array(n);
  // Where the path turns back on itself between two samples (a cusp): the robot has to stop there. The stop is taken
  // to be halfway between the two samples.
  const reversesAfter = new Uint8Array(n);
  for (let i = 0; i + 1 < n; i++) if (Math.abs(wrap(samples[i + 1].travel - samples[i].travel)) > Math.PI / 2) reversesAfter[i] = 1;
  for (let i = 0; i < n; i++) {
    const a = samples[i].travel - samples[i].h;
    const c = Math.cos(a);
    const s = Math.sin(a);
    acc[i] = mix(c, s, model.aForward * k, model.aStrafe * k);
    dec[i] = mix(c, s, model.dForward * k, model.dStrafe * k);
    // turn rate per inch travelled, from neighbouring samples
    const i0 = Math.max(0, i - 1);
    const i1 = Math.min(n - 1, i + 1);
    const ds = samples[i1].s - samples[i0].s;
    const dh = ds > 1e-9 ? Math.abs(samples[i1].h - samples[i0].h) / ds : 0;
    const translate = Math.abs(c) / vF + Math.abs(s) / vS;
    const turn = dh / omega;
    let best = 1 / (translate + turn);
    limit[i] = turn > translate * 0.5 ? 'turn' : 'direction';
    // How sharply the direction of travel bends here: the curvature at the sample, or the turn between it and a
    // neighbour if that's sharper. A hook tighter than the sample spacing would otherwise slip between samples and
    // let the robot change direction at full speed.
    let kappa = Math.abs(samples[i].kappa);
    for (const j of [i - 1, i + 1]) {
      if (j < 0 || j >= n || reversesAfter[Math.min(i, j)]) continue;
      const step = Math.abs(samples[j].s - samples[i].s);
      if (step > 1e-9) kappa = Math.max(kappa, Math.abs(wrap(samples[j].travel - samples[i].travel)) / step);
    }
    if (kappa > 1e-9) {
      const vc = Math.sqrt((model.aLateral * k) / kappa);
      if (vc < best) {
        best = vc;
        limit[i] = 'curvature';
      }
    }
    // Next to a cusp: able to stop by it, or only as fast as it can get from a standstill there.
    const capAt = (step: number, rate: number) => {
      const vr = Math.sqrt(rate * step); // v² = 2·rate·(step / 2)
      if (vr < best) {
        best = vr;
        limit[i] = 'curvature';
      }
    };
    if (i + 1 < n && reversesAfter[i]) capAt(samples[i + 1].s - samples[i].s, dec[i]);
    if (i > 0 && reversesAfter[i - 1]) capAt(samples[i].s - samples[i - 1].s, acc[i]);
    v[i] = best;
  }
  // Starts from rest.
  v[0] = 0;
  limit[0] = 'accel';
  for (let i = 1; i < n; i++) {
    const ds = samples[i].s - samples[i - 1].s;
    const reach = Math.sqrt(v[i - 1] * v[i - 1] + 2 * acc[i - 1] * ds);
    if (reach < v[i]) {
      v[i] = reach;
      limit[i] = 'accel';
    }
  }
  if (opts.endStopped) {
    v[n - 1] = 0;
    limit[n - 1] = 'stop';
  }
  for (let i = n - 2; i >= 0; i--) {
    const ds = samples[i + 1].s - samples[i].s;
    const reach = Math.sqrt(v[i + 1] * v[i + 1] + 2 * dec[i] * ds);
    if (reach < v[i]) {
      v[i] = reach;
      limit[i] = 'decel';
    }
  }
  const time = new Float64Array(n);
  for (let i = 1; i < n; i++) {
    const ds = samples[i].s - samples[i - 1].s;
    const vm = v[i] + v[i - 1];
    // Covering d inches between a standstill and speed vEnd (or rest to rest, if vEnd is 0).
    const fromRest = (d: number, vEnd: number, rate: number) => (vEnd > 1e-9 ? (2 * d) / vEnd : 2 * Math.sqrt(d / Math.max(1e-6, rate)));
    let dt: number;
    if (ds <= 0) dt = 0;
    else if (reversesAfter[i - 1]) dt = fromRest(ds / 2, v[i - 1], dec[i - 1]) + fromRest(ds / 2, v[i], acc[i]); // stop in the middle
    else if (vm > 1e-9) dt = (2 * ds) / vm;
    else dt = fromRest(ds, 0, Math.min(acc[i - 1], dec[i - 1]));
    time[i] = time[i - 1] + dt;
  }
  const total = time[n - 1] + (opts.endStopped ? model.stopOverhead : 0);
  return { v, time, limit, total };
}
