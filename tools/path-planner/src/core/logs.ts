/**
 * Robot path logs: parse the CSV the robot writes, cut it into runs, compare the speed model with what the robot did,
 * and fit the model. The model is applied to the path the robot actually drove (from the logged poses), so a log stays
 * useful after the plans change.
 *
 * Format (one file per Auto run, written by PathLog.java):
 *   # path-planner log v1
 *   # routine=CYCLE_GARDEN_PARK
 *   # alliance=RED
 *   t,voltage,x,y,heading,mode,busy,path,segment,tparam
 *   0.000,12.84,56.0,8.4,1.5708,IDLE,0,,-1,0
 * t: seconds since start; x, y: inches; heading: radians (field frame, as driven, not flipped for blue);
 * mode: FOLLOW / HOLD / MANUAL / IDLE; busy: Follower.isBusy(); path: PlannedPaths.nameOf(currentPath), "chain#geometry",
 * or empty;
 * segment: Follower.pathIndex(); tparam: Follower.parametricCompletion().
 */
import { sampleChain, type Sample } from './chain.ts';
import { cmaes } from './cmaes.ts';
import { wrap } from './geom.ts';
import type { Plan } from './optimize.ts';
import { speedProfile, type DriveSettings, type SpeedCap } from './model.ts';
import type { ModelConfig } from './project.ts';

export const LOG_COLUMNS = ['t', 'voltage', 'x', 'y', 'heading', 'mode', 'busy', 'path', 'segment', 'tparam'] as const;

export interface LogRow {
  t: number;
  voltage: number;
  x: number;
  y: number;
  heading: number;
  mode: string;
  busy: boolean;
  path: string;
  segment: number;
  tparam: number;
}

export interface PathLog {
  name: string;
  meta: Record<string, string>;
  rows: LogRow[];
}

export function parseLog(name: string, text: string): PathLog {
  const meta: Record<string, string> = {};
  const rows: LogRow[] = [];
  let header: string[] | null = null;
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line) continue;
    if (line.startsWith('#')) {
      const m = /^#\s*([A-Za-z_]+)\s*=\s*(.*)$/.exec(line);
      if (m) meta[m[1]] = m[2];
      continue;
    }
    const cells = line.split(',');
    if (!header) {
      header = cells;
      const missing = LOG_COLUMNS.filter((c) => !header!.includes(c));
      if (missing.length) throw new Error(`${name}: missing columns ${missing.join(', ')}`);
      continue;
    }
    const get = (c: string) => cells[header!.indexOf(c)] ?? '';
    rows.push({
      t: Number(get('t')),
      voltage: Number(get('voltage')),
      x: Number(get('x')),
      y: Number(get('y')),
      heading: Number(get('heading')),
      mode: get('mode'),
      busy: get('busy') === '1' || get('busy') === 'true',
      path: get('path'),
      segment: Number(get('segment')),
      tparam: Number(get('tparam')),
    });
  }
  if (!header) throw new Error(`${name}: no data`);
  // Blue runs are flipped to red, the frame the planner works in.
  if ((meta.alliance ?? 'RED').toUpperCase() === 'BLUE') {
    for (const r of rows) {
      r.x = 144 - r.x;
      r.y = 144 - r.y;
      r.heading = wrap(r.heading + Math.PI);
    }
  }
  return { name, meta, rows };
}

/** One follow of one planned chain. */
export interface Run {
  log: string;
  chain: string;
  /** True when the run was matched to the exact plan it drove (by geometry hash); otherwise the path is rebuilt from
   *  the logged poses, which rounds off sharp reversals a little. */
  matched: boolean;
  /** Measured time from the start of the follow to the parametric end, s. */
  duration: number;
  /** Time from the parametric end until the follower stopped being busy, if it stopped. */
  settle: number | null;
  endStopped: boolean;
  voltage: number;
  /** The driven path, resampled by arc length, with the measured speed at each sample. */
  samples: Sample[];
  measuredSpeed: number[];
  /** Speed caps on the planned path it drove (shooting on the move). */
  caps?: SpeedCap[];
}

/** Moving average over a window of samples, keeping the ends. */
function smooth(values: number[], half: number): number[] {
  return values.map((_, i) => {
    let s = 0;
    let n = 0;
    for (let k = Math.max(0, i - half); k <= Math.min(values.length - 1, i + half); k++) {
      s += values[k];
      n++;
    }
    return s / n;
  });
}

export function extractRuns(log: PathLog, plans: Plan[] = [], ds = 0.5): Run[] {
  const runs: Run[] = [];
  const rows = log.rows;
  let i = 0;
  while (i < rows.length) {
    if (rows[i].mode !== 'FOLLOW' || !rows[i].path) {
      i++;
      continue;
    }
    const chain = rows[i].path;
    let j = i;
    while (j + 1 < rows.length && rows[j + 1].mode === 'FOLLOW' && rows[j + 1].path === chain) j++;
    const part = rows.slice(i, j + 1);
    const next = rows[j + 1];
    const endStopped = !next || next.mode === 'HOLD' || next.mode === 'IDLE';
    let settle: number | null = null;
    if (next?.mode === 'HOLD') {
      let k = j + 1;
      while (k < rows.length && rows[k].mode === 'HOLD' && rows[k].busy) k++;
      if (k < rows.length && rows[k].mode === 'HOLD') settle = rows[k].t - rows[j].t;
    }
    // "name#geometry", or "name.2#geometry" for the second Pedro path of one that stops to shoot.
    const [id, geometry] = chain.split('#');
    const piece = /\.(\d+)$/.exec(id);
    const name = piece ? id.slice(0, piece.index) : id;
    const plan = plans.find((p) => p.chain === name && p.geometryHash === geometry);
    const run = plan ? matchedRun(log.name, plan, piece ? Number(piece[1]) - 1 : 0, part, endStopped, settle, ds) : buildRun(log.name, id, part, endStopped, settle, ds);
    if (run) runs.push(run);
    i = j + 1;
  }
  return runs;
}

/** Distance driven at each logged row. */
function driven(part: LogRow[]): number[] {
  const cum = [0];
  for (let k = 1; k < part.length; k++) cum.push(cum[k - 1] + Math.hypot(part[k].x - part[k - 1].x, part[k].y - part[k - 1].y));
  return cum;
}

/** Time at which `dist` inches had been driven. */
function timeAt(part: LogRow[], cum: number[], dist: number): number {
  let lo = 0;
  let hi = cum.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (cum[mid] <= dist) lo = mid;
    else hi = mid;
  }
  const span = cum[hi] - cum[lo];
  const f = span > 0 ? Math.min(Math.max((dist - cum[lo]) / span, 0), 1) : 0;
  return part[lo].t + (part[hi].t - part[lo].t) * f;
}

function medianVoltage(part: LogRow[]): number {
  const v = part.map((r) => r.voltage).filter((x) => x > 5).sort((a, b) => a - b);
  return v.length ? v[v.length >> 1] : NaN;
}

/** A run of a known plan: the planned geometry, with the measured speed mapped onto it by distance driven. */
function matchedRun(logName: string, plan: Plan, piece: number, part: LogRow[], endStopped: boolean, settle: number | null, ds: number): Run | null {
  if (part.length < 5) return null;
  const starts = plan.pieces ?? [0];
  if (piece >= starts.length) return null;
  const segments = plan.segments.slice(starts[piece], starts[piece + 1] ?? plan.segments.length);
  const sampled = sampleChain(segments, ds);
  const samples = sampled.samples;
  const caps: SpeedCap[] = [];
  segments.forEach((seg, j) => {
    if (seg.maxSpeed !== undefined) caps.push({ from: sampled.segStart[j], to: sampled.segStart[j + 1], vmax: seg.maxSpeed, fixedStart: true });
  });
  const cum = driven(part);
  const total = cum[cum.length - 1];
  const planLength = samples[samples.length - 1].s;
  if (total < 3 || planLength <= 0) return null;
  const times = samples.map((smp) => timeAt(part, cum, (smp.s / planLength) * total));
  const measuredSpeed = samples.map((_, i) => {
    const a = Math.max(0, i - 2);
    const b = Math.min(samples.length - 1, i + 2);
    const dt = times[b] - times[a];
    return dt > 1e-6 ? (samples[b].s - samples[a].s) / dt : 0;
  });
  return {
    log: logName,
    chain: plan.chain,
    matched: true,
    duration: part[part.length - 1].t - part[0].t,
    settle,
    endStopped,
    voltage: medianVoltage(part),
    samples,
    measuredSpeed,
    caps,
  };
}

function buildRun(logName: string, chain: string, part: LogRow[], endStopped: boolean, settle: number | null, ds: number): Run | null {
  if (part.length < 5) return null;
  // Cumulative distance driven, then resample every ds inches.
  const cum = driven(part);
  const total = cum[cum.length - 1];
  if (total < 3) return null;
  const n = Math.max(4, Math.round(total / ds));
  const xs: number[] = [];
  const ys: number[] = [];
  const hs: number[] = [];
  const ts: number[] = [];
  let k = 0;
  let unwrapped = part[0].heading;
  const hUnwrapped = part.map((r, idx) => (idx === 0 ? unwrapped : (unwrapped += wrap(r.heading - part[idx - 1].heading))));
  for (let q = 0; q <= n; q++) {
    const s = (total * q) / n;
    while (k < part.length - 2 && cum[k + 1] < s) k++;
    const span = cum[k + 1] - cum[k];
    const f = span > 0 ? Math.min(Math.max((s - cum[k]) / span, 0), 1) : 0;
    xs.push(part[k].x + (part[k + 1].x - part[k].x) * f);
    ys.push(part[k].y + (part[k + 1].y - part[k].y) * f);
    hs.push(hUnwrapped[k] + (hUnwrapped[k + 1] - hUnwrapped[k]) * f);
    ts.push(part[k].t + (part[k + 1].t - part[k].t) * f);
  }
  const sx = smooth(xs, 2);
  const sy = smooth(ys, 2);
  const samples: Sample[] = [];
  const measuredSpeed: number[] = [];
  for (let q = 0; q <= n; q++) {
    const a = Math.max(0, q - 1);
    const b = Math.min(n, q + 1);
    const dx = sx[b] - sx[a];
    const dy = sy[b] - sy[a];
    const travel = Math.atan2(dy, dx);
    // curvature from the turn of the travel direction between neighbours
    let kappa = 0;
    if (q > 0 && q < n) {
      const t1 = Math.atan2(sy[q] - sy[q - 1], sx[q] - sx[q - 1]);
      const t2 = Math.atan2(sy[q + 1] - sy[q], sx[q + 1] - sx[q]);
      kappa = wrap(t2 - t1) / ((total / n) * 1);
    }
    samples.push({ seg: 0, leg: 0, t: q / n, s: (total * q) / n, p: { x: xs[q], y: ys[q] }, travel, kappa, h: hs[q] });
    const dt = ts[b] - ts[a];
    measuredSpeed.push(dt > 1e-6 ? ((b - a) * total) / n / dt : 0);
  }
  const ks = smooth(samples.map((s) => s.kappa), 2);
  samples.forEach((s, q) => (s.kappa = ks[q]));
  // Smoothing rounds off a reversal (driving in and backing out), which the robot can only do by stopping.
  for (let q = 1; q < samples.length; q++) {
    if (Math.abs(wrap(samples[q].travel - samples[q - 1].travel)) > 1.5) {
      samples[q].kappa = 10;
      samples[q - 1].kappa = 10;
    }
  }
  return {
    log: logName,
    chain,
    matched: false,
    duration: part[part.length - 1].t - part[0].t,
    settle,
    endStopped,
    voltage: medianVoltage(part),
    samples,
    measuredSpeed,
  };
}

export interface RunComparison {
  run: Run;
  predicted: number;
  /** (predicted − measured) / measured. */
  error: number;
  /** Predicted speed at each sample, for plotting against run.measuredSpeed. */
  predictedSpeed: number[];
}

export function compareRun(run: Run, settings: DriveSettings): RunComparison {
  // The follow ends at the parametric end, before the settle, so leave the stop overhead out here.
  const profile = speedProfile(run.samples, { ...settings, model: { ...settings.model, stopOverhead: 0 } }, {
    endStopped: run.endStopped,
    voltage: Number.isFinite(run.voltage) ? run.voltage : undefined,
    caps: run.caps,
  });
  return {
    run,
    predicted: profile.total,
    error: (profile.total - run.duration) / run.duration,
    predictedSpeed: Array.from(profile.v),
  };
}

export interface Drift {
  flagged: boolean;
  /** Median |error| over the runs considered. */
  medianError: number;
  runs: number;
  message: string;
}

/** Flags the model when the latest runs (up to 5) disagree with it by more than `limit` in the median. */
export function drift(comparisons: RunComparison[], limit = 0.05): Drift {
  const recent = comparisons.slice(-5);
  if (recent.length < 3) {
    return { flagged: false, medianError: NaN, runs: recent.length, message: 'Not enough runs to judge yet (needs 3).' };
  }
  const errs = recent.map((c) => Math.abs(c.error)).sort((a, b) => a - b);
  const median = errs[errs.length >> 1];
  const flagged = median > limit;
  return {
    flagged,
    medianError: median,
    runs: recent.length,
    message: flagged
      ? `Model may be out of date: the last ${recent.length} runs are off by ${(median * 100).toFixed(1)} % in the median. Refit after checking the robot hasn't changed.`
      : `Model matches: the last ${recent.length} runs are within ${(median * 100).toFixed(1)} % in the median.`,
  };
}

/** The drivetrain numbers logs can pin down; motor specs and sizes come from the spec sheets. */
const FIT_KEYS = ['driveEfficiency', 'strafeEfficiency', 'grip', 'mass', 'coastForward', 'coastStrafe'] as const;

export interface FitResult {
  model: ModelConfig;
  before: number;
  after: number;
  runs: number;
}

/**
 * Fits the drivetrain's unknowns (efficiencies, grip, mass, coasting) to logged runs: least squares on speed along each run plus total time, in log space so every
 * value stays positive. The stop overhead is the median measured settle time.
 */
export function fitModel(runs: Run[], settings: DriveSettings, opts: { maxEvals?: number; seed?: number } = {}): FitResult {
  const start = settings.model;
  const usable = runs.filter((r) => r.samples.length >= 5);
  if (!usable.length) throw new Error('No runs of planned paths in these logs.');
  const toModel = (x: Float64Array): ModelConfig => {
    const m: ModelConfig = { ...start };
    FIT_KEYS.forEach((k, i) => (m[k] = start[k] * Math.exp(x[i])));
    return m;
  };
  const loss = (m: ModelConfig) => {
    let sum = 0;
    let count = 0;
    for (const r of usable) {
      const c = compareRun(r, { ...settings, model: m });
      for (let q = 0; q < c.predictedSpeed.length; q++) {
        const e = c.predictedSpeed[q] - r.measuredSpeed[q];
        sum += e * e;
        count++;
      }
      sum += 400 * (c.predicted - r.duration) ** 2;
      count++;
    }
    return sum / count;
  };
  const before = loss(start);
  const result = cmaes((x) => loss(toModel(x)), new Float64Array(FIT_KEYS.length), {
    sigma: 0.3,
    seed: opts.seed ?? 7,
    maxEvals: opts.maxEvals ?? 3000,
  });
  const model = toModel(result.x);
  const settles = usable.map((r) => r.settle).filter((s): s is number => s !== null).sort((a, b) => a - b);
  if (settles.length) model.stopOverhead = settles[settles.length >> 1];
  model.source = `fit from ${usable.length} run${usable.length === 1 ? '' : 's'}`;
  return { model, before, after: loss(model), runs: usable.length };
}

/**
 * A fake log of a planned chain, as if driven exactly at the model's speeds (plus optional pose noise). Used to test
 * the fitting before real logs exist.
 */
export function syntheticLog(
  chain: string,
  samples: Sample[],
  settings: DriveSettings,
  opts: { endStopped: boolean; voltage?: number; hz?: number; noise?: number; seed?: number; settle?: number },
): string {
  const voltage = opts.voltage ?? 12.5;
  const profile = speedProfile(samples, { ...settings, model: { ...settings.model, stopOverhead: 0 } }, { endStopped: opts.endStopped, voltage });
  const hz = opts.hz ?? 50;
  let seed = opts.seed ?? 1;
  const noise = () => {
    seed = (seed * 16807) % 2147483647;
    return ((seed / 2147483647) * 2 - 1) * (opts.noise ?? 0);
  };
  const lines = ['# path-planner log v1', '# routine=SYNTHETIC', '# alliance=RED', LOG_COLUMNS.join(',')];
  const total = profile.time[profile.time.length - 1];
  const row = (t: number, i: number, f: number, mode: string, busy: boolean, path: string) => {
    const a = samples[i];
    const b = samples[Math.min(i + 1, samples.length - 1)];
    const x = a.p.x + (b.p.x - a.p.x) * f + noise();
    const y = a.p.y + (b.p.y - a.p.y) * f + noise();
    const h = a.h + (b.h - a.h) * f;
    lines.push([t.toFixed(4), voltage.toFixed(2), x.toFixed(3), y.toFixed(3), h.toFixed(4), mode, busy ? 1 : 0, path, path ? a.seg : -1, a.t.toFixed(4)].join(','));
  };
  let i = 0;
  for (let q = 0; q * (1 / hz) <= total + 1e-9; q++) {
    const t = q / hz;
    while (i < samples.length - 2 && profile.time[i + 1] < t) i++;
    const span = profile.time[i + 1] - profile.time[i];
    const f = span > 0 && Number.isFinite(span) ? Math.min(Math.max((t - profile.time[i]) / span, 0), 1) : 0;
    row(t, i, f, 'FOLLOW', true, chain);
  }
  const settle = opts.settle ?? 0.25;
  for (let q = 1; q / hz <= settle + 0.2; q++) row(total + q / hz, samples.length - 1, 0, opts.endStopped ? 'HOLD' : 'IDLE', q / hz < settle, '');
  return lines.join('\n') + '\n';
}
