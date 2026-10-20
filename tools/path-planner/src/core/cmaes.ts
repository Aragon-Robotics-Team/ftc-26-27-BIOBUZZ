/**
 * CMA-ES (Hansen's (μ/μ_w, λ) evolution strategy with rank-one and rank-μ covariance updates), minimizing f.
 * Deterministic for a given seed.
 */
import { gaussian, rng } from './geom.ts';

export interface CmaOptions {
  sigma: number;
  seed: number;
  maxEvals: number;
  /** Stop when the best value hasn't improved by more than this over `patience` generations. */
  tolFun?: number;
  patience?: number;
  lambda?: number;
  /** Called after every generation; return false to stop early. */
  onGeneration?: (best: { x: Float64Array; f: number; evals: number }) => boolean | void;
}

export interface CmaResult {
  x: Float64Array;
  f: number;
  evals: number;
}

/**
 * Eigen-decomposition of a symmetric matrix: Householder reduction to tridiagonal form, then the implicit QL method
 * (tred2 / tql2, as in EISPACK and JAMA). Returns eigenvalues and the eigenvectors as columns of B.
 */
function eigen(C: Float64Array[], n: number): { d: Float64Array; B: Float64Array[] } {
  const V = C.map((r) => Float64Array.from(r));
  const d = new Float64Array(n);
  const e = new Float64Array(n);
  // tred2
  for (let j = 0; j < n; j++) d[j] = V[n - 1][j];
  for (let i = n - 1; i > 0; i--) {
    let scale = 0;
    let h = 0;
    for (let k = 0; k < i; k++) scale += Math.abs(d[k]);
    if (scale === 0) {
      e[i] = d[i - 1];
      for (let j = 0; j < i; j++) {
        d[j] = V[i - 1][j];
        V[i][j] = 0;
        V[j][i] = 0;
      }
    } else {
      for (let k = 0; k < i; k++) {
        d[k] /= scale;
        h += d[k] * d[k];
      }
      let f = d[i - 1];
      let g = Math.sqrt(h);
      if (f > 0) g = -g;
      e[i] = scale * g;
      h -= f * g;
      d[i - 1] = f - g;
      for (let j = 0; j < i; j++) e[j] = 0;
      for (let j = 0; j < i; j++) {
        f = d[j];
        V[j][i] = f;
        g = e[j] + V[j][j] * f;
        for (let k = j + 1; k <= i - 1; k++) {
          g += V[k][j] * d[k];
          e[k] += V[k][j] * f;
        }
        e[j] = g;
      }
      f = 0;
      for (let j = 0; j < i; j++) {
        e[j] /= h;
        f += e[j] * d[j];
      }
      const hh = f / (h + h);
      for (let j = 0; j < i; j++) e[j] -= hh * d[j];
      for (let j = 0; j < i; j++) {
        f = d[j];
        g = e[j];
        for (let k = j; k <= i - 1; k++) V[k][j] -= f * e[k] + g * d[k];
        d[j] = V[i - 1][j];
        V[i][j] = 0;
      }
    }
    d[i] = h;
  }
  for (let i = 0; i < n - 1; i++) {
    V[n - 1][i] = V[i][i];
    V[i][i] = 1;
    const h = d[i + 1];
    if (h !== 0) {
      for (let k = 0; k <= i; k++) d[k] = V[k][i + 1] / h;
      for (let j = 0; j <= i; j++) {
        let g = 0;
        for (let k = 0; k <= i; k++) g += V[k][i + 1] * V[k][j];
        for (let k = 0; k <= i; k++) V[k][j] -= g * d[k];
      }
    }
    for (let k = 0; k <= i; k++) V[k][i + 1] = 0;
  }
  for (let j = 0; j < n; j++) {
    d[j] = V[n - 1][j];
    V[n - 1][j] = 0;
  }
  V[n - 1][n - 1] = 1;
  e[0] = 0;
  // tql2
  for (let i = 1; i < n; i++) e[i - 1] = e[i];
  e[n - 1] = 0;
  let f = 0;
  let tst1 = 0;
  const eps = 2 ** -52;
  for (let l = 0; l < n; l++) {
    tst1 = Math.max(tst1, Math.abs(d[l]) + Math.abs(e[l]));
    let m = l;
    while (m < n && Math.abs(e[m]) > eps * tst1) m++;
    if (m > l) {
      for (let iter = 0; iter < 60; iter++) {
        let g = d[l];
        let p = (d[l + 1] - g) / (2 * e[l]);
        let r = Math.hypot(p, 1);
        if (p < 0) r = -r;
        d[l] = e[l] / (p + r);
        d[l + 1] = e[l] * (p + r);
        const dl1 = d[l + 1];
        let h = g - d[l];
        for (let i = l + 2; i < n; i++) d[i] -= h;
        f += h;
        p = d[m];
        let c = 1;
        let c2 = c;
        let c3 = c;
        const el1 = e[l + 1];
        let s = 0;
        let s2 = 0;
        for (let i = m - 1; i >= l; i--) {
          c3 = c2;
          c2 = c;
          s2 = s;
          g = c * e[i];
          h = c * p;
          r = Math.hypot(p, e[i]);
          e[i + 1] = s * r;
          s = e[i] / r;
          c = p / r;
          p = c * d[i] - s * g;
          d[i + 1] = h + s * (c * g + s * d[i]);
          for (let k = 0; k < n; k++) {
            h = V[k][i + 1];
            V[k][i + 1] = s * V[k][i] + c * h;
            V[k][i] = c * V[k][i] - s * h;
          }
        }
        p = (-s * s2 * c3 * el1 * e[l]) / dl1;
        e[l] = s * p;
        d[l] = c * p;
        if (Math.abs(e[l]) <= eps * tst1) break;
      }
    }
    d[l] += f;
    e[l] = 0;
  }
  for (let i = 0; i < n; i++) d[i] = Math.max(d[i], 1e-20);
  return { d, B: V };
}

export function cmaes(f: (x: Float64Array) => number, x0: Float64Array, opts: CmaOptions): CmaResult {
  const n = x0.length;
  if (n === 0) return { x: new Float64Array(0), f: f(new Float64Array(0)), evals: 1 };
  const random = rng(opts.seed);
  const normal = gaussian(random);
  const lambda = opts.lambda ?? 4 + Math.floor(3 * Math.log(n));
  const mu = Math.floor(lambda / 2);
  const wRaw = Array.from({ length: mu }, (_, i) => Math.log(mu + 0.5) - Math.log(i + 1));
  const wSum = wRaw.reduce((a, b) => a + b, 0);
  const w = wRaw.map((x) => x / wSum);
  const muEff = 1 / w.reduce((a, b) => a + b * b, 0);
  const cc = (4 + muEff / n) / (n + 4 + (2 * muEff) / n);
  const cs = (muEff + 2) / (n + muEff + 5);
  const c1 = 2 / ((n + 1.3) * (n + 1.3) + muEff);
  const cmu = Math.min(1 - c1, (2 * (muEff - 2 + 1 / muEff)) / ((n + 2) * (n + 2) + muEff));
  const damps = 1 + 2 * Math.max(0, Math.sqrt((muEff - 1) / (n + 1)) - 1) + cs;
  const chiN = Math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n));

  const mean = Float64Array.from(x0);
  let sigma = opts.sigma;
  const pc = new Float64Array(n);
  const ps = new Float64Array(n);
  let C: Float64Array[] = Array.from({ length: n }, (_, i) => {
    const r = new Float64Array(n);
    r[i] = 1;
    return r;
  });
  let B: Float64Array[] = C.map((r) => Float64Array.from(r));
  let D: Float64Array = new Float64Array(n).fill(1);
  let invSqrtC: Float64Array[] = C.map((r) => Float64Array.from(r));
  let eigenEval = 0;

  let best: CmaResult = { x: Float64Array.from(x0), f: f(x0), evals: 1 };
  let evals = 1;
  const history: number[] = [];
  const patience = opts.patience ?? 10 + Math.ceil((15 * n) / lambda);
  const tolFun = opts.tolFun ?? 1e-4;

  while (evals < opts.maxEvals) {
    const pop: { x: Float64Array; f: number }[] = [];
    for (let k = 0; k < lambda; k++) {
      const z = new Float64Array(n);
      for (let i = 0; i < n; i++) z[i] = normal();
      const y = new Float64Array(n);
      for (let i = 0; i < n; i++) {
        let s = 0;
        for (let j = 0; j < n; j++) s += B[i][j] * D[j] * z[j];
        y[i] = s;
      }
      const x = new Float64Array(n);
      for (let i = 0; i < n; i++) x[i] = mean[i] + sigma * y[i];
      const fx = f(x);
      evals++;
      pop.push({ x, f: Number.isFinite(fx) ? fx : 1e12 });
    }
    pop.sort((a, b) => a.f - b.f);
    if (pop[0].f < best.f) best = { x: Float64Array.from(pop[0].x), f: pop[0].f, evals };

    const oldMean = Float64Array.from(mean);
    for (let i = 0; i < n; i++) {
      let s = 0;
      for (let k = 0; k < mu; k++) s += w[k] * pop[k].x[i];
      mean[i] = s;
    }
    const yw = new Float64Array(n);
    for (let i = 0; i < n; i++) yw[i] = (mean[i] - oldMean[i]) / sigma;
    // ps ← (1 − cs)·ps + √(cs(2 − cs)μeff)·C^(−1/2)·yw
    const csFactor = Math.sqrt(cs * (2 - cs) * muEff);
    for (let i = 0; i < n; i++) {
      let s = 0;
      for (let j = 0; j < n; j++) s += invSqrtC[i][j] * yw[j];
      ps[i] = (1 - cs) * ps[i] + csFactor * s;
    }
    let psNorm = 0;
    for (let i = 0; i < n; i++) psNorm += ps[i] * ps[i];
    psNorm = Math.sqrt(psNorm);
    const gen = evals / lambda;
    const hsig = psNorm / Math.sqrt(1 - Math.pow(1 - cs, 2 * gen)) / chiN < 1.4 + 2 / (n + 1) ? 1 : 0;
    const ccFactor = Math.sqrt(cc * (2 - cc) * muEff);
    for (let i = 0; i < n; i++) pc[i] = (1 - cc) * pc[i] + hsig * ccFactor * yw[i];
    const Cn = C.map((r) => Float64Array.from(r));
    for (let i = 0; i < n; i++) {
      for (let j = 0; j <= i; j++) {
        let rankMu = 0;
        for (let k = 0; k < mu; k++) {
          const yi = (pop[k].x[i] - oldMean[i]) / sigma;
          const yj = (pop[k].x[j] - oldMean[j]) / sigma;
          rankMu += w[k] * yi * yj;
        }
        const v =
          (1 - c1 - cmu) * C[i][j] + c1 * (pc[i] * pc[j] + (1 - hsig) * cc * (2 - cc) * C[i][j]) + cmu * rankMu;
        Cn[i][j] = v;
        Cn[j][i] = v;
      }
    }
    C = Cn;
    sigma *= Math.exp((cs / damps) * (psNorm / chiN - 1));
    sigma = Math.min(sigma, 1e3);

    // Decompose C lazily, as Hansen's reference implementation does (a little less often, since it's the costly part).
    if (evals - eigenEval > lambda / (c1 + cmu) / n / 4) {
      eigenEval = evals;
      const e = eigen(C, n);
      B = e.B;
      D = e.d.map(Math.sqrt);
      invSqrtC = Array.from({ length: n }, (_, i) => {
        const r = new Float64Array(n);
        for (let j = 0; j < n; j++) {
          let s = 0;
          for (let k = 0; k < n; k++) s += (B[i][k] * B[j][k]) / D[k];
          r[j] = s;
        }
        return r;
      });
    }

    history.push(best.f);
    if (opts.onGeneration?.({ x: best.x, f: best.f, evals }) === false) break;
    if (history.length > patience && history[history.length - 1 - patience] - best.f < tolFun) break;
    if (sigma * Math.max(...D) < 1e-7) break;
  }
  return { ...best, evals };
}
