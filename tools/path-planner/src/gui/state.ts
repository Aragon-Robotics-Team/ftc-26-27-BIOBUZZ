/** App state, change notification, and keeping the work in this browser between visits. */
import type { PathLog, Run, RunComparison } from '../core/logs.ts';
import { chainSpecHash, type Plan, type Progress } from '../core/optimize.ts';
import { precheck, type PointIssue } from '../core/planner.ts';
import { defaultSettings, migrateSettings, type Chain, type Settings } from '../core/project.ts';

export type Selection =
  | { kind: 'point'; index: number }
  | { kind: 'keepout'; index: number; vertex: number }
  | null;

export interface State {
  settings: Settings;
  chains: Chain[];
  plans: Map<string, Plan>;
  /** Geometry hash of each path when its code was last copied, to show "not copied since". */
  copied: Map<string, string>;
  selected: string | null;
  selection: Selection;
  /** `shown` is the progress bar's position: it only moves forward. */
  running: { chain: string; progress: Progress | null; startedAt: number; shown: number } | null;
  logs: PathLog[];
  runs: Run[];
  comparisons: RunComparison[];
  playTime: number;
  playing: boolean;
}

export const state: State = {
  settings: defaultSettings(),
  chains: [],
  plans: new Map(),
  copied: new Map(),
  selected: null,
  selection: null,
  running: null,
  logs: [],
  runs: [],
  comparisons: [],
  playTime: 0,
  playing: false,
};

type Listener = (what: Set<string>) => void;
const listeners: Listener[] = [];

export function subscribe(fn: Listener): void {
  listeners.push(fn);
}

let pending = new Set<string>();
let scheduled = false;

/** Note what changed; views re-render once per frame. 'work' (an edit) also saves to this browser. */
export function changed(what: string): void {
  pending.add(what);
  if (scheduled) return;
  scheduled = true;
  requestAnimationFrame(() => {
    scheduled = false;
    const batch = pending;
    pending = new Set();
    if (batch.has('work')) persist();
    for (const fn of listeners) fn(batch);
  });
}

export function selectedChain(): Chain | null {
  return state.chains.find((c) => c.name === state.selected) ?? null;
}

export type PlanStatus = 'none' | 'stale' | 'ok' | 'infeasible';

export function planStatus(chain: Chain): PlanStatus {
  const plan = state.plans.get(chain.name);
  if (!plan) return 'none';
  if (plan.specHash !== chainSpecHash(state.settings, chain)) return 'stale';
  return plan.feasible ? 'ok' : 'infeasible';
}

/** True when the path's current plan differs from the code last copied. */
export function notCopied(chain: Chain): boolean {
  const plan = state.plans.get(chain.name);
  return !!plan && state.copied.get(chain.name) !== plan.geometryHash;
}

const KEY = 'path-planner-work-v1';

function persist(): void {
  try {
    localStorage.setItem(
      KEY,
      JSON.stringify({ settings: state.settings, chains: state.chains, plans: [...state.plans.values()], copied: [...state.copied], selected: state.selected }),
    );
  } catch {
    // storage is a convenience; the copied code is what counts
  }
}

/** Restores the work from the last visit in this browser. Returns false if there was none. */
export function restore(): boolean {
  try {
    const raw = localStorage.getItem(KEY);
    if (!raw) return false;
    const saved = JSON.parse(raw) as { settings: Settings; chains: Chain[]; plans: Plan[]; copied: [string, string][]; selected: string | null };
    state.settings = migrateSettings(saved.settings);
    state.chains = saved.chains;
    state.plans = new Map(saved.plans.map((p) => [p.chain, p]));
    state.copied = new Map(saved.copied);
    state.selected = saved.selected;
    return true;
  } catch {
    return false;
  }
}

let precheckKey = '';
let precheckResult: PointIssue[] = [];

/** Problems the selected path's points force, recomputed only when the path or settings change. */
export function pointIssues(chain: Chain): PointIssue[] {
  const key = JSON.stringify([chain, state.settings]);
  if (key !== precheckKey) {
    precheckKey = key;
    precheckResult = precheck(state.settings, chain);
  }
  return precheckResult;
}
