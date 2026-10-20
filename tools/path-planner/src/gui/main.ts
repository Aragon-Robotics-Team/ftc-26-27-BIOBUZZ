/** Path Planner page: wires the field, the side panel, the dialogs and the optimizer together. */
import { toPp } from '../core/export.ts';
import { chainSpecHash } from '../core/optimize.ts';
import type { Chain } from '../core/project.ts';
import { drawChart } from './chart.ts';
import { confirmDialog, copyDialog, helpDialog, logsDialog, pasteDialog, refreshRuns, settingsDialog } from './dialogs.ts';
import { h, toast } from './dom.ts';
import { clearDerived, derive, draw as drawField, resize } from './fieldView.ts';
import { optimize, stop } from './optimizer.ts';
import { renderPanel, renderPanelHeader, renderProgress, type PanelActions } from './panel.ts';
import { changed, notCopied, planStatus, pointIssues, restore, selectedChain, state, subscribe } from './state.ts';

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;

function uniqueName(base: string): string {
  const taken = (n: string) => state.chains.some((c) => c.name === n);
  if (!taken(base)) return base;
  for (let i = 2; ; i++) if (!taken(`${base}${i}`)) return `${base}${i}`;
}

/** A new path starting where the selected one ends, so paths chain together. */
function newPath(): void {
  const from = selectedChain();
  const end = from?.points[from.points.length - 1];
  const start = end ? { x: end.x, y: end.y, heading: end.heading } : { x: 24, y: 36, heading: 90 };
  const chain: Chain = {
    name: uniqueName('newPath'),
    points: [
      { kind: 'fixed', ...start, headingTol: 0 },
      { kind: 'fixed', x: Math.min(60, start.x + 12), y: start.y + 36 > 128 ? start.y - 36 : start.y + 36, heading: start.heading, headingTol: 0 },
    ],
    legs: [{ rules: [] }],
    endStopped: true,
    matchStart: false,
    markers: [],
    keepOut: [],
  };
  state.chains.push(chain);
  state.selected = chain.name;
  state.selection = null;
  state.playTime = 0;
  changed('work');
  changed('selected');
  requestAnimationFrame(() => ($('panel').querySelector('.path-name') as HTMLInputElement | null)?.select());
}

async function runOptimize(force = false): Promise<void> {
  const chain = selectedChain();
  if (!chain || state.running) return;
  const forced = pointIssues(chain);
  if (forced.length && !force) {
    confirmDialog('Some points can\'t fit where they are', forced.map((f) => f.message), 'Optimize anyway', () => void runOptimize(true));
    return;
  }
  // Optimizing an unchanged path again searches with a new seed and keeps whichever is faster: on long paths the
  // search can settle in different local optima, so a few tries can find a quicker one.
  const prev = state.plans.get(chain.name);
  const again = !!prev && prev.specHash === chainSpecHash(state.settings, chain);
  const seed = again ? (prev.searches ?? 1) + 1 : 1;
  state.running = { chain: chain.name, progress: null, startedAt: performance.now(), shown: 0 };
  changed('running');
  try {
    let plan = await optimize(structuredClone(state.settings), structuredClone(chain), seed, (p) => {
      if (state.running) {
        state.running.progress = p;
        state.running.shown = Math.max(state.running.shown, p.fraction);
      }
      changed('progress');
    });
    if (again && prev.feasible && (!plan.feasible || plan.time >= prev.time - 1e-6)) {
      plan = { ...prev, searches: seed };
      toast(`No faster path this time, kept ${prev.time.toFixed(2)} s.`);
    } else {
      plan = { ...plan, searches: seed };
      if (again && prev.feasible) toast(`Found a faster path: ${plan.time.toFixed(2)} s (was ${prev.time.toFixed(2)} s).`);
      else if (!plan.feasible) toast(`${chain.name} can't keep every gap. See the note in the panel.`, 5000);
    }
    state.plans.set(chain.name, plan);
    state.playTime = 0;
  } catch (e) {
    if ((e as Error).message !== 'stopped') toast((e as Error).message, 6000);
  }
  state.running = null;
  changed('work');
  changed('running');
}

const actions: PanelActions = {
  optimize: () => void runOptimize(),
  stop: () => {
    stop();
    state.running = null;
    changed('running');
  },
  copy: copyDialog,
  downloadPp: () => {
    const chain = selectedChain();
    const plan = chain && state.plans.get(chain.name);
    if (!chain || !plan) return;
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([toPp(chain, state.settings, plan)], { type: 'application/json' }));
    a.download = `${chain.name}.pp`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  },
  clear: () => {
    const chain = selectedChain();
    const plan = chain && state.plans.get(chain.name);
    if (!chain || !plan) return;
    state.plans.delete(chain.name);
    state.playTime = 0;
    state.playing = false;
    changed('work');
    toast('Cleared the optimized path.', 5000, {
      label: 'Undo',
      run: () => {
        state.plans.set(chain.name, plan);
        changed('work');
      },
    });
  },
  newPath,
  paste: pasteDialog,
};

function renderSelect(): void {
  const sel = $<HTMLSelectElement>('pathSelect');
  const mark = (c: Chain) => (planStatus(c) === 'infeasible' ? ' ⚠' : planStatus(c) !== 'ok' ? ' ○' : notCopied(c) ? ' •' : '');
  sel.replaceChildren(
    ...(state.chains.length ? [] : [h('option', { value: '' }, 'No paths')]),
    ...state.chains.map((c) => h('option', { value: c.name, selected: c.name === state.selected }, c.name + mark(c))),
  );
  sel.disabled = !state.chains.length;
  // No new or pasted paths while one is being optimized.
  $<HTMLButtonElement>('newPath').disabled = !!state.running;
  $<HTMLButtonElement>('pastePath').disabled = !!state.running;
  sel.title = 'Paths. ○ needs optimizing, ⚠ too close to something, • changed since copied';
}

function currentDerived() {
  const chain = selectedChain();
  const plan = chain && state.plans.get(chain.name);
  return chain && plan ? derive(plan, chain) : null;
}

function renderPlayback(): void {
  const d = currentDerived();
  const total = d ? d.profile.time[d.profile.time.length - 1] : 0;
  $<HTMLInputElement>('scrub').value = String(total > 0 ? Math.round((state.playTime / total) * 1000) : 0);
  $<HTMLInputElement>('scrub').disabled = !d;
  $<HTMLButtonElement>('play').disabled = !d;
  $('scrubTime').textContent = `${state.playTime.toFixed(2)} s`;
  $('playIcon').innerHTML = state.playing ? '<path d="M6 19h4V5H6v14zm8-14v14h4V5h-4z"/>' : '<path d="M8 5v14l11-7z"/>';
}

function renderChart(): void {
  if ($('chartBox').hidden) return;
  const chain = selectedChain();
  $('chartLegend').hidden = !currentDerived();
  drawChart(currentDerived(), chain ? state.comparisons.filter((c) => c.run.chain === chain.name) : [], state.playTime);
}

subscribe((batch) => {
  if (batch.has('work')) {
    clearDerived();
    if (state.logs.length) refreshRuns();
  }
  renderSelect();
  const panel = $('panel');
  const typing = batch.has('form') && panel.contains(document.activeElement) && !batch.has('selected') && !batch.has('selection');
  const onlyProgress = [...batch].every((w) => w === 'progress' || w === 'play');
  if (onlyProgress) renderProgress();
  else if (typing) renderPanelHeader(actions);
  else renderPanel(actions);
  drawField();
  renderPlayback();
  renderChart();
});

// ---- Controls ----

$('pathSelect').addEventListener('change', (e) => {
  state.selected = (e.target as HTMLSelectElement).value || null;
  state.selection = null;
  state.playTime = 0;
  state.playing = false;
  changed('selected');
});
$('newPath').addEventListener('click', newPath);
$('pastePath').addEventListener('click', pasteDialog);
$('openSettings').addEventListener('click', settingsDialog);
$('openLogs').addEventListener('click', logsDialog);
$('help').addEventListener('click', helpDialog);
$('chartToggle').addEventListener('click', () => {
  const box = $('chartBox');
  box.hidden = !box.hidden;
  $('chartToggle').textContent = box.hidden ? 'Speed ▸' : 'Speed ▾';
  $('chartToggle').setAttribute('aria-expanded', String(!box.hidden));
  resize();
  renderChart();
});

let last = 0;
function tick(now: number): void {
  if (!state.playing) return;
  const d = currentDerived();
  const total = d ? d.profile.time[d.profile.time.length - 1] : 0;
  state.playTime = Math.min(total, state.playTime + (now - last) / 1000);
  last = now;
  if (!d || state.playTime >= total) state.playing = false;
  changed('play');
  if (state.playing) requestAnimationFrame(tick);
}
$('play').addEventListener('click', () => {
  const d = currentDerived();
  if (!d) return;
  if (state.playTime >= d.profile.time[d.profile.time.length - 1] - 1e-3) state.playTime = 0;
  state.playing = !state.playing;
  last = performance.now();
  if (state.playing) requestAnimationFrame(tick);
  changed('play');
});
$('scrub').addEventListener('input', (e) => {
  const d = currentDerived();
  if (!d) return;
  state.playing = false;
  state.playTime = (Number((e.target as HTMLInputElement).value) / 1000) * d.profile.time[d.profile.time.length - 1];
  changed('play');
});

const themes = ['auto', 'light', 'dark'] as const;
let theme = 0;
try {
  theme = Math.max(0, themes.indexOf((localStorage.getItem('path-planner-theme') ?? 'auto') as (typeof themes)[number]));
} catch {
  // storage may be blocked
}
const applyTheme = () => {
  if (themes[theme] === 'auto') delete document.documentElement.dataset.theme;
  else document.documentElement.dataset.theme = themes[theme];
  changed('theme');
};
$('theme').addEventListener('click', () => {
  theme = (theme + 1) % themes.length;
  try {
    localStorage.setItem('path-planner-theme', themes[theme]);
  } catch {
    // ignore
  }
  applyTheme();
});
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => changed('theme'));

// ---- Start ----

applyTheme();
restore();
if (!state.chains.some((c) => c.name === state.selected)) state.selected = state.chains[0]?.name ?? null;
resize();
changed('all');
try {
  if (!localStorage.getItem('path-planner-seen-help')) {
    localStorage.setItem('path-planner-seen-help', '1');
    helpDialog();
  }
} catch {
  // ignore
}
