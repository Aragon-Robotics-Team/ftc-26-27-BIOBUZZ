/** The side panel for the selected path. */
import { sampleChain } from '../core/chain.ts';
import { wrap, type Vec } from '../core/geom.ts';
import type { Limit } from '../core/model.ts';
import { DRIFT_LIMIT, LAG_LIMIT, type Plan } from '../core/optimize.ts';
import { pointLabel } from '../core/planner.ts';
import { JAVA_NAME, MARKER_NAME, type Chain, type ChainPoint, type HeadingRule } from '../core/project.ts';
import { LIMIT_COLORS, LIMIT_NAMES } from './chart.ts';
import { fmt, h, num, select, text } from './dom.ts';
import { addPointAt, deletePoint } from './fieldView.ts';
import { changed, notCopied, planStatus, pointIssues, selectedChain, state } from './state.ts';

export interface PanelActions {
  optimize: () => void;
  stop: () => void;
  copy: () => void;
  clear: () => void;
  downloadPp: () => void;
  newPath: () => void;
  paste: () => void;
}

const root = () => document.getElementById('panel')!;

/** An edit from a form field: save it and re-render, but not the panel being typed in. */
const edited = () => {
  changed('work');
  changed('form');
};
/** An edit that changes the panel's structure: re-render it too. */
const restructured = () => changed('work');

function statusLine(chain: Chain): HTMLElement {
  const plan = state.plans.get(chain.name);
  const running = state.running?.chain === chain.name;
  const s = planStatus(chain);
  const [cls, text] = running
    ? ['dim', 'Optimizing…']
    : s === 'ok'
      ? ['good', 'Fits']
      : s === 'infeasible'
        ? ['bad', 'Too close']
        : s === 'stale'
          ? ['warn', 'Changed: optimize again']
          : ['dim', 'Not optimized'];
  return h('div', {},
    h('div', { class: 'status' },
      plan && !running ? h('span', { class: 'time' }, `${plan.time.toFixed(2)} s`) : null,
      h('span', { class: `pill ${cls}` }, text)),
    running ? h('div', { class: 'run-progress' }, progressStats()) : null);
}

/**
 * Refreshes just the optimizer's progress. The rest of the header (the Stop button in particular) stays the same
 * element: rebuilding it several times a second meant a click could land on a button that had just been replaced.
 */
export function renderProgress(): void {
  const box = root().querySelector('.run-progress');
  if (box && state.running) box.replaceChildren(progressStats());
}

/** How the optimizer is getting on: route, current and best times, effort, time taken. */
function progressStats(): HTMLElement {
  const r = state.running!;
  const p = r.progress;
  const elapsed = `${((performance.now() - r.startedAt) / 1000).toFixed(0)} s`;
  const bar = h('div', { class: 'progress', role: 'progressbar', 'aria-valuemin': '0', 'aria-valuemax': '100', 'aria-valuenow': String(Math.round(r.shown * 100)) },
    h('i', { style: `width:${(3 + 97 * r.shown).toFixed(1)}%` }));
  if (!p) return h('div', {}, bar, h('div', { class: 'stats' }, h('span', {}, 'Finding routes around obstacles…'), h('span', {}, elapsed)));
  const what = p.phase === 'screen'
    ? `Trying route ${p.seed + 1} of ${p.seeds}`
    : `Refining route ${p.seed + 1}${p.attempt <= p.runs ? ` · run ${p.attempt} of ${p.runs}` : ` · retry ${p.attempt - p.runs} of ${p.attempts - p.runs}`}`;
  return h('div', {}, bar, h('div', { class: 'stats' },
    h('span', {}, what),
    h('span', {}, `${p.evals.toLocaleString()} tries · ${elapsed}`),
    h('span', {}, 'This route ', h('b', {}, `${p.bestTime.toFixed(2)} s`), ' ', p.feasible ? h('span', { class: 'good' }, 'fits') : h('span', { class: 'warn' }, 'not clear yet')),
    h('span', {}, 'Best so far ', h('b', {}, p.bestSoFar === null ? '—' : `${p.bestSoFar.toFixed(2)} s`))));
}

/** The first thing wrong with the plan, in words. */
function problem(chain: Chain): string | null {
  const plan = state.plans.get(chain.name);
  if (!plan || planStatus(chain) === 'stale') return null;
  // Older plans could turn back on themselves and still count as fitting.
  const at = reversal(plan);
  if (at) {
    const near = chain.points.reduce((best, p, i) => (Math.hypot(p.x - at.x, p.y - at.y) < Math.hypot(chain.points[best].x - at.x, chain.points[best].y - at.y) ? i : best), 0);
    return `It turns back on itself near ${pointLabel(chain, near)}. Pedro only brakes at the end of a path, so it would overshoot there. Optimize again, or end this path at ${pointLabel(chain, near)} and start a new one there.`;
  }
  if (!plan || planStatus(chain) !== 'infeasible') return null;
  const drift = plan.check.drift;
  if (drift && drift.worst > DRIFT_LIMIT) return `Pedro would drift ${drift.worst.toFixed(1)} in wide ${Math.round(drift.at)} in along: it can't slow down for a curve that tight. Round it out or move a point.`;
  const lag = plan.check.lag;
  if (lag && lag.worst > LAG_LIMIT) return `Pedro's heading would fall ${Math.round((lag.worst * 180) / Math.PI)}° behind ${Math.round(lag.at)} in along: the plan turns faster than the robot can there.`;
  const c = [...plan.check.clearances].sort((a, b) => a.gap - a.margin - (b.gap - b.margin))[0];
  if (c && c.gap < c.margin - 1e-3) return `Too close to ${c.name.toLowerCase()}: ${Math.max(0, c.gap).toFixed(1)} in (needs ${c.margin}).`;
  const r = plan.check.rules[0];
  if (r) return `Can't keep the facing rule on leg ${r.leg + 1}→${r.leg + 2}.`;
  return null;
}

const reversals = new WeakMap<Plan, Vec | null>();

/** Where a plan turns back on itself (and so must stop), if anywhere. Pedro only brakes at the end of a path. */
function reversal(plan: Plan): Vec | null {
  if (!reversals.has(plan)) {
    const smp = sampleChain(plan.segments, 0.5).samples;
    let at: Vec | null = null;
    for (let i = 1; i < smp.length - 1 && !at; i++) if (Math.abs(wrap(smp[i].travel - smp[i - 1].travel)) > Math.PI / 2) at = smp[i].p;
    reversals.set(plan, at);
  }
  return reversals.get(plan) ?? null;
}

const FACING: [string, string][] = [
  ['free', 'Any way'],
  ['front-first', 'Intake first'],
  ['fixed', 'Fixed heading'],
  ['turret-reach', 'Turret aims at hive'],
];

function legRow(chain: Chain, i: number): HTMLElement {
  const leg = chain.legs[i];
  const rule: HeadingRule = leg.rules[0] ?? { type: 'free' };
  const set = (r: HeadingRule) => {
    leg.rules = r.type === 'free' ? [] : [r];
  };
  const extra: HTMLElement[] = [];
  const deg = (input: HTMLElement) => h('span', { class: 'unit-in' }, input, h('span', { class: 'dim' }, '°'));
  // Edit the rule in place: the panel isn't redrawn while typing, so a copy made at render time would be stale.
  if (rule.type === 'front-first') extra.push(h('span', { class: 'dim' }, 'within ±'), deg(num(rule.tol, (v) => ((rule.tol = v), edited()), { min: 0, max: 90, label: 'Tolerance in degrees', cls: 'tiny' })));
  if (rule.type === 'fixed') {
    extra.push(deg(num(rule.heading, (v) => ((rule.heading = v), edited()), { label: 'Heading in degrees', cls: 'tiny' })),
      h('span', { class: 'dim' }, '±'), deg(num(rule.tol, (v) => ((rule.tol = v), edited()), { min: 0, max: 180, label: 'Tolerance in degrees', cls: 'tiny' })));
  }
  return h('li', { class: 'leg' },
    h('span', { class: 'leg-line' }),
    h('div', { class: 'leg-body' },
      h('div', { class: 'inline' },
        h('span', { class: 'dim' }, 'Facing'),
        select(rule.type, FACING as [HeadingRule['type'], string][], (t) => {
          // Start a fixed heading from where the leg begins, so it doesn't clash with the point right away.
          const from = chain.points[i];
          set(t === 'fixed' ? { type: 'fixed', heading: from.heading, tol: 10 } : t === 'front-first' ? { type: 'front-first', tol: 15 } : t === 'turret-reach' ? { type: 'turret-reach', margin: 10 } : { type: 'free' });
          restructured();
        }, 'How the robot faces on this leg')),
      extra.length ? h('div', { class: 'inline leg-extra' }, ...extra) : null));
}

function pointRow(chain: Chain, i: number): HTMLElement {
  const p = chain.points[i];
  const first = i === 0;
  const last = i === chain.points.length - 1;
  const selected = state.selection?.kind === 'point' && state.selection.index === i;
  const name = first ? 'Start' : last ? 'End' : p.kind === 'region' ? 'Area' : 'Point';
  const anyHeading = p.headingTol >= 180;
  const headingText = anyHeading ? 'any heading' : `${fmt(p.heading, 1)}°${p.headingTol ? ` ±${fmt(p.headingTol, 1)}` : ''}`;
  const row = h('li', { class: `pt${selected ? ' selected' : ''}` },
    h('button', { class: 'pt-head', type: 'button', onclick: () => {
      state.selection = selected ? null : { kind: 'point', index: i };
      changed('selection');
    } },
      h('span', { class: `dot ${p.kind}${pointIssues(chain).some((f) => f.point === i) ? ' bad' : ''}` }, String(i + 1)),
      h('span', { class: 'pt-name' }, name),
      h('span', { class: 'pt-sum' }, `${fmt(p.x, 1)}, ${fmt(p.y, 1)} · ${headingText}`)),
  );
  if (!selected) return row;
  const set = (k: 'x' | 'y' | 'heading' | 'headingTol' | 'halfWidth' | 'halfHeight') => (v: number) => {
    (p as unknown as Record<string, number>)[k] = v;
    edited();
  };
  const fields: HTMLElement[] = [
    field('X', num(p.x, set('x'), { label: 'X' })),
    field('Y', num(p.y, set('y'), { label: 'Y' })),
  ];
  if (p.kind === 'region') {
    fields.push(field('± X', num(p.halfWidth, set('halfWidth'), { min: 0, label: 'Half width' })), field('± Y', num(p.halfHeight, set('halfHeight'), { min: 0, label: 'Half height' })));
  }
  const headingSel = select(anyHeading ? 'any' : p.headingTol > 0 ? 'about' : 'exact', [['exact', 'Exactly'], ['about', 'About'], ['any', 'Any']], (v) => {
    p.headingTol = v === 'any' ? 180 : v === 'about' ? 15 : 0;
    restructured();
  }, 'Heading here');
  const headingFields = anyHeading ? [] : [num(p.heading, set('heading'), { label: 'Heading', cls: 'tiny' }), h('span', { class: 'dim' }, '°')];
  if (!anyHeading && p.headingTol > 0) headingFields.push(h('span', { class: 'dim' }, '±'), num(p.headingTol, set('headingTol'), { min: 0, max: 179, label: 'Heading tolerance', cls: 'tiny' }), h('span', { class: 'dim' }, '°'));
  row.append(h('div', { class: 'pt-edit' },
    h('div', { class: 'grid' }, ...fields),
    h('div', { class: 'inline' }, h('span', { class: 'dim' }, 'Heading'), headingSel, ...headingFields),
    !first && !last
      ? h('div', { class: 'inline' },
          select(p.kind, [['fixed', 'Point'], ['region', 'Area (passes somewhere inside)']], (k) => {
            chain.points[i] = k === 'region'
              ? { kind: 'region', x: p.x, y: p.y, halfWidth: 6, halfHeight: 6, heading: p.heading, headingTol: p.headingTol }
              : { kind: 'fixed', x: p.x, y: p.y, heading: p.heading, headingTol: p.headingTol };
            restructured();
          }, 'Point type'),
          h('button', { class: 'link danger', type: 'button', onclick: () => deletePoint(i) }, 'Remove'))
      : null,
  ));
  return row;
}

function field(label: string, input: HTMLElement): HTMLElement {
  return h('label', { class: 'lbl' }, h('span', {}, label), input);
}

function fold(title: string, open: boolean, ...body: (HTMLElement | null)[]): HTMLDetailsElement {
  const d = h('details', { class: 'fold', open }, h('summary', {}, title), ...body);
  d.addEventListener('toggle', () => foldState.set(title.replace(/ \(\d+\)$/, ''), d.open));
  return d;
}
const foldState = new Map<string, boolean>();
const isOpen = (title: string, dflt: boolean) => foldState.get(title) ?? dflt;

function markers(chain: Chain): HTMLElement {
  const rows = chain.markers.map((m, i) => h('div', { class: 'inline' },
    text(m.name, (v) => {
      if (!MARKER_NAME.test(v)) return false;
      m.name = v;
      edited();
    }, { label: 'Marker name' }),
    select(String(m.leg), chain.legs.map((_, k) => [String(k), `${k + 1}→${k + 2}`] as [string, string]), (v) => ((m.leg = Number(v)), edited()), 'Leg'),
    num(Math.round(m.at * 100), (v) => ((m.at = v / 100), edited()), { min: 0, max: 100, label: 'Percent along the leg', cls: 'tiny' }),
    h('span', { class: 'dim' }, '%'),
    h('button', { class: 'x', type: 'button', 'aria-label': `Remove ${m.name}`, onclick: () => (chain.markers.splice(i, 1), restructured()) }, '×')));
  return fold(`Markers (${chain.markers.length})`, isOpen('Markers', chain.markers.length > 0),
    ...rows,
    h('button', { class: 'link', type: 'button', onclick: () => {
      chain.markers.push({ name: `MARKER_${chain.markers.length + 1}`, leg: 0, at: 0.5 });
      foldState.set('Markers', true);
      restructured();
    } }, '+ Marker'));
}

function avoid(chain: Chain): HTMLElement {
  const rows = chain.keepOut.map((k, i) => h('div', { class: 'inline' },
    text(k.name, (v) => ((k.name = v), edited()), { label: 'Zone name' }),
    h('button', { class: 'link', type: 'button', onclick: () => {
      const a = k.points[k.points.length - 1];
      const b = k.points[0];
      k.points.push({ x: (a.x + b.x) / 2 + 2, y: (a.y + b.y) / 2 + 2 });
      restructured();
    } }, '+ corner'),
    h('button', { class: 'x', type: 'button', 'aria-label': `Remove ${k.name}`, onclick: () => (chain.keepOut.splice(i, 1), restructured()) }, '×')));
  return fold(`Avoid zones (${chain.keepOut.length})`, isOpen('Avoid zones', chain.keepOut.length > 0),
    ...rows,
    h('button', { class: 'link', type: 'button', onclick: () => {
      const a = chain.points[0];
      const b = chain.points[chain.points.length - 1];
      const cx = Math.min(64, (a.x + b.x) / 2);
      const cy = (a.y + b.y) / 2;
      chain.keepOut.push({ name: `Zone ${chain.keepOut.length + 1}`, points: [{ x: cx - 6, y: cy - 6 }, { x: cx + 6, y: cy - 6 }, { x: cx + 6, y: cy + 6 }, { x: cx - 6, y: cy + 6 }] });
      foldState.set('Avoid zones', true);
      restructured();
    } }, '+ Zone'));
}

function options(chain: Chain): HTMLElement {
  // Gaps normally come from Robot & speed (all paths). A path can use its own, for a tight spot.
  const own = chain.margins !== undefined;
  const m = chain.margins ?? {};
  const margin = (key: 'obstacle' | 'centerline' | 'headroom', label: string) =>
    field(label, num(m[key] ?? state.settings.margins[key], (v) => {
      chain.margins = { ...(chain.margins ?? {}), [key]: v };
      edited();
    }, { min: 0, label }));
  return fold('Options', isOpen('Options', false),
    h('label', { class: 'check' }, h('input', { type: 'checkbox', checked: !chain.endStopped, onchange: (e: Event) => ((chain.endStopped = !(e.target as HTMLInputElement).checked), restructured()) }), 'Keep moving at the end (no stop)'),
    h('label', { class: 'check' }, h('input', { type: 'checkbox', checked: chain.matchStart, onchange: (e: Event) => ((chain.matchStart = (e.target as HTMLInputElement).checked), restructured()) }), 'Start is the match start position'),
    h('label', { class: 'check', title: 'Not allowed in AUTO (G402: major foul)' }, h('input', { type: 'checkbox', checked: !!chain.allowCrossing, onchange: (e: Event) => {
      if ((e.target as HTMLInputElement).checked) chain.allowCrossing = true;
      else delete chain.allowCrossing;
      restructured();
    } }), 'Allow crossing the centerline'),
    h('label', { class: 'check' }, h('input', { type: 'checkbox', checked: own, onchange: (e: Event) => {
      chain.margins = (e.target as HTMLInputElement).checked ? { ...state.settings.margins } : undefined;
      restructured();
    } }), 'Use different gaps for this path'),
    own ? h('div', { class: 'grid' }, margin('obstacle', 'Gap to obstacles, in'), margin('centerline', 'Gap to centerline, in'), margin('headroom', 'Headroom, in')) : null,
  );
}

function details(chain: Chain): HTMLElement | null {
  const plan = state.plans.get(chain.name);
  if (!plan) return null;
  const limits = (Object.entries(plan.limits) as [Limit, number][]).filter(([k]) => k !== 'stop').sort((a, b) => b[1] - a[1]);
  const clear = [...plan.check.clearances].sort((a, b) => a.gap - a.margin - (b.gap - b.margin)).slice(0, 4);
  return fold('Details', isOpen('Details', false),
    h('div', { class: 'sub' }, `${plan.length.toFixed(0)} in · speed model: ${plan.modelSource}`),
    h('div', { class: 'sub' }, 'What sets the speed'),
    ...limits.map(([k, v]) => h('div', { class: 'bar-row' },
      h('span', {}, LIMIT_NAMES[k]),
      h('span', { class: 'bar' }, h('i', { style: `width:${(v * 100).toFixed(0)}%;background:${LIMIT_COLORS[k]}` })),
      h('span', { class: 'num' }, `${(v * 100).toFixed(0)}%`))),
    h('div', { class: 'sub' }, 'Closest to'),
    ...clear.map((c) => h('div', { class: 'bar-row' },
      h('span', {}, c.name),
      h('span', {}),
      h('span', { class: `num ${c.gap < c.margin - 1e-3 ? 'bad' : ''}` }, `${c.gap.toFixed(1)} in`))));
}

/** Time, status, the two buttons and any problem: everything that changes as the path is edited. */
function header(chain: Chain, actions: PanelActions): HTMLElement {
  const running = state.running?.chain === chain.name;
  const plan = state.plans.get(chain.name);
  const issue = problem(chain);
  const forced = pointIssues(chain);
  return h('div', { class: 'head' },
    statusLine(chain),
    h('div', { class: 'actions' },
      running
        ? h('button', { class: 'btn', type: 'button', onclick: actions.stop }, 'Stop')
        : h('button', { class: 'btn primary', type: 'button', title: planStatus(chain) === 'ok' ? 'Search again with a different start; keeps the faster path' : '', onclick: actions.optimize }, planStatus(chain) === 'ok' ? 'Optimize again' : 'Optimize'),
      h('button', { class: `btn${notCopied(chain) ? ' attention' : ''}`, type: 'button', disabled: !plan || running, title: notCopied(chain) ? 'Changed since you last copied it' : '', onclick: actions.copy }, 'Copy code'),
      plan && !running ? h('button', { class: 'link minor', type: 'button', title: 'Download a file for the Pedro Visualizer', onclick: actions.downloadPp }, '.pp') : null,
      plan && !running ? h('button', { class: 'link minor clear', type: 'button', title: 'Remove the optimized path', onclick: actions.clear }, 'Clear') : null),
    forced.length
      ? h('div', { class: 'issue' },
          h('b', {}, 'Can\'t fit as placed:'),
          h('ul', {}, ...forced.slice(0, 4).map((f) => h('li', {}, f.message))),
          forced.length > 4 ? h('span', {}, `and ${forced.length - 4} more`) : null)
      : issue
        ? h('p', { class: planStatus(chain) === 'ok' ? 'issue soft' : 'issue' }, issue)
        : null);
}

/** While someone types in the panel, only refresh its header so the field they're in keeps focus. */
export function renderPanelHeader(actions: PanelActions): void {
  const chain = selectedChain();
  const old = root().querySelector('.head');
  if (chain && old) old.replaceWith(header(chain, actions));
}

export function renderPanel(actions: PanelActions): void {
  const el = root();
  const chain = selectedChain();
  if (!chain) {
    el.replaceChildren(h('div', { class: 'empty' },
      h('p', {}, state.chains.length ? 'Pick a path from the list, or click one on the field.' : 'No paths yet.'),
      h('div', { class: 'actions' },
        h('button', { class: 'btn primary', type: 'button', onclick: actions.newPath }, 'New path'),
        h('button', { class: 'btn', type: 'button', onclick: actions.paste }, 'Paste code'))));
    return;
  }
  const nameInput = text(chain.name, (v) => {
    if (!JAVA_NAME.test(v) || state.chains.some((c) => c !== chain && c.name === v)) return false;
    for (const map of [state.plans, state.copied] as Map<string, unknown>[]) {
      if (map.has(chain.name)) {
        map.set(v, map.get(chain.name));
        map.delete(chain.name);
      }
    }
    const plan = state.plans.get(v);
    if (plan) state.plans.set(v, { ...plan, chain: v });
    chain.name = v;
    state.selected = v;
    restructured();
  }, { label: 'Path name (the Java method name)' });
  nameInput.classList.add('path-name');
  const list: HTMLElement[] = [];
  chain.points.forEach((_, i) => {
    list.push(pointRow(chain, i));
    if (i < chain.legs.length) list.push(legRow(chain, i));
  });
  // Nothing can change while a path is being optimized: the result would be out of date before it arrived.
  const locked = !!state.running;
  nameInput.disabled = locked;
  const parts: (HTMLElement | null)[] = [
    nameInput,
    header(chain, actions),
    h('fieldset', { class: 'lock', disabled: locked, title: locked ? 'Editing is paused while it optimizes' : '' },
    h('ol', { class: 'points' }, ...list),
    h('div', { class: 'add-row' },
      h('button', { class: 'link', type: 'button', onclick: () => addMiddle(chain, 'fixed') }, '+ Point'),
      h('button', { class: 'link', type: 'button', onclick: () => addMiddle(chain, 'region') }, '+ Area')),
    markers(chain),
    avoid(chain),
    options(chain),
    details(chain),
    h('button', { class: 'link danger delete-path', type: 'button', onclick: () => {
      if (!confirm(`Delete ${chain.name}?`)) return;
      state.chains.splice(state.chains.indexOf(chain), 1);
      state.plans.delete(chain.name);
      state.copied.delete(chain.name);
      state.selected = state.chains[0]?.name ?? null;
      state.selection = null;
      changed('work');
      changed('selected');
    } }, 'Delete path')),
  ];
  el.replaceChildren(...parts.filter((p): p is HTMLElement => p !== null));
}

/** Adds a point halfway along the last leg (or the selected point's leg). */
function addMiddle(chain: Chain, kind: ChainPoint['kind']): void {
  const sel = state.selection?.kind === 'point' ? Math.min(state.selection.index, chain.points.length - 2) : chain.points.length - 2;
  const a = chain.points[sel];
  const b = chain.points[sel + 1];
  addPointAt({ x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 }, kind);
}
