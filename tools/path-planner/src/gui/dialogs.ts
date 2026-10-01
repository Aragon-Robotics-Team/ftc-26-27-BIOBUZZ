/** Dialogs: copy code, paste code, robot & speed settings, logs, help. */
import { fromJava, planFromSegments, toJava, type Imported } from '../core/export.ts';
import { compareRun, drift, extractRuns, fitModel, parseLog, type FitResult } from '../core/logs.ts';
import { drivetrain } from '../core/drivetrain.ts';
import { MOTORS, ProjectError, SAMPLE_MODEL, turretRange, type ModelConfig, type Settings, type Side } from '../core/project.ts';
import { DEFAULT_SHOOTER, Flywheel, flywheelMaxTicks, type ShooterConfig } from '../core/shot.ts';
import { fmt, h, num, select, toast } from './dom.ts';
import { applyForesight, parseForesight } from './foresight.ts';
import { changed, selectedChain, state } from './state.ts';

function dialog(title: string, cls: string, ...body: (HTMLElement | null)[]): HTMLDialogElement {
  const close = h('button', { class: 'x', type: 'button', 'aria-label': 'Close', onclick: () => dlg.close() }, '×');
  const dlg = h('dialog', { class: cls }, h('div', { class: 'dlg-head' }, h('h2', {}, title), close), h('div', { class: 'dlg-body' }, ...body));
  dlg.addEventListener('close', () => dlg.remove());
  dlg.addEventListener('click', (e) => e.target === dlg && dlg.close()); // click outside
  document.body.append(dlg);
  dlg.showModal();
  return dlg;
}

async function toClipboard(text: string, fallback: HTMLTextAreaElement): Promise<boolean> {
  try {
    await navigator.clipboard.writeText(text);
    return true;
  } catch {
    fallback.select();
    return document.execCommand('copy');
  }
}

export function copyDialog(): void {
  const chain = selectedChain();
  const plan = chain && state.plans.get(chain.name);
  if (!chain || !plan) return;
  const code = toJava(chain, state.settings, plan);
  const area = h('textarea', { class: 'code', readonly: true, rows: Math.min(18, code.split('\n').length + 1), spellcheck: 'false' }, code);
  const copied = () => {
    state.copied.set(chain.name, plan.geometryHash);
    changed('work');
  };
  const markers = plan.markers.length ? ` Wait for a marker with passed(path, ${chain.name.replace(/([A-Z])/g, '_$1').toUpperCase()}_${plan.markers[0].name}).` : '';
  const dlg = dialog(`Copy ${chain.name}`, 'wide',
    h('p', { class: 'dim' }, `Paste it inside your routine's class, then follow ${chain.name}(poseFactory).${markers}`),
    area,
    h('div', { class: 'actions end' },
      h('button', { class: 'btn primary', type: 'button', onclick: async () => {
        if (await toClipboard(code, area)) {
          copied();
          toast('Copied.');
          dlg.close();
        } else toast('Couldn\'t copy: select the text and copy it yourself.');
      } }, 'Copy to clipboard')));
  area.addEventListener('copy', copied);
}

/** Adds imported paths, replacing any with the same name. */
function addImported(found: Imported[], useTheirSettings: boolean): void {
  if (useTheirSettings) state.settings = structuredClone(found[0].settings);
  for (const f of found) {
    const plan = useTheirSettings ? f.plan : planFromSegments(state.settings, f.chain, f.plan.segments);
    const at = state.chains.findIndex((c) => c.name === f.chain.name);
    if (at >= 0) state.chains[at] = f.chain;
    else state.chains.push(f.chain);
    state.plans.set(f.chain.name, plan);
    state.copied.set(f.chain.name, plan.geometryHash);
  }
  state.selected = found[found.length - 1].chain.name;
  state.selection = null;
  changed('work');
  changed('selected');
  toast(`Added ${found.map((f) => f.chain.name).join(', ')}.`);
}

export function pasteDialog(): void {
  const area = h('textarea', { class: 'code', rows: 12, spellcheck: 'false', placeholder: '// path-planner {…}\nstatic Path …' });
  const msg = h('div', {});
  const dlg = dialog('Paste code', 'wide',
    h('p', { class: 'dim' }, 'Paste a path copied from here earlier (from its "// path-planner" line), or a whole routine file.'),
    area,
    msg,
    h('div', { class: 'actions end' },
      h('button', { class: 'btn primary', type: 'button', onclick: () => {
        let found: Imported[];
        try {
          found = fromJava(area.value);
        } catch (e) {
          msg.replaceChildren(h('p', { class: 'issue' }, ...(e instanceof ProjectError ? e.problems : [(e as Error).message]).flatMap((p) => [p, h('br')])));
          return;
        }
        const theirs = JSON.stringify(found[0].settings);
        const differs = JSON.stringify(state.settings) !== theirs;
        const others = state.chains.some((c) => !found.some((f) => f.chain.name === c.name));
        if (!differs || !others) {
          addImported(found, true);
          dlg.close();
          return;
        }
        msg.replaceChildren(h('p', { class: 'issue' }, 'This code was planned with a different robot or speed model than your other paths.'),
          h('div', { class: 'actions end' },
            h('button', { class: 'btn', type: 'button', onclick: () => (addImported(found, false), dlg.close()) }, 'Keep my settings'),
            h('button', { class: 'btn primary', type: 'button', onclick: () => (addImported(found, true), dlg.close()) }, 'Use its settings')));
      } }, 'Add')));
  area.focus();
}

/** Drivetrain inputs, in the order shown. [field, label, unit, how many decimals to show] */
const DRIVE_FIELDS: [keyof ModelConfig, string, string, number][] = [
  ['motorRpm', 'Motor free speed', 'RPM', 0],
  ['motorStallTorque', 'Motor stall torque', 'N·m', 3],
  ['reduction', 'Extra gear reduction (1 = none)', '×', 2],
  ['wheelDiameter', 'Wheel diameter', 'in', 3],
  ['wheelbase', 'Front to back axles', 'in', 2],
  ['trackWidth', 'Left to right wheels', 'in', 2],
  ['mass', 'Robot mass', 'kg', 2],
  ['grip', 'Grip (μ)', '', 2],
  ['driveEfficiency', 'Speed reached, of free speed', '×', 3],
  ['strafeEfficiency', 'Sideways speed, of forward', '×', 3],
  ['coastForward', 'Coasting, forward', 'in/s²', 1],
  ['coastStrafe', 'Coasting, sideways', 'in/s²', 1],
  ['stopOverhead', 'Settling at the end', 's', 2],
  ['nominalVoltage', 'Motor specs are at', 'V', 1],
];
/** The ones logs can fit. */
const FIT_FIELDS = DRIVE_FIELDS.filter(([k]) => ['driveEfficiency', 'strafeEfficiency', 'grip', 'mass', 'coastForward', 'coastStrafe', 'stopOverhead'].includes(k));

/** What the drivetrain can do, in a line, at a typical 12.5 V battery. */
function capabilities(model: ModelConfig, robot: Settings['robot']): string {
  const d = drivetrain(model, robot);
  const k = 12.5 / model.nominalVoltage;
  const launch = Math.min(d.aForward * k, d.traction);
  return `${Math.round(d.vForward * k)} in/s forward, ${Math.round(d.vStrafe * k)} sideways · turns ${(d.omegaMax * k).toFixed(1)} rad/s · starts at ${Math.round(launch)} in/s²${launch === d.traction ? ' (grip-limited)' : ''}`;
}

export function settingsDialog(): void {
  const s = state.settings;
  const edit = () => changed('work');
  const row = (label: string, unit: string, input: HTMLElement) => h('label', { class: 'set-row' }, h('span', {}, label), input, h('span', { class: 'unit' }, unit));
  const sides: [Side, string][] = [['front', 'Front'], ['back', 'Back'], ['left', 'Left'], ['right', 'Right']];
  const protrusions = h('div', {});
  const renderProtrusions = () => protrusions.replaceChildren(...s.robot.protrusions.map((p, i) => h('div', { class: 'inline' },
    select(p.side, sides, (v) => ((p.side = v), edit()), 'Side'),
    h('span', { class: 'dim' }, 'from'), num(p.from, (v) => ((p.from = v), edit()), { label: 'From (along the side, from its middle)', cls: 'tiny' }),
    h('span', { class: 'dim' }, 'to'), num(p.to, (v) => ((p.to = v), edit()), { label: 'To', cls: 'tiny' }),
    h('span', { class: 'dim' }, 'sticks out'), num(p.depth, (v) => ((p.depth = v), edit()), { min: 0, label: 'How far it sticks out', cls: 'tiny' }),
    h('button', { class: 'x', type: 'button', 'aria-label': 'Remove', onclick: () => (s.robot.protrusions.splice(i, 1), renderProtrusions(), edit()) }, '×'))));
  renderProtrusions();
  const paste = h('textarea', { class: 'code', rows: 4, placeholder: 'Paste the Foresight Tuner output', spellcheck: 'false' });
  const model = h('div', {});
  const motorPick = () => {
    const preset = MOTORS.find((m) => m.rpm === s.model.motorRpm && Math.abs(m.stallTorque - s.model.motorStallTorque) < 0.01);
    return select(preset?.name ?? 'custom', [...MOTORS.map((m) => [m.name, m.name] as [string, string]), ['custom', 'Other motor']], (name) => {
      const m = MOTORS.find((x) => x.name === name);
      if (!m) return;
      s.model.motorRpm = m.rpm;
      s.model.motorStallTorque = m.stallTorque;
      if (!/, edited$/.test(s.model.source)) s.model.source += ', edited';
      renderModel();
      refreshSummary();
      edit();
    }, 'Drive motor');
  };
  const source = h('p', {});
  const caps = h('p', { class: 'capabilities' });
  // Editing a field updates the summary in place, so tabbing to the next field keeps working.
  const refreshSummary = () => {
    source.className = `dim${/sample|placeholder/i.test(s.model.source) ? ' warn' : ''}`;
    source.textContent = `From: ${s.model.source}`;
    caps.textContent = capabilities(s.model, s.robot);
  };
  const renderModel = () => model.replaceChildren(
    source,
    caps,
    h('label', { class: 'set-row motor' }, h('span', {}, 'Drive motor'), motorPick()),
    ...DRIVE_FIELDS.map(([key, label, unit]) => row(label, unit, num(s.model[key] as number, (v) => {
      (s.model[key] as number) = v;
      if (!/, edited$/.test(s.model.source)) s.model.source += ', edited';
      refreshSummary();
      edit();
    }, { min: 0, label }))));
  renderModel();
  refreshSummary();
  const shooter = h('div', {});
  const renderShooter = () => {
    const summary = h('p', { class: 'capabilities' }, flywheelSummary(s.shooter));
    const input = ([key, label, unit]: [keyof ShooterConfig, string, string, number]) => row(label, unit, num(s.shooter[key], (v) => {
      s.shooter[key] = v;
      summary.textContent = flywheelSummary(s.shooter);
      edit();
    }, { min: 0, label }));
    shooter.replaceChildren(
      summary,
      ...SHOT_FIELDS.map(input),
      h('details', { class: 'fold' }, h('summary', {}, 'Ball flight (as in RobotConstants.Launcher)'), ...FLIGHT_FIELDS.map(input)),
      h('div', { class: 'actions' }, h('button', { class: 'link', type: 'button', onclick: () => {
        s.shooter = { ...DEFAULT_SHOOTER };
        renderShooter();
        edit();
      } }, 'Reset')));
  };
  renderShooter();
  const locked = !!state.running;
  dialog('Robot & speed', 'sheet',
    h('fieldset', { class: 'lock', disabled: locked, title: locked ? 'Editing is paused while a path optimizes' : '' },
    h('h3', {}, 'Robot'),
    row('Width (side to side)', 'in', num(s.robot.width, (v) => ((s.robot.width = v), edit()), { min: 1, label: 'Width' })),
    row('Length (front to back)', 'in', num(s.robot.length, (v) => ((s.robot.length = v), edit()), { min: 1, label: 'Length' })),
    row('Height, with turret', 'in', num(s.robot.height, (v) => ((s.robot.height = v), edit()), { min: 1, label: 'Height' })),
    h('div', { class: 'set-row turret' },
      h('span', { title: '0° is the robot\'s front; positive is to the left (counter-clockwise)' }, 'Turret turns from'),
      num(turretRange(s.robot).min, (v) => ((s.robot.turretMin = v), edit()), { label: 'Turret range start, degrees', cls: 'tiny' }),
      h('span', { class: 'dim' }, '° to'),
      num(turretRange(s.robot).max, (v) => ((s.robot.turretMax = v), edit()), { label: 'Turret range end, degrees', cls: 'tiny' }),
      h('span', { class: 'unit' }, '°')),
    protrusions,
    h('button', { class: 'link', type: 'button', onclick: () => (s.robot.protrusions.push({ side: 'front', from: -4, to: 4, depth: 2 }), renderProtrusions(), edit()) }, '+ Something sticking out'),
    h('h3', {}, 'Gaps to keep (all paths)'),
    row('From obstacles and walls', 'in', num(s.margins.obstacle, (v) => ((s.margins.obstacle = v), edit()), { min: 0, label: 'Obstacles' })),
    row('From the centerline', 'in', num(s.margins.centerline, (v) => ((s.margins.centerline = v), edit()), { min: 0, label: 'Centerline' })),
    row('Above the robot (hive frame)', 'in', num(s.margins.headroom, (v) => ((s.margins.headroom = v), edit()), { min: 0, label: 'Headroom' })),
    h('h3', {}, 'Drivetrain'),
    model,
    paste,
    h('div', { class: 'actions' },
      h('button', { class: 'btn', type: 'button', onclick: () => {
        try {
          s.model = { ...applyForesight(s.model, parseForesight(paste.value)), source: `Foresight Tuner, ${new Date().toISOString().slice(0, 10)}` };
          paste.value = '';
          renderModel();
          refreshSummary();
          edit();
        } catch (e) {
          toast((e as Error).message, 5000);
        }
      } }, 'Use tuner numbers'),
      h('button', { class: 'link', type: 'button', onclick: () => {
        s.model = { ...SAMPLE_MODEL };
        renderModel();
        refreshSummary();
        edit();
      } }, 'Reset')),
    h('h3', {}, 'Shooter'),
    shooter),
  );
}

/** Shooter inputs: [field, label, unit, decimals]. The first ones decide where shots work; the rest mirror
 *  RobotConstants.Launcher's flight model. */
const SHOT_FIELDS: [keyof ShooterConfig, string, string, number][] = [
  ['balls', 'Balls per volley', '', 0],
  ['volley', 'Volley length (gate open)', 's', 2],
  ['aimAccuracy', 'Aim accuracy', '± °', 1],
  ['speedAccuracy', 'Flywheel speed accuracy', '± %', 1],
  ['inertia', 'Flywheel inertia', 'kg·cm²', 2],
];
const FLIGHT_FIELDS: [keyof ShooterConfig, string, string, number][] = [
  ['motorRpm', 'Flywheel motor free speed', 'RPM', 0],
  ['motorStallTorque', 'Flywheel motor stall torque', 'N·m', 3],
  ['ticksPerExitSpeed', 'Ticks/s per in/s of ball speed', '', 2],
  ['hoodAngle', 'Hood angle', '°', 2],
  ['exitHeight', 'Ball leaves at height', 'in', 1],
  ['drag', 'Drag coefficient', '', 2],
  ['lift', 'Backspin lift coefficient', '', 2],
  ['scoringMargin', 'Ball clears the opening by', 'in', 2],
];

/** How the flywheel behaves, in a line: spinning up, and what a ball costs it. */
function flywheelSummary(cfg: ShooterConfig): string {
  const fw = new Flywheel(cfg);
  const typical = 0.75 * flywheelMaxTicks(cfg);
  const exit = typical / cfg.ticksPerExitSpeed;
  const after = fw.afterBall(typical, exit);
  return `Spins up from rest in ${fw.timeTo(0, typical, typical * 0.01).toFixed(2)} s · loses ${(((typical - after) / typical) * 100).toFixed(1)}% per ball, back in ${fw.timeTo(after, typical, typical * 0.005).toFixed(2)} s`;
}

// ---- Logs ----

let fit: FitResult | null = null;

export function refreshRuns(): void {
  const plans = [...state.plans.values()];
  state.runs = state.logs.flatMap((log) => extractRuns(log, plans));
  state.comparisons = state.runs.map((r) => compareRun(r, state.settings));
}

export function logsDialog(): void {
  const body = h('div', {});
  const render = () => {
    const input = h('input', { type: 'file', accept: '.csv,text/csv', multiple: true, hidden: true, onchange: (e: Event) => void load((e.target as HTMLInputElement).files ?? []) });
    const drop = h('div', { class: 'drop' }, 'Drop log files here or ', h('button', { class: 'link', type: 'button', onclick: () => input.click() }, 'choose them'), input);
    drop.addEventListener('dragover', (e) => (e.preventDefault(), drop.classList.add('over')));
    drop.addEventListener('dragleave', () => drop.classList.remove('over'));
    drop.addEventListener('drop', (e) => {
      e.preventDefault();
      if (e.dataTransfer?.files.length) void load(e.dataTransfer.files);
    });
    const parts: (HTMLElement | null)[] = [
      h('p', { class: 'dim' }, 'Get them on the robot\'s Wi-Fi at ', h('a', { href: 'http://192.168.43.1:8080/pathlogs', target: '_blank', rel: 'noopener' }, '192.168.43.1:8080/pathlogs'), '.'),
      drop,
    ];
    if (state.comparisons.length) {
      const d = drift(state.comparisons);
      parts.push(
        h('p', { class: d.flagged ? 'issue' : 'dim' }, d.message),
        h('table', { class: 'data' },
          h('thead', {}, h('tr', {}, h('th', {}, 'Path'), h('th', { class: 'num' }, 'Robot'), h('th', { class: 'num' }, 'Planned'), h('th', { class: 'num' }, 'Off'))),
          h('tbody', {}, ...state.comparisons.map((c) => h('tr', {},
            h('td', {}, c.run.chain, c.run.matched ? '' : h('span', { class: 'dim', title: 'Path changed since this run; rebuilt from the logged positions' }, ' *')),
            h('td', { class: 'num' }, `${c.run.duration.toFixed(2)} s`),
            h('td', { class: 'num' }, `${c.predicted.toFixed(2)} s`),
            h('td', { class: `num${Math.abs(c.error) > 0.05 ? ' warn' : ''}` }, `${c.error > 0 ? '+' : ''}${(c.error * 100).toFixed(0)}%`))))),
        h('div', { class: 'actions' },
          h('button', { class: 'btn', type: 'button', onclick: () => {
            try {
              fit = fitModel(state.runs, state.settings);
            } catch (e) {
              toast((e as Error).message);
            }
            render();
          } }, 'Fit speed model to these runs'),
          h('button', { class: 'link', type: 'button', onclick: () => ((state.logs = []), (fit = null), refreshRuns(), render()) }, 'Clear')),
      );
    } else if (state.logs.length) parts.push(h('p', { class: 'dim' }, 'These logs have no runs of planned paths.'));
    if (fit) {
      const f = fit;
      parts.push(
        h('table', { class: 'data' },
          h('thead', {}, h('tr', {}, h('th', {}, ''), h('th', { class: 'num' }, 'Now'), h('th', { class: 'num' }, 'Fitted'))),
          h('tbody', {}, ...FIT_FIELDS.map(([k, label, , digits]) => h('tr', {}, h('td', {}, label), h('td', { class: 'num dim' }, fmt(state.settings.model[k] as number, digits)), h('td', { class: 'num' }, fmt(f.model[k] as number, digits)))))),
        h('div', { class: 'actions' }, h('button', { class: 'btn primary', type: 'button', disabled: !!state.running, title: state.running ? 'Wait for the optimizer to finish' : '', onclick: () => {
          state.settings.model = { ...f.model, source: `${f.model.source}, ${new Date().toISOString().slice(0, 10)}`, nominalVoltage: state.settings.model.nominalVoltage };
          fit = null;
          refreshRuns();
          changed('work');
          render();
          toast('Speed model updated. Optimize your paths again.');
        } }, 'Use fitted model')),
      );
    }
    body.replaceChildren(...parts.filter((p): p is HTMLElement => !!p));
  };
  const load = async (files: FileList | File[]) => {
    for (const f of Array.from(files)) {
      try {
        const log = parseLog(f.name, await f.text());
        state.logs = state.logs.filter((l) => l.name !== log.name).concat(log);
      } catch (e) {
        toast((e as Error).message, 5000);
      }
    }
    fit = null;
    refreshRuns();
    changed('logs');
    render();
  };
  render();
  dialog('Robot logs', 'sheet', body);
}

/** Lists problems and asks whether to go ahead anyway. */
export function confirmDialog(title: string, items: string[], goLabel: string, go: () => void): void {
  const dlg = dialog(title, '',
    h('ul', { class: 'problems' }, ...items.map((t) => h('li', {}, t))),
    h('div', { class: 'actions end' },
      h('button', { class: 'btn', type: 'button', onclick: () => {
        dlg.close();
        go();
      } }, goLabel),
      h('button', { class: 'btn primary', type: 'button', onclick: () => dlg.close() }, 'Fix first')));
}

export function helpDialog(): void {
  dialog('How to use it', '',
    h('ol', {},
      h('li', {}, 'New path, or Paste code to edit one you copied before.'),
      h('li', {}, 'Drag the numbered points on the field. Drag a point\'s arrow to turn it. Double-click to add a point.'),
      h('li', {}, 'Optimize finds the fastest path that keeps clear of everything.'),
      h('li', {}, 'Copy code and paste it into your routine.')),
    h('p', { class: 'dim' }, 'Red side only; the robot flips it for blue. Gray is off limits, with the shaded band the gap the robot keeps. Your work stays in this browser, but the copied code is what counts.'));
}
