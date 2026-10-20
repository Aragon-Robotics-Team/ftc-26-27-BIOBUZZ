/**
 * Copy-paste exchange with robot code. A path exports as a short Java method that calls the PlannedPath helper in
 * TeamCode; the line above it carries the path's settings, so pasting the code back into the planner restores it.
 *
 *     // path-planner {"v":1,"chain":{…},"settings":{…},"legs":[0,1,2]}
 *     static Path gardenCycle(PoseFactory f) {
 *         return PlannedPath.of("gardenCycle#1a2b3c4d", f, true,
 *                 new double[] {x0, y0, x1, y1, x2, y2, x3, y3, f0, h0, f1, h1, …},   // one array per Bezier segment
 *                 …);
 *     }
 *
 * Each array is a cubic Bezier's four control points (red side) followed by heading breakpoints: fraction of the
 * segment's length, heading in degrees (red side), joined by straight-line sweeps.
 */
import type { PlannedSegment } from './chain.ts';
import { fieldObstacles } from './field.ts';
import { drivetrain } from './drivetrain.ts';
import { convexHull, deg } from './geom.ts';
import { evaluate, planOf, shotMarkerName, type Plan } from './optimize.ts';
import { pointLabel } from './planner.ts';
import { fromRow, geometryHash, n, rows, roundSegments, SLOW } from './rows.ts';
import { marginsFor, migrateSettings, ProjectError, validateChain, validateSettings, type Chain, type Settings } from './project.ts';

export { geometryHash, roundSegments } from './rows.ts';

export const EXPORT_VERSION = 1;
const TAG = '// path-planner ';

const constName = (chain: string, marker: string) => `${chain.replace(/([A-Z])/g, '_$1').toUpperCase()}_${marker}`;

/** Method name of each Pedro path a planned path is driven as: name, name2, name3, … */
export const pieceName = (chain: string, piece: number) => (piece === 0 ? chain : `${chain}${piece + 1}`);

export function toJava(chain: Chain, settings: Settings, plan: Plan): string {
  const segments = roundSegments(plan.segments);
  const meta = { v: EXPORT_VERSION, chain, settings, legs: segments.map((s) => s.leg) };
  const r = rows(segments);
  const starts = plan.pieces ?? [0];
  const hash = geometryHash(segments);
  const shots = plan.shots ?? [];
  const lines = [
    `    ${TAG}${JSON.stringify(meta)}`,
    `    // ${chain.name}: ${plan.time.toFixed(2)} s planned${shots.length ? `, ${shots.length} volley${shots.length > 1 ? 's' : ''}` : ''}${plan.feasible ? '' : ' (BREAKS A LIMIT)'}. To change it, paste this into tools/path-planner.html.`,
  ];
  // How to drive it, when it's more than one follow(): the paths in order, with the stops' volleys between.
  if (starts.length > 1 || shots.some((sh) => sh.mode === 'stop')) {
    const steps: string[] = [];
    if (chain.points[0].shoot?.mode === 'stop') steps.push('robot.shoot()');
    starts.forEach((_, p) => {
      steps.push(`follow(follower, ${pieceName(chain.name, p)}(f))`);
      const endLeg = p + 1 < starts.length ? segments[starts[p + 1]].leg : chain.points.length - 1;
      if (chain.points[endLeg].shoot?.mode === 'stop') steps.push('robot.shoot()');
    });
    lines.push(`    // Drive it as: ${steps.join(', ')}`);
  }
  for (const sh of shots.filter((x) => x.mode === 'move')) {
    const m = plan.markers.find((x) => x.name === shotMarkerName(sh.point));
    if (!m) continue;
    const path = `${pieceName(chain.name, m.piece)}(f)`;
    lines.push(`    // Shoot on the move at ${pointLabel(chain, sh.point)}: Path p = ${path}; deadline(follow(follower, p), sequential(passed(p, ${constName(chain.name, m.name)}), robot.shoot()))`);
  }
  for (const m of plan.markers) {
    const where = starts.length > 1 ? `   // on ${pieceName(chain.name, m.piece)}()` : '';
    lines.push(`    public static final PlannedPath.Marker ${constName(chain.name, m.name)} = new PlannedPath.Marker(${m.seg}, ${n(m.t)});${where}`);
  }
  starts.forEach((start, p) => {
    const end = starts[p + 1] ?? segments.length;
    const name = pieceName(chain.name, p);
    const stops = p + 1 < starts.length ? true : chain.endStopped;
    lines.push(`    static Path ${name}(PoseFactory f) {`);
    lines.push(`        return PlannedPath.of("${p === 0 ? chain.name : `${chain.name}.${p + 1}`}#${hash}", f, ${stops},`);
    for (let i = start; i < end; i++) lines.push(`                new double[] {${r[i].join(', ')}}${i < end - 1 ? ',' : ');'}`);
    lines.push('    }');
  });
  return lines.join('\n') + '\n';
}

/** A plan for segments that came from pasted code: everything recomputed except the geometry. */
export function planFromSegments(settings: Settings, chain: Chain, segments: PlannedSegment[]): Plan {
  const ev = evaluate(segments, settings, chain, marginsFor(settings, chain), 0.5);
  return planOf(settings, chain, segments, ev, 0, 0);
}

export interface Imported {
  chain: Chain;
  settings: Settings;
  plan: Plan;
}

/** Every planned path found in pasted text (a single method or a whole file). Throws if none can be read. */
export function fromJava(text: string): Imported[] {
  const found: Imported[] = [];
  const problems: string[] = [];
  const starts = [...text.matchAll(/\/\/ path-planner (\{.*\})\s*$/gm)];
  starts.forEach((m, k) => {
    const end = k + 1 < starts.length ? starts[k + 1].index : text.length;
    const body = text.slice(m.index + m[0].length, end);
    try {
      const meta = JSON.parse(m[1]) as { v: number; chain: Chain; settings: Settings; legs: number[] };
      if (meta.v !== EXPORT_VERSION) throw new Error(`made by a different planner version (${meta.v})`);
      validateChain(meta.chain);
      meta.settings = migrateSettings(meta.settings);
      validateSettings(meta.settings);
      const call = body.indexOf('PlannedPath.of(');
      if (call < 0) throw new Error("the PlannedPath.of(…) call after it is missing");
      const arrays = [...body.slice(call).matchAll(/new double\[\]\s*\{([^}]*)\}/g)].map((a) => a[1].split(',').map((x) => (x.trim() === SLOW ? NaN : Number(x.trim()))));
      if (arrays.length !== meta.legs.length) throw new Error(`expected ${meta.legs.length} segments, found ${arrays.length}`);
      if (arrays.some((a) => a.some((x, i) => !Number.isFinite(x) && !(i === 0 && Number.isNaN(x))))) throw new Error('a number in the code is unreadable');
      const segments = arrays.map((a, i) => fromRow(a, meta.legs[i]));
      found.push({ chain: meta.chain, settings: meta.settings, plan: planFromSegments(meta.settings, meta.chain, segments) });
    } catch (e) {
      const name = /"name":"([^"]+)"/.exec(m[1])?.[1] ?? `path ${k + 1}`;
      problems.push(`${name}: ${e instanceof ProjectError ? e.problems.join('; ') : (e as Error).message}`);
    }
  });
  if (!found.length) throw new ProjectError(problems.length ? problems : ['No planned path found. Paste the code that starts with "// path-planner".']);
  return found;
}

/**
 * A path as a Pedro Visualizer file (format 1.5.0), for viewing it there: one line per curve, its heading a piecewise
 * sweep through the planned breakpoints (the visualizer, like Pedro, places them by arc length), with the robot's size
 * and speeds and, as shapes, the field obstacles at the robot's height and the path's avoid zones.
 *
 * The visualizer times every line as its own start-and-stop move, so its clock and animation speed don't match the
 * plan; the planner's own playback does.
 */
export function toPp(chain: Chain, settings: Settings, plan: Plan): string {
  const segments = roundSegments(plan.segments);
  const colors = ['#2563eb', '#7c3aed', '#16a34a', '#d97706', '#db2777', '#0891b2'];
  const r = (v: number) => Math.round(v * 10000) / 10000;
  const lines = segments.map((seg, i) => {
    const { F, H } = seg.heading;
    const pieces = F.slice(1).map((f, j) => ({
      startProgress: r(F[j]),
      endProgress: r(f),
      interpolationType: 'linear',
      // The sweep's direction: the visualizer otherwise takes the short way round.
      reversed: Math.abs(H[j + 1] - H[j]) > Math.PI,
      parameters: { startDeg: r(deg(H[j])), endDeg: r(deg(H[j + 1])) },
    }));
    return {
      id: `line-${chain.name}-${i + 1}`,
      color: colors[i % colors.length],
      name: `${chain.name} ${i + 1}`,
      locked: false,
      waitBeforeMs: 0,
      waitAfterMs: 0,
      waitBeforeName: '',
      waitAfterName: '',
      kind: 'atomic',
      endPoint: { x: seg.curve[3].x, y: seg.curve[3].y },
      controlPoints: [
        { x: seg.curve[1].x, y: seg.curve[1].y },
        { x: seg.curve[2].x, y: seg.curve[2].y },
      ],
      heading: pieces.length === 1 && !pieces[0].reversed
        ? { type: 'linear', startDeg: r(deg(H[0])), endDeg: r(deg(H[1])) }
        : { type: 'piecewise', piecewiseHeading: { segments: pieces } },
    };
  });
  const d = drivetrain(settings.model, settings.robot);
  const k = 12.5 / settings.model.nominalVoltage;
  const launch = Math.min(d.aForward * k, d.traction);
  const vizSettings = {
    rWidth: settings.robot.width,
    rHeight: settings.robot.length,
    xVelocity: r(d.vForward * k),
    yVelocity: r(d.vStrafe * k),
    aVelocity: r(d.omegaMax * k),
    maxVelocity: r(d.vForward * k),
    maxAcceleration: r(launch),
    maxDeceleration: r(Math.min(d.traction, launch + settings.model.coastForward)),
  };
  const margins = marginsFor(settings, chain);
  const shapes = [
    ...fieldObstacles(settings.robot.height + margins.headroom).map((o) => ({ name: o.name, vertices: o.polygon, color: '#52525b', fillColor: '#a1a1aa' })),
    ...chain.keepOut.map((k) => ({ name: k.name, vertices: convexHull(k.points), color: '#d97706', fillColor: '#fcd34d' })),
  ].map((s, i) => ({ id: `shape-${i + 1}`, name: s.name, vertices: s.vertices.map((v) => ({ x: r(v.x), y: r(v.y) })), color: s.color, fillColor: s.fillColor }));
  const first = segments[0];
  return (
    JSON.stringify(
      {
        startPoint: { x: first.curve[0].x, y: first.curve[0].y, locked: false, headingDeg: r(deg(first.heading.H[0])) },
        lines,
        shapes,
        sequence: lines.map((l) => ({ kind: 'path', lineId: l.id })),
        settings: vizSettings,
        fieldPoints: [],
        version: '1.5.0',
        timestamp: new Date().toISOString(),
      },
      null,
      2,
    ) + '\n'
  );
}
