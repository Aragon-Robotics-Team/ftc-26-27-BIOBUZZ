import { describe, expect, it } from 'vitest';
import { fromJava, geometryHash, toJava, toPp } from '../src/core/export.ts';
import { optimizeChain } from '../src/core/optimize.ts';
import { garden } from './fixtures/garden.ts';

describe('copy-paste export', () => {
  const { settings, chains } = garden();
  const plans = chains.map((c) => optimizeChain(settings, c, { maxEvalsPerSeed: 800, maxSeeds: 1 }));
  const java = chains.map((c, i) => toJava(c, settings, plans[i])).join('\n');

  it('writes a short PlannedPath call with the settings on the line above', () => {
    expect(java).toContain('// path-planner {"v":1,"chain":{"name":"gardenCycle"');
    expect(java).toContain('static Path gardenCycle(PoseFactory f) {');
    expect(java).toContain(`return PlannedPath.of("gardenCycle#${plans[0].geometryHash}", f, true,`);
    expect(java).toMatch(/public static final PlannedPath\.Marker GARDEN_CYCLE_INTAKE_ON = new PlannedPath\.Marker\(\d+, [\d.]+\);/);
    expect(java.split('{').length).toBe(java.split('}').length);
  });

  it('reads back the same paths, settings and geometry, even from a whole file', () => {
    const file = `public class Routine {\n${java}\n    void other() {}\n}\n`;
    const back = fromJava(file);
    expect(back.map((b) => b.chain)).toEqual(chains);
    expect(back[0].settings).toEqual(settings);
    back.forEach((b, i) => {
      expect(b.plan.segments).toEqual(plans[i].segments);
      expect(b.plan.geometryHash).toBe(plans[i].geometryHash);
      expect(geometryHash(b.plan.segments)).toBe(plans[i].geometryHash);
      expect(b.plan.time).toBeCloseTo(plans[i].time, 6);
      expect(b.plan.specHash).toBe(plans[i].specHash);
    });
  });

  it('says what is wrong with code it cannot read', () => {
    expect(() => fromJava('static Path x() {}')).toThrow(/No planned path found/);
    const broken = java.replace(/new double\[\] \{[^}]*\},?\n/, '');
    expect(() => fromJava(broken.split('\n\n')[0])).toThrow(/expected \d+ segments/);
  });

  it('writes a Pedro Visualizer file: one line per curve, turning through the planned breakpoints', () => {
    const pp = JSON.parse(toPp(chains[0], settings, plans[0]));
    const segs = plans[0].segments;
    type Heading = { type: 'linear'; startDeg: number; endDeg: number } | { type: 'piecewise'; piecewiseHeading: { segments: { startProgress: number; endProgress: number; parameters: { startDeg: number; endDeg: number } }[] } };
    type Line = { id: string; endPoint: { x: number; y: number }; controlPoints: unknown[]; heading: Heading };
    expect(pp.version).toBe('1.5.0');
    expect(pp.lines).toHaveLength(segs.length);
    expect(pp.startPoint).toMatchObject({ x: segs[0].curve[0].x, y: segs[0].curve[0].y, headingDeg: 90 });
    pp.lines.forEach((line: Line, i: number) => {
      expect(line.endPoint).toEqual({ x: segs[i].curve[3].x, y: segs[i].curve[3].y });
      expect(line.controlPoints).toHaveLength(2);
      const { F, H } = segs[i].heading;
      const pieces = line.heading.type === 'linear'
        ? [{ startProgress: 0, endProgress: 1, parameters: { startDeg: line.heading.startDeg, endDeg: line.heading.endDeg } }]
        : line.heading.piecewiseHeading.segments;
      expect(pieces).toHaveLength(F.length - 1);
      pieces.forEach((p, j) => {
        expect(p.startProgress).toBeCloseTo(F[j], 3);
        expect(p.endProgress).toBeCloseTo(F[j + 1], 3);
        expect(p.parameters.startDeg).toBeCloseTo((H[j] * 180) / Math.PI, 3);
        expect(p.parameters.endDeg).toBeCloseTo((H[j + 1] * 180) / Math.PI, 3);
      });
    });
    expect(pp.sequence.map((s: { lineId: string }) => s.lineId)).toEqual(pp.lines.map((l: Line) => l.id));
    expect(pp.settings).toMatchObject({ rWidth: settings.robot.width, rHeight: settings.robot.length });
    expect(pp.shapes.map((s: { name: string }) => s.name)).toContain('Flower (red wall)');
  });
});
