import { describe, expect, it } from 'vitest';
import { checkStartPose } from '../src/core/constraints.ts';
import { fieldObstacles } from '../src/core/field.ts';
import { bounds } from '../src/core/geom.ts';
import { startProblems } from '../src/core/planner.ts';
import { defaultSettings } from '../src/core/project.ts';
import { startChain } from './fixtures/garden.ts';

const deg = (d: number) => (d * Math.PI) / 180;

describe('start poses (G304)', () => {
  const project = defaultSettings();

  it('accepts a robot touching the audience wall on our side', () => {
    expect(startProblems(project, startChain(56, 8.4, 90))).toEqual([]);
  });

  it('rejects a robot that is not touching a wall', () => {
    expect(checkStartPose(project, 'p', { x: 56, y: 20, h: deg(90) }).map((p) => p.problem).join()).toMatch(/touching a wall/);
  });

  it('rejects a robot past the centerline, in the loading zone, or touching a flower', () => {
    expect(checkStartPose(project, 'p', { x: 68, y: 8.4, h: deg(90) }).some((p) => /centerline/.test(p.problem))).toBe(true);
    expect(checkStartPose(project, 'p', { x: 8.4, y: 105, h: 0 }).some((p) => /loading zone/.test(p.problem))).toBe(true);
    expect(checkStartPose(project, 'p', { x: 8.4, y: 40, h: 0 }).some((p) => /flower/.test(p.problem))).toBe(true);
  });

  it('catches the old start pose, which assumed an 18 in robot and walls at 0', () => {
    expect(checkStartPose(project, 'start', { x: 56, y: 8, h: deg(90) }).length).toBeGreaterThan(0);
  });
});

describe('field', () => {
  it('lets the hive frame legs reach further in as the robot gets taller', () => {
    const leg = (top: number) => bounds(fieldObstacles(top).find((o) => o.name === 'Hive frame leg (audience)')!.polygon).x1;
    expect(leg(16)).toBeCloseTo(53.03, 1); // 14 in robot + 2 in headroom, from the CAD measurement
    expect(leg(20)).toBeGreaterThan(leg(16));
  });

  it('only counts the hives when the robot could reach them', () => {
    expect(fieldObstacles(16).some((o) => o.name === 'Red hive')).toBe(false);
    expect(fieldObstacles(32).some((o) => o.name === 'Red hive')).toBe(true);
  });
});
